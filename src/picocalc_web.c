#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <emscripten.h>

#include "emulator.h"
#include "gpio.h"
#include "timer.h"
#include "nvic.h"
#include "clocks.h"
#include "adc.h"
#include "rom.h"
#include "uart.h"
#include "spi.h"
#include "i2c.h"
#include "pwm.h"
#include "dma.h"
#include "pio.h"
#include "usb.h"
#include "rtc.h"
#include "corepool.h"

// Forward decalre from membus/corepool
extern void cpu_init(void);
extern void reset_runtime_peripherals(const char *tap_name);
extern int load_uf2(const char *path);
extern int load_uf2_buffer(const uint8_t *buf, size_t size); // we might need to add this to UF2

// We need a dummy reset for main.c's reset_runtime_peripherals
void reset_runtime_peripherals_web(void) {
    gpio_init();
    timer_init();
    nvic_init();
    rom_init();
    uart_init();
    spi_init();
    i2c_init();
    pwm_init();
    dma_init();
    pio_init();
    clocks_init();
    adc_init();
    usb_init();
    rtc_init();
}

static uint8_t display_buffer[320 * 320 * 4]; // RGBA
static int lcd_dc = 1;
static int lcd_cs = 1;

static uint32_t lcd_x = 0, lcd_y = 0;
static uint32_t lcd_xs = 0, lcd_xe = 319;
static uint32_t lcd_ys = 0, lcd_ye = 319;
static int lcd_state = 0; // 0=cmd, 1=caset, 2=paset, 3=ramwr
static int lcd_data_cnt = 0;
static uint8_t lcd_rgb[3];

// Diagnostic counters polled by JS
static uint32_t spi_xfer_count = 0;
static uint32_t uart_xfer_count = 0;
static uint32_t pixel_write_count = 0;
static uint64_t total_steps_count = 0;
static uint32_t last_sd_cmd = 0;
static uint32_t last_sd_sector = 0;

// Virtual SD Disk (4MB for now)
#define SD_DISK_SIZE (4 * 1024 * 1024)
static uint8_t *virtual_sd_disk = NULL;

static uint32_t pc_trace[NUM_CORES][16];
static int pc_trace_idx[NUM_CORES] = {0};

// UART Log Buffer
#define UART_LOG_SIZE 65536
static char uart_log_buf[UART_LOG_SIZE];
static uint32_t uart_log_head = 0;
static uint32_t uart_log_tail = 0;

extern gpio_state_t gpio_state;
extern int log_uart_enabled;

// Custom UART logger for web
void bus_log_uart(int num, int is_tx, uint8_t byte) {
    if (!is_tx) return;
    uart_xfer_count++;
    uart_log_buf[uart_log_head] = (char)byte;
    uart_log_head = (uart_log_head + 1) % UART_LOG_SIZE;
    // If head hits tail, advance tail (discard oldest)
    if (uart_log_head == uart_log_tail) {
        uart_log_tail = (uart_log_tail + 1) % UART_LOG_SIZE;
    }
}

uint8_t picocalc_spi_xfer(void *ctx, uint8_t mosi) {
    (void)ctx;
    spi_xfer_count++;
    // DC pin is GPIO 14 on PicoCalc
    int dc = (gpio_state.gpio_out & (1 << 14)) ? 1 : 0;
    
    if (dc == 0) {
        // Command
        if (mosi == 0x2A) { // CASET
            lcd_state = 1;
            lcd_data_cnt = 0;
        } else if (mosi == 0x2B) { // PASET
            lcd_state = 2;
            lcd_data_cnt = 0;
        } else if (mosi == 0x2C) { // RAMWR
            lcd_state = 3;
            lcd_data_cnt = 0;
            lcd_x = lcd_xs;
            lcd_y = lcd_ys;
        } else {
            lcd_state = 0;
        }
    } else {
        // Data
        if (lcd_state == 1) { // CASET
            if (lcd_data_cnt == 0) lcd_xs = ((uint32_t)mosi << 8);
            else if (lcd_data_cnt == 1) lcd_xs |= mosi;
            else if (lcd_data_cnt == 2) lcd_xe = ((uint32_t)mosi << 8);
            else if (lcd_data_cnt == 3) lcd_xe |= mosi;
            lcd_data_cnt++;
        } else if (lcd_state == 2) { // PASET
            if (lcd_data_cnt == 0) lcd_ys = ((uint32_t)mosi << 8);
            else if (lcd_data_cnt == 1) lcd_ys |= mosi;
            else if (lcd_data_cnt == 2) lcd_ye = ((uint32_t)mosi << 8);
            else if (lcd_data_cnt == 3) lcd_ye |= mosi;
            lcd_data_cnt++;
        } else if (lcd_state == 3) { // RAMWR
            lcd_rgb[lcd_data_cnt++] = mosi;
            if (lcd_data_cnt == 3) {
                lcd_data_cnt = 0;
                if (lcd_x < 320 && lcd_y < 320) {
                    uint32_t offset = (lcd_y * 320 + lcd_x) * 4;
                    display_buffer[offset + 0] = lcd_rgb[0];
                    display_buffer[offset + 1] = lcd_rgb[1];
                    display_buffer[offset + 2] = lcd_rgb[2];
                    display_buffer[offset + 3] = 255;
                    pixel_write_count++;
                }
                lcd_x++;
                if (lcd_x > lcd_xe) {
                    lcd_x = lcd_xs;
                    lcd_y++;
                    if (lcd_y > lcd_ye) {
                        lcd_y = lcd_ys;
                    }
                }
            }
        }
    }
    return 0; // MISO is not wired for display return
}

void picocalc_spi_cs(void *ctx, int asserted) {
    (void)ctx;
    lcd_cs = !asserted; // asserted is 1 when CS goes LOW (active low)
}

/* SD Card SPI State Machine */
typedef enum {
    SD_IDLE,
    SD_CMD,
    SD_RESP,
    SD_DATA_PRE,
    SD_DATA,
    SD_DATA_POST,
    SD_DATA_POST2
} sd_state_t;

static struct {
    sd_state_t state;
    uint8_t cmd[6];
    int cmd_idx;
    uint8_t resp[16];
    int resp_idx;
    int resp_len;
    uint32_t arg;
    uint32_t block_addr;
    int data_idx;
    int data_delay;
    int ncr_delay; // Added for protocol timing
    uint8_t last_miso;
} sd = {0};

uint8_t picocalc_sd_spi_xfer(void *ctx, uint8_t mosi) {
    (void)ctx;
    uint8_t miso = 0xFF;

    switch (sd.state) {
    case SD_IDLE:
        if (mosi >= 0x40 && mosi <= 0x7F) {
            sd.cmd[0] = mosi & 0x3F;
            sd.cmd_idx = 1;
            sd.state = SD_CMD;
        }
        break;

    case SD_CMD:
        sd.cmd[sd.cmd_idx++] = mosi;
        if (sd.cmd_idx == 6) {
            sd.arg = ((uint32_t)sd.cmd[1] << 24) | ((uint32_t)sd.cmd[2] << 16) | ((uint32_t)sd.cmd[3] << 8) | sd.cmd[4];
            uint8_t command = sd.cmd[0];
            last_sd_cmd = command;
            if (command == 17 || command == 18 || command == 24 || command == 25) {
                last_sd_sector = sd.arg;
            }
            printf("[SD] CMD%d Arg:0x%08X\n", command, sd.arg);
            
            sd.state = SD_RESP;
            sd.resp_idx = 0;
            sd.ncr_delay = 2; // Real SD cards have 1-8 byte delay before response
            
            if (command == 0) { // CMD0: GO_IDLE
                sd.resp[0] = 0x01;
                sd.resp_len = 1;
            } else if (command == 8) { // CMD8: SEND_IF_COND
                sd.resp[0] = 0x01;
                sd.resp[1] = 0x00;
                sd.resp[2] = 0x00;
                sd.resp[3] = 0x01;
                sd.resp[4] = 0xAA;
                sd.resp_len = 5;
            } else if (command == 55) { // CMD55: APP_CMD
                sd.resp[0] = 0x01;
                sd.resp_len = 1;
            } else if (command == 41) { // ACMD41: SD_SEND_OP_COND
                sd.resp[0] = 0x00; // Ready
                sd.resp_len = 1;
            } else if (command == 16) { // CMD16: SET_BLOCKLEN
                sd.resp[0] = 0;
                sd.resp_len = 1;
            } else if (command == 58) { // CMD58: READ_OCR
                sd.resp[0] = 0; sd.resp[1] = 0xC0; sd.resp[2] = 0; sd.resp[3] = 0; sd.resp[4] = 0;
                sd.resp_len = 5;
            } else if (command == 17) { // CMD17: READ_SINGLE_BLOCK
                sd.resp[0] = 0x00;
                sd.resp_len = 1;
                sd.block_addr = sd.arg;
                sd.data_delay = 10;
            } else {
                sd.resp[0] = 0x00;
                sd.resp_len = 1;
            }
            printf("[SD] CMD%d response set to 0x%02X\n", command, sd.resp[0]);
        }
        break;

    case SD_RESP:
        if (sd.ncr_delay > 0) {
            miso = 0xFF;
            sd.ncr_delay--;
        } else {
            miso = sd.resp[sd.resp_idx++];
            if (sd.resp_idx == sd.resp_len) {
                if (sd.cmd[0] == 17) sd.state = SD_DATA_PRE;
                else sd.state = SD_IDLE;
            }
        }
        break;

    case SD_DATA_PRE:
        if (sd.data_delay > 0) {
            miso = 0xFF;
            sd.data_delay--;
        } else {
            miso = 0xFE; // Data token
            sd.state = SD_DATA;
            sd.data_idx = 0;
            printf("[SD] Sending block %u\n", sd.block_addr);
        }
        break;

    case SD_DATA:
    {
        uint32_t addr = sd.block_addr * 512 + sd.data_idx;
        if (virtual_sd_disk && addr < SD_DISK_SIZE) {
            miso = virtual_sd_disk[addr];
        } else {
            miso = 0x00;
        }
        sd.data_idx++;
        if (sd.data_idx == 512) sd.state = SD_DATA_POST;
        break;
    }

    case SD_DATA_POST:
        miso = 0xFF; // CRC byte 1
        sd.state = SD_DATA_POST2;
        break;

    case SD_DATA_POST2:
        miso = 0xFF; // CRC byte 2
        sd.state = SD_IDLE;
        break;
    }

    sd.last_miso = miso;
    return miso;
}

void picocalc_sd_cs(void *ctx, int asserted) {
    (void)ctx;
    if (!asserted) sd.state = SD_IDLE; // CS went HIGH
}

static uint8_t kb_buffer[256];
static int kb_head = 0;
static int kb_tail = 0;

static uint8_t last_i2c_cmd = 0;
static int i2c_read_idx = 0;

void picocalc_web_set_key(uint8_t key) {
    kb_buffer[kb_head] = key;
    kb_head = (kb_head + 1) % 256;
}

uint8_t picocalc_i2c_read(void *ctx) {
    (void)ctx;
    uint8_t val = 0;

    if (last_i2c_cmd == 0x09) { // Keyboard read
        if (i2c_read_idx == 0) { // Status byte
            if (kb_head != kb_tail) val = 1; // Pressed
            else val = 0;
        } else { // Keycode byte
            if (kb_head != kb_tail) {
                val = kb_buffer[kb_tail];
                kb_tail = (kb_tail + 1) % 256;
            } else {
                val = 0;
            }
        }
    } else if (last_i2c_cmd == 0x0B) { // Battery
        val = (i2c_read_idx == 0) ? 0x80 : 0x0C; // Fake 4V battery
    }
    
    i2c_read_idx++;
    return val;
}

int picocalc_i2c_write(void *ctx, uint8_t data) {
    (void)ctx;
    last_i2c_cmd = data;
    i2c_read_idx = 0;
    return 1; // ACK
}


EMSCRIPTEN_KEEPALIVE
void picocalc_web_init(void) {
    printf("[Web] Initializing emulator...\n");
    cpu_init();
    memset(cpu.flash, 0xFF, FLASH_SIZE_MAX);
    reset_runtime_peripherals_web();
    
    log_uart_enabled = 1; // Enable UART tracing to circular buffer
    cpu.debug_enabled = 1; // Enable internal CPU diagnostic prints
    bus_log_uart(0, 1, 'H'); bus_log_uart(0, 1, 'e'); bus_log_uart(0, 1, 'l'); bus_log_uart(0, 1, 'l'); bus_log_uart(0, 1, 'o'); bus_log_uart(0, 1, '\n');

    // Initialize Virtual SD
    if (!virtual_sd_disk) {
        virtual_sd_disk = malloc(SD_DISK_SIZE);
        memset(virtual_sd_disk, 0, SD_DISK_SIZE);
    }

    // Spoof SD detection (GP22 is active low)
    gpio_state.gpio_in &= ~(1u << 22);

    // Attach to SPI1 for ILI9488 display
    spi_attach_device(1, picocalc_spi_xfer, picocalc_spi_cs, NULL);
    
    // Attach to SPI0 for SD Card
    spi_attach_device(0, picocalc_sd_spi_xfer, picocalc_sd_cs, NULL);
    
    // Attach to I2C1 for keyboard
    i2c_attach_device(1, 0x1F, picocalc_i2c_write, picocalc_i2c_read, NULL, NULL, NULL);
}

EMSCRIPTEN_KEEPALIVE
int picocalc_web_load_uf2(const uint8_t *buf, int size) {
    printf("[Web] Loading UF2 buffer of size %d...\n", size);
    
    // Bramble has uf2 loader that reads from file.
    // Let's dump the buffer to a virtual file then load it.
    FILE *f = fopen("/firmware.uf2", "wb");
    if (f) {
        fwrite(buf, 1, size, f);
        fclose(f);
    }
    
    int loaded = load_uf2("/firmware.uf2");
    if (!loaded) return 0;
    
    extern int cpu_has_boot2(void);
    extern void cpu_set_boot2(int);
    if (cpu_has_boot2()) {
        cpu_set_boot2(1);
        printf("[Web] Boot2 detected in firmware\n");
    }
    
    dual_core_init(); // Initializes state now that firmware is in memory
    cpu_reset_core(CORE0);
    return 1;
}

EMSCRIPTEN_KEEPALIVE
int picocalc_web_load_sd_image(const uint8_t *buf, int size) {
    if (!virtual_sd_disk) return 0;
    int to_copy = (size < SD_DISK_SIZE) ? size : SD_DISK_SIZE;
    memcpy(virtual_sd_disk, buf, to_copy);
    printf("[Web] Loaded %d bytes into virtual SD\n", to_copy);
    return to_copy;
}

EMSCRIPTEN_KEEPALIVE
uint8_t* picocalc_web_get_display_buffer(void) {
    return display_buffer;
}

EMSCRIPTEN_KEEPALIVE
uint32_t picocalc_web_get_spi_count(void) {
    return spi_xfer_count;
}

EMSCRIPTEN_KEEPALIVE
uint32_t picocalc_web_get_pixel_count(void) {
    return pixel_write_count;
}

EMSCRIPTEN_KEEPALIVE
uint32_t picocalc_web_get_pc0(void) {
    return cores[CORE0].r[15];
}

EMSCRIPTEN_KEEPALIVE
uint32_t picocalc_web_get_pc1(void) {
    return cores[CORE1].r[15];
}

EMSCRIPTEN_KEEPALIVE
uint32_t picocalc_web_get_sp0(void) {
    return cores[CORE0].r[13];
}

EMSCRIPTEN_KEEPALIVE
uint32_t picocalc_web_get_sp1(void) {
    return cores[CORE1].r[13];
}

EMSCRIPTEN_KEEPALIVE uint8_t picocalc_web_get_last_sd_cmd() { return last_sd_cmd; }
EMSCRIPTEN_KEEPALIVE uint32_t picocalc_web_get_last_sd_sector() { return last_sd_sector; }
EMSCRIPTEN_KEEPALIVE uint8_t picocalc_web_get_last_sd_miso() { return sd.last_miso; }

EMSCRIPTEN_KEEPALIVE
uint32_t picocalc_web_get_total_steps(void) {
    return (uint32_t)total_steps_count;
}

EMSCRIPTEN_KEEPALIVE
uint32_t picocalc_web_get_uart_enabled(int num) {
    if (num < 0 || num > 1) return 0;
    uint32_t base = (num == 0) ? 0x40034000 : 0x40038000;
    extern uint32_t mem_read32(uint32_t addr);
    return mem_read32(base + 0x30);
}

EMSCRIPTEN_KEEPALIVE
uint32_t picocalc_web_get_sh_count(void) {
    extern int sh_activity_count;
    return (uint32_t)sh_activity_count;
}

EMSCRIPTEN_KEEPALIVE
uint32_t picocalc_web_get_nvic_pending(int core) {
    if (core < 0 || core >= 2) return 0;
    extern nvic_state_t nvic_states[2];
    return nvic_states[core].pending;
}

EMSCRIPTEN_KEEPALIVE
uint32_t picocalc_web_get_nvic_enabled(int core) {
    if (core < 0 || core >= 2) return 0;
    extern nvic_state_t nvic_states[2];
    return nvic_states[core].enable;
}

EMSCRIPTEN_KEEPALIVE
uint32_t picocalc_web_get_uart_count(void) {
    return uart_xfer_count;
}

EMSCRIPTEN_KEEPALIVE
int picocalc_web_get_uart_log(char *buf, int max_len) {
    int count = 0;
    while (uart_log_tail != uart_log_head && count < max_len - 1) {
        buf[count++] = uart_log_buf[uart_log_tail];
        uart_log_tail = (uart_log_tail + 1) % UART_LOG_SIZE;
    }
    buf[count] = '\0';
    return count;
}

EMSCRIPTEN_KEEPALIVE
uint32_t picocalc_web_get_core_flags(int core) {
    uint32_t flags = 0;
    if (cores[core].is_halted) flags |= 1;
    if (cores[core].is_wfi)    flags |= 2;
    return flags;
}

EMSCRIPTEN_KEEPALIVE
uint32_t picocalc_web_get_irq(int core) {
    if (core < 0 || core >= NUM_CORES) return 0xFFFFFFFF;
    return cores[core].current_irq;
}

EMSCRIPTEN_KEEPALIVE
void picocalc_web_get_fault_frame(int core, uint32_t *out_frame) {
    if (core < 0 || core >= NUM_CORES) return;
    cpu_state_dual_t *c = &cores[core];
    uint32_t sp = c->r[13];
    // Cortex-M0+ stack frame: R0, R1, R2, R3, R12, LR, PC, xPSR
    for (int i = 0; i < 8; i++) {
        out_frame[i] = mem_read32_dual(core, sp + (i * 4));
    }
}

EMSCRIPTEN_KEEPALIVE
uint32_t picocalc_web_get_pc_trace(int core, uint32_t *out_trace) {
    if (core < 0 || core >= NUM_CORES) return 0;
    for (int i = 0; i < 16; i++) {
        out_trace[i] = pc_trace[core][(pc_trace_idx[core] - 1 - i) & 15];
    }
    return 16;
}

EMSCRIPTEN_KEEPALIVE
void picocalc_web_get_regs(int core, uint32_t *out_regs) {
    if (core < 0 || core >= NUM_CORES) return;
    cpu_state_dual_t *c = &cores[core];
    for (int i = 0; i < 16; i++) out_regs[i] = c->r[i];
    out_regs[16] = c->xpsr;
}

void picocalc_web_dump_mem(uint32_t addr, uint32_t count, uint8_t *out_data) {
    for (uint32_t i = 0; i < count; i++) {
        out_data[i] = mem_read8(addr + i);
    }
}

EMSCRIPTEN_KEEPALIVE
int picocalc_web_get_sio_launch_count(void) {
    extern int sio_get_launch_count(void);
    return sio_get_launch_count();
}

EMSCRIPTEN_KEEPALIVE
uint16_t picocalc_web_read_mem16(uint32_t addr) {
    return mem_read16(addr);
}

void picocalc_emscripten_loop(void) {
    // Run ~500k instructions per frame (~60fps → ~30M steps/sec → ~25% RP2040 speed)
    for(int i=0; i<500000; i++) {
        if (!cores[CORE0].is_halted || !cores[CORE1].is_halted) {
            // Update traces
            for (int c = 0; c < NUM_CORES; c++) {
                uint32_t pc = cores[c].r[15];
                if (pc != pc_trace[c][(pc_trace_idx[c] - 1) & 15]) {
                    pc_trace[c][pc_trace_idx[c]] = pc;
                    pc_trace_idx[c] = (pc_trace_idx[c] + 1) & 15;
                }
            }
            dual_core_step();
            total_steps_count++;
            pio_step();
            usb_step();
        } else {
            break;
        }
    }
}

EMSCRIPTEN_KEEPALIVE
void picocalc_web_start(void) {
    // simulateInfiniteLoop=0: returns normally so JS caller doesn't get an
    // 'unwind' string thrown at it (which ccall async can't handle).
    emscripten_set_main_loop(picocalc_emscripten_loop, 0, 0);
}
