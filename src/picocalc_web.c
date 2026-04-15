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
static int lcd_cs = 1;

/* ========================================================================
 * Virtual keyboard controller (STM32 @ I2C1 addr 0x1F)
 * ======================================================================== */

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

extern trace_entry_t pc_trace[NUM_CORES][16];
extern int pc_trace_idx[NUM_CORES];

// UART Log Buffer
#define UART_LOG_SIZE 65536
static char uart_log_buf[UART_LOG_SIZE];
static uint32_t uart_log_head = 0;
static uint32_t uart_log_tail = 0;

extern gpio_state_t gpio_state;
extern int log_uart_enabled;

// Custom UART logger for web
void bus_log_uart(int num, int is_tx, uint8_t byte) {
    (void)num;
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
    (void)lcd_cs;
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

/* I2C transaction log: last 64 register writes */
#define I2C_LOG_SIZE 64
static uint8_t i2c_write_log[I2C_LOG_SIZE];
static uint32_t i2c_write_count = 0;

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
    i2c_write_log[i2c_write_count % I2C_LOG_SIZE] = data;
    i2c_write_count++;
    return 1; // ACK
}


EMSCRIPTEN_KEEPALIVE
// Dummy function to force the linker to include malloc/free
void dummy_linker_fix(void) {
    void *ptr = malloc(1);
    free(ptr);
}

void picocalc_web_init(void) {
    EMU_LOG(1, "[Web] Initializing emulator...\n");
    cpu_init();
    memset(cpu.flash, 0xFF, FLASH_SIZE_MAX);
    reset_runtime_peripherals_web();
    
    log_uart_enabled = 1; // Enable UART tracing to circular buffer

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
    
    // Attach virtual keyboard controller (STM32) to I2C1 at address 0x1F
    i2c_attach_device(1, 0x1F, picocalc_i2c_write, picocalc_i2c_read, NULL, NULL, NULL);
}

EMSCRIPTEN_KEEPALIVE
int picocalc_web_load_uf2(const uint8_t *buf, int size) {
    EMU_LOG(1, "[Web] Loading UF2 buffer of size %d...\n", size);
    
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
        EMU_LOG(1, "[Web] Boot2 detected in firmware\n");
    }

    /* Pre-initialize the firmware options area at flash 0x100D0000.
     *
     * The PicoCalc/BRAMBLE firmware stores a validated options struct at
     * 0x100D0000 in flash (897 bytes, separate from the main UF2 image).
     * On blank flash (all 0xFF) the firmware enters USB recovery mode and
     * hangs in a tight loop — unusable in the emulator.
     *
     * The struct must satisfy these compile-time checks:
     *   [0..3]       = 0xE1473B93    (magic, LE)
     *   [4]          ≤ 4             (interface channel)
     *   [5]          ∈ {2,3,4,8}    (display family / USB class)
     *   [8..11]      = 0x00020000   (LE, size field = 128 KB)
     *   [0x12..0x15] ≠ 0            (non-zero field)
     *   [0x20..0x23] ∈ [0xBB80, 0x668A0] (unsigned, size range check)
     *   [0x77] | [0xE5] ≠ 0         (at least one flag set)
     *   [0xEB] = 1                  (display-configured flag)
     *
     * Only write the stub if the flash area is blank (all 0xFF).
     */
    {
        uint32_t opts_flash_offset = 0x100D0000 - 0x10000000; /* = 0xD0000 */
        int blank = 1;
        for (int k = 0; k < 4; k++) {
            if (cpu.flash[opts_flash_offset + k] != 0xFF) { blank = 0; break; }
        }
        if (blank) {
            EMU_LOG(1, "[Web] Options area at 0x100D0000 is blank — writing default stub\n");
            uint8_t *opts = cpu.flash + opts_flash_offset;
            /* zero the area first (flash was 0xFF; zero only 897 bytes to match copy count) */
            memset(opts, 0x00, 897);
            /* [0..3]: magic 0xE1473B93 in little-endian */
            opts[0] = 0x93; opts[1] = 0x3B; opts[2] = 0x47; opts[3] = 0xE1;
            /* [4]: interface/channel = 1 (SPI1 for PicoCalc ILI9488) */
            opts[4] = 1;
            /* [5]: display family = 2 (ILI9488 color TFT family) */
            opts[5] = 2;
            /* [8..11]: 0x00020000 little-endian (size field = 128 KB) */
            opts[8] = 0x00; opts[9] = 0x00; opts[10] = 0x02; opts[11] = 0x00;
            /* [0x18..0x1B]: field must be non-zero (validation check) */
            opts[0x18] = 0x01;
            /* [0x20..0x23]: 0x00039210 (midpoint of valid range [0xBB80, 0x668A0]) LE */
            opts[0x20] = 0x10; opts[0x21] = 0x92; opts[0x22] = 0x03; opts[0x23] = 0x00;
            /* [0x77]: non-zero flag */
            opts[0x77] = 1;
            /* [0xEB]: display-configured flag = 1 */
            opts[0xEB] = 1;
        } else {
            EMU_LOG(1, "[Web] Options area at 0x100D0000 already programmed — leaving as-is\n");
        }
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
    EMU_LOG(1, "[Web] Loaded %d bytes into virtual SD\n", to_copy);
    return to_copy;
}

EMSCRIPTEN_KEEPALIVE
uint8_t *picocalc_web_get_sd_image_ptr(void) {
    return virtual_sd_disk;
}

EMSCRIPTEN_KEEPALIVE
int picocalc_web_get_sd_image_size(void) {
    return virtual_sd_disk ? SD_DISK_SIZE : 0;
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
void picocalc_web_set_verbose(int level) {
    bramble_verbose = level;
}

EMSCRIPTEN_KEEPALIVE
int picocalc_web_get_verbose(void) {
    return bramble_verbose;
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

EMSCRIPTEN_KEEPALIVE
uint32_t picocalc_web_get_reg(int core, int reg) {
    if (core < 0 || core >= NUM_CORES) return 0;
    if (reg < 0 || reg > 16) return 0;
    return cores[core].r[reg];
}

EMSCRIPTEN_KEEPALIVE uint8_t picocalc_web_get_last_sd_cmd(void) { return last_sd_cmd; }
EMSCRIPTEN_KEEPALIVE uint32_t picocalc_web_get_last_sd_sector(void) { return last_sd_sector; }
EMSCRIPTEN_KEEPALIVE uint8_t picocalc_web_get_last_sd_miso(void) { return sd.last_miso; }

EMSCRIPTEN_KEEPALIVE uint32_t picocalc_web_get_i2c_write_count(void) { return i2c_write_count; }
EMSCRIPTEN_KEEPALIVE uint8_t  picocalc_web_get_i2c_write_log(uint32_t idx) {
    if (idx >= I2C_LOG_SIZE) return 0xFF;
    return i2c_write_log[idx % I2C_LOG_SIZE];
}

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
uint32_t picocalc_web_get_pc_trace(int core, uint32_t *out_pcs, uint16_t *out_instrs) {
    if (core < 0 || core >= NUM_CORES) return 0;
    for (int i = 0; i < 16; i++) {
        int idx = (pc_trace_idx[core] - 1 - i) & 15;
        out_pcs[i] = pc_trace[core][idx].pc;
        out_instrs[i] = pc_trace[core][idx].instr;
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

EMSCRIPTEN_KEEPALIVE
void picocalc_web_get_memory_range(uint32_t addr, uint32_t len, uint8_t *out_data) {
    for (uint32_t i = 0; i < len; i++) {
        out_data[i] = mem_read8(addr + i);
    }
}

void picocalc_emscripten_loop(void) {
    // Time-budgeted loop: run as many steps as possible within 12ms per frame.
    // This saturates the CPU on fast machines while never blocking the browser UI.
    // The time check is amortized every 2000 steps to keep overhead negligible.
    //
    // Peripheral stepping frequencies:
    //   pio_step  — every 8 CPU steps  (~15 MHz equivalent at 125 MHz CPU)
    //   usb_step  — every 32 CPU steps (~3.9 MHz; USB runs at 48 MHz but
    //               enumeration/CDC are slow enough that coarser stepping is fine)
    static uint32_t pio_counter = 0;
    static uint32_t usb_counter = 0;
    const double deadline = emscripten_get_now() + 12.0;
    for (;;) {
        for (int i = 0; i < 2000; i++) {
            if (cores[CORE0].is_halted && cores[CORE1].is_halted) return;
            for (int c = 0; c < NUM_CORES; c++) {
                uint32_t pc = cores[c].r[15];
                if (pc != pc_trace[c][(pc_trace_idx[c] - 1) & 15].pc) {
                    pc_trace[c][pc_trace_idx[c]].pc = pc;
                    pc_trace[c][pc_trace_idx[c]].instr = mem_read16(pc);
                    pc_trace_idx[c] = (pc_trace_idx[c] + 1) & 15;
                }
            }
            dual_core_step();
            total_steps_count++;
            if (++pio_counter >= 8)  { pio_counter = 0; pio_step(); }
            if (++usb_counter >= 32) { usb_counter = 0; usb_step(); }
        }
        if (emscripten_get_now() >= deadline) break;
    }
}

EMSCRIPTEN_KEEPALIVE
void picocalc_web_start(void) {
    // simulateInfiniteLoop=0: returns normally so JS caller doesn't get an
    // 'unwind' string thrown at it (which ccall async can't handle).
    emscripten_set_main_loop(picocalc_emscripten_loop, 0, 0);
}

EMSCRIPTEN_KEEPALIVE
uint32_t picocalc_web_run_steps(uint32_t n) {
    static int fn_entered = 0;   /* entered 0x10001478 function */
    static int error_logged = 0; /* logged the error-branch hit */
    for (uint32_t i = 0; i < n; i++) {
        if (cores[CORE0].is_halted && cores[CORE1].is_halted) break;
        uint32_t pre_pc = cores[CORE0].r[15];
        dual_core_step();
        total_steps_count++;
        pio_step();
        usb_step();
        uint32_t cur_pc = cores[CORE0].r[15];

        /* Hook A: catch call into mystery function at 0x10001478 */
        if (!fn_entered &&
            (pre_pc < 0x10001478 || pre_pc > 0x1000159A) &&
            cur_pc >= 0x10001478 && cur_pc <= 0x1000159A) {
            fn_entered = 1;
            EMU_ELOG(2, "[FN_ENTER] step=%llu pre_pc=0x%08X cur_pc=0x%08X\n",
                    (unsigned long long)total_steps_count, pre_pc, cur_pc);
            EMU_ELOG(2, "[FN_REGS] r0=0x%08X r1=0x%08X r2=0x%08X r3=0x%08X\n",
                    cores[CORE0].r[0], cores[CORE0].r[1],
                    cores[CORE0].r[2], cores[CORE0].r[3]);
            EMU_ELOG(2, "[FN_REGS] r8=0x%08X r9=0x%08X LR=0x%08X SP=0x%08X\n",
                    cores[CORE0].r[8], cores[CORE0].r[9],
                    cores[CORE0].r[14], cores[CORE0].r[13]);
            /* Dump struct at 0x2002E700 (r3 will point here later) */
            if (bramble_verbose >= 2) {
                fprintf(stderr, "[STRUCT@0x2002E700]");
                for (int k = 0; k < 32; k++)
                    fprintf(stderr, " %02X", mem_read8(0x2002E700 + k));
                fprintf(stderr, "\n");
                /* Dump the r0 argument struct */
                uint32_t r0 = cores[CORE0].r[0];
                fprintf(stderr, "[ARG0@0x%08X]", r0);
                for (int k = 0; k < 32; k++)
                    fprintf(stderr, " %02X", mem_read8(r0 + k));
                fprintf(stderr, "\n");
            }
        }

        /* Hook B: key branch-point tracing inside 0x10001478 function */
        if (fn_entered && !error_logged) {
            /* 0x1001B1E4: start of copy loop — log source pointer r4 and dest r0 */
            if (cur_pc == 0x1001B1E4) {
                static int copy_hook_done = 0;
                if (!copy_hook_done) {
                    copy_hook_done = 1;
                    uint32_t src = cores[CORE0].r[4]; /* r4 = source pointer */
                    uint32_t dst = cores[CORE0].r[0]; /* r0 = destination (struct base) */
                    uint32_t cnt = cores[CORE0].r[1]; /* r1 = count */
                    EMU_ELOG(2, "[COPY_LOOP] src=0x%08X dst=0x%08X cnt=%u\n", src, dst, cnt);
                    if (bramble_verbose >= 2) {
                        fprintf(stderr, "[COPY_SRC_BYTES]");
                        for (uint32_t k = 0; k < cnt && k < 48; k++)
                            fprintf(stderr, " %02X", mem_read8(src + k));
                        fprintf(stderr, "\n");
                    }
                }
            }
            /* 0x10001504: BEQ error if assembled [r3+0x18..0x1B] == 0 */
            if (cur_pc == 0x10001506) {
                static int b04_done = 0;
                if (!b04_done) {
                    b04_done = 1;
                    uint32_t r2 = cores[CORE0].r[2];
                    uint32_t r3 = cores[CORE0].r[3];
                    EMU_ELOG(2, "[CHECK_0x18] r2(assembled32)=0x%08X r3(struct)=0x%08X\n", r2, r3);
                    if (bramble_verbose >= 2) {
                        fprintf(stderr, "[STRUCT_NOW@0x%08X]", r3);
                        for (int k = 0; k < 48; k++)
                            fprintf(stderr, " %02X", mem_read8(r3 + k));
                        fprintf(stderr, "\n");
                    }
                }
            }
            /* After LDRB r3,[r3,#5] at 0x10001506 → cur_pc==0x10001508: r3=byte5 value */
            if (cur_pc == 0x10001508) {
                static int b06_done = 0;
                if (!b06_done) {
                    b06_done = 1;
                    uint32_t byte5 = cores[CORE0].r[3];
                    EMU_ELOG(2, "[BYTE5_CHECK] byte5=0x%02X (%u) expected {2,3,4,8}\n",
                            (uint8_t)byte5, (uint8_t)byte5);
                }
            }
            /* 0x1000150A: after SUBS r2,r3,#2 and CMP r2,#2: BLS check */
            /* 0x10001510: BNE error if byte5 not in {2,3,4,8} */
            if (cur_pc == 0x10001512) {
                EMU_ELOG(2, "[BYTE5_PASS] byte5 in expected set, continuing\n");
            }
            /* 0x10001588: error output function call — this is the failure */
            if (cur_pc == 0x10001588 || cur_pc == 0x1001C5A4) {
                error_logged = 1;
                EMU_ELOG(2, "[ERROR_PATH] at step=%llu PC=0x%08X\n",
                        (unsigned long long)total_steps_count, cur_pc);
                EMU_ELOG(2, "[ERROR_REGS] r0-r3: 0x%08X 0x%08X 0x%08X 0x%08X\n",
                        cores[CORE0].r[0], cores[CORE0].r[1],
                        cores[CORE0].r[2], cores[CORE0].r[3]);
            }
            /* 0x1000159C: success path */
            if (cur_pc == 0x1000159C) {
                EMU_ELOG(2, "[SUCCESS_PATH] r3=0x%08X at step=%llu\n",
                        cores[CORE0].r[3], (unsigned long long)total_steps_count);
            }
        }
    }
    return (uint32_t)total_steps_count;
}





