#include <stdio.h>
#include <string.h>
#include "cyw43.h"
#include "emulator.h"

/* CYW43439 chip ID */
#define CYW43439_CHIP_ID 0x00A9A6A7

/* WiFi GPIO pin assignments (Pico W) */
#define WL_CS   23
#define WL_CLK  24
#define WL_DIO  25

cyw43_state_t cyw43;

/* Default MAC address (locally administered) */
static const uint8_t default_mac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE};

void cyw43_init(void) {
    cyw43_reset();

    /* Add some default fake APs for testing */
    cyw43_add_scan_result("BrambleNet", -45, 6, 3);      /* WPA2, strong signal */
    cyw43_add_scan_result("PicoTestAP", -60, 1, 3);      /* WPA2, medium signal */
    cyw43_add_scan_result("OpenNetwork", -70, 11, 0);     /* Open, weak signal */
}

void cyw43_reset(void) {
    memset(&cyw43, 0, sizeof(cyw43_state_t));
    memcpy(cyw43.mac_addr, default_mac, 6);
    strcpy(cyw43.country, "XX");  /* World-wide */
    cyw43.wifi_state = CYW43_WIFI_OFF;
    cyw43.scan_count = 0;
}

int cyw43_is_wifi_gpio(uint32_t gpio) {
    return cyw43.enabled && (gpio == WL_CS || gpio == WL_CLK || gpio == WL_DIO);
}

/* Process a gSPI bus register read (function 0) */
static uint32_t cyw43_bus_read(uint32_t addr) {
    switch (addr & 0xFF) {
    case CYW43_REG_BUS_CTRL:
        return cyw43.bus_ctrl;
    case CYW43_REG_BUS_INTERRUPT:
        return cyw43.bus_int;
    case CYW43_REG_BUS_INTMASK:
        return cyw43.bus_intmask;
    case CYW43_REG_BUS_STATUS:
        return cyw43.bus_status | 0x01;  /* Ready bit */
    case CYW43_REG_BUS_FEEDBEAD:
        return 0xFEEDBEAD;  /* Magic identification value */
    case CYW43_REG_BUS_TEST:
        return cyw43.bus_test_reg;
    default:
        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] Bus read unknown reg 0x%02X\n", addr & 0xFF);
        return 0;
    }
}

/* Process a gSPI bus register write (function 0) */
static void cyw43_bus_write(uint32_t addr, uint32_t val) {
    switch (addr & 0xFF) {
    case CYW43_REG_BUS_CTRL:
        cyw43.bus_ctrl = val;
        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] Bus ctrl = 0x%08X\n", val);
        break;
    case CYW43_REG_BUS_INTERRUPT:
        cyw43.bus_int &= ~val;  /* W1C */
        break;
    case CYW43_REG_BUS_INTMASK:
        cyw43.bus_intmask = val;
        break;
    case CYW43_REG_BUS_TEST:
        cyw43.bus_test_reg = val;
        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] Test reg = 0x%08X\n", val);
        break;
    default:
        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] Bus write unknown reg 0x%02X = 0x%08X\n",
                    addr & 0xFF, val);
        break;
    }
}

/* Process a backplane read (function 1) */
static uint32_t cyw43_backplane_read(uint32_t addr) {
    uint32_t full_addr = (cyw43.bp_window & 0xFFFF8000) | (addr & 0x7FFF);

    /* Chip ID at base of chip common */
    if ((full_addr & 0xFFF) == 0x000) {
        return CYW43439_CHIP_ID;
    }

    if (cpu.debug_enabled)
        fprintf(stderr, "[CYW43] Backplane read 0x%08X (win=0x%08X, off=0x%04X)\n",
                full_addr, cyw43.bp_window, addr & 0x7FFF);
    return 0;
}

/* Process a backplane write (function 1) */
static void cyw43_backplane_write(uint32_t addr, uint32_t val) {
    /* Window address register (0x1000A) is special */
    if (addr == 0x1000A || addr == 0x0A) {
        cyw43.bp_window = val;
        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] Backplane window = 0x%08X\n", val);
        return;
    }

    if (cpu.debug_enabled)
        fprintf(stderr, "[CYW43] Backplane write 0x%04X = 0x%08X\n", addr, val);
}

/* Process a completed gSPI command */
static void cyw43_process_command(void) {
    cyw43_spi_state_t *s = &cyw43.spi;
    uint32_t cmd = s->cmd_word;

    s->is_write   = (cmd >> 31) & 1;
    s->function   = (cmd >> 28) & 0x7;
    s->address    = (cmd >> 17) & 0x7FF;
    s->size       = cmd & 0x1FFFF;
    s->in_data_phase = 1;
    s->data_offset = 0;

    if (cpu.debug_enabled)
        fprintf(stderr, "[CYW43] gSPI cmd: %s func=%d addr=0x%03X size=%d\n",
                s->is_write ? "WR" : "RD", s->function, s->address, s->size);

    /* For reads, prepare response buffer */
    if (!s->is_write) {
        memset(s->resp_buf, 0, sizeof(s->resp_buf));
        s->resp_offset = 0;

        uint32_t val = 0;
        switch (s->function) {
        case CYW43_FUNC_BUS:
            val = cyw43_bus_read(s->address);
            break;
        case CYW43_FUNC_BACKPLANE:
            val = cyw43_backplane_read(s->address);
            break;
        case CYW43_FUNC_WLAN:
            /* WLAN reads return empty for now */
            break;
        }

        /* Store response in little-endian */
        s->resp_buf[0] = (val >>  0) & 0xFF;
        s->resp_buf[1] = (val >>  8) & 0xFF;
        s->resp_buf[2] = (val >> 16) & 0xFF;
        s->resp_buf[3] = (val >> 24) & 0xFF;
        s->resp_len = (s->size > 0) ? s->size : 4;
    }
}

/* Process write data byte from firmware */
static void cyw43_write_data(uint8_t byte) {
    cyw43_spi_state_t *s = &cyw43.spi;

    /* Accumulate bytes into a 32-bit value */
    static uint32_t write_accum = 0;
    static int write_byte_count = 0;

    write_accum |= ((uint32_t)byte << (write_byte_count * 8));
    write_byte_count++;

    if (write_byte_count >= 4 || s->data_offset + 1 >= s->size) {
        switch (s->function) {
        case CYW43_FUNC_BUS:
            cyw43_bus_write(s->address, write_accum);
            break;
        case CYW43_FUNC_BACKPLANE:
            cyw43_backplane_write(s->address, write_accum);
            break;
        case CYW43_FUNC_WLAN:
            /* WLAN writes consumed silently */
            break;
        }
        write_accum = 0;
        write_byte_count = 0;
    }

    s->data_offset++;
}

/* DIO output value for firmware reads */
static uint8_t dio_out_value = 0;
static int dio_out_bit = 0;

/* Clock a single SPI bit (called on CLK rising edge) */
static void cyw43_clock_bit(int dio_in) {
    cyw43_spi_state_t *s = &cyw43.spi;

    if (!s->in_data_phase) {
        /* Command phase: shift in 32 command bits */
        s->cmd_word = (s->cmd_word << 1) | (dio_in & 1);
        s->cmd_bits++;

        if (s->cmd_bits >= 32) {
            cyw43_process_command();
        }
    } else if (s->is_write) {
        /* Write data phase: receive bits from firmware */
        static uint8_t byte_accum = 0;
        static int bit_count = 0;

        byte_accum = (byte_accum << 1) | (dio_in & 1);
        bit_count++;

        if (bit_count >= 8) {
            cyw43_write_data(byte_accum);
            byte_accum = 0;
            bit_count = 0;
        }
    } else {
        /* Read data phase: send bits to firmware */
        if (dio_out_bit == 0) {
            if (s->resp_offset < s->resp_len) {
                dio_out_value = s->resp_buf[s->resp_offset++];
            } else {
                dio_out_value = 0;
            }
        }
        /* MSB first */
        dio_out_bit++;
        if (dio_out_bit >= 8) {
            dio_out_bit = 0;
        }
    }
}

int cyw43_gpio_intercept(uint32_t gpio, uint32_t value) {
    if (!cyw43.enabled) return 0;

    switch (gpio) {
    case WL_CS:
        if (value == 0) {
            /* CS asserted (active low) - start transaction */
            cyw43.spi.cs_active = 1;
            cyw43.spi.cmd_word = 0;
            cyw43.spi.cmd_bits = 0;
            cyw43.spi.in_data_phase = 0;
            cyw43.spi.data_offset = 0;
            dio_out_bit = 0;
        } else {
            /* CS deasserted - end transaction */
            cyw43.spi.cs_active = 0;
        }
        return 1;

    case WL_CLK:
        if (cyw43.spi.cs_active && value == 1) {
            /* Rising edge - clock data */
            /* Read current DIO state (firmware-driven value for writes) */
            /* The DIO pin value comes from GPIO output register */
            int dio_in = 0;  /* Will be set by DIO write */
            cyw43_clock_bit(dio_in);
        }
        return 1;

    case WL_DIO:
        /* Firmware setting DIO value - store for next clock edge */
        /* This is handled implicitly through GPIO state */
        return 1;

    default:
        return 0;
    }
}

uint32_t cyw43_gpio_read_dio(void) {
    if (!cyw43.enabled) return 0;

    cyw43_spi_state_t *s = &cyw43.spi;
    if (!s->cs_active || s->is_write || !s->in_data_phase) {
        return 0;
    }

    /* Return current bit of response (MSB first) */
    return (dio_out_value >> (7 - dio_out_bit)) & 1;
}

void cyw43_add_scan_result(const char *ssid, int rssi, int channel, int auth) {
    if (cyw43.scan_count >= CYW43_MAX_SCAN_RESULTS) return;

    cyw43_scan_result_t *r = &cyw43.scan_results[cyw43.scan_count];
    strncpy(r->ssid, ssid, CYW43_MAX_SSID_LEN);
    r->ssid[CYW43_MAX_SSID_LEN] = '\0';
    r->rssi = (int16_t)rssi;
    r->channel = (uint8_t)channel;
    r->auth_mode = (uint8_t)auth;

    /* Generate a fake BSSID from SSID hash */
    uint32_t hash = 0x12345678;
    for (const char *p = ssid; *p; p++) {
        hash = hash * 31 + *p;
    }
    r->bssid[0] = 0x02;  /* Locally administered */
    r->bssid[1] = (hash >> 24) & 0xFF;
    r->bssid[2] = (hash >> 16) & 0xFF;
    r->bssid[3] = (hash >>  8) & 0xFF;
    r->bssid[4] = (hash >>  0) & 0xFF;
    r->bssid[5] = cyw43.scan_count;

    cyw43.scan_count++;
}

/* ========================================================================
 * PIO-Level gSPI Protocol Handling
 *
 * The Pico SDK CYW43 driver uses PIO0 SM0 for gSPI communication.
 * The PIO program sends/receives 32-bit words. The protocol is:
 *   1. First TX word = gSPI command (write/function/address/size)
 *   2. For writes: subsequent TX words = data
 *   3. For reads: RX FIFO returns response data
 *
 * The PIO program handles bit-banging, clock, and CS internally.
 * At the FIFO level, we see clean 32-bit word transactions.
 * ======================================================================== */

/* PIO transaction state */
static enum {
    PIO_CYW43_IDLE,      /* Waiting for command word */
    PIO_CYW43_WRITE,     /* Receiving write data */
    PIO_CYW43_READ       /* Read response available */
} pio_cyw43_phase = PIO_CYW43_IDLE;

static uint32_t pio_cmd_function;
static uint32_t pio_cmd_address;
static uint32_t pio_cmd_size;
static uint32_t pio_words_remaining;

/* Response buffer for reads */
static uint32_t pio_resp_buf[64];
static int pio_resp_count;
static int pio_resp_idx;

void cyw43_pio_tx_write(uint32_t val) {
    if (!cyw43.enabled) return;

    switch (pio_cyw43_phase) {
    case PIO_CYW43_IDLE: {
        /* This is a gSPI command word */
        int is_write  = (val >> 31) & 1;
        pio_cmd_function = (val >> 28) & 0x7;
        pio_cmd_address  = (val >> 17) & 0x7FF;
        pio_cmd_size     = val & 0x1FFFF;

        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] PIO gSPI: %s func=%d addr=0x%03X size=%d\n",
                    is_write ? "WR" : "RD", pio_cmd_function,
                    pio_cmd_address, pio_cmd_size);

        if (is_write) {
            pio_cyw43_phase = PIO_CYW43_WRITE;
            pio_words_remaining = (pio_cmd_size + 3) / 4;
            if (pio_words_remaining == 0) pio_words_remaining = 1;
        } else {
            /* Prepare read response */
            pio_resp_count = 0;
            pio_resp_idx = 0;
            uint32_t resp = 0;

            switch (pio_cmd_function) {
            case CYW43_FUNC_BUS:
                resp = cyw43_bus_read(pio_cmd_address);
                break;
            case CYW43_FUNC_BACKPLANE:
                resp = cyw43_backplane_read(pio_cmd_address);
                break;
            case CYW43_FUNC_WLAN:
                resp = 0;
                break;
            }

            pio_resp_buf[0] = resp;
            pio_resp_count = 1;

            /* For larger reads, fill additional words */
            int total_words = (pio_cmd_size + 3) / 4;
            if (total_words > 1 && total_words <= 64) {
                for (int i = 1; i < total_words; i++) {
                    pio_resp_buf[i] = 0;
                }
                pio_resp_count = total_words;
            }

            pio_cyw43_phase = PIO_CYW43_READ;
        }
        break;
    }

    case PIO_CYW43_WRITE: {
        /* Receive write data word */
        switch (pio_cmd_function) {
        case CYW43_FUNC_BUS:
            cyw43_bus_write(pio_cmd_address, val);
            break;
        case CYW43_FUNC_BACKPLANE:
            cyw43_backplane_write(pio_cmd_address, val);
            break;
        case CYW43_FUNC_WLAN:
            /* Consume silently */
            break;
        }
        pio_cmd_address += 4;  /* Auto-increment for multi-word */

        if (--pio_words_remaining <= 0) {
            pio_cyw43_phase = PIO_CYW43_IDLE;
        }
        break;
    }

    case PIO_CYW43_READ:
        /* TX write during read phase = end of read / new command */
        /* Treat as new command */
        pio_cyw43_phase = PIO_CYW43_IDLE;
        cyw43_pio_tx_write(val);  /* Re-process as command */
        break;
    }
}

uint32_t cyw43_pio_rx_read(void) {
    if (!cyw43.enabled) return 0;

    if (pio_cyw43_phase == PIO_CYW43_READ && pio_resp_idx < pio_resp_count) {
        uint32_t val = pio_resp_buf[pio_resp_idx++];
        if (pio_resp_idx >= pio_resp_count) {
            pio_cyw43_phase = PIO_CYW43_IDLE;
        }
        return val;
    }

    /* Not in a read phase - return status word */
    return 0;
}

int cyw43_pio_rx_ready(void) {
    return (pio_cyw43_phase == PIO_CYW43_READ && pio_resp_idx < pio_resp_count);
}
