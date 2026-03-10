/*
 * CYW43439 WiFi/BT Chip Emulation for Pico W
 *
 * The Pico W communicates with the CYW43439 via GPIO bit-banged SPI:
 *   GPIO 23 = WL_CS (chip select, directly driven)
 *   GPIO 24 = WL_CLK (clock, directly driven)
 *   GPIO 25 = WL_DIO (bidirectional data)
 *   GPIO 24 = WL_IRQ (active-low interrupt to RP2040, active via PIO)
 *
 * The Pico SDK cyw43_driver uses PIO for SPI bit-bang. We intercept
 * at the gSPI protocol level, decoding 32-bit command words.
 *
 * gSPI Protocol:
 *   Command word: [31]=write, [30:28]=function, [27:17]=address, [16:0]=size
 *   Functions: 0=backplane, 1=wlan (802.11), 2=bluetooth
 *
 * Usage: ./bramble firmware.uf2 -wifi
 */

#ifndef CYW43_H
#define CYW43_H

#include <stdint.h>

/* CYW43 gSPI functions */
#define CYW43_FUNC_BUS       0   /* Bus/SPI control */
#define CYW43_FUNC_BACKPLANE  1   /* Chip backplane (registers, memory) */
#define CYW43_FUNC_WLAN       2   /* WLAN (802.11 frames) */

/* Key SPI bus registers (function 0) */
#define CYW43_REG_BUS_FEEDBEAD    0x14  /* Should read 0xFEEDBEAD */
#define CYW43_REG_BUS_TEST        0x18  /* Test register (read/write) */
#define CYW43_REG_BUS_CTRL        0x00  /* Bus control */
#define CYW43_REG_BUS_INTERRUPT   0x04  /* Interrupt register */
#define CYW43_REG_BUS_INTMASK     0x08  /* Interrupt mask */
#define CYW43_REG_BUS_STATUS      0x0C  /* Status */

/* Backplane registers (function 1) */
#define CYW43_REG_BP_CHIPID       0x00  /* Chip identification */
#define CYW43_REG_BP_WIN_ADDR     0x1000A /* Backplane window address */

/* WiFi connection states */
#define CYW43_WIFI_OFF        0
#define CYW43_WIFI_SCANNING   1
#define CYW43_WIFI_CONNECTING 2
#define CYW43_WIFI_CONNECTED  3

/* Max scan results and SSIDs */
#define CYW43_MAX_SCAN_RESULTS  8
#define CYW43_MAX_SSID_LEN     32
#define CYW43_MAC_LEN           6

/* Scan result entry */
typedef struct {
    char ssid[CYW43_MAX_SSID_LEN + 1];
    uint8_t bssid[CYW43_MAC_LEN];
    int16_t rssi;
    uint8_t channel;
    uint8_t auth_mode;    /* 0=open, 1=WEP, 2=WPA, 3=WPA2 */
} cyw43_scan_result_t;

/* gSPI transaction state */
typedef struct {
    int cs_active;          /* Chip select asserted */
    uint32_t cmd_word;      /* Current 32-bit command being assembled */
    int cmd_bits;           /* Bits received in current command */
    int in_data_phase;      /* 1 = data phase (after command) */
    int is_write;           /* Current transaction direction */
    int function;           /* Target function (0/1/2) */
    uint32_t address;       /* Register address */
    uint32_t size;          /* Transfer size */
    uint32_t data_offset;   /* Bytes transferred in data phase */

    /* Response buffer for reads */
    uint8_t resp_buf[2048];
    uint32_t resp_len;
    uint32_t resp_offset;   /* Current read position */
} cyw43_spi_state_t;

/* CYW43 emulator state */
typedef struct {
    int enabled;            /* WiFi emulation active */
    int initialized;        /* Chip init sequence completed */

    /* gSPI state */
    cyw43_spi_state_t spi;

    /* Bus registers (function 0) */
    uint32_t bus_test_reg;      /* Test register (R/W echo) */
    uint32_t bus_ctrl;          /* Bus control */
    uint32_t bus_int;           /* Interrupt register */
    uint32_t bus_intmask;       /* Interrupt mask */
    uint32_t bus_status;        /* Status register */

    /* Backplane (function 1) */
    uint32_t bp_window;         /* Backplane address window */

    /* WiFi state */
    int wifi_state;             /* CYW43_WIFI_* */
    char connected_ssid[CYW43_MAX_SSID_LEN + 1];
    uint8_t mac_addr[CYW43_MAC_LEN];
    uint32_t ip_addr;           /* Assigned IP (network byte order) */

    /* Scan results */
    cyw43_scan_result_t scan_results[CYW43_MAX_SCAN_RESULTS];
    int scan_count;

    /* Country code */
    char country[4];            /* e.g. "US\0\0" */
} cyw43_state_t;

extern cyw43_state_t cyw43;

/* Initialize CYW43 emulation */
void cyw43_init(void);

/* Reset CYW43 to power-on defaults */
void cyw43_reset(void);

/* GPIO intercept: called when firmware drives WL_CS/CLK/DIO GPIOs.
 * Returns 1 if the access was consumed by CYW43, 0 otherwise. */
int cyw43_gpio_intercept(uint32_t gpio, uint32_t value);

/* Read WL_DIO value (for firmware SPI reads) */
uint32_t cyw43_gpio_read_dio(void);

/* Check if a GPIO is a CYW43 WiFi pin */
int cyw43_is_wifi_gpio(uint32_t gpio);

/* Add a fake AP to scan results (for testing) */
void cyw43_add_scan_result(const char *ssid, int rssi, int channel, int auth);

/* PIO-level intercept: called when firmware writes to PIO0 SM0 TX FIFO.
 * Processes gSPI command/data words from the CYW43 PIO program. */
void cyw43_pio_tx_write(uint32_t val);

/* PIO-level intercept: called when firmware reads PIO0 SM0 RX FIFO.
 * Returns gSPI response data. */
uint32_t cyw43_pio_rx_read(void);

/* Check if PIO RX has data available (for FSTAT reporting) */
int cyw43_pio_rx_ready(void);

#endif /* CYW43_H */
