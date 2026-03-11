#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>
#include "cyw43.h"
#include "tapif.h"
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

/* ========================================================================
 * RX Queue Management
 * ======================================================================== */

static int rx_queue_count(void) {
    return (cyw43.rx_head - cyw43.rx_tail + CYW43_RX_QUEUE_SIZE) % CYW43_RX_QUEUE_SIZE;
}

static int rx_queue_push(const uint8_t *data, int len) {
    if (rx_queue_count() >= CYW43_RX_QUEUE_SIZE - 1) return -1;
    if (len > CYW43_MAX_FRAME_SIZE) return -1;

    cyw43_rx_frame_t *f = &cyw43.rx_queue[cyw43.rx_head];
    memcpy(f->data, data, len);
    f->len = len;
    cyw43.rx_head = (cyw43.rx_head + 1) % CYW43_RX_QUEUE_SIZE;
    return 0;
}

static cyw43_rx_frame_t *rx_queue_peek(void) {
    if (rx_queue_count() == 0) return NULL;
    return &cyw43.rx_queue[cyw43.rx_tail];
}

static void rx_queue_pop(void) {
    if (rx_queue_count() > 0)
        cyw43.rx_tail = (cyw43.rx_tail + 1) % CYW43_RX_QUEUE_SIZE;
}

/* ========================================================================
 * SDPCM Frame Construction Helpers
 * ======================================================================== */

static void sdpcm_fill_header(uint8_t *buf, int total_size, uint8_t channel, uint8_t hdr_len) {
    sdpcm_header_t *h = (sdpcm_header_t *)buf;
    memset(h, 0, sizeof(*h));
    h->size = (uint16_t)total_size;
    h->size_com = ~h->size;
    h->sequence = cyw43.tx_seq++;
    h->channel_and_flags = channel;
    h->header_length = hdr_len;
    h->bus_data_credit = (uint8_t)(cyw43.last_fw_seq + 4);
}

/* Build an IOCTL response and queue it */
static void cyw43_queue_ioctl_response(uint32_t cmd, uint16_t ioctl_id,
                                        const uint8_t *payload, int payload_len,
                                        uint32_t status) {
    uint8_t frame[CYW43_MAX_FRAME_SIZE];
    int offset = 0;

    /* SDPCM header (12 bytes) - control channel, no pad */
    int total = 12 + 16 + payload_len;
    sdpcm_fill_header(frame, total, SDPCM_CONTROL_CHANNEL, 12);
    offset = 12;

    /* CDC header (16 bytes) */
    cdc_header_t *cdc = (cdc_header_t *)(frame + offset);
    cdc->cmd = cmd;
    cdc->len = (uint32_t)payload_len;
    cdc->flags = ((uint32_t)ioctl_id << CDC_ID_SHIFT) | CDC_REPLY;
    cdc->status = status;
    offset += 16;

    /* Payload */
    if (payload && payload_len > 0) {
        memcpy(frame + offset, payload, payload_len);
        offset += payload_len;
    }

    rx_queue_push(frame, offset);
}

/* Build an async event and queue it */
static void cyw43_queue_event(uint32_t event_type, uint32_t status,
                               uint32_t reason, uint32_t auth_type) {
    uint8_t frame[256];
    int offset = 0;

    /* Reserve space for SDPCM (12) + pad (2) + BDC (4) = 18 bytes */
    offset = 18;

    /* Ethernet header (14 bytes) */
    memset(frame + offset, 0xFF, 6);                    /* dst: broadcast */
    memcpy(frame + offset + 6, cyw43.mac_addr, 6);      /* src: device MAC */
    frame[offset + 12] = 0x88;                           /* ethertype: 0x886C */
    frame[offset + 13] = 0x6C;
    offset += 14;

    /* Broadcom event header */
    frame[offset++] = 0x00; frame[offset++] = 0x01;     /* subtype = 1 (BE) */
    /* length placeholder - fill later */
    int len_offset = offset;
    frame[offset++] = 0x00; frame[offset++] = 0x00;
    frame[offset++] = 0x00; frame[offset++] = 0x10; frame[offset++] = 0x18; /* OUI */
    frame[offset++] = 0x80; frame[offset++] = 0x02;     /* usr_subtype (BE) */

    /* Event fields (all big-endian) */
    frame[offset++] = 0x00; frame[offset++] = 0x02;     /* version = 2 */
    frame[offset++] = 0x00; frame[offset++] = 0x00;     /* flags = 0 */

    /* event_type (BE u32) */
    frame[offset++] = (event_type >> 24) & 0xFF;
    frame[offset++] = (event_type >> 16) & 0xFF;
    frame[offset++] = (event_type >>  8) & 0xFF;
    frame[offset++] = (event_type >>  0) & 0xFF;

    /* status (BE u32) */
    frame[offset++] = (status >> 24) & 0xFF;
    frame[offset++] = (status >> 16) & 0xFF;
    frame[offset++] = (status >>  8) & 0xFF;
    frame[offset++] = (status >>  0) & 0xFF;

    /* reason (BE u32) */
    frame[offset++] = (reason >> 24) & 0xFF;
    frame[offset++] = (reason >> 16) & 0xFF;
    frame[offset++] = (reason >>  8) & 0xFF;
    frame[offset++] = (reason >>  0) & 0xFF;

    /* auth_type (BE u32) */
    frame[offset++] = (auth_type >> 24) & 0xFF;
    frame[offset++] = (auth_type >> 16) & 0xFF;
    frame[offset++] = (auth_type >>  8) & 0xFF;
    frame[offset++] = (auth_type >>  0) & 0xFF;

    /* datalen (BE u32) = 0 */
    frame[offset++] = 0; frame[offset++] = 0;
    frame[offset++] = 0; frame[offset++] = 0;

    /* addr (6 bytes) = device MAC */
    memcpy(frame + offset, cyw43.mac_addr, 6);
    offset += 6;

    /* ifname (16 bytes) = "wl0" */
    memset(frame + offset, 0, 16);
    frame[offset] = 'w'; frame[offset+1] = 'l'; frame[offset+2] = '0';
    offset += 16;

    /* ifidx, bsscfgidx */
    frame[offset++] = 0;  /* ifidx */
    frame[offset++] = 0;  /* bsscfgidx */

    /* Fill length field (length of event data from version onward) */
    uint16_t ev_len = (uint16_t)(offset - len_offset - 2 - 3 - 2);
    frame[len_offset] = (ev_len >> 8) & 0xFF;
    frame[len_offset + 1] = ev_len & 0xFF;

    /* Now fill SDPCM + pad + BDC at the beginning */
    sdpcm_fill_header(frame, offset, SDPCM_EVENT_CHANNEL, 14);

    /* 2-byte pad at offset 12 */
    frame[12] = 0; frame[13] = 0;

    /* BDC header at offset 14 */
    bdc_header_t *bdc = (bdc_header_t *)(frame + 14);
    bdc->flags = 0x20;
    bdc->priority = 0;
    bdc->flags2 = 0;
    bdc->data_offset = 0;

    rx_queue_push(frame, offset);
}

/* Queue connection events (simulates successful WiFi join) */
static void cyw43_queue_connect_events(void) {
    cyw43_queue_event(CYW43_EV_AUTH, CYW43_STATUS_SUCCESS, 0, 0);
    cyw43_queue_event(CYW43_EV_LINK, CYW43_STATUS_SUCCESS, 0, 0);
    cyw43_queue_event(CYW43_EV_SET_SSID, CYW43_STATUS_SUCCESS, 0, 0);
    cyw43_queue_event(CYW43_EV_PSK_SUP, CYW43_SUP_KEYED, 0, 0);
    cyw43.wifi_state = CYW43_WIFI_CONNECTED;

    if (cpu.debug_enabled)
        fprintf(stderr, "[CYW43] Queued connection events for SSID '%s'\n",
                cyw43.connected_ssid);
}

/* Wrap an Ethernet frame from TAP in SDPCM + BDC and queue for firmware */
static void cyw43_queue_rx_data(const uint8_t *eth_frame, int eth_len) {
    uint8_t frame[CYW43_MAX_FRAME_SIZE];
    int total = 18 + eth_len;  /* SDPCM(12) + pad(2) + BDC(4) + ethernet */

    if (total > CYW43_MAX_FRAME_SIZE) return;

    sdpcm_fill_header(frame, total, SDPCM_DATA_CHANNEL, 14);

    /* 2-byte pad */
    frame[12] = 0; frame[13] = 0;

    /* BDC header */
    bdc_header_t *bdc = (bdc_header_t *)(frame + 14);
    bdc->flags = 0x20;
    bdc->priority = 0;
    bdc->flags2 = 0;
    bdc->data_offset = 0;

    /* Ethernet frame */
    memcpy(frame + 18, eth_frame, eth_len);

    rx_queue_push(frame, total);
}

/* ========================================================================
 * IOCTL Handler
 * ======================================================================== */

static void cyw43_handle_ioctl(const uint8_t *buf, int len) {
    if (len < 12 + 16) return;  /* SDPCM + CDC minimum */

    sdpcm_header_t *sdpcm = (sdpcm_header_t *)buf;
    cdc_header_t *cdc = (cdc_header_t *)(buf + 12);

    cyw43.last_fw_seq = sdpcm->sequence;

    uint32_t cmd = cdc->cmd;
    uint16_t ioctl_id = (uint16_t)(cdc->flags >> CDC_ID_SHIFT);
    int is_set = (cdc->flags & 0x02) != 0;
    const uint8_t *payload = buf + 28;  /* After SDPCM(12) + CDC(16) */
    int payload_len = len - 28;

    if (cpu.debug_enabled)
        fprintf(stderr, "[CYW43] IOCTL cmd=%d %s id=%d payload_len=%d\n",
                cmd, is_set ? "SET" : "GET", ioctl_id, payload_len);

    switch (cmd) {
    case WLC_GET_VAR: {
        /* payload is null-terminated iovar name */
        const char *varname = (const char *)payload;

        if (strcmp(varname, "cur_etheraddr") == 0) {
            cyw43_queue_ioctl_response(cmd, ioctl_id, cyw43.mac_addr, 6, 0);
            return;
        }
        if (strcmp(varname, "ver") == 0) {
            const char *ver = "wl0: Bramble CYW43 Emulator\n";
            cyw43_queue_ioctl_response(cmd, ioctl_id,
                                        (const uint8_t *)ver, (int)strlen(ver) + 1, 0);
            return;
        }
        /* Default: return zeros */
        uint8_t zeros[256];
        memset(zeros, 0, sizeof(zeros));
        int resp_len = payload_len > 0 ? payload_len : 4;
        if (resp_len > (int)sizeof(zeros)) resp_len = (int)sizeof(zeros);
        cyw43_queue_ioctl_response(cmd, ioctl_id, zeros, resp_len, 0);
        return;
    }

    case WLC_SET_SSID: {
        /* Payload: 4 bytes SSID length + SSID string (32 bytes) */
        if (payload_len >= 36) {
            uint32_t ssid_len = payload[0] | (payload[1] << 8) |
                                (payload[2] << 16) | (payload[3] << 24);
            if (ssid_len > CYW43_MAX_SSID_LEN) ssid_len = CYW43_MAX_SSID_LEN;
            memcpy(cyw43.connected_ssid, payload + 4, ssid_len);
            cyw43.connected_ssid[ssid_len] = '\0';

            fprintf(stderr, "[CYW43] JOIN SSID: '%s'\n", cyw43.connected_ssid);
        }

        /* Respond success */
        cyw43_queue_ioctl_response(cmd, ioctl_id, NULL, 0, 0);

        /* Queue connection events */
        cyw43_queue_connect_events();
        return;
    }

    case WLC_GET_SSID: {
        /* Return current SSID: 4 bytes length + 32 bytes SSID */
        uint8_t resp[36];
        memset(resp, 0, sizeof(resp));
        uint32_t slen = (uint32_t)strlen(cyw43.connected_ssid);
        resp[0] = slen & 0xFF;
        resp[1] = (slen >> 8) & 0xFF;
        resp[2] = (slen >> 16) & 0xFF;
        resp[3] = (slen >> 24) & 0xFF;
        memcpy(resp + 4, cyw43.connected_ssid, slen);
        cyw43_queue_ioctl_response(cmd, ioctl_id, resp, 36, 0);
        return;
    }

    case WLC_GET_BSSID: {
        /* Return a fake BSSID */
        uint8_t bssid[6] = {0x02, 0xCA, 0xFE, 0xBA, 0xBE, 0x01};
        cyw43_queue_ioctl_response(cmd, ioctl_id, bssid, 6, 0);
        return;
    }

    default:
        /* All other IOCTLs: return success with echo payload for GET, empty for SET */
        if (is_set) {
            cyw43_queue_ioctl_response(cmd, ioctl_id, NULL, 0, 0);
        } else {
            /* GET: return payload echoed (or zeros) */
            uint8_t zeros[256];
            memset(zeros, 0, sizeof(zeros));
            int resp_len = payload_len > 0 ? payload_len : 4;
            if (resp_len > (int)sizeof(zeros)) resp_len = (int)sizeof(zeros);
            cyw43_queue_ioctl_response(cmd, ioctl_id, zeros, resp_len, 0);
        }
        return;
    }
}

/* ========================================================================
 * WLAN TX Processing (firmware -> CYW43)
 * ======================================================================== */

static void cyw43_wlan_tx_word(uint32_t val) {
    if (cyw43.wlan_tx_offset + 4 <= CYW43_WLAN_TX_BUF_SIZE) {
        cyw43.wlan_tx_buf[cyw43.wlan_tx_offset++] = (val >>  0) & 0xFF;
        cyw43.wlan_tx_buf[cyw43.wlan_tx_offset++] = (val >>  8) & 0xFF;
        cyw43.wlan_tx_buf[cyw43.wlan_tx_offset++] = (val >> 16) & 0xFF;
        cyw43.wlan_tx_buf[cyw43.wlan_tx_offset++] = (val >> 24) & 0xFF;
    }
}

static void cyw43_wlan_tx_complete(void) {
    int len = cyw43.wlan_tx_offset;
    if (len < 12) {
        cyw43.wlan_tx_offset = 0;
        return;
    }

    sdpcm_header_t *sdpcm = (sdpcm_header_t *)cyw43.wlan_tx_buf;
    cyw43.last_fw_seq = sdpcm->sequence;

    uint8_t channel = sdpcm->channel_and_flags & 0x0F;

    if (cpu.debug_enabled)
        fprintf(stderr, "[CYW43] WLAN TX: size=%d channel=%d seq=%d\n",
                sdpcm->size, channel, sdpcm->sequence);

    switch (channel) {
    case SDPCM_CONTROL_CHANNEL:
        cyw43_handle_ioctl(cyw43.wlan_tx_buf, len);
        break;

    case SDPCM_DATA_CHANNEL: {
        /* Extract Ethernet frame: skip SDPCM(12) + pad(2) + BDC(4) = 18 bytes */
        if (len > 18 && cyw43.tap_fd >= 0) {
            int eth_offset = 18;
            /* Check BDC data_offset for extra padding */
            bdc_header_t *bdc = (bdc_header_t *)(cyw43.wlan_tx_buf + 14);
            eth_offset += bdc->data_offset * 4;

            int eth_len = len - eth_offset;
            if (eth_len > 0) {
                tapif_write(cyw43.tap_fd, cyw43.wlan_tx_buf + eth_offset, eth_len);
                if (cpu.debug_enabled)
                    fprintf(stderr, "[CYW43] TAP TX: %d bytes\n", eth_len);
            }
        }
        break;
    }

    default:
        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] WLAN TX unknown channel %d\n", channel);
        break;
    }

    cyw43.wlan_tx_offset = 0;
}

/* ========================================================================
 * Init / Reset
 * ======================================================================== */

void cyw43_init(void) {
    cyw43_reset();

    /* Add default fake APs for testing */
    cyw43_add_scan_result("BrambleNet", -45, 6, 3);
    cyw43_add_scan_result("PicoTestAP", -60, 1, 3);
    cyw43_add_scan_result("OpenNetwork", -70, 11, 0);
}

void cyw43_reset(void) {
    int saved_enabled = cyw43.enabled;
    int saved_tap_fd = cyw43.tap_fd;
    char saved_tap_name[32];
    memcpy(saved_tap_name, cyw43.tap_name, sizeof(saved_tap_name));

    memset(&cyw43, 0, sizeof(cyw43_state_t));

    cyw43.enabled = saved_enabled;
    cyw43.tap_fd = saved_tap_fd;
    memcpy(cyw43.tap_name, saved_tap_name, sizeof(cyw43.tap_name));

    memcpy(cyw43.mac_addr, default_mac, 6);
    strcpy(cyw43.country, "XX");
    cyw43.wifi_state = CYW43_WIFI_OFF;
    cyw43.chipclkcsr = CYW43_HT_AVAIL | CYW43_ALP_AVAIL;
    cyw43.pio_num = -1;
    cyw43.pio_sm = -1;
}

int cyw43_is_wifi_gpio(uint32_t gpio) {
    return cyw43.enabled && (gpio == WL_CS || gpio == WL_CLK || gpio == WL_DIO);
}

/* ========================================================================
 * TAP Bridge
 * ======================================================================== */

int cyw43_tap_open(const char *name) {
    cyw43.tap_fd = tapif_open(name);
    if (cyw43.tap_fd >= 0) {
        strncpy(cyw43.tap_name, name, sizeof(cyw43.tap_name) - 1);
        fprintf(stderr, "[CYW43] TAP bridge enabled on '%s'\n", name);
    }
    return cyw43.tap_fd;
}

void cyw43_tap_close(void) {
    if (cyw43.tap_fd >= 0) {
        tapif_close(cyw43.tap_fd);
        cyw43.tap_fd = -1;
    }
}

void cyw43_tap_poll(void) {
    if (cyw43.tap_fd < 0) return;
    if (cyw43.wifi_state != CYW43_WIFI_CONNECTED) return;

    /* Read Ethernet frames from TAP and queue for firmware */
    uint8_t eth_buf[1600];
    int n = tapif_read(cyw43.tap_fd, eth_buf, sizeof(eth_buf));
    if (n > 0) {
        cyw43_queue_rx_data(eth_buf, n);
        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] TAP RX: %d bytes queued\n", n);
    }
}

/* ========================================================================
 * Bus Register Access (Function 0)
 * ======================================================================== */

static uint32_t cyw43_bus_read(uint32_t addr) {
    switch (addr & 0xFF) {
    case CYW43_REG_BUS_CTRL:
        return cyw43.bus_ctrl;
    case CYW43_REG_BUS_INTERRUPT: {
        uint32_t val = cyw43.bus_int;
        if (rx_queue_count() > 0)
            val |= CYW43_BUS_INT_F2_PKT_AVAIL;
        return val;
    }
    case CYW43_REG_BUS_INTMASK:
        return cyw43.bus_intmask;
    case CYW43_REG_BUS_STATUS: {
        uint32_t val = cyw43.bus_status | 0x01;  /* Ready bit */
        cyw43_rx_frame_t *f = rx_queue_peek();
        if (f) {
            val |= ((uint32_t)f->len & 0xFFFF) << 16;
        }
        return val;
    }
    case CYW43_REG_BUS_FEEDBEAD:
        return 0xFEEDBEAD;
    case CYW43_REG_BUS_TEST:
        return cyw43.bus_test_reg;
    default:
        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] Bus read unknown reg 0x%02X\n", addr & 0xFF);
        return 0;
    }
}

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

/* ========================================================================
 * Backplane Access (Function 1)
 * ======================================================================== */

static uint32_t cyw43_backplane_read(uint32_t addr) {
    uint32_t full_addr = (cyw43.bp_window & 0xFFFF8000) | (addr & 0x7FFF);

    /* Chip ID */
    if ((full_addr & 0xFFF) == 0x000) {
        return CYW43439_CHIP_ID;
    }

    /* CHIPCLKCSR - clock status */
    if (full_addr == CYW43_BP_CHIPCLKCSR) {
        return cyw43.chipclkcsr;
    }

    if (cpu.debug_enabled)
        fprintf(stderr, "[CYW43] Backplane read 0x%08X (win=0x%08X, off=0x%04X)\n",
                full_addr, cyw43.bp_window, addr & 0x7FFF);
    return 0;
}

static void cyw43_backplane_write(uint32_t addr, uint32_t val) {
    /* Window address register */
    if (addr == 0x1000A || addr == 0x0A) {
        cyw43.bp_window = val;
        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] Backplane window = 0x%08X\n", val);
        return;
    }

    uint32_t full_addr = (cyw43.bp_window & 0xFFFF8000) | (addr & 0x7FFF);

    /* CHIPCLKCSR write */
    if (full_addr == CYW43_BP_CHIPCLKCSR) {
        cyw43.chipclkcsr = val | CYW43_HT_AVAIL | CYW43_ALP_AVAIL;
        return;
    }

    if (cpu.debug_enabled)
        fprintf(stderr, "[CYW43] Backplane write 0x%04X = 0x%08X\n", addr, val);
}

/* ========================================================================
 * GPIO-Level gSPI (legacy, unused by Pico SDK PIO driver)
 * ======================================================================== */

static uint8_t dio_out_value = 0;
static int dio_out_bit = 0;

static void cyw43_clock_bit(int dio_in) {
    cyw43_spi_state_t *s = &cyw43.spi;

    if (!s->in_data_phase) {
        s->cmd_word = (s->cmd_word << 1) | (dio_in & 1);
        s->cmd_bits++;
        if (s->cmd_bits >= 32) {
            /* Would need to process command here */
            s->in_data_phase = 1;
        }
    } else if (s->is_write) {
        /* Write data phase */
    } else {
        /* Read data phase */
        if (dio_out_bit == 0) {
            if (s->resp_offset < s->resp_len)
                dio_out_value = s->resp_buf[s->resp_offset++];
            else
                dio_out_value = 0;
        }
        dio_out_bit++;
        if (dio_out_bit >= 8) dio_out_bit = 0;
    }
}

int cyw43_gpio_intercept(uint32_t gpio, uint32_t value) {
    if (!cyw43.enabled) return 0;

    switch (gpio) {
    case WL_CS:
        if (value == 0) {
            cyw43.spi.cs_active = 1;
            cyw43.spi.cmd_word = 0;
            cyw43.spi.cmd_bits = 0;
            cyw43.spi.in_data_phase = 0;
            cyw43.spi.data_offset = 0;
            dio_out_bit = 0;
        } else {
            cyw43.spi.cs_active = 0;
        }
        return 1;
    case WL_CLK:
        if (cyw43.spi.cs_active && value == 1) {
            int dio_in = 0;
            cyw43_clock_bit(dio_in);
        }
        return 1;
    case WL_DIO:
        return 1;
    default:
        return 0;
    }
}

uint32_t cyw43_gpio_read_dio(void) {
    if (!cyw43.enabled) return 0;
    cyw43_spi_state_t *s = &cyw43.spi;
    if (!s->cs_active || s->is_write || !s->in_data_phase) return 0;
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

    uint32_t hash = 0x12345678;
    for (const char *p = ssid; *p; p++)
        hash = hash * 31 + *p;
    r->bssid[0] = 0x02;
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
 * TX FIFO writes = gSPI command/data words
 * RX FIFO reads  = gSPI response words
 * ======================================================================== */

static enum {
    PIO_CYW43_IDLE,
    PIO_CYW43_WRITE,
    PIO_CYW43_READ
} pio_cyw43_phase = PIO_CYW43_IDLE;

static uint32_t pio_cmd_function;
static uint32_t pio_cmd_address;
static uint32_t pio_cmd_size;
static uint32_t pio_words_remaining;

static uint32_t pio_resp_buf[512];
static int pio_resp_count;
static int pio_resp_idx;

void cyw43_pio_tx_write(uint32_t val) {
    if (!cyw43.enabled) return;

    switch (pio_cyw43_phase) {
    case PIO_CYW43_IDLE: {
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

            /* Reset WLAN TX buffer for accumulation */
            if (pio_cmd_function == CYW43_FUNC_WLAN)
                cyw43.wlan_tx_offset = 0;
        } else {
            /* Prepare read response */
            pio_resp_count = 0;
            pio_resp_idx = 0;

            int total_words = (pio_cmd_size + 3) / 4;
            if (total_words <= 0) total_words = 1;
            if (total_words > 512) total_words = 512;

            memset(pio_resp_buf, 0, total_words * 4);

            switch (pio_cmd_function) {
            case CYW43_FUNC_BUS: {
                uint32_t resp = cyw43_bus_read(pio_cmd_address);
                pio_resp_buf[0] = resp;
                pio_resp_count = total_words;
                break;
            }
            case CYW43_FUNC_BACKPLANE: {
                uint32_t resp = cyw43_backplane_read(pio_cmd_address);
                pio_resp_buf[0] = resp;
                pio_resp_count = total_words;
                break;
            }
            case CYW43_FUNC_WLAN: {
                /* Return queued RX frame */
                cyw43_rx_frame_t *f = rx_queue_peek();
                if (f && f->len > 0) {
                    int copy_len = f->len;
                    if (copy_len > (int)pio_cmd_size) copy_len = (int)pio_cmd_size;
                    /* Copy frame bytes into response word buffer (LE) */
                    int words = (copy_len + 3) / 4;
                    if (words > 512) words = 512;
                    memset(pio_resp_buf, 0, words * 4);
                    memcpy(pio_resp_buf, f->data, copy_len);
                    pio_resp_count = total_words;
                    rx_queue_pop();

                    if (cpu.debug_enabled)
                        fprintf(stderr, "[CYW43] WLAN RX: delivering %d byte frame\n",
                                copy_len);
                } else {
                    /* No data - return empty SDPCM (size=0) */
                    pio_resp_count = total_words;
                }
                break;
            }
            }

            pio_cyw43_phase = PIO_CYW43_READ;
        }
        break;
    }

    case PIO_CYW43_WRITE: {
        switch (pio_cmd_function) {
        case CYW43_FUNC_BUS:
            cyw43_bus_write(pio_cmd_address, val);
            break;
        case CYW43_FUNC_BACKPLANE:
            cyw43_backplane_write(pio_cmd_address, val);
            break;
        case CYW43_FUNC_WLAN:
            cyw43_wlan_tx_word(val);
            break;
        }
        pio_cmd_address += 4;

        if (--pio_words_remaining <= 0) {
            if (pio_cmd_function == CYW43_FUNC_WLAN)
                cyw43_wlan_tx_complete();
            pio_cyw43_phase = PIO_CYW43_IDLE;
        }
        break;
    }

    case PIO_CYW43_READ:
        pio_cyw43_phase = PIO_CYW43_IDLE;
        cyw43_pio_tx_write(val);
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

    return 0;
}

int cyw43_pio_rx_ready(void) {
    return (pio_cyw43_phase == PIO_CYW43_READ && pio_resp_idx < pio_resp_count);
}
