/*
 * RP2040 RTC Peripheral
 *
 * Implements a real-time clock that ticks seconds when enabled.
 * SETUP_0/SETUP_1 configure date/time, CTRL.LOAD copies setup into
 * running registers, CTRL.ENABLE starts ticking.
 * RTC_1/RTC_0 return the current running time.
 *
 * RP2040 SETUP_0 / RTC_1 layout:
 *   [23:12] YEAR, [11:8] MONTH, [4:0] DAY
 * RP2040 SETUP_1 / RTC_0 layout:
 *   [26:24] DOTW, [20:16] HOUR, [13:8] MIN, [5:0] SEC
 */

#include <string.h>
#include "rtc.h"

rtc_state_t rtc_state;

void rtc_init(void) {
    memset(&rtc_state, 0, sizeof(rtc_state));
}

/* Days in each month */
static int days_in_month(int month, int year) {
    static const int dim[] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
    if (month < 1 || month > 12) return 30;
    if (month == 2) {
        if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))
            return 29;
    }
    return dim[month];
}

/* Advance RTC by one second */
static void rtc_advance_second(void) {
    rtc_state.sec++;
    if (rtc_state.sec >= 60) {
        rtc_state.sec = 0;
        rtc_state.min++;
        if (rtc_state.min >= 60) {
            rtc_state.min = 0;
            rtc_state.hour++;
            if (rtc_state.hour >= 24) {
                rtc_state.hour = 0;
                rtc_state.dotw = (rtc_state.dotw + 1) % 7;
                rtc_state.day++;
                if (rtc_state.day > days_in_month(rtc_state.month, rtc_state.year)) {
                    rtc_state.day = 1;
                    rtc_state.month++;
                    if (rtc_state.month > 12) {
                        rtc_state.month = 1;
                        rtc_state.year++;
                        if (rtc_state.year > 4095) rtc_state.year = 0;
                    }
                }
            }
        }
    }
}

/* Load SETUP values into running time registers */
static void rtc_load_setup(void) {
    rtc_state.year  = (rtc_state.setup_0 >> 12) & 0xFFF;
    rtc_state.month = (rtc_state.setup_0 >> 8)  & 0xF;
    rtc_state.day   = rtc_state.setup_0 & 0x1F;
    rtc_state.dotw  = (rtc_state.setup_1 >> 24) & 0x7;
    rtc_state.hour  = (rtc_state.setup_1 >> 16) & 0x1F;
    rtc_state.min   = (rtc_state.setup_1 >> 8)  & 0x3F;
    rtc_state.sec   = rtc_state.setup_1 & 0x3F;
    rtc_state.tick_acc = 0;
}

/* Pack current time into RTC_1 format (same as SETUP_0) */
static uint32_t rtc_pack_rtc1(void) {
    return ((uint32_t)(rtc_state.year & 0xFFF) << 12) |
           ((uint32_t)(rtc_state.month & 0xF) << 8) |
           ((uint32_t)(rtc_state.day & 0x1F));
}

/* Pack current time into RTC_0 format (same as SETUP_1) */
static uint32_t rtc_pack_rtc0(void) {
    return ((uint32_t)(rtc_state.dotw & 0x7) << 24) |
           ((uint32_t)(rtc_state.hour & 0x1F) << 16) |
           ((uint32_t)(rtc_state.min & 0x3F) << 8) |
           ((uint32_t)(rtc_state.sec & 0x3F));
}

int rtc_match(uint32_t addr) {
    uint32_t base = addr & ~0x3000;  /* Strip atomic alias bits */
    return (base >= RTC_BASE && base < RTC_BASE + RTC_BLOCK_SIZE) ? 1 : 0;
}

uint32_t rtc_read32(uint32_t offset) {
    switch (offset) {
    case RTC_CLKDIV_M1:   return rtc_state.clkdiv_m1;
    case RTC_SETUP_0:     return rtc_state.setup_0;
    case RTC_SETUP_1:     return rtc_state.setup_1;
    case RTC_CTRL:
        /* Report ACTIVE when ENABLE is set */
        if (rtc_state.ctrl & RTC_CTRL_ENABLE)
            return rtc_state.ctrl | RTC_CTRL_ACTIVE;
        return rtc_state.ctrl & ~RTC_CTRL_ACTIVE;
    case RTC_IRQ_SETUP_0: return rtc_state.irq_setup_0;
    case RTC_IRQ_SETUP_1: return rtc_state.irq_setup_1;
    case RTC_RTC_1:       return rtc_pack_rtc1();
    case RTC_RTC_0:       return rtc_pack_rtc0();
    case RTC_INTR:        return 0;
    case RTC_INTE:        return rtc_state.inte;
    case RTC_INTF:        return rtc_state.intf;
    case RTC_INTS:        return rtc_state.intf & rtc_state.inte;
    default:              return 0;
    }
}

void rtc_write32(uint32_t offset, uint32_t val) {
    switch (offset) {
    case RTC_CLKDIV_M1:   rtc_state.clkdiv_m1 = val; break;
    case RTC_SETUP_0:     rtc_state.setup_0 = val; break;
    case RTC_SETUP_1:     rtc_state.setup_1 = val; break;
    case RTC_CTRL:
        /* LOAD is a strobe: writing with LOAD=1 copies SETUP into running time */
        if (val & RTC_CTRL_LOAD) {
            rtc_load_setup();
        }
        /* Store ctrl without LOAD bit (it's a strobe, self-clearing) */
        rtc_state.ctrl = val & ~RTC_CTRL_LOAD;
        break;
    case RTC_IRQ_SETUP_0: rtc_state.irq_setup_0 = val; break;
    case RTC_IRQ_SETUP_1: rtc_state.irq_setup_1 = val; break;
    case RTC_INTE:        rtc_state.inte = val & 1; break;
    case RTC_INTF:        rtc_state.intf = val & 1; break;
    default: break;
    }
}

/* Tick RTC based on elapsed microseconds from timer */
void rtc_tick(uint32_t elapsed_us) {
    if (!(rtc_state.ctrl & RTC_CTRL_ENABLE)) return;

    rtc_state.tick_acc += elapsed_us;

    /* 1 second = 1,000,000 µs */
    while (rtc_state.tick_acc >= 1000000) {
        rtc_state.tick_acc -= 1000000;
        rtc_advance_second();
    }
}
