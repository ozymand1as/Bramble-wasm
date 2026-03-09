/*
 * RP2040 RTC Peripheral Stub
 *
 * Minimal emulation: registers are readable/writable but the clock
 * does not actually tick. CTRL.ACTIVE is set when CTRL.ENABLE is set.
 * The setup values are returned as-is from RTC_1/RTC_0 reads.
 */

#include <string.h>
#include "rtc.h"

rtc_state_t rtc_state;

void rtc_init(void) {
    memset(&rtc_state, 0, sizeof(rtc_state));
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
    case RTC_RTC_1:       return rtc_state.setup_0;  /* Return setup values */
    case RTC_RTC_0:       return rtc_state.setup_1;
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
        rtc_state.ctrl = val;
        break;
    case RTC_IRQ_SETUP_0: rtc_state.irq_setup_0 = val; break;
    case RTC_IRQ_SETUP_1: rtc_state.irq_setup_1 = val; break;
    case RTC_INTE:        rtc_state.inte = val & 1; break;
    case RTC_INTF:        rtc_state.intf = val & 1; break;
    default: break;
    }
}
