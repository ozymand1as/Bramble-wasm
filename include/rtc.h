#ifndef RTC_H
#define RTC_H

#include <stdint.h>

/* RP2040 RTC Base Address */
#define RTC_BASE        0x4005C000
#define RTC_BLOCK_SIZE  0x30

/* RTC Register Offsets */
#define RTC_CLKDIV_M1   0x00  /* Clock divider minus 1 */
#define RTC_SETUP_0     0x04  /* Year, month, day */
#define RTC_SETUP_1     0x08  /* Day-of-week, hour, minute, second */
#define RTC_CTRL        0x0C  /* Control: RTC_ENABLE, RTC_ACTIVE, LOAD */
#define RTC_IRQ_SETUP_0 0x10  /* Alarm match enable + date fields */
#define RTC_IRQ_SETUP_1 0x14  /* Alarm match enable + time fields */
#define RTC_RTC_1       0x18  /* Read: year, month, day */
#define RTC_RTC_0       0x1C  /* Read: day-of-week, hour, minute, second */
#define RTC_INTR        0x20  /* Raw interrupt status */
#define RTC_INTE        0x24  /* Interrupt enable */
#define RTC_INTF        0x28  /* Interrupt force */
#define RTC_INTS        0x2C  /* Interrupt status (after mask) */

/* RTC CTRL bits */
#define RTC_CTRL_ENABLE     (1u << 0)
#define RTC_CTRL_ACTIVE     (1u << 1)
#define RTC_CTRL_LOAD       (1u << 4)

/* RTC state */
typedef struct {
    uint32_t clkdiv_m1;
    uint32_t setup_0;
    uint32_t setup_1;
    uint32_t ctrl;
    uint32_t irq_setup_0;
    uint32_t irq_setup_1;
    uint32_t inte;
    uint32_t intf;

    /* Running time fields (loaded from SETUP on CTRL.LOAD) */
    int year;       /* 0-4095 */
    int month;      /* 1-12 */
    int day;        /* 1-28/29/30/31 */
    int dotw;       /* 0-6, 0=Sunday */
    int hour;       /* 0-23 */
    int min;        /* 0-59 */
    int sec;        /* 0-59 */

    /* Tick accumulator: counts timer µs, ticks seconds */
    uint64_t tick_acc;
} rtc_state_t;

extern rtc_state_t rtc_state;

void     rtc_init(void);
int      rtc_match(uint32_t addr);
uint32_t rtc_read32(uint32_t offset);
void     rtc_write32(uint32_t offset, uint32_t val);

/* Called from main loop to tick RTC based on elapsed microseconds */
void     rtc_tick(uint32_t elapsed_us);

#endif /* RTC_H */
