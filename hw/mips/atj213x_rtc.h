/* Timer Unit - RTC Block
 * It has RTC, 2x timers and Watchdog
 */

#ifndef ATJ213X_RTC_H
#define ATJ213X_RTC_H
#include "hw/ptimer.h"

#include "atj213x_cmu.h"
#include "atj213x_intc.h"

#define TYPE_ATJ213X_RTC "atj213x-RTC"
#define ATJ213X_RTC(obj) \
    OBJECT_CHECK(AtjRTCState, (obj), TYPE_ATJ213X_RTC)
enum {
    RTC_CTL,
    RTC_DHMS,
    RTC_YMD,
    RTC_DHMSALM,
    RTC_YMDALM,
    RTC_WDCTL,
    RTC_T0CTL,
    RTC_T0,
    RTC_T1CTL,
    RTC_T1,
    RTC_REG_NUM
};

struct AtjRTCState {
    SysBusDevice parent_obj;
    MemoryRegion regs_region;
    AtjCMUState  *cmu;
    qemu_irq     tmr_irq[2];
    QEMUBH       *bh[2];
    ptimer_state *ptimer[2];
    QEMUTimer    *wdt_timer;
    qemu_irq     wdt_irq;
    uint32_t     freq_hz;
    uint32_t     regs[RTC_REG_NUM];
};
typedef struct AtjRTCState AtjRTCState;

DeviceState *RTC_create(hwaddr base, AtjCMUState *cmu, AtjINTCState *intc);

#endif
