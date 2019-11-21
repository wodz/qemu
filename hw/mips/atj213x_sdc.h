/* SD Controller Unit - SDC Block */

#ifndef ATJ213X_SDC_H
#define ATJ213X_SDC_H
#include "hw/sd/sd.h"

#include "atj213x_intc.h"

#define TYPE_ATJ213X_SDC "atj213x-SDC"
#define ATJ213X_SDC(obj) \
    OBJECT_CHECK(AtjSDCState, (obj), TYPE_ATJ213X_SDC)

enum {
    SDC_CTL,
    SDC_CMDRSP,
    SDC_RW,
    SDC_FIFOCTL,
    SDC_CMD,
    SDC_ARG,
    SDC_CRC7,
    SDC_RSPBUF0,
    SDC_RSPBUF1,
    SDC_RSPBUF2,
    SDC_RSPBUF3,
    SDC_RSPBUF4,
    SDC_DAT,
    SDC_CLK,
    SDC_BYTECNT,
    SDC_REG_NUM
};

struct AtjSDCState {
    SysBusDevice parent_obj;
    MemoryRegion regs_region;
    SDState *card;
    qemu_irq sdc_irq;
    uint32_t regs[SDC_REG_NUM];
};
typedef struct AtjSDCState AtjSDCState;

//DeviceState *SDC_create(hwaddr base, AtjINTCState *intc);

#endif
