/* Direct Memory Access Unit - DMAC Block */
#ifndef ATJ213X_DMAC_H
#define ATJ213X_DMAC_H

#include "atj213x_intc.h"

#define TYPE_ATJ213X_DMAC "atj213x-DMAC"
#define ATJ213X_DMAC(obj) \
    OBJECT_CHECK(AtjDMACState, (obj), TYPE_ATJ213X_DMAC)

#define E_DMA_CH(n, offset) \
    DMA##n##_MODE = ((offset) >> 2), \
    DMA##n##_SRC, \
    DMA##n##_DST, \
    DMA##n##_CNT, \
    DMA##n##_REM, \
    DMA##n##_CMD 

#define DMAC_CHANNELS_NUM 8

enum {
    DMA_CTL,
    DMA_IRQEN,
    DMA_IRQPD,

    E_DMA_CH(0, 0x100),
    E_DMA_CH(1, 0x120),
    E_DMA_CH(2, 0x140),
    E_DMA_CH(3, 0x160),
    E_DMA_CH(4, 0x180),
    E_DMA_CH(5, 0x1a0),
    E_DMA_CH(6, 0x1c0),
    E_DMA_CH(7, 0x1e0),

    DMAC_REG_NUM
};

enum {
    DMA_DAC_drq,
    DMA_ADC_drq,
    DMA_NAND_drq,
    DMA_SD_drq,
    DMA_OTG_drq,
    DMA_LCM_drq,
    DMA_drq_NUM
};


typedef struct AtjDMAChannel AtjDMAChannel;
struct AtjDMAdrqState {
    bool asserted;
    AtjDMAChannel *ch;
};
typedef struct AtjDMAdrqState AtjDMAdrqState;

struct AtjDMAchCtrl {
    uint32_t mode;
    uint32_t src;
    uint32_t dst;
    uint32_t cnt;
    uint32_t rem;
    uint32_t cmd;
};
typedef struct AtjDMAchCtrl AtjDMAchCtrl;

struct AtjDMAChannel {
    QEMUTimer *timer;
    AtjDMAdrqState *drq;

    int no;
    AtjDMAchCtrl ctrl[2];
};

struct AtjDMACState {
    SysBusDevice parent_obj;
    MemoryRegion regs_region;

    /* global control */
    uint32_t ctl;
    uint32_t irqen;
    uint32_t irqpd;

    /* channel description */
    AtjDMAChannel ch[DMAC_CHANNELS_NUM];

    /* irq line */
    qemu_irq dmac_irq;

    /* input drq lines */
    AtjDMAdrqState drq[DMA_drq_NUM];
};
typedef struct AtjDMACState AtjDMACState;

DeviceState *DMAC_create(hwaddr base, AtjINTCState *intc);

#endif
