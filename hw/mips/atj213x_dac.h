/* DAC Block */
#ifndef ATJ213X_DAC_H
#define ATJ213X_DAC_H

#define TYPE_ATJ213X_DAC "atj213x-DAC"
#define ATJ213X_DAC(obj) \
    OBJECT_CHECK(AtjDACState, (obj), TYPE_ATJ213X_DAC)

/* 1024 samples, two channels, S32 format */
#define ATJ_DAC_BUFFER_SIZE (1024 * 2)

enum {
    DAC_CTL,
    DAC_FIFOCTL,
    DAC_DAT,
    DAC_DEBUG,
    DAC_ANALOG,
    DAC_REG_NUM
};

struct AtjDACState {
    SysBusDevice parent_obj;
    MemoryRegion regs_region;
    uint32_t regs[DAC_REG_NUM];

    QEMUSoundCard card;
    SWVoiceOut *voice;
    qemu_irq dma_rdy;

    uint32_t silence[ATJ_DAC_BUFFER_SIZE/4];
    uint32_t buffer[ATJ_DAC_BUFFER_SIZE];
    uint32_t buffer_level;
};
typedef struct AtjDACState AtjDACState;

DeviceState *DAC_create(hwaddr base);

#endif
