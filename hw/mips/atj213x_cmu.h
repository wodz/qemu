/* Clock Management Unit - CMU Block */
#ifndef ATJ213X_CMU_H
#define ATJ213X_CMU_H

#define TYPE_ATJ213X_CMU "atj213x-CMU"
#define ATJ213X_CMU(obj) \
    OBJECT_CHECK(AtjCMUState, (obj), TYPE_ATJ213X_CMU)

enum {
    CMU_COREPLL,
    CMU_DSPPLL,
    CMU_AUDIOPLL,
    CMU_BUSCLK,
    CMU_SDRCLK,
    CMU_NANDCLK = 0x06,
    CMU_SDCLK,
    CMU_MHACLK,
    CMU_UART2CLK = 0x0b,
    CMU_DMACLK,
    CMU_FMCLK,
    CMU_MCACLK,
    CMU_DEVCLKEN = 0x20,
    CMU_DEVRST,
    CMU_REG_NUM
};

typedef enum {
    CLK_HOSC,
    CLK_LOSC,
    CLK_COREPLL,
    CLK_CCLK,
    CLK_SCLK,
    CLK_PCLK
} AtjClk;

struct AtjCMUState {
    SysBusDevice parent_obj;
    MemoryRegion regs_region;
    uint32_t regs[CMU_REG_NUM];

    uint32_t (*get_clock_frequency) (void *opaque, AtjClk clk);
};
typedef struct AtjCMUState AtjCMUState;

DeviceState *CMU_create(hwaddr base);

#endif
