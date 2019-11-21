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
    CMU_DEVCLKEN_YUV = (1<<1),
    CMU_DEVCLKEN_RMOC = (1<<2),
    CMU_DEVCLKEN_DSPM = (1<<4),
    CMU_DEVCLKEN_SDRC = (1<<5),
    CMU_DEVCLKEN_SDRM = (1<<6),
    CMU_DEVCLKEN_PCNT = (1<<7),
    CMU_DEVCLKEN_DMAC = (1<<8),
    CMU_DEVCLKEN_NAND = (1<<9),
    CMU_DEVCLKEN_SD = (1<<11),
    CMU_DEVCLKEN_USBC = (1<<13),
    CMU_DEVCLKEN_MHA = (1<<14),
    CMU_DEVCLKEN_MCA = (1<<15),
    CMU_DEVCLKEN_DSPC = (1<<16),
    CMU_DEVCLKEN_DAC = (1<<17),
    CMU_DEVCLKEN_ADC = (1<<18),
    CMU_DEVCLKEN_UART = (1<<22),
    CMU_DEVCLKEN_IIC = (1<<23),
    CMU_DEVCLKEN_KEY = (1<<25),
    CMU_DEVCLKEN_GPIO = (1<<26)
} AtjClkEn;

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
    bool (*clk_enabled) (void *opaque, AtjClkEn dev);
};
typedef struct AtjCMUState AtjCMUState;

DeviceState *CMU_create(hwaddr base);

#endif
