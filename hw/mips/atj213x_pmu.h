/* Power Management Unit - PMU Block */
#ifndef ATJ213X_PMU_H
#define ATJ213X_PMU_H

#define TYPE_ATJ213X_PMU "atj213x-PMU"
#define ATJ213X_PMU(obj) \
    OBJECT_CHECK(AtjPMUState, (obj), TYPE_ATJ213X_PMU)
enum {
    PMU_CTL,
    PMU_LRADC,
    PMU_CHG,
    PMU_REG_NUM
};

enum lradc_ch_e {
    LRADC_CH_KEY,
    LRADC_CH_BAT,
    LRADC_CH_TEMP
};
typedef enum lradc_ch_e lradc_ch_e;

struct AtjPMUState {
    SysBusDevice parent_obj;
    MemoryRegion regs_region;
    uint32_t regs[PMU_REG_NUM];

    void (*lradc_set)(void *opaque, lradc_ch_e channel, int value);
    int (*lradc_get)(void *opaque, lradc_ch_e channel);
    int (*backlight_get)(void *opaque);
};
typedef struct AtjPMUState AtjPMUState;

DeviceState *PMU_create(hwaddr base);

#endif
