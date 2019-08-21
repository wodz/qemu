/* LCD interface - YUV2RGB block */
#ifndef ATJ213X_YUV2RGB_H
#define ATJ213X_YUV2RGB_H

#include "atj213x_pmu.h"

#define TYPE_ATJ213X_YUV2RGB "atj213x-YUV2RGB"
#define ATJ213X_YUV2RGB(obj) \
    OBJECT_CHECK(AtjYUV2RGBState, (obj), TYPE_ATJ213X_YUV2RGB)

/* Iriver e150 screen size */
#define LCD_WIDTH  240
#define LCD_HEIGHT 320

enum {
    YUV2RGB_CTL,
    YUV2RGB_DAT,
    YUV2RGB_CLKCTL,
    YUV2RGB_FRAMECOUNT,
    YUV2RGB_REG_NUM
};

enum {
    LCM_CMD,
    LCM_DATA,
    LCM_RGB,
    LCM_YUV
};

struct AtjYUV2RGBState {
    SysBusDevice parent_obj;
    MemoryRegion regs_region;
    uint32_t regs[YUV2RGB_REG_NUM];

    AtjPMUState *pmu;

    uint32_t mode;
    uint8_t cmd;
    uint16_t col_start;
    uint16_t col_end;
    uint16_t row_start;
    uint16_t row_end;

    uint16_t col;
    uint16_t row;

    QemuConsole *con;
    MemoryRegion vram;
    uint32_t *vramptr;
};
typedef struct AtjYUV2RGBState AtjYUV2RGBState;

DeviceState *YUV2RGB_create(hwaddr base, AtjPMUState *pmu);
#endif
