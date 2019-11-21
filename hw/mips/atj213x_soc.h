#ifndef ATJ213X_SOC_H
#define ATJ213X_SOC_H
#include "cpu.h"
#include "qemu/osdep.h"

#include "atj213x_cmu.h"
#include "atj213x_pmu.h"
#include "atj213x_intc.h"
#include "atj213x_rtc.h"
#include "atj213x_sdc.h"
#include "atj213x_gpio.h"
#include "atj213x_dmac.h"
#include "atj213x_dac.h"
#include "atj213x_yuv2rgb.h"

#define TYPE_ATJ213X_SOC "atj213x-soc"
#define ATJ213X_SOC(obj) OBJECT_CHECK(Atj213xSocState, (obj), TYPE_ATJ213X_SOC)

struct Atj213xSocState
{
    /*< private >*/
    DeviceState parent_obj;

    /*< public >*/
    MIPSCPU cpu;
    MemoryRegion iram;

    AtjPMUState pmu;
    AtjINTCState intc;
    AtjCMUState cmu;
    AtjRTCState rtc;
    AtjSDCState sdc;
    AtjYUV2RGBState yuv2rgb;
    AtjGPIOState gpio;
    AtjDMACState dmac;
    AtjDACState dac;
};
typedef struct Atj213xSocState Atj213xSocState;

#endif
