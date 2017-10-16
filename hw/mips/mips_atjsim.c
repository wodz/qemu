/*
 * QEMU/mipssim emulation
 *
 * Emulates a very simple machine model similar to the one used by the
 * proprietary MIPS emulator.
 * 
 * Copyright (c) 2007 Thiemo Seufer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "cpu.h"
#include "hw/hw.h"
#include "hw/mips/mips.h"
#include "hw/mips/cpudevs.h"
#include "hw/ptimer.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/mips/bios.h"
#include "hw/loader.h"
#include "elf.h"
#include "hw/sysbus.h"
#include "exec/address-spaces.h"
#include "qemu/error-report.h"
#include "sysemu/qtest.h"
#include "exec/log.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"

static struct _loaderparams {
    int ram_size;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
} loaderparams;

typedef struct ResetData {
    MIPSCPU *cpu;
    uint64_t vector;
} ResetData;

static int64_t load_kernel(void)
{
    int64_t entry;

    int kernel_size = load_image_targphys(loaderparams.kernel_filename, 0, 8*1024*1024);

    if (kernel_size <= 0)
        exit(1);

    entry = (int32_t)0x80000000;
    return entry;
}

static void main_cpu_reset(void *opaque)
{
    ResetData *s = (ResetData *)opaque;
    CPUMIPSState *env = &s->cpu->env;

    cpu_reset(CPU(s->cpu));
    env->active_tc.PC = s->vector & ~(target_ulong)1;
    if (s->vector & 1) {
        env->hflags |= MIPS_HFLAG_M16;
    }
}

/* CMU Block */
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

static uint32_t CMU_get_clock_frequency(void *opaque, AtjClk clk)
{
    AtjCMUState *s = ATJ213X_CMU(opaque);
    /* bit7 - pll enable
     * bit10 - pll bypass
     */
    uint32_t corepllfreq = (s->regs[CMU_COREPLL] & (1<<7)) ?
                               (s->regs[CMU_COREPLL] & (1<<10)) ?
                                   24000000 : 
                                   (s->regs[CMU_COREPLL] & 0x3f) * 6000000 :
                               0;

    /* coreclk source 24M, 32.768k or pll */
    uint32_t coreclks = (s->regs[CMU_BUSCLK] >> 6) & 0x03;
    uint32_t coreclk = ((coreclks == 2) ?
                           corepllfreq : ((coreclks == 1) ?
                               24000000 : 32768));

    /* clock dividers */
    uint32_t cclkdiv = ((s->regs[CMU_BUSCLK] >> 2) & 0x03) + 1;
    uint32_t sclkdiv = ((s->regs[CMU_BUSCLK] >> 4) & 0x03) + 1;
    uint32_t pclkdiv = ((s->regs[CMU_BUSCLK] >> 8) & 0x0f);
    pclkdiv = pclkdiv ? (pclkdiv + 1) : 2;

    switch (clk)
    {
        case CLK_HOSC:
            return 24000000;

        case CLK_LOSC:
            return 32768;

        case CLK_COREPLL:
            return corepllfreq;

        case CLK_CCLK:
            return coreclk/cclkdiv;

        case CLK_SCLK:
            return (coreclk/cclkdiv)/sclkdiv;

        case CLK_PCLK:
            return ((coreclk/cclkdiv)/sclkdiv)/pclkdiv;

        default:
            return 0;
    }
}

static uint64_t CMU_read(void *opaque, hwaddr  addr, unsigned size)
{
    AtjCMUState *s = opaque;

    addr >>= 2;
    switch (addr)
    {
        default:
            qemu_log("%s() addr: 0x" TARGET_FMT_plx "\n", __func__, addr<<2);
            break;
    }

    return s->regs[addr];
}

static void CMU_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    AtjCMUState *s = opaque;

    addr >>= 2;
    switch (addr)
    {
        default:
            s->regs[addr] = value;
            qemu_log("%s() addr: 0x" TARGET_FMT_plx " value: 0x%lx\n", __func__, addr<<2, value);
            break;
    }
};

static const MemoryRegionOps CMU_mmio_ops = {
    .read = CMU_read,
    .write = CMU_write,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void CMU_reset(DeviceState *d)
{
    AtjCMUState *s = ATJ213X_CMU(d);

    for (int i=0; i<CMU_REG_NUM; i++)
    {
        s->regs[i] = 0;
    }
}

static void CMU_init(Object *obj)
{
    AtjCMUState *s = ATJ213X_CMU(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->regs_region, obj, &CMU_mmio_ops, s, TYPE_ATJ213X_CMU, CMU_REG_NUM * 4);
    sysbus_init_mmio(dev, &s->regs_region);
}

static void CMU_realize(DeviceState *dev, Error **errp)
{
    AtjCMUState *s = ATJ213X_CMU(dev);
    s->get_clock_frequency = CMU_get_clock_frequency;
}

static const VMStateDescription vmstate_CMU = {
    .name = TYPE_ATJ213X_CMU,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AtjCMUState, CMU_REG_NUM),
        VMSTATE_END_OF_LIST()
    }
};

static void CMU_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = CMU_realize;
    dc->reset = CMU_reset;
    dc->vmsd = &vmstate_CMU;
    //dc->props = milkymist_sysctl_properties;
}

static const TypeInfo CMU_info = {
    .name          = TYPE_ATJ213X_CMU,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AtjCMUState),
    .instance_init = CMU_init,
    .class_init    = CMU_class_init,
};

static void CMU_register_types(void)
{
    type_register_static(&CMU_info);
}

type_init(CMU_register_types)


static DeviceState *CMU_create(hwaddr base)
{
    DeviceState *dev;
    dev = qdev_create(NULL, TYPE_ATJ213X_CMU);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    return dev;
}

/* PMU Block */
#define TYPE_ATJ213X_PMU "atj213x-PMU"
#define ATJ213X_PMU(obj) \
    OBJECT_CHECK(AtjPMUState, (obj), TYPE_ATJ213X_PMU)
enum {
    PMU_CTL,
    PMU_LRADC,
    PMU_CHG,
    PMU_REG_NUM
};

struct AtjPMUState {
    SysBusDevice parent_obj;
    MemoryRegion regs_region;
    uint32_t regs[PMU_REG_NUM];
};
typedef struct AtjPMUState AtjPMUState;

static uint64_t PMU_read(void *opaque, hwaddr  addr, unsigned size)
{
    AtjPMUState *s = opaque;

    addr >>= 2;
    switch (addr)
    {
        default:
            qemu_log("%s() addr: 0x" TARGET_FMT_plx "\n", __func__, addr<<2);
            break;
    }

    return s->regs[addr];
}
uint8_t backlight_val = 0x1f;
static void PMU_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    AtjPMUState *s = opaque;
    qemu_log("%s() addr: 0x" TARGET_FMT_plx " value: 0x%lx\n", __func__, addr, value);

    addr >>= 2;
    switch (addr)
    {
        case PMU_CHG:
            backlight_val = (value >> 8) & 0x1f;
            qemu_log("Backlight: %ld\n", (value >> 8) & 0x1f);
        default:
            break;
    }

    s->regs[addr] = value;
};

static const MemoryRegionOps PMU_mmio_ops = {
    .read = PMU_read,
    .write = PMU_write,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void PMU_reset(DeviceState *d)
{
    AtjPMUState *s = ATJ213X_PMU(d);

    for (int i=0; i<PMU_REG_NUM; i++)
    {
        s->regs[i] = 1;
    }
}

static void PMU_init(Object *obj)
{
    AtjPMUState *s = ATJ213X_PMU(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->regs_region, obj, &PMU_mmio_ops, s, TYPE_ATJ213X_PMU, PMU_REG_NUM * 4);
    sysbus_init_mmio(dev, &s->regs_region);
}

static void PMU_realize(DeviceState *dev, Error **errp)
{
}

static const VMStateDescription vmstate_PMU = {
    .name = TYPE_ATJ213X_PMU,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AtjPMUState, PMU_REG_NUM),
        VMSTATE_END_OF_LIST()
    }
};

static void PMU_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = PMU_realize;
    dc->reset = PMU_reset;
    dc->vmsd = &vmstate_PMU;
    //dc->props = milkymist_sysctl_properties;
}

static const TypeInfo PMU_info = {
    .name          = TYPE_ATJ213X_PMU,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AtjPMUState),
    .instance_init = PMU_init,
    .class_init    = PMU_class_init,
};

static void PMU_register_types(void)
{
    type_register_static(&PMU_info);
}

type_init(PMU_register_types)


static DeviceState *PMU_create(hwaddr base)
{
    DeviceState *dev;
    dev = qdev_create(NULL, TYPE_ATJ213X_PMU);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    return dev;
}

/* INTC */
#define TYPE_ATJ213X_INTC "atj213x-INTC"
#define ATJ213X_INTC(obj) OBJECT_CHECK(AtjINTCState, (obj), TYPE_ATJ213X_INTC)
enum {
    INTC_PD,
    INTC_MSK,
    INTC_CFG0,
    INTC_CFG1,
    INTC_CFG2,
    INTC_EXTCTL,
    INTC_REG_NUM
};

enum {
    IRQ_MCA,
    IRQ_Reserved0,
    IRQ_SD,
    IRQ_MHA,
    IRQ_USB,
    IRQ_DSP,
    IRQ_Reserved1,
    IRQ_PCNT,
    IRQ_WD,
    IRQ_T1,
    IRQ_T0,
    IRQ_RTC,
    IRQ_DMA,
    IRQ_KEY,
    IRQ_EXT,
    IRQ_Reserved2,
    IRQ_Reserved3,
    IRQ_IIC2,
    IRQ_IIC1,
    IRQ_Reserved4,
    IRQ_Reserved5,
    IRQ_ADC,
    IRQ_DAC,
    IRQ_Reserved6,
    IRQ_NAND,
    IRQ_Reserved7,
    INTC_IRQS_NUM
};

/* Out lines from interrupt controller are 0-4 */
#define INTC_OUT_IRQS_NUM 5
struct AtjINTCState {
    SysBusDevice parent_obj;
    MemoryRegion regs_region;

    /* controller register file */
    uint32_t regs[INTC_REG_NUM];

    /* controller input irq lines
     * are allocated by qdev_init_gpio_in()
     */

    /* controller output irq lines */
    qemu_irq irq_out[INTC_OUT_IRQS_NUM];
};
typedef struct AtjINTCState AtjINTCState;

static void INTC_update_irq(AtjINTCState *s)
{
    uint32_t mask = 1;
    for (int i=0; i<INTC_IRQS_NUM; mask <<= 1, i++)
    {
        if (s->regs[INTC_MSK] & mask)
        {
            int level = (s->regs[INTC_PD] & mask) ? 1 : 0;
            int irq_out_no = (s->regs[INTC_CFG2] & mask) ? (1 << 2) : 0 |
                             (s->regs[INTC_CFG1] & mask) ? (1 << 1) : 0 |
                             (s->regs[INTC_CFG0] & mask) ? (1 << 0) : 0;

            irq_out_no = (irq_out_no > 4) ? 4 : irq_out_no;

            /* update mips irq line */
            qemu_set_irq(s->irq_out[irq_out_no], level);
        }
    }
}

static void INTC_set_irq(void *opaque, int n_IRQ, int level)
{
    AtjINTCState *s = ATJ213X_INTC(opaque);
    uint32_t mask = (1 << n_IRQ);

    /* set pending bit */
    if (level)
    {
        s->regs[INTC_PD] |= mask;
    }
    else
    {
        s->regs[INTC_PD] &= ~mask;
    }

    /* if unmasked update irq state */
    if (s->regs[INTC_MSK] & mask)
    {
        INTC_update_irq(s);
    }
}

static uint64_t INTC_read(void *opaque, hwaddr addr,
                          unsigned size)
{
    AtjINTCState *s = opaque;

    addr >>= 2;
    switch (addr)
    {
        default:
            qemu_log("%s() addr: 0x" TARGET_FMT_plx "\n", __func__, addr << 2);
            return s->regs[addr];
    }
}

static void INTC_write(void *opaque, hwaddr addr, uint64_t value,
                       unsigned size)
{
    AtjINTCState *s = opaque;

    addr >>= 2;
    switch (addr)
    {
        case INTC_PD:
            /* read only */
            qemu_log("%s() READONLY addr: 0x" TARGET_FMT_plx " value: 0x"
                     TARGET_FMT_plx "\n", __func__, addr << 2, value);
//            return;

        default:
            s->regs[addr] = value;
            INTC_update_irq(s);
            qemu_log("%s() addr: 0x" TARGET_FMT_plx " value: 0x" TARGET_FMT_plx
                     "\n", __func__, addr << 2, value);
    }
}

static const MemoryRegionOps INTC_mmio_ops = {
    .read = INTC_read,
    .write = INTC_write,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void INTC_reset(void *opaque)
{
    AtjINTCState *s = ATJ213X_INTC(opaque);

    for (int i=0; i<INTC_REG_NUM; i++)
    {
        s->regs[i] = 0;
    }
}

static void INTC_init(Object *obj)
{
    AtjINTCState *s = ATJ213X_INTC(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->regs_region, obj, &INTC_mmio_ops, s,
                          TYPE_ATJ213X_INTC, INTC_REG_NUM * 4);
    sysbus_init_mmio(dev, &s->regs_region);

    qdev_init_gpio_out(DEVICE(obj), s->irq_out, INTC_OUT_IRQS_NUM);

    qemu_register_reset(INTC_reset, s);
}

static void INTC_realize(DeviceState *dev, Error **errp)
{
    qdev_init_gpio_in(dev, INTC_set_irq, INTC_IRQS_NUM);
}

static const VMStateDescription INTC_vmstate = {
    .name = TYPE_ATJ213X_INTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AtjINTCState, INTC_REG_NUM),
        VMSTATE_END_OF_LIST()
    }
};

static void INTC_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = INTC_realize;
    dc->vmsd = &INTC_vmstate;
}

static const TypeInfo INTC_info = {
    .name          = TYPE_ATJ213X_INTC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AtjINTCState),
    .instance_init = INTC_init,
    .class_init    = INTC_class_init,
};

static void INTC_register_types(void)
{
    type_register_static(&INTC_info);
}
type_init(INTC_register_types)

static inline DeviceState *INTC_create(hwaddr base, MIPSCPU *cpu)
{
    DeviceState *dev = qdev_create(NULL, TYPE_ATJ213X_INTC);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);

    CPUMIPSState *env = &cpu->env;

    // void qdev_connect_gpio_out(DeviceState *dev, int n, qemu_irq pin);
    qdev_connect_gpio_out(dev, 0, env->irq[2]);
    qdev_connect_gpio_out(dev, 1, env->irq[3]);
    qdev_connect_gpio_out(dev, 2, env->irq[4]);
    qdev_connect_gpio_out(dev, 3, env->irq[5]);
    qdev_connect_gpio_out(dev, 4, env->irq[6]);

    return dev;
}

/* RTC Block */
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
    uint32_t     freq_hz;
    uint32_t     regs[RTC_REG_NUM];
};
typedef struct AtjRTCState AtjRTCState;

static void tmr_update_cnt(AtjRTCState *s, int n)
{
    uint32_t cnt = ptimer_get_count(s->ptimer[n]);
    s->regs[n ? RTC_T1 : RTC_T0] = cnt;
}

static void tmr_update_irq(void *opaque)
{
    AtjRTCState *s = ATJ213X_RTC(opaque);

    if (s->regs[RTC_T0CTL] & (1<<1))
    {
        if ((s->regs[RTC_T0CTL] & 1))
        {
            /* T0 interrupt enabled and pending bit set */
            qemu_irq_raise(s->tmr_irq[0]);
        }
        else
        {
            qemu_irq_lower(s->tmr_irq[0]);
        }
    }

    if (s->regs[RTC_T1CTL] & (1<<1))
    {
        if ((s->regs[RTC_T1CTL] & 1))
        {
            /* T1 interrupt enabled and pending bit set */
            qemu_irq_raise(s->tmr_irq[1]);
        }
        else
        {
            qemu_irq_lower(s->tmr_irq[1]);
        }
    }
}

static void T0_zero_cb(void *opaque)
{
    AtjRTCState *s = ATJ213X_RTC(opaque);
    s->regs[RTC_T0CTL] |= 1; /* ZIPD bit */

    tmr_update_irq(opaque);
}

static void T1_zero_cb(void *opaque)
{
    AtjRTCState *s = ATJ213X_RTC(opaque);
    s->regs[RTC_T1CTL] |= 1; /* ZIPD bit */

    tmr_update_irq(opaque);
}

static uint64_t RTC_read(void *opaque, hwaddr  addr, unsigned size)
{
    AtjRTCState *s = opaque;

    addr >>= 2;
    switch (addr)
    {
        case RTC_T0:
        case RTC_T1:
            tmr_update_cnt(s, addr > RTC_T0 ? 1 : 0);
            break;

        default:
            break;
    }

    qemu_log("%s() addr: 0x" TARGET_FMT_plx "\n", __func__, addr<<2);
    return s->regs[addr];
}

static void RTC_tmr_run(AtjRTCState *s, int n, int oneshot)
{
    uint32_t tmr_freq = s->cmu->get_clock_frequency(s->cmu, CLK_PCLK);
    ptimer_set_freq(s->ptimer[n], tmr_freq);
    ptimer_run(s->ptimer[n], oneshot);
}

static void RTC_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    AtjRTCState *s = opaque;

    qemu_log("%s() addr: 0x" TARGET_FMT_plx " value: 0x%lx\n", __func__, addr, value);

    addr >>= 2;
    switch (addr)
    {
        case RTC_T0CTL:
        case RTC_T1CTL:
            if (value & 1)
            {
               /* writing ZIPD bit clears irq pending flag */
               s->regs[addr] &= ~1;
               value &= ~1;
            }

            /* timer enable bit */
            if (value & (1<<5))
            {
                /* reload bit */
                int oneshot = (value & (1<<2)) ? 0 : 1;
                RTC_tmr_run(s, (addr > RTC_T0CTL) ? 1 : 0, oneshot);
            }
            tmr_update_irq(s);
            break;

        case RTC_T0:
            ptimer_set_limit(s->ptimer[0], value, 1); // initial countdown value
            break;

        case RTC_T1:
            ptimer_set_limit(s->ptimer[1], value, 1); // initial countdown value
            break;

        default:
            break;
    }

    s->regs[addr] = value;
};

static const MemoryRegionOps RTC_mmio_ops = {
    .read = RTC_read,
    .write = RTC_write,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void RTC_reset(DeviceState *d)
{
    AtjRTCState *s = ATJ213X_RTC(d);

    for (int i=0; i<RTC_REG_NUM; i++)
    {
        s->regs[i] = 0;
    }

    ptimer_stop(s->ptimer[0]);
    ptimer_stop(s->ptimer[1]);
    ptimer_set_limit(s->ptimer[0], 0xffffff, 1);
    ptimer_set_limit(s->ptimer[1], 0xffffff, 1);
}

static void RTC_init(Object *obj)
{
    AtjRTCState *s = ATJ213X_RTC(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->regs_region, obj, &RTC_mmio_ops, s, TYPE_ATJ213X_RTC, RTC_REG_NUM * 4);
    sysbus_init_mmio(dev, &s->regs_region);

    qdev_init_gpio_out(DEVICE(obj), s->tmr_irq, 2);
}

static void RTC_realize(DeviceState *dev, Error **errp)
{
    AtjRTCState *s = ATJ213X_RTC(dev);
    s->bh[0] = qemu_bh_new(T0_zero_cb, s);
    s->bh[1] = qemu_bh_new(T1_zero_cb, s);

    s->ptimer[0] = ptimer_init(s->bh[0], PTIMER_POLICY_DEFAULT);
    s->ptimer[1] = ptimer_init(s->bh[1], PTIMER_POLICY_DEFAULT);

}

static const VMStateDescription RTC_vmstate = {
    .name = TYPE_ATJ213X_INTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AtjRTCState, RTC_REG_NUM),
        VMSTATE_END_OF_LIST()
    }
};

static void RTC_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = RTC_realize;
    dc->reset = RTC_reset;
    dc->vmsd = &RTC_vmstate;
    //dc->props = milkymist_sysctl_properties;
}

static const TypeInfo RTC_info = {
    .name          = TYPE_ATJ213X_RTC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AtjRTCState),
    .instance_init = RTC_init,
    .class_init    = RTC_class_init,
};

static void RTC_register_types(void)
{
    type_register_static(&RTC_info);
}
type_init(RTC_register_types)

static DeviceState *RTC_create(hwaddr base, AtjCMUState *cmu, AtjINTCState *intc)
{
    DeviceState *dev;
    dev = qdev_create(NULL, TYPE_ATJ213X_RTC);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);

    AtjRTCState *s = (AtjRTCState *)dev;
    s->cmu = cmu;

    qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in(DEVICE(intc), IRQ_T0));
    qdev_connect_gpio_out(dev, 1, qdev_get_gpio_in(DEVICE(intc), IRQ_T1));

    return dev;
}

/* YUV2RGB block */
#define TYPE_ATJ213X_YUV2RGB "atj213x-YUV2RGB"
#define ATJ213X_YUV2RGB(obj) \
    OBJECT_CHECK(AtjYUV2RGBState, (obj), TYPE_ATJ213X_YUV2RGB)

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

static uint64_t YUV2RGB_read(void *opaque, hwaddr  addr, unsigned size)
{
    AtjYUV2RGBState *s = opaque;
    qemu_log("%s() addr: 0x" TARGET_FMT_plx "\n", __func__, addr);

    addr >>= 2;
    switch (addr)
    {
        case YUV2RGB_CTL:
            /* pretend fifo is always empty */
            return s->regs[addr] | 0x04;

        default:
            break;
    }

    return s->regs[addr];
}

static void *vramptr(void *opaque, uint16_t row, uint16_t col)
{
    AtjYUV2RGBState *s = opaque;
    void *vram = memory_region_get_ram_ptr(&s->vram);

    return vram + 2 * ((row * LCD_WIDTH) + col);
}

static void vram_write_windowed(void *opaque, uint32_t value)
{
    AtjYUV2RGBState *s = opaque;
    uint32_t *dst = vramptr(opaque, s->row, s->col);
    *dst = value;

    s->col += 2;
    if (s->col > s->col_end)
    {
        s->col = s->col_start;
        s->row++;
        if (s->row > s->row_end)
        {
            s->row = s->row_start;
        }
    }
}

static void YUV2RGB_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    AtjYUV2RGBState *s = opaque;
    qemu_log("%s() addr: 0x" TARGET_FMT_plx " value: 0x%lx\n", __func__, addr, value);
    addr >>= 2;
    switch (addr)
    {
        case YUV2RGB_CTL:
            s->mode = (value >> 16) & 0x03;
            break;

        case YUV2RGB_DAT:
            switch (s->mode)
            {
                case LCM_CMD:
                    s->cmd = value;

                    if (s->cmd == 0x22)
                    {
                        /* set vram pointer at the beginning of window */
                        s->col = s->col_start;
                        s->row = s->row_start;
                    }
                    break;

                case LCM_DATA:
                    switch (s->cmd)
                    {
                        case 0x02:
                            s->col_start = (s->col_start & 0x00ff) | ((uint8_t)value)<<8;
                            break;

                        case 0x03:
                            s->col_start = (s->col_start & 0xff00) | (uint8_t)value;
                            break;

                        case 0x04:
                            s->col_end = (s->col_end & 0x00ff) | ((uint8_t)value)<<8;
                            break;

                        case 0x05:
                            s->col_end = (s->col_end & 0xff00) | (uint8_t)value;
                            break;

                        case 0x06:
                            s->row_start = (s->row_start & 0x00ff) | ((uint8_t)value)<<8;
                            break;

                        case 0x07:
                            s->row_start = (s->row_start & 0xff00) | (uint8_t)value;
                            break;

                        case 0x08:
                            s->row_end = (s->row_end & 0x00ff) | ((uint8_t)value)<<8;
                            break;

                        case 0x09:
                            s->row_end = (s->row_end & 0xff00) | (uint8_t)value;
                            break;
                    }
                    break;

                case LCM_RGB:
                    if (s->cmd == 0x22)
                    {
                        vram_write_windowed(opaque, value);
                    }
                    break;

                case LCM_YUV:
                    break;
            }

        default:
            break;
    }

    s->regs[addr] = value;
};

static const MemoryRegionOps YUV2RGB_mmio_ops = {
    .read = YUV2RGB_read,
    .write = YUV2RGB_write,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void YUV2RGB_reset(DeviceState *d)
{
    AtjYUV2RGBState *s = ATJ213X_YUV2RGB(d);

    for (int i=0; i<YUV2RGB_REG_NUM; i++)
    {
        s->regs[i] = 0;
    }

    s->col_start = 0;
    s->col_end = LCD_WIDTH - 1;
    s->row_start = 0;
    s->row_end = LCD_HEIGHT - 1;
}

static void yuv2rgb_invalidate_display(void *opaque)
{
}


static void yuv2rgb_update_display(void *opaque)
{
    AtjYUV2RGBState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    void *src = memory_region_get_ram_ptr(&s->vram);
    void *dst = surface_data(surface);
    unsigned int r, g, b;

    for (int i=0; i<LCD_WIDTH*LCD_HEIGHT; i++)
    {

        r = backlight_val * ((*(uint16_t *)src & 0xf800) >> 8)/31;
        g = backlight_val * ((*(uint16_t *)src & 0x07e0) >> 3)/31;
        b = backlight_val * ((*(uint16_t *)src & 0x001f) << 3)/31;
        src += 2;

        switch (surface_bits_per_pixel(surface))
        {
            case 8:
                *(uint8_t *)dst = rgb_to_pixel8(r, g, b);
                dst += 1;
                break;

            case 15:
                *(uint16_t *)dst = rgb_to_pixel15(r, g, b);
                dst += 2;
                break;

            case 16:
                *(uint16_t *)dst = rgb_to_pixel16(r, g, b);
                dst += 2;
                break;

            case 24:
                *(uint32_t *)dst = rgb_to_pixel24(r, g, b);
                dst += 3;
                break;

            case 32:
                *(uint32_t *)dst = rgb_to_pixel32(r, g, b);
                dst += 4;
        }
    }

    dpy_gfx_update(s->con, 0, 0, LCD_WIDTH, LCD_HEIGHT);
}

static const GraphicHwOps yuv2rgb_ops = {
    .invalidate  = yuv2rgb_invalidate_display,
    .gfx_update  = yuv2rgb_update_display,
};

static void YUV2RGB_init(Object *obj)
{
    AtjYUV2RGBState *s = ATJ213X_YUV2RGB(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->regs_region, obj, &YUV2RGB_mmio_ops, s, TYPE_ATJ213X_YUV2RGB, YUV2RGB_REG_NUM * 4);
    sysbus_init_mmio(dev, &s->regs_region);

    memory_region_init_ram(&s->vram, NULL, "atj.vram", LCD_WIDTH*LCD_HEIGHT*sizeof(uint16_t), &error_fatal);
    s->vramptr = memory_region_get_ram_ptr(&s->vram);

    memset(s->vramptr, 0, LCD_WIDTH*LCD_HEIGHT*sizeof(uint16_t));
    s->con = graphic_console_init(NULL, 0, &yuv2rgb_ops, s);
    qemu_console_resize(s->con, LCD_WIDTH, LCD_HEIGHT);

    uint16_t *dst = (uint16_t *)s->vramptr;

    /* random pixels as in uninitialized real hardware */
    srand (time(NULL));
    for (int i=0; i<LCD_WIDTH*LCD_HEIGHT; i++)
    {
        *dst++ = rand();
    }

    dpy_gfx_update(s->con, 0, 0, LCD_WIDTH, LCD_HEIGHT);                                
}

static void YUV2RGB_realize(DeviceState *dev, Error **errp)
{
}

static const VMStateDescription YUV2RGB_vmstate = {
    .name = TYPE_ATJ213X_YUV2RGB,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AtjYUV2RGBState, YUV2RGB_REG_NUM),
        VMSTATE_END_OF_LIST()
    }
};

static void YUV2RGB_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = YUV2RGB_realize;
    dc->reset = YUV2RGB_reset;
    dc->vmsd = &YUV2RGB_vmstate;
    //dc->props = milkymist_sysctl_properties;
}

static const TypeInfo YUV2RGB_info = {
    .name          = TYPE_ATJ213X_YUV2RGB,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AtjYUV2RGBState),
    .instance_init = YUV2RGB_init,
    .class_init    = YUV2RGB_class_init,
};

static void YUV2RGB_register_types(void)
{
    type_register_static(&YUV2RGB_info);
}
type_init(YUV2RGB_register_types)

static DeviceState *YUV2RGB_create(hwaddr base)
{
    DeviceState *dev;
    dev = qdev_create(NULL, TYPE_ATJ213X_YUV2RGB);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    return dev;
}

/* PMU Block */
#define TYPE_ATJ213X_GPIO "atj213x-GPIO"
#define ATJ213X_GPIO(obj) \
    OBJECT_CHECK(AtjGPIOState, (obj), TYPE_ATJ213X_GPIO)
enum {
    GPIO_AOUTEN,
    GPIO_AINEN,
    GPIO_ADAT,
    GPIO_BOUTEN,
    GPIO_BINEN,
    GPIO_BDAT,
    GPIO_MFCTL0,
    GPIO_MFCTL1,
    PAD_DRV = 0x22,
    GPIO_REG_NUM
};

struct AtjGPIOState {
    SysBusDevice parent_obj;
    MemoryRegion regs_region;
    uint32_t regs[GPIO_REG_NUM];
};
typedef struct AtjGPIOState AtjGPIOState;

static uint64_t GPIO_read(void *opaque, hwaddr  addr, unsigned size)
{
    AtjGPIOState *s = opaque;

    addr >>= 2;
    switch (addr)
    {
        default:
            qemu_log("%s() addr: 0x" TARGET_FMT_plx "\n", __func__, addr<<2);
            break;
    }

    return s->regs[addr];
}

static void GPIO_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    AtjGPIOState *s = opaque;
    qemu_log("%s() addr: 0x" TARGET_FMT_plx " value: 0x%lx\n", __func__, addr, value);

    addr >>= 2;
    switch (addr)
    {
        default:
            break;
    }

    s->regs[addr] = value;
};

static const MemoryRegionOps GPIO_mmio_ops = {
    .read = GPIO_read,
    .write = GPIO_write,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void GPIO_reset(DeviceState *d)
{
    AtjGPIOState *s = ATJ213X_GPIO(d);

    for (int i=0; i<GPIO_REG_NUM; i++)
    {
        s->regs[i] = 0;
    }
}

static void GPIO_init(Object *obj)
{
    AtjGPIOState *s = ATJ213X_GPIO(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->regs_region, obj, &GPIO_mmio_ops, s, TYPE_ATJ213X_GPIO, GPIO_REG_NUM * 4);
    sysbus_init_mmio(dev, &s->regs_region);
}

static void GPIO_realize(DeviceState *dev, Error **errp)
{
}

static const VMStateDescription vmstate_GPIO = {
    .name = TYPE_ATJ213X_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AtjGPIOState, GPIO_REG_NUM),
        VMSTATE_END_OF_LIST()
    }
};

static void GPIO_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = GPIO_realize;
    dc->reset = GPIO_reset;
    dc->vmsd = &vmstate_GPIO;
    //dc->props = milkymist_sysctl_properties;
}

static const TypeInfo GPIO_info = {
    .name          = TYPE_ATJ213X_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AtjGPIOState),
    .instance_init = GPIO_init,
    .class_init    = GPIO_class_init,
};

static void GPIO_register_types(void)
{
    type_register_static(&GPIO_info);
}

type_init(GPIO_register_types)


static DeviceState *GPIO_create(hwaddr base)
{
    DeviceState *dev;
    dev = qdev_create(NULL, TYPE_ATJ213X_GPIO);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    return dev;
}

static void
mips_atjsim_init(MachineState *machine)
{
    ram_addr_t ram_size = machine->ram_size;
    const char *cpu_model = machine->cpu_model;
    const char *kernel_filename = machine->kernel_filename;
    const char *kernel_cmdline = machine->kernel_cmdline;
    const char *initrd_filename = machine->initrd_filename;
    char *filename;
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *ram = g_new(MemoryRegion, 1);
    MemoryRegion *iram = g_new(MemoryRegion, 1);
    MemoryRegion *bios = g_new(MemoryRegion, 1);
    MIPSCPU *cpu;
    CPUMIPSState *env;
    ResetData *reset_info;
    int bios_size;

    /* Init CPUs. */
    if (cpu_model == NULL) {
        cpu_model = "4KEc";
    }
    cpu = MIPS_CPU(cpu_generic_init(TYPE_MIPS_CPU, cpu_model));
    env = &cpu->env;

    reset_info = g_malloc0(sizeof(ResetData));
    reset_info->cpu = cpu;
    reset_info->vector = env->active_tc.PC;
    qemu_register_reset(main_cpu_reset, reset_info);

    /* Allocate RAM. */
    memory_region_allocate_system_memory(ram, NULL, "mips_atjsim.ram",
                                         ram_size);
    memory_region_init_ram(bios, NULL, "mips_mipssim.bios", BIOS_SIZE,
                           &error_fatal);
    memory_region_add_subregion(address_space_mem, 0, ram);

    memory_region_init_ram(iram, NULL, "mips_atjsim.iram", 96*1024,
                           &error_fatal);
    memory_region_add_subregion(address_space_mem, 0x14040000, iram);

    memory_region_set_readonly(bios, true);


    /* Map the BIOS / boot exception handler. */
    memory_region_add_subregion(address_space_mem, 0x1fc00000LL, bios);
    /* Load a BIOS / boot exception handler image. */
    if (bios_name == NULL)
        bios_name = BIOS_FILENAME;
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    if (filename) {
        bios_size = load_image_targphys(filename, 0x1fc00000LL, BIOS_SIZE);
        g_free(filename);
    } else {
        bios_size = -1;
    }
    if ((bios_size < 0 || bios_size > BIOS_SIZE) &&
        !kernel_filename && !qtest_enabled()) {
        /* Bail out if we have neither a kernel image nor boot vector code. */
        error_report("Could not load MIPS bios '%s', and no "
                     "-kernel argument was specified", bios_name);
        exit(1);
    } else {
        /* We have a boot vector start address. */
        env->active_tc.PC = (target_long)(int32_t)0xbfc00000;
    }

    if (kernel_filename) {
        loaderparams.ram_size = ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        reset_info->vector = load_kernel();
    }

    /* Init CPU internal devices. */
    cpu_mips_irq_init_cpu(cpu);
    cpu_mips_clock_init(cpu);

    /* PMU */
    PMU_create(0x10000000);

    /* INTC */
    AtjINTCState *intc = (AtjINTCState *)INTC_create(0x10020000, cpu);

    /* CMU */
    AtjCMUState *cmu = (AtjCMUState *)CMU_create(0x10010000);

    /* RTC */
    RTC_create(0x10018000, cmu, intc);

    /* YUV2RGB */
    YUV2RGB_create(0x100f0000);

    /* GPIO */
    GPIO_create(0x101c0000);
}

static void mips_atjsim_machine_init(MachineClass *mc)
{
    mc->desc = "MIPS ATJsim platform";
    mc->init = mips_atjsim_init;
}

DEFINE_MACHINE("atjsim", mips_atjsim_machine_init)
