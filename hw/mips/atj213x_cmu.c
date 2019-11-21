#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "trace.h"

#include "atj213x_cmu.h"

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

static bool CMU_clk_enabled(void *opaque, AtjClkEn dev)
{
    AtjCMUState *s = opaque;
    return (s->regs[CMU_DEVCLKEN] & dev);
}

static uint64_t CMU_read(void *opaque, hwaddr  addr, unsigned size)
{
    AtjCMUState *s = opaque;

    addr >>= 2;
    switch (addr)
    {
        default:
            break;
    }

    trace_atj_cmu_read(addr<<2, s->regs[addr]);
    return s->regs[addr];
}

static void CMU_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    AtjCMUState *s = opaque;
    trace_atj_cmu_write(addr, value);

    addr >>= 2;
    switch (addr)
    {
        default:
            s->regs[addr] = value;
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
    s->clk_enabled = CMU_clk_enabled;
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
    dc->user_creatable = false;
    dc->vmsd = &vmstate_CMU;
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
