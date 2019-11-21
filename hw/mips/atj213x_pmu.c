#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "trace.h"

#include "atj213x_pmu.h"

static void PMU_lradc_set(void *opaque, lradc_ch_e channel, int value)
{
    AtjPMUState *s = opaque;
    uint32_t tmp =  s->regs[PMU_LRADC];

    switch (channel)
    {
        case LRADC_CH_KEY:
            tmp &= ~(0xf << 24);
            tmp |= (value & 0x0f) << 24;
            break;

        case LRADC_CH_BAT:
            tmp &= ~(0x3f << 16);
            tmp |= (value & 0x3f) << 16;
            break;

        case LRADC_CH_TEMP:
            tmp &= ~(0x3f << 8);
            tmp |= (value & 0x3f) << 8;
            break;
    }

    s->regs[PMU_LRADC] = tmp;
}

static int PMU_lradc_get(void *opaque, lradc_ch_e channel)
{
    AtjPMUState *s = opaque;
    uint32_t value =  s->regs[PMU_LRADC];

    switch (channel)
    {
        case LRADC_CH_KEY:
            return (value >> 24) & 0x0f;
        case LRADC_CH_BAT:
            return (value >> 16) & 0x3f;
        case LRADC_CH_TEMP:
            return (value >> 8) & 0x3f;
        default:
            return 0;
    }

}

static int PMU_backlight_get(void *opaque)
{
    AtjPMUState *s = opaque;
    return (s->regs[PMU_CHG] >> 8) & 0x1f;
}

static uint64_t PMU_read(void *opaque, hwaddr  addr, unsigned size)
{
    AtjPMUState *s = opaque;
    addr >>= 2;
    switch (addr)
    {
        default:
            break;
    }

    trace_atj_pmu_read(addr<<2, s->regs[addr]);
    return s->regs[addr];
}

static void PMU_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    AtjPMUState *s = opaque;
    trace_atj_pmu_write(addr, value);

    addr >>= 2;
    switch (addr)
    {
        case PMU_CHG:
            trace_atj_backlight(s->backlight_get(s));
            break;

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
        s->regs[i] = 0;
    }

    /* set battery to ~3.9V */
    s->regs[PMU_LRADC] = (0x0f<<24) | (0x2f<<16);
}

static void PMU_init(Object *obj)
{
    AtjPMUState *s = ATJ213X_PMU(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->regs_region, obj, &PMU_mmio_ops, s, TYPE_ATJ213X_PMU, PMU_REG_NUM * 4);
    sysbus_init_mmio(dev, &s->regs_region);

    s->lradc_set = PMU_lradc_set;
    s->lradc_get = PMU_lradc_get;
    s->backlight_get = PMU_backlight_get;
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
    dc->user_creatable = false;
    dc->vmsd = &vmstate_PMU;
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
