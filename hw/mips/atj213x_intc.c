#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "exec/log.h"
#include "trace.h"

#include "atj213x_intc.h"

static void INTC_update_irq(AtjINTCState *s)
{
    int irq_lines[INTC_OUT_IRQS_NUM] = {0, 0, 0, 0, 0};

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

            irq_lines[irq_out_no] |= level;

        }
    }

    for (int i=0; i<INTC_OUT_IRQS_NUM; i++)
    {
        /* update mips irq lines */
        trace_atj_intc_out(i, irq_lines[i]);
        qemu_set_irq(s->irq_out[i], irq_lines[i]);
    }
}

static void INTC_set_irq(void *opaque, int n_IRQ, int level)
{
    AtjINTCState *s = ATJ213X_INTC(opaque);
    uint32_t mask = (1 << n_IRQ);

    /* set or clear pending bit */
    if (level)
    {
        s->regs[INTC_PD] |= mask;
    }
    else
    {
        s->regs[INTC_PD] &= ~mask;
    }

    /* if unmasked, update irq state */
    if (s->regs[INTC_MSK] & mask)
    {
        trace_atj_intc_in(mask, level);
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
            trace_atj_intc_read(addr<<2, s->regs[addr]);
            return s->regs[addr];
    }
}

static void INTC_write(void *opaque, hwaddr addr, uint64_t value,
                       unsigned size)
{
    AtjINTCState *s = opaque;
    trace_atj_intc_write(addr, value);

    addr >>= 2;
    switch (addr)
    {
        case INTC_PD:
            /* read only */
            qemu_log("%s() READONLY addr: 0x" TARGET_FMT_plx " value: 0x"
                     TARGET_FMT_plx "\n", __func__, addr << 2, value);
            return;

        default:
            s->regs[addr] = value;
            INTC_update_irq(s);
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

DeviceState *INTC_create(hwaddr base, MIPSCPU *cpu)
{
    DeviceState *dev = qdev_create(NULL, TYPE_ATJ213X_INTC);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);

    CPUMIPSState *mipscpu = &cpu->env;

    /* void qdev_connect_gpio_out(DeviceState *dev, int n, qemu_irq pin);
     * INTC out lines 0-4 are connected to MIPS irq lines 2-6
     */
    qdev_connect_gpio_out(dev, 0, mipscpu->irq[2]);
    qdev_connect_gpio_out(dev, 1, mipscpu->irq[3]);
    qdev_connect_gpio_out(dev, 2, mipscpu->irq[4]);
    qdev_connect_gpio_out(dev, 3, mipscpu->irq[5]);
    qdev_connect_gpio_out(dev, 4, mipscpu->irq[6]);

    return dev;
}

