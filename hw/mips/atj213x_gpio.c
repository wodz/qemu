#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "exec/log.h"
#include "trace.h"

#include "atj213x_gpio.h"

static void GPIO_out_update(AtjGPIOState *s)
{
    /* assert GPIO port A qemu output lines*/
    for (int bit=0; bit < 32; bit++)
    {
        if (s->regs[GPIO_AOUTEN] & (1 << bit))
        {
            qemu_set_irq(s->gpio_out[bit],
                         s->regs[GPIO_ADAT] & (1 << bit) ? 1 : 0);
        }
    }

    /* assert GPIO port B qemu output lines */
    for (int bit=0; bit < 32; bit++)
    {
        if (s->regs[GPIO_BOUTEN] & (1 << bit))
        {
            qemu_set_irq(s->gpio_out[32 + bit],
                         s->regs[GPIO_BDAT] & (1 << bit) ? 1 : 0);
        }
    }
}

static void GPIO_in_update(void * opaque, int n_IRQ, int level)
{
    AtjGPIOState *s = opaque;

    int port = n_IRQ / 32;
    int bit = n_IRQ % 32;

    trace_atj_gpio_in_update(port, bit, level);

    if (port == GPIOA)
    {
        if (level)
        {
            s->regs[GPIO_ADAT] |= (1 << bit);
        }
        else
        {
            s->regs[GPIO_ADAT] &= ~(1 << bit);
        }
    }
    else if (port == GPIOB)
    {
        if (level)
        {
            s->regs[GPIO_BDAT] |= (1 << bit);
        }
        else
        {
            s->regs[GPIO_BDAT] &= ~(1 << bit);
        }
    }
}

static uint64_t GPIO_read(void *opaque, hwaddr  addr, unsigned size)
{
    AtjGPIOState *s = opaque;

    addr >>= 2;
    trace_atj_gpio_read(addr<<2, s->regs[addr]);
    return s->regs[addr];
}

static void GPIO_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    AtjGPIOState *s = opaque;
    trace_atj_gpio_write(addr, value);

    addr >>= 2;
    switch (addr)
    {
        case GPIO_AOUTEN:
        case GPIO_ADAT:
        case GPIO_BOUTEN:
        case GPIO_BDAT:
            GPIO_out_update(s);
            break;

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

    /* Do not reset GPIO_ADAT and GPIO_BDAT
     * this will be set to default value by
     * GPIO_in_update() when connecting in gpio lines
     * setting it as qemu_irq_inverse() will set line high
     * and propagate to DAT registers.
     */
    s->regs[GPIO_AOUTEN] = 0;
    s->regs[GPIO_AINEN] = 0;
    s->regs[GPIO_BOUTEN] = 0;
    s->regs[GPIO_BINEN] = 0;
    s->regs[GPIO_MFCTL0] = (1<<27) | (3<<20) | (3<<16) | (2<<14) | (1<<12);
    s->regs[GPIO_MFCTL1] = 1<<13;
    s->regs[PAD_DRV] = (1<<15) | (1<<14) | (1<<13) | (1<<12) | (1<<11) | (1<<10) | (1<<9);
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

    AtjGPIOState *s = (AtjGPIOState *)dev;
    qdev_init_gpio_in(dev, GPIO_in_update, 2*32);
    qdev_init_gpio_out(dev, s->gpio_out, 2*32);
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
    dc->user_creatable = false;
    dc->vmsd = &vmstate_GPIO;
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
