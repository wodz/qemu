#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "exec/log.h"
#include "trace.h"

#include "atj213x_dac.h"

static void update_drq(AtjDACState *s)
{
    if (s->buffer_level < ATJ_DAC_BUFFER_SIZE)
    {
        qemu_irq_raise(s->dma_rdy);
    }
    else
    {
        qemu_irq_lower(s->dma_rdy);
    }
}

static uint64_t DAC_read(void *opaque, hwaddr  addr, unsigned size)
{
    AtjDACState *s = opaque;

    addr >>= 2;
    switch (addr)
    {
        default:
            break;
    }

    trace_atj_dac_read(addr<<2, s->regs[addr]);
    return s->regs[addr];
}

static void DAC_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    AtjDACState *s = opaque;
    trace_atj_dac_write(addr, value);

    addr >>= 2;
    switch (addr)
    {
        case DAC_FIFOCTL:
            if (value & 1)
            {
                /* fifo reset */
                s->buffer_level = 0;
                update_drq(s);
            }
            break;

        case DAC_DAT:
            if (s->buffer_level < ATJ_DAC_BUFFER_SIZE)
            {
                s->buffer[s->buffer_level++] = value;

                /* if last transfer made buffer full
                 * deassert DMA RDY line
                 */
                if (s->buffer_level >= ATJ_DAC_BUFFER_SIZE)
                {
                    update_drq(s);
                }
            }
            break;

        case DAC_ANALOG:
        {
            /* scale 32 levels -1.8dB each of ATJ213X DAC
             * to 256 levels of AUD
             */
            unsigned int vol = 255 - (((value >> 3) & 0x1f) << 3);
            AUD_set_volume_out(s->voice, 0, vol, vol);

            /* no break so default case will be taken as well */
        }

        default:
            s->regs[addr] = value;
            break;
    }
};

static const MemoryRegionOps DAC_mmio_ops = {
    .read = DAC_read,
    .write = DAC_write,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void DAC_reset(DeviceState *d)
{
    AtjDACState *s = ATJ213X_DAC(d);

    for (int i=0; i<DAC_REG_NUM; i++)
    {
        s->regs[i] = 0;
    }
}

static int xfer_to_AUD(void *opaque, void *buffer, int bytes)
{
    AtjDACState *s = opaque;
    int xfered = 0;
    int n;

    do
    {
        n = AUD_write(s->voice, buffer, bytes);
        bytes -= n;
        buffer +=n;
        xfered += n;
    } while (n);

    return xfered;
}

static void DAC_out_cb(void *opaque, int free_b)
{
    AtjDACState *s = opaque;

    void *buffer;
    int bytes;

    trace_atj_dac_out_cb(s->buffer_level, free_b);

    if (s->buffer_level)
    {
        buffer = s->buffer;
        bytes = s->buffer_level * sizeof(uint32_t);

        int xfered = xfer_to_AUD(opaque, buffer, bytes);
        s->buffer_level -= xfered/sizeof(uint32_t);

        if (s->buffer_level)
        {
            /* Some samples from DAC buffer were not consumed by AUD
             * move remining samples to the head of the buffer
             */
            memmove(s->buffer, s->buffer + (xfered/sizeof(uint32_t)), s->buffer_level*sizeof(uint32_t));
        }
    }
    else
    {
        buffer = s->silence;
        bytes = sizeof(s->silence);
        xfer_to_AUD(opaque, buffer, bytes);
    }

    update_drq(s);
}

static void DAC_init(Object *obj)
{
    AtjDACState *s = ATJ213X_DAC(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->regs_region, obj, &DAC_mmio_ops, s, TYPE_ATJ213X_DAC, DAC_REG_NUM * 4);
    sysbus_init_mmio(dev, &s->regs_region);


    /* register audio card */
    AUD_register_card("dac", &s->card);

    struct audsettings as;
    as.freq = 44100;
    as.nchannels = 2;
    as.fmt = AUD_FMT_S32;
    as.endianness = 0;

    s->voice = AUD_open_out(&s->card,
                            s->voice,
                            "dac.out",
                            s,
                            DAC_out_cb,
                            &as
    );

    AUD_set_volume_out(s->voice, 0, 255, 255);

    memset(s->silence, 0x00, sizeof(s->silence));
    memset(s->buffer, 0x00, sizeof(s->buffer));
    s->buffer_level = 0;


    AUD_set_active_out(s->voice, 1);
}

static void DAC_realize(DeviceState *dev, Error **errp)
{
    AtjDACState *s = ATJ213X_DAC(dev);
    qdev_init_gpio_out(dev, &s->dma_rdy, 1);
}

static const VMStateDescription vmstate_DAC = {
    .name = TYPE_ATJ213X_DAC,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AtjDACState, DAC_REG_NUM),
        VMSTATE_END_OF_LIST()
    }
};

static void DAC_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = DAC_realize;
    dc->reset = DAC_reset;
    dc->vmsd = &vmstate_DAC;
    //dc->props = milkymist_sysctl_properties;
}

static const TypeInfo DAC_info = {
    .name          = TYPE_ATJ213X_DAC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AtjDACState),
    .instance_init = DAC_init,
    .class_init    = DAC_class_init,
};

static void DAC_register_types(void)
{
    type_register_static(&DAC_info);
}

type_init(DAC_register_types)
