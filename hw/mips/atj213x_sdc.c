#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "exec/log.h"
#include "hw/sd/sd.h"
#include "sysemu/blockdev.h"
#include "trace.h"

#include "atj213x_sdc.h"

static void sd_command(AtjSDCState *s)
{
    SDRequest req;
    uint8_t rsp[16];

    req.cmd = s->regs[SDC_CMD] & 0x3f;
    req.arg = s->regs[SDC_ARG];

    int len = sd_do_command(s->card, &req, rsp);

    /* response request bitfield
     * response    | len|CMDRSP| crc 
     * r1, r1b, r6 |  4 | bit1 | CRC7
     * r0          |  0 | bit2 | N/A
     * r3          |  4 | bit3 | no CRC7
     * r2          | 16 | bit4 | CRC7
     */
    if ((s->regs[SDC_CMDRSP] & (1<<2)) && (len != 0))
    {
        qemu_log("%s() requested NRSP but rsp len = %d\n", __func__, len);
    }
    else if((s->regs[SDC_CMDRSP] & (1<<1)) && (len != 4))
    {
        qemu_log("%s() requested RSP1 but rsp len = %d\n", __func__, len);
    }
    else if((s->regs[SDC_CMDRSP] & (1<<3)) && (len != 4))
    {
        qemu_log("%s() requested RSP3 but rsp len = %d\n", __func__, len);
    }
    else if((s->regs[SDC_CMDRSP] & (1<<4)) && (len != 16))
    {
        qemu_log("%s() requested RSP2 but rsp len = %d\n", __func__, len);
    }

    switch (len)
    {
        case 0:
            s->regs[SDC_RSPBUF0] = 0;
            s->regs[SDC_RSPBUF1] = 0;
            s->regs[SDC_RSPBUF2] = 0;
            s->regs[SDC_RSPBUF3] = 0;
            break;

        case 4:
            /* bits 0-7 in RSPBUF0 is crc7 */
            s->regs[SDC_RSPBUF0] = (rsp[1]<<24)|(rsp[2]<<16)|(rsp[3]<<8);
            s->regs[SDC_RSPBUF1] = rsp[0];
            s->regs[SDC_RSPBUF2] = 0;
            s->regs[SDC_RSPBUF3] = 0;

            /* no crc error right? */
            s->regs[SDC_CRC7] = s->regs[SDC_RSPBUF0] & 0xff;
            break;

        case 16:
            s->regs[SDC_RSPBUF3] = (rsp[0]<<24)|(rsp[1]<<16)|(rsp[2]<<8)|rsp[3];
            s->regs[SDC_RSPBUF2] = (rsp[4]<<24)|(rsp[5]<<16)|(rsp[6]<<8)|rsp[7];
            s->regs[SDC_RSPBUF1] = (rsp[8]<<24)|(rsp[9]<<16)|(rsp[10]<<8)|rsp[11];
            s->regs[SDC_RSPBUF0] = (rsp[12]<<24)|(rsp[13]<<16)|(rsp[14]<<8)|rsp[15];
            break;

        default:
            qemu_log("%s() invalid reponse len\n", __func__);
    }

    s->regs[SDC_CMDRSP] = 0;
}

static bool SDC_enabled(AtjSDCState *s)
{
    return (s->regs[SDC_CTL] & (1<<7)) ? true : false;
}

static uint64_t SDC_read(void *opaque, hwaddr  addr, unsigned size)
{
    AtjSDCState *s = opaque;
    uint32_t dat;

    addr >>= 2;
    switch (addr)
    {
        case SDC_DAT:
            if (SDC_enabled(s) && s->regs[SDC_BYTECNT])
            {
                dat = sd_read_data(s->card) << 0  |
                      sd_read_data(s->card) << 8  |
                      sd_read_data(s->card) << 16 |
                      sd_read_data(s->card) << 24;

                s->regs[SDC_BYTECNT] -= size;
            }
            else
            {
                dat = 0xffffffff;
            }
            trace_atj_sdc_dat_read(dat);
            return dat;

        default:
            trace_atj_sdc_read(addr<<2, s->regs[addr]);
            return s->regs[addr];
    }
}

static void SDC_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    AtjSDCState *s = opaque;
    trace_atj_sdc_write(addr, value);

    addr >>= 2;
    s->regs[addr] = value;

    switch (addr)
    {
        case SDC_CTL:
        {
            int datawid = value & 3;
            qemu_log("%s() SD interface data width: %d\n", __func__, datawid ? ((datawid > 1) ? 8 : 4) : 1);
            break;
        }

        case SDC_CMD:
            break;

        case SDC_CMDRSP:
            if (SDC_enabled(s))
	    {
		if (value == 1)
                {
                    s->regs[addr] &= ~1;
                }

                if (value & (0xf << 1))
                {
                    sd_command(s);
                }
	    }
            break;

        case SDC_DAT:
            if (SDC_enabled(s) && s->regs[SDC_BYTECNT])
            {
                trace_atj_sdc_dat_write(value);

                sd_write_data(s->card, (value >> 0)  & 0xff);
                sd_write_data(s->card, (value >> 8)  & 0xff);
                sd_write_data(s->card, (value >> 16) & 0xff);
                sd_write_data(s->card, (value >> 24) & 0xff);
                s->regs[SDC_BYTECNT] -= size;
            }
            break;

        case SDC_CLK:
            s->regs[addr] = 0;
            break;

        default:
            break;
    }
};

static const MemoryRegionOps SDC_mmio_ops = {
    .read = SDC_read,
    .write = SDC_write,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void SDC_reset(DeviceState *d)
{
    AtjSDCState *s = ATJ213X_SDC(d);

    for (int i=0; i<SDC_REG_NUM; i++)
    {
        s->regs[i] = 0;
    }
}

static void SDC_init(Object *obj)
{
    AtjSDCState *s = ATJ213X_SDC(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->regs_region, obj, &SDC_mmio_ops, s, TYPE_ATJ213X_SDC, SDC_REG_NUM * 4);
    sysbus_init_mmio(dev, &s->regs_region);

    DriveInfo *dinfo = drive_get_next(IF_SD);
    BlockBackend *blk = dinfo ? blk_by_legacy_dinfo(dinfo) : NULL;
    s->card = sd_init(blk, false);

    /* SDC irq line */
    qdev_init_gpio_out(DEVICE(obj), &s->sdc_irq, 1);
}

static void SDC_realize(DeviceState *dev, Error **errp)
{
}

static const VMStateDescription vmstate_SDC = {
    .name = TYPE_ATJ213X_SDC,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AtjSDCState, SDC_REG_NUM),
        VMSTATE_END_OF_LIST()
    }
};

static void SDC_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = SDC_realize;
    dc->reset = SDC_reset;
    dc->vmsd = &vmstate_SDC;
    //dc->props = milkymist_sysctl_properties;
}

static const TypeInfo SDC_info = {
    .name          = TYPE_ATJ213X_SDC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AtjSDCState),
    .instance_init = SDC_init,
    .class_init    = SDC_class_init,
};

static void SDC_register_types(void)
{
    type_register_static(&SDC_info);
}

type_init(SDC_register_types)

DeviceState *SDC_create(hwaddr base, AtjINTCState *intc)
{
    DeviceState *dev;
    dev = qdev_create(NULL, TYPE_ATJ213X_SDC);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);

    /* connect SDC irq line to input mux of INTC */
    qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in(DEVICE(intc), IRQ_SD));
    return dev;
}

