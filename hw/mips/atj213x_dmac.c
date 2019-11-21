#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/ptimer.h"
#include "hw/sysbus.h"
#include "exec/log.h"
#include "trace.h"

#include "atj213x_dmac.h"

enum {
    DMA_TRG_DAC = 0x06,
    DMA_TRG_ADC = 0x07,
    DMA_TRG_SDARM = 0x10,
    DMA_TRG_IRAM = 0x11,
    DMA_TRG_NAND = 0x14,
    DMA_TRG_SD = 0x16,
    DMA_TRG_UDC = 0x17,
    DMA_TRG_LCM = 0x18
};

/* forward declarations */
static void DMAC_ch_run(AtjDMACState *s, int ch_no);
static void DMAC_ch_finish(AtjDMACState *s, int ch_no);

static void DMAC_update_irq(AtjDMACState *s)
{
    if (s->irqen & s->irqpd)
    {
        qemu_irq_raise(s->dmac_irq);
    }
    else
    {
        qemu_irq_lower(s->dmac_irq);
    }
}

static void DMAC_dma_xfer(AtjDMACState *s, int ch_no)
{
    uint8_t buf[4];

    AtjDMAChannel *chan = &s->ch[ch_no];
    AtjDMAchCtrl *act = &chan->ctrl[1];
    AtjDMAchCtrl *orig = &chan->ctrl[0];

    unsigned int stranwid = 1 << ((act->mode >> 1) & 0x03);
    unsigned int dtranwid = 1 << ((act->mode >> 17) & 0x03);

    while (act->rem && (!chan->drq || (chan->drq && chan->drq->asserted)))
    {
        /* source fixed size mode */
        if (!(act->mode & (1<<0)))
        {
            /* DMA will transfer in 8bit mode when remain counter is less
             * than Tran_Wide.
             */
            stranwid = (act->rem < stranwid) ? 1 : stranwid;
        }

        /* destination fixed size mode */
        if (!(act->mode & (1<<16)))
        {
            /* DMA will transfer in 8bit mode when remain counter is less
             * than Tran_Wide.
             */
            dtranwid = (act->rem < dtranwid) ? 1 : dtranwid;
        }

	int xsize = (dtranwid < stranwid) ? stranwid : dtranwid;
        for (int n = 0; n < xsize; n += stranwid)
        {
            cpu_physical_memory_read(act->src, buf + n, stranwid);

            /* source fixed address */
            if (!(act->mode & (1<<8)))
            {
                /* source address change direction */
                if (act->mode & (1 << 9))
                {
                    /* SDIR = 1 - decrease */
                    act->src -= stranwid;
                }
                else
                {
                    /* SDIR = 0 - increase */
                    act->src += stranwid;
                }
            }
        }

       for (int n = 0; n < xsize; n += dtranwid)
        {
            cpu_physical_memory_write(act->dst, buf + n, dtranwid);

            /* destination fixed address */
            if (!(act->mode & (1 << 24)))
            {
                /* destination address change direction */
                if (act->mode & (1 << 25))
                {
                    /* DDIR = 1 - decrease */
                    act->dst -= dtranwid;
                }
                else
                {
                    /* DDIR = 0 - increase */
                    act->dst += dtranwid;
                }
            }
        }
        act->rem -= xsize;
    }
    orig->rem = act->rem;

    /* half transfer passed */
    if (act->rem < act->cnt / 2)
    {
        s->irqpd |= (2 << ch_no);
    }

    /* transfer complete */
    if (!act->rem)
    {
        s->irqpd |= (1 << ch_no);

        if (act->mode & (1<<28))
        {
            /* In relode mode schedule next transfer */
            timer_mod(chan->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1e6);
        }
        else
        {
            /* mark channle as stopped */
            DMAC_ch_finish(s, ch_no);
        }
    }

    DMAC_update_irq(s);
}

static void DMA_drq(void * opaque, int n_IRQ, int level)
{
    AtjDMACState *s = opaque;
    AtjDMAdrqState *d = &s->drq[n_IRQ];

    trace_atj_dmac_drq(n_IRQ, level, d->ch ? d->ch->no : -1, d->ch ? d->ch->ctrl[0].cmd : -1);
    d->asserted = level;

    if (d->ch)
    {
        if (level)// && DMA_ch_active(opaque, d->ch->no))
        {
            DMAC_dma_xfer(opaque, d->ch->no);
        }
    }
}

static AtjDMAdrqState *drq_from_trg(AtjDMACState *s, int trg)
{
    switch (trg)
    {
        case DMA_TRG_DAC:
            return &s->drq[DMA_DAC_drq];

        case DMA_TRG_ADC:
            return &s->drq[DMA_ADC_drq];

        case DMA_TRG_NAND:
            return &s->drq[DMA_NAND_drq];

        /* don't simulate DRQ as SD reads and writes
         * are done whole at once
         * case DMA_TRG_SD:
         *   return &s->drq[DMA_SD_drq];
         */

        case DMA_TRG_UDC:
            return &s->drq[DMA_OTG_drq];

        /* don't simulate DRQ for LCM as
         * we update whole screen instantly
         */

        case DMA_TRG_LCM:
            return &s->drq[DMA_LCM_drq];

        default:
            return NULL;
    }
}

static void DMAC_ch_run(AtjDMACState *s, int ch_no)
{
    AtjDMAChannel *chan = &s->ch[ch_no];
    AtjDMAchCtrl *act = &chan->ctrl[1];
    AtjDMAchCtrl *orig = &chan->ctrl[0];


    /* copy transfer params to working copy */
    if (!act->cmd)
    {
        act->mode = orig->mode;
        act->src = orig->src;
        act->dst = orig->dst;
        act->cnt = orig->cnt;
        act->rem = orig->cnt;
        act->cmd = orig->cmd;

        trace_atj_dmac_ch_run(ch_no, 0, chan->ctrl[0].mode, chan->ctrl[0].src, chan->ctrl[0].dst, chan->ctrl[0].cnt);
    }
    else
    {
        trace_atj_dmac_ch_run(ch_no, 0, chan->ctrl[0].mode, chan->ctrl[0].src, chan->ctrl[0].dst, chan->ctrl[0].cnt);
    }


    int strg = (act->mode >> 3) & 0x1f;
    int dtrg = (act->mode >> 19) & 0x1f;

    AtjDMAdrqState *sdrq = drq_from_trg(s, strg);
    AtjDMAdrqState *ddrq = drq_from_trg(s, dtrg);

    if (sdrq && !ddrq)
    {
        chan->drq = sdrq;
        sdrq->ch = chan;
    }
    else if (!sdrq && ddrq)
    {
        chan->drq = ddrq;
        ddrq->ch = chan;
    }

    if (!chan->drq || (chan->drq && chan->drq->asserted))
    {
        DMAC_dma_xfer(s, ch_no);
    }
}

static void DMAC_ch0_cb(void *opaque)
{
    DMAC_ch_run(opaque, 0);
}

static void DMAC_ch1_cb(void *opaque)
{
    DMAC_ch_run(opaque, 1);
}

static void DMAC_ch2_cb(void *opaque)
{
    DMAC_ch_run(opaque, 2);
}

static void DMAC_ch3_cb(void *opaque)
{
    DMAC_ch_run(opaque, 3);
}

static void DMAC_ch4_cb(void *opaque)
{
    DMAC_ch_run(opaque, 4);
}

static void DMAC_ch5_cb(void *opaque)
{
    DMAC_ch_run(opaque, 5);
}

static void DMAC_ch6_cb(void *opaque)
{
    DMAC_ch_run(opaque, 6);
}

static void DMAC_ch7_cb(void *opaque)
{
    DMAC_ch_run(opaque, 7);
}

static void DMAC_ch_finish(AtjDMACState *s, int ch_no)
{
    AtjDMAChannel *chan = &s->ch[ch_no];
    AtjDMAchCtrl *orig = &chan->ctrl[0];
    AtjDMAchCtrl *act = &chan->ctrl[1];

    trace_atj_dmac_ch_finish(ch_no);

    orig->cmd &= ~1;
    act->cmd = 0;

    if (chan->drq)
    {
        chan->drq->ch = NULL;
        chan->drq = NULL;
    }
}

static uint64_t DMAC_read(void *opaque, hwaddr  addr, unsigned size)
{
    AtjDMACState *s = opaque;
    int ch_no;
    AtjDMAChannel *chan;
    AtjDMAchCtrl *act;
    uint32_t r = 0;

    addr >>= 2;
    switch (addr)
    {
        case DMA_CTL:
            r = s->ctl;
            break;

        case DMA_IRQEN:
            r = s->irqen;
            break;

        case DMA_IRQPD:
            r = s->irqpd;
            break;

        case DMA0_MODE:
        case DMA1_MODE:
        case DMA2_MODE:
        case DMA3_MODE:
        case DMA4_MODE:
        case DMA5_MODE:
        case DMA6_MODE:
        case DMA7_MODE:
            ch_no = (addr - DMA0_MODE)/8;
            chan = &s->ch[ch_no];
            act = &chan->ctrl[1];
            r = act->mode;
            break;

        case DMA0_SRC:
        case DMA1_SRC:
        case DMA2_SRC:
        case DMA3_SRC:
        case DMA4_SRC:
        case DMA5_SRC:
        case DMA6_SRC:
        case DMA7_SRC:
            ch_no = (addr - DMA0_SRC)/8;
            chan = &s->ch[ch_no];
            act = &chan->ctrl[1];
            r = act->src;
            break;

        case DMA0_DST:
        case DMA1_DST:
        case DMA2_DST:
        case DMA3_DST:
        case DMA4_DST:
        case DMA5_DST:
        case DMA6_DST:
        case DMA7_DST:
            ch_no = (addr - DMA0_DST)/8;
            chan = &s->ch[ch_no];
            act = &chan->ctrl[1];
            r = act->dst;
            break;

        case DMA0_CNT:
        case DMA1_CNT:
        case DMA2_CNT:
        case DMA3_CNT:
        case DMA4_CNT:
        case DMA5_CNT:
        case DMA6_CNT:
        case DMA7_CNT:
            ch_no = (addr - DMA0_CNT)/8;
            chan = &s->ch[ch_no];
            act = &chan->ctrl[1];
            r = act->cnt;
            break;

	case DMA0_REM:
        case DMA1_REM:
        case DMA2_REM:
        case DMA3_REM:
        case DMA4_REM:
        case DMA5_REM:
        case DMA6_REM:
        case DMA7_REM:
            ch_no = (addr - DMA0_REM)/8;
            chan = &s->ch[ch_no];
            act = &chan->ctrl[1];
            r = act->rem;
            break;

        case DMA0_CMD:
        case DMA1_CMD:
        case DMA2_CMD:
        case DMA3_CMD:
        case DMA4_CMD:
        case DMA5_CMD:
        case DMA6_CMD:
        case DMA7_CMD:
            ch_no = (addr - DMA0_CMD)/8;
            chan = &s->ch[ch_no];
            act = &chan->ctrl[1];
            r = act->cmd;
            break;

        default:
            qemu_log("%s() Restricted area access\n", __func__);
            break;
    }

    trace_atj_dmac_read(addr<<2, r);
    return r;
}

static void DMAC_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    AtjDMACState *s = opaque;
    int ch_no;
    AtjDMAChannel *chan;
    AtjDMAchCtrl *orig;
    trace_atj_dmac_write(addr, value);

    addr >>= 2;
    switch (addr)
    {
        case DMA_CTL:
            s->ctl = value;
            break;

        case DMA_IRQEN:
            s->irqen = value;
            DMAC_update_irq(s);
            break;

        case DMA_IRQPD:
            s->irqpd &= ~value;
            DMAC_update_irq(s);
            break;

        case DMA0_MODE:
        case DMA1_MODE:
        case DMA2_MODE:
        case DMA3_MODE:
        case DMA4_MODE:
        case DMA5_MODE:
        case DMA6_MODE:
        case DMA7_MODE:
            ch_no = (addr - DMA0_MODE)/8;
            chan = &s->ch[ch_no];
            orig = &chan->ctrl[0];
            orig->mode = value;
            break;

	case DMA0_SRC:
        case DMA1_SRC:
        case DMA2_SRC:
        case DMA3_SRC:
        case DMA4_SRC:
        case DMA5_SRC:
        case DMA6_SRC:
        case DMA7_SRC:
            ch_no = (addr - DMA0_SRC)/8;
            chan = &s->ch[ch_no];
            orig = &chan->ctrl[0];
            orig->src = value;
            break;

        case DMA0_DST:
        case DMA1_DST:
        case DMA2_DST:
        case DMA3_DST:
        case DMA4_DST:
        case DMA5_DST:
        case DMA6_DST:
        case DMA7_DST:
            ch_no = (addr - DMA0_DST)/8;
            chan = &s->ch[ch_no];
            orig = &chan->ctrl[0];
            orig->dst = value;
            break;

        case DMA0_CNT:
        case DMA1_CNT:
        case DMA2_CNT:
        case DMA3_CNT:
        case DMA4_CNT:
        case DMA5_CNT:
        case DMA6_CNT:
        case DMA7_CNT:
            ch_no = (addr - DMA0_CNT)/8;
            chan = &s->ch[ch_no];
            orig = &chan->ctrl[0];
            orig->cnt = value;
            break;

	case DMA0_REM:
        case DMA1_REM:
        case DMA2_REM:
        case DMA3_REM:
        case DMA4_REM:
        case DMA5_REM:
        case DMA6_REM:
        case DMA7_REM:
            ch_no = (addr - DMA0_REM)/8;
            chan = &s->ch[ch_no];
            orig = &chan->ctrl[0];
            orig->rem = value;
            break;

        case DMA0_CMD:
        case DMA1_CMD:
        case DMA2_CMD:
        case DMA3_CMD:
        case DMA4_CMD:
        case DMA5_CMD:
        case DMA6_CMD:
        case DMA7_CMD:
            ch_no = (addr - DMA0_CMD)/8;
            chan = &s->ch[ch_no];
            orig = &chan->ctrl[0];
            orig->cmd = value;
            if ((value & 1) && !(value & 2))
            {
                DMAC_ch_run(s, ch_no);
            }
            break;

        default:
            qemu_log("%s() Restricted area access\n", __func__);
    }
};

static const MemoryRegionOps DMAC_mmio_ops = {
    .read = DMAC_read,
    .write = DMAC_write,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void DMAC_ch_reset(void *opaque, int ch_no)
{
    AtjDMACState *s = opaque;
    AtjDMAChannel *ch = &s->ch[ch_no];
    AtjDMAchCtrl *orig = &ch->ctrl[0];
    AtjDMAchCtrl *act = &ch->ctrl[1];

    ch->no = ch_no;
    ch->drq = NULL;
    orig->mode = act->mode = 0;
    orig->src = act->src = 0;
    orig->dst = act->dst = 0;
    orig->cnt = act->cnt = 0;
    orig->rem = act->rem = 0;
    orig->cmd = act->cmd = 0;
}

static void DMAC_reset(DeviceState *d)
{
    AtjDMACState *s = ATJ213X_DMAC(d);

    s->ctl = 0;
    s->irqen = 0;
    s->irqpd = 0;

    for (int i=0; i<DMAC_CHANNELS_NUM; i++)
    {
        DMAC_ch_reset(s, i);
    }

    for (int i=0; i<DMA_drq_NUM; i++)
    {
        AtjDMAdrqState *drq = &s->drq[i];
        drq->ch = NULL;
    }
}

static void DMAC_init(Object *obj)
{
    AtjDMACState *s = ATJ213X_DMAC(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->regs_region, obj, &DMAC_mmio_ops, s, TYPE_ATJ213X_DMAC, DMAC_REG_NUM * 4);
    sysbus_init_mmio(dev, &s->regs_region);

    qdev_init_gpio_out(DEVICE(obj), &s->dmac_irq, 1);

    s->ch[0].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, DMAC_ch0_cb, (void *)s);
    s->ch[1].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, DMAC_ch1_cb, (void *)s);
    s->ch[2].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, DMAC_ch2_cb, (void *)s);
    s->ch[3].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, DMAC_ch3_cb, (void *)s);
    s->ch[4].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, DMAC_ch4_cb, (void *)s);
    s->ch[5].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, DMAC_ch5_cb, (void *)s);
    s->ch[6].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, DMAC_ch6_cb, (void *)s);
    s->ch[7].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, DMAC_ch7_cb, (void *)s);
}

static void DMAC_realize(DeviceState *dev, Error **errp)
{
    /* drq lines of DAC, ADC, NAND, SD, OTG, LCM */
    qdev_init_gpio_in(dev, DMA_drq, DMA_drq_NUM);
}

static const VMStateDescription vmstate_DMAC_ch_ctrl = {
    .name = "atj213x_DMAC_ch_ctrl",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(mode, AtjDMAchCtrl),
        VMSTATE_UINT32(src, AtjDMAchCtrl),
        VMSTATE_UINT32(dst, AtjDMAchCtrl),
        VMSTATE_UINT32(cnt, AtjDMAchCtrl),
        VMSTATE_UINT32(rem, AtjDMAchCtrl),
        VMSTATE_UINT32(cmd, AtjDMAchCtrl),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_DMAC_ch = {
    .name = "atj213x_DMAC_channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, AtjDMAChannel),
        VMSTATE_INT32(no, AtjDMAChannel),
        VMSTATE_STRUCT_ARRAY(ctrl, AtjDMAChannel, 2, 1, vmstate_DMAC_ch_ctrl, AtjDMAchCtrl),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_DMAC = {
    .name = TYPE_ATJ213X_DMAC,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(ctl, AtjDMACState),
        VMSTATE_UINT32(irqen, AtjDMACState),
        VMSTATE_UINT32(irqpd, AtjDMACState),
        VMSTATE_STRUCT_ARRAY(ch, AtjDMACState, DMAC_CHANNELS_NUM,
                             1, vmstate_DMAC_ch, AtjDMAChannel),
        VMSTATE_END_OF_LIST()
    }
};

static void DMAC_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = DMAC_realize;
    dc->reset = DMAC_reset;
    dc->user_creatable = false;
    dc->vmsd = &vmstate_DMAC;
}

static const TypeInfo DMAC_info = {
    .name          = TYPE_ATJ213X_DMAC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AtjDMACState),
    .instance_init = DMAC_init,
    .class_init    = DMAC_class_init,
};

static void DMAC_register_types(void)
{
    type_register_static(&DMAC_info);
}

type_init(DMAC_register_types)
