#include "qemu/osdep.h"
#include "qapi/error.h"
#include "cpu.h"
#include "sysemu/sysemu.h"
#include "sysemu/watchdog.h"
#include "hw/sysbus.h"
#include "trace.h"

#include "atj213x_rtc.h"

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

static void WDT_expire_cb(void *opaque)
{
    AtjRTCState *s = opaque;
    watchdog_perform_action();
    timer_del(s->wdt_timer);
}

static void WDT_reload(void *opaque)
{
    AtjRTCState *s = opaque;
    const int wdt_reload[] = {176, 352, 1400, 5600, 22200, 45000, 90000, 180000};

    int idx = (s->regs[RTC_WDCTL] >> 1) & 7;
    int reload = wdt_reload[idx];

    timer_mod(s->wdt_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + reload);
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

    trace_atj_rtc_read(addr<<2, s->regs[addr]);
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

    trace_atj_rtc_write(addr, value);

    addr >>= 2;
    switch (addr)
    {
        case RTC_WDCTL:
            /* WDT is enabled */
            if (s->regs[addr] & (1<<4))
            {
                /* WDT feed */
                if (value & 1)
                {
                    WDT_reload(opaque);
                }

                if (!(value & (1<<4)))
                {
                   timer_del(s->wdt_timer);
                }
            }
            else
            {
                if (value & (1<<4))
                {
                    WDT_reload(opaque);
                }
            }

            value &= ~1;
            break;

        case RTC_T0CTL:
        case RTC_T1CTL:
            if (value & 1)
            {
               /* writing ZIPD bit clears irq pending flag */
               value &= ~1;
               s->regs[addr] &= ~1;
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
            /* initial countdown value for timer0 */
            ptimer_set_limit(s->ptimer[0], value, 1);
            break;

        case RTC_T1:
            /* initial countdown value for timer1 */
            ptimer_set_limit(s->ptimer[1], value, 1);
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

    timer_del(s->wdt_timer);
}

static void RTC_init(Object *obj)
{
    AtjRTCState *s = ATJ213X_RTC(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    object_property_add_link(OBJECT(s), "cmu", TYPE_ATJ213X_CMU,
                             (Object **)&s->cmu,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_UNREF_ON_RELEASE,
                             &error_abort);

    memory_region_init_io(&s->regs_region, obj, &RTC_mmio_ops, s, TYPE_ATJ213X_RTC, RTC_REG_NUM * 4);
    sysbus_init_mmio(dev, &s->regs_region);

    /* irq out lines from RTC block */
    qdev_init_gpio_out(DEVICE(obj), s->tmr_irq, 2);
    qdev_init_gpio_out(DEVICE(obj), &s->wdt_irq, 1);
}

static void RTC_realize(DeviceState *dev, Error **errp)
{
    AtjRTCState *s = ATJ213X_RTC(dev);
    s->bh[0] = qemu_bh_new(T0_zero_cb, s);
    s->bh[1] = qemu_bh_new(T1_zero_cb, s);

    s->ptimer[0] = ptimer_init(s->bh[0], PTIMER_POLICY_DEFAULT);
    s->ptimer[1] = ptimer_init(s->bh[1], PTIMER_POLICY_DEFAULT);

    s->wdt_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, WDT_expire_cb, dev);
    select_watchdog_action("reset");
}

static const VMStateDescription RTC_vmstate = {
    .name = TYPE_ATJ213X_RTC,
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
    dc->user_creatable = false;
    dc->vmsd = &RTC_vmstate;
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
