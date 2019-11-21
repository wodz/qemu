#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "exec/address-spaces.h"
#include "hw/misc/unimp.h"
#include "hw/mips/cpudevs.h"
#include "trace.h"

#include "atj213x_cmu.h"
#include "atj213x_pmu.h"
#include "atj213x_intc.h"
#include "atj213x_rtc.h"
#include "atj213x_sdc.h"
#include "atj213x_gpio.h"
#include "atj213x_dmac.h"
#include "atj213x_dac.h"
#include "atj213x_yuv2rgb.h"
#include "atj213x_soc.h"

#define ATJ213X_PMU_BASE        0x10000000
#define ATJ213X_CMU_BASE        0x10010000
#define ATJ213X_RTC_BASE        0x10018000
#define ATJ213X_INTC_BASE       0x10020000
#define ATJ213X_SRAMOC_BASE     0x10030000
#define ATJ213X_PCNT_BASE       0x1003c000
#define ATJ213X_DSP_BASE        0x10050000
#define ATJ213X_DMAC_BASE       0x10060000
#define ATJ213X_SDRAMC_BASE     0x10070000
#define ATJ213X_MCA_BASE        0x10080000
#define ATJ213X_NAND_BASE       0x100a0000
#define ATJ213X_SDC_BASE        0x100b0000
#define ATJ213X_MHA_BASE        0x100c0000
#define ATJ213X_UDC_BASE        0x100e0000
#define ATJ213X_YUV2RGB_BASE    0x100f0000
#define ATJ213X_DAC_BASE        0x10100000
#define ATJ213X_ADC_BASE        0x10110000
#define ATJ213X_UART1_BASE      0x10160000
#define ATJ213X_UART2_BASE      0x10160020
#define ATJ213X_I2C1_BASE       0x10180000
#define ATJ213X_I2C2_BASE       0x10180020
#define ATJ213X_KEY_BASE        0x101a0000
#define ATJ213X_GPIO_BASE       0x101c0000

#define ATJ213X_IRAM_BASE       0x14040000

static void atj213x_soc_init(Object *obj)
{
    Atj213xSocState *s = ATJ213X_SOC(obj);

    object_initialize(&s->cpu, sizeof(s->cpu), "4KEc-" TYPE_MIPS_CPU);
    object_property_add_child(obj, "cpu", OBJECT(&s->cpu), NULL);

    object_initialize(&s->pmu, sizeof(s->pmu), TYPE_ATJ213X_PMU);
    object_property_add_child(obj, "pmu", OBJECT(&s->pmu), NULL);
    qdev_set_parent_bus(DEVICE(&s->pmu), sysbus_get_default());

    object_initialize(&s->intc, sizeof(s->intc), TYPE_ATJ213X_INTC);
    object_property_add_child(obj, "intc", OBJECT(&s->intc), NULL);
    qdev_set_parent_bus(DEVICE(&s->intc), sysbus_get_default());

    object_initialize(&s->cmu, sizeof(s->cmu), TYPE_ATJ213X_CMU);
    object_property_add_child(obj, "cmu", OBJECT(&s->cmu), NULL);
    qdev_set_parent_bus(DEVICE(&s->cmu), sysbus_get_default());

    object_initialize(&s->rtc, sizeof(s->rtc), TYPE_ATJ213X_RTC);
    object_property_add_child(obj, "rtc", OBJECT(&s->rtc), NULL);
    qdev_set_parent_bus(DEVICE(&s->rtc), sysbus_get_default());

    object_initialize(&s->sdc, sizeof(s->sdc), TYPE_ATJ213X_SDC);
    object_property_add_child(obj, "sdc", OBJECT(&s->sdc), NULL);
    qdev_set_parent_bus(DEVICE(&s->sdc), sysbus_get_default());

    object_initialize(&s->yuv2rgb, sizeof(s->yuv2rgb), TYPE_ATJ213X_YUV2RGB);
    object_property_add_child(obj, "yuv2rgb", OBJECT(&s->yuv2rgb), NULL);
    qdev_set_parent_bus(DEVICE(&s->yuv2rgb), sysbus_get_default());

    object_initialize(&s->gpio, sizeof(s->gpio), TYPE_ATJ213X_GPIO);
    object_property_add_child(obj, "gpio", OBJECT(&s->gpio), NULL);
    qdev_set_parent_bus(DEVICE(&s->gpio), sysbus_get_default());

    object_initialize(&s->dmac, sizeof(s->dmac), TYPE_ATJ213X_DMAC);
    object_property_add_child(obj, "dmac", OBJECT(&s->dmac), NULL);
    qdev_set_parent_bus(DEVICE(&s->dmac), sysbus_get_default());

    object_initialize(&s->dac, sizeof(s->dac), TYPE_ATJ213X_DAC);
    object_property_add_child(obj, "dac", OBJECT(&s->dac), NULL);
    qdev_set_parent_bus(DEVICE(&s->dac), sysbus_get_default());
}

static void atj213x_soc_realize(DeviceState *dev, Error **errp)
{
    Atj213xSocState *s = ATJ213X_SOC(dev);
    Error *err = NULL;

    object_property_set_bool(OBJECT(&s->cpu), true, "realized", &err);
    if (err != NULL)
    {
        error_propagate(errp, err);
        return;
    }

    /* Init CPU internal devices. */
    cpu_mips_irq_init_cpu(&s->cpu);
    cpu_mips_clock_init(&s->cpu);

    /* Internal SRAM memory */
    memory_region_init_ram(&s->iram, OBJECT(dev), "atj213x_soc.iram", 96*1024,
                           &err);
    if (err != NULL)
    {
        error_propagate(errp, err);
        return;
    }

    memory_region_add_subregion(get_system_memory(),
		                ATJ213X_IRAM_BASE,
				&s->iram);

    object_property_set_bool(OBJECT(&s->pmu), true, "realized", &err);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->pmu), 0, ATJ213X_PMU_BASE);

    object_property_set_bool(OBJECT(&s->intc), true, "realized", &err);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->intc), 0, ATJ213X_INTC_BASE);

    /* connect INTC out lines 0-4 to MIPS irq lines 2-6 */
    MIPSCPU *cpu = MIPS_CPU(&s->cpu);
    CPUMIPSState *env = &cpu->env;
    qdev_connect_gpio_out(DEVICE(&s->intc), 0, env->irq[2]);
    qdev_connect_gpio_out(DEVICE(&s->intc), 1, env->irq[3]);
    qdev_connect_gpio_out(DEVICE(&s->intc), 2, env->irq[4]);
    qdev_connect_gpio_out(DEVICE(&s->intc), 3, env->irq[5]);
    qdev_connect_gpio_out(DEVICE(&s->intc), 4, env->irq[6]);

    object_property_set_bool(OBJECT(&s->cmu), true, "realized", &err);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->cmu), 0, ATJ213X_CMU_BASE);

    object_property_set_bool(OBJECT(&s->rtc), true, "realized", &err);
    object_property_set_link(OBJECT(&s->rtc), OBJECT(&s->cmu), "cmu", &error_abort);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->rtc), 0, ATJ213X_RTC_BASE);

    /* connect RTC irq lines to INTC mux */
    qdev_connect_gpio_out(DEVICE(&s->rtc), 0, qdev_get_gpio_in(DEVICE(&s->intc), IRQ_T0));
    qdev_connect_gpio_out(DEVICE(&s->rtc), 1, qdev_get_gpio_in(DEVICE(&s->intc), IRQ_T1));
    qdev_connect_gpio_out(DEVICE(&s->rtc), 2, qdev_get_gpio_in(DEVICE(&s->intc), IRQ_WD));

    object_property_set_bool(OBJECT(&s->sdc), true, "realized", &err);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->sdc), 0, ATJ213X_SDC_BASE);

    /* connect SDC irq line to INTC mux */
    qdev_connect_gpio_out(DEVICE(&s->sdc), 0, qdev_get_gpio_in(DEVICE(&s->intc), IRQ_SD));

    object_property_set_bool(OBJECT(&s->yuv2rgb), true, "realized", &err);
    object_property_set_link(OBJECT(&s->yuv2rgb), OBJECT(&s->pmu), "pmu", &error_abort);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->yuv2rgb), 0, ATJ213X_YUV2RGB_BASE);
    
    object_property_set_bool(OBJECT(&s->gpio), true, "realized", &err);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gpio), 0, ATJ213X_GPIO_BASE);

    object_property_set_bool(OBJECT(&s->dmac), true, "realized", &err);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->dmac), 0, ATJ213X_DMAC_BASE);

    /* connect DMAC irq line to INTC mux */
    qdev_connect_gpio_out(DEVICE(&s->dmac), 0, qdev_get_gpio_in(DEVICE(&s->intc), IRQ_DMA));

    /* connect DAC drq line to DMAC */
    qdev_connect_gpio_out(DEVICE(&s->dac), 0, qdev_get_gpio_in(DEVICE(&s->dmac), 0));

    object_property_set_bool(OBJECT(&s->dac), true, "realized", &err);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->dac), 0, ATJ213X_DAC_BASE);


    create_unimplemented_device("SRAMOC", ATJ213X_SRAMOC_BASE, 0x08);
    create_unimplemented_device("PCNT", ATJ213X_PCNT_BASE, 0x0c);
    create_unimplemented_device("DSP", ATJ213X_DSP_BASE, 0x24);
    create_unimplemented_device("SDRAMC", ATJ213X_SDRAMC_BASE, 0x1c);
    create_unimplemented_device("MCA", ATJ213X_MCA_BASE, 0xaf7);
    create_unimplemented_device("NAND", ATJ213X_NAND_BASE, 0x74);
    create_unimplemented_device("MHA", ATJ213X_MHA_BASE, 0x24);
    create_unimplemented_device("UDC", ATJ213X_UDC_BASE, 0x1000);
    create_unimplemented_device("ADC", ATJ213X_ADC_BASE, 0x14);
    create_unimplemented_device("UART1", ATJ213X_UART1_BASE, 0x14);
    create_unimplemented_device("UART2", ATJ213X_UART2_BASE, 0x14);
    create_unimplemented_device("I2C1", ATJ213X_I2C1_BASE, 0x14);
    create_unimplemented_device("I2C2", ATJ213X_I2C2_BASE, 0x14);
    create_unimplemented_device("KEY", ATJ213X_KEY_BASE, 0x14);
}

static void atj213x_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = atj213x_soc_realize;
    dc->user_creatable = false;
}

static const TypeInfo atj213x_soc_type_info = {
    .name = TYPE_ATJ213X_SOC,
    .parent = TYPE_DEVICE,
    .instance_init = atj213x_soc_init,
    .instance_size = sizeof(Atj213xSocState),
    .class_init = atj213x_soc_class_init
};

static void atj213x_soc_register_types(void)
{
    type_register_static(&atj213x_soc_type_info);
}
type_init(atj213x_soc_register_types)

