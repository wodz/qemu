/* Interrupt Controller - INTC */
#ifndef ATJ213X_INTC_H
#define ATJ213X_INTC_H

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

//DeviceState *INTC_create(hwaddr base, MIPSCPU *cpu);

#endif
