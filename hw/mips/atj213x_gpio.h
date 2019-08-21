/* GPIO Block */
#ifndef ATJ213X_GPIO_H
#define ATJ213X_GPIO_H

#define TYPE_ATJ213X_GPIO "atj213x-GPIO"
#define ATJ213X_GPIO(obj) \
    OBJECT_CHECK(AtjGPIOState, (obj), TYPE_ATJ213X_GPIO)
enum {
    GPIO_AOUTEN,
    GPIO_AINEN,
    GPIO_ADAT,
    GPIO_BOUTEN,
    GPIO_BINEN,
    GPIO_BDAT,
    GPIO_MFCTL0,
    GPIO_MFCTL1,
    PAD_DRV = 0x22,
    GPIO_REG_NUM
};

enum {
    GPIOA,
    GPIOB,
    GPIO_PORT_NUM
};

struct AtjGPIOState {
    SysBusDevice parent_obj;
    MemoryRegion regs_region;

    /* gpio_in are allocated with qdev_init_gpio_in */
    qemu_irq gpio_out[GPIO_PORT_NUM * 32];

    uint32_t regs[GPIO_REG_NUM];
};
typedef struct AtjGPIOState AtjGPIOState;

DeviceState *GPIO_create(hwaddr base);
#endif
