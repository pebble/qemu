#include "hw/sysbus.h"
#include "hw/arm/stm32_clktree.h"
#include "hw/arm/stm32.h"

#define TYPE_STM32_RCC "stm32_rcc"

#define OBJECT_STM32_RCC(obj) \
    OBJECT_CHECK(Stm32Rcc, (obj), TYPE_STM32_RCC);

// class -> class
#define STM32_RCC_CLASS(klass) \
     OBJECT_CLASS_CHECK(Stm32RccClass, (klass), TYPE_STM32_RCC)

// object -> class
#define STM32_RCC_GET_CLASS(obj) \
    OBJECT_GET_CLASS(Stm32RccClass, (obj), TYPE_STM32_RCC)

typedef struct Stm32Rcc {
    /* Inherited */
    SysBusDevice busdev;

    /* Properties */
    uint32_t osc_freq;
    uint32_t osc32_freq;
} Stm32Rcc;

typedef struct Stm32RccClass {
    /*< private >*/
    SysBusDeviceClass parent_class;

    /*< public >*/
    void (*check_periph_clk)(Stm32Rcc *s, stm32_periph_t periph, SysBusDevice *busdev);
    void (*set_periph_clk_irq)(Stm32Rcc *s, stm32_periph_t periph, qemu_irq periph_irq);
    uint32_t (*get_periph_freq)(Stm32Rcc *s, stm32_periph_t periph);
} Stm32RccClass;

