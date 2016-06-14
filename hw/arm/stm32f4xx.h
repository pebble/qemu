#include "hw/arm/stm32.h"


#define STM32F4XX_GPIO_COUNT (STM32_GPIOK - STM32_GPIOA + 1)
#define STM32F4XX_SPI_COUNT 6

#define STM32F4XX_UART_COUNT 8
#define STM32F4XX_TIM_COUNT 14

struct stm32f4xx {
    DeviceState *spi_dev[STM32F4XX_SPI_COUNT];
    DeviceState *qspi_dev;

    qemu_irq display_done_signal;
};
