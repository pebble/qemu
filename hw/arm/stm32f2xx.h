#include "hw/arm/stm32.h"

#define STM32F2XX_GPIO_COUNT (STM32_GPIOI - STM32_GPIOA + 1)
#define STM32F2XX_SPI_COUNT 3

#define STM32F2XX_UART_COUNT 6

struct stm32f2xx {
    DeviceState *spi_dev[STM32F2XX_SPI_COUNT];
};
