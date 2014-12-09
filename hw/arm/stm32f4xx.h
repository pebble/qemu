#include "hw/arm/stm32.h"


#define STM32F4XX_GPIO_COUNT (STM32_GPIOK - STM32_GPIOA + 1)
#define STM32F4XX_SPI_COUNT 3

#define STM32F4XX_UART_COUNT 8

struct stm32f4xx {
    DeviceState *spi_dev[STM32F4XX_SPI_COUNT];
};
