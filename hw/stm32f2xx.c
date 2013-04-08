/*
 * STM32 Microcontroller
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Implementation based on ST Microelectronics "RM0008 Reference Manual Rev 10"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "stm32.h"
#include "stm32f2xx.h"
#include "exec/address-spaces.h"
#include "exec/memory.h"

/* Init STM32F2XX CPU and memory.
 flash_size and sram_size are in kb. */

static uint64_t kernel_load_translate_fn(void *opaque, uint64_t from_addr) {
    if (from_addr == 0x08000000) {
        return 0x00000000;
    }
    return from_addr;
}

void stm32f2xx_init(
            ram_addr_t flash_size,
            ram_addr_t ram_size,
            const char *kernel_filename,
            Stm32Gpio **stm32_gpio,
            Stm32Uart **stm32_uart,
            uint32_t osc_freq,
            uint32_t osc32_freq)
{
    MemoryRegion *address_space_mem = get_system_memory();
    qemu_irq *pic;
    int i;

    pic = armv7m_translated_init(address_space_mem, flash_size, ram_size, kernel_filename, kernel_load_translate_fn, NULL, "cortex-m3");

    // Create alias at 0x08000000 for internal flash, that is hard-coded at 0x00000000 in armv7m.c:
    // TODO: Let BOOT0 and BOOT1 configuration pins determine what is mapped at 0x00000000, see SYSCFG_MEMRMP.
    const hwaddr ARMV7_FLASH_ADDR = 0x00000000;
    MemoryRegionSection mrs = memory_region_find(address_space_mem, ARMV7_FLASH_ADDR, WORD_ACCESS_SIZE);
    MemoryRegion *flash_alias = g_new(MemoryRegion, 1);
    memory_region_init_alias(flash_alias, "stm32f2xx.flash.alias", mrs.mr, STM32_FLASH_ADDR_START, flash_size * 1024);
    memory_region_add_subregion(address_space_mem, STM32_FLASH_ADDR_START, flash_alias);

    DeviceState *rcc_dev = qdev_create(NULL, "stm32f2xx_rcc");
    qdev_prop_set_uint32(rcc_dev, "osc_freq", osc_freq);
    qdev_prop_set_uint32(rcc_dev, "osc32_freq", osc32_freq);
    stm32_init_periph(rcc_dev, STM32_RCC, 0x40023800, pic[STM32_RCC_IRQ]);

    DeviceState **gpio_dev = (DeviceState **)g_malloc0(sizeof(DeviceState *) * STM32F2XX_GPIO_COUNT);
    for(i = 0; i < STM32F2XX_GPIO_COUNT; i++) {
        stm32_periph_t periph = STM32_GPIOA + i;
        gpio_dev[i] = qdev_create(NULL, "stm32_gpio");
        QDEV_PROP_SET_PERIPH_T(gpio_dev[i], "periph", periph);
        qdev_prop_set_ptr(gpio_dev[i], "stm32_rcc", rcc_dev);
        stm32_init_periph(gpio_dev[i], periph, 0x40020000 + (i * 0x400), NULL);
        stm32_gpio[i] = (Stm32Gpio *)gpio_dev[i];
    }

    DeviceState *exti_dev = qdev_create(NULL, "stm32_exti");
    qdev_prop_set_ptr(exti_dev, "stm32_gpio", gpio_dev);
    stm32_init_periph(exti_dev, STM32_EXTI, 0x40013C00, NULL);
    SysBusDevice *exti_busdev = SYS_BUS_DEVICE(exti_dev);
    sysbus_connect_irq(exti_busdev, 0, pic[STM32_EXTI0_IRQ]);
    sysbus_connect_irq(exti_busdev, 1, pic[STM32_EXTI1_IRQ]);
    sysbus_connect_irq(exti_busdev, 2, pic[STM32_EXTI2_IRQ]);
    sysbus_connect_irq(exti_busdev, 3, pic[STM32_EXTI3_IRQ]);
    sysbus_connect_irq(exti_busdev, 4, pic[STM32_EXTI4_IRQ]);
    sysbus_connect_irq(exti_busdev, 5, pic[STM32_EXTI9_5_IRQ]);
    sysbus_connect_irq(exti_busdev, 6, pic[STM32_EXTI15_10_IRQ]);
    sysbus_connect_irq(exti_busdev, 7, pic[STM32_PVD_IRQ]);
    sysbus_connect_irq(exti_busdev, 8, pic[STM32_RTCAlarm_IRQ]);
    sysbus_connect_irq(exti_busdev, 9, pic[STM32_OTG_FS_WKUP_IRQ]);

    DeviceState *syscfg_dev = qdev_create(NULL, "stm32f2xx_syscfg");
    qdev_prop_set_ptr(syscfg_dev, "stm32_rcc", rcc_dev);
    qdev_prop_set_ptr(syscfg_dev, "stm32_exti", exti_dev);
    qdev_prop_set_bit(syscfg_dev, "boot0", 0);
    qdev_prop_set_bit(syscfg_dev, "boot1", 0);
    stm32_init_periph(syscfg_dev, STM32_SYSCFG, 0x40013800, NULL);

//    stm32_uart[STM32_UART1_INDEX] = stm32_create_uart_dev(STM32_UART1, rcc_dev, gpio_dev, afio_dev, 0x40011000, pic[STM32_UART1_IRQ]);
//    stm32_uart[STM32_UART2_INDEX] = stm32_create_uart_dev(STM32_UART2, rcc_dev, gpio_dev, afio_dev, 0x40004400, pic[STM32_UART2_IRQ]);
//    stm32_uart[STM32_UART3_INDEX] = stm32_create_uart_dev(STM32_UART3, rcc_dev, gpio_dev, afio_dev, 0x40004800, pic[STM32_UART3_IRQ]);
//    stm32_uart[STM32_UART4_INDEX] = stm32_create_uart_dev(STM32_UART4, rcc_dev, gpio_dev, afio_dev, 0x40004C00, pic[STM32_UART4_IRQ]);
//    stm32_uart[STM32_UART5_INDEX] = stm32_create_uart_dev(STM32_UART5, rcc_dev, gpio_dev, afio_dev, 0x40005000, pic[STM32_UART5_IRQ]);
//    stm32_uart[STM32_UART6_INDEX] = stm32_create_uart_dev(STM32_UART6, rcc_dev, gpio_dev, afio_dev, 0x40011400, pic[STM32_UART6_IRQ]);
}
