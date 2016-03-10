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

#include "hw/arm/stm32.h"
#include "stm32f2xx.h"
#include "exec/address-spaces.h"
#include "exec/memory.h"
#include "hw/ssi.h"
#include "hw/block/flash.h"
#include "sysemu/blockdev.h" // drive_get

static const char *stm32f2xx_periph_name_arr[] = {
    ENUM_STRING(STM32_UART1),
    ENUM_STRING(STM32_UART2),
    ENUM_STRING(STM32_UART3),
    ENUM_STRING(STM32_UART4),
    ENUM_STRING(STM32_UART5),
    ENUM_STRING(STM32_UART6),

    ENUM_STRING(STM32_SPI1),
    ENUM_STRING(STM32_SPI2),
    ENUM_STRING(STM32_SPI3),

    ENUM_STRING(STM32_I2C1),
    ENUM_STRING(STM32_I2C2),
    ENUM_STRING(STM32_I2C3),

    ENUM_STRING(STM32_TIM1),
    ENUM_STRING(STM32_TIM2),
    ENUM_STRING(STM32_TIM3),
    ENUM_STRING(STM32_TIM4),
    ENUM_STRING(STM32_TIM5),
    ENUM_STRING(STM32_TIM6),
    ENUM_STRING(STM32_TIM7),
    ENUM_STRING(STM32_TIM8),
    ENUM_STRING(STM32_TIM9),
    ENUM_STRING(STM32_TIM10),
    ENUM_STRING(STM32_TIM11),
    ENUM_STRING(STM32_TIM12),
    ENUM_STRING(STM32_TIM13),
    ENUM_STRING(STM32_TIM14),

    ENUM_STRING(STM32_GPIOA),
    ENUM_STRING(STM32_GPIOB),
    ENUM_STRING(STM32_GPIOC),
    ENUM_STRING(STM32_GPIOD),
    ENUM_STRING(STM32_GPIOE),
    ENUM_STRING(STM32_GPIOF),
    ENUM_STRING(STM32_GPIOG),
    ENUM_STRING(STM32_GPIOH),
    ENUM_STRING(STM32_GPIOI),
    ENUM_STRING(STM32_GPIOJ),
    ENUM_STRING(STM32_GPIOK),

    ENUM_STRING(STM32_PERIPH_COUNT)
};

/* Init STM32F2XX CPU and memory.
 flash_size and sram_size are in kb. */

static uint64_t kernel_load_translate_fn(void *opaque, uint64_t from_addr) {
    if (from_addr == STM32_FLASH_ADDR_START) {
        return 0x00000000;
    }
    return from_addr;
}

static
void do_sys_reset(void *opaque, int n, int level)
{
    if (level) {
        qemu_system_reset_request();
    }
}

void stm32f2xx_init(
            ram_addr_t flash_size,
            ram_addr_t ram_size,
            const char *kernel_filename,
            Stm32Gpio **stm32_gpio,
            Stm32Uart **stm32_uart,
            Stm32Timer **stm32_timer,
            DeviceState **stm32_rtc,
            uint32_t osc_freq,
            uint32_t osc32_freq,
            struct stm32f2xx *stm,
            ARMCPU **cpu)
{
    MemoryRegion *address_space_mem = get_system_memory();
    DriveInfo *dinfo;
    DeviceState *nvic;
    int i;

    Object *stm32_container = container_get(qdev_get_machine(), "/stm32");

    nvic = armv7m_translated_init(
                stm32_container,
                address_space_mem,
                flash_size * 1024,
                ram_size * 1024,
                0, /* default number of irqs */
                kernel_filename,
                kernel_load_translate_fn,
                NULL,
                "cortex-m3",
                cpu);

    qdev_connect_gpio_out_named(nvic, "SYSRESETREQ", 0,
                                qemu_allocate_irq(&do_sys_reset, NULL, 0));

    dinfo = drive_get(IF_PFLASH, 0, 0);
    if (dinfo) {
        f2xx_flash_register(blk_by_legacy_dinfo(dinfo), 1 * 0x08000000, flash_size * 1024);
    }

    // Create alias at 0x08000000 for internal flash, that is hard-coded at 0x00000000 in armv7m.c:
    // TODO: Let BOOT0 and BOOT1 configuration pins determine what is mapped at 0x00000000, see SYSCFG_MEMRMP.

    MemoryRegionSection mrs = memory_region_find(address_space_mem, STM32_FLASH_ADDR_START, WORD_ACCESS_SIZE);
    MemoryRegion *flash_alias = g_new(MemoryRegion, 1);
    memory_region_init_alias(
            flash_alias,
            NULL,
            "stm32f2xx.flash.alias",
            mrs.mr,
            0,
            flash_size * 1024);
    memory_region_add_subregion(address_space_mem, 0, flash_alias);

    DeviceState *rcc_dev = qdev_create(NULL, "stm32f2xx_rcc");
    qdev_prop_set_uint32(rcc_dev, "osc_freq", osc_freq);
    qdev_prop_set_uint32(rcc_dev, "osc32_freq", osc32_freq);
    object_property_add_child(stm32_container, "rcc", OBJECT(rcc_dev), NULL);
    stm32_init_periph(rcc_dev, STM32_RCC_PERIPH, 0x40023800,
                      qdev_get_gpio_in(nvic, STM32_RCC_IRQ));

    DeviceState **gpio_dev = (DeviceState **)g_malloc0(sizeof(DeviceState *) * STM32F2XX_GPIO_COUNT);
    for(i = 0; i < STM32F2XX_GPIO_COUNT; i++) {
        stm32_periph_t periph = STM32_GPIOA + i;
        gpio_dev[i] = qdev_create(NULL, "stm32f2xx_gpio");
        gpio_dev[i]->id = stm32f2xx_periph_name_arr[periph];
        qdev_prop_set_int32(gpio_dev[i], "periph", periph);
//        qdev_prop_set_ptr(gpio_dev[i], "stm32_rcc", rcc_dev);
        stm32_init_periph(gpio_dev[i], periph, 0x40020000 + (i * 0x400), NULL);
        stm32_gpio[i] = (Stm32Gpio *)gpio_dev[i];
    }

    /* Connect the WKUP pin (GPIO A, pin 0) to the NVIC's WKUP handler */
    qemu_irq nvic_wake_irq = qdev_get_gpio_in_named(DEVICE((*cpu)->env.nvic), "wakeup_in", 0);
    f2xx_gpio_wake_set((stm32f2xx_gpio *)(stm32_gpio[STM32_GPIOA_INDEX]), 0, nvic_wake_irq);


    /* EXTI */
    DeviceState *exti_dev = qdev_create(NULL, "stm32_exti");
    qdev_prop_set_ptr(exti_dev, "stm32_gpio", gpio_dev);
    stm32_init_periph(exti_dev, STM32_EXTI_PERIPH, 0x40013C00, NULL);
    SysBusDevice *exti_busdev = SYS_BUS_DEVICE(exti_dev);
    /* IRQs from EXTI to NVIC */
    sysbus_connect_irq(exti_busdev, 0, qdev_get_gpio_in(nvic, STM32_EXTI0_IRQ));
    sysbus_connect_irq(exti_busdev, 1, qdev_get_gpio_in(nvic, STM32_EXTI1_IRQ));
    sysbus_connect_irq(exti_busdev, 2, qdev_get_gpio_in(nvic, STM32_EXTI2_IRQ));
    sysbus_connect_irq(exti_busdev, 3, qdev_get_gpio_in(nvic, STM32_EXTI3_IRQ));
    sysbus_connect_irq(exti_busdev, 4, qdev_get_gpio_in(nvic, STM32_EXTI4_IRQ));
    sysbus_connect_irq(exti_busdev, 5, qdev_get_gpio_in(nvic, STM32_EXTI9_5_IRQ));
    sysbus_connect_irq(exti_busdev, 6, qdev_get_gpio_in(nvic, STM32_EXTI15_10_IRQ));
    sysbus_connect_irq(exti_busdev, 7, qdev_get_gpio_in(nvic, STM32_PVD_IRQ));
    sysbus_connect_irq(exti_busdev, 8, qdev_get_gpio_in(nvic, STM32_RTCAlarm_IRQ));
    sysbus_connect_irq(exti_busdev, 9, qdev_get_gpio_in(nvic, STM32_OTG_FS_WKUP_IRQ));
    sysbus_connect_irq(exti_busdev, 10, qdev_get_gpio_in(nvic, STM32_ETH_WKUP_IRQ));
    sysbus_connect_irq(exti_busdev, 11, qdev_get_gpio_in(nvic, STM32_OTG_FS_WKUP_IRQ));
    sysbus_connect_irq(exti_busdev, 12, qdev_get_gpio_in(nvic, STM32_TAMP_STAMP_IRQ));
    sysbus_connect_irq(exti_busdev, 13, qdev_get_gpio_in(nvic, STM32_RTC_WKUP_IRQ));

    DeviceState *syscfg_dev = qdev_create(NULL, "stm32f2xx_syscfg");
    qdev_prop_set_ptr(syscfg_dev, "stm32_rcc", rcc_dev);
    qdev_prop_set_ptr(syscfg_dev, "stm32_exti", exti_dev);
    qdev_prop_set_bit(syscfg_dev, "boot0", 0);
    qdev_prop_set_bit(syscfg_dev, "boot1", 0);
    stm32_init_periph(syscfg_dev, STM32_SYSCFG, 0x40013800, NULL);

    struct {
        uint32_t addr;
        uint8_t irq_idx;
    } const uart_desc[] = {
        {0x40011000, STM32_UART1_IRQ},
        {0x40004400, STM32_UART2_IRQ},
        {0x40004800, STM32_UART3_IRQ},
        {0x40004C00, STM32_UART4_IRQ},
        {0x40005000, STM32_UART5_IRQ},
        {0x40011400, STM32_UART6_IRQ},
    };
    for (i = 0; i < ARRAY_LENGTH(uart_desc); ++i) {
        assert(i < STM32F2XX_UART_COUNT);
        const stm32_periph_t periph = STM32_UART1 + i;
        DeviceState *uart_dev = qdev_create(NULL, "stm32-uart");
        uart_dev->id = stm32f2xx_periph_name_arr[periph];
        qdev_prop_set_int32(uart_dev, "periph", periph);
       qdev_prop_set_ptr(uart_dev, "stm32_rcc", rcc_dev);
//        qdev_prop_set_ptr(uart_dev, "stm32_gpio", gpio_dev);
//        qdev_prop_set_ptr(uart_dev, "stm32_afio", afio_dev);
//        qdev_prop_set_ptr(uart_dev, "stm32_check_tx_pin_callback", (void *)stm32_afio_uart_check_tx_pin_callback);
        stm32_init_periph(uart_dev, periph, uart_desc[i].addr,
          qdev_get_gpio_in(nvic, uart_desc[i].irq_idx));
        stm32_uart[i] = (Stm32Uart *)uart_dev;
    }


    /* SPI */
    struct {
        uint32_t addr;
        uint8_t irq_idx;
    } const spi_desc[] = {
        {0x40013000, STM32_SPI1_IRQ},
        {0x40003800, STM32_SPI2_IRQ},
        {0x40003CD0, STM32_SPI3_IRQ},
    };
    for (i = 0; i < ARRAY_LENGTH(spi_desc); ++i) {
        const stm32_periph_t periph = STM32_SPI1 + i;
        stm->spi_dev[i] = qdev_create(NULL, "stm32f2xx_spi");
        stm->spi_dev[i]->id = stm32f2xx_periph_name_arr[periph];
        qdev_prop_set_int32(stm->spi_dev[i], "periph", periph);
        stm32_init_periph(stm->spi_dev[i], periph, spi_desc[i].addr,
          qdev_get_gpio_in(nvic, spi_desc[i].irq_idx));

    }

//    stm32_uart[STM32_UART1_INDEX] = stm32_create_uart_dev(STM32_UART1, rcc_dev, gpio_dev, afio_dev, 0x40011000, qdev_get_gpio_in(nvic, STM32_UART1_IRQ]);
//    stm32_uart[STM32_UART2_INDEX] = stm32_create_uart_dev(STM32_UART2, rcc_dev, gpio_dev, afio_dev, 0x40004400, qdev_get_gpio_in(nvic, STM32_UART2_IRQ]);
//    stm32_uart[STM32_UART3_INDEX] = stm32_create_uart_dev(STM32_UART3, rcc_dev, gpio_dev, afio_dev, 0x40004800, qdev_get_gpio_in(nvic, STM32_UART3_IRQ]);
//    stm32_uart[STM32_UART4_INDEX] = stm32_create_uart_dev(STM32_UART4, rcc_dev, gpio_dev, afio_dev, 0x40004C00, qdev_get_gpio_in(nvic, STM32_UART4_IRQ]);
//    stm32_uart[STM32_UART5_INDEX] = stm32_create_uart_dev(STM32_UART5, rcc_dev, gpio_dev, afio_dev, 0x40005000, qdev_get_gpio_in(nvic, STM32_UART5_IRQ]);
//    stm32_uart[STM32_UART6_INDEX] = stm32_create_uart_dev(STM32_UART6, rcc_dev, gpio_dev, afio_dev, 0x40011400, qdev_get_gpio_in(nvic, STM32_UART6_IRQ]);
    DeviceState *adc_dev = qdev_create(NULL, "stm32f2xx_adc");
    stm32_init_periph(adc_dev, STM32_ADC1, 0x40012000, NULL);

    /* RTC real time clock */
    DeviceState *rtc_dev = qdev_create(NULL, "f2xx_rtc");
    *stm32_rtc = rtc_dev;
    stm32_init_periph(rtc_dev, STM32_RTC, 0x40002800, NULL);
    // Alarm A
    sysbus_connect_irq(SYS_BUS_DEVICE(rtc_dev), 0, qdev_get_gpio_in(exti_dev, 17));
    // Alarm B
    sysbus_connect_irq(SYS_BUS_DEVICE(rtc_dev), 1, qdev_get_gpio_in(exti_dev, 17));
    // Wake up timer
    sysbus_connect_irq(SYS_BUS_DEVICE(rtc_dev), 2, qdev_get_gpio_in(exti_dev, 22));

    /* Power management */
    DeviceState *pwr_dev = qdev_create(NULL, "f2xx_pwr");
    stm32_init_periph(pwr_dev, STM32_RTC, 0x40007000, NULL);
    qdev_prop_set_ptr((*cpu)->env.nvic, "stm32_pwr", pwr_dev);


#define dummy_dev(name, start, size) do {\
    DeviceState *dummy = qdev_create(NULL, "f2xx_dummy"); \
    qdev_prop_set_ptr(dummy, "name", (void *)name); \
    qdev_prop_set_int32(dummy, "size", size); \
    qdev_init_nofail(dummy); \
    sysbus_mmio_map(SYS_BUS_DEVICE(dummy), 0, start); \
} while (0)

    /* Timers */
    struct {
        uint8_t timer_num;
        uint32_t addr;
        uint8_t irq_idx;
    } const timer_desc[] = {
        {2, 0x40000000, STM32_TIM2_IRQ},
        {3, 0x40000400, STM32_TIM3_IRQ},
        {4, 0x40000800, STM32_TIM4_IRQ},
        {5, 0x40000C00, STM32_TIM5_IRQ},
        {6, 0x40001000, STM32_TIM6_IRQ},
        {7, 0x40001400, STM32_TIM7_IRQ},
        {9, 0x40014000, STM32_TIM1_BRK_TIM9_IRQ},
        {10, 0x40014400, STM32_TIM1_UP_TIM10_IRQ},
        {11, 0x40014800, STM32_TIM1_TRG_COM_TIM11_IRQ},
        {12, 0x40001800, STM32_TIM8_BRK_TIM12_IRQ},
        {13, 0x40001C00, STM32_TIM8_UP_TIM13_IRQ},
        {14, 0x40002000, STM32_TIM8_TRG_COMM_TIM14_IRQ},
    };
    for (i = 0; i < ARRAY_LENGTH(timer_desc); ++i) {
        assert(i < STM32F2XX_TIM_COUNT);
        const stm32_periph_t periph = STM32_TIM1 + timer_desc[i].timer_num - 1;

        DeviceState *timer = qdev_create(NULL, "f2xx_tim");
        timer->id = stm32f2xx_periph_name_arr[periph];
        stm32_init_periph(timer, periph, timer_desc[i].addr, qdev_get_gpio_in(nvic, timer_desc[i].irq_idx));
        stm32_timer[timer_desc[i].timer_num - 1] = (Stm32Timer *)timer;
    }

    dummy_dev("Reserved",  0x40002400, 0x400);
    //
    dummy_dev("WWDG",      0x40002C00, 0x400);
    dummy_dev("IWDG",      0x40003000, 0x400);
    dummy_dev("Reserved",  0x40003400, 0x400);
    //
    //
    dummy_dev("Reserved",  0x40004000, 0x400);
    //
    //
    //
    //

    DeviceState *i2c1 = qdev_create(NULL, "f2xx_i2c");
    i2c1->id = stm32f2xx_periph_name_arr[STM32_I2C1];
    qdev_prop_set_int32(i2c1, "periph", STM32_I2C1);
    stm32_init_periph(i2c1, STM32_I2C1, 0x40005400, qdev_get_gpio_in(nvic, STM32_I2C1_EV_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(i2c1), 1, qdev_get_gpio_in(nvic, STM32_I2C1_ER_IRQ));

    DeviceState *i2c2 = qdev_create(NULL, "f2xx_i2c");
    i2c2->id = stm32f2xx_periph_name_arr[STM32_I2C2];
    qdev_prop_set_int32(i2c2, "periph", STM32_I2C2);
    stm32_init_periph(i2c2, STM32_I2C2, 0x40005800, qdev_get_gpio_in(nvic, STM32_I2C2_EV_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(i2c2), 1, qdev_get_gpio_in(nvic, STM32_I2C2_ER_IRQ));

    DeviceState *i2c3 = qdev_create(NULL, "f2xx_i2c");
    i2c3->id = stm32f2xx_periph_name_arr[STM32_I2C3];
    qdev_prop_set_int32(i2c3, "periph", STM32_I2C3);
    stm32_init_periph(i2c3, STM32_I2C2, 0x40005C00, qdev_get_gpio_in(nvic, STM32_I2C3_EV_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(i2c3), 1, qdev_get_gpio_in(nvic, STM32_I2C3_ER_IRQ));

    dummy_dev("Reserved",  0x40006000, 0x400);
    dummy_dev("BxCAN1",    0x40006400, 0x400);
    dummy_dev("BxCAN2",    0x40006800, 0x400);
    dummy_dev("Reserved",  0x40006C00, 0x400);
    // PWR
    dummy_dev("DAC1/DAC2", 0x40007400, 0x400);
    dummy_dev("Reserved",  0x40007800, 0x400);
    dummy_dev("Reserved",  0x40008000, 0x8000);
    dummy_dev("TIM1/PWM1", 0x40010000, 0x400);
    dummy_dev("TIM8/PWM2", 0x40010400, 0x400);
    // USART1
    // USART6
    dummy_dev("Reserved",  0x40011800, 0x800);
    // ADC1 - ADC2 - ADC3
    // skipped reserved from here on
    dummy_dev("SDIO",      0x40012C00, 0x400);
    // SPI1
    // SYSCFG needed

    DeviceState *crc = qdev_create(NULL, "f2xx_crc");
    stm32_init_periph(crc, STM32_CRC, 0x40023000, NULL);
    
    DeviceState *dma1 = qdev_create(NULL, "f2xx_dma");
    stm32_init_periph(dma1, STM32_DMA1, 0x40026000, NULL);
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 0, qdev_get_gpio_in(nvic, STM32_DMA1_STREAM0_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 1, qdev_get_gpio_in(nvic, STM32_DMA1_STREAM1_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 2, qdev_get_gpio_in(nvic, STM32_DMA1_STREAM2_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 3, qdev_get_gpio_in(nvic, STM32_DMA1_STREAM3_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 4, qdev_get_gpio_in(nvic, STM32_DMA1_STREAM4_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 5, qdev_get_gpio_in(nvic, STM32_DMA1_STREAM5_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 6, qdev_get_gpio_in(nvic, STM32_DMA1_STREAM6_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 7, qdev_get_gpio_in(nvic, STM32_DMA1_STREAM7_IRQ));

    DeviceState *dma2 = qdev_create(NULL, "f2xx_dma");
    stm32_init_periph(dma2, STM32_DMA2, 0x40026400, NULL);
    sysbus_connect_irq(SYS_BUS_DEVICE(dma2), 0, qdev_get_gpio_in(nvic, STM32_DMA2_STREAM0_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma2), 1, qdev_get_gpio_in(nvic, STM32_DMA2_STREAM1_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma2), 2, qdev_get_gpio_in(nvic, STM32_DMA2_STREAM2_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma2), 3, qdev_get_gpio_in(nvic, STM32_DMA2_STREAM3_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma2), 4, qdev_get_gpio_in(nvic, STM32_DMA2_STREAM4_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma2), 5, qdev_get_gpio_in(nvic, STM32_DMA2_STREAM5_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma2), 6, qdev_get_gpio_in(nvic, STM32_DMA2_STREAM6_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma2), 7, qdev_get_gpio_in(nvic, STM32_DMA2_STREAM7_IRQ));
}
