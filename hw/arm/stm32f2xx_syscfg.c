/*
 * STM32F2XX Microcontroller SYSCFG (System Configuration Controller) module
 *
 * Copyright (C) 2013 Martijn The
 *
 * Implementation based on ST Microelectronics "RM0033 Reference Manual Rev 4"
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

#include "stm32f2xx.h"



/* DEFINITIONS */

#define SYSCFG_MEMRMP_OFFSET 0x00
#define SYSCFG_MEMRMP_MEM_MODE_MASK 0x3

#define SYSCFG_PMC_OFFSET 0x04

#define SYSCFG_EXTICR1_OFFSET 0x08
#define SYSCFG_EXTICR2_OFFSET 0x0c
#define SYSCFG_EXTICR3_OFFSET 0x10
#define SYSCFG_EXTICR4_OFFSET 0x14

#define SYSCFG_EXTICR_COUNT 4
#define SYSCFG_EXTI_PER_CR 4

#define SYSCFG_CMPCR_OFFSET 0x20

typedef struct {
    /* Inherited */
    SysBusDevice busdev;

    /* Properties */
    void *stm32_rcc_prop;
    void *stm32_exti_prop;
    uint32_t boot_pins; //!< BOOT0 and BOOT1 pins

    /* Private */
    MemoryRegion iomem;

    Stm32Rcc *stm32_rcc;
    Stm32Exti *stm32_exti;

    uint32_t
        USART1_REMAP,
        USART2_REMAP,
        USART3_REMAP,
        SYSCFG_MEMRMP,
        SYSCFG_EXTICR[SYSCFG_EXTICR_COUNT];
} Stm32Syscfg;


/* REGISTER IMPLEMENTATION */

static uint32_t stm32_syscfg_SYSCFG_MEMRMP_read(Stm32Syscfg *s)
{
    return s->SYSCFG_MEMRMP;
}

static void stm32_syscfg_SYSCFG_MEMRMP_write(Stm32Syscfg *s, uint32_t new_value,
                                        bool init)
{
    if (init) {
        // "After reset these bits take the value selected by the BOOT pins."
        s->SYSCFG_MEMRMP = (SYSCFG_MEMRMP_MEM_MODE_MASK & s->boot_pins);
    } else {
        s->SYSCFG_MEMRMP = new_value;
    }
}

/* Write the External Interrupt Configuration Register.
 * There are four of these registers, each of which configures
 * four EXTI interrupt lines.  Each line is represented by four bits, which
 * indicate which GPIO the line is connected to.  When the register is
 * written, the changes are propagated to the EXTI module.
 */
static void stm32_syscfg_SYSCFG_EXTICR_write(Stm32Syscfg *s, unsigned index,
                                            uint32_t new_value, bool init)
{
    int i;
    unsigned exti_line;
    unsigned start;
    unsigned old_gpio_index, new_gpio_index;

    assert(index < SYSCFG_EXTICR_COUNT);

    /* Loop through the four EXTI lines controlled by this register. */
    for(i = 0; i < SYSCFG_EXTI_PER_CR; i++) {
        /* For each line, notify the EXTI module if it has changed. */
        exti_line = (index * SYSCFG_EXTI_PER_CR) + i;
        start = i * 4;

        new_gpio_index = (new_value >> start) & 0xf;
        if(!init) {
            old_gpio_index = (s->SYSCFG_EXTICR[index] >> start) & 0xf;
            if (old_gpio_index == new_gpio_index) {
                continue;
            }
            stm32_exti_reset_gpio(s->stm32_exti, exti_line, old_gpio_index);
        }
        stm32_exti_set_gpio(s->stm32_exti, exti_line, new_gpio_index);
    }

    s->SYSCFG_EXTICR[index] = new_value;
}


static uint64_t stm32_syscfg_readw(Stm32Syscfg *s, hwaddr offset)
{
    switch (offset) {
        case SYSCFG_MEMRMP_OFFSET:
            return stm32_syscfg_SYSCFG_MEMRMP_read(s);
        case SYSCFG_PMC_OFFSET:
            STM32_NOT_IMPL_REG(SYSCFG_PMC_OFFSET, WORD_ACCESS_SIZE);
            return 0;
        case SYSCFG_EXTICR1_OFFSET:
            return s->SYSCFG_EXTICR[0];
        case SYSCFG_EXTICR2_OFFSET:
            return s->SYSCFG_EXTICR[1];
        case SYSCFG_EXTICR3_OFFSET:
            return s->SYSCFG_EXTICR[2];
        case SYSCFG_EXTICR4_OFFSET:
            return s->SYSCFG_EXTICR[3];
        case SYSCFG_CMPCR_OFFSET:
            STM32_NOT_IMPL_REG(SYSCFG_CMPCR_OFFSET, WORD_ACCESS_SIZE);
            return 0;
        default:
            STM32_BAD_REG(offset, WORD_ACCESS_SIZE);
            return 0;
    }
}

static void stm32_syscfg_writew(Stm32Syscfg *s, hwaddr offset,
                          uint64_t value)
{
    switch (offset) {
        case SYSCFG_MEMRMP_OFFSET:
            stm32_syscfg_SYSCFG_MEMRMP_write(s, value, false);
            break;
        case SYSCFG_PMC_OFFSET:
            STM32_NOT_IMPL_REG(SYSCFG_PMC_OFFSET, WORD_ACCESS_SIZE);
            break;
        case SYSCFG_EXTICR1_OFFSET:
            stm32_syscfg_SYSCFG_EXTICR_write(s, 0, value, false);
            break;
        case SYSCFG_EXTICR2_OFFSET:
            stm32_syscfg_SYSCFG_EXTICR_write(s, 1, value, false);
            break;
        case SYSCFG_EXTICR3_OFFSET:
            stm32_syscfg_SYSCFG_EXTICR_write(s, 2, value, false);
            break;
        case SYSCFG_EXTICR4_OFFSET:
            stm32_syscfg_SYSCFG_EXTICR_write(s, 3, value, false);
            break;
        case SYSCFG_CMPCR_OFFSET:
            STM32_NOT_IMPL_REG(SYSCFG_CMPCR_OFFSET, WORD_ACCESS_SIZE);
            break;
        default:
            STM32_BAD_REG(offset, WORD_ACCESS_SIZE);
            break;
    }
}

static uint64_t stm32_syscfg_read(void *opaque, hwaddr offset,
                          unsigned size)
{
    Stm32Syscfg *s = (Stm32Syscfg *)opaque;

    stm32_rcc_check_periph_clk((Stm32Rcc *)s->stm32_rcc, STM32_SYSCFG);

    switch(size) {
        case 4:
            return stm32_syscfg_readw(opaque, offset);
        default:
            STM32_BAD_REG(offset, size);
            return 0;
    }
}

static void stm32_syscfg_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    Stm32Syscfg *s = (Stm32Syscfg *)opaque;

    stm32_rcc_check_periph_clk((Stm32Rcc *)s->stm32_rcc, STM32_SYSCFG);

    switch(size) {
        case 4:
            stm32_syscfg_writew(opaque, offset, value);
            break;
        default:
            STM32_BAD_REG(offset, size);
            break;
    }
}

static const MemoryRegionOps stm32_syscfg_ops = {
    .read = stm32_syscfg_read,
    .write = stm32_syscfg_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void stm32_syscfg_reset(DeviceState *dev)
{
    Stm32Syscfg *s = FROM_SYSBUS(Stm32Syscfg, SYS_BUS_DEVICE(dev));

    stm32_syscfg_SYSCFG_MEMRMP_write(s, 0x00000000, true);
    stm32_syscfg_SYSCFG_EXTICR_write(s, 0, 0x00000000, true);
    stm32_syscfg_SYSCFG_EXTICR_write(s, 1, 0x00000000, true);
    stm32_syscfg_SYSCFG_EXTICR_write(s, 2, 0x00000000, true);
    stm32_syscfg_SYSCFG_EXTICR_write(s, 3, 0x00000000, true);
}



/* DEVICE INITIALIZATION */

static int stm32_syscfg_init(SysBusDevice *dev)
{
    Stm32Syscfg *s = FROM_SYSBUS(Stm32Syscfg, dev);

    s->stm32_rcc = (Stm32Rcc *)s->stm32_rcc_prop;
    s->stm32_exti = (Stm32Exti *)s->stm32_exti_prop;

    memory_region_init_io(&s->iomem, OBJECT(s), &stm32_syscfg_ops, s,
                          "syscfg", 0x03ff);
    sysbus_init_mmio(dev, &s->iomem);

    return 0;
}

static Property stm32_syscfg_properties[] = {
    DEFINE_PROP_PTR("stm32_rcc", Stm32Syscfg, stm32_rcc_prop),
    DEFINE_PROP_PTR("stm32_exti", Stm32Syscfg, stm32_exti_prop),
    DEFINE_PROP_BIT("boot0", Stm32Syscfg, boot_pins, 0, 0), // BOOT0 pin
    DEFINE_PROP_BIT("boot1", Stm32Syscfg, boot_pins, 1, 0), // BOOT1 pin
    DEFINE_PROP_END_OF_LIST()
};

static void stm32_syscfg_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = stm32_syscfg_init;
    dc->reset = stm32_syscfg_reset;
    dc->props = stm32_syscfg_properties;
}

static TypeInfo stm32_syscfg_info = {
    .name  = "stm32f2xx_syscfg",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32Syscfg),
    .class_init = stm32_syscfg_class_init
};

static void stm32_syscfg_register_types(void)
{
    type_register_static(&stm32_syscfg_info);
}

type_init(stm32_syscfg_register_types)
