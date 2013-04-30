/*-
 * Copyright (c) 2013
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/*
 * QEMU model of the stm32f2xx GPIO module
 */

#include "sysbus.h"
#include "stm32.h"

#define R_GPIO_MODER   (0x00 / 4)
#define R_GPIO_OTYPER  (0x04 / 4)
#define R_GPIO_OSPEEDR (0x08 / 4)
#define R_GPIO_PUPDR   (0x0c / 4)
#define R_GPIO_IDR     (0x10 / 4)
#define R_GPIO_ODR     (0x14 / 4)
#define R_GPIO_BSRR    (0x18 / 4)
#define R_GPIO_LCKR    (0x1c / 4)
#define R_GPIO_AFRL    (0x20 / 4)
#define R_GPIO_AFRH    (0x24 / 4)
#define R_GPIO_MAX     (0x28 / 4)

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;

    stm32_periph_t periph;

    uint32_t regs[R_GPIO_MAX];
    uint32_t ccr;
} stm32f2xx_gpio;

static uint64_t
stm32f2xx_gpio_read(void *arg, hwaddr offset, unsigned int size)
{
    stm32f2xx_gpio *s = arg;
    uint32_t r;

    offset >>= 2;
    r = s->regs[offset];
printf("unit %d reg %x return 0x%x\n", s->periph, (int)offset << 2, r);
    return r;
}

static void
stm32f2xx_gpio_write(void *arg, hwaddr offset, uint64_t data, unsigned int size)
{
    stm32f2xx_gpio *s = arg;

    offset >>= 2;
    switch (offset) {
    default:
printf("unit %d reg %x write 0x%x\n", s->periph, (int)offset << 2, (int)data);
        if (offset < R_GPIO_MAX) {
            s->regs[offset] = data;
        }
        break;
    }
}

static const MemoryRegionOps stm32f2xx_gpio_ops = {
    .read = stm32f2xx_gpio_read,
    .write = stm32f2xx_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4, /* XXX actually 1 */
        .max_access_size = 4
    }
};

static void
stm32f2xx_gpio_reset(DeviceState *dev)
{
    stm32f2xx_gpio *s = FROM_SYSBUS(stm32f2xx_gpio, SYS_BUS_DEVICE(dev));

    switch (s->periph) {
    case 0:
        s->regs[R_GPIO_MODER] = 0xa8000000;
        s->regs[R_GPIO_OSPEEDR] = 0x00000000;
        s->regs[R_GPIO_PUPDR] = 0x64000000;
        break;
    case 1:
        s->regs[R_GPIO_MODER] = 0x00000280;
        s->regs[R_GPIO_OSPEEDR] = 0x000000C0;
        s->regs[R_GPIO_PUPDR] = 0x00000100;
        break;
    default:
        s->regs[R_GPIO_MODER] = 0x00000000;
        s->regs[R_GPIO_OSPEEDR] = 0x00000000;
        s->regs[R_GPIO_PUPDR] = 0x00000000;
        break;
    }
}

static int
stm32f2xx_gpio_init(SysBusDevice *dev)
{
    stm32f2xx_gpio *s = FROM_SYSBUS(stm32f2xx_gpio, dev);

    memory_region_init_io(&s->iomem, &stm32f2xx_gpio_ops, s, "gpio", 0x400);
    sysbus_init_mmio(dev, &s->iomem);

    return 0;
}

static Property stm32f2xx_gpio_properties[] = {
    DEFINE_PROP_INT32("periph", stm32f2xx_gpio, periph, -1),
    DEFINE_PROP_END_OF_LIST()
};

static void
stm32f2xx_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *sc = SYS_BUS_DEVICE_CLASS(klass);

    sc->init = stm32f2xx_gpio_init;
    dc->reset = stm32f2xx_gpio_reset;
    dc->props = stm32f2xx_gpio_properties;
}

static const TypeInfo stm32f2xx_gpio_info = {
    .name = "stm32f2xx_gpio",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(stm32f2xx_gpio),
    .class_init = stm32f2xx_gpio_class_init
};

static void
stm32f2xx_gpio_register_types(void)
{
    type_register_static(&stm32f2xx_gpio_info);
}

type_init(stm32f2xx_gpio_register_types)
