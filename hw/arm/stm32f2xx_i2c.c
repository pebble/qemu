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
 * QEMU model of the stm32f2xx I2C controller.
 */

#include "hw/sysbus.h"
#include "hw/arm/stm32.h"
#include "hw/i2c/i2c.h"

#define	R_I2C_CR1      (0x00 / 4)
#define	R_I2C_CR2      (0x04 / 4)
#define R_I2C_OAR1     (0x08 / 4)
#define R_I2C_OAR2     (0x0c / 4)
#define R_I2C_DR       (0x10 / 4)
#define R_I2C_SR1      (0x14 / 4)
#define R_I2C_SR2      (0x18 / 4)
#define R_I2C_CCR      (0x1c / 4)
#define R_I2C_TRISE    (0x20 / 4)
#define R_I2C_MAX      (0x24 / 4)

typedef struct f2xx_i2c {
    SysBusDevice busdev;
    MemoryRegion iomem;
    qemu_irq irq;

    I2CBus *bus;

    stm32_periph_t periph;

    int32_t rx;
    int rx_full; 
    uint16_t regs[R_I2C_MAX];
} f2xx_i2c;

static uint64_t
f2xx_i2c_read(void *arg, hwaddr offset, unsigned size)
{
    f2xx_i2c *s = arg;
    uint16_t r = UINT16_MAX;

    if (!(size == 2 || size == 4 || (offset & 0x3) != 0)) {
        STM32_BAD_REG(offset, size);
    }
    offset >>= 2;
    if (offset < R_I2C_MAX) {
        r = s->regs[offset];
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "Out of range I2C write, offset 0x%x",
          (unsigned)offset << 2);
    }
    switch (offset) {
//    case R_DR:
//        s->regs[R_SR] &= ~R_SR_RXNE;
    }
    return r;
}


static void
f2xx_i2c_write(void *arg, hwaddr offset, uint64_t data, unsigned size)
{
    struct f2xx_i2c *s = (struct f2xx_i2c *)arg;

    if (size != 2 && size != 4) {
        STM32_BAD_REG(offset, size);
    }
    /* I2C registers are all at most 16 bits wide */
    data &= 0xFFFFF;
    offset >>= 2;

    switch (offset) {
    case R_I2C_DR:
//        if (s->regs[R_SR] & R_SR_RXNE) {
//            s->regs[R_SR] |= R_SR_OVR;
//        }
        i2c_send(s->bus, (uint8_t)data);
//        s->regs[R_SR] |= R_SR_RXNE;
//        stm32_hw_warn("SPI %d DR write %x", s->periph, (unsigned int)data);
        break;
    default:
        if (offset < ARRAY_SIZE(s->regs)) {
            s->regs[offset] = data;
        } else {
            STM32_BAD_REG(offset, WORD_ACCESS_SIZE);
        }
    }
}

static const MemoryRegionOps f2xx_i2c_ops = {
    .read = f2xx_i2c_read,
    .write = f2xx_i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void
f2xx_i2c_reset(DeviceState *dev)
{
    struct f2xx_i2c *s = FROM_SYSBUS(struct f2xx_i2c,
      SYS_BUS_DEVICE(dev));

//    s->regs[R_SR] = R_SR_RESET;
    switch (s->periph) {
    case 0:
        break;
    case 1:
        break;
    default:
        break;
    }
}

static int
f2xx_i2c_init(SysBusDevice *dev)
{
    struct f2xx_i2c *s = FROM_SYSBUS(struct f2xx_i2c, dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &f2xx_i2c_ops, s, "i2c", 0x3ff);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->irq);
    s->bus = i2c_init_bus(DEVICE(dev), "i2c");

    return 0;
}


static Property f2xx_i2c_properties[] = {
    DEFINE_PROP_INT32("periph", struct f2xx_i2c, periph, -1),
    DEFINE_PROP_END_OF_LIST()
};

static void
f2xx_i2c_class_init(ObjectClass *c, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(c);
    SysBusDeviceClass *sc = SYS_BUS_DEVICE_CLASS(c);

    sc->init = f2xx_i2c_init;
    dc->reset = f2xx_i2c_reset;
    dc->props = f2xx_i2c_properties;
}

static const TypeInfo f2xx_i2c_info = {
    .name = "f2xx_i2c",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(struct f2xx_i2c),
    .class_init = f2xx_i2c_class_init
};

static void
f2xx_i2c_register_types(void)
{
    type_register_static(&f2xx_i2c_info);
}

type_init(f2xx_i2c_register_types)
