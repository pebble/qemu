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
 * QEMU model of the stm32f2xx SPI controller.
 */

#include "hw/sysbus.h"
#include "hw/arm/stm32.h"
#include "hw/ssi.h"

#define	R_CR1      (0x00 / 4)
#define	R_CR2      (0x04 / 4)

#define	R_SR       (0x08 / 4)
#define	R_SR_RESET    0x0002
#define	R_SR_MASK     0x01FF
#define R_SR_OVR     (1 << 6)
#define R_SR_TXE     (1 << 1)
#define R_SR_RXNE    (1 << 0)

#define	R_DR       (0x0C / 4)
#define	R_CRCPR    (0x10 / 4)
#define	R_CRCPR_RESET 0x0007
#define	R_RXCRCR   (0x14 / 4)
#define	R_TXCRCR   (0x18 / 4)
#define	R_I2SCFGR  (0x1C / 4)
#define	R_I2SPR    (0x20 / 4)
#define	R_I2SPR_RESET 0x0002
#define R_MAX      (0x24 / 4)

typedef struct stm32f2xx_spi_s {
    SysBusDevice busdev;
    MemoryRegion iomem;
    qemu_irq irq;

    SSIBus *spi;

    stm32_periph_t periph;

    int32_t rx;
    int rx_full; 
    uint16_t regs[R_MAX];
} Stm32Spi;

static uint64_t
stm32f2xx_spi_read(void *arg, hwaddr offset, unsigned size)
{
    Stm32Spi *s = arg;
    uint16_t r = UINT16_MAX;

    if (!(size == 2 || size == 4 || (offset & 0x3) != 0)) {
        STM32_BAD_REG(offset, size);
    }
    offset >>= 2;
    if (offset < R_MAX) {
        r = s->regs[offset];
    } else {
        stm32_hw_warn("Out of range SPI write, offset 0x%x", (unsigned)offset<<2);
    }
    switch (offset) {
    case R_DR:
        s->regs[R_SR] &= ~R_SR_RXNE;
    }
    return r;
}


static void
stm32f2xx_spi_write(void *arg, hwaddr offset, uint64_t data, unsigned size)
{
    struct stm32f2xx_spi_s *s = (struct stm32f2xx_spi_s *)arg;

    if (size != 2 && size != 4) {
        STM32_BAD_REG(offset, size);
    }
    /* SPI registers are all at most 16 bits wide */
    data &= 0xFFFFF;
    offset >>= 2;

    switch (offset) {
    case R_DR:
        s->regs[R_SR] &= ~R_SR_TXE;
        if (s->regs[R_SR] & R_SR_RXNE) {
            s->regs[R_SR] |= R_SR_OVR;
        }
        s->regs[R_DR] = ssi_transfer(s->spi, data);
        s->regs[R_SR] |= R_SR_RXNE;
        s->regs[R_SR] |= R_SR_TXE;
        break;
    default:
        if (offset < ARRAY_SIZE(s->regs)) {
            s->regs[offset] = data;
        } else {
            STM32_BAD_REG(offset, WORD_ACCESS_SIZE);
        }
    }
}

static const MemoryRegionOps stm32f2xx_spi_ops = {
    .read = stm32f2xx_spi_read,
    .write = stm32f2xx_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void
stm32f2xx_spi_reset(DeviceState *dev)
{
    struct stm32f2xx_spi_s *s = FROM_SYSBUS(struct stm32f2xx_spi_s,
      SYS_BUS_DEVICE(dev));

    s->regs[R_SR] = R_SR_RESET;
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
stm32f2xx_spi_init(SysBusDevice *dev)
{
    struct stm32f2xx_spi_s *s = FROM_SYSBUS(struct stm32f2xx_spi_s, dev);

    memory_region_init_io(&s->iomem, &stm32f2xx_spi_ops, s, "spi", 0x3ff);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->irq);
    s->spi = ssi_create_bus(&dev->qdev, "ssi");

    return 0;
}


static Property stm32f2xx_spi_properties[] = {
    DEFINE_PROP_INT32("periph", struct stm32f2xx_spi_s, periph, -1),
    DEFINE_PROP_END_OF_LIST()
};

static void
stm32f2xx_spi_class_init(ObjectClass *c, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(c);
    SysBusDeviceClass *sc = SYS_BUS_DEVICE_CLASS(c);

    sc->init = stm32f2xx_spi_init;
    dc->reset = stm32f2xx_spi_reset;
    dc->props = stm32f2xx_spi_properties;
}

static const TypeInfo stm32f2xx_spi_info = {
    .name = "stm32f2xx_spi",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(struct stm32f2xx_spi_s),
    .class_init = stm32f2xx_spi_class_init
};

static void
stm32f2xx_spi_register_types(void)
{
    type_register_static(&stm32f2xx_spi_info);
}

type_init(stm32f2xx_spi_register_types)

/*
 *
 */
/*
 * Serial peripheral interface (SPI) RM0033 section 25
 */

