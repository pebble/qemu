/*
 * Copyright (c) 2016
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
 * QEMU model of the stm32f7xx I2C controller.
 */

#include "hw/sysbus.h"
#include "hw/arm/stm32.h"
#include "hw/i2c/i2c.h"

#define R_I2C_CR1       (0x00 / 4)
#define R_I2C_CR2       (0x04 / 4)
#define R_I2C_OAR1      (0x08 / 4)
#define R_I2C_OAR2      (0x0c / 4)
#define R_I2C_TIMINGR   (0x10 / 4)
#define R_I2C_TIMEOUTR  (0x14 / 4)
#define R_I2C_ISR       (0x18 / 4)
#define R_I2C_ICR       (0x1C / 4)
#define R_I2C_PECR      (0x20 / 4)
#define R_I2C_RXDR      (0x24 / 4)
#define R_I2C_TXDR      (0x28 / 4)
#define R_I2C_MAX       (0x28 / 4)

#define R_I2C_CR1_PE_BIT (1 << 0)

#define R_I2C_CR2_START_BIT (1 << 13)

#define R_I2C_ISR_BERR_BIT (1 << 8)


#ifdef DEBUG_STM32_I2C
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("STM32F7XX_I2c: " fmt , ## __VA_ARGS__); \
        usleep(1000); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

static const char *stm32f7xx_i2c_reg_name_arr[] = {
    "CR1",
    "CR2",
    "OAR1",
    "OAR2",
    "TIMINGR",
    "TIMEOUTR",
    "ISR",
    "ICR",
    "PECR",
    "RXDR",
    "TXDR",
};



typedef struct stm32f7xx_i2c {
    SysBusDevice busdev;
    MemoryRegion iomem;
    qemu_irq evt_irq;
    qemu_irq err_irq;

    I2CBus *bus;

    stm32_periph_t periph;

    int32_t rx;
    int rx_full;
    uint16_t regs[R_I2C_MAX];

} stm32f7xx_i2c;


/* Routine which updates the I2C's IRQs.  This should be called whenever
 * an interrupt-related flag is updated.
 */
static void stm32f7xx_i2c_update_irq(stm32f7xx_i2c *s) {
    int new_err_irq_level = (s->regs[R_I2C_ISR] & R_I2C_ISR_BERR_BIT);

    int new_evt_irq_level = 0;

    DPRINTF("%s %s: setting evt_irq to %d\n", __func__, s->busdev.parent_obj.id,
              !!new_evt_irq_level);
    qemu_set_irq(s->evt_irq, !!new_evt_irq_level);

    DPRINTF("%s %s: setting err_irq to %d\n", __func__, s->busdev.parent_obj.id,
              !!new_err_irq_level);
    qemu_set_irq(s->err_irq, !!new_err_irq_level);
}



static uint64_t stm32f7xx_i2c_read(void *arg, hwaddr offset, unsigned size)
{
    stm32f7xx_i2c *s = arg;
    uint32_t r = UINT32_MAX;
    const char *reg_name = "UNKNOWN";

    if (!(size == 2 || size == 4 || (offset & 0x3) != 0)) {
        STM32_BAD_REG(offset, size);
    }
    offset >>= 2;
    if (offset < R_I2C_MAX) {
        r = s->regs[offset];
        reg_name = stm32f7xx_i2c_reg_name_arr[offset];
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "Out of range I2C write, offset 0x%x\n",
          (unsigned)offset << 2);
    }

    DPRINTF("%s %s:  register %s, result: 0x%x\n", __func__, s->busdev.parent_obj.id,
              reg_name, r);
    return r;
}


static void stm32f7xx_i2c_write(void *arg, hwaddr offset, uint64_t data, unsigned size)
{
    const char *reg_name = "UNKNOWN";
    struct stm32f7xx_i2c *s = (struct stm32f7xx_i2c *)arg;

    if (size != 2 && size != 4) {
        STM32_BAD_REG(offset, size);
    }
    /* I2C registers are all at most 32 bits wide */
    data &= 0xffffffff;
    offset >>= 2;

    if (offset < R_I2C_MAX) {
        reg_name = stm32f7xx_i2c_reg_name_arr[offset];
    }
    DPRINTF("%s %s: register %s, data: 0x%llx, size:%d\n", __func__, s->busdev.parent_obj.id,
            reg_name, data, size);


    switch (offset) {
    case R_I2C_CR1:
        if ((data & R_I2C_CR1_PE_BIT) == 0) {
            s->regs[R_I2C_ISR] = 0;
        }
        break;

    case R_I2C_CR2:
        if (data & R_I2C_CR2_START_BIT) {
            // Abort all attempted master transfers with a bus error
            s->regs[R_I2C_ISR] |= R_I2C_ISR_BERR_BIT;
        } else {
            s->regs[offset] = data;
        }
        break;

    case R_I2C_TXDR:
        i2c_send(s->bus, (uint8_t)data);
        break;

    case R_I2C_ICR:
        s->regs[R_I2C_ISR] &= ~data;
        break;

    case R_I2C_ISR:
    case R_I2C_RXDR:
        STM32_WARN_RO_REG(offset);
        break;

    case R_I2C_OAR1:
    case R_I2C_OAR2:
    case R_I2C_TIMINGR:
    case R_I2C_TIMEOUTR:
    case R_I2C_PECR:
        s->regs[offset] = data;
        break;

    default:
        STM32_BAD_REG(offset, size);
        break;
    }
    stm32f7xx_i2c_update_irq(s);
}

static const MemoryRegionOps stm32f7xx_i2c_ops = {
    .read = stm32f7xx_i2c_read,
    .write = stm32f7xx_i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void stm32f7xx_i2c_reset(DeviceState *dev)
{
}

static int stm32f7xx_i2c_init(SysBusDevice *dev)
{
    struct stm32f7xx_i2c *s = FROM_SYSBUS(struct stm32f7xx_i2c, dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &stm32f7xx_i2c_ops, s, "i2c", 0x3ff);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->evt_irq);
    sysbus_init_irq(dev, &s->err_irq);
    s->bus = i2c_init_bus(DEVICE(dev), "i2c");

    DPRINTF("%s %s: INITIALIZED\n", __func__, s->busdev.parent_obj.id);
    return 0;
}


static Property stm32f7xx_i2c_properties[] = {
    DEFINE_PROP_INT32("periph", struct stm32f7xx_i2c, periph, -1),
    DEFINE_PROP_END_OF_LIST()
};

static void stm32f7xx_i2c_class_init(ObjectClass *c, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(c);
    SysBusDeviceClass *sc = SYS_BUS_DEVICE_CLASS(c);

    sc->init = stm32f7xx_i2c_init;
    dc->reset = stm32f7xx_i2c_reset;
    dc->props = stm32f7xx_i2c_properties;
}

static const TypeInfo stm32f7xx_i2c_info = {
    .name = "stm32f7xx_i2c",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(struct stm32f7xx_i2c),
    .class_init = stm32f7xx_i2c_class_init
};

static void stm32f7xx_i2c_register_types(void)
{
    type_register_static(&stm32f7xx_i2c_info);
}

type_init(stm32f7xx_i2c_register_types)
