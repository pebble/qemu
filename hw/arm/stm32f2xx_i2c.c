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


#define R_I2C_CR1_PE_BIT          0x00001
#define R_I2C_CR1_SMBUS_BIT       0x00002
#define R_I2C_CR1_SMBTYPE_BIT     0x00008
#define R_I2C_CR1_ENARB_BIT       0x00010
#define R_I2C_CR1_ENPEC_BIT       0x00020
#define R_I2C_CR1_ENGC_BIT        0x00040
#define R_I2C_CR1_NOSTRETCH_BIT   0x00080
#define R_I2C_CR1_START_BIT       0x00100
#define R_I2C_CR1_STOP_BIT        0x00200
#define R_I2C_CR1_ACK_BIT         0x00400
#define R_I2C_CR1_POS_BIT         0x00800
#define R_I2C_CR1_PEC_BIT         0x01000
#define R_I2C_CR1_ALERT_BIT       0x02000
#define R_I2C_CR1_SWRTS_BIT       0x08000


#define R_I2C_CR2_ITERREN_BIT     0x00100
#define R_I2C_CR2_ITEVTEN_BIT     0x00200
#define R_I2C_CR2_ITBUFEN_BIT     0x00400
#define R_I2C_CR2_DMAEN_BIT       0x00800
#define R_I2C_CR2_LAST_BIT        0x01000

#define R_I2C_SR1_SB_BIT          0x00001
#define R_I2C_SR1_ADDR_BIT        0x00002
#define R_I2C_SR1_BTF_BIT         0x00004
#define R_I2C_SR1_ADD10_BIT       0x00008
#define R_I2C_SR1_STOPF_BIT       0x00010
#define R_I2C_SR1_RxNE_BIT        0x00040
#define R_I2C_SR1_TxE_BIT         0x00080
#define R_I2C_SR1_BERR_BIT        0x00100
#define R_I2C_SR1_ARLO_BIT        0x00200
#define R_I2C_SR1_AF_BIT          0x00400
#define R_I2C_SR1_OVR_BIT         0x00800
#define R_I2C_SR1_PECERR_BIT      0x01000
#define R_I2C_SR1_TIMEOUT_BIT     0x04000
#define R_I2C_SR1_SMBALERT_BIT    0x08000



//#define DEBUG_STM32F2XX_I2c
#ifdef DEBUG_STM32F2XX_I2c
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("STM32F2XX_I2c: " fmt , ## __VA_ARGS__); \
         usleep(1000); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

static const char *f2xx_i2c_reg_name_arr[] = {
    "CR1",
    "CR2",
    "OAR1",
    "OAR2",
    "DR",
    "SR1",
    "SR2",
    "CCR",
    "TRISE"
};



typedef struct f2xx_i2c {
    SysBusDevice busdev;
    MemoryRegion iomem;
    qemu_irq evt_irq;
    qemu_irq err_irq;

    I2CBus *bus;

    stm32_periph_t periph;

    int32_t rx;
    int rx_full; 
    uint16_t regs[R_I2C_MAX];

} f2xx_i2c;


/* Routine which updates the I2C's IRQs.  This should be called whenever
 * an interrupt-related flag is updated.
 */
static void f2xx_i2c_update_irq(f2xx_i2c *s) {
    int new_err_irq_level = 0;
    if (s->regs[R_I2C_CR2] & R_I2C_CR2_ITERREN_BIT) {
        new_err_irq_level =  (s->regs[R_I2C_SR1]  & R_I2C_SR1_BERR_BIT)
                           | (s->regs[R_I2C_SR1]  & R_I2C_SR1_ARLO_BIT)
                           | (s->regs[R_I2C_SR1]  & R_I2C_SR1_AF_BIT)
                           | (s->regs[R_I2C_SR1]  & R_I2C_SR1_OVR_BIT)
                           | (s->regs[R_I2C_SR1]  & R_I2C_SR1_PECERR_BIT)
                           | (s->regs[R_I2C_SR1]  & R_I2C_SR1_TIMEOUT_BIT)
                           | (s->regs[R_I2C_SR1]  & R_I2C_SR1_SMBALERT_BIT);
    }

    int new_evt_irq_level = 0;
    if (s->regs[R_I2C_CR2] & R_I2C_CR2_ITEVTEN_BIT) {
        new_evt_irq_level =  (s->regs[R_I2C_SR1]  & R_I2C_SR1_SB_BIT)
                           | (s->regs[R_I2C_SR1]  & R_I2C_SR1_ADDR_BIT)
                           | (s->regs[R_I2C_SR1]  & R_I2C_SR1_ADD10_BIT)
                           | (s->regs[R_I2C_SR1]  & R_I2C_SR1_STOPF_BIT)
                           | (s->regs[R_I2C_SR1]  & R_I2C_SR1_BTF_BIT);

        if (s->regs[R_I2C_CR2] & R_I2C_CR2_ITBUFEN_BIT) {
            new_evt_irq_level |= (s->regs[R_I2C_SR1]  & R_I2C_SR1_TxE_BIT)
                               | (s->regs[R_I2C_SR1]  & R_I2C_SR1_RxNE_BIT);
        }
    }

    DPRINTF("%s %s: setting evt_irq to %d\n", __func__, s->busdev.parent_obj.id,
              !!new_evt_irq_level);
    qemu_set_irq(s->evt_irq, !!new_evt_irq_level);

    DPRINTF("%s %s: setting err_irq to %d\n", __func__, s->busdev.parent_obj.id,
              !!new_err_irq_level);
    qemu_set_irq(s->err_irq, !!new_err_irq_level);
}



static uint64_t
f2xx_i2c_read(void *arg, hwaddr offset, unsigned size)
{
    f2xx_i2c *s = arg;
    uint16_t r = UINT16_MAX;
    const char *reg_name = "UNKNOWN";

    if (!(size == 2 || size == 4 || (offset & 0x3) != 0)) {
        STM32_BAD_REG(offset, size);
    }
    offset >>= 2;
    if (offset < R_I2C_MAX) {
        r = s->regs[offset];
        reg_name = f2xx_i2c_reg_name_arr[offset];
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "Out of range I2C write, offset 0x%x\n",
          (unsigned)offset << 2);
    }

    DPRINTF("%s %s:  register %s, result: 0x%x\n", __func__, s->busdev.parent_obj.id,
              reg_name, r);
    return r;
}


static void
f2xx_i2c_write(void *arg, hwaddr offset, uint64_t data, unsigned size)
{
    const char *reg_name = "UNKNOWN";
    struct f2xx_i2c *s = (struct f2xx_i2c *)arg;

    if (size != 2 && size != 4) {
        STM32_BAD_REG(offset, size);
    }
    /* I2C registers are all at most 16 bits wide */
    data &= 0xFFFFF;
    offset >>= 2;

    if (offset < R_I2C_MAX) {
        reg_name = f2xx_i2c_reg_name_arr[offset];
    }
    DPRINTF("%s %s: register %s, data: 0x%llx, size:%d\n", __func__, s->busdev.parent_obj.id,
            reg_name, data, size);


    switch (offset) {
    case R_I2C_CR1:
        s->regs[offset] = data;
        if (data & R_I2C_CR1_START_BIT) {
            // For now, abort all attempted master transfers with a bus error
            s->regs[R_I2C_SR1] |= R_I2C_SR1_BERR_BIT;
        }
        if ((data & R_I2C_CR1_PE_BIT) == 0) {
            s->regs[R_I2C_SR1] = 0;
        }
        break;

    case R_I2C_DR:
        i2c_send(s->bus, (uint8_t)data);
        break;

    default:
        if (offset < ARRAY_SIZE(s->regs)) {
            s->regs[offset] = data;
        } else {
            STM32_BAD_REG(offset, WORD_ACCESS_SIZE);
        }
    }
    f2xx_i2c_update_irq(s);
}

static const MemoryRegionOps f2xx_i2c_ops = {
    .read = f2xx_i2c_read,
    .write = f2xx_i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void
f2xx_i2c_reset(DeviceState *dev)
{
    //struct f2xx_i2c *s = FROM_SYSBUS(struct f2xx_i2c, SYS_BUS_DEVICE(dev));

//    s->regs[R_SR] = R_SR_RESET;
}

static int
f2xx_i2c_init(SysBusDevice *dev)
{
    struct f2xx_i2c *s = FROM_SYSBUS(struct f2xx_i2c, dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &f2xx_i2c_ops, s, "i2c", 0x3ff);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->evt_irq);
    sysbus_init_irq(dev, &s->err_irq);
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
