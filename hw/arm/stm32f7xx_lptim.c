/*
 * Source code based on stm32f2xx_tim.c
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
/*
 * QEMU stm32f7xx LPTIM emulation
 */
#include "hw/sysbus.h"
#include "hw/arm/stm32.h"
#include "qemu/timer.h"

//#define DEBUG_STM32F7XX_LPTIM
#ifdef DEBUG_STM32F7XX_LPTIM
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...) \
    do { \
        printf("DEBUG_STM32F7XX_LPTIM %s: " fmt , __func__, ## __VA_ARGS__); \
        usleep(100); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif


#define R_LPTIM_ISR   (0x00 / 4)
#define R_LPTIM_ICR   (0x04 / 4)
#define R_LPTIM_IER   (0x08 / 4)
#define R_LPTIM_CFGR  (0x0C / 4)
#define R_LPTIM_CR    (0x10 / 4)
#define R_LPTIM_CMP   (0x14 / 4)
#define R_LPTIM_ARR   (0x18 / 4)
#define R_LPTIM_CNT   (0x1C / 4)
#define R_LPTIM_OR    (0x20 / 4)
#define R_LPTIM_MAX   (0x24 / 4)

static const char *f7xx_lptim_reg_names[] = {
    ENUM_STRING(R_LPTIM_ISR),
    ENUM_STRING(R_LPTIM_ICR),
    ENUM_STRING(R_LPTIM_IER),
    ENUM_STRING(R_LPTIM_CFGR),
    ENUM_STRING(R_LPTIM_CR),
    ENUM_STRING(R_LPTIM_CMP),
    ENUM_STRING(R_LPTIM_ARR),
    ENUM_STRING(R_LPTIM_CNT),
    ENUM_STRING(R_LPTIM_OR),
};


typedef struct f7xx_lptim {
    SysBusDevice busdev;
    MemoryRegion iomem;
    QEMUTimer *timer;
    qemu_irq irq;
    uint32_t regs[R_LPTIM_MAX];

    qemu_irq pwm_ratio_changed;
    qemu_irq pwm_enable;
} f7xx_lptim;

static uint32_t f7xx_lptim_period(f7xx_lptim *s) {
    /* FIXME: hard coded to 32kHz */
    return 31250;
}

static int64_t f7xx_lptim_next_transition(f7xx_lptim *s, int64_t current_time) {
    return current_time + f7xx_lptim_period(s) * s->regs[R_LPTIM_ARR];
}

static void f7xx_lptim_timer(void *arg) {
    f7xx_lptim *s = arg;
    if (s->regs[R_LPTIM_CR] & 1) {
        timer_mod(s->timer, f7xx_lptim_next_transition(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)));
    }
    qemu_set_irq(s->irq, 1);
}

static uint64_t f7xx_lptim_read(void *arg, hwaddr addr, unsigned int size) {
    f7xx_lptim *s = arg;
    uint32_t r = 0;
    int offset = addr & 0x3;
    addr >>= 2;
    if (addr >= R_LPTIM_MAX) {
        DPRINTF("invalid read register 0x%x\n", (unsigned int)addr << 2);
        qemu_log_mask(LOG_GUEST_ERROR, "f7xx lptim invalid read register 0x%x\n",
                      (unsigned int)addr << 2);
        return 0;
    }

    switch (addr) {
        case R_LPTIM_CFGR:
        case R_LPTIM_CMP:
        case R_LPTIM_ARR:
        case R_LPTIM_CNT:
        case R_LPTIM_CR:
            r = (s->regs[addr] >> offset * 8) & ((1ull << (8 * size)) - 1);
            break;
        default:
            DPRINTF("unimplemented read 0x%x+%u size %u\n", (unsigned int)addr << 2, offset, size);
            qemu_log_mask(LOG_UNIMP, "f7xx lptim unimplemented read 0x%x+%u size %u\n",
                          (unsigned int)addr << 2, offset, size);
            break;
    }

    DPRINTF("reg: %s, size: %d, value: 0x%x\n", f7xx_lptim_reg_names[addr], size, r);
    return r;
}

static void f7xx_lptim_write(void *arg, hwaddr addr, uint64_t data, unsigned int size) {
    f7xx_lptim *s = arg;
    int offset = addr & 0x3;
    addr >>= 2;
    DPRINTF("reg:%s, size: %d, value: 0x%llx\n", f7xx_lptim_reg_names[addr], size, data);
    if (addr >= R_LPTIM_MAX) {
        DPRINTF("invalid write register 0x%x\n", (unsigned int)addr << 2);
        qemu_log_mask(LOG_GUEST_ERROR, "f7xx lptim invalid write register 0x%x\n",
                      (unsigned int)addr << 2);
        return;
    }

    switch (size) {
        case 1:
            data = (s->regs[addr] & ~(0xff << (offset * 8))) | data << (offset * 8);
            break;
        case 2:
            data = (s->regs[addr] & ~(0xffff << (offset * 8))) | data << (offset * 8);
            break;
        case 4:
            break;
        default:
            abort();
    }

    switch (addr) {
        case R_LPTIM_CR:
            if ((s->regs[addr] & 1) == 0 && data & 1) {
                DPRINTF("started\n");
                timer_mod(s->timer, f7xx_lptim_next_transition(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)));
                qemu_set_irq(s->pwm_enable, 1);
            } else if (s->regs[addr] & 1 && (data & 1) == 0) {
                DPRINTF("stopped\n");
                timer_del(s->timer);
                qemu_set_irq(s->pwm_enable, 0);
            }
            s->regs[addr] = data;
            break;
        case R_LPTIM_CMP:
        {
            s->regs[addr] = data;
            uint32_t ratio = (s->regs[R_LPTIM_CMP] * 255) / s->regs[R_LPTIM_ARR];
            DPRINTF("Setting PWM ratio to %d (0x%x, 0x%x)\n", ratio, s->regs[R_LPTIM_CMP], s->regs[R_LPTIM_ARR]);
            qemu_set_irq(s->pwm_ratio_changed, ratio);
            break;
        }
        case R_LPTIM_CFGR:
        case R_LPTIM_ARR:
        case R_LPTIM_CNT:
            s->regs[addr] = data;
            break;
        default:
            DPRINTF("unimplemented write 0x%x+%u size %u val 0x%x\n", (unsigned int)addr << 2,
                    offset, size, (unsigned int)data);
            qemu_log_mask(LOG_UNIMP, "f7xx lptim unimplemented write 0x%x+%u size %u val 0x%x\n",
                          (unsigned int)addr << 2, offset, size, (unsigned int)data);
            break;
    }
}

static const MemoryRegionOps f7xx_lptim_ops = {
    .read = f7xx_lptim_read,
    .write = f7xx_lptim_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

static void f7xx_lptim_reset(DeviceState *dev) {
    f7xx_lptim *s = FROM_SYSBUS(f7xx_lptim, SYS_BUS_DEVICE(dev));

    timer_del(s->timer);
    memset(&s->regs, 0, sizeof(s->regs));
}


static int f7xx_lptim_init(SysBusDevice *dev) {
    f7xx_lptim *s = FROM_SYSBUS(f7xx_lptim, dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &f7xx_lptim_ops, s, "lptim", 0xa0);
    sysbus_init_mmio(dev, &s->iomem);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, f7xx_lptim_timer, s);
    sysbus_init_irq(dev, &s->irq);

    qdev_init_gpio_out_named(DEVICE(dev), &s->pwm_ratio_changed, "pwm_ratio_changed", 1);
    qdev_init_gpio_out_named(DEVICE(dev), &s->pwm_enable, "pwm_enable", 1);

    return 0;
}

static Property f7xx_lptim_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void f7xx_lptim_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *sc = SYS_BUS_DEVICE_CLASS(klass);
    sc->init = f7xx_lptim_init;
    //TODO: fix this: dc->no_user = 1;
    dc->props = f7xx_lptim_properties;
    dc->reset = f7xx_lptim_reset;
}

static const TypeInfo f7xx_lptim_info = {
    .name          = "f7xx_lptim",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(f7xx_lptim),
    .class_init    = f7xx_lptim_class_init,
};

static void f7xx_lptim_register_types(void) {
    type_register_static(&f7xx_lptim_info);
}

type_init(f7xx_lptim_register_types)
