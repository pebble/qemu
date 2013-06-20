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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
/*
 * QEMU stm32f2xx RTC emulation
 */
#include "hw/sysbus.h"
#include "qemu/timer.h"

#define R_TIM_CR1    (0x00 / 4) //p
#define R_TIM_CR2    (0x04 / 4)
#define R_TIM_SMCR   (0x08 / 4)
#define R_TIM_DIER   (0x0c / 4)
#define R_TIM_SR     (0x10 / 4) //p
#define R_TIM_EGR    (0x14 / 4) //p
#define R_TIM_CCMR1  (0x18 / 4)
#define R_TIM_CCMR2  (0x1c / 4)
#define R_TIM_CCER   (0x20 / 4)
#define R_TIM_CNT    (0x24 / 4)
#define R_TIM_PSC    (0x28 / 4) //p
#define R_TIM_ARR    (0x2c / 4) //p
#define R_TIM_CCR1   (0x34 / 4)
#define R_TIM_CCR2   (0x38 / 4)
#define R_TIM_CCR3   (0x3c / 4)
#define R_TIM_CCR4   (0x40 / 4)
#define R_TIM_DCR    (0x48 / 4)
#define R_TIM_DMAR   (0x4c / 4)
#define R_TIM_OR     (0x50 / 4)
#define R_TIM_MAX    (0x54 / 4)

typedef struct f2xx_tim {
    SysBusDevice busdev;
    MemoryRegion iomem;
    QEMUTimer *timer;
    time_t t;
    uint32_t regs[R_TIM_MAX];
} f2xx_tim;

static uint32_t
f2xx_period(f2xx_tim *s)
{
#if 0
    uint32_t prer = s->regs[R_RTC_PRER];
    unsigned int prescale;

    prescale = (((prer >> R_RTC_PRER_PREDIV_A_SHIFT) & R_RTC_PRER_PREDIV_A_MASK) + 1) *
               (((prer >> R_RTC_PRER_PREDIV_S_SHIFT) & R_RTC_PRER_PREDIV_S_MASK) + 1);
    return 1000000000LL * prescale / 32768;
#else
    return 1000000000LL;
#endif
}

static void
f2xx_tim_timer(void *arg)
{
    f2xx_tim *s = arg;

    s->t++;
    qemu_mod_timer(s->timer, qemu_get_clock_ns(vm_clock) + f2xx_period(s));
}

static uint64_t
f2xx_tim_read(void *arg, hwaddr addr, unsigned int size)
{
    f2xx_tim *s = arg;
    uint32_t r;
    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= R_TIM_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid read f2xx rtc register 0x%x\n",
          (unsigned int)addr << 2);
        return 0;
    }
    r = (s->regs[addr] >> offset * 8) & ((1ull << (8 * size)) - 1);
    return r;
}

static void
f2xx_tim_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    f2xx_tim *s = arg;
    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= R_TIM_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid write f2xx rtc register 0x%x\n",
          (unsigned int)addr << 2);
        return;
    }

    switch(size) {
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

    switch(addr) {
    case R_TIM_CR1:
        break;
    case R_TIM_SR:
        break;
    case R_TIM_EGR:
        break;
    case R_TIM_PSC:
        break;
    case R_TIM_ARR:
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "f2xx rtc unimplemented write 0x%x+%u size %u val %u\n",
          (unsigned int)addr << 2, offset, size, (unsigned int)data);
    }
    s->regs[addr] = data;

}

static const MemoryRegionOps f2xx_tim_ops = {
    .read = f2xx_tim_read,
    .write = f2xx_tim_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    }
};

static int
f2xx_tim_init(SysBusDevice *dev)
{
    f2xx_tim *s = FROM_SYSBUS(f2xx_tim, dev);

    memory_region_init_io(&s->iomem, &f2xx_tim_ops, s, "rtc", 0xa0);
    sysbus_init_mmio(dev, &s->iomem);
    //s->regs[R_RTC_ISR] = R_RTC_ISR_RESET;
    ////s->regs[R_RTC_PRER] = R_RTC_PRER_RESET;
    //s->regs[R_RTC_WUTR] = R_RTC_WUTR_RESET;
    s->timer = qemu_new_timer_ns(vm_clock, f2xx_tim_timer, s);
    qemu_mod_timer(s->timer, qemu_get_clock_ns(vm_clock) + f2xx_period(s));
    return 0;
}

static Property f2xx_tim_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void
f2xx_tim_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *sc = SYS_BUS_DEVICE_CLASS(klass);
    sc->init = f2xx_tim_init;
    dc->no_user = 1;
    dc->props = f2xx_tim_properties;
}

static const TypeInfo
f2xx_tim_info = {
    .name          = "f2xx_tim",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(f2xx_tim),
    .class_init    = f2xx_tim_class_init,
};

static void
f2xx_tim_register_types(void)
{
    type_register_static(&f2xx_tim_info);
}

type_init(f2xx_tim_register_types)
