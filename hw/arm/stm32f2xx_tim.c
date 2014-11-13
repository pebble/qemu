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
 * QEMU stm32f2xx TIM emulation
 */
#include "hw/sysbus.h"
#include "qemu/timer.h"

#define R_TIM_CR1    (0x00 / 4) //p
#define R_TIM_CR2    (0x04 / 4)
#define R_TIM_SMCR   (0x08 / 4)
#define R_TIM_DIER   (0x0c / 4) //p
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
    qemu_irq irq;
    uint32_t regs[R_TIM_MAX];
} f2xx_tim;

static uint32_t
f2xx_tim_period(f2xx_tim *s)
{
    /* FIXME: hard coded to 32kHz */
    return 31250;
}

static int64_t
f2xx_tim_next_transition(f2xx_tim *s, int64_t current_time)
{
    if (s->regs[R_TIM_CR1] & 0x70) {
        qemu_log_mask(LOG_UNIMP, "f2xx tim, only upedge-aligned mode supported\n");
        return -1;
    }
    return current_time + f2xx_tim_period(s) * s->regs[R_TIM_ARR];
}

static void
f2xx_tim_timer(void *arg)
{
    f2xx_tim *s = arg;

    if (s->regs[R_TIM_CR1] & 1) {
        timer_mod(s->timer, f2xx_tim_next_transition(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)));
    }
    if (!(s->regs[R_TIM_SR] & 1))
        //printf("f2xx tim timer expired, setting int\n");
    s->regs[R_TIM_SR] |= 1;
    qemu_set_irq(s->irq, 1);
}

static uint64_t
f2xx_tim_read(void *arg, hwaddr addr, unsigned int size)
{
    f2xx_tim *s = arg;
    uint32_t r;
    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= R_TIM_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "f2xx tim invalid read register 0x%x\n",
          (unsigned int)addr << 2);
        return 0;
    }
    r = (s->regs[addr] >> offset * 8) & ((1ull << (8 * size)) - 1);
    switch (addr) {
    case R_TIM_CR1:
    case R_TIM_DIER:
    case R_TIM_SR:
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "f2xx tim unimplemented read 0x%x+%u size %u val 0x%x\n",
          (unsigned int)addr << 2, offset, size, (unsigned int)r);
    }
    return r;
}

static void
f2xx_tim_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    f2xx_tim *s = arg;
    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= R_TIM_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "f2xx tim invalid write register 0x%x\n",
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
        if (data & ~1) {
            qemu_log_mask(LOG_UNIMP, "f2xx tim non-zero CR1 unimplemented\n");
        }
        if ((s->regs[addr] & 1) == 0 && data & 1) {
            //printf("f2xx tim started\n");
            timer_mod(s->timer, f2xx_tim_next_transition(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)));
        } else if (s->regs[addr] & 1 && (data & 1) == 0) {
            timer_del(s->timer);
        }
        s->regs[addr] = data;
        break;
    case R_TIM_SR:
        if (s->regs[addr] & 1 && (data & 1) == 0) {
            //printf("f2xx tim clearing int\n");
            qemu_set_irq(s->irq, 0);
        }
        s->regs[addr] &= data;
        break;
    case R_TIM_EGR:
        qemu_log_mask(LOG_UNIMP, "f2xx tim unimplemented write EGR+%u size %u val 0x%x\n",
          offset, size, (unsigned int)data);
        break;
    case R_TIM_DIER:
    case R_TIM_PSC:
    case R_TIM_ARR:
        s->regs[addr] = data;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "f2xx tim unimplemented write 0x%x+%u size %u val 0x%x\n",
          (unsigned int)addr << 2, offset, size, (unsigned int)data);
    }

}

static const MemoryRegionOps f2xx_tim_ops = {
    .read = f2xx_tim_read,
    .write = f2xx_tim_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

static int
f2xx_tim_init(SysBusDevice *dev)
{
    f2xx_tim *s = FROM_SYSBUS(f2xx_tim, dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &f2xx_tim_ops, s, "tim", 0xa0);
    sysbus_init_mmio(dev, &s->iomem);
    //s->regs[R_RTC_ISR] = R_RTC_ISR_RESET;
    ////s->regs[R_RTC_PRER] = R_RTC_PRER_RESET;
    //s->regs[R_RTC_WUTR] = R_RTC_WUTR_RESET;
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, f2xx_tim_timer, s);
    sysbus_init_irq(dev, &s->irq);
    
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
    //TODO: fix this: dc->no_user = 1;
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
