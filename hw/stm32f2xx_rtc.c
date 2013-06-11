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
#include "sysbus.h"
#include "qemu/timer.h"

#define R_RTC_TR     (0x00 / 4)
#define R_RTC_DR     (0x04 / 4)
#define R_RTC_CR     (0x08 / 4)
#define R_RTC_ISR    (0x0c / 4)
#define R_RTC_ISR_RESET 0x00000007
#define R_RTC_PRER   (0x10 / 4)
#define R_RTC_PRER_RESET 0x007f00ff
#define R_RTC_WUTR   (0x14 / 4)
#define R_RTC_WUTR_RESET 0x0000ffff
#define R_RTC_CALIBR (0x18 / 4)
#define R_RTC_ALRMAR (0x1c / 4)
#define R_RTC_ALRMBR (0x20 / 4)
#define R_RTC_WPR    (0x24 / 4)
#define R_RTC_TSTR   (0x30 / 4)
#define R_RTC_TSDR   (0x34 / 4)
#define R_RTC_TAFCR  (0x40 / 4)
#define R_RTC_BKPxR  (0x50 / 4)
#define R_RTC_BKPxR_LAST (0x9c / 4)
#define R_RTC_MAX    (0xa0 / 4)

typedef struct f2xx_rtc {
    SysBusDevice busdev;
    MemoryRegion iomem;
    uint32_t regs[R_RTC_MAX];
    int wp_count; /* Number of correct writes to WP reg */
} f2xx_rtc;

static uint64_t
f2xx_rtc_read(void *arg, hwaddr addr, unsigned int size)
{
    f2xx_rtc *s = arg;
    uint32_t r;
    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= R_RTC_MAX) {
        printf("guest error; unimpl\n");
        return 0;
    }
    r = (s->regs[addr] >> offset * 8) & ((1ull << (8 * size)) - 1);
//    if (addr < R_RTC_BKPxR) {
//        printf("%s: reg 0x%x offset %x ret 0x%x\n", __func__, (int)addr << 2, offset, r);
//    }
    return r;
}

static void
f2xx_rtc_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    f2xx_rtc *s = arg;
    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= R_RTC_MAX) {
        printf("unimpl\n");
        return;
    }

    /* Special case for write protect state machine. */
    if (addr == R_RTC_WPR) {
        if (offset > 0) {
            return;
        }
        data &= 0xff;
        if ((s->wp_count == 0 && data == 0xca) ||
          (s->wp_count == 1 && data == 0x53)) {
            s->wp_count++;
        } else {
            s->wp_count = 0;
        }
        s->regs[addr] = data;
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
        printf("unimpl\n");
        return;
    }
    if (addr >= R_RTC_BKPxR && addr <= R_RTC_BKPxR_LAST) {
        s->regs[addr] = data;
        return;
    }
    /* Write protect */
    if (s->wp_count < 2 && addr != R_RTC_TAFCR && addr != R_RTC_ISR &&
      addr != R_RTC_WPR) {
        printf("write to RTC reg 0x%x w/o write protect disable first\n",
          (int)addr << 2);
        return;
    }
    switch(addr) {
    case R_RTC_TR:
        break;
    case R_RTC_CR:
        break;
    case R_RTC_ISR:
        break;
    case R_RTC_PRER:
        break;
    default:
        printf("%s: reg 0x%x %d write %d\n", __func__, (int)addr << 2, offset, size);
    }
    s->regs[addr] = data;

}

static const MemoryRegionOps f2xx_rtc_ops = {
    .read = f2xx_rtc_read,
    .write = f2xx_rtc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    }
};

static void
f2xx_rtc_set_from_host(f2xx_rtc *s)
{
    struct tm now;
    uint8_t wday;

    qemu_get_timedate(&now, 0);
    wday = now.tm_wday == 0 ? now.tm_wday : 7;
    s->regs[R_RTC_TR] = to_bcd(now.tm_sec) |
                        to_bcd(now.tm_min) << 8 |
                        to_bcd(now.tm_hour) << 16;
    s->regs[R_RTC_DR] = to_bcd(now.tm_mday) |
                        to_bcd(now.tm_mon + 1) << 8 |
                        wday << 13 |
                        to_bcd(now.tm_year % 100) << 16;
    printf("set to 0x%x 0x%x", s->regs[R_RTC_TR], s->regs[R_RTC_DR]);
    printf(" %d %d %d\n", now.tm_mday, now.tm_mon, now.tm_year);
}

static int
f2xx_rtc_init(SysBusDevice *dev)
{
    f2xx_rtc *s = FROM_SYSBUS(f2xx_rtc, dev);

    memory_region_init_io(&s->iomem, &f2xx_rtc_ops, s, "rtc", 0xa0);
    sysbus_init_mmio(dev, &s->iomem);
    f2xx_rtc_set_from_host(s);
    s->regs[R_RTC_ISR] = R_RTC_ISR_RESET;
    s->regs[R_RTC_PRER] = R_RTC_PRER_RESET;
    s->regs[R_RTC_WUTR] = R_RTC_WUTR_RESET;
    return 0;
}

static Property f2xx_rtc_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void
f2xx_rtc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *sc = SYS_BUS_DEVICE_CLASS(klass);
    sc->init = f2xx_rtc_init;
    dc->no_user = 1;
    dc->props = f2xx_rtc_properties;
}

static const TypeInfo
f2xx_rtc_info = {
    .name          = "f2xx_rtc",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(f2xx_rtc),
    .class_init    = f2xx_rtc_class_init,
};

static void
f2xx_rtc_register_types(void)
{
    type_register_static(&f2xx_rtc_info);
}

type_init(f2xx_rtc_register_types)
