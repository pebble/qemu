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
#include <sys/time.h>
#include "hw/sysbus.h"
#include "qemu/timer.h"

//#define DEBUG_STM32F2XX_RTC
#ifdef DEBUG_STM32F2XX_RTC
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("DEBUG_STM32F2XX_RTC: " fmt , ## __VA_ARGS__); \
         usleep(1000); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif



#define R_RTC_TR     (0x00 / 4)
#define R_RTC_DR     (0x04 / 4)
#define R_RTC_CR     (0x08 / 4)
#define R_RTC_ISR    (0x0c / 4)
#define R_RTC_ISR_RESET 0x00000007
#define R_RTC_ISR_RSF 0x00000020
#define R_RTC_PRER   (0x10 / 4)
#define R_RTC_PRER_PREDIV_A_MASK 0x7f
#define R_RTC_PRER_PREDIV_A_SHIFT 16
#define R_RTC_PRER_PREDIV_S_MASK 0x1fff
#define R_RTC_PRER_PREDIV_S_SHIFT 0
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

#define R_RTC_CR_FMT_MASK (0x01 << 6)

#define DEBUG_ALARM(x...)

typedef struct f2xx_rtc {
    SysBusDevice busdev;
    MemoryRegion iomem;
    QEMUTimer *timer;
    qemu_irq irq[2];
    // target_us = host_us + host_to_target_offset_us
    int64_t    host_to_target_offset_us;
    uint32_t regs[R_RTC_MAX];
    int wp_count; /* Number of correct writes to WP reg */
} f2xx_rtc;


static uint64_t
f2xx_period(f2xx_rtc *s)
{
    uint32_t prer = s->regs[R_RTC_PRER];
    unsigned int prescale;

    prescale = (((prer >> R_RTC_PRER_PREDIV_A_SHIFT) & R_RTC_PRER_PREDIV_A_MASK) + 1) *
               (((prer >> R_RTC_PRER_PREDIV_S_SHIFT) & R_RTC_PRER_PREDIV_S_MASK) + 1);
    uint64_t result = 1000000000LL * prescale / 32768;

    //DPRINTF("%s: period = %lldns\n", __func__, result);
    return result;
}


static void
f2xx_rtc_set_tdr(f2xx_rtc *s, struct tm *tm)
{
    uint8_t wday;

    wday = tm->tm_wday == 0 ? tm->tm_wday : 7;
    s->regs[R_RTC_TR] = to_bcd(tm->tm_sec) |
                        to_bcd(tm->tm_min) << 8 |
                        to_bcd(tm->tm_hour) << 16;
    s->regs[R_RTC_DR] = to_bcd(tm->tm_mday) |
                        to_bcd(tm->tm_mon + 1) << 8 |
                        wday << 13 |
                        to_bcd(tm->tm_year % 100) << 16;
}


// Set the time and date registers in the RTC from the host's date and time, shifted by the
// shift amount we determined that the firmware is using (host_to_target_offset).
// The rtc_period_ns passed in is the number of nanoseconds in host time that corresponds to 1
// "tick" (i.e. a second increment in the RTC register). Usually, you would expect this to
// be 1e9, but on Pebble plastic for example, the firmware sets up the RTC so that the
// seconds increment 1024 times/sec.
static void
f2xx_rtc_update_tdr_from_host(f2xx_rtc *s, uint64_t rtc_period_ns)
{
    struct tm now;

    // Get the host time in microseconds
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t host_time_us = tv.tv_sec * 1000000LL + (tv.tv_usec);

    // Compute the target time by adding the offset
    int64_t target_time_us = host_time_us + s->host_to_target_offset_us;

    // Convert to target ticks according period set in the RTC
    time_t target_time_sec = (target_time_us * 1000) / rtc_period_ns;
    
    // Convert to date, hour, min, sec components
    gmtime_r(&target_time_sec, &now);


#ifdef DEBUG_STM32F2XX_RTC
    char new_date_time[256];
    strftime(new_date_time, sizeof(new_date_time), "%c", &now);
    //DPRINTF("%s: setting new date & time to: %s\n", __func__, new_date_time);
#endif
    f2xx_rtc_set_tdr(s, &now);
}


// Fill in a tm struct from the contents of the time and date registers in the RTC
static void
f2xx_rtc_get_tm_struct(f2xx_rtc *s, struct tm *target_tm)
{
    memset(target_tm, 0, sizeof(*target_tm));

    // Fill in the target_tm from the contents of the time register and date registers
    uint32_t time_reg = s->regs[R_RTC_TR];

    target_tm->tm_hour = from_bcd((time_reg & 0x003F0000) >> 16);
    bool pm = (time_reg & 0x00800000) != 0;
    if (pm && (s->regs[R_RTC_CR] & R_RTC_CR_FMT_MASK)) {
        target_tm->tm_hour += 12;
    }
    target_tm->tm_min = from_bcd((time_reg & 0x00007F00) >> 8);
    target_tm->tm_sec = from_bcd(time_reg & 0x0000007F);

    uint32_t date_reg = s->regs[R_RTC_DR];
    target_tm->tm_mday = from_bcd(date_reg & 0x3f);
    target_tm->tm_mon = from_bcd((date_reg & 0x00001F00) >> 8) - 1;
    target_tm->tm_year = from_bcd((date_reg & 0x00FF0000) >> 16) + 100;

    // Have mktime fill in the remaining fields
    mktime(target_tm);
}



// Update the host to target offset by comparing the time set on the target vs. the host's
// current time
static void
f2xx_rtc_update_host_to_target_offset(f2xx_rtc *s, int64_t period_ns)
{
    struct tm target_tm;

    // Get the target ticks
    f2xx_rtc_get_tm_struct(s, &target_tm);
    time_t target_time_ticks = mktimegm(&target_tm);

    // Compute target time in microseconds
    int64_t target_time_us = target_time_ticks * period_ns / 1000;

    // Get the host time in microseconds
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t host_time_us = tv.tv_sec * 1000000LL + (tv.tv_usec);

    // Get the host to target offset in micro seconds
    s->host_to_target_offset_us = target_time_us - host_time_us;

#ifdef DEBUG_STM32F2XX_RTC
    char new_date_time[256];
    strftime(new_date_time, sizeof(new_date_time), "%c", &target_tm);
    DPRINTF("%s: set target date/time to %s, host_to_target_offset now %lld us\n",
              __func__, new_date_time, s->host_to_target_offset_us);
#endif
}

static bool
f2xx_alarm_match(f2xx_rtc *s, uint32_t alarm_reg)
{
    uint32_t tr = s->regs[R_RTC_TR];

    if ((alarm_reg & (1<<7)) == 0 && (tr & 0x7f) != (alarm_reg & 0x7f)) {
        /* Seconds match requested, but do not match. */
        return false;
    }
    if ((alarm_reg & (1<<15)) == 0 && (tr & 0x7f00) != (alarm_reg & 0x7f00)) {
        /* Minutes match requested, but do not match. */
        return false;
    }
    if ((alarm_reg & (1<<23)) == 0 && (tr & 0x7f0000) != (alarm_reg & 0x7f0000)) {
        /* Hours match requested, but do not match. */
        return false;
    }
    if ((alarm_reg & (1<<31)) == 0) { /* Day match. */
        uint32_t dr = s->regs[R_RTC_DR];
        if (alarm_reg & (1<<30)) { /* Day is week day. */
            if (((alarm_reg>>24) & 0xf) != ((dr>>13) & 0xf)) {
                return false;
            }
        } else { /* Day is day of month. */
            if (((alarm_reg>>24) & 0x3f) != (dr & 0x3f)) {
                return false;
            }
        }
    }
    return true;
}

static void
f2xx_alarm_check(f2xx_rtc *s, int unit)
{
    uint32_t cr = s->regs[R_RTC_CR];
    uint32_t isr = s->regs[R_RTC_ISR];

#if 0 
    if ((cr & 1<<(8 + unit)) == 0) {
        return; /* Not enabled. */
    }
#endif
    if ((isr & 1<<(8 + unit)) == 0) {
        if (f2xx_alarm_match(s, s->regs[R_RTC_ALRMAR + unit])) {
            isr |= 1<<(8 + unit);
            s->regs[R_RTC_ISR] = isr;
            DEBUG_ALARM("f2xx rtc alarm activated 0x%x 0x%x\n", isr, cr);
        }
    }
    qemu_set_irq(s->irq[unit], cr & 1<<(12 + unit) && isr & 1<<(8 + unit));
}

static uint64_t
f2xx_rtc_read(void *arg, hwaddr addr, unsigned int size)
{
    f2xx_rtc *s = arg;
    uint32_t r;
    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= R_RTC_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid read f2xx rtc register 0x%x\n",
          (unsigned int)addr << 2);
        DPRINTF("  %s: result: 0\n", __func__);
        return 0;
    }

    uint32_t value = s->regs[addr];
    if (addr == R_RTC_ISR) {
        value |= R_RTC_ISR_RSF;
    }

    r = (value >> offset * 8) & ((1ull << (8 * size)) - 1);

#ifdef DEBUG_STM32F2XX_RTC
    // Debug printing
    if (addr == R_RTC_TR || addr == R_RTC_DR) {
        struct tm target_tm;
        f2xx_rtc_get_tm_struct(s, &target_tm);

        char date_time_str[256];
        strftime(date_time_str, sizeof(date_time_str), "%c", &target_tm);
        //DPRINTF("%s: reading date/time: %s\n", __func__, date_time_str);
    } else {
        //DPRINTF("%s: addr: 0x%llx, size: %d, value: 0x%x\n", __func__, addr, size, r);
    }
#endif

    return r;
}

static void
f2xx_rtc_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    f2xx_rtc *s = arg;
    int offset = addr & 0x3;
    bool    compute_new_target_offset = false;

    DPRINTF("%s: addr: 0x%llx, data: 0x%llx, size: %d\n", __func__, addr, data, size);

    addr >>= 2;
    if (addr >= R_RTC_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid write f2xx rtc register 0x%x\n",
          (unsigned int)addr << 2);
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
        abort();
    }
    if (addr >= R_RTC_BKPxR && addr <= R_RTC_BKPxR_LAST) {
        s->regs[addr] = data;
        return;
    }
    /* Write protect */
    if (s->wp_count < 2 && addr != R_RTC_TAFCR && addr != R_RTC_ISR &&
      addr != R_RTC_WPR) {
        qemu_log_mask(LOG_GUEST_ERROR, "f2xx rtc write reg 0x%x+%u without wp disable\n",
          (unsigned int)addr << 2, offset);
        return;
    }
    switch(addr) {
    case R_RTC_TR:
    case R_RTC_DR:
        compute_new_target_offset = true;
        break;
    case R_RTC_CR:
        break;
    case R_RTC_ISR:
        if ((data & 1<<8) == 0 && (s->regs[R_RTC_ISR] & 1<<8) != 0) {
            DEBUG_ALARM("f2xx rtc isr lowered\n");
            qemu_irq_lower(s->irq[0]);
        }
        break;
    case R_RTC_PRER:
        /*
         * XXX currently updates upon next clock tick.  To do this properly we
         * would need to account for the time already elapsed, and then update
         * the timer for the remaining period.
         */
        break;
    case R_RTC_ALRMAR:
    case R_RTC_ALRMBR:
        break;
    case R_RTC_TAFCR:
        if (data) {
            qemu_log_mask(LOG_UNIMP,
              "f2xx rtc unimplemented write TAFCR+%u size %u val %u\n",
              offset, size, (unsigned int)data);
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "f2xx rtc unimplemented write 0x%x+%u size %u val 0x%x\n",
          (unsigned int)addr << 2, offset, size, (unsigned int)data);
    }
    s->regs[addr] = data;


    // Do we need to recompute the host to target offset?
    if (compute_new_target_offset) {
        f2xx_rtc_update_host_to_target_offset(s, f2xx_period(s));
    }

}


// This timer runs periodically in order to trigger alarms. For updating the time, we
// actually rely on calling f2xx_rtc_update_tdr_from_host() whenever the current time or
// date register is read from the MPU. 
static void
f2xx_timer(void *arg)
{
    f2xx_rtc *s = arg;
    uint64_t period_ns = f2xx_period(s);
    
    //DPRINTF("%s: executing, time_sec=%ld\n", __func__, s->time_sec);
    
    f2xx_rtc_update_tdr_from_host(s, period_ns);
    f2xx_alarm_check(s, 0);
    f2xx_alarm_check(s, 1);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_HOST) + period_ns);
}

static const MemoryRegionOps f2xx_rtc_ops = {
    .read = f2xx_rtc_read,
    .write = f2xx_rtc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

static int
f2xx_rtc_init(SysBusDevice *dev)
{
    f2xx_rtc *s = FROM_SYSBUS(f2xx_rtc, dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &f2xx_rtc_ops, s, "rtc", 0xa0);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->irq[0]);
    sysbus_init_irq(dev, &s->irq[1]);

    s->regs[R_RTC_ISR] = R_RTC_ISR_RESET;
    s->regs[R_RTC_PRER] = R_RTC_PRER_RESET;
    s->regs[R_RTC_WUTR] = R_RTC_WUTR_RESET;

    uint32_t period_ns = f2xx_period(s);
    DPRINTF("%s: period: %d ns", __func__, period_ns);

    // Init the time and date registers from the time on the host as the default
    s->host_to_target_offset_us = 0;
    struct tm now;
    qemu_get_timedate(&now, 0);
    f2xx_rtc_set_tdr(s, &now);
    f2xx_rtc_update_host_to_target_offset(s, period_ns);

    s->timer = timer_new_ns(QEMU_CLOCK_HOST, f2xx_timer, s);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_HOST) + period_ns);
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
    //TODO: fix this: dc->no_user = 1;
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
