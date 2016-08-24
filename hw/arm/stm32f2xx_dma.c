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
 * QEMU DMA controller device model
 */
#include "hw/sysbus.h"

//#define DEBUG_STM32F2XX_DMA
#ifdef DEBUG_STM32F2XX_DMA

// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("STM32F2XX_DMA: " fmt , ## __VA_ARGS__); \
         usleep(1000); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif


/* Common interrupt status / clear registers. */
#define R_DMA_LISR           (0x00 / 4)
#define R_DMA_HISR           (0x04 / 4)//r
#define R_DMA_LIFCR          (0x08 / 4)
#define R_DMA_HIFCR          (0x0c / 4)//w
#define R_DMA_ISR_FIEF     (1 << 0)
#define R_DMA_ISR_DMEIF    (1 << 2)
#define R_DMA_ISR_TIEF     (1 << 3)
#define R_DMA_ISR_HTIF     (1 << 4)
#define R_DMA_ISR_TCIF     (1 << 5)

/* Per-stream registers. */
#define R_DMA_Sx             (0x10 / 4)
#define R_DMA_Sx_COUNT           8
#define R_DMA_Sx_REGS            6
#define R_DMA_SxCR           (0x00 / 4)
#define R_DMA_SxCR_EN   0x00000001
#define R_DMA_SxCR_DIR	0x000000C0
#define R_DMA_SxNDTR         (0x04 / 4)
#define R_DMA_SxNDTR_EN 0x00000001
#define R_DMA_SxPAR          (0x08 / 4)
#define R_DMA_SxM0AR         (0x0c / 4)
#define R_DMA_SxM1AR         (0x10 / 4)
#define R_DMA_SxFCR          (0x14 / 4)

#define R_DMA_MAX            (0xd0 / 4)

typedef struct f2xx_dma_stream {
    qemu_irq irq;

    uint32_t cr;
    uint16_t ndtr;
    uint32_t par;
    uint32_t m0ar;
    uint32_t m1ar;
    uint8_t isr;
} f2xx_dma_stream;

static int msize_table[] = {1, 2, 4, 0};

typedef struct f2xx_dma {
    SysBusDevice busdev;
    MemoryRegion iomem;

    uint32_t ifcr[R_DMA_HIFCR - R_DMA_LIFCR + 1];
    f2xx_dma_stream stream[R_DMA_Sx_COUNT]; 
} f2xx_dma;

/* Pack ISR bits from four streams, for {L,H}ISR. */
static uint32_t
f2xx_dma_pack_isr(struct f2xx_dma *s, int start_stream)
{
    uint32_t r = 0;
    int i;

    for (i = 0; i < 4; i++) {
        r |= s->stream[i + start_stream].isr << (6 * i);
    }
    return r;
}

/* Per-stream read. */
static uint32_t
f2xx_dma_stream_read(f2xx_dma_stream *s, int stream_no, uint32_t reg)
{
    switch (reg) {
    case R_DMA_SxCR:
        DPRINTF("   %s: stream: %d, register CR\n", __func__, stream_no);
        return s->cr;
    case R_DMA_SxNDTR:
        DPRINTF("   %s: stream: %d, register NDTR (UNIMPLEMENTED)\n", __func__, stream_no);
        qemu_log_mask(LOG_UNIMP, "f2xx dma unimp read reg NDTR\n");
        return 0;
    case R_DMA_SxPAR:
        DPRINTF("   %s: stream: %d, register PAR (UNIMPLEMENTED)\n", __func__, stream_no);
        qemu_log_mask(LOG_UNIMP, "f2xx dma unimp read reg PAR\n");
        return 0;
    case R_DMA_SxM0AR:
        DPRINTF("   %s: stream: %d, register M0AR (UNIMPLEMENTED)\n", __func__, stream_no);
        qemu_log_mask(LOG_UNIMP, "f2xx dma unimp read reg M0AR\n");
        return 0;
    case R_DMA_SxM1AR:
        DPRINTF("   %s: stream: %d, register M1AR (UNIMPLEMENTED)\n", __func__, stream_no);
        qemu_log_mask(LOG_UNIMP, "f2xx dma unimp read reg M1AR\n");
        return 0;
    case R_DMA_SxFCR:
        DPRINTF("   %s: stream: %d, register FCR (UNIMPLEMENTED)\n", __func__, stream_no);
        qemu_log_mask(LOG_UNIMP, "f2xx dma unimp read reg FCR\n");
        return 0;
    default:
        DPRINTF("   %s: stream: %d, register 0x%02x\n", __func__, stream_no, reg<<2);
        qemu_log_mask(LOG_UNIMP, "f2xx dma unimp read stream reg 0x%02x\n",
          (unsigned int)reg<<2);
    }
    return 0;
}

/* Register read. */
static uint64_t
f2xx_dma_read(void *arg, hwaddr addr, unsigned int size)
{
    f2xx_dma *s = arg;
    uint64_t result;

    DPRINTF("%s: addr: 0x%llx, size:%d...\n", __func__, addr, size);

    if (size != 4) {
        qemu_log_mask(LOG_UNIMP, "f2xx crc only supports 4-byte reads\n");
        return 0;
    }

    addr >>= 2;
    if (addr >= R_DMA_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid read f2xx dma register 0x%02x\n",
                      (unsigned int)addr << 2);
        result = 0;
    } else {
        switch(addr) {
        case R_DMA_LISR:
            DPRINTF("   %s: register LISR\n", __func__);
            result = f2xx_dma_pack_isr(s, 0);
            break;
        case R_DMA_HISR:
            DPRINTF("   %s: register HISR\n", __func__);
            result = f2xx_dma_pack_isr(s, 4);
            break;
        case R_DMA_LIFCR:
            DPRINTF("   %s: register LIFCR\n", __func__);
            result = s->ifcr[addr - R_DMA_LIFCR];
            break;
        case R_DMA_HIFCR:
            DPRINTF("   %s: register HIFCR\n", __func__);
            result = s->ifcr[addr - R_DMA_LIFCR];
            break;
        default:
            /* Only per-stream registers remain. */
            addr -= R_DMA_Sx;
            int stream_no = addr / R_DMA_Sx_REGS;
            result = f2xx_dma_stream_read(&s->stream[stream_no], stream_no,
                                          addr % R_DMA_Sx_REGS);
            break;
        }
    }

    DPRINTF("    %s: result:0x%llx\n", __func__, result);
    return result;
}

/* Start a DMA transfer for a given stream. */
static void
f2xx_dma_stream_start(f2xx_dma_stream *s, int stream_no)
{
    uint8_t buf[4];
    int msize = msize_table[(s->cr >> 13) & 0x3];

    DPRINTF("%s: stream: %d\n", __func__, stream_no);

    if (msize == 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "f2xx dma: invalid MSIZE\n");
        return;
    }
    /* XXX Skip USART, as pacing control is not yet in place. */
    if (s->par == 0x40011004) {
        qemu_log_mask(LOG_UNIMP, "f2xx dma: skipping USART\n");
        return;
    }

    /* XXX hack do the entire transfer here for now. */
    /* Read from peripheral and store to memory */
    if ((s->cr & R_DMA_SxCR_DIR) == 0x00) {
	DPRINTF("%s: DIR 0 transferring %d x %d byte(s) from 0x%08x to 0x%08x\n", __func__, s->ndtr,
              msize, s->par, s->m0ar);
	while (s->ndtr--) {
	    cpu_physical_memory_read(s->par, buf, msize);
	    cpu_physical_memory_write(s->m0ar, buf, msize);
	    s->m0ar += msize;
	}
	/* Read from memory and store to peripheral */
    } else if ((s->cr & R_DMA_SxCR_DIR) == 0x40) {
	DPRINTF("%s: DIR 1 transferring %d x %d byte(s) from 0x%08x to 0x%08x\n", __func__, s->ndtr,
              msize, s->m0ar, s->par);
	while (s->ndtr--) {
	    cpu_physical_memory_read(s->m0ar, buf, msize);
	    cpu_physical_memory_write(s->par, buf, msize);
	    s->m0ar += msize;
	}
    }
    /* Transfer complete. */
    s->cr &= ~R_DMA_SxCR_EN;
    s->isr |= R_DMA_ISR_TCIF;
    qemu_set_irq(s->irq, 1);
}

/* Per-stream register write. */
static void
f2xx_dma_stream_write(f2xx_dma_stream *s, int stream_no, uint32_t addr, uint32_t data)
{
    switch (addr) {
    case R_DMA_SxCR:
        DPRINTF("%s: stream: %d, register CR, data:0x%x\n", __func__, stream_no, data);
        if ((s->cr & R_DMA_SxCR_EN) == 0 && (data & R_DMA_SxCR_EN) != 0) {
            f2xx_dma_stream_start(s, stream_no);
        }
        s->cr = data;
        break;
    case R_DMA_SxNDTR:
        DPRINTF("%s: stream: %d, register NDTR, data:0x%x\n", __func__, stream_no, data);
        if (s->cr & R_DMA_SxNDTR_EN) {
            qemu_log_mask(LOG_GUEST_ERROR, "f2xx dma write to NDTR while enabled\n");
            return;
        }
        s->ndtr = data;
        break;
    case R_DMA_SxPAR:
        DPRINTF("%s: stream: %d, register PAR, data:0x%x\n", __func__, stream_no, data);
        s->par = data;
        break;
    case R_DMA_SxM0AR:
        DPRINTF("%s: stream: %d, register M0AR, data:0x%x\n", __func__, stream_no, data);
        s->m0ar = data;
        break;
    case R_DMA_SxM1AR:
        DPRINTF("%s: stream: %d, register M1AR, data:0x%x\n", __func__, stream_no, data);
        s->m1ar = data;
        break;
    case R_DMA_SxFCR:
        DPRINTF("%s: stream: %d, register FCR (UINIMPLEMENTED), data:0x%x\n", __func__,
                        stream_no, data);
        qemu_log_mask(LOG_UNIMP, "f2xx dma SxFCR unimplemented\n");
        break;
    }
}

/* Register write. */
static void
f2xx_dma_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    f2xx_dma *s = arg;
    int offset = addr & 0x3;

    (void)offset;

    /* XXX Check DMA peripheral clock enable. */
    if (size != 4) {
        qemu_log_mask(LOG_UNIMP, "f2xx dma only supports 4-byte writes\n");
        return;
    }

    addr >>= 2;
    if (addr >= R_DMA_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid write f2xx dma register 0x%02x\n",
          (unsigned int)addr << 2);
        return;
    }
    if (addr >= R_DMA_Sx && addr <= 0xcc) {
        int num = (addr - R_DMA_Sx) / R_DMA_Sx_REGS;
        f2xx_dma_stream_write(&s->stream[num], num,
          (addr - R_DMA_Sx) % R_DMA_Sx_REGS, data);
        return;
    }
    switch(addr) {
    case R_DMA_LISR:
        DPRINTF("%s: register LISR (READ-ONLY), data: 0x%llx\n", __func__, data);
        qemu_log_mask(LOG_GUEST_ERROR, "f2xx dma: invalid write to ISR\n");
        break;
    case R_DMA_HISR:
        DPRINTF("%s: register HISR (READ-ONLY), data: 0x%llx\n", __func__, data);
        qemu_log_mask(LOG_GUEST_ERROR, "f2xx dma: invalid write to ISR\n");
        break;
    case R_DMA_LIFCR:
        DPRINTF("%s: register LIFCR, data: 0x%llx\n", __func__, data);
        // Any interrupt clear write to stream x clears all interrupts for that stream
        s->ifcr[addr - R_DMA_LIFCR] = data;
        if (data & 0x0f400000) {
            s->stream[3].isr = 0;
            qemu_set_irq(s->stream[3].irq, 0);
        }
        if (data & 0x003d0000) {
            s->stream[2].isr = 0;
            qemu_set_irq(s->stream[2].irq, 0);
        }
        if (data & 0x00000f40) {
            s->stream[1].isr = 0;
            qemu_set_irq(s->stream[1].irq, 0);
        }
        if (data & 0x0000003d) {
            s->stream[0].isr = 0;
            qemu_set_irq(s->stream[0].irq, 0);
        }
        break;
    case R_DMA_HIFCR:
        DPRINTF("%s: register HIFCR, data: 0x%llx\n", __func__, data);
        // Any interrupt clear write to stream x clears all interrupts for that stream
        s->ifcr[addr - R_DMA_LIFCR] = data;
        if (data & 0x0f400000) {
            s->stream[7].isr = 0;
            qemu_set_irq(s->stream[7].irq, 0);
        }
        if (data & 0x003d0000) {
            s->stream[6].isr = 0;
            qemu_set_irq(s->stream[6].irq, 0);
        }
        if (data & 0x00000f40) {
            s->stream[5].isr = 0;
            qemu_set_irq(s->stream[5].irq, 0);
        }
        if (data & 0x0000003d) {
            s->stream[4].isr = 0;
            qemu_set_irq(s->stream[4].irq, 0);
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "f2xx dma unimpl write reg 0x%02x\n",
          (unsigned int)addr << 2);
    }
}

static const MemoryRegionOps f2xx_dma_ops = {
    .read = f2xx_dma_read,
    .write = f2xx_dma_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

static int
f2xx_dma_init(SysBusDevice *dev)
{
    f2xx_dma *s = FROM_SYSBUS(f2xx_dma, dev);
    int i;

    memory_region_init_io(&s->iomem, OBJECT(s), &f2xx_dma_ops, s, "dma", 0x400);
    sysbus_init_mmio(dev, &s->iomem);

    for (i = 0; i < R_DMA_Sx_COUNT; i++) {
        sysbus_init_irq(dev, &s->stream[i].irq);
    }

    return 0;
}

static void
f2xx_dma_reset(DeviceState *ds)
{
    f2xx_dma *s = FROM_SYSBUS(f2xx_dma, SYS_BUS_DEVICE(ds));

    memset(&s->ifcr, 0, sizeof(s->ifcr));

    int i;
    for (i=0; i<R_DMA_Sx_COUNT; i++) {
        qemu_irq save = s->stream[i].irq;
        memset(&s->stream[i], 0, sizeof(f2xx_dma_stream));
        s->stream[i].irq = save;
    }
}

static Property f2xx_dma_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void
f2xx_dma_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *sc = SYS_BUS_DEVICE_CLASS(klass);
    sc->init = f2xx_dma_init;
    dc->reset = f2xx_dma_reset;
    //TODO: fix this: dc->no_user = 1;
    dc->props = f2xx_dma_properties;
}

static const TypeInfo
f2xx_dma_info = {
    .name          = "f2xx_dma",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(f2xx_dma),
    .class_init    = f2xx_dma_class_init,
};

static void
f2xx_dma_register_types(void)
{
    type_register_static(&f2xx_dma_info);
}

type_init(f2xx_dma_register_types)
