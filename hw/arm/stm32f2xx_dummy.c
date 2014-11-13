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
 * QEMU dummy emulation
 */
#include "hw/sysbus.h"

typedef struct f2xx_dummy {
    SysBusDevice busdev;
    MemoryRegion iomem;
    void *name;
    int32_t size;
} f2xx_dummy;

static uint64_t
f2xx_dummy_read(void *arg, hwaddr addr, unsigned int size)
{
    f2xx_dummy *s = arg;

    qemu_log_mask(LOG_UNIMP, "%s dummy read 0x%x %d byte%s\n", s->name,
      (unsigned int)addr, size, size != 1 ? "s" : "");

    return 0;
}

static void
f2xx_dummy_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    f2xx_dummy *s = arg;

    qemu_log_mask(LOG_UNIMP, "%s dummy write 0x%x %d byte%s value 0x%x\n",
      s->name, (unsigned int)addr, size, size != 1 ? "s" : "",
      (unsigned int)data);
}

static const MemoryRegionOps f2xx_dummy_ops = {
    .read = f2xx_dummy_read,
    .write = f2xx_dummy_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

static int
f2xx_dummy_init(SysBusDevice *dev)
{
    f2xx_dummy *s = FROM_SYSBUS(f2xx_dummy, dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &f2xx_dummy_ops, s, "dummy", s->size);
    sysbus_init_mmio(dev, &s->iomem);
    return 0;
}

static Property f2xx_dummy_properties[] = {
    DEFINE_PROP_PTR("name", f2xx_dummy, name),
    DEFINE_PROP_INT32("size", f2xx_dummy, size, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void
f2xx_dummy_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *sc = SYS_BUS_DEVICE_CLASS(klass);
    sc->init = f2xx_dummy_init;
    // TODO: fix this: dc->no_user = 1;
    dc->props = f2xx_dummy_properties;
}

static const TypeInfo
f2xx_dummy_info = {
    .name          = "f2xx_dummy",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(f2xx_dummy),
    .class_init    = f2xx_dummy_class_init,
};

static void
f2xx_dummy_register_types(void)
{
    type_register_static(&f2xx_dummy_info);
}

type_init(f2xx_dummy_register_types)
