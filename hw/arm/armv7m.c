/*
 * ARMV7M System emulation.
 *
 * Copyright (c) 2006-2007 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

#include "hw/sysbus.h"
#include "hw/arm/arm.h"
#include "hw/arm/stm32.h"
#include "hw/loader.h"
#include "elf.h"
#include "sysemu/qtest.h"
#include "qemu/error-report.h"

/* Bitbanded IO.  Each word corresponds to a single bit.  */

/* Get the byte address of the real memory for a bitband access.  */
static inline uint32_t bitband_addr(void * opaque, uint32_t addr)
{
    uint32_t res;

    res = *(uint32_t *)opaque;
    res |= (addr & 0x1ffffff) >> 5;
    return res;

}

static uint32_t bitband_readb(void *opaque, hwaddr offset)
{
    uint8_t v;
    cpu_physical_memory_read(bitband_addr(opaque, offset), &v, 1);
    return (v & (1 << ((offset >> 2) & 7))) != 0;
}

static void bitband_writeb(void *opaque, hwaddr offset,
                           uint32_t value)
{
    uint32_t addr;
    uint8_t mask;
    uint8_t v;
    addr = bitband_addr(opaque, offset);
    mask = (1 << ((offset >> 2) & 7));
    cpu_physical_memory_read(addr, &v, 1);
    if (value & 1)
        v |= mask;
    else
        v &= ~mask;
    cpu_physical_memory_write(addr, &v, 1);
}

static uint32_t bitband_readw(void *opaque, hwaddr offset)
{
    uint32_t addr;
    uint16_t mask;
    uint16_t v;
    addr = bitband_addr(opaque, offset) & ~1;
    mask = (1 << ((offset >> 2) & 15));
    mask = tswap16(mask);
    cpu_physical_memory_read(addr, &v, 2);
    return (v & mask) != 0;
}

static void bitband_writew(void *opaque, hwaddr offset,
                           uint32_t value)
{
    uint32_t addr;
    uint16_t mask;
    uint16_t v;
    addr = bitband_addr(opaque, offset) & ~1;
    mask = (1 << ((offset >> 2) & 15));
    mask = tswap16(mask);
    cpu_physical_memory_read(addr, &v, 2);
    if (value & 1)
        v |= mask;
    else
        v &= ~mask;
    cpu_physical_memory_write(addr, &v, 2);
}

static uint32_t bitband_readl(void *opaque, hwaddr offset)
{
    uint32_t addr;
    uint32_t mask;
    uint32_t v;
    addr = bitband_addr(opaque, offset) & ~3;
    mask = (1 << ((offset >> 2) & 31));
    mask = tswap32(mask);
    cpu_physical_memory_read(addr, &v, 4);
    return (v & mask) != 0;
}

static void bitband_writel(void *opaque, hwaddr offset,
                           uint32_t value)
{
    uint32_t addr;
    uint32_t mask;
    uint32_t v;
    addr = bitband_addr(opaque, offset) & ~3;
    mask = (1 << ((offset >> 2) & 31));
    mask = tswap32(mask);
    cpu_physical_memory_read(addr, &v, 4);
    if (value & 1)
        v |= mask;
    else
        v &= ~mask;
    cpu_physical_memory_write(addr, &v, 4);
}

static const MemoryRegionOps bitband_ops = {
    .old_mmio = {
        .read = { bitband_readb, bitband_readw, bitband_readl, },
        .write = { bitband_writeb, bitband_writew, bitband_writel, },
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

#define TYPE_BITBAND "ARM,bitband-memory"
#define BITBAND(obj) OBJECT_CHECK(BitBandState, (obj), TYPE_BITBAND)

typedef struct {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    uint32_t base;
} BitBandState;

static int bitband_init(SysBusDevice *dev)
{
    BitBandState *s = BITBAND(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &bitband_ops, &s->base,
                          "bitband", 0x02000000);
    sysbus_init_mmio(dev, &s->iomem);
    return 0;
}

static void armv7m_bitband_init(Object *parent)
{
    DeviceState *dev;

    dev = qdev_create(NULL, TYPE_BITBAND);
    qdev_prop_set_uint32(dev, "base", 0x20000000);
    if(parent) {
        object_property_add_child(parent, "bitband-sram", OBJECT(dev), NULL);
    }
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x22000000);

    dev = qdev_create(NULL, TYPE_BITBAND);
    qdev_prop_set_uint32(dev, "base", 0x40000000);
    if(parent) {
        object_property_add_child(parent, "bitband-periph", OBJECT(dev), NULL);
    }
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x42000000);
}

/* Board init.  */

static void armv7m_reset(void *opaque)
{
    ARMCPU *cpu = opaque;

    cpu_reset(CPU(cpu));
}

/* Init CPU and memory for a v7-M based board.
   flash_size and sram_size are in bytes.
   Returns the NVIC array.  */


DeviceState *armv7m_init(Object *parent, MemoryRegion *system_memory,
                      int flash_size, int sram_size, int num_irq,
                      const char *kernel_filename, const char *cpu_model)
{
    ARMCPU *cpu;
    return armv7m_translated_init(parent, system_memory, flash_size, sram_size, num_irq,
            kernel_filename, NULL, NULL, cpu_model, &cpu);
}

DeviceState *armv7m_translated_init(Object *parent, MemoryRegion *system_memory,
                                 int flash_size, int sram_size, int num_irq,
                                 const char *kernel_filename,
                                 uint64_t (*translate_fn)(void *, uint64_t),
                                 void *translate_opaque,
                                 const char *cpu_model,
                                 ARMCPU **cpu_device)
{
    ARMCPU *cpu;
    CPUARMState *env;
    DeviceState *nvic;
    if (num_irq == 0) {
        num_irq = STM32_MAX_IRQ + 1;
    }
    int image_size;
    uint64_t entry;
    uint64_t lowaddr;
    int big_endian;
    MemoryRegion *hack = g_new(MemoryRegion, 1);
    MemoryRegion *flash = NULL;
    MemoryRegion *sram = g_new(MemoryRegion, 1);
    ObjectClass *cpu_oc;
    Error *err = NULL;

    if (kernel_filename) {
        flash = g_new(MemoryRegion, 1);
    }

    if (cpu_model == NULL) {
        cpu_model = "cortex-m3";
    }
    cpu_oc = cpu_class_by_name(TYPE_ARM_CPU, cpu_model);
    cpu = ARM_CPU(object_new(object_class_get_name(cpu_oc)));
    if (cpu == NULL) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }
    /* On Cortex-M3/M4, the MPU has 8 windows */
    object_property_set_int(OBJECT(cpu), 8, "pmsav7-dregion", &err);
    if (err) {
        error_report_err(err);
        exit(1);
    }
    object_property_set_bool(OBJECT(cpu), true, "realized", &err);
    if (err) {
        error_report_err(err);
        exit(1);
    }
    *cpu_device = cpu;
    env = &cpu->env;

    if (kernel_filename) {
        memory_region_init_ram(flash, NULL, "armv7m.flash", flash_size, &err);
        vmstate_register_ram_global(flash);
        memory_region_set_readonly(flash, true);
        memory_region_add_subregion(system_memory, 0, flash);
    }

    if (sram_size) {
        memory_region_init_ram(sram, NULL, "armv7m.sram", sram_size, &err);
        vmstate_register_ram_global(sram);
        memory_region_add_subregion(system_memory, 0x20000000, sram);
    }
    armv7m_bitband_init(parent);

    /* If this is an M4, create the core-coupled memory region */
    if (!strcmp(cpu_model, "cortex-m4")) {
        MemoryRegion *ccm = g_new(MemoryRegion, 1);
        memory_region_init_ram(ccm, NULL, "armv7m.ccm", 64 * 1024 /* 64K */, &err);
        vmstate_register_ram_global(ccm);
        memory_region_add_subregion(system_memory, 0x10000000, ccm);
    }

    nvic = qdev_create(NULL, "armv7m_nvic");
    qdev_prop_set_uint32(nvic, "num-irq", num_irq);
    env->nvic = nvic;
    if(parent) {
        object_property_add_child(parent, "nvic", OBJECT(nvic), NULL);
    }
    qdev_init_nofail(nvic);

    // Connect the nvic's CPU #0 "parent_irq" output to the CPU's IRQ input handler
    sysbus_connect_irq(SYS_BUS_DEVICE(nvic), 0,
                       qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ));

    // Connect the nvic's "wakeup_out" output to the CPU's WKUP input handler
    qemu_irq cpu_wakeup_in = qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_WKUP);
    qdev_connect_gpio_out_named(DEVICE(nvic), "wakeup_out", 0, cpu_wakeup_in);



#ifdef TARGET_WORDS_BIGENDIAN
    big_endian = 1;
#else
    big_endian = 0;
#endif

    if (kernel_filename) {
        image_size = load_elf(kernel_filename, translate_fn, translate_opaque, &entry, &lowaddr,
                              NULL, big_endian, EM_ARM, 1);
        if (image_size < 0) {
            image_size = load_image_targphys(kernel_filename, 0, flash_size);
            lowaddr = 0;
        }
        if (image_size < 0) {
            error_report("Could not load kernel '%s'", kernel_filename);
            exit(1);
        }
    }

    /* Hack to map an additional page of ram at the top of the address
       space.  This stops qemu complaining about executing code outside RAM
       when returning from an exception.  */
    memory_region_init_ram(hack, NULL, "armv7m.hack", 0x1000, &error_fatal);
    vmstate_register_ram_global(hack);
    memory_region_add_subregion(system_memory, 0xfffff000, hack);

    qemu_register_reset(armv7m_reset, cpu);
    return nvic;
}

static Property bitband_properties[] = {
    DEFINE_PROP_UINT32("base", BitBandState, base, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void bitband_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = bitband_init;
    dc->props = bitband_properties;
}

static const TypeInfo bitband_info = {
    .name          = TYPE_BITBAND,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BitBandState),
    .class_init    = bitband_class_init,
};

static void armv7m_register_types(void)
{
    type_register_static(&bitband_info);
}

type_init(armv7m_register_types)
