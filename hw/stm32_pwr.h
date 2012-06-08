
/* Implements the PWR memory map, 0x4000 7000 - 0x4000 73FF */

typedef struct {
    MemoryRegion iomem;
    uint32_t dummy;
} stm32_pwr_state;

static uint64_t stm32_pwr_read(void *opaque, target_phys_addr_t offset,
                          unsigned size)
{
    (void) opaque;
    (void) size;
    printf("stm32_pwr_read: Ignoring read to offset %u", offset);

    return 0;
}

static void stm32_pwr_write(void *opaque, target_phys_addr_t offset,
                       uint64_t value, unsigned size)
{
    (void) opaque;
    (void) size;
    printf("stm32_pwr_write: Ignoring write to offset %u value %"PRIu64" (0x%"PRIx64")", offset, value, value);
}

static const MemoryRegionOps pwr_ops = {
    .read = stm32_pwr_read,
    .write = stm32_pwr_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void stm32_pwr_reset(stm32_pwr_state* s)
{
    s->dummy = 0;
}

static int stm32_pwr_post_load(void *opaque, int version_id)
{
    (void) opaque;
    (void) version_id;

    //stm32_pwr_state *s = opaque;
    return 0;
}

static const VMStateDescription vmstate_stm32_pwr = {
    .name = "stm32_pwr",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .post_load = stm32_pwr_post_load,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(dummy, stm32_pwr_state),
        VMSTATE_END_OF_LIST()
    }
};

static int stm32_pwr_init(void)
{
    stm32_pwr_state *s;

    s = (stm32_pwr_state*) g_malloc0(sizeof(stm32_pwr_state));

    memory_region_init_io(&s->iomem, &pwr_ops, s, "pwr", 0x00000400);
    memory_region_add_subregion(get_system_memory(), 0x40007000, &s->iomem);

    stm32_pwr_reset(s);

    vmstate_register(NULL, -1, &vmstate_stm32_pwr, s);
    return 0;
}
