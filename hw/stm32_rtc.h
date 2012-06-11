
/* Implements the RTC memory map, 0x40002800-0x40002BFF */

typedef struct {
    MemoryRegion iomem;
    uint32_t dummy;
} stm32_rtc_state;

static uint64_t stm32_rtc_read(void *opaque, target_phys_addr_t offset,
                          unsigned size)
{
    (void) opaque;
    (void) size;
    //printf("stm32_rtc_read: Ignoring read to offset %u\n", offset);

    return 0;
}

static void stm32_rtc_write(void *opaque, target_phys_addr_t offset,
                       uint64_t value, unsigned size)
{
    (void) opaque;
    (void) size;
    printf("stm32_rtc_write: Ignoring write to offset %u value %"PRIu64" (0x%"PRIx64")\n",
           offset, value, value);
}

static const MemoryRegionOps stm32_rtc_ops = {
    .read = stm32_rtc_read,
    .write = stm32_rtc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void stm32_rtc_reset(stm32_rtc_state* s)
{
    s->dummy = 0;
}

static int stm32_rtc_post_load(void *opaque, int version_id)
{
    (void) opaque;
    (void) version_id;

    //rtc_state *s = opaque;
    return 0;
}

static const VMStateDescription vmstate_stm32_rtc = {
    .name = "stm32_rtc",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .post_load = stm32_rtc_post_load,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(dummy, stm32_rtc_state),
        VMSTATE_END_OF_LIST()
    }
};

static int stm32_rtc_init(void)
{
    stm32_rtc_state *s = (stm32_rtc_state *)g_malloc0(sizeof(stm32_rtc_state));

    memory_region_init_io(&s->iomem, &stm32_rtc_ops, s, "rtc", 0x00000400);
    memory_region_add_subregion(get_system_memory(), 0x40002800, &s->iomem);

    stm32_rtc_reset(s);

    vmstate_register(NULL, -1, &vmstate_stm32_rtc, s);
    return 0;
}
