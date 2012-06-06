
/* Implements the GPIO memory map, 0x40020000-0x40022400 */

typedef struct {
    MemoryRegion iomem;
    uint32_t dummy;
} gpio_state;

static uint64_t gpio_read(void *opaque, target_phys_addr_t offset,
                          unsigned size)
{
    (void) opaque;
    (void) size;
    printf("stm32_gpio_read: Ignoring read to offset %u", offset);

    return 0;
}

static void gpio_write(void *opaque, target_phys_addr_t offset,
                       uint64_t value, unsigned size)
{
    (void) opaque;
    (void) size;
    printf("stm32_gpio_write: Ignoring write to offset %u value %llu (0x%llx)", offset, value, value);
}

static const MemoryRegionOps gpio_ops = {
    .read = gpio_read,
    .write = gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void gpio_reset(gpio_state* s)
{
    s->dummy = 0;
}

static int stm32_gpio_post_load(void *opaque, int version_id)
{
    (void) opaque;
    (void) version_id;

    //gpio_state *s = opaque;
    return 0;
}

static const VMStateDescription vmstate_stm32_gpio = {
    .name = "stm32_gpio",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .post_load = stm32_gpio_post_load,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(dummy, gpio_state),
        VMSTATE_END_OF_LIST()
    }
};

static int stm32_gpio_init(void)
{
    gpio_state *s;

    s = (gpio_state *)g_malloc0(sizeof(gpio_state));

    memory_region_init_io(&s->iomem, &gpio_ops, s, "gpio", 0x00002400);
    memory_region_add_subregion(get_system_memory(), 0x40020000, &s->iomem);
    gpio_reset(s);
    vmstate_register(NULL, -1, &vmstate_stm32_gpio, s);
    return 0;
}
