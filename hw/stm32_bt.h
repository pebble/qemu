
/* Implements the UART1 memory map, 0x40011000-0x400113ff
 * By implement, I mean stub.
 */


typedef struct {
    MemoryRegion iomem;
    uint32_t reg_sr;
} stm32_bt_state;

static uint64_t stm32_bt_read(void *opaque, target_phys_addr_t offset,
                          unsigned size)
{
    (void) size;

    stm32_bt_state* s = (stm32_bt_state*) opaque;

    uint64_t result;

    switch(offset)
    {
    case 0x00:
        result = s->reg_sr; break;
    default:
        //printf("stm32_bt_read: Ignoring read to offset %u", offset);
        return 0;
    }

    //printf("stm32_bt_read: offset %u result %"PRIu64" (0x%"PRIx64")\n", offset, result, result);
    return result;
}

static void stm32_bt_write(void *opaque, target_phys_addr_t offset,
                       uint64_t value, unsigned size)
{
    (void) size;

    stm32_bt_state* s = (stm32_bt_state*) opaque;

    switch(offset)
    {
    case 0x00:
        s->reg_sr = value;
        break;
    default:
        //printf("stm32_bt_write: Ignoring write to offset %u value %"PRIu64" (0x%"PRIx64")\n", offset, value, value);
        return;
    }

    printf("stm32_sys_write: offset %u value %"PRIu64" (0x%"PRIx64") size %u\n", offset, value, value, size);
}

static const MemoryRegionOps bt_ops = {
    .read = stm32_bt_read,
    .write = stm32_bt_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int stm32_bt_post_load(void *opaque, int version_id)
{
    (void) opaque;
    (void) version_id;

    //bt_state *s = opaque;
    return 0;
}

static void stm32_bt_reset(stm32_bt_state* s)
{
    s->reg_sr = 0x000000c0; // USART_SR_TC | USART_SR_TXE
}

static const VMStateDescription vmstate_stm32_bt = {
    .name = "stm32_bt",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .post_load = stm32_bt_post_load,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(reg_sr, stm32_bt_state),
        VMSTATE_END_OF_LIST()
    }
};

static int stm32_bt_init(void)
{
    stm32_bt_state *s;

    s = (stm32_bt_state *)g_malloc0(sizeof(stm32_bt_state));

    memory_region_init_io(&s->iomem, &bt_ops, s, "bt", 0x00000400);
    memory_region_add_subregion(get_system_memory(), 0x40011000, &s->iomem);

    stm32_bt_reset(s);

    vmstate_register(NULL, -1, &vmstate_stm32_bt, s);
    return 0;
}
