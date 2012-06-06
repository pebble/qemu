
/* Implements the UART3 memory map, 0x40020000-0x40022400 */

#include "qemu-char.h"

typedef struct {
    MemoryRegion iomem;
    CharDriverState* char_driver;
    uint32_t reg_sr;
} stm32_uart_state;

static uint64_t stm32_uart_read(void *opaque, target_phys_addr_t offset,
                          unsigned size)
{
    (void) size;

    stm32_uart_state* s = (stm32_uart_state*) opaque;

    uint64_t result;

    switch(offset)
    {
    case 0x00:
        result = s->reg_sr; break;
    default:
        printf("stm32_uart_read: Ignoring read to offset %u", offset);
        return 0;
    }

    printf("stm32_uart_read: offset %u result %llu (0x%llx)\n", offset, result, result);
    return result;
}

static void stm32_uart_write(void *opaque, target_phys_addr_t offset,
                       uint64_t value, unsigned size)
{
    (void) size;

    stm32_uart_state* s = (stm32_uart_state*) opaque;

    switch(offset)
    {
    case 0x00:
        s->reg_sr = value;
        break;
    case 0x04:
    {
        unsigned char c = value & 0xff;
        if (s->char_driver)
        {
            qemu_chr_fe_write(s->char_driver, &c, 1);
        }
    }
    default:
        printf("stm32_uart_write: Ignoring write to offset %u value %llu (0x%llx)\n", offset, value, value);
        return;
    }

    printf("stm32_sys_write: offset %u value %llu (0x%llx) size %u\n", offset, value, value, size);
}

static const MemoryRegionOps uart_ops = {
    .read = stm32_uart_read,
    .write = stm32_uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void stm32_uart_reset(stm32_uart_state* s)
{
    s->reg_sr = 0x000000c0;
}

static int stm32_uart_post_load(void *opaque, int version_id)
{
    (void) opaque;
    (void) version_id;

    //uart_state *s = opaque;
    return 0;
}

static const VMStateDescription vmstate_stm32_uart = {
    .name = "stm32_uart",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .post_load = stm32_uart_post_load,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(reg_sr, stm32_uart_state),
        VMSTATE_END_OF_LIST()
    }
};

static int stm32_uart_can_receive(void* opaque)
{
    (void) opaque;

    printf("stm32_uart_can_receive\n");

    return 1;
}
  
static void stm32_uart_receive(void *opaque, const uint8_t *buf, int size)
{
    (void) opaque;
    (void) buf;
    (void) size;

    printf("stm32_uart_receive: size %d\n", size);
    abort();
}

static void stm32_uart_event(void *opaque, int event)
{
    (void) opaque;
    (void) event;

    printf("stm32_uart_event: event %d\n", event);
}

static int stm32_uart_init(void)
{
    stm32_uart_state *s;

    s = (stm32_uart_state *)g_malloc0(sizeof(stm32_uart_state));

    memory_region_init_io(&s->iomem, &uart_ops, s, "uart3", 0x00000500);
    memory_region_add_subregion(get_system_memory(), 0x40004800, &s->iomem);
    s->char_driver = qemu_char_get_next_serial();

    stm32_uart_reset(s);

    if (s->char_driver) {
        qemu_chr_add_handlers(s->char_driver,
                              stm32_uart_can_receive,
                              stm32_uart_receive,
                              stm32_uart_event,
                              s);
    }
    vmstate_register(NULL, -1, &vmstate_stm32_uart, s);
    return 0;
}
