
/* Implements "system" registers, namely the RCC registers. */


typedef struct {
    MemoryRegion iomem;
    uint32_t rcc_cr;
    uint32_t rcc_pllcfgr;
    uint32_t rcc_cfgr;
    uint32_t rcc_bdcr;
} ssys_state;

//static void ssys_update(ssys_state *s)
//{
//  qemu_set_irq(s->irq, (s->int_status & s->int_mask) != 0);
//}

static uint64_t ssys_read(void *opaque, target_phys_addr_t offset,
                          unsigned size)
{
    ssys_state *s = (ssys_state *)opaque;

    uint64_t result = 0;

    switch (offset)
    {
    case 0x00:
        result = s->rcc_cr; break;
    case 0x04:
        result = s->rcc_pllcfgr; break;
    case 0x08:
        result = s->rcc_cfgr; break;
    case 0x70: // BDCR
        result = s->rcc_bdcr; break;
    default:
        printf("stm32_sys_read: Ignoring read to offset %u\n", offset);
        return 0; 
        //hw_error("ssys_write: Bad offset 0x%x\n", (int)offset);
    }

    printf("stm32_sys_read: offset %u result %"PRIu64" (0x%"PRIx64")\n", offset, result, result);
    return result;
}

/*
 * Caculate the sys. clock period in ms.
 */
static void ssys_calculate_system_clock(ssys_state *s)
{
    /* We assume you're using the PLL. */
    system_clock_scale = 5 * (((s->rcc_pllcfgr >> 23) & 0xf) + 1);
}

static void ssys_write(void *opaque, target_phys_addr_t offset,
                       uint64_t value, unsigned size)
{
    ssys_state *s = (ssys_state *)opaque;

    switch (offset) {
    case 0x00:
        s->rcc_cr = value;
        if (s->rcc_cr | RCC_CR_PLLON)
        {
            s->rcc_cr |= RCC_CR_PLLRDY;
        }
        break;
    case 0x04:
        s->rcc_pllcfgr = value;
        ssys_calculate_system_clock(s);
        break;
    case 0x08:
    {
        // Clear out the previously selected system clock and paste in the new ones.
        // selected is the mask 0x03, used is the mask 0x0c
        s->rcc_cfgr = value & (~RCC_CFGR_SWS);
        s->rcc_cfgr |= (s->rcc_cfgr & RCC_CFGR_SW) << 2;
        break;
    }
    case 0x70:
        if (value == 0x01) // LSEON
        {
            s->rcc_bdcr = 0x01 | 0x02; // ON and RDY
        }
        else
        {
            s->rcc_bdcr = value;
        }
        break;
    default:
        printf("stm32_sys_write: Ignoring write to offset %u value %"PRIu64" (0x%"PRIx64")\n", offset, value, value);
        return;
        //hw_error("ssys_write: Bad offset 0x%x\n", (int)offset);
    }

    printf("stm32_sys_write: offset %u value %"PRIu64" (0x%"PRIx64") size %u\n", offset, value, value, size);
    //ssys_update(s);
}

static const MemoryRegionOps ssys_ops = {
    .read = ssys_read,
    .write = ssys_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void ssys_reset(ssys_state* s)
{
    s->rcc_cr = 0x00000083;
    s->rcc_pllcfgr = 0x24003010;
    s->rcc_cfgr = 0;
    s->rcc_bdcr = 0;

    ssys_calculate_system_clock(s);
}

static int stm32_sys_post_load(void *opaque, int version_id)
{
    ssys_state *s = opaque;

    ssys_calculate_system_clock(s);

    return 0;
}

static const VMStateDescription vmstate_stm32_sys = {
    .name = "stm32_sys",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .post_load = stm32_sys_post_load,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(rcc_cr, ssys_state),
        VMSTATE_UINT32(rcc_pllcfgr, ssys_state),
        VMSTATE_UINT32(rcc_cfgr, ssys_state),
        VMSTATE_UINT32(rcc_bdcr, ssys_state),
        VMSTATE_END_OF_LIST()
    }
};

static int stm32_sys_init(void)
{
    ssys_state *s;

    s = (ssys_state *)g_malloc0(sizeof(ssys_state));

    memory_region_init_io(&s->iomem, &ssys_ops, s, "ssys", 0x00000500);
    memory_region_add_subregion(get_system_memory(), 0x40023800, &s->iomem);
    ssys_reset(s);
    vmstate_register(NULL, -1, &vmstate_stm32_sys, s);
    return 0;
}

