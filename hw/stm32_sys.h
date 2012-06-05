
typedef struct {
    MemoryRegion iomem;
    uint32_t pborctl;
    uint32_t ldopctl;
    uint32_t int_status;
    uint32_t int_mask;
    uint32_t resc;
    uint32_t rcc;

    uint32_t rcc_cr;
    uint32_t rcc_cfgr;
    uint32_t rcc_anything_else;

    uint32_t rcc2;
    uint32_t rcgc[3];
    uint32_t scgc[3];
    uint32_t dcgc[3];
    uint32_t clkvclr;
    uint32_t ldoarst;
    qemu_irq irq;
    stm32_board_info *board;
} ssys_state;

static void ssys_update(ssys_state *s)
{
  qemu_set_irq(s->irq, (s->int_status & s->int_mask) != 0);
}

static uint64_t ssys_read(void *opaque, target_phys_addr_t offset,
                          unsigned size)
{
    ssys_state *s = (ssys_state *)opaque;

    uint64_t result = 0;

    switch (offset)
    {
    case 0x00:
        result = s->rcc_cr; break;
    case 0x08:
        result = s->rcc_cfgr; break;
    default:
        result = s->rcc_anything_else; break;
        //hw_error("ssys_write: Bad offset 0x%x\n", (int)offset);
    }

    printf("stm32_read running: offset %u result %llu (0x%llx)\n", offset, result, result);

    return result;
}

static bool ssys_use_rcc2(ssys_state *s)
{
    return (s->rcc2 >> 31) & 0x1;
}

/*
 * Caculate the sys. clock period in ms.
 */
static void ssys_calculate_system_clock(ssys_state *s)
{
    if (ssys_use_rcc2(s)) {
        system_clock_scale = 5 * (((s->rcc2 >> 23) & 0x3f) + 1);
    } else {
        system_clock_scale = 5 * (((s->rcc >> 23) & 0xf) + 1);
    }
}

static void ssys_write(void *opaque, target_phys_addr_t offset,
                       uint64_t value, unsigned size)
{
    printf("stm32_write running: offset %u value %llu (0x%llx) size %u\n", offset, value, value, size);
    ssys_state *s = (ssys_state *)opaque;

    switch (offset) {
    case 0x00:
        s->rcc_cr = value;
        if (s->rcc_cr | RCC_CR_PLLON)
        {
            s->rcc_cr |= RCC_CR_PLLRDY;
        }
        break;
    case 0x08:
    {
        // Clear out the previously selected system clock and paste in the new ones.
        // selected is the mask 0x03, used is the mask 0x0c
        s->rcc_cfgr = value & (~RCC_CFGR_SWS);
        s->rcc_cfgr |= (s->rcc_cfgr & RCC_CFGR_SW) << 2;
        break;
    }
    default:
        s->rcc_anything_else = value; break;
        //hw_error("ssys_write: Bad offset 0x%x\n", (int)offset);
    }
    ssys_update(s);
}

static const MemoryRegionOps ssys_ops = {
    .read = ssys_read,
    .write = ssys_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void ssys_reset(void *opaque)
{
    ssys_state *s = (ssys_state *)opaque;

    s->pborctl = 0x7ffd;
    s->rcc = 0x078e3ac0;

    s->rcc2 = 0;
    s->rcgc[0] = 1;
    s->scgc[0] = 1;
    s->dcgc[0] = 1;
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
    .version_id = 2,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .post_load = stm32_sys_post_load,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(pborctl, ssys_state),
        VMSTATE_UINT32(ldopctl, ssys_state),
        VMSTATE_UINT32(int_mask, ssys_state),
        VMSTATE_UINT32(int_status, ssys_state),
        VMSTATE_UINT32(resc, ssys_state),
        VMSTATE_UINT32(rcc, ssys_state),
        VMSTATE_UINT32_V(rcc2, ssys_state, 2),
        VMSTATE_UINT32_ARRAY(rcgc, ssys_state, 3),
        VMSTATE_UINT32_ARRAY(scgc, ssys_state, 3),
        VMSTATE_UINT32_ARRAY(dcgc, ssys_state, 3),
        VMSTATE_UINT32(clkvclr, ssys_state),
        VMSTATE_UINT32(ldoarst, ssys_state),
        VMSTATE_END_OF_LIST()
    }
};

static int stm32_sys_init(uint32_t base, qemu_irq irq,
                          stm32_board_info * board)
{
    ssys_state *s;

    s = (ssys_state *)g_malloc0(sizeof(ssys_state));
    s->irq = irq;
    s->board = board;

    memory_region_init_io(&s->iomem, &ssys_ops, s, "ssys", 0x00000500);
    memory_region_add_subregion(get_system_memory(), base, &s->iomem);
    ssys_reset(s);
    vmstate_register(NULL, -1, &vmstate_stm32_sys, s);
    return 0;
}

