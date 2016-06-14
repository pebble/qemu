/*
 * Emulate a QSPI flash device following the mx25u command set.
 * Modelled after the m25p80 emulation found in hw/block/m25p80.c
 */

#include "hw/hw.h"
#include "sysemu/block-backend.h"
#include "sysemu/blockdev.h"
#include "hw/ssi.h"


// TODO: These should be made configurable to support different flash parts
#define FLASH_SECTOR_SIZE (64 << 10)
#define FLASH_NUM_SECTORS (128)
#define FLASH_PAGE_SIZE (256)
const uint8_t MX25U_ID[] = { 0xc2, 0x25, 0x37 };

#ifndef MX25U_ERR_DEBUG
#define MX25U_ERR_DEBUG 0
#endif

// The usleep() helps MacOS stdout from freezing when printing a lot
#define DB_PRINT_L(level, ...) do { \
    if (MX25U_ERR_DEBUG > (level)) { \
        fprintf(stderr,  "%d: %s: ", level, __func__); \
        fprintf(stderr, ## __VA_ARGS__); \
        fprintf(stderr, "\n"); \
        usleep(1000); \
    } \
} while (0);

typedef enum {
    WRITE_ENABLE = 0x06,
    WRITE_DISABLE = 0x04,

    READ_STATUS_REG = 0x05,
    READ_SCUR_REG = 0x2b,

    READ = 0x03,
    FAST_READ = 0x0b,
    QREAD = 0x6b,
    READ_ID = 0x9f,
    READ_QID = 0xaf,

    PAGE_PROGRAM = 0x02,
    QPAGE_PROGRAM = 0x38,

    ERASE_SUBSECTOR = 0x20, // Erase 4k sector
    ERASE_SECTOR = 0x52, // Erase 32k sector
    ERASE_BLOCK = 0xd8, // Erase 64k block
    ERASE_CHIP = 0xc7,

    ERASE_SUSPEND = 0xB0,
    ERASE_RESUME = 0x30,

    DEEP_SLEEP = 0xb9,
    WAKE = 0xab,

    QUAD_ENABLE = 0x35,
    QUAD_DISABLE = 0xf5,
} FlashCmd;

typedef enum {
    STATE_IDLE,

    STATE_COLLECT_CMD_DATA,

    STATE_WRITE,
    STATE_READ,
    STATE_READ_ID,
    STATE_READ_REGISTER,
} CMDState;

#define R_SR_WIP  (1 << 0)
#define R_SR_WEL  (1 << 1)
#define R_SR_BP0  (1 << 2)
#define R_SR_BP1  (1 << 3)
#define R_SR_BP2  (1 << 4)
#define R_SR_BP3  (1 << 5)
#define R_SR_QE   (1 << 6)
#define R_SR_SRWD (1 << 7)

#define R_SCUR_SOTP  (1 << 0)
#define R_SCUR_LDSO  (1 << 1)
#define R_SCUR_PSB   (1 << 2)
#define R_SCUR_ESB   (1 << 3)
#define R_SCUR_PFAIL (1 << 5)
#define R_SCUR_EFAIL (1 << 6)
#define R_SCUR_WPSEL (1 << 7)

typedef struct FLASH {
    SSISlave parent_obj;

    //--- Storage ---
    BlockBackend *blk;
    uint8_t *storage;
    uint32_t size;
    int page_size;

    int64_t dirty_page;

    //--- Registers ---
    uint8_t SR;
    uint8_t SCUR;

    //--- Command state ---
    CMDState state;
    FlashCmd cmd_in_progress;
    uint8_t cmd_data[4]; //! Commands can require up to 4 bytes of additional data
    uint8_t cmd_bytes;   //! Number of bytes required by command [0,4]
    uint32_t len;
    uint32_t pos;
    uint64_t current_address;

    uint8_t *current_register;
    uint8_t register_read_mask; //! mask to apply after reading current_register
} Flash;

typedef struct M25P80Class {
    SSISlaveClass parent_class;
    //FlashPartInfo *pi;
} MX25UClass;

#define TYPE_MX25U "mx25u-generic"
#define MX25U(obj) \
     OBJECT_CHECK(Flash, (obj), TYPE_MX25U)
#define MX25U_CLASS(klass) \
     OBJECT_CLASS_CHECK(MX25UClass, (klass), TYPE_MX25U)
#define MX25U_GET_CLASS(obj) \
     OBJECT_GET_CLASS(MX25UClass, (obj), TYPE_MX25U)

static void
blk_sync_complete(void *opaque, int ret)
{
    /* do nothing. Masters do not directly interact with the backing store,
     * only the working copy so no mutexing required.
     */
}

static void
mx25u_flash_sync_page(Flash *s, int page)
{
    int blk_sector, nb_sectors;
    QEMUIOVector iov;

    if (!s->blk || blk_is_read_only(s->blk)) {
        return;
    }

    blk_sector = (page * s->page_size) / BDRV_SECTOR_SIZE;
    nb_sectors = DIV_ROUND_UP(s->page_size, BDRV_SECTOR_SIZE);
    qemu_iovec_init(&iov, 1);
    qemu_iovec_add(&iov, s->storage + blk_sector * BDRV_SECTOR_SIZE,
                   nb_sectors * BDRV_SECTOR_SIZE);
    blk_aio_writev(s->blk, blk_sector, &iov, nb_sectors, blk_sync_complete,
                   NULL);
}

static inline void
mx25u_flash_sync_area(Flash *s, int64_t off, int64_t len)
{
    int64_t start, end, nb_sectors;
    QEMUIOVector iov;

    if (!s->blk || blk_is_read_only(s->blk)) {
        return;
    }

    assert(!(len % BDRV_SECTOR_SIZE));
    start = off / BDRV_SECTOR_SIZE;
    end = (off + len) / BDRV_SECTOR_SIZE;
    nb_sectors = end - start;
    qemu_iovec_init(&iov, 1);
    qemu_iovec_add(&iov, s->storage + (start * BDRV_SECTOR_SIZE),
                                        nb_sectors * BDRV_SECTOR_SIZE);
    blk_aio_writev(s->blk, start, &iov, nb_sectors, blk_sync_complete, NULL);
}

static inline void
flash_sync_dirty(Flash *s, int64_t newpage)
{
    if (s->dirty_page >= 0 && s->dirty_page != newpage) {
        mx25u_flash_sync_page(s, s->dirty_page);
        s->dirty_page = newpage;
    }
}

static void
mx25u_flash_erase(Flash *s, uint32_t offset, FlashCmd cmd)
{
  uint32_t len;

  switch (cmd) {
  case ERASE_SUBSECTOR: // Erase 4k sector
    len = 4 << 10;
    break;
  case ERASE_SECTOR: // Erase 32k sector
    len = 32 << 10;
    break;
  case ERASE_BLOCK: // Erase 64k block
    len = 64 << 10;
    break;
  case ERASE_CHIP:
    len = s->size;
    break;
  default:
    abort();
  }

  DB_PRINT_L(0, "erase offset = %#x, len = %d", offset, len);

  if (!(s->SR & R_SR_WEL)) {
    qemu_log_mask(LOG_GUEST_ERROR, "MX25U: erase with write protect!\n");
    return;
  }

  memset(s->storage + offset, 0xff, len);
  mx25u_flash_sync_area(s, offset, len);
}

static void
mx25u_decode_new_cmd(Flash *s, uint32_t value)
{
    s->cmd_in_progress = value;
    DB_PRINT_L(0, "decoding new command: 0x%x", value);

    switch (value) {
    case WRITE_ENABLE:
        s->SR |= R_SR_WEL;
        s->state = STATE_IDLE;
        break;
    case WRITE_DISABLE:
        s->SR &= ~R_SR_WEL;
        s->state = STATE_IDLE;
        break;

    case READ_STATUS_REG:
        s->current_register = &s->SR;
        s->state = STATE_READ_REGISTER;
        break;
    case READ_SCUR_REG:
        s->current_register = &s->SCUR;
        s->state = STATE_READ_REGISTER;
        break;

    case READ:
    case FAST_READ:
    case QREAD:
        s->cmd_bytes = 3;
        s->pos = 0;
        s->state = STATE_COLLECT_CMD_DATA;
        break;

    case READ_ID:
    case READ_QID:
        s->cmd_bytes = 0;
        s->state = STATE_READ_ID;
        s->len = 3;
        s->pos = 0;
        break;

    case PAGE_PROGRAM:
    case QPAGE_PROGRAM:
        s->pos = 0;
        s->cmd_bytes = 3;
        s->state = STATE_COLLECT_CMD_DATA;
        break;

    case ERASE_SUBSECTOR:
    case ERASE_SECTOR:
    case ERASE_BLOCK:
        s->pos = 0;
        s->cmd_bytes = 3;
        s->state = STATE_COLLECT_CMD_DATA;
        break;

    case ERASE_CHIP:
        mx25u_flash_erase(s, 0, ERASE_CHIP);
        s->SR |= R_SR_WIP;
        s->register_read_mask = R_SR_WIP;
        s->state = STATE_IDLE;
        break;

    case ERASE_SUSPEND:
    case ERASE_RESUME:
        break;

    case DEEP_SLEEP:
    case WAKE:
        break;

    case QUAD_ENABLE:
    case QUAD_DISABLE:
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "MX25U: Unknown cmd 0x%x\n", value);
    }
}

static void
mx25u_handle_cmd_data(Flash *s)
{
    s->current_address = (s->cmd_data[2] << 16) | (s->cmd_data[1] << 8) | (s->cmd_data[0]);
    s->state = STATE_IDLE;

    switch (s->cmd_in_progress) {
    case PAGE_PROGRAM:
    case QPAGE_PROGRAM:
        s->state = STATE_WRITE;
        break;
    case READ_STATUS_REG:
    case READ_SCUR_REG:
        assert(false);
        break;
    case READ:
    case FAST_READ:
    case QREAD:
        DB_PRINT_L(1, "Read From: 0x%"PRIu64, s->current_address);
        s->state = STATE_READ;
        break;
    case ERASE_SUBSECTOR:
    case ERASE_SECTOR:
    case ERASE_BLOCK:
        mx25u_flash_erase(s, s->current_address, s->cmd_in_progress);
        s->SR |= R_SR_WIP;
        s->register_read_mask = R_SR_WIP;
        break;

    case ERASE_SUSPEND:
    case ERASE_RESUME:
        break;

    case ERASE_CHIP:
    default:
        break;
    }
}

static void
mx25u_write8(Flash *s, uint8_t value)
{
    int64_t page = s->current_address / s->page_size;

    // TODO: Write protection

    uint8_t current = s->storage[s->current_address];
    if (value & ~current) {
        qemu_log_mask(LOG_GUEST_ERROR, "MX25U: Flipping bit from 0 => 1\n");
    }
    DB_PRINT_L(1, "Write 0x%"PRIx8" = 0x%"PRIx64, (uint8_t)value, s->current_address);
    s->storage[s->current_address] = (uint8_t)value;

    flash_sync_dirty(s, page);
    s->dirty_page = page;
}

static uint32_t
mx25u_transfer8(SSISlave *ss, uint32_t tx)
{
    Flash *s = MX25U(ss);
    uint32_t r = 0;

    switch (s->state) {
    case STATE_COLLECT_CMD_DATA:
        DB_PRINT_L(2, "Collected: 0x%"PRIx32, (uint32_t)tx);
        s->cmd_data[s->pos++] = (uint8_t)tx;
        if (s->pos == s->cmd_bytes) {
            mx25u_handle_cmd_data(s);
        }
        break;
    case STATE_WRITE:
        if (s->current_address > s->size) {
          qemu_log_mask(LOG_GUEST_ERROR,
              "MX25U: Out of bounds flash write to 0x%"PRIx64"\n", s->current_address);
        } else {
          mx25u_write8(s, tx);
          s->current_address += 1;
        }
        break;
    case STATE_READ:
        if (s->current_address > s->size) {
          qemu_log_mask(LOG_GUEST_ERROR,
              "MX25U: Out of bounds flash read from 0x%"PRIx64"\n", s->current_address);
        } else {
          DB_PRINT_L(1, "Read 0x%"PRIx64" = 0x%"PRIx8, s->current_address, (uint8_t)r);
          r = s->storage[s->current_address];
          s->current_address = (s->current_address + 1) % s->size;
        }
        break;
    case STATE_READ_ID:
        r = MX25U_ID[s->pos];
        DB_PRINT_L(2, "Read ID 0x%x (pos 0x%x)", (uint8_t)r, s->pos);
        ++s->pos;
        if (s->pos == s->len) {
            s->pos = 0;
            s->state = STATE_IDLE;
        }
        break;
    case STATE_READ_REGISTER:
        r = *s->current_register;
        *s->current_register &= ~s->register_read_mask;
        s->register_read_mask = 0;
        s->state = STATE_IDLE;
        DB_PRINT_L(1, "Read register");
        break;
    case STATE_IDLE:
        mx25u_decode_new_cmd(s, tx);
        break;
    }

    return r;
}

static int
mx25u_init(SSISlave *ss)
{
    DriveInfo *dinfo;
    Flash *s = MX25U(ss);

    s->state = STATE_IDLE;
    s->size = FLASH_SECTOR_SIZE * FLASH_NUM_SECTORS;
    s->page_size = FLASH_PAGE_SIZE;
    s->dirty_page = -1;
    s->SR = 0;

    /* FIXME use a qdev drive property instead of drive_get_next() */
    dinfo = drive_get_next(IF_MTD);

    if (dinfo) {
        DB_PRINT_L(0, "Binding to IF_MTD drive");
        s->blk = blk_by_legacy_dinfo(dinfo);
        blk_attach_dev_nofail(s->blk, s);

        s->storage = blk_blockalign(s->blk, s->size);

        /* FIXME: Move to late init */
        if (blk_read(s->blk, 0, s->storage,
                     DIV_ROUND_UP(s->size, BDRV_SECTOR_SIZE))) {
            fprintf(stderr, "Failed to initialize SPI flash!\n");
            return 1;
        }
    } else {
        DB_PRINT_L(0, "No BDRV - binding to RAM");
        s->storage = blk_blockalign(NULL, s->size);
        memset(s->storage, 0xFF, s->size);
    }
    return 0;
}

static int
mx25u_cs(SSISlave *ss, bool select)
{
    Flash *s = MX25U(ss);

    if (select) {
        s->len = 0;
        s->pos = 0;
        s->state = STATE_IDLE;
        flash_sync_dirty(s, -1);
    }

    DB_PRINT_L(0, "CS %s", select ? "HIGH" : "LOW");

    return 0;
}

static void
mx25u_pre_save(void *opaque)
{
    flash_sync_dirty((Flash *)opaque, -1);
}

static const VMStateDescription vmstate_mx25u = {
    .name = "mx25u",
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = mx25u_pre_save,
    .fields = (VMStateField[]) {
#if 0
      VMSTATE_UINT8(SR, Flash),
      VMSTATE_UINT8(SCUR, Flash),
      VMSTATE_UINT8(state, Flash),
      VMSTATE_UINT8_ARRAY(cmd_data, Flash, 4),
      VMSTATE_UINT8(cmd_bytes, Flash),
      VMSTATE_UINT32(len, Flash),
      VMSTATE_UINT32(pos, Flash),
      VMSTATE_UINT32(current_address, Flash),
      VMSTATE_UINT8(cmd_in_progress, Flash),
#endif
      VMSTATE_END_OF_LIST()
    }
};

static void
mx25u_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    SSISlaveClass *c = SSI_SLAVE_CLASS(class);

    c->init = mx25u_init;
    c->transfer = mx25u_transfer8;
    c->set_cs = mx25u_cs;
    c->cs_polarity = SSI_CS_LOW;
    dc->vmsd = &vmstate_mx25u;
    //mc->pi = data;
}

static const TypeInfo mx25u_info = {
    .name           = TYPE_MX25U,
    .parent         = TYPE_SSI_SLAVE,
    .instance_size  = sizeof(Flash),
    .class_size     = sizeof(MX25UClass),
    .abstract       = true,
};

static void
mx25u_register_types(void)
{
    type_register_static(&mx25u_info);

    TypeInfo ti = {
        .name   = "mx25u6435f",
        .parent = TYPE_MX25U,
        .class_init = mx25u_class_init,
        //.class_data = (void *)
    };
    type_register(&ti);
}

type_init(mx25u_register_types)
