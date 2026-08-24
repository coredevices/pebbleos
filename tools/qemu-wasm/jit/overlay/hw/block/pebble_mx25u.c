/*
 * Emulate a QSPI flash device following the MX25U command set.
 * Modelled after the m25p80 emulation found in hw/block/m25p80.c.
 *
 * Ported from QEMU 2.5.0-pebble8 mx25u.c to QEMU 10.x APIs.
 *
 * Copyright (c) 2013-2016 Pebble Technology
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "hw/ssi/ssi.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/arm/pebble.h"
#include "system/block-backend.h"
#include "system/block-backend-io.h"
#include "system/memory.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/log.h"

/* MX25U6435F: 64Mbit = 8MB */
#define FLASH_SECTOR_SIZE (64 << 10)   /* 64 KB per sector */
#define FLASH_NUM_SECTORS (128)        /* 128 sectors = 8MB */
#define FLASH_PAGE_SIZE   (256)        /* 256 bytes per page */

static const uint8_t MX25U_ID[] = { 0xc2, 0x25, 0x37 };

/* #define MX25U_DEBUG */
#ifdef MX25U_DEBUG
#define DB_PRINT_L(level, ...) do { \
    fprintf(stderr, "%d: %s: ", level, __func__); \
    fprintf(stderr, ## __VA_ARGS__); \
    fprintf(stderr, "\n"); \
} while (0)
#else
#define DB_PRINT_L(level, ...) do {} while (0)
#endif

typedef enum {
    WRITE_ENABLE     = 0x06,
    WRITE_DISABLE    = 0x04,

    READ_STATUS_REG  = 0x05,
    READ_SCUR_REG    = 0x2b,

    READ             = 0x03,
    FAST_READ        = 0x0b,
    QREAD            = 0x6b,
    READ_ID          = 0x9f,
    READ_QID         = 0xaf,

    PAGE_PROGRAM     = 0x02,
    QPAGE_PROGRAM    = 0x38,

    ERASE_SUBSECTOR  = 0x20,   /* 4k */
    ERASE_SECTOR     = 0x52,   /* 32k */
    ERASE_BLOCK      = 0xd8,   /* 64k */
    ERASE_CHIP       = 0xc7,

    ERASE_SUSPEND    = 0xB0,
    ERASE_RESUME     = 0x30,

    DEEP_SLEEP       = 0xb9,
    WAKE             = 0xab,

    QUAD_ENABLE      = 0x35,
    QUAD_DISABLE     = 0xf5,
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

typedef struct Flash {
    SSIPeripheral parent_obj;

    BlockBackend *blk;
    uint8_t *storage;
    uint32_t size;
    uint32_t capacity;  /* configurable via property */
    int page_size;

    int64_t dirty_page;

    MemoryRegion mmap;  /* memory-mapped read region */

    /* Registers */
    uint8_t SR;
    uint8_t SCUR;

    /* Command state */
    CMDState state;
    FlashCmd cmd_in_progress;
    uint8_t cmd_data[4];
    uint8_t cmd_bytes;
    uint32_t len;
    uint32_t pos;
    uint64_t current_address;

    uint8_t *current_register;
    uint8_t register_read_mask;
} Flash;

typedef struct MX25UClass {
    SSIPeripheralClass parent_class;
} MX25UClass;

#define TYPE_MX25U "mx25u-generic"
OBJECT_DECLARE_TYPE(Flash, MX25UClass, MX25U)

static void
mx25u_flash_sync_page(Flash *s, int page)
{
    if (!s->blk || !blk_is_writable(s->blk)) {
        return;
    }

    int64_t offset = (int64_t)page * s->page_size;
    blk_pwrite(s->blk, offset, s->page_size,
               s->storage + offset, 0);
}

static void
mx25u_flash_sync_area(Flash *s, int64_t off, int64_t len)
{
    if (!s->blk || !blk_is_writable(s->blk)) {
        return;
    }
    blk_pwrite(s->blk, off, len, s->storage + off, 0);
}

static void
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
    case ERASE_SUBSECTOR:
        len = 4 << 10;
        break;
    case ERASE_SECTOR:
        len = 32 << 10;
        break;
    case ERASE_BLOCK:
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
        qemu_log_mask(LOG_GUEST_ERROR,
                      "MX25U: Unknown cmd 0x%x\n", value);
    }
}

static void
mx25u_handle_cmd_data(Flash *s)
{
    s->current_address = (s->cmd_data[2] << 16) |
                         (s->cmd_data[1] << 8) |
                          s->cmd_data[0];
    s->state = STATE_IDLE;

    switch (s->cmd_in_progress) {
    case PAGE_PROGRAM:
    case QPAGE_PROGRAM:
        s->state = STATE_WRITE;
        break;
    case READ:
    case FAST_READ:
    case QREAD:
        DB_PRINT_L(1, "Read From: 0x%" PRIu64, s->current_address);
        s->state = STATE_READ;
        break;
    case ERASE_SUBSECTOR:
    case ERASE_SECTOR:
    case ERASE_BLOCK:
        mx25u_flash_erase(s, s->current_address, s->cmd_in_progress);
        s->SR |= R_SR_WIP;
        s->register_read_mask = R_SR_WIP;
        break;
    default:
        break;
    }
}

static void
mx25u_write8(Flash *s, uint8_t value)
{
    int64_t page = s->current_address / s->page_size;

    uint8_t current = s->storage[s->current_address];
    if (value & ~current) {
        qemu_log_mask(LOG_GUEST_ERROR, "MX25U: Flipping bit from 0 => 1\n");
        value &= current;
    }
    DB_PRINT_L(1, "Write 0x%" PRIx8 " = 0x%" PRIx64,
               (uint8_t)value, s->current_address);
    s->storage[s->current_address] = (uint8_t)value;

    flash_sync_dirty(s, page);
    s->dirty_page = page;
}

static uint32_t
mx25u_transfer8(SSIPeripheral *ss, uint32_t tx)
{
    Flash *s = MX25U(ss);
    uint32_t r = 0;

    switch (s->state) {
    case STATE_COLLECT_CMD_DATA:
        DB_PRINT_L(2, "Collected: 0x%" PRIx32, (uint32_t)tx);
        s->cmd_data[s->pos++] = (uint8_t)tx;
        if (s->pos == s->cmd_bytes) {
            mx25u_handle_cmd_data(s);
        }
        break;
    case STATE_WRITE:
        if (s->current_address > s->size) {
            qemu_log_mask(LOG_GUEST_ERROR,
                "MX25U: Out of bounds flash write to 0x%" PRIx64 "\n",
                s->current_address);
        } else {
            mx25u_write8(s, tx);
            s->current_address += 1;
        }
        break;
    case STATE_READ:
        if (s->current_address > s->size) {
            qemu_log_mask(LOG_GUEST_ERROR,
                "MX25U: Out of bounds flash read from 0x%" PRIx64 "\n",
                s->current_address);
        } else {
            r = s->storage[s->current_address];
            DB_PRINT_L(1, "Read 0x%" PRIx64 " = 0x%" PRIx8,
                        s->current_address, (uint8_t)r);
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
mx25u_cs(SSIPeripheral *ss, bool select)
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
mx25u_realize(SSIPeripheral *ss, Error **errp)
{
    Flash *s = MX25U(ss);

    s->state = STATE_IDLE;
    s->size = s->capacity;
    s->page_size = FLASH_PAGE_SIZE;
    s->dirty_page = -1;
    s->SR = 0;

    if (s->blk) {
        DB_PRINT_L(0, "Binding to block backend drive");

        /* Request block permissions */
        uint64_t perm = BLK_PERM_CONSISTENT_READ;
        if (blk_supports_write_perm(s->blk)) {
            perm |= BLK_PERM_WRITE;
        }
        if (blk_set_perm(s->blk, perm, BLK_PERM_ALL, errp) < 0) {
            return;
        }

        s->storage = blk_blockalign(s->blk, s->size);

        if (blk_pread(s->blk, 0, s->size, s->storage, 0) < 0) {
            error_setg(errp, "Failed to initialize SPI flash from drive");
            return;
        }
    } else {
        DB_PRINT_L(0, "No drive - binding to RAM");
        s->storage = blk_blockalign(NULL, s->size);
        memset(s->storage, 0xFF, s->size);
    }

    /* Create memory region backed by flash storage for memory-mapped reads.
     * Not marked read-only: firmware may write to this range (e.g. FMC init).
     * Direct writes update the buffer but don't sync to the block backend;
     * proper flash writes go through the QSPI controller instead. */
    memory_region_init_ram_ptr(&s->mmap, OBJECT(ss), "mx25u.mmap",
                                s->size, s->storage);
}

static int
mx25u_pre_save(void *opaque)
{
    flash_sync_dirty((Flash *)opaque, -1);
    return 0;
}

static const VMStateDescription vmstate_mx25u = {
    .name = "mx25u",
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = mx25u_pre_save,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static const Property mx25u_properties[] = {
    DEFINE_PROP_DRIVE("drive", Flash, blk),
    DEFINE_PROP_UINT32("capacity", Flash, capacity,
                       FLASH_SECTOR_SIZE * FLASH_NUM_SECTORS),
};

static void
mx25u_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = mx25u_realize;
    k->transfer = mx25u_transfer8;
    k->set_cs = mx25u_cs;
    k->cs_polarity = SSI_CS_LOW;
    dc->vmsd = &vmstate_mx25u;
    device_class_set_props(dc, mx25u_properties);
}

MemoryRegion *mx25u_get_mmap(DeviceState *dev)
{
    return &MX25U(dev)->mmap;
}

static const TypeInfo mx25u_info = {
    .name          = TYPE_MX25U,
    .parent        = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(Flash),
    .class_size    = sizeof(MX25UClass),
    .class_init    = mx25u_class_init,
    .abstract      = true,
};

static const TypeInfo mx25u6435f_info = {
    .name   = "mx25u6435f",
    .parent = TYPE_MX25U,
};

static void
mx25u_register_types(void)
{
    type_register_static(&mx25u_info);
    type_register_static(&mx25u6435f_info);
}

type_init(mx25u_register_types)
