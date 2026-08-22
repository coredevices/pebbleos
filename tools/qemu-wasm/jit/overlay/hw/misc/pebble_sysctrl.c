/*
 * Pebble System Control
 *
 * Simple system control peripheral for Pebble generic machines.
 * Provides board identification and feature flags.
 *
 * Registers (0x1000 region):
 *   0x00 BOARD_ID  - Board identifier (read-only)
 *   0x04 FEATURES  - Feature flags (read-only)
 *                    Bit 0: has touch
 *                    Bit 1: has audio
 *                    Bit 2: round display
 *   0x08 DISPLAY_W - Display width (read-only)
 *   0x0C DISPLAY_H - Display height (read-only)
 *   0x10 DISPLAY_F - Display format: bpp (read-only)
 *
 * Copyright (c) 2026 Core Devices LLC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"

#define TYPE_PEBBLE_SYSCTRL "pebble-sysctrl"
OBJECT_DECLARE_SIMPLE_TYPE(PblSysCtrl, PEBBLE_SYSCTRL)

/* Register offsets */
#define SYSCTRL_BOARD_ID    0x00
#define SYSCTRL_FEATURES    0x04
#define SYSCTRL_DISPLAY_W   0x08
#define SYSCTRL_DISPLAY_H   0x0C
#define SYSCTRL_DISPLAY_F   0x10

/* Feature bits */
#define FEAT_TOUCH  (1 << 0)
#define FEAT_AUDIO  (1 << 1)
#define FEAT_ROUND  (1 << 2)

struct PblSysCtrl {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    uint32_t board_id;
    uint32_t features;
    uint32_t display_w;
    uint32_t display_h;
    uint32_t display_fmt;
};

static uint64_t pbl_sysctrl_read(void *opaque, hwaddr offset, unsigned size)
{
    PblSysCtrl *s = opaque;

    switch (offset) {
    case SYSCTRL_BOARD_ID:
        return s->board_id;
    case SYSCTRL_FEATURES:
        return s->features;
    case SYSCTRL_DISPLAY_W:
        return s->display_w;
    case SYSCTRL_DISPLAY_H:
        return s->display_h;
    case SYSCTRL_DISPLAY_F:
        return s->display_fmt;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pebble-sysctrl: bad read offset 0x%" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
}

static void pbl_sysctrl_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    qemu_log_mask(LOG_GUEST_ERROR,
                  "pebble-sysctrl: write to read-only register 0x%" HWADDR_PRIx "\n",
                  offset);
}

static const MemoryRegionOps pbl_sysctrl_ops = {
    .read = pbl_sysctrl_read,
    .write = pbl_sysctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void pbl_sysctrl_init(Object *obj)
{
    PblSysCtrl *s = PEBBLE_SYSCTRL(obj);

    memory_region_init_io(&s->iomem, obj, &pbl_sysctrl_ops, s,
                          "pebble-sysctrl", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const Property pbl_sysctrl_properties[] = {
    DEFINE_PROP_UINT32("board-id", PblSysCtrl, board_id, 0),
    DEFINE_PROP_UINT32("features", PblSysCtrl, features, 0),
    DEFINE_PROP_UINT32("display-width", PblSysCtrl, display_w, 0),
    DEFINE_PROP_UINT32("display-height", PblSysCtrl, display_h, 0),
    DEFINE_PROP_UINT32("display-format", PblSysCtrl, display_fmt, 0),
};

static void pbl_sysctrl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_props(dc, pbl_sysctrl_properties);
}

static const TypeInfo pbl_sysctrl_info = {
    .name          = TYPE_PEBBLE_SYSCTRL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PblSysCtrl),
    .instance_init = pbl_sysctrl_init,
    .class_init    = pbl_sysctrl_class_init,
};

static void pbl_sysctrl_register_types(void)
{
    type_register_static(&pbl_sysctrl_info);
}

type_init(pbl_sysctrl_register_types)
