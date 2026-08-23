/*
 * Pebble Touch Controller
 *
 * Simple touch input device for Pebble generic machines (Emery, Gabbro).
 * Receives mouse events from QemuConsole and provides touch coordinates.
 *
 * Registers (0x1000 region):
 *   0x00 STATE   - Bit 0: finger down
 *   0x04 X       - Touch X coordinate (16-bit)
 *   0x08 Y       - Touch Y coordinate (16-bit)
 *   0x0C INTCTRL - Bit 0: touch event IRQ enable
 *   0x10 INTSTAT - Bit 0: touch event occurred (write 1 to clear)
 *
 * Copyright (c) 2026 Core Devices LLC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "ui/console.h"
#include "ui/input.h"

#ifdef EMSCRIPTEN
#include <emscripten/emscripten.h>
#endif

#define TYPE_PEBBLE_TOUCH "pebble-touch"
OBJECT_DECLARE_SIMPLE_TYPE(PblTouch, PEBBLE_TOUCH)

/* Register offsets */
#define TOUCH_STATE    0x00
#define TOUCH_X        0x04
#define TOUCH_Y        0x08
#define TOUCH_INTCTRL  0x0C
#define TOUCH_INTSTAT  0x10

#define INT_TOUCH_EVENT (1 << 0)

struct PblTouch {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t state;     /* bit 0: finger down */
    uint32_t x;
    uint32_t y;
    uint32_t intctrl;
    uint32_t intstat;

    uint32_t display_w;
    uint32_t display_h;

    QemuInputHandlerState *input_handler;

#ifdef EMSCRIPTEN
    QEMUTimer *wasm_poll_timer;
    uint32_t wasm_last_seq;
#endif
};

#ifdef EMSCRIPTEN
/* Browser touch injection: the page writes display-pixel coordinates
 * and the finger state, then bumps seq with Atomics; an 8 ms
 * virtual-clock poll applies changes exactly like input-layer events. */
static struct {
    uint32_t seq;
    uint32_t down;
    uint32_t x;
    uint32_t y;
} s_wasm_touch;

EMSCRIPTEN_KEEPALIVE void *pebble_wasm_touch_ctrl(void)
{
    return &s_wasm_touch;
}
#endif

static void pbl_touch_update_irq(PblTouch *s)
{
    qemu_set_irq(s->irq, (s->intctrl & INT_TOUCH_EVENT) &&
                          (s->intstat & INT_TOUCH_EVENT));
}

static void pbl_touch_input_event(DeviceState *dev, QemuConsole *src,
                                   InputEvent *evt)
{
    PblTouch *s = PEBBLE_TOUCH(dev);

    if (evt->type == INPUT_EVENT_KIND_BTN) {
        InputBtnEvent *btn = evt->u.btn.data;
        if (btn->button == INPUT_BUTTON_LEFT) {
            if (btn->down) {
                s->state = 1;
            } else {
                s->state = 0;
            }
            s->intstat |= INT_TOUCH_EVENT;
            pbl_touch_update_irq(s);
        }
    } else if (evt->type == INPUT_EVENT_KIND_ABS) {
        InputMoveEvent *move = evt->u.abs.data;
        if (move->axis == INPUT_AXIS_X) {
            s->x = move->value * s->display_w / INPUT_EVENT_ABS_MAX;
        } else if (move->axis == INPUT_AXIS_Y) {
            s->y = move->value * s->display_h / INPUT_EVENT_ABS_MAX;
        }
        if (s->state) {
            s->intstat |= INT_TOUCH_EVENT;
            pbl_touch_update_irq(s);
        }
    }
}

static const QemuInputHandler pbl_touch_input_handler = {
    .name  = "Pebble Touch",
    .mask  = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = pbl_touch_input_event,
};

#ifdef EMSCRIPTEN
static void pbl_touch_wasm_poll(void *opaque)
{
    PblTouch *s = opaque;
    uint32_t seq = qatomic_load_acquire(&s_wasm_touch.seq);

    if (seq != s->wasm_last_seq) {
        s->wasm_last_seq = seq;
        uint32_t down = s_wasm_touch.down ? 1 : 0;
        uint32_t x = MIN(s_wasm_touch.x, s->display_w - 1);
        uint32_t y = MIN(s_wasm_touch.y, s->display_h - 1);
        bool changed = (down != s->state) ||
                       (down && (x != s->x || y != s->y));
        s->x = x;
        s->y = y;
        s->state = down;
        if (changed) {
            s->intstat |= INT_TOUCH_EVENT;
            pbl_touch_update_irq(s);
        }
    }
    timer_mod(s->wasm_poll_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 8);
}
#endif

/* === Register access === */

static uint64_t pbl_touch_read(void *opaque, hwaddr offset, unsigned size)
{
    PblTouch *s = opaque;

    switch (offset) {
    case TOUCH_STATE:
        return s->state;
    case TOUCH_X:
        return s->x;
    case TOUCH_Y:
        return s->y;
    case TOUCH_INTCTRL:
        return s->intctrl;
    case TOUCH_INTSTAT:
        return s->intstat;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pebble-touch: bad read offset 0x%" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
}

static void pbl_touch_write(void *opaque, hwaddr offset,
                             uint64_t value, unsigned size)
{
    PblTouch *s = opaque;

    switch (offset) {
    case TOUCH_INTCTRL:
        s->intctrl = value & INT_TOUCH_EVENT;
        pbl_touch_update_irq(s);
        break;
    case TOUCH_INTSTAT:
        s->intstat &= ~value;
        pbl_touch_update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pebble-touch: bad write offset 0x%" HWADDR_PRIx "\n",
                      offset);
        break;
    }
}

static const MemoryRegionOps pbl_touch_ops = {
    .read = pbl_touch_read,
    .write = pbl_touch_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void pbl_touch_realize(DeviceState *dev, Error **errp)
{
    PblTouch *s = PEBBLE_TOUCH(dev);

    s->input_handler = qemu_input_handler_register(dev,
                                                    &pbl_touch_input_handler);
    qemu_input_handler_activate(s->input_handler);

#ifdef EMSCRIPTEN
    s->wasm_poll_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                      pbl_touch_wasm_poll, s);
    timer_mod(s->wasm_poll_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 8);
#endif
}

static void pbl_touch_init(Object *obj)
{
    PblTouch *s = PEBBLE_TOUCH(obj);

    memory_region_init_io(&s->iomem, obj, &pbl_touch_ops, s,
                          "pebble-touch", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void pbl_touch_reset(DeviceState *dev)
{
    PblTouch *s = PEBBLE_TOUCH(dev);

    s->state = 0;
    s->x = 0;
    s->y = 0;
    s->intctrl = 0;
    s->intstat = 0;
}

static const Property pbl_touch_properties[] = {
    DEFINE_PROP_UINT32("display-width", PblTouch, display_w, 200),
    DEFINE_PROP_UINT32("display-height", PblTouch, display_h, 228),
};

static void pbl_touch_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = pbl_touch_realize;
    device_class_set_legacy_reset(dc, pbl_touch_reset);
    device_class_set_props(dc, pbl_touch_properties);
}

static const TypeInfo pbl_touch_info = {
    .name          = TYPE_PEBBLE_TOUCH,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PblTouch),
    .instance_init = pbl_touch_init,
    .class_init    = pbl_touch_class_init,
};

static void pbl_touch_register_types(void)
{
    type_register_static(&pbl_touch_info);
}

type_init(pbl_touch_register_types)
