/*
 * Pebble Null Input Device
 *
 * Registers a no-op absolute-pointer input handler. Its sole purpose is to
 * make qemu_input_is_absolute() return true for non-touch Pebble machines,
 * which prevents the QEMU UI frontend from grabbing the host cursor on
 * click. All events are dropped.
 *
 * Copyright (c) 2026 Core Devices LLC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/qdev.h"
#include "ui/console.h"
#include "ui/input.h"
#include "qom/object.h"
#include "hw/misc/pebble_null_input.h"

OBJECT_DECLARE_SIMPLE_TYPE(PblNullInput, PEBBLE_NULL_INPUT)

struct PblNullInput {
    DeviceState parent_obj;

    QemuInputHandlerState *input_handler;
};

static void pbl_null_input_event(DeviceState *dev, QemuConsole *src,
                                  InputEvent *evt)
{
    /* Intentionally empty: absolute pointer events are dropped. */
}

static const QemuInputHandler pbl_null_input_handler = {
    .name  = "Pebble Null Input",
    .mask  = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = pbl_null_input_event,
};

static void pbl_null_input_realize(DeviceState *dev, Error **errp)
{
    PblNullInput *s = PEBBLE_NULL_INPUT(dev);

    s->input_handler = qemu_input_handler_register(dev,
                                                    &pbl_null_input_handler);
    qemu_input_handler_activate(s->input_handler);
}

static void pbl_null_input_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = pbl_null_input_realize;
}

static const TypeInfo pbl_null_input_info = {
    .name          = TYPE_PEBBLE_NULL_INPUT,
    .parent        = TYPE_DEVICE,
    .instance_size = sizeof(PblNullInput),
    .class_init    = pbl_null_input_class_init,
};

static void pbl_null_input_register_types(void)
{
    type_register_static(&pbl_null_input_info);
}

type_init(pbl_null_input_register_types)
