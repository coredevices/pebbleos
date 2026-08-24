/*
 * Pebble Generic GPIO (Buttons)
 *
 * Simple button input device for Pebble generic machines.
 * Provides 4 buttons (Back, Up, Select, Down) with edge interrupts.
 * Registers a keyboard input handler for host-side button simulation.
 *
 * Keyboard mapping:
 *   Q / Left arrow  → Back   (bit 0)
 *   W / Up arrow    → Up     (bit 1)
 *   S / Right arrow → Select (bit 2)
 *   X / Down arrow  → Down   (bit 3)
 *
 * Registers (0x1000 region):
 *   0x00 BTN_STATE - Current button state (bit per button, 1=pressed)
 *   0x04 BTN_EDGE  - Edge flags (set on press/release, write 1 to clear)
 *   0x08 INTCTRL   - Bit 0: edge IRQ enable
 *   0x0C INTSTAT   - Alias for BTN_EDGE (read), write 1 to clear
 *
 * Copyright (c) 2026 Core Devices LLC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "ui/console.h"
#include "ui/input.h"
#include "hw/arm/pebble_gpio.h"

#define TYPE_PEBBLE_GPIO "pebble-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(PblGpio, PEBBLE_GPIO)

/* Register offsets */
#define GPIO_BTN_STATE  0x00
#define GPIO_BTN_EDGE   0x04
#define GPIO_INTCTRL    0x08
#define GPIO_INTSTAT    0x0C

/* Button bit positions */
#define BTN_BACK    (1 << 0)
#define BTN_UP      (1 << 1)
#define BTN_SELECT  (1 << 2)
#define BTN_DOWN    (1 << 3)
#define BTN_ALL     (BTN_BACK | BTN_UP | BTN_SELECT | BTN_DOWN)

/* Debounce time in ms */
#define DEBOUNCE_MS 250

struct PblGpio {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t btn_state;   /* current button state */
    uint32_t btn_edge;    /* edge detection flags */
    uint32_t intctrl;

    QEMUTimer *debounce_timer;
    uint32_t pending_release;  /* buttons waiting to be released */

    QemuInputHandlerState *input_handler;
};

static void pbl_gpio_update_irq(PblGpio *s)
{
    qemu_set_irq(s->irq, (s->intctrl & 1) && (s->btn_edge != 0));
}

static int pbl_gpio_qcode_to_button(int qcode)
{
    switch (qcode) {
    case Q_KEY_CODE_Q:
    case Q_KEY_CODE_LEFT:
        return BTN_BACK;
    case Q_KEY_CODE_W:
    case Q_KEY_CODE_UP:
        return BTN_UP;
    case Q_KEY_CODE_S:
    case Q_KEY_CODE_RIGHT:
        return BTN_SELECT;
    case Q_KEY_CODE_X:
    case Q_KEY_CODE_DOWN:
        return BTN_DOWN;
    default:
        return 0;
    }
}

static void pbl_gpio_debounce_cb(void *opaque)
{
    PblGpio *s = opaque;
    uint32_t released = s->pending_release;

    if (released) {
        s->btn_state &= ~released;
        s->btn_edge |= released;
        s->pending_release = 0;
        pbl_gpio_update_irq(s);
    }
}

static void pbl_gpio_input_event(DeviceState *dev, QemuConsole *src,
                                  InputEvent *evt)
{
    PblGpio *s = PEBBLE_GPIO(dev);
    InputKeyEvent *key;
    int qcode, btn;

    if (evt->type != INPUT_EVENT_KIND_KEY) {
        return;
    }

    key = evt->u.key.data;
    qcode = qemu_input_key_value_to_qcode(key->key);
    btn = pbl_gpio_qcode_to_button(qcode);

    if (btn == 0 || !key->down) {
        return;
    }

    /* Release any previously pressed button first */
    if (s->pending_release && s->pending_release != (uint32_t)btn) {
        s->btn_state &= ~s->pending_release;
        s->btn_edge |= s->pending_release;
        s->pending_release = 0;
    }

    /* Press the button */
    if (!(s->btn_state & btn)) {
        s->btn_state |= btn;
        s->btn_edge |= btn;
        pbl_gpio_update_irq(s);
    }

    /* Schedule release after debounce period */
    s->pending_release = btn;
    timer_mod(s->debounce_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + DEBOUNCE_MS);
}

static const QemuInputHandler pbl_gpio_input_handler = {
    .name  = "Pebble Buttons",
    .mask  = INPUT_EVENT_MASK_KEY,
    .event = pbl_gpio_input_event,
};

#ifdef __EMSCRIPTEN__
#include <emscripten.h>

/* Browser bridge: the web shell writes a button bitmask into shared memory
 * (Atomics.store) and a virtual-clock timer applies the press/release edges,
 * avoiding cross-thread calls into QEMU under PROXY_TO_PTHREAD. Unlike the
 * keyboard path there is no auto-release, so holds work. */
static uint32_t s_wasm_btn_state;
static uint32_t s_wasm_btn_last;
static QEMUTimer *s_wasm_btn_timer;

EMSCRIPTEN_KEEPALIVE uint32_t pebble_wasm_button_state_addr(void)
{
    return (uint32_t)(uintptr_t)&s_wasm_btn_state;
}

static void pbl_gpio_wasm_poll(void *opaque)
{
    PblGpio *s = opaque;
    uint32_t state = __atomic_load_n(&s_wasm_btn_state,
                                     __ATOMIC_SEQ_CST) & BTN_ALL;

    if (state != s_wasm_btn_last) {
        s->pending_release = 0;
        s->btn_edge |= state ^ s_wasm_btn_last;
        s->btn_state = state;
        s_wasm_btn_last = state;
        pbl_gpio_update_irq(s);
    }
    timer_mod(s_wasm_btn_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 16);
}
#endif

/* === Register access === */

static uint64_t pbl_gpio_read(void *opaque, hwaddr offset, unsigned size)
{
    PblGpio *s = opaque;

    switch (offset) {
    case GPIO_BTN_STATE:
        return s->btn_state;
    case GPIO_BTN_EDGE:
    case GPIO_INTSTAT:
        return s->btn_edge;
    case GPIO_INTCTRL:
        return s->intctrl;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pebble-gpio: bad read offset 0x%" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
}

static void pbl_gpio_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    PblGpio *s = opaque;

    switch (offset) {
    case GPIO_BTN_EDGE:
    case GPIO_INTSTAT:
        /* Write 1 to clear edge flags */
        s->btn_edge &= ~(value & BTN_ALL);
        pbl_gpio_update_irq(s);
        break;
    case GPIO_INTCTRL:
        s->intctrl = value & 1;
        pbl_gpio_update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pebble-gpio: bad write offset 0x%" HWADDR_PRIx "\n",
                      offset);
        break;
    }
}

static const MemoryRegionOps pbl_gpio_ops = {
    .read = pbl_gpio_read,
    .write = pbl_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* === Device lifecycle === */

/* Global instance pointer (set on realize, only one GPIO device per machine) */
static PblGpio *s_pbl_gpio_instance;

static void pbl_gpio_realize(DeviceState *dev, Error **errp)
{
    PblGpio *s = PEBBLE_GPIO(dev);

    s->debounce_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                      pbl_gpio_debounce_cb, s);

    s->input_handler = qemu_input_handler_register(dev,
                                                    &pbl_gpio_input_handler);
    qemu_input_handler_activate(s->input_handler);

    s_pbl_gpio_instance = s;

#ifdef __EMSCRIPTEN__
    s_wasm_btn_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, pbl_gpio_wasm_poll, s);
    timer_mod(s_wasm_btn_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 16);
#endif
}

static void pbl_gpio_init(Object *obj)
{
    PblGpio *s = PEBBLE_GPIO(obj);

    memory_region_init_io(&s->iomem, obj, &pbl_gpio_ops, s,
                          "pebble-gpio", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void pbl_gpio_reset(DeviceState *dev)
{
    PblGpio *s = PEBBLE_GPIO(dev);

    s->btn_state = 0;
    s->btn_edge = 0;
    s->intctrl = 0;
    s->pending_release = 0;
    if (s->debounce_timer) {
        timer_del(s->debounce_timer);
    }
}

static void pbl_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = pbl_gpio_realize;
    device_class_set_legacy_reset(dc, pbl_gpio_reset);
}

/* === Public API for pebble_control === */

void pbl_gpio_set_button_state(uint32_t button_state)
{
    PblGpio *s = s_pbl_gpio_instance;
    if (!s) {
        return;
    }

    uint32_t new_state = button_state & BTN_ALL;
    uint32_t changed = s->btn_state ^ new_state;

    if (changed) {
        s->btn_state = new_state;
        s->btn_edge |= changed;
        pbl_gpio_update_irq(s);
    }
}

static const TypeInfo pbl_gpio_info = {
    .name          = TYPE_PEBBLE_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PblGpio),
    .instance_init = pbl_gpio_init,
    .class_init    = pbl_gpio_class_init,
};

static void pbl_gpio_register_types(void)
{
    type_register_static(&pbl_gpio_info);
}

type_init(pbl_gpio_register_types)
