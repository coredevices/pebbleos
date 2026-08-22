/*
 * Pebble Simple Timer
 *
 * Downcounting timer with interrupt for Pebble generic machines.
 *
 * Registers (0x1000 region):
 *   0x00 LOAD      - Counter reload value (ticks)
 *   0x04 VALUE     - Current counter value (read-only, counts down)
 *   0x08 CTRL      - Bit 0: enable, Bit 1: IRQ enable, Bit 2: one-shot
 *   0x0C INTSTAT   - Bit 0: timer fired (write 1 to clear)
 *   0x10 PRESCALER - Clock divider (timer_clk = sysclk / (prescaler + 1))
 *
 * When enabled, the timer counts down from LOAD. On reaching 0 it sets
 * the fired bit in INTSTAT. In periodic mode it reloads; in one-shot mode
 * it stops.
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
#include "hw/core/qdev-clock.h"

#define TYPE_PEBBLE_GENERIC_TIMER "pebble-timer"
OBJECT_DECLARE_SIMPLE_TYPE(PblTimer, PEBBLE_GENERIC_TIMER)

/* Register offsets */
#define TMR_LOAD      0x00
#define TMR_VALUE     0x04
#define TMR_CTRL      0x08
#define TMR_INTSTAT   0x0C
#define TMR_PRESCALER 0x10

/* CTRL bits */
#define CTRL_ENABLE   (1 << 0)
#define CTRL_IRQ_EN   (1 << 1)
#define CTRL_ONESHOT  (1 << 2)

/* INTSTAT bits */
#define INT_FIRED     (1 << 0)

struct PblTimer {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    Clock *clk;

    uint32_t load;
    uint32_t ctrl;
    uint32_t intstat;
    uint32_t prescaler;

    /* Timestamp when counter was last loaded/started */
    int64_t tick_start;
};

static void pbl_timer_update_irq(PblTimer *s)
{
    qemu_set_irq(s->irq, (s->ctrl & CTRL_IRQ_EN) && (s->intstat & INT_FIRED));
}

static int64_t pbl_timer_get_period_ns(PblTimer *s)
{
    uint64_t clk_hz = clock_get_hz(s->clk);
    if (clk_hz == 0 || s->load == 0) {
        return 0;
    }
    uint64_t div = (uint64_t)(s->prescaler + 1);
    /* period = load * div / clk_hz seconds = load * div * 1e9 / clk_hz ns */
    return muldiv64(s->load, div * NANOSECONDS_PER_SECOND, clk_hz);
}

static void pbl_timer_fire(void *opaque)
{
    PblTimer *s = opaque;

    s->intstat |= INT_FIRED;
    pbl_timer_update_irq(s);

    if (!(s->ctrl & CTRL_ONESHOT) && (s->ctrl & CTRL_ENABLE)) {
        /* Periodic: reload */
        int64_t period = pbl_timer_get_period_ns(s);
        if (period > 0) {
            s->tick_start = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            timer_mod(s->timer, s->tick_start + period);
        }
    } else {
        /* One-shot: stop */
        s->ctrl &= ~CTRL_ENABLE;
    }
}

static void pbl_timer_start(PblTimer *s)
{
    int64_t period = pbl_timer_get_period_ns(s);
    if (period > 0) {
        s->tick_start = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        timer_mod(s->timer, s->tick_start + period);
    }
}

static void pbl_timer_stop(PblTimer *s)
{
    timer_del(s->timer);
}

static uint32_t pbl_timer_get_value(PblTimer *s)
{
    if (!(s->ctrl & CTRL_ENABLE) || s->load == 0) {
        return 0;
    }

    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t elapsed_ns = now - s->tick_start;
    int64_t period_ns = pbl_timer_get_period_ns(s);

    if (period_ns <= 0) {
        return 0;
    }

    /* How many ticks have elapsed */
    uint64_t elapsed_ticks = muldiv64(elapsed_ns, s->load, period_ns);
    if (elapsed_ticks >= s->load) {
        return 0;
    }
    return s->load - (uint32_t)elapsed_ticks;
}

static uint64_t pbl_timer_read(void *opaque, hwaddr offset, unsigned size)
{
    PblTimer *s = opaque;

    switch (offset) {
    case TMR_LOAD:
        return s->load;
    case TMR_VALUE:
        return pbl_timer_get_value(s);
    case TMR_CTRL:
        return s->ctrl;
    case TMR_INTSTAT:
        return s->intstat;
    case TMR_PRESCALER:
        return s->prescaler;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pebble-timer: bad read offset 0x%" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
}

static void pbl_timer_write(void *opaque, hwaddr offset,
                             uint64_t value, unsigned size)
{
    PblTimer *s = opaque;

    switch (offset) {
    case TMR_LOAD:
        s->load = value;
        if (s->ctrl & CTRL_ENABLE) {
            pbl_timer_start(s);
        }
        break;

    case TMR_CTRL:
    {
        uint32_t old = s->ctrl;
        s->ctrl = value & (CTRL_ENABLE | CTRL_IRQ_EN | CTRL_ONESHOT);
        if ((s->ctrl & CTRL_ENABLE) && !(old & CTRL_ENABLE)) {
            pbl_timer_start(s);
        } else if (!(s->ctrl & CTRL_ENABLE) && (old & CTRL_ENABLE)) {
            pbl_timer_stop(s);
        }
        pbl_timer_update_irq(s);
        break;
    }

    case TMR_INTSTAT:
        /* Write 1 to clear */
        s->intstat &= ~value;
        pbl_timer_update_irq(s);
        break;

    case TMR_PRESCALER:
        s->prescaler = value;
        if (s->ctrl & CTRL_ENABLE) {
            pbl_timer_start(s);
        }
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pebble-timer: bad write offset 0x%" HWADDR_PRIx "\n",
                      offset);
        break;
    }
}

static const MemoryRegionOps pbl_timer_ops = {
    .read = pbl_timer_read,
    .write = pbl_timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void pbl_timer_init(Object *obj)
{
    PblTimer *s = PEBBLE_GENERIC_TIMER(obj);

    memory_region_init_io(&s->iomem, obj, &pbl_timer_ops, s,
                          "pebble-timer", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pbl_timer_fire, s);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", NULL, NULL, 0);
}

static void pbl_timer_reset(DeviceState *dev)
{
    PblTimer *s = PEBBLE_GENERIC_TIMER(dev);

    timer_del(s->timer);
    s->load = 0;
    s->ctrl = 0;
    s->intstat = 0;
    s->prescaler = 0;
    s->tick_start = 0;
}

static void pbl_timer_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, pbl_timer_reset);
}

static const TypeInfo pbl_timer_info = {
    .name          = TYPE_PEBBLE_GENERIC_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PblTimer),
    .instance_init = pbl_timer_init,
    .class_init    = pbl_timer_class_init,
};

static void pbl_timer_register_types(void)
{
    type_register_static(&pbl_timer_info);
}

type_init(pbl_timer_register_types)
