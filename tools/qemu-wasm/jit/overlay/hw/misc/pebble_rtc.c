/*
 * Pebble Generic RTC
 *
 * Simple RTC with backup registers for Pebble generic machines.
 * Provides host time and backup registers for QEMU settings.
 *
 * Registers (0x1000 region):
 *   0x00 TIME_LO  - Unix timestamp low 32 bits (read: current time)
 *   0x04 TIME_HI  - Unix timestamp high 32 bits
 *   0x08 ALARM    - Alarm time (low 32 bits)
 *   0x0C CTRL     - Bit 0: alarm enable, Bit 1: alarm IRQ pending (w1c)
 *   0x40..0x7F    - Backup registers (16 x 32-bit, read/write, persist across reset)
 *
 * Copyright (c) 2026 Core Devices LLC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"

#define TYPE_PEBBLE_GENERIC_RTC "pebble-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(PblRtc, PEBBLE_GENERIC_RTC)

/* Register offsets */
#define RTC_TIME_LO     0x00
#define RTC_TIME_HI     0x04
#define RTC_ALARM       0x08
#define RTC_CTRL        0x0C
#define RTC_TICKS       0x10  /* Monotonic 1000Hz tick counter */

#define RTC_BACKUP_BASE 0x40
#define RTC_BACKUP_END  0x80
#define RTC_NUM_BACKUP  16

/* CTRL bits */
#define CTRL_ALARM_EN   (1 << 0)
#define CTRL_ALARM_IRQ  (1 << 1)

struct PblRtc {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    /* Offset (in seconds) added to host wall-clock to produce the RTC time.
     * The guest sets the RTC by writing TIME_LO/TIME_HI; we record an offset
     * rather than a base so the clock keeps ticking after the write. */
    int64_t time_offset_s;

    uint32_t alarm;
    uint32_t ctrl;
    uint32_t backup[RTC_NUM_BACKUP];
};

static void pbl_rtc_update_irq(PblRtc *s)
{
    qemu_set_irq(s->irq, (s->ctrl & CTRL_ALARM_EN) &&
                          (s->ctrl & CTRL_ALARM_IRQ));
}

/* Return the host wall-clock time, in seconds, as a UTC unix timestamp. */
static int64_t pbl_rtc_host_time(void)
{
    return qemu_clock_get_ms(QEMU_CLOCK_HOST) / 1000;
}

/* Return the RTC's current time: host wall-clock plus the guest-set offset.
 * Guests running on the generic Pebble machine are expected to apply their
 * own timezone conversion. */
static uint64_t pbl_rtc_get_time(PblRtc *s)
{
    return (uint64_t)(pbl_rtc_host_time() + s->time_offset_s);
}

static uint64_t pbl_rtc_read(void *opaque, hwaddr offset, unsigned size)
{
    PblRtc *s = opaque;
    uint64_t now;

    switch (offset) {
    case RTC_TIME_LO:
        now = pbl_rtc_get_time(s);
        return (uint32_t)now;

    case RTC_TIME_HI:
        now = pbl_rtc_get_time(s);
        return (uint32_t)(now >> 32);

    case RTC_ALARM:
        return s->alarm;

    case RTC_CTRL:
        return s->ctrl;

    case RTC_TICKS:
        /* Monotonic millisecond counter (1000Hz ticks) */
        return (uint32_t)qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);

    default:
        if (offset >= RTC_BACKUP_BASE && offset < RTC_BACKUP_END) {
            int idx = (offset - RTC_BACKUP_BASE) / 4;
            return s->backup[idx];
        }
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pebble-rtc: bad read offset 0x%" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
}

static void pbl_rtc_write(void *opaque, hwaddr offset,
                           uint64_t value, unsigned size)
{
    PblRtc *s = opaque;
    uint64_t target;

    switch (offset) {
    case RTC_TIME_LO:
        /* Pebble's time_t is 32 bits, so the firmware only writes TIME_LO.
         * Treat it as the low 32 bits of the desired UTC timestamp, preserving
         * whatever the guest last wrote to TIME_HI (zero by default). */
        target = ((uint64_t)pbl_rtc_get_time(s) & ~0xFFFFFFFFULL) |
                 (uint32_t)value;
        s->time_offset_s = (int64_t)target - pbl_rtc_host_time();
        break;

    case RTC_TIME_HI:
        target = ((uint64_t)pbl_rtc_get_time(s) & 0xFFFFFFFFULL) |
                 ((uint64_t)(uint32_t)value << 32);
        s->time_offset_s = (int64_t)target - pbl_rtc_host_time();
        break;

    case RTC_ALARM:
        s->alarm = value;
        break;

    case RTC_CTRL:
        /* Write 1 to clear IRQ pending bit */
        if (value & CTRL_ALARM_IRQ) {
            s->ctrl &= ~CTRL_ALARM_IRQ;
        }
        /* Update enable bit */
        s->ctrl = (s->ctrl & CTRL_ALARM_IRQ) | (value & CTRL_ALARM_EN);
        pbl_rtc_update_irq(s);
        break;

    default:
        if (offset >= RTC_BACKUP_BASE && offset < RTC_BACKUP_END) {
            int idx = (offset - RTC_BACKUP_BASE) / 4;
            s->backup[idx] = value;
            return;
        }
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pebble-rtc: bad write offset 0x%" HWADDR_PRIx "\n",
                      offset);
        break;
    }
}

static const MemoryRegionOps pbl_rtc_ops = {
    .read = pbl_rtc_read,
    .write = pbl_rtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void pbl_rtc_init(Object *obj)
{
    PblRtc *s = PEBBLE_GENERIC_RTC(obj);

    memory_region_init_io(&s->iomem, obj, &pbl_rtc_ops, s,
                          "pebble-rtc", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void pbl_rtc_reset(DeviceState *dev)
{
    PblRtc *s = PEBBLE_GENERIC_RTC(dev);

    s->alarm = 0;
    s->ctrl = 0;
    s->time_offset_s = 0;
    /* Backup registers intentionally NOT cleared on reset */
}

static void pbl_rtc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, pbl_rtc_reset);
}

static const TypeInfo pbl_rtc_info = {
    .name          = TYPE_PEBBLE_GENERIC_RTC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PblRtc),
    .instance_init = pbl_rtc_init,
    .class_init    = pbl_rtc_class_init,
};

static void pbl_rtc_register_types(void)
{
    type_register_static(&pbl_rtc_info);
}

type_init(pbl_rtc_register_types)
