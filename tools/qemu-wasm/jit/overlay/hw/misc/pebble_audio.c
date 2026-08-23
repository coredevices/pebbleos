/*
 * Pebble Audio DAC (drain-only stub)
 *
 * Register-compatible with the coredevices pebble-audio device, but does
 * not produce host audio: samples pushed via DATA are consumed from the
 * ring at the programmed sample rate by a virtual-clock timer, and the
 * BUFAVAIL IRQ paces the firmware exactly like the real device. This
 * keeps the emery/flint machines bootable in builds where the QEMU audio
 * subsystem is unavailable (the wasm JIT build).
 *
 * Registers (0x1000 region):
 *   0x00 CTRL       - Bit 0: enable
 *   0x04 STATUS     - Bit 0: FIFO ready (always 1)
 *   0x08 SAMPLERATE - Sample rate in Hz
 *   0x0C DATA       - Write: push 16-bit PCM sample
 *   0x10 INTCTRL    - Bit 0: buffer-available IRQ enable
 *   0x14 INTSTAT    - Bit 0: buffer-available IRQ pending (write 1 to clear)
 *   0x18 BUFAVAIL   - Read: number of free samples in ring buffer
 *   0x1C VOLUME     - Volume 0-100
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
#include "hw/core/qdev-properties-system.h"

#ifdef EMSCRIPTEN
#include <emscripten/emscripten.h>
#endif

#define TYPE_PEBBLE_AUDIO "pebble-audio"
OBJECT_DECLARE_SIMPLE_TYPE(PblAudio, PEBBLE_AUDIO)

#define AUDIO_CTRL       0x00
#define AUDIO_STATUS     0x04
#define AUDIO_SAMPLERATE 0x08
#define AUDIO_DATA       0x0C
#define AUDIO_INTCTRL    0x10
#define AUDIO_INTSTAT    0x14
#define AUDIO_BUFAVAIL   0x18
#define AUDIO_VOLUME     0x1C

#define INT_BUFAVAIL     (1 << 0)

#define RING_BUF_SAMPLES 4096
#define DRAIN_INTERVAL_NS (10 * 1000 * 1000)
#define IRQ_FREE_THRESHOLD 1024

struct PblAudio {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t ctrl;
    uint32_t samplerate;
    uint32_t intctrl;
    uint32_t intstat;
    uint32_t volume;

    uint32_t ring_count;

    QEMUTimer *drain_timer;
    bool running;

    char *audiodev; /* accepted and ignored */
};

#ifdef EMSCRIPTEN
/* Browser playback: every sample the firmware pushes via DATA is also
 * appended to this ring; the page drains it into WebAudio. Head/tail
 * are free-running sample counters (index = counter % size). rate,
 * volume and playing mirror the device registers so the page can
 * (re)configure its AudioContext. */
#define WASM_AUDIO_RING 32768

static int16_t s_wasm_audio_buf[WASM_AUDIO_RING];
static struct {
    uint32_t buf, size;
    uint32_t head, tail;
    uint32_t rate, volume, playing;
} s_wasm_audio_ctrl;

EMSCRIPTEN_KEEPALIVE void *pebble_wasm_audio_ctrl(void)
{
    s_wasm_audio_ctrl.buf = (uint32_t)(uintptr_t)s_wasm_audio_buf;
    s_wasm_audio_ctrl.size = WASM_AUDIO_RING;
    return &s_wasm_audio_ctrl;
}

static void pbl_audio_wasm_push(int16_t sample)
{
    uint32_t head = s_wasm_audio_ctrl.head;
    if (head - qatomic_load_acquire(&s_wasm_audio_ctrl.tail) >= WASM_AUDIO_RING) {
        return; /* page not draining — drop */
    }
    s_wasm_audio_buf[head % WASM_AUDIO_RING] = sample;
    qatomic_store_release(&s_wasm_audio_ctrl.head, head + 1);
}
#endif

static void pbl_audio_update_irq(PblAudio *s)
{
    qemu_set_irq(s->irq, (s->intstat & s->intctrl) != 0);
}

static uint32_t pbl_audio_ring_free(PblAudio *s)
{
    return RING_BUF_SAMPLES - s->ring_count;
}

static void pbl_audio_drain_tick(void *opaque)
{
    PblAudio *s = opaque;
    uint32_t rate = s->samplerate ? s->samplerate : 16000;
    uint32_t drain = rate / 100; /* samples per 10 ms */

    if (drain == 0) {
        drain = 1;
    }
    if (drain > s->ring_count) {
        drain = s->ring_count;
    }
    s->ring_count -= drain;

    if (pbl_audio_ring_free(s) >= IRQ_FREE_THRESHOLD) {
        s->intstat |= INT_BUFAVAIL;
        pbl_audio_update_irq(s);
    }

    if (s->running) {
        timer_mod(s->drain_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + DRAIN_INTERVAL_NS);
    }
}

static void pbl_audio_start(PblAudio *s)
{
    if (s->running) {
        return;
    }
    s->running = true;
    timer_mod(s->drain_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + DRAIN_INTERVAL_NS);
}

static void pbl_audio_stop(PblAudio *s)
{
    s->running = false;
    timer_del(s->drain_timer);
    s->ring_count = 0;
}

static uint64_t pbl_audio_read(void *opaque, hwaddr offset, unsigned size)
{
    PblAudio *s = opaque;

    switch (offset) {
    case AUDIO_CTRL:
        return s->ctrl;
    case AUDIO_STATUS:
        return 1; /* FIFO always ready */
    case AUDIO_SAMPLERATE:
        return s->samplerate;
    case AUDIO_INTCTRL:
        return s->intctrl;
    case AUDIO_INTSTAT:
        return s->intstat;
    case AUDIO_BUFAVAIL:
        return pbl_audio_ring_free(s);
    case AUDIO_VOLUME:
        return s->volume;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pebble-audio: bad read offset 0x%" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
}

static void pbl_audio_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    PblAudio *s = opaque;

    switch (offset) {
    case AUDIO_CTRL:
        s->ctrl = value & 1;
        if (s->ctrl & 1) {
            pbl_audio_start(s);
        } else {
            pbl_audio_stop(s);
        }
#ifdef EMSCRIPTEN
        s_wasm_audio_ctrl.playing = s->ctrl & 1;
#endif
        break;
    case AUDIO_SAMPLERATE:
        s->samplerate = value;
#ifdef EMSCRIPTEN
        s_wasm_audio_ctrl.rate = s->samplerate;
#endif
        break;
    case AUDIO_DATA:
        if (s->ring_count < RING_BUF_SAMPLES) {
            s->ring_count++;
        }
#ifdef EMSCRIPTEN
        pbl_audio_wasm_push((int16_t)(value & 0xffff));
#endif
        break;
    case AUDIO_INTCTRL:
        s->intctrl = value & INT_BUFAVAIL;
        pbl_audio_update_irq(s);
        break;
    case AUDIO_INTSTAT:
        s->intstat &= ~value;
        pbl_audio_update_irq(s);
        break;
    case AUDIO_VOLUME:
        s->volume = value & 0x7F;
#ifdef EMSCRIPTEN
        s_wasm_audio_ctrl.volume = s->volume;
#endif
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pebble-audio: bad write offset 0x%" HWADDR_PRIx "\n",
                      offset);
        break;
    }
}

static const MemoryRegionOps pbl_audio_ops = {
    .read = pbl_audio_read,
    .write = pbl_audio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void pbl_audio_reset(DeviceState *dev)
{
    PblAudio *s = PEBBLE_AUDIO(dev);

    pbl_audio_stop(s);
    s->ctrl = 0;
    s->samplerate = 16000;
    s->intctrl = 0;
    s->intstat = 0;
    s->volume = 100;
#ifdef EMSCRIPTEN
    s_wasm_audio_ctrl.rate = s->samplerate;
    s_wasm_audio_ctrl.volume = s->volume;
    s_wasm_audio_ctrl.playing = 0;
#endif
    pbl_audio_update_irq(s);
}

static void pbl_audio_realize(DeviceState *dev, Error **errp)
{
    PblAudio *s = PEBBLE_AUDIO(dev);

    s->drain_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                  pbl_audio_drain_tick, s);
}

static void pbl_audio_init(Object *obj)
{
    PblAudio *s = PEBBLE_AUDIO(obj);

    memory_region_init_io(&s->iomem, obj, &pbl_audio_ops, s,
                          "pebble-audio", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property pbl_audio_properties[] = {
    DEFINE_PROP_STRING("audiodev", PblAudio, audiodev),
};

static void pbl_audio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = pbl_audio_realize;
    device_class_set_legacy_reset(dc, pbl_audio_reset);
    device_class_set_props(dc, pbl_audio_properties);
}

static const TypeInfo pbl_audio_info = {
    .name          = TYPE_PEBBLE_AUDIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PblAudio),
    .instance_init = pbl_audio_init,
    .class_init    = pbl_audio_class_init,
};

static void pbl_audio_register_types(void)
{
    type_register_static(&pbl_audio_info);
}

type_init(pbl_audio_register_types)
