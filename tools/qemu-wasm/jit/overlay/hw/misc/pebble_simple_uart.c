/*
 * Pebble Simple UART
 *
 * Minimal UART peripheral for Pebble generic machines.
 * Simple register interface — not tied to any specific MCU.
 *
 * Registers (0x1000 region):
 *   0x00 DATA    - Write: TX byte, Read: RX byte (clears RX ready)
 *   0x04 STATE   - Bit 0: TX ready, Bit 1: RX data available
 *   0x08 CTRL    - Bit 0: TX IRQ enable, Bit 1: RX IRQ enable
 *   0x0C INTSTAT - Interrupt status (write 1 to clear bits)
 *                  Bit 0: TX complete, Bit 1: RX ready
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
#include "hw/arm/pebble_simple_uart.h"
#include "chardev/char-fe.h"

#ifdef EMSCRIPTEN
#include <emscripten/emscripten.h>
#endif

#define TYPE_PEBBLE_SIMPLE_UART "pebble-simple-uart"
OBJECT_DECLARE_SIMPLE_TYPE(PblSimpleUart, PEBBLE_SIMPLE_UART)

/* Register offsets */
#define UART_DATA       0x00
#define UART_STATE      0x04
#define UART_CTRL       0x08
#define UART_INTSTAT    0x0C

/* STATE bits */
#define STATE_TX_READY  (1 << 0)
#define STATE_RX_AVAIL  (1 << 1)

/* CTRL bits */
#define CTRL_TX_IRQ_EN  (1 << 0)
#define CTRL_RX_IRQ_EN  (1 << 1)

/* INTSTAT bits */
#define INT_TX_COMPLETE (1 << 0)
#define INT_RX_READY    (1 << 1)

/* RX FIFO size */
#define RX_FIFO_SIZE 256

/* Write handler callback — intercepts firmware TX data */
typedef int (*PblUartWriteHandler)(void *opaque, const uint8_t *buf, int len);

struct PblSimpleUart {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    CharFrontend chr;
    qemu_irq irq;

    uint32_t state;
    uint32_t ctrl;
    uint32_t intstat;

    uint8_t rx_fifo[RX_FIFO_SIZE];
    uint32_t rx_head;  /* next read position */
    uint32_t rx_count; /* bytes in FIFO */

    /* Optional write interceptor (for pebble_control) */
    PblUartWriteHandler write_handler;
    void *write_handler_opaque;

    /* Browser console input (UART2 on the wasm build) */
    bool wasm_console;
#ifdef EMSCRIPTEN
    QEMUTimer *wasm_poll_timer;
#endif
};

#ifdef EMSCRIPTEN
/* Ring for browser -> guest console bytes. The page appends at head
 * (Atomics); an 8 ms virtual-clock poll feeds the RX FIFO. Enabled on
 * the UART the machine marks wasm-console (the dbgserial prompt). */
#define WASM_CON_RING 4096
static uint8_t s_wasm_con_buf[WASM_CON_RING];
static struct {
    uint32_t buf, size;
    uint32_t head, tail;
} s_wasm_con_ctrl;

EMSCRIPTEN_KEEPALIVE void *pebble_wasm_console_ctrl(void)
{
    s_wasm_con_ctrl.buf = (uint32_t)(uintptr_t)s_wasm_con_buf;
    s_wasm_con_ctrl.size = WASM_CON_RING;
    return &s_wasm_con_ctrl;
}
#endif

static void pbl_uart_update_irq(PblSimpleUart *s)
{
    bool level = false;

    if ((s->ctrl & CTRL_TX_IRQ_EN) && (s->intstat & INT_TX_COMPLETE)) {
        level = true;
    }
    /* RX is level-sensitive: as long as bytes are pending in the FIFO and
     * RX IRQ is enabled, the line stays asserted.  The firmware ISR drains
     * the FIFO on each entry, but its upper layer can flip rx_int_enabled
     * off mid-drain (ring buffer full) and leave bytes behind; those bytes
     * must trigger another IRQ once the upper layer re-enables RX.  Driving
     * the line off rx_count (instead of an edge-set intstat bit) is what
     * makes that work.  See PEBBLE-INSTALL-HANG fix. */
    if ((s->ctrl & CTRL_RX_IRQ_EN) && (s->rx_count > 0)) {
        level = true;
    }

    qemu_set_irq(s->irq, level);
}

static void pbl_uart_update_state(PblSimpleUart *s)
{
    s->state = STATE_TX_READY; /* TX always ready in QEMU */
    if (s->rx_count > 0) {
        s->state |= STATE_RX_AVAIL;
    }
}

static uint64_t pbl_uart_read(void *opaque, hwaddr offset, unsigned size)
{
    PblSimpleUart *s = opaque;

    switch (offset) {
    case UART_DATA:
        if (s->rx_count > 0) {
            uint8_t val = s->rx_fifo[s->rx_head];
            s->rx_head = (s->rx_head + 1) % RX_FIFO_SIZE;
            s->rx_count--;
            pbl_uart_update_state(s);
            /* When the FIFO drains to empty, drop the IRQ line so the ISR
             * can return without immediately re-firing. */
            if (s->rx_count == 0) {
                pbl_uart_update_irq(s);
            }
            return val;
        }
        return 0;

    case UART_STATE:
        pbl_uart_update_state(s);
        return s->state;

    case UART_CTRL:
        return s->ctrl;

    case UART_INTSTAT:
        /* Synthesize INT_RX_READY from the FIFO state.  The firmware ISR
         * clears INT_RX_PENDING at entry and only re-reads UART_INT on the
         * next IRQ; reporting the bit live off rx_count means a partial
         * drain doesn't strand bytes when RX_IE is later re-enabled. */
        return s->intstat | (s->rx_count > 0 ? INT_RX_READY : 0);

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pebble-simple-uart: bad read offset 0x%" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
}

static void pbl_uart_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    PblSimpleUart *s = opaque;

    switch (offset) {
    case UART_DATA:
    {
        uint8_t ch = (uint8_t)value;
        if (s->write_handler) {
            s->write_handler(s->write_handler_opaque, &ch, 1);
        } else {
            /* Non-blocking write: if the chardev backing buffer is full
             * (e.g. host pulse-console stopped reading, or the TCP socket
             * buffer filled during a log burst), DROP the byte rather than
             * blocking the vCPU thread.  qemu_chr_fe_write_all would block
             * inside this MMIO store callback, which freezes the guest in
             * the middle of `uart_write_byte` — STATE_TX_READY still reads
             * true so the firmware never knows to back off, KernelMain
             * spins, pebble-tool times out.  Dropping a few log bytes
             * during host slowness is the correct tradeoff for a dev/debug
             * transport. */
            (void)qemu_chr_fe_write(&s->chr, &ch, 1);
        }
        /* TX is instant in QEMU — signal completion */
        s->intstat |= INT_TX_COMPLETE;
        pbl_uart_update_irq(s);
        break;
    }

    case UART_CTRL:
        s->ctrl = value & (CTRL_TX_IRQ_EN | CTRL_RX_IRQ_EN);
        pbl_uart_update_irq(s);
        break;

    case UART_INTSTAT:
        /* Write 1 to clear */
        s->intstat &= ~value;
        pbl_uart_update_irq(s);
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pebble-simple-uart: bad write offset 0x%" HWADDR_PRIx "\n",
                      offset);
        break;
    }
}

static const MemoryRegionOps pbl_uart_ops = {
    .read = pbl_uart_read,
    .write = pbl_uart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static int pbl_uart_can_receive(void *opaque)
{
    PblSimpleUart *s = opaque;
    return RX_FIFO_SIZE - s->rx_count;
}

static void pbl_uart_receive(void *opaque, const uint8_t *buf, int size)
{
    PblSimpleUart *s = opaque;

    uint32_t space = RX_FIFO_SIZE - s->rx_count;
    if ((uint32_t)size > space) {
        size = space;
    }

    bool was_empty = (s->rx_count == 0);

    for (int i = 0; i < size; i++) {
        uint32_t tail = (s->rx_head + s->rx_count) % RX_FIFO_SIZE;
        s->rx_fifo[tail] = buf[i];
        s->rx_count++;
    }

    if (size > 0) {
        pbl_uart_update_state(s);
        /* Only assert from the receive path on the empty -> non-empty edge,
         * so we don't re-pulse the IRQ between TBs while the guest ISR is
         * already draining.  When the FIFO is already non-empty the IRQ
         * line is already high (driven off rx_count by pbl_uart_update_irq),
         * so additional bytes don't need to call update_irq again — once
         * the firmware drains to empty, the read path will deassert. */
        if (was_empty) {
            pbl_uart_update_irq(s);
        }
    }
}

#ifdef EMSCRIPTEN
static void pbl_uart_wasm_poll(void *opaque)
{
    PblSimpleUart *s = opaque;
    uint32_t head = qatomic_load_acquire(&s_wasm_con_ctrl.head);
    uint32_t tail = s_wasm_con_ctrl.tail;

    while (tail != head) {
        int space = pbl_uart_can_receive(s);
        if (space <= 0) {
            break;
        }
        uint32_t off = tail % WASM_CON_RING;
        uint32_t n = MIN(head - tail, WASM_CON_RING - off);
        n = MIN(n, (uint32_t)space);
        pbl_uart_receive(s, &s_wasm_con_buf[off], n);
        tail += n;
        qatomic_store_release(&s_wasm_con_ctrl.tail, tail);
    }
    timer_mod(s->wasm_poll_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 8);
}
#endif

static void pbl_uart_realize(DeviceState *dev, Error **errp)
{
    PblSimpleUart *s = PEBBLE_SIMPLE_UART(dev);

    qemu_chr_fe_set_handlers(&s->chr, pbl_uart_can_receive,
                              pbl_uart_receive, NULL, NULL,
                              s, NULL, true);

#ifdef EMSCRIPTEN
    if (s->wasm_console) {
        s->wasm_poll_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                          pbl_uart_wasm_poll, s);
        timer_mod(s->wasm_poll_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 8);
    }
#endif
}

static void pbl_uart_init(Object *obj)
{
    PblSimpleUart *s = PEBBLE_SIMPLE_UART(obj);

    memory_region_init_io(&s->iomem, obj, &pbl_uart_ops, s,
                          "pebble-simple-uart", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    s->state = STATE_TX_READY;
}

static void pbl_uart_reset(DeviceState *dev)
{
    PblSimpleUart *s = PEBBLE_SIMPLE_UART(dev);

    s->ctrl = 0;
    s->intstat = 0;
    s->rx_head = 0;
    s->rx_count = 0;
    s->state = STATE_TX_READY;
}

static const Property pbl_uart_properties[] = {
    DEFINE_PROP_CHR("chardev", PblSimpleUart, chr),
    DEFINE_PROP_BOOL("wasm-console", PblSimpleUart, wasm_console, false),
};

static void pbl_uart_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = pbl_uart_realize;
    device_class_set_legacy_reset(dc, pbl_uart_reset);
    device_class_set_props(dc, pbl_uart_properties);
}

/* === Public API for pebble_control integration === */

void pbl_uart_set_write_handler(DeviceState *dev, void *opaque,
                                PblUartWriteHandler handler)
{
    PblSimpleUart *s = PEBBLE_SIMPLE_UART(dev);
    s->write_handler = handler;
    s->write_handler_opaque = opaque;
}

void pbl_uart_get_rcv_handlers(DeviceState *dev,
                               IOCanReadHandler **can_read,
                               IOReadHandler **read,
                               IOEventHandler **event)
{
    (void)dev;
    *can_read = pbl_uart_can_receive;
    *read = pbl_uart_receive;
    *event = NULL;
}

void pbl_uart_inject_rx(DeviceState *dev, const uint8_t *buf, int size)
{
    PblSimpleUart *s = PEBBLE_SIMPLE_UART(dev);
    pbl_uart_receive(s, buf, size);
}

static const TypeInfo pbl_uart_info = {
    .name          = TYPE_PEBBLE_SIMPLE_UART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PblSimpleUart),
    .instance_init = pbl_uart_init,
    .class_init    = pbl_uart_class_init,
};

static void pbl_uart_register_types(void)
{
    type_register_static(&pbl_uart_info);
}

type_init(pbl_uart_register_types)
