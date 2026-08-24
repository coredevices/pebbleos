/*
 * STM32 UART for Pebble - Ported to QEMU 10.x APIs
 *
 * Based on the original Pebble QEMU 2.5 stm32_uart.c by Andre Beckus.
 * Simplified for initial port: no baud rate delay simulation, no RCC/AFIO
 * integration. Maintains the Pebble-specific write handler hooks needed
 * by pebble_control.c.
 *
 * Copyright (C) 2010 Andre Beckus
 * Copyright (c) 2013-2016 Pebble Technology
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/arm/stm32_common.h"
#include "chardev/char-fe.h"
#include "qemu/log.h"
#include "qapi/error.h"

//#define DEBUG_STM32_UART
#ifdef DEBUG_STM32_UART
#define DPRINTF(fmt, ...) \
    do { printf("STM32_UART: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

/* Register offsets */
#define USART_SR_OFFSET   0x00
#define USART_DR_OFFSET   0x04
#define USART_BRR_OFFSET  0x08
#define USART_CR1_OFFSET  0x0C
#define USART_CR2_OFFSET  0x10
#define USART_CR3_OFFSET  0x14
#define USART_GTPR_OFFSET 0x18

/* SR bits */
#define USART_SR_TXE_BIT   7
#define USART_SR_TC_BIT    6
#define USART_SR_RXNE_BIT  5
#define USART_SR_ORE_BIT   3

/* CR1 bits */
#define USART_CR1_UE_BIT     13
#define USART_CR1_TXEIE_BIT  7
#define USART_CR1_TCIE_BIT   6
#define USART_CR1_RXNEIE_BIT 5
#define USART_CR1_TE_BIT     3
#define USART_CR1_RE_BIT     2

#define USART_RCV_BUF_LEN 256

struct Stm32Uart {
    /* Inherited */
    SysBusDevice parent_obj;

    /* Properties */
    stm32_periph_t periph;

    /* Private */
    MemoryRegion iomem;
    qemu_irq irq;

    /* Register values */
    uint32_t USART_RDR;
    uint32_t USART_TDR;
    uint32_t USART_BRR;
    uint32_t USART_CR1;
    uint32_t USART_CR2;
    uint32_t USART_CR3;

    /* Register field values (cached for fast access) */
    uint32_t USART_SR_TXE;
    uint32_t USART_SR_TC;
    uint32_t USART_SR_RXNE;
    uint32_t USART_SR_ORE;
    uint32_t USART_CR1_UE;
    uint32_t USART_CR1_TXEIE;
    uint32_t USART_CR1_TCIE;
    uint32_t USART_CR1_RXNEIE;
    uint32_t USART_CR1_TE;
    uint32_t USART_CR1_RE;

    bool sr_read_since_ore_set;

    /* CharFrontend for chardev connection */
    CharFrontend chr;

    /* Custom write handler (for pebble_control interception) */
    void *chr_write_obj;
    int (*chr_write)(void *chr_write_obj, const uint8_t *buf, int len);

    /* Receive buffer */
    uint8_t rcv_char_buf[USART_RCV_BUF_LEN];
    uint32_t rcv_char_bytes;

    int curr_irq_level;
};

/* Forward declarations */
static int stm32_uart_can_receive(void *opaque);
static void stm32_uart_receive(void *opaque, const uint8_t *buf, int size);
static void stm32_uart_event(void *opaque, QEMUChrEvent event);

/* Update IRQ state */
static void stm32_uart_update_irq(Stm32Uart *s)
{
    int new_level =
        (s->USART_CR1_TXEIE && s->USART_SR_TXE) ||
        (s->USART_CR1_TCIE && s->USART_SR_TC) ||
        (s->USART_CR1_RXNEIE && (s->USART_SR_RXNE || s->USART_SR_ORE));

    if (new_level != s->curr_irq_level) {
        qemu_set_irq(s->irq, new_level);
        s->curr_irq_level = new_level;
    }
}

/* Fill the receive data register from the buffer.
 * Only transfers a byte when RDR is empty (RXNE clear) to prevent
 * overwriting unread data.  The pebble_control timer delivers chunks
 * asynchronously on QEMU_CLOCK_HOST, so the emulated CPU may not have
 * read the previous byte yet. */
static void stm32_uart_fill_rdr(Stm32Uart *s)
{
    if (s->rcv_char_bytes == 0) {
        return;
    }
    if (!s->USART_CR1_UE || !s->USART_CR1_RE) {
        return;
    }
    if (s->USART_SR_RXNE) {
        /* RDR still holds unread data — leave the byte in the buffer
         * so it is not lost.  It will be transferred on the next DR read. */
        return;
    }

    /* Pull next byte from buffer into RDR */
    uint8_t byte = s->rcv_char_buf[0];
    s->rcv_char_bytes--;
    memmove(s->rcv_char_buf, s->rcv_char_buf + 1, s->rcv_char_bytes);

    s->USART_RDR = byte;
    s->USART_SR_RXNE = 1;
    stm32_uart_update_irq(s);
}

/* Chardev receive handler - can we receive? */
static int stm32_uart_can_receive(void *opaque)
{
    Stm32Uart *s = (Stm32Uart *)opaque;

    /* Return available buffer space (matches QEMU 2.5 behavior) */
    return (USART_RCV_BUF_LEN - s->rcv_char_bytes);
}

/* Chardev receive handler - data received.
 * Matches QEMU 2.5 behavior: buffer all bytes first, then fill RDR. */
static void stm32_uart_receive(void *opaque, const uint8_t *buf, int size)
{
    Stm32Uart *s = (Stm32Uart *)opaque;

    assert(size > 0);

    if (!s->USART_CR1_UE || !s->USART_CR1_RE) {
        DPRINTF("Dropping %d chars, UART not enabled (UE=%d RE=%d)\n",
                size, s->USART_CR1_UE, s->USART_CR1_RE);
        return;
    }
    DPRINTF("receive %d bytes, RXNE=%d, buf_bytes=%d, first=0x%02x\n",
            size, s->USART_SR_RXNE, s->rcv_char_bytes, buf[0]);

    /* Buffer all incoming bytes first */
    assert(size <= USART_RCV_BUF_LEN - s->rcv_char_bytes);
    memmove(s->rcv_char_buf + s->rcv_char_bytes, buf, size);
    s->rcv_char_bytes += size;

    /* Move next byte into RDR if ready */
    stm32_uart_fill_rdr(s);
}

/* Chardev event handler */
static void stm32_uart_event(void *opaque, QEMUChrEvent event)
{
    /* Nothing to do */
}

/* MMIO read */
static uint64_t stm32_uart_read(void *opaque, hwaddr offset, unsigned size)
{
    Stm32Uart *s = (Stm32Uart *)opaque;
    uint32_t value;

    switch (offset) {
    case USART_SR_OFFSET:
        value = 0;
        value |= s->USART_SR_TXE << USART_SR_TXE_BIT;
        value |= s->USART_SR_TC << USART_SR_TC_BIT;
        value |= s->USART_SR_RXNE << USART_SR_RXNE_BIT;
        value |= s->USART_SR_ORE << USART_SR_ORE_BIT;
        s->sr_read_since_ore_set = true;
        return value;

    case USART_DR_OFFSET:
        if (s->USART_SR_ORE && s->sr_read_since_ore_set) {
            s->USART_SR_ORE = 0;
        }
        value = s->USART_RDR;
        s->USART_SR_RXNE = 0;
        /* Fill from buffer if there's more data */
        stm32_uart_fill_rdr(s);
        stm32_uart_update_irq(s);
        qemu_chr_fe_accept_input(&s->chr);
        return value & 0x1FF;

    case USART_BRR_OFFSET:
        return s->USART_BRR;

    case USART_CR1_OFFSET:
        return s->USART_CR1;

    case USART_CR2_OFFSET:
        return s->USART_CR2;

    case USART_CR3_OFFSET:
        return s->USART_CR3;

    case USART_GTPR_OFFSET:
        return 0;

    default:
        STM32_BAD_REG(offset, size);
        return 0;
    }
}

/* MMIO write */
static void stm32_uart_write(void *opaque, hwaddr offset,
                              uint64_t val64, unsigned size)
{
    Stm32Uart *s = (Stm32Uart *)opaque;
    uint32_t value = (uint32_t)val64;

    switch (offset) {
    case USART_SR_OFFSET:
        /* Only some bits are writable - TC can be cleared by writing 0 */
        if (!(value & (1 << USART_SR_TC_BIT))) {
            s->USART_SR_TC = 0;
        }
        if (!(value & (1 << USART_SR_RXNE_BIT))) {
            s->USART_SR_RXNE = 0;
        }
        stm32_uart_update_irq(s);
        break;

    case USART_DR_OFFSET: {
        uint8_t ch = value & 0xFF;
        if (s->chr_write_obj && s->chr_write) {
            s->chr_write(s->chr_write_obj, &ch, 1);
        }
        /* Immediate transmit - mark TXE and TC */
        s->USART_SR_TXE = 1;
        s->USART_SR_TC = 1;
        stm32_uart_update_irq(s);
        break;
    }

    case USART_BRR_OFFSET:
        s->USART_BRR = value & 0xFFFF;
        break;

    case USART_CR1_OFFSET:
        s->USART_CR1 = value & 0x3FFF;
        s->USART_CR1_UE = extract32(value, USART_CR1_UE_BIT, 1);
        s->USART_CR1_TXEIE = extract32(value, USART_CR1_TXEIE_BIT, 1);
        s->USART_CR1_TCIE = extract32(value, USART_CR1_TCIE_BIT, 1);
        s->USART_CR1_RXNEIE = extract32(value, USART_CR1_RXNEIE_BIT, 1);
        s->USART_CR1_TE = extract32(value, USART_CR1_TE_BIT, 1);
        s->USART_CR1_RE = extract32(value, USART_CR1_RE_BIT, 1);
        stm32_uart_update_irq(s);
        break;

    case USART_CR2_OFFSET:
        s->USART_CR2 = value;
        break;

    case USART_CR3_OFFSET:
        s->USART_CR3 = value;
        break;

    case USART_GTPR_OFFSET:
        break;

    default:
        STM32_BAD_REG(offset, size);
        break;
    }
}

static const MemoryRegionOps stm32_uart_ops = {
    .read = stm32_uart_read,
    .write = stm32_uart_write,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* === Public functions (used by pebble_control.c and pebble.c) === */

void stm32_uart_set_write_handler(Stm32Uart *s, void *obj,
        int (*chr_write_handler)(void *chr_write_obj, const uint8_t *buf, int len))
{
    s->chr_write_obj = obj;
    s->chr_write = chr_write_handler;
}

void stm32_uart_get_rcv_handlers(Stm32Uart *s, IOCanReadHandler **can_read,
                                  IOReadHandler **read, IOEventHandler **event)
{
    *can_read = stm32_uart_can_receive;
    *read = stm32_uart_receive;
    *event = stm32_uart_event;
}

static int stm32_uart_chr_fe_write_stub(void *opaque, const uint8_t *buf, int len)
{
    Stm32Uart *s = (Stm32Uart *)opaque;
    return qemu_chr_fe_write_all(&s->chr, buf, len);
}

void stm32_uart_connect(Stm32Uart *s, Chardev *chr, uint32_t afio_board_map)
{
    if (chr) {
        qemu_chr_fe_init(&s->chr, chr, &error_abort);
        stm32_uart_set_write_handler(s, s, stm32_uart_chr_fe_write_stub);
        IOCanReadHandler *can_read_cb;
        IOReadHandler *read_cb;
        IOEventHandler *event_cb;
        stm32_uart_get_rcv_handlers(s, &can_read_cb, &read_cb, &event_cb);
        qemu_chr_fe_set_handlers(&s->chr, can_read_cb, read_cb, event_cb,
                                  NULL, s, NULL, true);
    }
}

/* === Device lifecycle === */

static void stm32_uart_reset(DeviceState *dev)
{
    Stm32Uart *s = STM32_UART(dev);

    s->USART_RDR = 0;
    s->USART_TDR = 0;
    s->USART_BRR = 0;
    s->USART_CR1 = 0;
    s->USART_CR2 = 0;
    s->USART_CR3 = 0;

    s->USART_SR_TXE = 1;  /* Transmit buffer empty at reset */
    s->USART_SR_TC = 1;   /* Transmit complete at reset */
    s->USART_SR_RXNE = 0;
    s->USART_SR_ORE = 0;
    s->USART_CR1_UE = 0;
    s->USART_CR1_TXEIE = 0;
    s->USART_CR1_TCIE = 0;
    s->USART_CR1_RXNEIE = 0;
    s->USART_CR1_TE = 0;
    s->USART_CR1_RE = 0;

    s->sr_read_since_ore_set = false;
    s->rcv_char_bytes = 0;
    s->curr_irq_level = 0;
}

static void stm32_uart_realize(DeviceState *dev, Error **errp)
{
    Stm32Uart *s = STM32_UART(dev);

    /* If a chardev was set via property (not via stm32_uart_connect),
     * set up handlers now */
    if (qemu_chr_fe_backend_connected(&s->chr)) {
        stm32_uart_set_write_handler(s, s, stm32_uart_chr_fe_write_stub);
        qemu_chr_fe_set_handlers(&s->chr,
                                  stm32_uart_can_receive,
                                  stm32_uart_receive,
                                  stm32_uart_event,
                                  NULL, s, NULL, true);
    }
}

static void stm32_uart_init(Object *obj)
{
    Stm32Uart *s = STM32_UART(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    memory_region_init_io(&s->iomem, obj, &stm32_uart_ops, s,
                          "stm32-uart", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const Property stm32_uart_properties[] = {
    DEFINE_PROP_INT32("periph", Stm32Uart, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_CHR("chardev", Stm32Uart, chr),
};

static void stm32_uart_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, stm32_uart_reset);
    dc->realize = stm32_uart_realize;
    device_class_set_props(dc, stm32_uart_properties);
}

static const TypeInfo stm32_uart_info = {
    .name          = TYPE_STM32_UART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32Uart),
    .instance_init = stm32_uart_init,
    .class_init    = stm32_uart_class_init,
};

static void stm32_uart_register_types(void)
{
    type_register_static(&stm32_uart_info);
}

type_init(stm32_uart_register_types)
