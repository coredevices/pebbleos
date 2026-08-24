/*
 * STM32F412 QSPI Controller Emulation
 *
 * Manages QSPI register interface and coordinates with SSI bus for flash
 * transfers. Ported from QEMU 2.5.0-pebble8 stm32f412_qspi.c to QEMU 10.x.
 *
 * Copyright (c) 2013-2016 Pebble Technology
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/ssi/ssi.h"
#include "hw/arm/stm32_common.h"
#include "qemu/log.h"

/* #define QSPI_DEBUG */
#ifdef QSPI_DEBUG
#define DB_PRINT_L(level, ...) do { \
    fprintf(stderr, "%d: %s: ", level, __func__); \
    fprintf(stderr, ## __VA_ARGS__); \
    fprintf(stderr, "\n"); \
} while (0)
#else
#define DB_PRINT_L(level, ...) do {} while (0)
#endif

/* QSPI Register Definitions */
#define R_CR   (0x00 / 4)
#define CR_MATCH_MODE_MASK (0x01 << 23)

#define R_DCR  (0x04 / 4)

#define R_SR   (0x08 / 4)
#define R_SR_TEF (1 << 0)
#define R_SR_TCF (1 << 1)
#define R_SR_FTF (1 << 2)
#define R_SR_SMF (1 << 3)
#define R_SR_TOF (1 << 4)
#define R_SR_BSY (1 << 5)

#define R_FCR  (0x0c / 4)
#define R_FCR_CTOF (1 << 4)
#define R_FCR_CSMF (1 << 3)
#define R_FCR_CTCF (1 << 1)
#define R_FCR_CTEF (1 << 0)

#define R_DLR  (0x10 / 4)

#define R_CCR  (0x14 / 4)
#define CCR_INSTR_MASK     (0xff << 0)
#define CCR_IMODE_MASK     (0x3 << 8)
#define CCR_ADMODE_MASK    (0x3 << 10)
#define CCR_ADSIZE_MASK    (0x3 << 12)
#define CCR_MODE_MASK      (0x3 << 26)

#define CCR_IMODE_NONE     (0x0 << 8)
#define CCR_IMODE_ONELINE  (0x1 << 8)
#define CCR_IMODE_TWOLINE  (0x2 << 8)
#define CCR_IMODE_FOURLINE (0x3 << 8)

#define CCR_ADMODE_NONE     (0x0 << 10)
#define CCR_ADMODE_ONELINE  (0x1 << 10)
#define CCR_ADMODE_TWOLINE  (0x2 << 10)
#define CCR_ADMODE_FOURLINE (0x3 << 10)

#define CCR_ADSIZE_8  (0x0 << 12)
#define CCR_ADSIZE_16 (0x1 << 12)
#define CCR_ADSIZE_24 (0x2 << 12)
#define CCR_ADSIZE_32 (0x3 << 12)

#define CCR_MODE_INDIRECT_WRITE (0x00 << 26)
#define CCR_MODE_INDIRECT_READ  (0x01 << 26)
#define CCR_MODE_AUTOMATIC_POLL (0x2 << 26)
#define CCR_MODE_MEMORY_MAPPED  (0x3 << 26)

#define R_AR   (0x18 / 4)
#define R_ABR  (0x1c / 4)
#define R_DR   (0x20 / 4)
#define R_PSMKR (0x24 / 4)
#define R_PSMAR (0x28 / 4)
#define R_PIR  (0x2c / 4)
#define R_LPTR (0x30 / 4)

#define R_MAX  (0x34 / 4)

#define TYPE_STM32F412_QSPI "stm32f412_qspi"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32f412Qspi, STM32F412_QSPI)

typedef enum {
    CS_STATE_LOW = 0,
    CS_STATE_HIGH = 1,
} CsState;

struct Stm32f412Qspi {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    SSIBus *qspi;

    uint32_t regs[R_MAX];

    qemu_irq cs_irq;
    CsState cs_state;

    int64_t tx_remaining;
};

static uint32_t
stm32f412_auto_poll(Stm32f412Qspi *s, uint8_t reg)
{
    bool and_match = (s->regs[R_CR] & CR_MATCH_MODE_MASK) == 0;
    uint32_t match = s->regs[R_PSMAR];
    uint32_t mask = s->regs[R_PSMKR];
    bool set;

    if (and_match) {
        set = (reg & mask) == match;
    } else {
        set = ((reg & mask) & match) != 0;
    }
    DB_PRINT_L(1, "Auto Poll %s", set ? "Success" : "Fail");
    return set;
}

static bool
stm32f412_in_CCR_mode(Stm32f412Qspi *s, uint32_t mode)
{
    return (s->regs[R_CCR] & CCR_MODE_MASK) == mode;
}

static void
stm32f412_set_cs(Stm32f412Qspi *s, CsState state)
{
    if (s->cs_state != state) {
        qemu_set_irq(s->cs_irq, state == CS_STATE_HIGH);
        s->cs_state = state;
    }
}

static uint64_t
stm32f412_qspi_read(void *arg, hwaddr offset, unsigned size)
{
    Stm32f412Qspi *s = arg;
    uint32_t r = UINT32_MAX;

    offset >>= 2;

    if (offset < R_MAX) {
        switch (offset) {
        case R_DR:
            DB_PRINT_L(1, "Read %u of %" PRIi64 " remaining",
                        size, s->tx_remaining);
            r = ssi_transfer(s->qspi, 0);
            s->tx_remaining -= size;
            break;
        case R_SR:
            if (stm32f412_in_CCR_mode(s, CCR_MODE_AUTOMATIC_POLL)) {
                DB_PRINT_L(2, "POLL STATUS REG. Command? %d",
                            s->cs_state == CS_STATE_LOW);
                stm32f412_set_cs(s, CS_STATE_LOW);
                ssi_transfer(s->qspi, s->regs[R_CCR] & CCR_INSTR_MASK);
                uint8_t reg = ssi_transfer(s->qspi, 0);
                stm32f412_set_cs(s, CS_STATE_HIGH);
                DB_PRINT_L(2, "REG: 0x%" PRIx8, reg);
                if (stm32f412_auto_poll(s, reg)) {
                    s->regs[R_SR] |= R_SR_SMF;
                    s->regs[R_CCR] &= ~CCR_MODE_AUTOMATIC_POLL;
                }
            }
            r = s->regs[R_SR];
            break;
        default:
            r = s->regs[offset];
            break;
        }
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "QSPI: Out of range register 0x%x\n", (unsigned)offset);
    }

    if (s->tx_remaining <= 0) {
        stm32f412_set_cs(s, CS_STATE_HIGH);
        s->regs[R_SR] &= ~R_SR_TCF;
        s->regs[R_SR] &= ~R_SR_BSY;
    }

    return r;
}

static void
stm32f412_CCR_write(Stm32f412Qspi *s, uint32_t CCR, unsigned size)
{
    if ((CCR & CCR_ADMODE_MASK) != CCR_ADMODE_NONE) {
        uint8_t addrsize = ((CCR & CCR_ADSIZE_MASK) >> 12) + 1;
        DB_PRINT_L(1, "Adding %" PRIu8 " for address", addrsize);
        s->tx_remaining += addrsize;
    }
    if ((CCR & CCR_MODE_MASK) != CCR_MODE_AUTOMATIC_POLL) {
        if ((CCR & CCR_IMODE_MASK) != CCR_IMODE_NONE) {
            DB_PRINT_L(1, "Command 0x%x to tx %" PRIi64 " B",
                        CCR & CCR_INSTR_MASK, s->tx_remaining);
            stm32f412_set_cs(s, CS_STATE_LOW);
            ssi_transfer(s->qspi, CCR & CCR_INSTR_MASK);
        }
    }
    s->regs[R_CCR] = CCR;
}

static void
stm32f412_qspi_xfer(Stm32f412Qspi *s, uint32_t data, unsigned size)
{
    int i;
    for (i = 0; i < size; ++i) {
        uint8_t val = (data >> (i * 8)) & 0xff;
        ssi_transfer(s->qspi, val);
    }
}

static void
stm32f412_qspi_write(void *arg, hwaddr addr, uint64_t data, unsigned size)
{
    Stm32f412Qspi *s = arg;

    addr >>= 2;

    switch (addr) {
    case R_FCR:
        s->regs[R_SR] &= ~(data);
        break;
    case R_CCR:
        stm32f412_CCR_write(s, data, size);
        break;
    case R_AR: {
        s->regs[R_AR] = data;
        uint8_t addrsize = ((s->regs[R_CCR] & CCR_ADSIZE_MASK) >> 12) + 1;
        stm32f412_qspi_xfer(s, data, addrsize);
        s->tx_remaining -= addrsize;
        break;
    }
    case R_DR:
        DB_PRINT_L(2, "Write %u of %" PRIi64 " remaining",
                    size, s->tx_remaining);
        s->regs[R_DR] = 0;
        stm32f412_qspi_xfer(s, data, size);
        s->tx_remaining -= size;
        break;
    case R_DLR:
        s->tx_remaining = data + 1;
        DB_PRINT_L(1, "Set DLR to %" PRIu64 " + 1", data);
        break;
    default:
        s->regs[addr] = data;
        break;
    }

    if (s->tx_remaining <= 0) {
        stm32f412_set_cs(s, CS_STATE_HIGH);
    }

    s->regs[R_SR] |= R_SR_TCF;
}

static const MemoryRegionOps stm32f412_qspi_ops = {
    .read = stm32f412_qspi_read,
    .write = stm32f412_qspi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void
stm32f412_qspi_reset(DeviceState *dev)
{
    Stm32f412Qspi *s = STM32F412_QSPI(dev);
    s->cs_state = CS_STATE_HIGH;
    s->tx_remaining = 0;
}

static void
stm32f412_qspi_realize(DeviceState *dev, Error **errp)
{
    Stm32f412Qspi *s = STM32F412_QSPI(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &stm32f412_qspi_ops, s,
                          "qspi", 0x3ff);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->qspi = ssi_create_bus(dev, "ssi");

    s->cs_state = CS_STATE_HIGH;
    s->tx_remaining = 0;

    qdev_init_gpio_out_named(dev, &s->cs_irq, "qspi-gpio-cs", 1);
}

static void
stm32f412_qspi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = stm32f412_qspi_realize;
    device_class_set_legacy_reset(dc, stm32f412_qspi_reset);
}

static const TypeInfo stm32f412_qspi_info = {
    .name          = TYPE_STM32F412_QSPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32f412Qspi),
    .class_init    = stm32f412_qspi_class_init,
};

static void
stm32f412_qspi_register_types(void)
{
    type_register_static(&stm32f412_qspi_info);
}

type_init(stm32f412_qspi_register_types)
