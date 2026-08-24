/*-
 * Copyright (c) 2013
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/*
 * QEMU model of the stm32f2xx I2C controller.
 * Ported to QEMU 10.x APIs.
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/arm/stm32_common.h"
#include "hw/i2c/i2c.h"
#include "qemu/log.h"
#include "qapi/error.h"

#define R_I2C_CR1      (0x00 / 4)
#define R_I2C_CR2      (0x04 / 4)
#define R_I2C_OAR1     (0x08 / 4)
#define R_I2C_OAR2     (0x0c / 4)
#define R_I2C_DR       (0x10 / 4)
#define R_I2C_SR1      (0x14 / 4)
#define R_I2C_SR2      (0x18 / 4)
#define R_I2C_CCR      (0x1c / 4)
#define R_I2C_TRISE    (0x20 / 4)
#define R_I2C_MAX      (0x24 / 4)


#define R_I2C_CR1_PE_BIT          0x00001
#define R_I2C_CR1_SMBUS_BIT       0x00002
#define R_I2C_CR1_SMBTYPE_BIT     0x00008
#define R_I2C_CR1_ENARB_BIT       0x00010
#define R_I2C_CR1_ENPEC_BIT       0x00020
#define R_I2C_CR1_ENGC_BIT        0x00040
#define R_I2C_CR1_NOSTRETCH_BIT   0x00080
#define R_I2C_CR1_START_BIT       0x00100
#define R_I2C_CR1_STOP_BIT        0x00200
#define R_I2C_CR1_ACK_BIT         0x00400
#define R_I2C_CR1_POS_BIT         0x00800
#define R_I2C_CR1_PEC_BIT         0x01000
#define R_I2C_CR1_ALERT_BIT       0x02000
#define R_I2C_CR1_SWRTS_BIT       0x08000


#define R_I2C_CR2_ITERREN_BIT     0x00100
#define R_I2C_CR2_ITEVTEN_BIT     0x00200
#define R_I2C_CR2_ITBUFEN_BIT     0x00400
#define R_I2C_CR2_DMAEN_BIT       0x00800
#define R_I2C_CR2_LAST_BIT        0x01000

#define R_I2C_SR1_SB_BIT          0x00001
#define R_I2C_SR1_ADDR_BIT        0x00002
#define R_I2C_SR1_BTF_BIT         0x00004
#define R_I2C_SR1_ADD10_BIT       0x00008
#define R_I2C_SR1_STOPF_BIT       0x00010
#define R_I2C_SR1_RxNE_BIT        0x00040
#define R_I2C_SR1_TxE_BIT         0x00080
#define R_I2C_SR1_BERR_BIT        0x00100
#define R_I2C_SR1_ARLO_BIT        0x00200
#define R_I2C_SR1_AF_BIT          0x00400
#define R_I2C_SR1_OVR_BIT         0x00800
#define R_I2C_SR1_PECERR_BIT      0x01000
#define R_I2C_SR1_TIMEOUT_BIT     0x04000
#define R_I2C_SR1_SMBALERT_BIT    0x08000

#define R_I2C_SR2_MSL_BIT         0x00001
#define R_I2C_SR2_BUSY_BIT        0x00002
#define R_I2C_SR2_TRA_BIT         0x00004


//#define DEBUG_STM32F2XX_I2c
#ifdef DEBUG_STM32F2XX_I2c
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("STM32F2XX_I2c: " fmt , ## __VA_ARGS__); \
         usleep(1000); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

static const char *f2xx_i2c_reg_name_arr[] = {
    "CR1",
    "CR2",
    "OAR1",
    "OAR2",
    "DR",
    "SR1",
    "SR2",
    "CCR",
    "TRISE"
};



typedef struct f2xx_i2c {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq evt_irq;
    qemu_irq err_irq;

    I2CBus *bus;

    stm32_periph_t periph;

    int32_t rx;
    int rx_full;
    uint16_t regs[R_I2C_MAX];

} f2xx_i2c;


/* Routine which updates the I2C's IRQs.  This should be called whenever
 * an interrupt-related flag is updated.
 */
static void f2xx_i2c_update_irq(f2xx_i2c *s) {
    int evt_level = 0;
    int err_level = 0;

    if (s->regs[R_I2C_CR2] & R_I2C_CR2_ITEVTEN_BIT) {
        /* Event interrupt: SB, ADDR, ADD10, STOPF, BTF */
        if (s->regs[R_I2C_SR1] & (R_I2C_SR1_SB_BIT | R_I2C_SR1_ADDR_BIT |
                                    R_I2C_SR1_ADD10_BIT | R_I2C_SR1_STOPF_BIT |
                                    R_I2C_SR1_BTF_BIT)) {
            evt_level = 1;
        }
        /* Buffer interrupt: TxE, RxNE (only if ITBUFEN enabled) */
        if (s->regs[R_I2C_CR2] & R_I2C_CR2_ITBUFEN_BIT) {
            if (s->regs[R_I2C_SR1] & (R_I2C_SR1_TxE_BIT | R_I2C_SR1_RxNE_BIT)) {
                evt_level = 1;
            }
        }
    }

    if (s->regs[R_I2C_CR2] & R_I2C_CR2_ITERREN_BIT) {
        if (s->regs[R_I2C_SR1] & (R_I2C_SR1_BERR_BIT | R_I2C_SR1_ARLO_BIT |
                                    R_I2C_SR1_AF_BIT | R_I2C_SR1_OVR_BIT)) {
            err_level = 1;
        }
    }

    qemu_set_irq(s->evt_irq, evt_level);
    qemu_set_irq(s->err_irq, err_level);
}



static uint64_t
f2xx_i2c_read(void *arg, hwaddr offset, unsigned size)
{
    f2xx_i2c *s = arg;
    uint16_t r = UINT16_MAX;
    const char *reg_name = "UNKNOWN";

    if (!(size == 2 || size == 4 || (offset & 0x3) != 0)) {
        STM32_BAD_REG(offset, size);
    }
    offset >>= 2;
    if (offset < R_I2C_MAX) {
        r = s->regs[offset];
        reg_name = f2xx_i2c_reg_name_arr[offset];
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "Out of range I2C read, offset 0x%x\n",
          (unsigned)offset << 2);
    }

    /* SR1 read is part of the flag-clearing sequence */
    if (offset == R_I2C_SR1) {
        s->rx_full = 0;
    }
    /* Reading SR2 after SR1 clears ADDR and STOPF */
    if (offset == R_I2C_SR2) {
        s->regs[R_I2C_SR1] &= ~(R_I2C_SR1_ADDR_BIT | R_I2C_SR1_STOPF_BIT);
        f2xx_i2c_update_irq(s);
    }
    /* Reading DR returns received byte */
    if (offset == R_I2C_DR && s->rx_full) {
        r = s->rx & 0xFF;
        s->rx_full = 0;
        s->regs[R_I2C_SR1] &= ~R_I2C_SR1_RxNE_BIT;
        f2xx_i2c_update_irq(s);
    }

    DPRINTF("%s %s:  register %s, result: 0x%x\n", __func__, s->parent_obj.parent_obj.id,
              reg_name, r);
    return r;
}


static void
f2xx_i2c_write(void *arg, hwaddr offset, uint64_t data, unsigned size)
{
    const char *reg_name = "UNKNOWN";
    struct f2xx_i2c *s = (struct f2xx_i2c *)arg;

    if (size != 2 && size != 4) {
        STM32_BAD_REG(offset, size);
    }
    /* I2C registers are all at most 16 bits wide */
    data &= 0xFFFFF;
    offset >>= 2;

    if (offset < R_I2C_MAX) {
        reg_name = f2xx_i2c_reg_name_arr[offset];
    }
    DPRINTF("%s %s: register %s, data: 0x%llx, size:%d\n", __func__, s->parent_obj.parent_obj.id,
            reg_name, data, size);


    switch (offset) {
    case R_I2C_CR1:
        s->regs[offset] = data;
        if (data & R_I2C_CR1_START_BIT) {
            /* Match QEMU 2.5 behavior: immediately fail with bus error.
             * No I2C slave devices are emulated, so any transfer would NACK.
             * Setting BERR lets the firmware error ISR handle it instantly
             * instead of going through the full SB→address→AF chain that
             * causes ~3s timeout per probe. */
            s->regs[R_I2C_SR1] |= R_I2C_SR1_BERR_BIT;
            s->regs[offset] &= ~R_I2C_CR1_START_BIT;
        }
        if (data & R_I2C_CR1_STOP_BIT) {
            /* STOP condition → end any ongoing transfer, leave master mode */
            i2c_end_transfer(s->bus);
            s->regs[R_I2C_SR2] &= ~(R_I2C_SR2_MSL_BIT | R_I2C_SR2_BUSY_BIT |
                                      R_I2C_SR2_TRA_BIT);
            s->regs[offset] &= ~R_I2C_CR1_STOP_BIT;
        }
        if ((data & R_I2C_CR1_PE_BIT) == 0) {
            /* PE disabled → reset all status */
            s->regs[R_I2C_SR1] = 0;
            s->regs[R_I2C_SR2] = 0;
        }
        break;

    case R_I2C_DR:
        if (s->regs[R_I2C_SR1] & R_I2C_SR1_SB_BIT) {
            /* First write after START → slave address */
            uint8_t addr = (uint8_t)data >> 1;
            int is_recv = data & 1;
            s->regs[R_I2C_SR1] &= ~R_I2C_SR1_SB_BIT;
            if (i2c_start_transfer(s->bus, addr, is_recv)) {
                /* NACK → no slave at this address */
                s->regs[R_I2C_SR1] |= R_I2C_SR1_AF_BIT;
                s->regs[R_I2C_SR2] &= ~(R_I2C_SR2_MSL_BIT |
                                          R_I2C_SR2_BUSY_BIT);
            } else {
                /* ACK → slave responded, address phase complete */
                s->regs[R_I2C_SR1] |= R_I2C_SR1_ADDR_BIT | R_I2C_SR1_TxE_BIT;
                if (!is_recv) {
                    s->regs[R_I2C_SR2] |= R_I2C_SR2_TRA_BIT;
                }
            }
        } else {
            /* Data byte transfer */
            if (i2c_send(s->bus, (uint8_t)data)) {
                s->regs[R_I2C_SR1] |= R_I2C_SR1_AF_BIT;
            } else {
                s->regs[R_I2C_SR1] |= R_I2C_SR1_TxE_BIT | R_I2C_SR1_BTF_BIT;
            }
        }
        break;

    case R_I2C_SR1:
        /* SR1 error flags (bits 8-15) are rc_w0: writing 0 clears them,
         * writing 1 has no effect. Event flags (bits 0-6) are read-only
         * and cleared by hardware sequences, not by software writes. */
        {
            uint16_t error_mask = R_I2C_SR1_BERR_BIT | R_I2C_SR1_ARLO_BIT |
                                  R_I2C_SR1_AF_BIT | R_I2C_SR1_OVR_BIT |
                                  R_I2C_SR1_PECERR_BIT | R_I2C_SR1_TIMEOUT_BIT |
                                  R_I2C_SR1_SMBALERT_BIT;
            /* Clear error bits where firmware wrote 0 */
            uint16_t bits_to_clear = ~data & error_mask;
            s->regs[R_I2C_SR1] &= ~bits_to_clear;
        }
        break;

    default:
        if (offset < ARRAY_SIZE(s->regs)) {
            s->regs[offset] = data;
        } else {
            STM32_BAD_REG(offset, WORD_ACCESS_SIZE);
        }
    }
    f2xx_i2c_update_irq(s);
}

static const MemoryRegionOps f2xx_i2c_ops = {
    .read = f2xx_i2c_read,
    .write = f2xx_i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void
f2xx_i2c_reset(DeviceState *dev)
{
    struct f2xx_i2c *s = (struct f2xx_i2c *)dev;

    memset(s->regs, 0, sizeof(s->regs));
    s->rx = 0;
    s->rx_full = 0;
    qemu_set_irq(s->evt_irq, 0);
    qemu_set_irq(s->err_irq, 0);
}

static void
f2xx_i2c_realize(DeviceState *dev, Error **errp)
{
    struct f2xx_i2c *s = (struct f2xx_i2c *)dev;
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &f2xx_i2c_ops, s, "i2c", 0x3ff);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->evt_irq);
    sysbus_init_irq(sbd, &s->err_irq);
    s->bus = i2c_init_bus(dev, "i2c");
}


static const Property f2xx_i2c_properties[] = {
    DEFINE_PROP_INT32("periph", struct f2xx_i2c, periph, -1),
};

static void
f2xx_i2c_class_init(ObjectClass *c, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(c);

    dc->realize = f2xx_i2c_realize;
    device_class_set_legacy_reset(dc, f2xx_i2c_reset);
    device_class_set_props(dc, f2xx_i2c_properties);
}

static const TypeInfo f2xx_i2c_info = {
    .name = "f2xx_i2c",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(struct f2xx_i2c),
    .class_init = f2xx_i2c_class_init
};

static void
f2xx_i2c_register_types(void)
{
    type_register_static(&f2xx_i2c_info);
}

type_init(f2xx_i2c_register_types)
