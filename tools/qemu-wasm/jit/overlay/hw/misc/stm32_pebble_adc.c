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
 * QEMU model of the stm32f2xx ADC.
 * Ported to QEMU 10.x APIs.
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "qemu/log.h"
#include "qapi/error.h"

/* Per-ADC registers */
#define R_ADC_SR             (0x00 / 4)
#define R_ADC_SR_MASK         0x3f
#define R_ADC_SR_EOC       (1 << 1)
#define R_ADC_CR1            (0x04 / 4)
#define R_ADC_CR2            (0x08 / 4)
#define R_ADC_SMPR1          (0x0c / 4)
#define R_ADC_SMPR2          (0x10 / 4)
#define R_ADC_JOFRx          (0x14 / 4)
#define R_ADC_HTR            (0x24 / 4)
#define R_ADC_HTR_RESET 0x00000fff
#define R_ADC_LTR            (0x28 / 4)
#define R_ADC_SQR1           (0x2c / 4)
#define R_ADC_SQR2           (0x30 / 4)
#define R_ADC_SQR3           (0x34 / 4)
#define R_ADC_JSQR           (0x38 / 4)
#define R_ADC_JDRx           (0x3c / 4)
#define R_ADC_DR             (0x4c / 4)
#define R_ADC_MAX            (0x50 / 4)

/* Common registers */
#define R_ADC_CSR            (0x00 / 4)
#define R_ADC_CCR            (0x04 / 4)
#define R_ADC_CDR            (0x08 / 4)
#define R_ADC_CMAX           (0x0c / 4)

typedef struct {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t regs[3][R_ADC_MAX];
    uint32_t ccr;
} stm32_adc;

static uint64_t
stm32f2xx_adc_common_read(stm32_adc *s, hwaddr offset, unsigned int size)
{
    uint32_t r = 0;
    int i;

    offset >>= 2;
    switch(offset) {
    case R_ADC_CSR:
        for (i = 0; i < 3; i++) {
            r |= (s->regs[i][R_ADC_SR] & R_ADC_SR_MASK) << 8 * i;
        }
        break;
    case R_ADC_CCR:
        r = s->ccr;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "f2xx adc unimplemented read reg 0x%x\n",
          (int)offset << 2);
    }
    return 0;
}

static uint64_t
stm32f2xx_adc_read(void *arg, hwaddr offset, unsigned int size)
{
    stm32_adc *s = arg;
    uint32_t r;
    int unit = (offset & 0x300) >> 8;

    if (unit == 3) {
        return stm32f2xx_adc_common_read(s, offset - 0x300, size);
    }
    offset = (offset & 0xFF) >> 2;
    r = s->regs[unit][offset];
    switch (offset) {
    case R_ADC_SR:
        r |= R_ADC_SR_EOC;
        break;

    /* Registers with standard read behaviour. */
    case R_ADC_CR1:
    case R_ADC_CR2:
    case R_ADC_SMPR1:
    case R_ADC_SMPR2:
    case R_ADC_SQR1:
    case R_ADC_SQR2:
    case R_ADC_SQR3:
        break;
    case R_ADC_DR:
        break; /* Hack */
    default:
        qemu_log_mask(LOG_UNIMP, "adc %d reg %x return 0x%x\n", unit, (int)offset << 2, r);
    }
    return r;
}

static void
stm32f2xx_adc_common_write(stm32_adc *s, hwaddr offset, uint64_t data,
  unsigned int size)
{
    offset >>= 2;
    switch (offset) {
    case R_ADC_CSR:
    case R_ADC_CDR:
        break;
    case R_ADC_CCR:
        //printf("ccr set 0x%x\n", (int)data);
        s->ccr = data;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "f2xx adc unimplemented write reg 0x%x\n",
          (int)offset << 2);
    }
}

static void
stm32f2xx_adc_write(void *arg, hwaddr offset, uint64_t data, unsigned int size)
{
    stm32_adc *s = arg;
    int unit = (offset & 0x300) >> 8;

    if (unit == 3) {
        stm32f2xx_adc_common_write(s, offset - 0x300, data, size);
        return;
    }
    offset = (offset & 0xFF) >> 2;
    switch (offset) {
    case R_ADC_SR:
        s->regs[unit][offset] &= data; /* rc_w0 */
        break;
    case R_ADC_CR1:
        s->regs[unit][offset] = data;
        if (data != 0) {
            qemu_log_mask(LOG_UNIMP, "f2xx adc unimplemented CR1 write 0x%08x\n", (unsigned int)data);
        }
        break;
    case R_ADC_CR2:
        if (data & ~0x40000001) {
            qemu_log_mask(LOG_UNIMP, "f2xx adc unimplemented CR2 write 0x%08x\n", (unsigned int)data);
        }
        s->regs[unit][offset] = data;
        break;
    case R_ADC_SMPR1: /* XXX Ignore sampling time setting. */
    case R_ADC_SQR1:
    case R_ADC_SQR2:
    case R_ADC_SQR3:
        s->regs[unit][offset] = data;
        break;
    case R_ADC_DR:
        qemu_log_mask(LOG_GUEST_ERROR, "f2xx adc write to r/o data reg\n");
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "adc %d reg %x write 0x%x\n", unit,
          (int)offset << 2, (int)data);
        if (offset < R_ADC_MAX) {
            s->regs[unit][offset] = data;
        }
        break;
    }
}

static const MemoryRegionOps stm32f2xx_adc_ops = {
    .read = stm32f2xx_adc_read,
    .write = stm32f2xx_adc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4, /* XXX actually 1 */
        .max_access_size = 4
    }
};

static void
f2xx_adc_reset(DeviceState *ds)
{
    stm32_adc *s = (stm32_adc *)ds;

    /* Hack for low ambient light level (2500) */
    s->regs[0][R_ADC_DR] = 2730;
    s->regs[1][R_ADC_DR] = 2500;
}

static void
stm32f2xx_adc_realize(DeviceState *dev, Error **errp)
{
    stm32_adc *s = (stm32_adc *)dev;
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

#if 0
    sysbus_init_irq(sbd, &s->irq);
#endif

    memory_region_init_io(&s->iomem, OBJECT(dev), &stm32f2xx_adc_ops, s, "adc", 0x400);
    sysbus_init_mmio(sbd, &s->iomem);
}


static void
stm32f2xx_adc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32f2xx_adc_realize;
    device_class_set_legacy_reset(dc, f2xx_adc_reset);
}

static const TypeInfo stm32f2xx_adc_info = {
    .name = "stm32f2xx_adc",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(stm32_adc),
    .class_init = stm32f2xx_adc_class_init
};

static void
stm32f2xx_adc_register_types(void)
{
    type_register_static(&stm32f2xx_adc_info);
}

type_init(stm32f2xx_adc_register_types);
