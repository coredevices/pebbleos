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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
/*
 * QEMU stm32f2xx TIM emulation
 * Ported to QEMU 10.x APIs.
 */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/arm/stm32_common.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qapi/error.h"

//#define DEBUG_STM32F2XX_TIM
#ifdef DEBUG_STM32F2XX_TIM
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("DEBUG_STM32F2XX_TIM: " fmt , ## __VA_ARGS__); \
         usleep(100); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif



#define R_TIM_CR1    (0x00 / 4) //p
#define R_TIM_CR2    (0x04 / 4)
#define R_TIM_SMCR   (0x08 / 4)
#define R_TIM_DIER   (0x0c / 4) //p
#define R_TIM_SR     (0x10 / 4) //p
#define R_TIM_EGR    (0x14 / 4) //p
#define R_TIM_CCMR1  (0x18 / 4)
#define R_TIM_CCMR2  (0x1c / 4)
#define R_TIM_CCER   (0x20 / 4)
#define R_TIM_CNT    (0x24 / 4)
#define R_TIM_PSC    (0x28 / 4) //p
#define R_TIM_ARR    (0x2c / 4) //p
#define R_TIM_CCR1   (0x34 / 4)
#define R_TIM_CCR2   (0x38 / 4)
#define R_TIM_CCR3   (0x3c / 4)
#define R_TIM_CCR4   (0x40 / 4)
#define R_TIM_DCR    (0x48 / 4)
#define R_TIM_DMAR   (0x4c / 4)
#define R_TIM_OR     (0x50 / 4)
#define R_TIM_MAX    (0x54 / 4)

static const char *f2xx_tim_reg_names[] = {
    ENUM_STRING(R_TIM_CR1),
    ENUM_STRING(R_TIM_CR2),
    ENUM_STRING(R_TIM_SMCR),
    ENUM_STRING(R_TIM_DIER),
    ENUM_STRING(R_TIM_SR),
    ENUM_STRING(R_TIM_EGR),
    ENUM_STRING(R_TIM_CCMR1),
    ENUM_STRING(R_TIM_CCMR2),
    ENUM_STRING(R_TIM_CCER),
    ENUM_STRING(R_TIM_CNT),
    ENUM_STRING(R_TIM_PSC),
    ENUM_STRING(R_TIM_ARR),
    ENUM_STRING(R_TIM_CCR1),
    ENUM_STRING(R_TIM_CCR2),
    ENUM_STRING(R_TIM_CCR3),
    ENUM_STRING(R_TIM_CCR4),
    ENUM_STRING(R_TIM_DCR),
    ENUM_STRING(R_TIM_CCMR1),
    ENUM_STRING(R_TIM_DMAR),
    ENUM_STRING(R_TIM_OR)
};


typedef struct f2xx_tim {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    QEMUTimer *timer;
    qemu_irq irq;
    uint32_t regs[R_TIM_MAX];

    qemu_irq pwm_ratio_changed;
    qemu_irq pwm_enable;
} f2xx_tim;

static uint32_t
f2xx_tim_period(f2xx_tim *s)
{
    /* FIXME: hard coded to 32kHz */
    return 31250;
}

static int64_t
f2xx_tim_next_transition(f2xx_tim *s, int64_t current_time)
{
    if (s->regs[R_TIM_CR1] & 0x70) {
        qemu_log_mask(LOG_UNIMP, "f2xx tim, only upedge-aligned mode supported\n");
        return -1;
    }
    return current_time + f2xx_tim_period(s) * s->regs[R_TIM_ARR];
}

static void
f2xx_tim_timer(void *arg)
{
    f2xx_tim *s = arg;

    if (s->regs[R_TIM_CR1] & 1) {
        timer_mod(s->timer, f2xx_tim_next_transition(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)));
    }
    if (!(s->regs[R_TIM_SR] & 1)) {
        //printf("f2xx tim timer expired, setting int\n");
        s->regs[R_TIM_SR] |= 1;
    }
    qemu_set_irq(s->irq, 1);
}

static uint64_t
f2xx_tim_read(void *arg, hwaddr addr, unsigned int size)
{
    f2xx_tim *s = arg;
    uint32_t r;
    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= R_TIM_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "f2xx tim invalid read register 0x%x\n",
          (unsigned int)addr << 2);
        return 0;
    }
    r = (s->regs[addr] >> offset * 8) & ((1ull << (8 * size)) - 1);
    switch (addr) {
    case R_TIM_CR1:
    case R_TIM_DIER:
    case R_TIM_SR:
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "f2xx tim unimplemented read 0x%x+%u size %u val 0x%x\n",
          (unsigned int)addr << 2, offset, size, (unsigned int)r);
    }

    DPRINTF("%s %s: reg: %s, size: %d, value: 0x%x\n", s->parent_obj.parent_obj.id,
                  __func__, f2xx_tim_reg_names[addr], size, r);
    return r;
}

static void
f2xx_tim_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    f2xx_tim *s = arg;
    int offset = addr & 0x3;

    addr >>= 2;

    DPRINTF("%s %s: reg:%s, size: %d, value: 0x%llx\n", s->parent_obj.parent_obj.id,
                    __func__, f2xx_tim_reg_names[addr], size, data);
    if (addr >= R_TIM_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "f2xx tim invalid write register 0x%x\n",
          (unsigned int)addr << 2);
        return;
    }

    switch(size) {
    case 1:
        data = (s->regs[addr] & ~(0xff << (offset * 8))) | data << (offset * 8);
        break;
    case 2:
        data = (s->regs[addr] & ~(0xffff << (offset * 8))) | data << (offset * 8);
        break;
    case 4:
        break;
    default:
        abort();
    }

    switch(addr) {
    case R_TIM_CR1:
        if (data & ~1) {
            qemu_log_mask(LOG_UNIMP, "f2xx tim non-zero CR1 unimplemented\n");
        }
        if ((s->regs[addr] & 1) == 0 && data & 1) {
            //printf("f2xx tim started\n");
            timer_mod(s->timer, f2xx_tim_next_transition(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)));
            qemu_set_irq(s->pwm_enable, 1);
        } else if (s->regs[addr] & 1 && (data & 1) == 0) {
            timer_del(s->timer);
            qemu_set_irq(s->pwm_enable, 0);
        }
        s->regs[addr] = data;
        break;
    case R_TIM_SR:
        if (s->regs[addr] & 1 && (data & 1) == 0) {
            //printf("f2xx tim clearing int\n");
            qemu_set_irq(s->irq, 0);
        }
        s->regs[addr] &= data;
        break;
    case R_TIM_EGR:
        qemu_log_mask(LOG_UNIMP, "f2xx tim unimplemented write EGR+%u size %u val 0x%x\n",
          offset, size, (unsigned int)data);
        break;
    case R_TIM_CCER:
        // Capture/Compare Enable register
        s->regs[addr] = data;

        // If we are enabling PWM mode in output 1, notify the callback if we have one registered
        if (    (data & 0x01) // Capture Compare 1 output enable
             && ((s->regs[R_TIM_CCMR1] & 0x60) == 0x60)
             && ((s->regs[R_TIM_CCMR1] & 0x03) == 0x00)) {

            uint32_t ratio = (s->regs[R_TIM_CCR1] * 255) / s->regs[R_TIM_ARR];
            DPRINTF("Setting PWM ratio to %d\n", ratio);
            qemu_set_irq(s->pwm_ratio_changed, ratio);
        }

        // If we are enabling PWM mode in output 1, notify the callback if we have one registered
        else if (    (data & 0x10) // Capture Compare 2 output enable
             && ((s->regs[R_TIM_CCMR1] & 0x6000) == 0x6000)
             && ((s->regs[R_TIM_CCMR1] & 0x0300) == 0x0000)) {

            uint32_t ratio = (s->regs[R_TIM_CCR2] * 255) / s->regs[R_TIM_ARR];
            DPRINTF("Setting PWM ratio to %d\n", ratio);
            qemu_set_irq(s->pwm_ratio_changed, ratio);

        } else {
            qemu_set_irq(s->pwm_ratio_changed, 0);
        }

        break;
    case R_TIM_CCMR1:
        // Capture/Compare mode register 1
        s->regs[addr] = data;
        break;
    case R_TIM_CCMR2:
        // Capture/Compare mode register 2
        s->regs[addr] = data;
        break;
    case R_TIM_DIER:
    case R_TIM_PSC:
    case R_TIM_ARR:
    case R_TIM_CCR1:
    case R_TIM_CCR2:
        s->regs[addr] = data;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "f2xx tim unimplemented write 0x%x+%u size %u val 0x%x\n",
          (unsigned int)addr << 2, offset, size, (unsigned int)data);
    }

}

static const MemoryRegionOps f2xx_tim_ops = {
    .read = f2xx_tim_read,
    .write = f2xx_tim_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

static void f2xx_tim_reset(DeviceState *dev)
{
    f2xx_tim *s = (f2xx_tim *)dev;

    timer_del(s->timer);
    memset(&s->regs, 0, sizeof(s->regs));
}


static void
f2xx_tim_realize(DeviceState *dev, Error **errp)
{
    f2xx_tim *s = (f2xx_tim *)dev;
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &f2xx_tim_ops, s, "tim", 0xa0);
    sysbus_init_mmio(sbd, &s->iomem);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, f2xx_tim_timer, s);
    sysbus_init_irq(sbd, &s->irq);

    qdev_init_gpio_out_named(dev, &s->pwm_ratio_changed, "pwm_ratio_changed", 1);
    qdev_init_gpio_out_named(dev, &s->pwm_enable, "pwm_enable", 1);
}

static void
f2xx_tim_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = f2xx_tim_realize;
    device_class_set_legacy_reset(dc, f2xx_tim_reset);
}

static const TypeInfo
f2xx_tim_info = {
    .name          = "f2xx_tim",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(f2xx_tim),
    .class_init    = f2xx_tim_class_init,
};

static void
f2xx_tim_register_types(void)
{
    type_register_static(&f2xx_tim_info);
}

type_init(f2xx_tim_register_types)
