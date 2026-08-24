/*-
 * Copyright (c) 2014
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
 * QEMU stm32f2xx PWR emulation
 * Ported to QEMU 10.x APIs.
 */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qapi/error.h"

//#define DEBUG_STM32F2XX_PWR
#ifdef DEBUG_STM32F2XX_PWR
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("DEBUG_STM32F2XX_PWR: " fmt , ## __VA_ARGS__); \
         usleep(1000); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

#define R_PWR_CR      (0x00/4)
#define R_PWR_CR_LPDS   0x00000001
#define R_PWR_CR_PDDS   0x00000002

#define R_PWR_CSR     (0x04/4)

#define R_PWR_MAX     (0x08/4)

typedef struct f2xx_pwr {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem;

    uint32_t      regs[R_PWR_MAX];
} f2xx_pwr;



static uint64_t
f2xx_pwr_read(void *arg, hwaddr addr, unsigned int size)
{
    f2xx_pwr *s = arg;
    uint32_t r;
    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= R_PWR_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid read f2xx pwr register 0x%x\n",
          (unsigned int)addr << 2);
        DPRINTF("  %s: result: 0\n", __func__);
        return 0;
    }

    uint32_t value = s->regs[addr];

    r = (value >> offset * 8) & ((1ull << (8 * size)) - 1);

    DPRINTF("%s: addr: 0x%llx, size: %d, value: 0x%x\n", __func__, addr, size, r);
    return r;
}

static void
f2xx_pwr_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    f2xx_pwr *s = arg;
    int offset = addr & 0x3;

    DPRINTF("%s: addr: 0x%llx, data: 0x%llx, size: %d\n", __func__, addr, data, size);

    addr >>= 2;
    if (addr >= R_PWR_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid write f2xx pwr register 0x%x\n",
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
    case R_PWR_CR:
    case R_PWR_CSR:
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "f2xx pwr unimplemented write 0x%x+%u size %u val 0x%x\n",
          (unsigned int)addr << 2, offset, size, (unsigned int)data);
    }

    // Save new value
    s->regs[addr] = data;
}

static const MemoryRegionOps f2xx_pwr_ops = {
    .read = f2xx_pwr_read,
    .write = f2xx_pwr_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};


// Return true if we should go into standby mode when the process goes into deepsleep.
// deepsleep is entered if the SLEEPDEEP bit is set in the SCR register when the processor
// executes a WFI instruction.
bool f2xx_pwr_powerdown_deepsleep(void *opaqe)
{
    f2xx_pwr *s = opaqe;
    return s->regs[R_PWR_CR] & R_PWR_CR_PDDS;
}

static void f2xx_pwr_reset(DeviceState *dev)
{
    f2xx_pwr *s = (f2xx_pwr *)dev;

    memset(s->regs, 0, sizeof(s->regs));
}


static void
f2xx_pwr_realize(DeviceState *dev, Error **errp)
{
    f2xx_pwr *s = (f2xx_pwr *)dev;
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &f2xx_pwr_ops, s, "pwr", 0x08);
    sysbus_init_mmio(sbd, &s->iomem);

    s->regs[R_PWR_CR] = 0;
    s->regs[R_PWR_CSR] = 0;
}

static void
f2xx_pwr_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = f2xx_pwr_realize;
    device_class_set_legacy_reset(dc, f2xx_pwr_reset);
}

static const TypeInfo
f2xx_pwr_info = {
    .name          = "f2xx_pwr",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(f2xx_pwr),
    .class_init    = f2xx_pwr_class_init,
};

static void
f2xx_pwr_register_types(void)
{
    type_register_static(&f2xx_pwr_info);
}

type_init(f2xx_pwr_register_types)
