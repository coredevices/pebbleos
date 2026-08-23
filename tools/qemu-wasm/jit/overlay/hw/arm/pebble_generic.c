/*
 * Pebble Generic Machine Types
 *
 * Generic ARMv7M-based machine types for new Pebble platforms.
 * Uses simple custom MMIO peripherals instead of MCU-specific emulation.
 *
 * Machine types:
 *   pebble-emery   - Cortex-M33, 512KB RAM, 4MB flash
 *   pebble-flint   - Cortex-M4, 256KB RAM, 4MB flash
 *   pebble-gabbro  - Cortex-M33, 512KB RAM, 4MB flash
 *
 * Copyright (c) 2026 Core Devices LLC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/arm/armv7m.h"
#include "hw/arm/machines-qom.h"
#include "hw/arm/boot.h"
#include "hw/core/boards.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/unimp.h"
#include "chardev/char-fe.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "qemu/audio.h"
#include "system/blockdev.h"
#include "exec/cpu-common.h"
#include "qom/object.h"
#include "hw/arm/pebble_generic.h"
#include "hw/misc/pebble_null_input.h"
#include "pebble_control.h"

/* ===== Board configurations ===== */

static const PblGenericBoardConfig board_cfg_emery = {
    .name          = "pebble-emery",
    .desc          = "Pebble Emery (obelix, Cortex-M33)",
    .cpu_type      = ARM_CPU_TYPE_NAME("cortex-m33"),
    .board_type    = PBL_BOARD_EMERY,
    .board_id      = PBL_BOARD_ID_EMERY,
    .flash_size    = 4 * MiB,
    .ram_size      = 512 * KiB,
    .sysclk_frq    = PBL_SYSCLK_FRQ,
    .display_width = 200,
    .display_height = 228,
    .display_bpp   = 8,
    .display_round = false,
    .has_touch     = true,
    .has_audio     = true,
};

static const PblGenericBoardConfig board_cfg_flint = {
    .name          = "pebble-flint",
    .desc          = "Pebble Flint (asterix, Cortex-M4)",
    .cpu_type      = ARM_CPU_TYPE_NAME("cortex-m4"),
    .board_type    = PBL_BOARD_FLINT,
    .board_id      = PBL_BOARD_ID_FLINT,
    .flash_size    = 4 * MiB,
    .ram_size      = 256 * KiB,
    .sysclk_frq    = PBL_SYSCLK_FRQ,
    .display_width = 144,
    .display_height = 168,
    .display_bpp   = 1,
    .display_round = false,
    .has_touch     = false,
    .has_audio     = true,
};

static const PblGenericBoardConfig board_cfg_gabbro = {
    .name          = "pebble-gabbro",
    .desc          = "Pebble Gabbro (getafix, Cortex-M33)",
    .cpu_type      = ARM_CPU_TYPE_NAME("cortex-m33"),
    .board_type    = PBL_BOARD_GABBRO,
    .board_id      = PBL_BOARD_ID_GABBRO,
    .flash_size    = 4 * MiB,
    .ram_size      = 512 * KiB,
    .sysclk_frq    = PBL_SYSCLK_FRQ,
    .display_width = 260,
    .display_height = 260,
    .display_bpp   = 8,
    .display_round = true,
    .has_touch     = true,
    .has_audio     = false,
};

/* ===== Machine init ===== */

static void pbl_generic_init(MachineState *machine)
{
    PblGenericMachineState *s = PBL_GENERIC_MACHINE(machine);
    PblGenericMachineClass *mc = PBL_GENERIC_MACHINE_GET_CLASS(machine);
    const PblGenericBoardConfig *cfg = mc->board_cfg;
    MemoryRegion *system_memory = get_system_memory();
    DeviceState *armv7m;

    /* Clocks */
    s->sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(s->sysclk, cfg->sysclk_frq);

    s->refclk = clock_new(OBJECT(machine), "REFCLK");
    clock_set_hz(s->refclk, PBL_REFCLK_FRQ);

    /* Flash (code memory) */
    memory_region_init_ram(&s->flash, NULL, "pebble.flash",
                           cfg->flash_size, &error_fatal);
    memory_region_set_readonly(&s->flash, true);
    memory_region_add_subregion(system_memory, PBL_FLASH_BASE, &s->flash);

    /* SRAM */
    memory_region_init_ram(&s->sram, NULL, "pebble.sram",
                           cfg->ram_size, &error_fatal);
    memory_region_add_subregion(system_memory, PBL_SRAM_BASE, &s->sram);

    /* ARMv7M CPU + NVIC */
    object_initialize_child(OBJECT(s), "armv7m", &s->armv7m, TYPE_ARMV7M);
    armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", PBL_NUM_IRQS);
    /* Pebble firmware assumes 3 priority bits (__NVIC_PRIO_BITS=3), matching
     * the nRF52/STM32F4 watches. QEMU defaults to 8 for ARMv7+, which means
     * CMSIS-shifted IRQ priorities like 0xA0 are numerically less than
     * BASEPRI=0xBF and would preempt critical sections — leaking
     * uxCriticalNesting via portCLEAR_INTERRUPT_MASK_FROM_ISR.
     */
    qdev_prop_set_uint8(armv7m, "num-prio-bits", 3);
    qdev_prop_set_string(armv7m, "cpu-type", cfg->cpu_type);
    qdev_connect_clock_in(armv7m, "cpuclk", s->sysclk);
    qdev_connect_clock_in(armv7m, "refclk", s->refclk);
    qdev_prop_set_bit(armv7m, "enable-bitband", true);
    object_property_set_link(OBJECT(&s->armv7m), "memory",
                             OBJECT(system_memory), &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), &error_fatal);

    /* === UARTs === */
    DeviceState *uart1_dev = NULL;
    for (int i = 0; i < 3; i++) {
        static const hwaddr uart_base[] = {
            PBL_UART0_BASE, PBL_UART1_BASE, PBL_UART2_BASE
        };
        static const int uart_irq[] = {
            PBL_IRQ_UART0, PBL_IRQ_UART1, PBL_IRQ_UART2
        };

        DeviceState *dev = qdev_new(TYPE_PEBBLE_SIMPLE_UART);
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

        /* UART1 gets no chardev — pebble_control manages its CharBackend */
        if (i != 1) {
            qdev_prop_set_chr(dev, "chardev", serial_hd(i));
        }
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, uart_base[i]);
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(armv7m, uart_irq[i]));

        if (i == 1) {
            uart1_dev = dev;
        }
    }

    /* === Pebble Control Protocol on UART1 === */
    {
        Chardev *chr = serial_hd(1);
        if (uart1_dev) {
            /* chr may be NULL in the browser build; the wasm ring
             * bridge inside pebble_control still needs the hookup. */
            pebble_control_create_generic(chr, uart1_dev);
        }
    }

    /* === Timers === */
    for (int i = 0; i < 2; i++) {
        static const hwaddr timer_base[] = {
            PBL_TIMER0_BASE, PBL_TIMER1_BASE
        };
        static const int timer_irq[] = {
            PBL_IRQ_TIMER0, PBL_IRQ_TIMER1
        };

        DeviceState *dev = qdev_new(TYPE_PEBBLE_GENERIC_TIMER);
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

        qdev_connect_clock_in(dev, "clk", s->sysclk);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, timer_base[i]);
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(armv7m, timer_irq[i]));
    }

    /* === RTC === */
    {
        DeviceState *dev = qdev_new(TYPE_PEBBLE_GENERIC_RTC);
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, PBL_RTC_BASE);
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(armv7m, PBL_IRQ_RTC));
    }

    /* === System Control === */
    {
        DeviceState *dev = qdev_new(TYPE_PEBBLE_SYSCTRL);
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
        uint32_t features = 0;

        if (cfg->has_touch) features |= (1 << 0);
        if (cfg->has_audio) features |= (1 << 1);
        if (cfg->display_round) features |= (1 << 2);

        qdev_prop_set_uint32(dev, "board-id", cfg->board_id);
        qdev_prop_set_uint32(dev, "features", features);
        qdev_prop_set_uint32(dev, "display-width", cfg->display_width);
        qdev_prop_set_uint32(dev, "display-height", cfg->display_height);
        qdev_prop_set_uint32(dev, "display-format", cfg->display_bpp);

        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, PBL_SYSCTRL_BASE);
    }

    /* === Display === */
    {
        DeviceState *dev = qdev_new(TYPE_PEBBLE_DISPLAY);
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

        qdev_prop_set_uint32(dev, "width", cfg->display_width);
        qdev_prop_set_uint32(dev, "height", cfg->display_height);
        qdev_prop_set_uint32(dev, "format", cfg->display_bpp);
        qdev_prop_set_bit(dev, "round-mask", cfg->display_round);

        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, PBL_DISPLAY_BASE);     /* registers */
        sysbus_mmio_map(sbd, 1, PBL_DISPLAY_FB_BASE);   /* framebuffer */
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(armv7m, PBL_IRQ_DISPLAY));
    }

    /* === External Flash (XIP) === */
    {
        DeviceState *dev = qdev_new(TYPE_PEBBLE_EXTFLASH);
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

        qdev_prop_set_uint32(dev, "size", PBL_EXTFLASH_SIZE);

        /* Attach block device if user provided -drive if=mtd */
        DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);
        if (dinfo) {
            qdev_prop_set_drive(dev, "drive", blk_by_legacy_dinfo(dinfo));
        }

        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, PBL_EXTFLASH_CTRL_BASE);  /* registers */
        sysbus_mmio_map(sbd, 1, PBL_EXTFLASH_BASE);        /* XIP memory */
    }

    /* === GPIO (Buttons) === */
    {
        DeviceState *dev = qdev_new(TYPE_PEBBLE_GPIO);
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, PBL_GPIO_BASE);
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(armv7m, PBL_IRQ_GPIO));
    }

    /* === Touch Controller (Emery/Gabbro only) === */
    if (cfg->has_touch) {
        DeviceState *dev = qdev_new(TYPE_PEBBLE_TOUCH);
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

        qdev_prop_set_uint32(dev, "display-width", cfg->display_width);
        qdev_prop_set_uint32(dev, "display-height", cfg->display_height);

        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, PBL_TOUCH_BASE);
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(armv7m, PBL_IRQ_TOUCH));
    } else {
        /* Non-touch boards: install a no-op absolute-pointer handler so the
         * UI does not grab the host cursor on click. */
        DeviceState *dev = qdev_new(TYPE_PEBBLE_NULL_INPUT);
        qdev_realize_and_unref(dev, NULL, &error_fatal);
    }

    /* === Audio DAC (Emery/Flint) === */
    if (cfg->has_audio) {
        DeviceState *dev = qdev_new(TYPE_PEBBLE_AUDIO);
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

        if (machine->audiodev) {
            qdev_prop_set_string(dev, "audiodev", machine->audiodev);
        }
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, PBL_AUDIO_BASE);
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(armv7m, PBL_IRQ_AUDIO));
    }

    /* === QEMU Settings (RTC backup register 0) === */
    /* Firmware reads these flags to detect QEMU mode and configure behavior.
     * Written directly to the RTC MMIO region (backup reg 0 = offset 0x40). */
    {
#define QEMU_SETTING_FIRST_BOOT_LOGIC  0x00000001
#define QEMU_SETTING_START_CONNECTED   0x00000002
#define QEMU_SETTING_START_PLUGGED_IN  0x00000004
        uint32_t qemu_flags = QEMU_SETTING_START_CONNECTED;

        const char *env;
        env = getenv("PEBBLE_QEMU_FIRST_BOOT_LOGIC_ENABLE");
        if (env && atoi(env)) {
            qemu_flags |= QEMU_SETTING_FIRST_BOOT_LOGIC;
        }
        env = getenv("PEBBLE_QEMU_START_CONNECTED");
        if (env && !atoi(env)) {
            qemu_flags &= ~QEMU_SETTING_START_CONNECTED;
        }
        env = getenv("PEBBLE_QEMU_START_PLUGGED_IN");
        if (env && atoi(env)) {
            qemu_flags |= QEMU_SETTING_START_PLUGGED_IN;
        }

        /* Write to RTC backup register 0 via physical memory write */
        uint32_t flags_le = cpu_to_le32(qemu_flags);
        cpu_physical_memory_write(PBL_RTC_BASE + 0x40,
                                  &flags_le, sizeof(flags_le));
    }

    /* Load firmware */
    armv7m_load_kernel(s->armv7m.cpu, machine->kernel_filename,
                       0, cfg->flash_size);
}

/* ===== Machine class hierarchy ===== */

static void pbl_generic_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->init = pbl_generic_init;
    mc->max_cpus = 1;
    mc->ignore_memory_transaction_failures = true;
}

static void pbl_emery_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    PblGenericMachineClass *pmc = PBL_GENERIC_MACHINE_CLASS(oc);
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m33"),
        NULL
    };

    mc->desc = board_cfg_emery.desc;
    mc->default_cpu_type = board_cfg_emery.cpu_type;
    mc->valid_cpu_types = valid_cpu_types;
    pmc->board_cfg = &board_cfg_emery;
    machine_add_audiodev_property(mc);
}

static void pbl_flint_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    PblGenericMachineClass *pmc = PBL_GENERIC_MACHINE_CLASS(oc);
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m4"),
        NULL
    };

    mc->desc = board_cfg_flint.desc;
    mc->default_cpu_type = board_cfg_flint.cpu_type;
    mc->valid_cpu_types = valid_cpu_types;
    pmc->board_cfg = &board_cfg_flint;
    machine_add_audiodev_property(mc);
}

static void pbl_gabbro_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    PblGenericMachineClass *pmc = PBL_GENERIC_MACHINE_CLASS(oc);
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m33"),
        NULL
    };

    mc->desc = board_cfg_gabbro.desc;
    mc->default_cpu_type = board_cfg_gabbro.cpu_type;
    mc->valid_cpu_types = valid_cpu_types;
    pmc->board_cfg = &board_cfg_gabbro;
}

/* ===== Type registration ===== */

static const TypeInfo pbl_generic_info = {
    .name          = TYPE_PBL_GENERIC_MACHINE,
    .parent        = TYPE_MACHINE,
    .abstract      = true,
    .instance_size = sizeof(PblGenericMachineState),
    .class_size    = sizeof(PblGenericMachineClass),
    .class_init    = pbl_generic_class_init,
    /* Single-binary QEMU filters machines per target; without this the
     * pebble machines are invisible to qemu-system-arm. */
    .interfaces    = arm_machine_interfaces,
};

static const TypeInfo pbl_emery_info = {
    .name          = MACHINE_TYPE_NAME("pebble-emery"),
    .parent        = TYPE_PBL_GENERIC_MACHINE,
    .class_init    = pbl_emery_class_init,
};

static const TypeInfo pbl_flint_info = {
    .name          = MACHINE_TYPE_NAME("pebble-flint"),
    .parent        = TYPE_PBL_GENERIC_MACHINE,
    .class_init    = pbl_flint_class_init,
};

static const TypeInfo pbl_gabbro_info = {
    .name          = MACHINE_TYPE_NAME("pebble-gabbro"),
    .parent        = TYPE_PBL_GENERIC_MACHINE,
    .class_init    = pbl_gabbro_class_init,
};

static void pbl_generic_machine_init(void)
{
    type_register_static(&pbl_generic_info);
    type_register_static(&pbl_emery_info);
    type_register_static(&pbl_flint_info);
    type_register_static(&pbl_gabbro_info);
}

type_init(pbl_generic_machine_init)
