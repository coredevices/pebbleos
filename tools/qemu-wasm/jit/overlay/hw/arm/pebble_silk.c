/*
 * Pebble Silk (Diorite/Flint) Board Init
 *
 * STM32F412-based Pebble with QSPI external flash and SM-LCD display.
 * Ported from QEMU 2.5.0-pebble8 pebble_silk.c to QEMU 10.x APIs.
 *
 * Copyright (c) 2013-2016 Pebble Technology
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/boards.h"
#if __has_include("hw/arm/machines-qom.h")
#include "hw/arm/machines-qom.h"
#endif
#ifndef DEFINE_MACHINE_ARM
#define DEFINE_MACHINE_ARM DEFINE_MACHINE
#endif
#include "hw/arm/pebble.h"
#include "hw/ssi/ssi.h"
#include "hw/core/qdev-properties.h"
#include "system/blockdev.h"
#include "system/block-backend.h"
#include "target/arm/cpu-qom.h"
#include "qemu/error-report.h"

const static PblBoardConfig s_board_config_silk_bb = {
    .dbgserial_uart_index = 0,       /* USART1 */
    .pebble_control_uart_index = 1,  /* USART2 */
    .button_map = {
        { STM32_GPIOC_INDEX, 13, true },
        { STM32_GPIOD_INDEX, 2, true },
        { STM32_GPIOH_INDEX, 0, true },
        { STM32_GPIOH_INDEX, 1, true },
    },
    .gpio_idr_masks = {
        [STM32_GPIOC_INDEX] = 1 << 13,
        [STM32_GPIOD_INDEX] = 1 << 2,
        [STM32_GPIOH_INDEX] = (1 << 1) | (1 << 0),
    },
    .flash_size = 4096,
    .ram_size = 256,
    .num_rows = 172,
    .num_cols = 148,
    .num_border_rows = 2,
    .num_border_cols = 2,
    .row_major = false,
    .row_inverted = false,
    .col_inverted = false,
    .round_mask = false
};

void pebble_32f412_init(MachineState *machine,
                        const PblBoardConfig *board_config)
{
    Stm32Gpio *gpio[STM32F4XX_GPIO_COUNT];
    Stm32Uart *uart[STM32F4XX_UART_COUNT];
    Stm32Timer *timer[STM32F4XX_TIM_COUNT];
    DeviceState *rtc_dev;
    SSIBus *spi;
    struct stm32f4xx stm;
    ARMCPU *cpu;

    stm32f4xx_init(board_config->flash_size,
                   board_config->ram_size,
                   machine->kernel_filename,
                   gpio,
                   board_config->gpio_idr_masks,
                   uart,
                   timer,
                   &rtc_dev,
                   8000000, /* osc_freq */
                   32768,   /* osc32_freq */
                   &stm,
                   &cpu);

    pebble_set_qemu_settings(rtc_dev);

    /* === QSPI Flash (MX25U6435F) === */
    if (stm.qspi_dev) {
        spi = (SSIBus *)qdev_get_child_bus(stm.qspi_dev, "ssi");
        DeviceState *flash_dev = qdev_new("mx25u6435f");
        DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);
        if (dinfo) {
            qdev_prop_set_drive(flash_dev, "drive",
                                blk_by_legacy_dinfo(dinfo));
        }
        if (spi) {
            ssi_realize_and_unref(flash_dev, spi, &error_fatal);
        }
        /* Wire QSPI CS output to the flash's CS input */
        qemu_irq flash_cs = qdev_get_gpio_in_named(flash_dev,
                                                     SSI_GPIO_CS, 0);
        qdev_connect_gpio_out_named(stm.qspi_dev, "qspi-gpio-cs", 0,
                                    flash_cs);
    }

    /* === Display (SM-LCD on SPI2) === */
    spi = (SSIBus *)qdev_get_child_bus(stm.spi_dev[1], "ssi");
    DeviceState *display_dev = qdev_new("sm-lcd");
    qdev_prop_set_bit(display_dev, "rotate_display", false);
    if (spi) {
        ssi_realize_and_unref(display_dev, spi, &error_fatal);
    }

    /* Backlight enable: GPIO B pin 13 */
    qemu_irq backlight_enable = qdev_get_gpio_in_named(display_dev,
                                                        "backlight_enable", 0);
    qdev_connect_gpio_out((DeviceState *)gpio[STM32_GPIOB_INDEX], 13,
                          backlight_enable);

    /* Backlight level: TIM3 (timer[2]) PWM */
    qemu_irq backlight_level = qdev_get_gpio_in_named(display_dev,
                                                       "backlight_level", 0);
    qdev_connect_gpio_out_named((DeviceState *)timer[2],
                                "pwm_ratio_changed", 0, backlight_level);

    /* === UARTs === */
    pebble_connect_uarts(uart, board_config);

    /* === Buttons === */
    pebble_init_buttons(gpio, board_config->button_map);

    /* === Board device (vibrate fan-out) === */
    qemu_irq display_vibe = qdev_get_gpio_in_named(display_dev,
                                                     "vibe_ctl", 0);
    DeviceState *board = pebble_init_board(gpio, display_vibe);

    /* Vibrate: TIM14 (timer[13]) pwm_enable -> board vibe_in */
    qemu_irq board_vibe_in = qdev_get_gpio_in_named(board,
                                                      "pebble_board_vibe_in",
                                                      0);
    qdev_connect_gpio_out_named((DeviceState *)timer[13],
                                "pwm_enable", 0, board_vibe_in);
}

static void pebble_silk_init(MachineState *machine)
{
    pebble_32f412_init(machine, &s_board_config_silk_bb);
}

static void pebble_silk_bb_machine_init(MachineClass *mc)
{
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m4"),
        NULL
    };
    mc->desc = "Pebble smartwatch (silk/diorite)";
    mc->init = pebble_silk_init;
    mc->valid_cpu_types = valid_cpu_types;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE_ARM("pebble-silk-bb", pebble_silk_bb_machine_init)
