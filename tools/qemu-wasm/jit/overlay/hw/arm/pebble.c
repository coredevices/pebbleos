/*
 * Pebble Smartwatch Board/Machine Definitions
 *
 * Ported from QEMU 2.5.0-pebble8 to QEMU 10.x APIs.
 *
 * Copyright (c) 2013, 2014 Pebble Technology
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

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/boards.h"
#ifdef HW_ARM_MACHINES_QOM_H
/* QEMU 10.2+: already included */
#else
/* Try to include for DEFINE_MACHINE_ARM, fall back to DEFINE_MACHINE */
#if __has_include("hw/arm/machines-qom.h")
#include "hw/arm/machines-qom.h"
#endif
#endif

/* Use DEFINE_MACHINE_ARM if available (QEMU 10.2+), else plain DEFINE_MACHINE */
#ifndef DEFINE_MACHINE_ARM
#define DEFINE_MACHINE_ARM DEFINE_MACHINE
#endif
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/ssi/ssi.h"
#include "hw/block/flash.h"
#include "hw/core/loader.h"
#include "system/address-spaces.h"
#include "hw/arm/boot.h"
#include "hw/arm/stm32_common.h"
#include "hw/arm/pebble.h"
#include "hw/misc/pebble_null_input.h"
#include "pebble_control.h"
#include "ui/console.h"
#include "ui/input.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/system.h"

/* #define DEBUG_PEBBLE */
#ifdef DEBUG_PEBBLE
#define DPRINTF(fmt, ...)                                       \
    do { printf("DEBUG_PEBBLE: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

/* ====================================================================
 * Board configurations
 * ==================================================================== */

const static PblBoardConfig s_board_config_snowy_bb = {
    .dbgserial_uart_index = 2,       /* USART3 */
    .pebble_control_uart_index = 1,  /* USART2 */
    .button_map = {
        {STM32_GPIOG_INDEX, 4, false}, /* back */
        {STM32_GPIOG_INDEX, 3, false}, /* up */
        {STM32_GPIOG_INDEX, 1, false}, /* select */
        {STM32_GPIOG_INDEX, 2, false}, /* down */
    },
    .flash_size = 4096,
    .ram_size = 256,
    .num_rows = 172,
    .num_cols = 148,
    .num_border_rows = 2,
    .num_border_cols = 2,
    .row_major = false,
    .row_inverted = true,
    .col_inverted = false,
    .round_mask = false
};

const static PblBoardConfig s_board_config_snowy_emery_bb = {
    .dbgserial_uart_index = 2,       /* USART3 */
    .pebble_control_uart_index = 1,  /* USART2 */
    .button_map = {
        {STM32_GPIOG_INDEX, 4, false}, /* back */
        {STM32_GPIOG_INDEX, 3, false}, /* up */
        {STM32_GPIOG_INDEX, 1, false}, /* select */
        {STM32_GPIOG_INDEX, 2, false}, /* down */
    },
    .flash_size = 4096,
    .ram_size = 512,
    .num_rows = 228,
    .num_cols = 200,
    .num_border_rows = 0,
    .num_border_cols = 0,
    .row_major = true,
    .row_inverted = true,
    .col_inverted = true,
    .round_mask = false
};

const static PblBoardConfig s_board_config_s4_bb = {
    .dbgserial_uart_index = 2,
    .pebble_control_uart_index = 1,
    .button_map = {
        {STM32_GPIOG_INDEX, 4, false},
        {STM32_GPIOG_INDEX, 3, false},
        {STM32_GPIOG_INDEX, 1, false},
        {STM32_GPIOG_INDEX, 2, false},
    },
    .flash_size = 4096,
    .ram_size = 256,
    .num_rows = 180,
    .num_cols = 180,
    .num_border_rows = 0,
    .num_border_cols = 0,
    .row_major = true,
    .row_inverted = false,
    .col_inverted = false,
    .round_mask = true
};

const static PblBoardConfig s_board_config_bb2_ev1_ev2 = {
    .dbgserial_uart_index = 2,       /* USART3 */
    .pebble_control_uart_index = 1,  /* USART2 */
    .button_map = {
        {STM32_GPIOC_INDEX, 3, false},   /* back */
        {STM32_GPIOA_INDEX, 2, false},   /* up */
        {STM32_GPIOC_INDEX, 6, false},   /* select */
        {STM32_GPIOA_INDEX, 1, false},   /* down */
    },
    .flash_size = 4096,
    .ram_size = 128,
    .num_rows = 172,
    .num_cols = 148,
    .num_border_rows = 2,
    .num_border_cols = 2,
    .row_major = false,
    .row_inverted = false,
    .col_inverted = false,
    .round_mask = false
};

const static PblBoardConfig s_board_config_bigboard = {
    .dbgserial_uart_index = 2,       /* USART3 */
    .pebble_control_uart_index = 1,  /* USART2 */
    .button_map = {
        {STM32_GPIOA_INDEX, 2, false}, /* back */
        {STM32_GPIOA_INDEX, 1, false}, /* up */
        {STM32_GPIOA_INDEX, 3, false}, /* select */
        {STM32_GPIOC_INDEX, 9, false}, /* down */
    },
    .flash_size = 4096,
    .ram_size = 128,
    .num_rows = 172,
    .num_cols = 148,
    .num_border_rows = 2,
    .num_border_cols = 2,
    .row_major = false,
    .row_inverted = false,
    .col_inverted = false,
    .round_mask = false
};

const static PblBoardConfig s_board_config_spalding_gabbro_bb = {
    .dbgserial_uart_index = 2,       // USART3
    .pebble_control_uart_index = 1,  // USART2
    .button_map = {
        {STM32_GPIOG_INDEX, 4, false}, // back
        {STM32_GPIOG_INDEX, 3, false}, // up
        {STM32_GPIOG_INDEX, 1, false}, // select
        {STM32_GPIOG_INDEX, 2, false}, // down
    },
    .flash_size = 4096,  /* Kbytes - larger to aid in development and debugging */
    .ram_size = 512,  /* Kbytes */
    .num_rows = 260,
    .num_cols = 260,
    .num_border_rows = 0,
    .num_border_cols = 0,
    .row_major = true,
    .row_inverted = false,
    .col_inverted = false,
    .round_mask = true
};

/* ====================================================================
 * Button handling
 * ==================================================================== */
static PblButtonID s_waiting_key_up_id = PBL_BUTTON_ID_NONE;
static QEMUTimer *s_button_timer;
static PebbleControl *s_pebble_control;
static qemu_irq s_button_irq[PBL_NUM_BUTTONS];
static bool s_buttons_initialized = false;
static qemu_irq s_button_wakeup;

static void prv_send_key_up(void *opaque)
{
    qemu_irq *button_irqs = opaque;
    if (s_waiting_key_up_id == PBL_BUTTON_ID_NONE) {
        return;
    }
    DPRINTF("button %d released\n", s_waiting_key_up_id);
    qemu_set_irq(button_irqs[s_waiting_key_up_id], true);
    qemu_set_irq(s_button_wakeup, false);
    s_waiting_key_up_id = PBL_BUTTON_ID_NONE;
}

static int pebble_qcode_to_button(int qcode)
{
    switch (qcode) {
    case Q_KEY_CODE_Q:       return PBL_BUTTON_ID_BACK;
    case Q_KEY_CODE_W:       return PBL_BUTTON_ID_UP;
    case Q_KEY_CODE_S:       return PBL_BUTTON_ID_SELECT;
    case Q_KEY_CODE_X:       return PBL_BUTTON_ID_DOWN;
    case Q_KEY_CODE_UP:      return PBL_BUTTON_ID_UP;
    case Q_KEY_CODE_DOWN:    return PBL_BUTTON_ID_DOWN;
    case Q_KEY_CODE_LEFT:    return PBL_BUTTON_ID_BACK;
    case Q_KEY_CODE_RIGHT:   return PBL_BUTTON_ID_SELECT;
    default:                 return PBL_BUTTON_ID_NONE;
    }
}

static void pebble_input_event(DeviceState *dev, QemuConsole *src,
                                InputEvent *evt)
{
    InputKeyEvent *key;
    int qcode, button_id;
    bool pressed;

    if (evt->type != INPUT_EVENT_KIND_KEY) {
        return;
    }
    key = evt->u.key.data;
    qcode = qemu_input_key_value_to_qcode(key->key);
    pressed = key->down;
    button_id = pebble_qcode_to_button(qcode);

    if (button_id == PBL_BUTTON_ID_NONE || !pressed) {
        return;
    }

    if (s_waiting_key_up_id != PBL_BUTTON_ID_NONE &&
        button_id != s_waiting_key_up_id) {
        prv_send_key_up(s_button_irq);
    }

    if (s_waiting_key_up_id != button_id) {
        DPRINTF("button %d pressed\n", button_id);
        s_waiting_key_up_id = button_id;
        qemu_set_irq(s_button_irq[button_id], false);
        qemu_set_irq(s_button_wakeup, true);
    }

    if (!s_button_timer) {
        s_button_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, prv_send_key_up,
                                      s_button_irq);
    }
    timer_mod(s_button_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 250);
}

static const QemuInputHandler pebble_keyboard_handler = {
    .name  = "Pebble Buttons",
    .mask  = INPUT_EVENT_MASK_KEY,
    .event = pebble_input_event,
};

void pebble_set_button_state(uint32_t button_state)
{
    if (!s_buttons_initialized) {
        return;
    }
    int button_id;
    for (button_id = 0; button_id < PBL_NUM_BUTTONS; button_id++) {
        uint32_t mask = 1 << button_id;
        qemu_set_irq(s_button_irq[button_id], !(button_state & mask));
    }
}

#ifdef __EMSCRIPTEN__
#include <emscripten.h>

/* Shared button state for JavaScript → QEMU communication.
 * JavaScript writes button bitmask, QEMU timer reads and applies it.
 * Bit 0=back, 1=up, 2=select, 3=down. */
static uint32_t pebble_wasm_button_state = 0;
static uint32_t pebble_wasm_last_button_state = 0;
static QEMUTimer *pebble_wasm_button_timer;

/* Export the address of button state so JavaScript can write directly
 * to shared memory via Atomics.store(), bypassing slow PROXY_TO_PTHREAD
 * function call proxying. */
EMSCRIPTEN_KEEPALIVE uint32_t pebble_button_state_addr(void)
{
    return (uint32_t)(uintptr_t)&pebble_wasm_button_state;
}

EMSCRIPTEN_KEEPALIVE void pebble_set_buttons(uint32_t state)
{
    __atomic_store_n(&pebble_wasm_button_state, state, __ATOMIC_SEQ_CST);
}

static void pebble_wasm_button_poll(void *opaque)
{
    uint32_t state = __atomic_load_n(&pebble_wasm_button_state, __ATOMIC_SEQ_CST);
    if (state != pebble_wasm_last_button_state) {
        pebble_set_button_state(state);
        pebble_wasm_last_button_state = state;
    }
    timer_mod(pebble_wasm_button_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 16);
}
#endif

/* ====================================================================
 * UART connections
 * ==================================================================== */
void pebble_connect_uarts(Stm32Uart *uart[],
                           const PblBoardConfig *board_config)
{
    /* serial_hd(1) = pebble control channel (host<->emulated Pebble protocol)
     * serial_hd(2) = debug serial (GDB/console)
     */
    Chardev *control_chr = serial_hd(1);
    Chardev *debug_chr = serial_hd(2);

    s_pebble_control = pebble_control_create(
        control_chr,
        uart[board_config->pebble_control_uart_index]);

    stm32_uart_connect(uart[board_config->dbgserial_uart_index],
                       debug_chr, 0);
}

/* ====================================================================
 * Button initialization
 * ==================================================================== */
void pebble_init_buttons(Stm32Gpio *gpio[], const PblButtonMap *map)
{
    int i;
    for (i = 0; i < PBL_NUM_BUTTONS; i++) {
        qemu_irq irq = qdev_get_gpio_in((DeviceState *)gpio[map[i].gpio],
                                         map[i].pin);
        if (map[i].active_high) {
            s_button_irq[i] = qemu_irq_invert(irq);
        } else {
            s_button_irq[i] = irq;
        }
    }
    s_buttons_initialized = true;
    s_button_wakeup = qdev_get_gpio_in((DeviceState *)gpio[STM32_GPIOA_INDEX],
                                        0);
    QemuInputHandlerState *ihs =
        qemu_input_handler_register(NULL, &pebble_keyboard_handler);
    qemu_input_handler_activate(ihs);

#ifdef __EMSCRIPTEN__
    /* Start polling timer for WASM button input */
    pebble_wasm_button_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                             pebble_wasm_button_poll, NULL);
    timer_mod(pebble_wasm_button_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
#endif
}

/* ====================================================================
 * Board device (fan-out GPIO for vibrate)
 * ==================================================================== */
typedef struct PebbleBoard {
    SysBusDevice parent_obj;
    qemu_irq vibe_out_irq;
} PebbleBoard;

#define TYPE_PEBBLE_BOARD "pebble-board"
#define PEBBLE_BOARD(obj) OBJECT_CHECK(PebbleBoard, (obj), TYPE_PEBBLE_BOARD)

static void pebble_board_vibe_ctl(void *opaque, int n, int level)
{
    PebbleBoard *s = (PebbleBoard *)opaque;
    assert(n == 0);
    pebble_control_send_vibe_notification(s_pebble_control, level != 0);
    qemu_set_irq(s->vibe_out_irq, level);
}

static void pebble_board_realize(DeviceState *dev, Error **errp)
{
    qdev_init_gpio_in_named(dev, pebble_board_vibe_ctl,
                            "pebble_board_vibe_in", 1);
}

static void pebble_board_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = pebble_board_realize;
}

static const TypeInfo pebble_board_info = {
    .name          = TYPE_PEBBLE_BOARD,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PebbleBoard),
    .class_init    = pebble_board_class_init,
};

static void pebble_board_register_types(void)
{
    type_register_static(&pebble_board_info);
}

type_init(pebble_board_register_types)

DeviceState *pebble_init_board(Stm32Gpio *gpio[], qemu_irq display_vibe)
{
    DeviceState *board = qdev_new(TYPE_PEBBLE_BOARD);
    PebbleBoard *s = PEBBLE_BOARD(board);
    s->vibe_out_irq = display_vibe;
    sysbus_realize_and_unref(SYS_BUS_DEVICE(board), &error_fatal);
    return board;
}

/* ====================================================================
 * QEMU-specific RTC settings
 * ==================================================================== */
void pebble_set_qemu_settings(DeviceState *rtc_dev)
{
#define QEMU_REG_0_FIRST_BOOT_LOGIC_ENABLE  0x00000001
#define QEMU_REG_0_START_CONNECTED          0x00000002
#define QEMU_REG_0_START_PLUGGED_IN         0x00000004

    uint32_t flags = QEMU_REG_0_START_CONNECTED;
    char *strval;

    strval = getenv("PEBBLE_QEMU_FIRST_BOOT_LOGIC_ENABLE");
    if (strval) {
        if (atoi(strval)) {
            flags |= QEMU_REG_0_FIRST_BOOT_LOGIC_ENABLE;
        } else {
            flags &= ~QEMU_REG_0_FIRST_BOOT_LOGIC_ENABLE;
        }
    }

    strval = getenv("PEBBLE_QEMU_START_CONNECTED");
    if (strval) {
        if (atoi(strval)) {
            flags |= QEMU_REG_0_START_CONNECTED;
        } else {
            flags &= ~QEMU_REG_0_START_CONNECTED;
        }
    }

    strval = getenv("PEBBLE_QEMU_START_PLUGGED_IN");
    if (strval) {
        if (atoi(strval)) {
            flags |= QEMU_REG_0_START_PLUGGED_IN;
        } else {
            flags &= ~QEMU_REG_0_START_PLUGGED_IN;
        }
    }

    f2xx_rtc_set_extra_bkup_reg(rtc_dev, 0, flags);
}

/* ====================================================================
 * STM32F2xx-based Pebble init (aplite/tintin — bb, bb2)
 * ==================================================================== */
void pebble_32f2_init(MachineState *machine,
                       const PblBoardConfig *board_config)
{
    Stm32Gpio *gpio[STM32F2XX_GPIO_COUNT];
    Stm32Uart *uart[STM32F2XX_UART_COUNT];
    Stm32Timer *timer[STM32F2XX_TIM_COUNT];
    DeviceState *rtc_dev;
    SSIBus *spi;
    struct stm32f2xx stm;
    ARMCPU *cpu;

    stm32f2xx_init(board_config->flash_size,
                   board_config->ram_size,
                   machine->kernel_filename,
                   gpio,
                   uart,
                   timer,
                   &rtc_dev,
                   8000000, /* osc_freq */
                   32768,   /* osc32_freq */
                   &stm,
                   &cpu);

    pebble_set_qemu_settings(rtc_dev);

    /* SPI flash (n25q032a11 on SPI1, CS via GPIO A pin 4) */
    spi = (SSIBus *)qdev_get_child_bus(stm.spi_dev[0], "ssi");
    DeviceState *spi_flash = qdev_new("n25q032a11");
    DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);
    if (dinfo) {
        qdev_prop_set_drive(spi_flash, "drive", blk_by_legacy_dinfo(dinfo));
    }
    if (spi) {
        ssi_realize_and_unref(spi_flash, spi, &error_fatal);
    }
    qemu_irq cs = qdev_get_gpio_in_named(spi_flash, SSI_GPIO_CS, 0);
    qdev_connect_gpio_out((DeviceState *)gpio[STM32_GPIOA_INDEX], 4, cs);

    /* Display (sm-lcd on SPI2) */
    spi = (SSIBus *)qdev_get_child_bus(stm.spi_dev[1], "ssi");
    DeviceState *display_dev = qdev_new("sm-lcd");
    qdev_prop_set_bit(display_dev, "rotate_display", true);
    if (spi) {
        ssi_realize_and_unref(display_dev, spi, &error_fatal);
    }

    qemu_irq backlight_enable = qdev_get_gpio_in_named(display_dev,
                                                        "backlight_enable", 0);
    qdev_connect_gpio_out_named((DeviceState *)gpio[STM32_GPIOB_INDEX],
                                "af", 5, backlight_enable);

    qemu_irq backlight_level = qdev_get_gpio_in_named(display_dev,
                                                       "backlight_level", 0);
    qdev_connect_gpio_out_named((DeviceState *)timer[2],
                                "pwm_ratio_changed", 0, backlight_level);

    /* Connect UARTs */
    pebble_connect_uarts(uart, board_config);

    /* Init buttons */
    pebble_init_buttons(gpio, board_config->button_map);

    /* Board device (vibrate fan-out) */
    qemu_irq display_vibe = qdev_get_gpio_in_named(display_dev,
                                                     "vibe_ctl", 0);
    DeviceState *board = pebble_init_board(gpio, display_vibe);

    qemu_irq board_vibe_in = qdev_get_gpio_in_named(board,
                                                      "pebble_board_vibe_in",
                                                      0);
    qdev_connect_gpio_out((DeviceState *)gpio[STM32_GPIOB_INDEX], 0,
                          board_vibe_in);

    /* No-op absolute-pointer handler: stops the UI from grabbing the host
     * cursor on click for these non-touch boards. */
    qdev_realize_and_unref(qdev_new(TYPE_PEBBLE_NULL_INPUT), NULL,
                           &error_fatal);
}

/* ====================================================================
 * STM32F439-based Pebble init (snowy, emery, chalk/s4)
 * ==================================================================== */
void pebble_32f439_init(MachineState *machine,
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

    /* Storage flash (NOR-flash on Snowy/Emery) - 16MB at 0x60000000.
     * Use pflash_cfi02 (AMD/JEDEC compatible) to emulate Macronix MX29VS128FB.
     * Pass via: -drive if=none,id=spi-flash,file=qemu_spi_flash.bin,format=raw
     */
    {
        const uint32_t flash_size_bytes = 16 * 1024 * 1024;
        const uint32_t sector_size = 32 * 1024;
        BlockBackend *blk = blk_by_name("spi-flash");
        if (!blk) {
            fprintf(stderr, "WARNING: pflash drive 'spi-flash' not found, "
                            "flash will be empty\n");
        }
        pflash_cfi02_register(0x60000000,
                              "pebble.spi_flash",
                              flash_size_bytes,
                              blk,
                              sector_size,
                              1,      /* nb_mappings */
                              2,      /* width (16-bit) */
                              0x00c2, /* id0: Macronix */
                              0x007e, /* id1 */
                              0x0065, /* id2 */
                              0x0001, /* id3 */
                              0x555,  /* unlock_addr0 */
                              0x2AA,  /* unlock_addr1 */
                              0);     /* big_endian = false */
    }


    /* === Display === */
    spi = (SSIBus *)qdev_get_child_bus(stm.spi_dev[5], "ssi");
    DeviceState *display_dev = qdev_new("pebble-snowy-display");

    qemu_irq display_done_irq = qdev_get_gpio_in(
        (DeviceState *)gpio[STM32_GPIOG_INDEX], 9);
    qemu_irq display_intn_irq = qdev_get_gpio_in(
        (DeviceState *)gpio[STM32_GPIOG_INDEX], 10);

    qdev_prop_set_int32(display_dev, "num_rows", board_config->num_rows);
    qdev_prop_set_int32(display_dev, "num_cols", board_config->num_cols);
    qdev_prop_set_int32(display_dev, "num_border_rows",
                        board_config->num_border_rows);
    qdev_prop_set_int32(display_dev, "num_border_cols",
                        board_config->num_border_cols);
    qdev_prop_set_uint8(display_dev, "row_major", board_config->row_major);
    qdev_prop_set_uint8(display_dev, "row_inverted",
                        board_config->row_inverted);
    qdev_prop_set_uint8(display_dev, "col_inverted",
                        board_config->col_inverted);
    qdev_prop_set_uint8(display_dev, "round_mask", board_config->round_mask);

    if (!spi) {
        error_report("SPI6 bus not found - display cannot be attached");
    } else {
        ssi_realize_and_unref(display_dev, spi, &error_fatal);
    }

    /* Connect GPIO outputs to display inputs */
    qemu_irq display_cs = qdev_get_gpio_in_named(display_dev, SSI_GPIO_CS, 0);
    qdev_connect_gpio_out((DeviceState *)gpio[STM32_GPIOG_INDEX], 8,
                          display_cs);

    qemu_irq display_reset = qdev_get_gpio_in_named(display_dev, "reset", 0);
    qdev_connect_gpio_out((DeviceState *)gpio[STM32_GPIOG_INDEX], 15,
                          display_reset);

    qemu_irq display_sclk = qdev_get_gpio_in_named(display_dev, "sclk", 0);
    qdev_connect_gpio_out((DeviceState *)gpio[STM32_GPIOG_INDEX], 13,
                          display_sclk);

    /* Connect display outputs to GPIO inputs (DONE and INTN signals) */
    qdev_connect_gpio_out_named(display_dev, "done_output", 0,
                                display_done_irq);
    qdev_connect_gpio_out_named(display_dev, "intn_output", 0,
                                display_intn_irq);

    qemu_irq backlight_enable = qdev_get_gpio_in_named(display_dev,
                                                        "backlight_enable", 0);
    qdev_connect_gpio_out_named((DeviceState *)gpio[STM32_GPIOB_INDEX],
                                "af", 14, backlight_enable);

    qemu_irq backlight_level = qdev_get_gpio_in_named(display_dev,
                                                       "backlight_level", 0);
    qdev_connect_gpio_out_named((DeviceState *)timer[11],
                                "pwm_ratio_changed", 0, backlight_level);

    /* Connect UARTs */
    pebble_connect_uarts(uart, board_config);

    /* Init buttons */
    pebble_init_buttons(gpio, board_config->button_map);

    /* Board device (vibrate fan-out) */
    qemu_irq display_vibe = qdev_get_gpio_in_named(display_dev,
                                                     "vibe_ctl", 0);
    DeviceState *board = pebble_init_board(gpio, display_vibe);

    qemu_irq board_vibe_in = qdev_get_gpio_in_named(board,
                                                      "pebble_board_vibe_in",
                                                      0);
    qdev_connect_gpio_out((DeviceState *)gpio[STM32_GPIOF_INDEX], 4,
                          board_vibe_in);

    /* No-op absolute-pointer handler: stops the UI from grabbing the host
     * cursor on click for these non-touch boards. */
    qdev_realize_and_unref(qdev_new(TYPE_PEBBLE_NULL_INPUT), NULL,
                           &error_fatal);
}

/* ====================================================================
 * Machine definitions
 * ==================================================================== */

static void pebble_snowy_init(MachineState *machine)
{
    pebble_32f439_init(machine, &s_board_config_snowy_bb);
}

static void pebble_snowy_emery_init(MachineState *machine)
{
    pebble_32f439_init(machine, &s_board_config_snowy_emery_bb);
}

static void pebble_s4_init(MachineState *machine)
{
    pebble_32f439_init(machine, &s_board_config_s4_bb);
}

/* --- Machine class inits --- */

static void pebble_snowy_bb_machine_init(MachineClass *mc)
{
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m4"),
        NULL
    };
    mc->desc = "Pebble smartwatch (snowy/basalt)";
    mc->init = pebble_snowy_init;
    mc->valid_cpu_types = valid_cpu_types;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE_ARM("pebble-snowy-bb", pebble_snowy_bb_machine_init)

static void pebble_snowy_emery_machine_init(MachineClass *mc)
{
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m4"),
        NULL
    };
    mc->desc = "Pebble smartwatch (snowy, but emery)";
    mc->init = pebble_snowy_emery_init;
    mc->valid_cpu_types = valid_cpu_types;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE_ARM("pebble-snowy-emery-bb", pebble_snowy_emery_machine_init)

static void pebble_s4_bb_machine_init(MachineClass *mc)
{
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m4"),
        NULL
    };
    mc->desc = "Pebble smartwatch (chalk/s4)";
    mc->init = pebble_s4_init;
    mc->valid_cpu_types = valid_cpu_types;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE_ARM("pebble-s4-bb", pebble_s4_bb_machine_init)

static void pebble_spalding_gabbro_init(MachineState *machine)
{
    pebble_32f439_init(machine, &s_board_config_spalding_gabbro_bb);
}

static void pebble_spalding_gabbro_machine_init(MachineClass *mc)
{
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m4"),
        NULL
    };
    mc->desc = "Pebble smartwatch (spalding, but gabbro)";
    mc->init = pebble_spalding_gabbro_init;
    mc->valid_cpu_types = valid_cpu_types;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE_ARM("pebble-spalding-gabbro-bb", pebble_spalding_gabbro_machine_init)

/* --- Aplite (STM32F2xx) machines --- */

static void pebble_bb2_init(MachineState *machine)
{
    pebble_32f2_init(machine, &s_board_config_bb2_ev1_ev2);
}

static void pebble_bb2_machine_init(MachineClass *mc)
{
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m3"),
        NULL
    };
    mc->desc = "Pebble smartwatch (bb2/ev1/ev2)";
    mc->init = pebble_bb2_init;
    mc->valid_cpu_types = valid_cpu_types;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE_ARM("pebble-bb2", pebble_bb2_machine_init)

static void pebble_bb_init(MachineState *machine)
{
    pebble_32f2_init(machine, &s_board_config_bigboard);
}

static void pebble_bb_machine_init(MachineClass *mc)
{
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m3"),
        NULL
    };
    mc->desc = "Pebble smartwatch (bb)";
    mc->init = pebble_bb_init;
    mc->valid_cpu_types = valid_cpu_types;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE_ARM("pebble-bb", pebble_bb_machine_init)
