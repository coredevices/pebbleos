/*
 * Pebble Generic Machine Types
 *
 * Generic ARMv7M-based machine types for Pebble smartwatches.
 * These use simple custom MMIO peripherals rather than emulating
 * specific MCU register sets (NRF52840, SF32LB52, etc.).
 *
 * Copyright (c) 2026 Core Devices LLC
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_PEBBLE_GENERIC_H
#define HW_ARM_PEBBLE_GENERIC_H

#include "hw/arm/armv7m.h"
#include "hw/core/boards.h"
#include "hw/core/qdev-clock.h"
#include "qom/object.h"

/* ===== Board IDs ===== */
#define PBL_BOARD_ID_FLINT   1
#define PBL_BOARD_ID_EMERY   2
#define PBL_BOARD_ID_GABBRO  3

/* ===== Peripheral type names ===== */
#define TYPE_PEBBLE_SIMPLE_UART   "pebble-simple-uart"
#define TYPE_PEBBLE_SYSCTRL       "pebble-sysctrl"
#define TYPE_PEBBLE_GENERIC_RTC   "pebble-rtc"
#define TYPE_PEBBLE_GENERIC_TIMER "pebble-timer"
#define TYPE_PEBBLE_DISPLAY       "pebble-display"
#define TYPE_PEBBLE_EXTFLASH      "pebble-extflash"
#define TYPE_PEBBLE_GPIO          "pebble-gpio"
#define TYPE_PEBBLE_TOUCH         "pebble-touch"
#define TYPE_PEBBLE_AUDIO         "pebble-audio"

/* ===== Memory map ===== */
#define PBL_FLASH_BASE          0x00000000
#define PBL_EXTFLASH_BASE       0x10000000
#define PBL_SRAM_BASE           0x20000000

#define PBL_UART0_BASE          0x40000000
#define PBL_UART1_BASE          0x40001000
#define PBL_UART2_BASE          0x40002000
#define PBL_TIMER0_BASE         0x40003000
#define PBL_TIMER1_BASE         0x40004000
#define PBL_RTC_BASE            0x40005000
#define PBL_GPIO_BASE           0x40006000
#define PBL_SYSCTRL_BASE        0x40007000
#define PBL_DISPLAY_BASE        0x40008000
#define PBL_DISPLAY_FB_BASE     0x50000000
#define PBL_DISPLAY_FB_MAX      (128 * 1024) /* 128K max framebuffer */
#define PBL_EXTFLASH_CTRL_BASE  0x40010000
#define PBL_EXTFLASH_SIZE       (32 * 1024 * 1024)  /* 32 MB */
#define PBL_TOUCH_BASE          0x40011000
#define PBL_AUDIO_BASE          0x40012000

/* ===== IRQ numbers (NVIC external interrupts) ===== */
#define PBL_IRQ_UART0     0
#define PBL_IRQ_UART1     1
#define PBL_IRQ_UART2     2
#define PBL_IRQ_TIMER0    3
#define PBL_IRQ_TIMER1    4
#define PBL_IRQ_RTC       5
#define PBL_IRQ_GPIO      6
#define PBL_IRQ_DISPLAY   7
#define PBL_IRQ_TOUCH     9
#define PBL_IRQ_AUDIO     10

#define PBL_NUM_IRQS      32

/* ===== Clock frequencies ===== */
#define PBL_SYSCLK_FRQ    64000000   /* 64 MHz */
#define PBL_REFCLK_FRQ    1000000    /* 1 MHz (systick ref) */

/* ===== Board configuration ===== */
typedef enum {
    PBL_BOARD_FLINT,
    PBL_BOARD_EMERY,
    PBL_BOARD_GABBRO,
} PblBoardType;

typedef struct {
    const char *name;
    const char *desc;
    const char *cpu_type;
    PblBoardType board_type;
    uint32_t board_id;
    uint32_t flash_size;      /* bytes */
    uint32_t ram_size;        /* bytes */
    uint32_t sysclk_frq;     /* Hz */
    /* Display (for Phase 2) */
    uint16_t display_width;
    uint16_t display_height;
    uint8_t  display_bpp;
    bool     display_round;
    /* Feature flags */
    bool has_touch;
    bool has_audio;
} PblGenericBoardConfig;

/* ===== Machine state ===== */
struct PblGenericMachineClass {
    MachineClass parent;
    const PblGenericBoardConfig *board_cfg;
};

struct PblGenericMachineState {
    MachineState parent;

    ARMv7MState armv7m;
    MemoryRegion flash;
    MemoryRegion sram;

    Clock *sysclk;
    Clock *refclk;
};

#define TYPE_PBL_GENERIC_MACHINE "pebble-generic"
OBJECT_DECLARE_TYPE(PblGenericMachineState, PblGenericMachineClass,
                    PBL_GENERIC_MACHINE)

#endif /* HW_ARM_PEBBLE_GENERIC_H */
