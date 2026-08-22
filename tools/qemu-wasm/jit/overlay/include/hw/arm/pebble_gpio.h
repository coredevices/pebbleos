/*
 * Pebble Generic GPIO — public API
 *
 * Copyright (c) 2026 Core Devices LLC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_PEBBLE_GPIO_H
#define HW_ARM_PEBBLE_GPIO_H

#include <stdint.h>

/* Button bit positions (matching pebble_gpio.c register layout) */
#define PBL_BTN_BACK    (1 << 0)
#define PBL_BTN_UP      (1 << 1)
#define PBL_BTN_SELECT  (1 << 2)
#define PBL_BTN_DOWN    (1 << 3)

/* Set button state from a bitmask. Called by pebble_control protocol handler. */
void pbl_gpio_set_button_state(uint32_t button_state);

#endif /* HW_ARM_PEBBLE_GPIO_H */
