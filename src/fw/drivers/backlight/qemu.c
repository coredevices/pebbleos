/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/backlight.h>

#include "board/board.h"
#include "console/prompt.h"
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
#include <pbl/drivers/backlight.h>
#endif

#include <stdlib.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))

// Display register offsets (must match QEMU pebble-display)
#define DISP_CTRL        0x000
#define DISP_BRIGHTNESS  0x018
#define DISP_BL_RED      0x024
#define DISP_BL_GREEN    0x028
#define DISP_BL_BLUE     0x02C
#define CTRL_UPDATE      (1 << 1)

// Brightness levels for QEMU display grayscale path
#define BACKLIGHT_OFF_LEVEL  180
#define BACKLIGHT_ON_LEVEL   255

static bool s_initialized;

#ifdef CONFIG_BACKLIGHT_QEMU_COLOR
static uint8_t s_brightness = 100U;
static uint32_t s_rgb_current_color = BACKLIGHT_COLOR_WHITE;
#endif

#ifdef CONFIG_BACKLIGHT_QEMU_COLOR
static void prv_write_channels(uint8_t r, uint8_t g, uint8_t b) {
  REG32(QEMU_DISPLAY_BASE + DISP_BL_RED) = r;
  REG32(QEMU_DISPLAY_BASE + DISP_BL_GREEN) = g;
  REG32(QEMU_DISPLAY_BASE + DISP_BL_BLUE) = b;
}
#endif

void backlight_init(void) {
#ifdef CONFIG_BACKLIGHT_QEMU_COLOR
  prv_write_channels(0xFFU, 0xFFU, 0xFFU);
#else
  REG32(QEMU_DISPLAY_BASE + DISP_BRIGHTNESS) = BACKLIGHT_OFF_LEVEL;
  REG32(QEMU_DISPLAY_BASE + DISP_CTRL) |= CTRL_UPDATE;
#endif
  s_initialized = true;
}

void backlight_set_brightness(uint8_t brightness) {
  if (!s_initialized) {
    return;
  }

  if (brightness > 100U) {
    brightness = 100U;
  }

#ifdef CONFIG_BACKLIGHT_QEMU_COLOR
  s_brightness = brightness;
  backlight_set_color(s_rgb_current_color);
#else
  uint32_t level;
  if (brightness == 0) {
    level = BACKLIGHT_OFF_LEVEL;
  } else {
    uint32_t range = BACKLIGHT_ON_LEVEL - BACKLIGHT_OFF_LEVEL;
    level = BACKLIGHT_OFF_LEVEL + ((uint32_t)brightness * range) / 100;
    if (level > BACKLIGHT_ON_LEVEL) {
      level = BACKLIGHT_ON_LEVEL;
    }
  }

  REG32(QEMU_DISPLAY_BASE + DISP_BRIGHTNESS) = level;
  REG32(QEMU_DISPLAY_BASE + DISP_CTRL) |= CTRL_UPDATE;
#endif
}

uint8_t backlight_get_level(uint8_t brightness) {
  // Emulated backlight: every brightness value is distinct.
  return brightness;
}

void backlight_refresh(void) {
}

#ifdef CONFIG_BACKLIGHT_QEMU_COLOR
void backlight_set_color(uint32_t rgb_color) {
  uint8_t r = ((rgb_color >> 16) & 0xFFU) * s_brightness / 100U;
  uint8_t g = ((rgb_color >> 8) & 0xFFU) * s_brightness / 100U;
  uint8_t b = (rgb_color & 0xFFU) * s_brightness / 100U;

  prv_write_channels(r, g, b);
  s_rgb_current_color = rgb_color;
}

uint32_t backlight_get_color(void) {
  return s_rgb_current_color;
}
#endif