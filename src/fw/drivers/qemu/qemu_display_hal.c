/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/display/display.h>

#include "board/board.h"
#include "system/passert.h"

#include <string.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))

// Display MMIO register offsets (must match QEMU pebble-display device)
#define DISP_CTRL        0x000
#define DISP_STATUS      0x004
#define DISP_WIDTH       0x008
#define DISP_HEIGHT      0x00C
#define DISP_FORMAT      0x010
#define DISP_FLAGS       0x014
#define DISP_BRIGHTNESS  0x018
#define DISP_INT_STATUS  0x01C
#define DISP_INT_CTRL    0x020

// CTRL register bits
#define CTRL_ENABLE          (1 << 0)
#define CTRL_UPDATE_REQUEST  (1 << 1)

// STATUS register bits
#define STATUS_BUSY          (1 << 0)
#define STATUS_UPDATE_DONE   (1 << 1)

// INT bits
#define INT_UPDATE_DONE_IE      (1 << 0)
#define INT_UPDATE_DONE_PENDING (1 << 0)

static bool s_enabled;
static bool s_updating;

void display_init(void) {
  uint32_t base = DISPLAY->base_addr;

  // Clear any pending interrupts
  REG32(base + DISP_INT_STATUS) = INT_UPDATE_DONE_PENDING;

  // Enable the display
  REG32(base + DISP_CTRL) = CTRL_ENABLE;
  s_enabled = true;
  s_updating = false;
}

void display_clear(void) {
  uint32_t fb_addr = DISPLAY->fb_addr;
  uint16_t width = DISPLAY->width;
  uint16_t height = DISPLAY->height;

#if PBL_BW
  uint32_t fb_size = ROUND_TO_MOD_CEIL_U(width, 32) / 8 * height;
#else
  uint32_t fb_size = width * height;
#endif
  memset((void *)fb_addr, 0, fb_size);

  // Request the QEMU display to update
  REG32(DISPLAY->base_addr + DISP_CTRL) |= CTRL_UPDATE_REQUEST;
}

void display_set_enabled(bool enabled) {
  uint32_t base = DISPLAY->base_addr;
  uint32_t ctrl = REG32(base + DISP_CTRL);

  if (enabled) {
    ctrl |= CTRL_ENABLE;
  } else {
    ctrl &= ~CTRL_ENABLE;
  }
  REG32(base + DISP_CTRL) = ctrl;
  s_enabled = enabled;
}

void display_set_rotated(bool rotated) {
  // QEMU display does not support rotation
  (void)rotated;
}

bool display_update_in_progress(void) {
  return s_updating;
}

void display_update(NextRowCallback nrcb, UpdateCompleteCallback uccb) {
  PBL_ASSERTN(nrcb != NULL);
  PBL_ASSERTN(uccb != NULL);

  uint32_t fb_addr = DISPLAY->fb_addr;
  uint16_t width = DISPLAY->width;
  DisplayRow row;

  s_updating = true;

  // Iterate through rows provided by the compositor
#if PBL_BW
  uint16_t row_bytes = ROUND_TO_MOD_CEIL_U(width, 32) / 8;
#endif
  while (nrcb(&row)) {
#if PBL_BW
    // 1bpp: copy packed row data directly to QEMU framebuffer
    volatile uint8_t *dst = (volatile uint8_t *)(fb_addr + (row.address * row_bytes));
    memcpy((void *)dst, row.data, row_bytes);
#else
    // 8bpp: copy row data directly
    volatile uint8_t *dst = (volatile uint8_t *)(fb_addr + (row.address * width));
    memcpy((void *)dst, row.data, width);
#endif
  }

  // Tell QEMU to refresh the display
  REG32(DISPLAY->base_addr + DISP_CTRL) |= CTRL_UPDATE_REQUEST;

  s_updating = false;

  // In QEMU the update is immediate, so call the completion callback directly
  uccb();
}

void display_update_boot_frame(uint8_t *framebuffer) {
  uint32_t fb_addr = DISPLAY->fb_addr;
  uint16_t width = DISPLAY->width;
  uint16_t height = DISPLAY->height;

#if PBL_BW
  // Boot splash provides 8bpp data; convert to 1bpp packed for QEMU
  uint16_t row_bytes = ROUND_TO_MOD_CEIL_U(width, 32) / 8;
  volatile uint8_t *dst = (volatile uint8_t *)fb_addr;
  memset((void *)dst, 0, row_bytes * height);
  for (uint16_t y = 0; y < height; y++) {
    for (uint16_t x = 0; x < width; x++) {
      uint8_t pixel = framebuffer[y * width + x];
      if (pixel != 0xFF) {  // Non-white = black pixel
        dst[y * row_bytes + x / 8] |= (1 << (x & 7));
      }
    }
  }
#else
  memcpy((void *)fb_addr, framebuffer, width * height);
#endif

  // Request display refresh
  REG32(DISPLAY->base_addr + DISP_CTRL) |= CTRL_UPDATE_REQUEST;
}
