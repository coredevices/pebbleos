/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/thread.h"
#include "pbl/services/boot_splash.h"

#if defined(CONFIG_PBLBOOT) || defined(CONFIG_QEMU)

#include "board/display.h"
#include "board/splash.h"
#include <pbl/drivers/display/display.h>
#include "kernel/pbl_malloc.h"
#include "kernel/util/sleep.h"

#include <string.h>

// Platform-specific colors: PebbleOS uses ARGB2222 (GColor8), not raw RGB332
#ifdef CONFIG_QEMU
// QEMU display natively renders ARGB2222
#define SPLASH_COLOR_WHITE  0xFF  // A=3, R=3, G=3, B=3
#define SPLASH_COLOR_BLACK  0xC0  // A=3, R=0, G=0, B=0
#define SPLASH_COLOR_LGRAY  0xEA  // A=3, R=2, G=2, B=2
// QEMU boot splash always uses 8bpp framebuffer regardless of platform bit depth
#define BOOT_SPLASH_FB_BYTES (PBL_DISPLAY_WIDTH * PBL_DISPLAY_HEIGHT)
#else
// Hardware displays: the compositor handles GColor8 → native conversion,
// but boot_splash writes raw pixels before the compositor is initialized.
// On obelix/getafix the LCDC accepts RGB332 directly.
#define SPLASH_COLOR_WHITE  0xFF
#define SPLASH_COLOR_BLACK  0x00
#define SPLASH_COLOR_LGRAY  0xB6
#define BOOT_SPLASH_FB_BYTES DISPLAY_FRAMEBUFFER_BYTES
#endif

// Progress bar configuration
#define PROGRESS_BAR_WIDTH      80
#define PROGRESS_BAR_HEIGHT     4
#define PROGRESS_INDICATOR_WIDTH 20
#define PROGRESS_FRAME_DELAY_MS 100
#define PROGRESS_TOTAL_FRAMES   20
#define BOOT_SPLASH_TASK_STACK_SIZE 2048
#define BOOT_SPLASH_TASK_PRIORITY   (PBL_PRIO_MAX - 1)

// Boot splash state
static struct pbl_thread s_boot_splash_thread;
PBL_THREAD_STACK_DEFINE(s_boot_splash_stack, BOOT_SPLASH_TASK_STACK_SIZE);
static struct pbl_thread *s_boot_splash_task;
static volatile bool s_boot_splash_running;
static uint8_t *s_boot_splash_fb;

// Draw a filled rectangle
static void prv_draw_filled_rect(uint8_t *fb, int16_t x0, int16_t y0,
                                 int16_t width, int16_t height, uint8_t color) {
  for (int16_t y = y0; y < y0 + height; y++) {
    for (int16_t x = x0; x < x0 + width; x++) {
      if (x >= 0 && x < PBL_DISPLAY_WIDTH && y >= 0 && y < PBL_DISPLAY_HEIGHT) {
        fb[(uint32_t)y * PBL_DISPLAY_WIDTH + x] = color;
      }
    }
  }
}

// Draw progress bar with animated indicator
static void prv_draw_progress_bar(uint8_t *fb, int16_t center_x, int16_t center_y,
                                  uint16_t frame) {
  const uint8_t COLOR_TRACK = SPLASH_COLOR_LGRAY;
  const uint8_t COLOR_INDICATOR = SPLASH_COLOR_BLACK;

  int16_t bar_x0 = center_x - PROGRESS_BAR_WIDTH / 2;
  int16_t bar_y0 = center_y - PROGRESS_BAR_HEIGHT / 2;

  // Draw track (background)
  prv_draw_filled_rect(fb, bar_x0, bar_y0, PROGRESS_BAR_WIDTH, PROGRESS_BAR_HEIGHT, COLOR_TRACK);

  // Calculate indicator position (ping-pong animation)
  int32_t max_travel = PROGRESS_BAR_WIDTH - PROGRESS_INDICATOR_WIDTH;
  int32_t half_frames = PROGRESS_TOTAL_FRAMES / 2;
  int32_t cycle_frame = frame % PROGRESS_TOTAL_FRAMES;
  int32_t indicator_offset;

  if (cycle_frame < half_frames) {
    // Moving right
    indicator_offset = (cycle_frame * max_travel) / half_frames;
  } else {
    // Moving left
    indicator_offset = max_travel - ((cycle_frame - half_frames) * max_travel) / half_frames;
  }

  // Draw indicator
  prv_draw_filled_rect(fb, bar_x0 + indicator_offset, bar_y0,
                       PROGRESS_INDICATOR_WIDTH, PROGRESS_BAR_HEIGHT, COLOR_INDICATOR);
}

// Boot splash task - runs until stopped
static void prv_boot_splash_task(void *param) {
  uint8_t *fb = s_boot_splash_fb;
  uint16_t frame = 0;

  // Get splash logo dimensions
  const uint16_t logo_width = splash_width;
  const uint16_t logo_height = splash_height;
  const uint8_t *logo_data = splash_bits;

  // Calculate logo position (centered)
  uint16_t logo_x0 = (PBL_DISPLAY_WIDTH - logo_width) / 2;
  uint16_t logo_y0 = (PBL_DISPLAY_HEIGHT - logo_height) / 2;

  // Calculate progress bar center position (below centered logo)
  int16_t progress_cx = PBL_DISPLAY_WIDTH / 2;
  int16_t progress_cy = logo_y0 + logo_height + 20;

  // Animation loop - runs until s_boot_splash_running is cleared
  while (s_boot_splash_running) {
    // Clear framebuffer to white
    memset(fb, SPLASH_COLOR_WHITE, BOOT_SPLASH_FB_BYTES);

    // Draw logo
    for (uint16_t y = 0U; y < logo_height; y++) {
      for (uint16_t x = 0U; x < logo_width; x++) {
        if (logo_data[y * (logo_width / 8) + x / 8] & (0x1U << (x & 7))) {
          fb[(y + logo_y0) * PBL_DISPLAY_WIDTH + (x + logo_x0)] = SPLASH_COLOR_BLACK;
        }
      }
    }

    // Draw progress bar
    prv_draw_progress_bar(fb, progress_cx, progress_cy, frame);

    // Send frame to display
    display_update_boot_frame(fb);

    frame++;

    // Wait for next frame using short delays to allow quick stop
    for (uint8_t i = 0; i < (PROGRESS_FRAME_DELAY_MS / 10) && s_boot_splash_running; i++) {
      pbl_thread_sleep(PBL_MSEC(10));
    }
  }

  // Cleanup
  kernel_free(fb);
  s_boot_splash_fb = NULL;
  s_boot_splash_task = NULL;
  pbl_thread_abort(NULL);
}

void boot_splash_start(void) {
  // Initialize the display
  display_init();

  // Allocate framebuffer for boot splash
  s_boot_splash_fb = kernel_malloc(BOOT_SPLASH_FB_BYTES);
  if (!s_boot_splash_fb) {
    return;
  }

  // Start the boot splash task
  s_boot_splash_running = true;
  struct pbl_thread_attr attr = {
    .name = "BootSplash",
    .entry = prv_boot_splash_task,
    .prio = BOOT_SPLASH_TASK_PRIORITY,
    .privileged = true,
    .stack = s_boot_splash_stack,
    .stack_size = sizeof(s_boot_splash_stack),
  };
  if (pbl_thread_create(&s_boot_splash_thread, &attr) == 0) {
    s_boot_splash_task = &s_boot_splash_thread;
  }
}

void boot_splash_stop(void) {
  if (s_boot_splash_running && s_boot_splash_task != NULL) {
    s_boot_splash_running = false;
    // Wait for the splash task to finish (max ~10ms delay due to short sleep intervals)
    while (s_boot_splash_task != NULL) {
      psleep(10);
    }

    // Draw final frame with logo only (no progress bar)
    uint8_t *fb = kernel_malloc(BOOT_SPLASH_FB_BYTES);
    if (fb) {
      memset(fb, SPLASH_COLOR_WHITE, BOOT_SPLASH_FB_BYTES);

      // Get splash logo dimensions
      const uint16_t logo_width = splash_width;
      const uint16_t logo_height = splash_height;
      const uint8_t *logo_data = splash_bits;

      // Calculate logo position (centered, same as animation)
      uint16_t logo_x0 = (PBL_DISPLAY_WIDTH - logo_width) / 2;
      uint16_t logo_y0 = (PBL_DISPLAY_HEIGHT - logo_height) / 2;

      // Draw logo
      for (uint16_t y = 0U; y < logo_height; y++) {
        for (uint16_t x = 0U; x < logo_width; x++) {
          if (logo_data[y * (logo_width / 8) + x / 8] & (0x1U << (x & 7))) {
            fb[(y + logo_y0) * PBL_DISPLAY_WIDTH + (x + logo_x0)] = SPLASH_COLOR_BLACK;
          }
        }
      }

      display_update_boot_frame(fb);
      kernel_free(fb);
    }
  }
}

#else // Everything else

void boot_splash_start(void) {
  // No-op on platforms where the bootloader handles the splash
}

void boot_splash_stop(void) {
  // No-op on platforms where the bootloader handles the splash
}

#endif
