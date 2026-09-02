/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "board/board.h"
#include <pbl/drivers/touch/touch_sensor.h>
#include "pbl/services/system_task.h"
#include "pbl/services/touch/touch.h"

#include <cmsis_core.h>
#include <stdbool.h>
#include <stdint.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))

// QEMU touch register offsets (must match pebble-touch device)
#define TOUCH_STATE     0x00
#define TOUCH_X         0x04
#define TOUCH_Y         0x08
#define TOUCH_INTCTRL   0x0C
#define TOUCH_INTSTAT   0x10

#define INT_TOUCH_EVENT (1u << 0)

static bool s_callback_scheduled = false;

static void prv_process_touch_update(void *unused) {
  s_callback_scheduled = false;

  const uint32_t base = QEMU_TOUCH_BASE;
  const uint32_t state = REG32(base + TOUCH_STATE);
  const int16_t x = (int16_t)REG32(base + TOUCH_X);
  const int16_t y = (int16_t)REG32(base + TOUCH_Y);

  if (state & INT_TOUCH_EVENT) {
    touch_handle_update(TouchState_FingerDown, x, y);
  } else {
    touch_handle_update(TouchState_FingerUp, x, y);
  }
}

void TOUCH_IRQHandler(void) {
  REG32(QEMU_TOUCH_BASE + TOUCH_INTSTAT) = INT_TOUCH_EVENT;

  bool should_context_switch = false;
  if (!s_callback_scheduled) {
    if (system_task_add_callback_from_isr(prv_process_touch_update, NULL,
                                          &should_context_switch)) {
      s_callback_scheduled = true;
    }
  }
}

void touch_sensor_init(void) {
  const uint32_t base = QEMU_TOUCH_BASE;

  REG32(base + TOUCH_INTSTAT) = INT_TOUCH_EVENT;
  REG32(base + TOUCH_INTCTRL) = INT_TOUCH_EVENT;

  NVIC_SetPriority(TOUCH_IRQn, 6);
  NVIC_EnableIRQ(TOUCH_IRQn);
}

void touch_sensor_set_enabled(bool enabled) {
  REG32(QEMU_TOUCH_BASE + TOUCH_INTCTRL) = enabled ? INT_TOUCH_EVENT : 0;
}
