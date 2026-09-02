/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/irq.h"
#include <pbl/drivers/button.h>
#include <pbl/drivers/debounced_button.h>

#include "board/board.h"
#include "console/prompt.h"
#include "kernel/events.h"

#include <cmsis_core.h>
#include <stdlib.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))

// QEMU GPIO register offsets (must match pebble-gpio device)
#define GPIO_BTN_STATE  0x00
#define GPIO_BTN_EDGE   0x04
#define GPIO_INTCTRL    0x08
#define GPIO_INTSTAT    0x0C

// Button bit positions (must match QEMU pebble-gpio device)
#define BTN_BIT_BACK    (1 << 0)
#define BTN_BIT_UP      (1 << 1)
#define BTN_BIT_SELECT  (1 << 2)
#define BTN_BIT_DOWN    (1 << 3)

static const uint32_t s_button_bits[NUM_BUTTONS] = {
  [BUTTON_ID_BACK]   = BTN_BIT_BACK,
  [BUTTON_ID_UP]     = BTN_BIT_UP,
  [BUTTON_ID_SELECT] = BTN_BIT_SELECT,
  [BUTTON_ID_DOWN]   = BTN_BIT_DOWN,
};

static uint32_t s_last_state;

static void prv_gpio_irq_handler(void) {
  uint32_t base = QEMU_GPIO_BASE;

  // Read which buttons changed
  uint32_t edge = REG32(base + GPIO_BTN_EDGE);
  // Clear edge flags
  REG32(base + GPIO_INTSTAT) = edge;

  // Read current state
  uint32_t state = REG32(base + GPIO_BTN_STATE);
  uint32_t changed = s_last_state ^ state;
  s_last_state = state;

  // Generate button events for each changed button
  for (int i = 0; i < NUM_BUTTONS; i++) {
    if (changed & s_button_bits[i]) {
      bool is_pressed = (state & s_button_bits[i]) != 0;
      PebbleEvent e = {
        .type = is_pressed ? PEBBLE_BUTTON_DOWN_EVENT : PEBBLE_BUTTON_UP_EVENT,
        .button.button_id = i,
      };
      event_put_isr(&e);
    }
  }
}

// IRQ trampoline for GPIO IRQ (IRQ 6)
void GPIO_IRQHandler(void) {
  prv_gpio_irq_handler();
}

void button_init(void) {
  uint32_t base = QEMU_GPIO_BASE;

  // Clear any pending edge flags
  REG32(base + GPIO_INTSTAT) = 0xF;

  // Enable edge interrupt
  REG32(base + GPIO_INTCTRL) = 1;

  // Enable GPIO IRQ in NVIC - priority must be >= PBL_IRQ_PRIO_MAX_SYSCALL
  // to safely call FreeRTOS API from ISR. Use priority 6 (lower urgency than max syscall).
  NVIC_SetPriority(GPIO_IRQn, 6);
  NVIC_EnableIRQ(GPIO_IRQn);

  s_last_state = REG32(base + GPIO_BTN_STATE);
}

bool button_is_pressed(ButtonId id) {
  if (id >= NUM_BUTTONS) {
    return false;
  }
  uint32_t state = REG32(QEMU_GPIO_BASE + GPIO_BTN_STATE);
  return (state & s_button_bits[id]) != 0;
}

uint8_t button_get_state_bits(void) {
  return (uint8_t)(REG32(QEMU_GPIO_BASE + GPIO_BTN_STATE) & 0xF);
}

void button_set_rotated(bool rotated) {
  (void)rotated;
}

void command_button_read(const char *button_id_str) {
  int button = atoi(button_id_str);
  if (button < 0 || button >= NUM_BUTTONS) {
    prompt_send_response("Invalid button");
    return;
  }
  if (button_is_pressed(button)) {
    prompt_send_response("down");
  } else {
    prompt_send_response("up");
  }
}

void debounced_button_init(void) {
  // QEMU handles debounce in the GPIO device itself
  button_init();
}

void command_put_raw_button_event(const char *button_index, const char *is_button_down_event) {
  int button = atoi(button_index);
  int is_down = atoi(is_button_down_event);
  if (button < 0 || button >= NUM_BUTTONS) {
    return;
  }
  PebbleEvent e = {
    .type = is_down ? PEBBLE_BUTTON_DOWN_EVENT : PEBBLE_BUTTON_UP_EVENT,
    .button.button_id = button,
  };
  event_put(&e);
}
