/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "board/board.h"
#include <pbl/drivers/gpio.h>
#include <pbl/drivers/backlight.h>
#include "kernel/util/delay.h"
#include <pbl/logging/logging.h>
#include "pbl/util/math.h"

#include "FreeRTOS.h"

// AW9364E 1-wire dimming protocol implementation
// The AW9364E uses pulse counting for brightness control:
// - 1 pulse = 20mA (brightest)
// - 16 pulses = 1.25mA (dimmest)
// Timing: THI > 0.5us, 0.5us < TLO < 500us
// Shutdown: EN low for > 2.5ms
// The protocol is asymmetric: extra pulses step the current further down
// without a restart, but raising the current requires a full shutdown
// followed by a fresh pulse train.

#define AW9364E_TON_US 20U
#define AW9364E_THI_US 1U
#define AW9364E_TLO_US 1U
#define AW9364E_MAX_PULSES 16U
#define AW9364E_OFF_TIME_US 2600U

//! Pulse count currently latched by the chip; 0 = shut down.
static uint8_t s_pulse_count;

void backlight_init(void) {
  gpio_output_init(&AW9364E.gpio, GPIO_OType_PP);
}

//! Emit dimming pulses. A low period stretched past 500us puts the chip in
//! undefined territory (past 2.5ms it shuts down), so the train runs with
//! the scheduler and most interrupts masked; worst case is ~55us.
static void prv_send_pulses(uint8_t count, bool from_shutdown) {
  portENTER_CRITICAL();
  for (uint8_t i = 0U; i < count; i++) {
    gpio_output_set(&AW9364E.gpio, false);
    delay_us(AW9364E_TLO_US);
    gpio_output_set(&AW9364E.gpio, true);
    delay_us((from_shutdown && i == 0U) ? AW9364E_TON_US : AW9364E_THI_US);
  }
  portEXIT_CRITICAL();
}

uint8_t backlight_get_level(uint8_t brightness) {
  if (brightness > 100U) {
    brightness = 100U;
  }

  if (brightness == 0U) {
    return 0U;
  }

  return AW9364E_MAX_PULSES - DIVIDE_CEIL(brightness * AW9364E_MAX_PULSES, 100U) + 1U;
}

void backlight_set_brightness(uint8_t brightness) {
  // 0 = off, otherwise the pulse count to train (1 brightest, 16 dimmest)
  const uint8_t pulse_count = backlight_get_level(brightness);

  if (pulse_count == 0U) {
    // The chip shuts itself down after EN has been low for 2.5ms; no need to
    // block here, as the turn-on path below always waits that out itself.
    gpio_output_set(&AW9364E.gpio, false);
    s_pulse_count = 0U;
    return;
  }

  if (pulse_count == s_pulse_count) {
    return;
  }

  if (s_pulse_count != 0U && pulse_count > s_pulse_count) {
    // Dimming: step down from the latched level, no blackout.
    prv_send_pulses(pulse_count - s_pulse_count, false);
  } else {
    // Turning on or brightening: full shutdown, then re-train from scratch.
    gpio_output_set(&AW9364E.gpio, false);
    delay_us(AW9364E_OFF_TIME_US);
    prv_send_pulses(pulse_count, true);
  }
  s_pulse_count = pulse_count;
}

void backlight_refresh(void) {
  // Intentionally a no-op: re-applying state on this chip requires a 2.5ms
  // shutdown, which would visibly blink the backlight on every wake with
  // unchanged brightness.
}
