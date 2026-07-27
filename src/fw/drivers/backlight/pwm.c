/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/backlight.h>
#include <pbl/drivers/gpio.h>

#include "board/board.h"
#include <pbl/drivers/pwm.h>

//! The counter reload value. The timer will count from 0 to this value and then reset again.
static const uint32_t TIMER_PERIOD_RESOLUTION = 1024;

//! The number of periods we have per second.
static const uint32_t PWM_OUTPUT_FREQUENCY_HZ = 256;

void backlight_init(void) {
  if (BACKLIGHT_PWM.ctl.gpio != NULL) {
    gpio_output_init(&BACKLIGHT_PWM.ctl, GPIO_OType_PP);
    gpio_output_set(&BACKLIGHT_PWM.ctl, false);
  }

  pwm_init(&BACKLIGHT_PWM.pwm, TIMER_PERIOD_RESOLUTION,
           TIMER_PERIOD_RESOLUTION * PWM_OUTPUT_FREQUENCY_HZ);
}

void backlight_set_brightness(uint8_t brightness) {
  if (brightness == 0) {
    pwm_enable(&BACKLIGHT_PWM.pwm, false);
    if (BACKLIGHT_PWM.ctl.gpio != NULL) {
      gpio_output_set(&BACKLIGHT_PWM.ctl, false);
    }
  } else {
    if (BACKLIGHT_PWM.ctl.gpio != NULL) {
      gpio_output_set(&BACKLIGHT_PWM.ctl, true);
    }

    pwm_enable(&BACKLIGHT_PWM.pwm, true);

    const uint32_t desired_duty_cycle = brightness * BACKLIGHT_PWM.max_duty_cycle_percent *
                                        TIMER_PERIOD_RESOLUTION / 10000;
    pwm_set_duty_cycle(&BACKLIGHT_PWM.pwm, desired_duty_cycle);
  }
}

uint8_t backlight_get_level(uint8_t brightness) {
  // Continuous PWM control: every brightness value is distinct.
  return brightness;
}

void backlight_refresh(void) {
}