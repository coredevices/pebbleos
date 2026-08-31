/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/pwm.h>
#include "system/passert.h"

#include <nrfx.h>

void pwm_init(const PwmConfig *pwm, uint32_t resolution, uint32_t frequency) {
  nrfx_pwm_config_t config = NRFX_PWM_DEFAULT_CONFIG(
    pwm->output.gpio_pin,
    NRF_PWM_PIN_NOT_CONNECTED,
    NRF_PWM_PIN_NOT_CONNECTED,
    NRF_PWM_PIN_NOT_CONNECTED);
  // this is hokey and imprecise, but oh well
  if      (frequency >= 16000000) config.base_clock = NRF_PWM_CLK_16MHz;
  else if (frequency >=  8000000) config.base_clock = NRF_PWM_CLK_8MHz;
  else if (frequency >=  4000000) config.base_clock = NRF_PWM_CLK_4MHz;
  else if (frequency >=  2000000) config.base_clock = NRF_PWM_CLK_2MHz;
  else if (frequency >=  1000000) config.base_clock = NRF_PWM_CLK_1MHz;
  else if (frequency >=   500000) config.base_clock = NRF_PWM_CLK_500kHz;
  else if (frequency >=   250000) config.base_clock = NRF_PWM_CLK_250kHz;
  else if (frequency >=   125000) config.base_clock = NRF_PWM_CLK_125kHz;
  else WTF;
  config.count_mode = NRF_PWM_MODE_UP;
  config.top_value = resolution;
  config.load_mode = NRF_PWM_LOAD_COMMON;
  config.step_mode = NRF_PWM_STEP_TRIGGERED;
    
  nrfx_err_t rv = nrfx_pwm_init(&pwm->peripheral, &config, NULL, NULL);
  PBL_ASSERTN(rv == NRFX_SUCCESS);
  
  pwm->state->enabled = 0;
  pwm->state->value = 0;
  pwm->state->seq = (nrf_pwm_sequence_t) { .values = { .p_common = &pwm->state->value }, .length = 1, .repeats = 0, .end_delay = 0 };
  pwm->state->resolution = resolution;
}

void pwm_set_duty_cycle(const PwmConfig *pwm, uint32_t duty_cycle) {
  PBL_ASSERTN(duty_cycle < 0xFFFF);
  pwm->state->value = pwm->state->resolution - duty_cycle;
  if (pwm->state->enabled)
    nrfx_pwm_simple_playback(&pwm->peripheral, &pwm->state->seq, 1, 0);
  if (duty_cycle == 0)
    nrfx_pwm_stop(&pwm->peripheral, 0);
}

void pwm_enable(const PwmConfig *pwm, bool enable) {
  if (enable) {
    nrfx_pwm_simple_playback(&pwm->peripheral, &pwm->state->seq, 1, 0);
  } else {
    // no need to set the output to low; I think the nRF does it for us?
    nrfx_pwm_stop(&pwm->peripheral, 0);
  }
  pwm->state->enabled = !!enable;
}
