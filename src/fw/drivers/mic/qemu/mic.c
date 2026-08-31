/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/mic.h>
#include <pbl/drivers/mic/qemu/mic_definitions.h>

#include "board/board.h"
#include "console/prompt.h"
#include "pbl/services/new_timer/new_timer.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

#include <string.h>

PBL_LOG_MODULE_DEFINE(driver_mic_qemu, CONFIG_DRIVER_MIC_LOG_LEVEL);

static void prv_timer_cb(void *data) {
  MicDevice *this = data;
  MicDeviceState *state = this->state;

  if (!state->is_running || !state->data_handler || !state->audio_buffer) {
    return;
  }

  // QEMU has no real microphone — feed the consumer silence at the expected
  // cadence so the voice service stays in lockstep with `pebble transcribe`.
  memset(state->audio_buffer, 0, state->audio_buffer_len * sizeof(int16_t));
  state->data_handler(state->audio_buffer, state->audio_buffer_len, state->handler_context);
}

void mic_init(MicDevice *this) {
  PBL_ASSERTN(this);
  PBL_ASSERTN(this->state);

  MicDeviceState *state = this->state;
  if (state->is_initialized) {
    return;
  }

  state->timer = new_timer_create();
  PBL_ASSERTN(state->timer != TIMER_INVALID_ID);
  state->is_initialized = true;
}

void mic_set_volume(MicDevice *this, uint16_t volume) {
  // No gain stage to tweak on the QEMU stub.
}

bool mic_start(MicDevice *this, MicDataHandlerCB data_handler, void *context,
               int16_t *audio_buffer, size_t audio_buffer_len) {
  PBL_ASSERTN(this);
  PBL_ASSERTN(this->state);
  PBL_ASSERTN(data_handler);
  PBL_ASSERTN(audio_buffer);
  PBL_ASSERTN(audio_buffer_len > 0);

  MicDeviceState *state = this->state;
  PBL_ASSERTN(state->is_initialized);

  if (state->is_running) {
    return false;
  }

  state->data_handler = data_handler;
  state->handler_context = context;
  state->audio_buffer = audio_buffer;
  state->audio_buffer_len = audio_buffer_len;

  // Match the wall-clock cadence of a real mic: one buffer's worth of samples
  // per period at MIC_SAMPLE_RATE. Round up to keep the period >= 1 ms.
  uint32_t period_ms = (audio_buffer_len * 1000U + MIC_SAMPLE_RATE - 1U) / MIC_SAMPLE_RATE;
  if (period_ms == 0) {
    period_ms = 1;
  }
  state->period_ms = period_ms;
  state->is_running = true;

  if (!new_timer_start(state->timer, period_ms, prv_timer_cb, (void *)this,
                       TIMER_START_FLAG_REPEATING)) {
    state->is_running = false;
    return false;
  }

  return true;
}

void mic_stop(MicDevice *this) {
  PBL_ASSERTN(this);
  PBL_ASSERTN(this->state);

  MicDeviceState *state = this->state;
  if (!state->is_running) {
    return;
  }

  state->is_running = false;
  new_timer_stop(state->timer);
  state->data_handler = NULL;
  state->handler_context = NULL;
  state->audio_buffer = NULL;
  state->audio_buffer_len = 0;
}

bool mic_is_running(MicDevice *this) {
  PBL_ASSERTN(this);
  PBL_ASSERTN(this->state);
  return this->state->is_running;
}

uint32_t mic_get_channels(MicDevice *this) {
  PBL_ASSERTN(this);
  return this->channels ? this->channels : 1;
}

void command_mic_start(char *timeout_str, char *sample_size_str, char *sample_rate_str,
                      char *format_str) {
  prompt_send_response("Microphone console commands not supported on QEMU");
}

void command_mic_read(void) {
  prompt_send_response("Microphone read command not supported on QEMU");
}
