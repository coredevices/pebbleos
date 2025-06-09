/*
 * Copyright 2025 Core Devices LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mfg_speaker_data.h"

#include "applib/app.h"
#include "applib/tick_timer_service.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window.h"
#include "board/board.h"
#include "drivers/i2c.h"
#include "drivers/i2c_definitions.h"
#include "drivers/i2c_hal.h"
#include "kernel/pbl_malloc.h"
#include "kernel/util/sleep.h"
#include "process_management/pebble_process_md.h"
#include "process_state/app_state/app_state.h"
#include "util/size.h"

#include "hal/nrf_clock.h"
#include "nrfx_i2s.h"

#define TIMEOUT 1000
#define SAMPLE_BIT_WIDTH 16
#define INITIAL_BLOCKS 2
#define BLOCK_COUNT (INITIAL_BLOCKS + 2)

static const nrfx_i2s_t prv_i2s = NRFX_I2S_INSTANCE(0);
static nrfx_i2s_config_t prv_i2s_cfg =
    NRFX_I2S_DEFAULT_CONFIG(NRF_GPIO_PIN_MAP(0, 12), NRF_GPIO_PIN_MAP(0, 7), NRF_GPIO_PIN_MAP(1, 9),
                            NRF_GPIO_PIN_MAP(0, 13), NRF_I2S_PIN_NOT_CONNECTED);
static const nrfx_i2s_buffers_t prv_i2s_bufs = {
  .p_rx_buffer = NULL,
  .p_tx_buffer = (uint32_t *)audio_data,
  .buffer_size = BLOCK_SIZE / sizeof(uint32_t),
};

typedef struct {
  Window window;

  TextLayer title;
  TextLayer status;
} AppData;

static bool prv_codec_setup(void) {
  int ret;
  uint8_t data[2];

  uint8_t init[][2] = {
      // word freq to 44.1khz
      {0x22, 0x0a},
      // codec in slave mode, 32 BCLK per WCLK
      {0x28, 0x00},
      // enable DAC_L
      {0x69, 0x88},
      // setup LINE_AMP_GAIN to 15db
      {0x4a, 0x3f},
      // enable LINE amplifier
      {0x6d, 0x80},
      // enable DAC_R
      {0x6a, 0x80},
      // setup MIXIN_R_GAIN to 0dB
      {0x35, 0x03},
      // enable MIXIN_R
      {0x66, 0x80},
      // setup DIG_ROUTING_DAI to DAI
      {0x21, 0x32},
      // setup DIG_ROUTING_DAC to mono
      {0x2a, 0xba},
      // setup DAC_L_GAIN to 0dB
      {0x45, 0x6f},
      // setup DAC_R_GAIN to 0dB
      {0x46, 0x6f},
      // enable DAI, 16bit per channel
      {0x29, 0x80},
      // setup SYSTEM_MODES_OUTPUT to use DAC_R,DAC_L and LINE
      {0X51, 0x00},
      // setup Master bias enable
      {0X23, 0x08},
      // Sets the input clock range for the PLL 40-80MHz
      {0X27, 0x00},
      // setup MIXOUT_R_SELECT to DAC_R selected
      {0X4C, 0x08},
      // setup MIXOUT_R_CTRL to MIXOUT_R mixer amp enable and MIXOUT R mixer enable
      {0X6F, 0x98},
  };

  i2c_use(I2C_DA7212);

  // CIF_CTRL: soft reset
  data[0] = 0x1d;
  data[1] = 0x80;
  ret = i2c_write_block(I2C_DA7212, 2, data);
  if (!ret) {
    goto end;
  }

  psleep(10);

  // SYSTEM_ACTIVE: wake-up
  data[0] = 0xfd;
  data[1] = 0x01;
  ret = i2c_write_block(I2C_DA7212, 2, data);
  if (!ret) {
    goto end;
  }

  for (size_t i = 0U; i < ARRAY_LENGTH(init); ++i) {
    const uint8_t *entry = init[i];

    ret = i2c_write_block(I2C_DA7212, 2, entry);
    if (!ret) {
      goto end;
    }
  }

end:
  i2c_release(I2C_DA7212);

  return ret;
}

static bool prv_codec_standby(void) {
  bool ret;
  uint8_t data[2];

  i2c_use(I2C_DA7212);
  data[0] = 0xfd;
  data[1] = 0x00;
  ret = i2c_write_block(I2C_DA7212, 2, data);
  i2c_release(I2C_DA7212);

  return ret;
}

static void prv_data_handler(nrfx_i2s_buffers_t const *p_released, uint32_t status) {
  if (status == NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) {
    nrfx_i2s_next_buffers_set(&prv_i2s, &prv_i2s_bufs);
    return;
  }
}

static bool prv_speaker_play(void) {
  nrfx_err_t err;
  bool ret;

  nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);
  nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
  while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED)) {
  }
  nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);

  prv_i2s_cfg.channels = NRF_I2S_CHANNELS_STEREO;
  prv_i2s_cfg.mck_setup = NRF_I2S_MCK_32MDIV23;

  err = nrfx_i2s_init(&prv_i2s, &prv_i2s_cfg, prv_data_handler);
  if (err != NRFX_SUCCESS) {
    return false;
  }

  err = nrfx_i2s_start(&prv_i2s, &prv_i2s_bufs, 0);
  if (err != NRFX_SUCCESS) {
    return false;
  }

  ret = prv_codec_setup();
  if (!ret) {
    return ret;
  }

  return true;
}

static bool prv_speaker_stop(void) {
  bool ret;

  ret = prv_codec_standby();
  if (!ret) {
    return ret;
  }

  nrfx_i2s_stop(&prv_i2s);
  nrfx_i2s_uninit(&prv_i2s);

  return 0;
}

static void prv_timer_callback(void *cb_data) {
  (void)prv_speaker_stop();
  app_window_stack_pop(true /* animated */);
}

static void prv_handle_init(void) {
  AppData *data = app_malloc_check(sizeof(AppData));

  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, "");
  window_set_fullscreen(window, true);
  window_set_overrides_back_button(window, true);

  TextLayer *title = &data->title;
  text_layer_init(title, &window->layer.bounds);
  text_layer_set_font(title, fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
  text_layer_set_text_alignment(title, GTextAlignmentCenter);
  text_layer_set_text(title, "SPEAKER TEST");
  layer_add_child(&window->layer, &title->layer);

  app_window_stack_push(window, true /* Animated */);

  (void)prv_speaker_play();
  app_timer_register(5000, prv_timer_callback, NULL);
}

static void s_main(void) {
  prv_handle_init();

  app_event_loop();
}

const PebbleProcessMd *mfg_speaker_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
      .common.main_func = &s_main,
      // UUID: 27047635-68f1-4ece-9ca7-52dd8e22d1dd
      .common.uuid = {0x27, 0x04, 0x76, 0x35, 0x68, 0xf1, 0x4e, 0xce, 0x9c, 0xa7, 0x52, 0xdd, 0x8e,
                      0x22, 0xd1, 0xdd},
      .name = "MfgSpeaker",
  };
  return (const PebbleProcessMd *)&s_app_info;
}
