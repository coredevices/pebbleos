/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/app.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/dialogs/confirmation_dialog.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window.h"
#include "apps/prf/mfg_test_result.h"
#include "kernel/pbl_malloc.h"
#include "board/board.h"
#include <pbl/drivers/pmic/npm1300.h>
#include "process_management/pebble_process_md.h"
#include "process_state/app_state/app_state.h"
#include <pbl/drivers/audio.h>

typedef struct {
  Window window;

  TextLayer title;
  TextLayer status;
} AppData;

static const int16_t sine_wave_4k[] = {
  0, 32767, 0, -32768, 0, 32767, 0, -32768,
  0, 32767, 0, -32768, 0, 32767, 0, -32768,
};

static void prv_audio_trans_handler(uint32_t *free_size) {
    uint32_t available_size = *free_size;
    while (available_size > sizeof(sine_wave_4k)) {
      available_size = audio_write(AUDIO, (void*)&sine_wave_4k[0], sizeof(sine_wave_4k));
    }
}

static void prv_play_audio(void) {
  audio_start(AUDIO, prv_audio_trans_handler);
  audio_set_volume(AUDIO, 30);
}

static void prv_result_confirmed(ClickRecognizerRef recognizer, void *context) {
  ConfirmationDialog *confirmation_dialog = (ConfirmationDialog *)context;
  confirmation_dialog_pop(confirmation_dialog);

  bool passed = (click_recognizer_get_button_id(recognizer) == BUTTON_ID_UP);
  mfg_test_result_report(MfgTestId_Speaker, passed, 0);
  app_window_stack_pop(false);
}

static void prv_result_click_config(void *context) {
  window_single_click_subscribe(BUTTON_ID_UP, prv_result_confirmed);
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_result_confirmed);
  window_single_click_subscribe(BUTTON_ID_BACK, prv_result_confirmed);
}

static void prv_show_result_dialog(void) {
  ConfirmationDialog *confirmation_dialog = confirmation_dialog_create("Speaker Result");
  Dialog *dialog = confirmation_dialog_get_dialog(confirmation_dialog);

  dialog_set_text(dialog, "Speaker OK?");

  confirmation_dialog_set_click_config_provider(confirmation_dialog, prv_result_click_config);

  ActionBarLayer *action_bar = confirmation_dialog_get_action_bar(confirmation_dialog);
  action_bar_layer_set_context(action_bar, confirmation_dialog);

  app_confirmation_dialog_push(confirmation_dialog);
}

static void prv_timer_callback(void *cb_data) {
  audio_stop(AUDIO);
  prv_show_result_dialog();
}

static void prv_handle_init(void) {
  AppData *data = app_malloc_check(sizeof(AppData));

  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, "");
  window_set_fullscreen(window, true);

  TextLayer *title = &data->title;
  text_layer_init(title, &window->layer.bounds);
  text_layer_set_font(title, fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
  text_layer_set_text_alignment(title, GTextAlignmentCenter);
  text_layer_set_text(title, "SPEAKER TEST");
  layer_add_child(&window->layer, &title->layer);

  app_window_stack_push(window, true /* Animated */);
}

static void s_main(void) {
  // HACK(OBELIX): we need proper regulator API (with consumer current, etc.)
  (void)NPM1300_OPS.dischg_limit_ma_set(NPM1300_DISCHG_LIMIT_MA_MAX);
  prv_handle_init();
  prv_play_audio();
  app_timer_register(5000, prv_timer_callback, NULL);
  app_event_loop();
  audio_stop(AUDIO);
  // HACK(OBELIX): we need proper regulator API (with consumer current, etc.)
  (void)NPM1300_OPS.dischg_limit_ma_set(NPM1300_CONFIG.dischg_limit_ma);
}

const PebbleProcessMd *mfg_speaker_obelix_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
      .common.main_func = &s_main,
      // UUID: c1479d03-5550-4444-b1e7-e2cbad0e5678
      .common.uuid = {0xc1, 0x47, 0x9d, 0x03, 0x55, 0x50, 0x44, 0x44, 0xb1, 0xe7, 0xe2, 0xcb, 0xad,
                      0x0e, 0x56, 0x78},
      .name = "MfgSpeakerObelix",
  };
  return (const PebbleProcessMd *)&s_app_info;
}
