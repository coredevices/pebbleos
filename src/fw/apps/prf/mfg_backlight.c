/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/app.h"
#include "applib/graphics/graphics.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/dialogs/confirmation_dialog.h"
#include "applib/ui/window.h"
#include "apps/prf/mfg_test_result.h"
#include <pbl/drivers/backlight.h>
#include "kernel/pbl_malloc.h"
#include "process_management/app_manager.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/light.h"

typedef enum {
  TestPattern_White,
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
  TestPattern_Red,
  TestPattern_Green,
  TestPattern_Blue,
  TestPattern_Black,
#endif
  NumTestPatterns
} TestPattern;

typedef struct {
  Window window;
  TestPattern test_pattern;
} AppData;


static void prv_update_proc(struct Layer *layer, GContext* ctx) {
  graphics_context_set_fill_color(ctx, GColorWhite);
  graphics_fill_rect(ctx, &layer->bounds);

#ifdef CONFIG_BACKLIGHT_HAS_COLOR
  AppData *app_data = app_state_get_user_data();

  PBL_LOG_INFO("backlight id:%d", app_data->test_pattern);
  switch (app_data->test_pattern) {
  case TestPattern_White:
    backlight_set_color(BACKLIGHT_COLOR_WHITE);
    break;
  case TestPattern_Red:
    backlight_set_color(BACKLIGHT_COLOR_RED);
    break;
  case TestPattern_Green:
    backlight_set_color(BACKLIGHT_COLOR_GREEN);
    break;
  case TestPattern_Blue:
    backlight_set_color(BACKLIGHT_COLOR_BLUE);
    break;
  case TestPattern_Black:
    backlight_set_color(BACKLIGHT_COLOR_BLACK);
    break;
  default:
    break;
  }
#endif
}

static void prv_result_confirmed(ClickRecognizerRef recognizer, void *context) {
  ConfirmationDialog *confirmation_dialog = (ConfirmationDialog *)context;
  confirmation_dialog_pop(confirmation_dialog);

  bool passed = (click_recognizer_get_button_id(recognizer) == BUTTON_ID_UP);
  mfg_test_result_report(MfgTestId_Backlight, passed, 0);
  app_window_stack_pop(false);
}

static void prv_result_click_config(void *context) {
  window_single_click_subscribe(BUTTON_ID_UP, prv_result_confirmed);
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_result_confirmed);
  window_single_click_subscribe(BUTTON_ID_BACK, prv_result_confirmed);
}

static void prv_show_result_dialog(void) {
  ConfirmationDialog *confirmation_dialog = confirmation_dialog_create("Backlight Result");
  Dialog *dialog = confirmation_dialog_get_dialog(confirmation_dialog);

  dialog_set_text(dialog, "Backlight OK?");

  confirmation_dialog_set_click_config_provider(confirmation_dialog, prv_result_click_config);

  ActionBarLayer *action_bar = confirmation_dialog_get_action_bar(confirmation_dialog);
  action_bar_layer_set_context(action_bar, confirmation_dialog);

  app_confirmation_dialog_push(confirmation_dialog);
}

static void prv_button_click_handler(ClickRecognizerRef recognizer, void *data) {
  AppData *app_data = app_state_get_user_data();

  app_data->test_pattern++;

  if (app_data->test_pattern >= NumTestPatterns) {
    prv_show_result_dialog();
    return;
  }

  layer_mark_dirty(&app_data->window.layer);
}

static void prv_config_provider(void *data) {
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_button_click_handler);
}

static void prv_handle_init(void) {
  AppData *data = app_malloc_check(sizeof(AppData));
  *data = (AppData) {
    .test_pattern = (TestPattern) app_manager_get_task_context()->args
  };

  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, "");
  window_set_fullscreen(window, true);
  window_set_click_config_provider(window, prv_config_provider);

  Layer *layer = window_get_root_layer(window);
  layer_set_update_proc(layer, prv_update_proc);

  app_window_stack_push(window, true /* Animated */);
}

static void s_main(void) {
  light_enable(true);

  prv_handle_init();

  app_event_loop();

#ifdef CONFIG_BACKLIGHT_HAS_COLOR
  backlight_set_color(BACKLIGHT_COLOR_WARM_WHITE);
#endif

  light_enable(false);
}

const PebbleProcessMd* mfg_backlight_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common.main_func = &s_main,
    // UUID: 2d825a20-2fa3-4b26-be6f-ab41d1280c73
    .common.uuid = { 0x2d, 0x82, 0x5a, 0x20, 0x2f, 0xa3, 0x4b, 0x26,
                     0xbe, 0x6f, 0xab, 0x41, 0xd1, 0x28, 0x0c, 0x73 },
    .name = "MfgBacklight",
  };
  return (const PebbleProcessMd*) &s_app_info;
}
