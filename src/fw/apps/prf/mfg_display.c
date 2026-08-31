/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/app.h"
#include "applib/graphics/graphics.h"
#include "applib/graphics/text.h"
#include "applib/tick_timer_service.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/dialogs/confirmation_dialog.h"
#include "applib/ui/window.h"
#include "apps/prf/mfg_test_result.h"
#include "console/prompt.h"
#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"
#include "mfg/mfg_mode/mfg_factory_mode.h"
#include "process_management/app_manager.h"
#include "process_management/pebble_process_md.h"
#include "process_state/app_state/app_state.h"
#include "pbl/util/size.h"

typedef enum {
  TestPattern_Crosshair,
  TestPattern_Black,
  TestPattern_White,
#if PBL_COLOR
  TestPattern_Red,
  TestPattern_Green,
  TestPattern_Blue,
#else
  TestPattern_Gray,
#endif

  NumTestPatterns
} TestPattern;

typedef struct {
  Window window;

  TestPattern test_pattern;
} AppData;

static void prv_draw_solid(Layer *layer, GContext *ctx, GColor color) {
  graphics_context_set_fill_color(ctx, color);
  graphics_fill_rect(ctx, &layer->bounds);
}

#if PBL_ROUND

static void prv_draw_round_border(Layer *layer, GContext *ctx, uint8_t radial_padding_size) {
  for (int i = 0; i < layer->bounds.size.h / 2 - radial_padding_size; ++i) {
    const GBitmapDataRowInfoInternal *data_row_infos = g_gbitmap_data_row_infos;
    const uint8_t mask = data_row_infos[i].min_x + radial_padding_size;
    const int offset = i + radial_padding_size;
    // Draw both row-wise and column-wise to fill in any discontinuities
    // in the border circle.

    // Top-left quadrant
    graphics_draw_pixel(ctx, GPoint(mask, offset));
    graphics_draw_pixel(ctx, GPoint(offset, mask));
    // Top-right quadrant
    graphics_draw_pixel(ctx, GPoint(mask, layer->bounds.size.h - offset - 1));
    graphics_draw_pixel(ctx, GPoint(layer->bounds.size.w - offset - 1, mask));
    // Bottom-left quadrant
    graphics_draw_pixel(ctx, GPoint(layer->bounds.size.w - mask - 1, offset));
    graphics_draw_pixel(ctx, GPoint(offset, layer->bounds.size.h - mask - 1));
    // Bottom-right quadrant
    graphics_draw_pixel(ctx, GPoint(layer->bounds.size.w - mask - 1,
                                    layer->bounds.size.h - offset - 1));
    graphics_draw_pixel(ctx, GPoint(layer->bounds.size.w - offset - 1,
                                    layer->bounds.size.h - mask - 1));
  }
}

static void prv_draw_border(Layer *layer, GContext *ctx, uint8_t radial_padding_size) {
  if (radial_padding_size != 0) {
    prv_draw_round_border(layer, ctx, 0);
  }
  prv_draw_round_border(layer, ctx, radial_padding_size);

  // Draw letter to identify screen
  if (radial_padding_size >= 2) {
    GRect identifier_area = GRect(40, 40, 20, 20);
    char identifier[] = {'A' + radial_padding_size - 2, 0};
    graphics_context_set_text_color(ctx, GColorWhite);
    graphics_draw_text(ctx, identifier, fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD),
                       identifier_area, GTextOverflowModeWordWrap, GTextAlignmentCenter, NULL);
  }
}

#else

static void prv_draw_border(Layer *layer, GContext *ctx, uint8_t radial_padding_size) {
  graphics_draw_rect(ctx, &layer->bounds);
}

#endif

static void prv_draw_crosshair_screen(Layer *layer, GContext *ctx, uint8_t radial_padding_size) {
  graphics_context_set_fill_color(ctx, GColorBlack);
  graphics_fill_rect(ctx, &layer->bounds);

  // Draw crosshair
  graphics_context_set_stroke_color(ctx, GColorWhite);
  graphics_draw_line(
      ctx, GPoint(layer->bounds.size.w / 2, radial_padding_size),
      GPoint(layer->bounds.size.w / 2, layer->bounds.size.h - radial_padding_size - 1));
  graphics_draw_line(
      ctx, GPoint(radial_padding_size, layer->bounds.size.h / 2),
      GPoint(layer->bounds.size.w - radial_padding_size - 1, layer->bounds.size.h / 2));

  prv_draw_border(layer, ctx, radial_padding_size);
}

static void prv_update_proc(struct Layer *layer, GContext* ctx) {
  AppData *app_data = app_state_get_user_data();

  switch (app_data->test_pattern) {
  case TestPattern_Crosshair:
    prv_draw_crosshair_screen(layer, ctx, 0);
    break;
  case TestPattern_Black:
    prv_draw_solid(layer, ctx, GColorBlack);
    break;
  case TestPattern_White:
    prv_draw_solid(layer, ctx, GColorWhite);
    break;
#if PBL_COLOR
  case TestPattern_Red:
    prv_draw_solid(layer, ctx, GColorRed);
    break;
  case TestPattern_Green:
    prv_draw_solid(layer, ctx, GColorGreen);
    break;
  case TestPattern_Blue:
    prv_draw_solid(layer, ctx, GColorBlue);
    break;
#else
  case TestPattern_Gray:
    prv_draw_solid(layer, ctx, GColorDarkGray);
    break;
#endif
  default:
    break;
  }
}

static void prv_result_confirmed(ClickRecognizerRef recognizer, void *context) {
  ConfirmationDialog *confirmation_dialog = (ConfirmationDialog *)context;
  confirmation_dialog_pop(confirmation_dialog);

  bool passed = (click_recognizer_get_button_id(recognizer) == BUTTON_ID_UP);
  mfg_test_result_report(MfgTestId_Display, passed, 0);
  app_window_stack_pop(false);
}

static void prv_result_click_config(void *context) {
  window_single_click_subscribe(BUTTON_ID_UP, prv_result_confirmed);
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_result_confirmed);
  window_single_click_subscribe(BUTTON_ID_BACK, prv_result_confirmed);
}

static void prv_show_result_dialog(void) {
  ConfirmationDialog *confirmation_dialog = confirmation_dialog_create("Display Result");
  Dialog *dialog = confirmation_dialog_get_dialog(confirmation_dialog);

  dialog_set_text(dialog, "Display OK?");

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

static void prv_change_pattern(void *data) {
  AppData *app_data = app_state_get_user_data();

  app_data->test_pattern = (TestPattern) data;

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
  prv_handle_init();

  app_event_loop();
}

const PebbleProcessMd* mfg_display_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common.main_func = &s_main,
    // UUID: df582042-5beb-410f-9fed-76eccd31821e
    .common.uuid = { 0xdf, 0x58, 0x20, 0x42, 0x5b, 0xeb, 0x41, 0x0f,
                     0x9f, 0xed, 0x76, 0xec, 0xcd, 0x31, 0x82, 0x1e },
    .name = "MfgDisplay",
  };
  return (const PebbleProcessMd*) &s_app_info;
}


// Prompt Commands
///////////////////////////////////////////////////////////////////////////////

static void prv_launch_app_cb(void *data) {
  if (app_manager_get_current_app_md() == mfg_display_app_get_info()) {
    process_manager_send_callback_event_to_process(PebbleTask_App, prv_change_pattern, data);
  } else {
    app_manager_launch_new_app(&(AppLaunchConfig) {
      .md = mfg_display_app_get_info(),
      .common.args = data,
    });
  }
}

void command_display_set(const char *color) {
  const char * const ARGS[] = {
    [TestPattern_Crosshair] = "crosshair",
    [TestPattern_Black] = "black",
    [TestPattern_White] = "white",
#if PBL_COLOR
    [TestPattern_Red] = "red",
    [TestPattern_Green] = "green",
    [TestPattern_Blue] = "blue",
#else
    [TestPattern_Gray] = "gray",
#endif
  };

  for (unsigned int i = 0; i < ARRAY_LENGTH(ARGS); ++i) {
    if (!strcmp(color, ARGS[i])) {
      // Do this first because it launches the mfg menu using a callback, as if we did this in the
      // callback we send below to launch the display app it would end up launching the menu on
      // top of the display app.
      if (!mfg_is_mfg_mode()) {
        mfg_enter_mfg_mode_and_launch_app();
      }

      launcher_task_add_callback(prv_launch_app_cb, (void*) i);
      prompt_send_response("OK");
      return;
    }
  }

  prompt_send_response("Invalid command");
}
