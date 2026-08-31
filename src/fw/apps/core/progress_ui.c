/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "progress_ui.h"

#include "applib/app.h"
#include "applib/app_timer.h"
#include "applib/graphics/graphics.h"
#include "applib/ui/dialogs/dialog.h"
#include "applib/ui/dialogs/simple_dialog.h"
#include "applib/ui/layer.h"
#include "applib/ui/progress_layer.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window_private.h"
#include "applib/ui/app_window_stack.h"
#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"
#include "process_management/app_manager.h"
#include "process_state/app_state/app_state.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/firmware_update.h"
#include "pbl/services/i18n/i18n.h"
#include <pbl/logging/logging.h>

#include <stdio.h>

#define UPDATE_FREQ_MS 1000
#define FAIL_SCREEN_VISIBLE_DURATION_MS 10000
#define COMPLETE_SCREEN_VISIBLE_DURATION_MS 5000
#define PROG_LAYER_START_VAL 6
// Used to force the progress bar to start at PROG_LAYER_START_VAL and scale
// the reset of the progress between that value and MAX_PROGRESS_PERCENT
#define PROG_LAYER_TRANSFORM(real_prog) \
    (PROG_LAYER_START_VAL + (real_prog * \
     (MAX_PROGRESS_PERCENT - PROG_LAYER_START_VAL) / MAX_PROGRESS_PERCENT))

////////////////////////////////////////////////////////////
// Data structures

typedef struct {
  Window window;
  TextLayer percent_done_text_layer;
  char percent_done_text_buffer[5]; //<! Text for progress percentage label, format %02d%%
  SimpleDialog finished_dialog;
  ProgressLayer progress_layer;
  AppTimer *timer;
  unsigned int percent_complete;
  ProgressUISource progress_source;
  bool is_finished;
} ProgressUIData;

////////////////////////////////////////////////////////////
// Progress Logic

static void prv_quit(void *data) {
  i18n_free_all(data);
  app_window_stack_pop_all(true);
}

static const char *prv_get_dialog_text(ProgressUIData *data, bool success) {
  if (data->progress_source == PROGRESS_UI_SOURCE_FW_UPDATE) {
    return success ? i18n_get("Update Complete", data) : i18n_get("Update Failed", data);
  } else {
    return "";
  }
}

static void prv_handle_finished(ProgressUIData *data, bool success) {
  if (data->is_finished) {
    return;
  }
  data->is_finished = true;
  layer_set_hidden(&data->percent_done_text_layer.layer, true);
  layer_set_hidden(&data->progress_layer.layer, true);

  uint32_t res_id;
  uint32_t end_screen_timeout;
  if (success) {
    res_id = RESOURCE_ID_GENERIC_CONFIRMATION_LARGE;
    end_screen_timeout = COMPLETE_SCREEN_VISIBLE_DURATION_MS;
    dialog_set_background_color(simple_dialog_get_dialog(&data->finished_dialog), GColorGreen);
    simple_dialog_set_buttons_enabled(&data->finished_dialog, false);
  } else {
    res_id = RESOURCE_ID_GENERIC_WARNING_LARGE;
    end_screen_timeout = FAIL_SCREEN_VISIBLE_DURATION_MS;
  }

  dialog_set_icon(&data->finished_dialog.dialog, res_id);
  dialog_set_text(&data->finished_dialog.dialog, prv_get_dialog_text(data, success));
  // Show the status screen for a bit before closing the app
  dialog_set_timeout(&data->finished_dialog.dialog, end_screen_timeout);

  simple_dialog_push(&data->finished_dialog, app_state_get_window_stack());

  app_timer_cancel(data->timer);
}

static void prv_update_progress_text(ProgressUIData *data) {
  sniprintf(data->percent_done_text_buffer,
            sizeof(data->percent_done_text_buffer), "%u%%", data->percent_complete);
  layer_mark_dirty(&data->percent_done_text_layer.layer);
}

static void prv_update_progress(ProgressUIData *data) {
  switch (data->progress_source) {
    case PROGRESS_UI_SOURCE_COREDUMP: {
      break;
    }
    case PROGRESS_UI_SOURCE_LOGS: {
      break;
    }
    case PROGRESS_UI_SOURCE_FACTORY_RESET: {
      return;
    }
    case PROGRESS_UI_SOURCE_FW_UPDATE: {
      if (firmware_update_current_status() == FirmwareUpdateFailed) {
        prv_handle_finished(data, false /* success */);
        return;
      } else if (firmware_update_current_status() == FirmwareUpdateStopped) {
        prv_handle_finished(data, true /* success */);
        return;
      }

      data->percent_complete = firmware_update_get_percent_progress();
      break;
    }
  }

  prv_update_progress_text(data);
  progress_layer_set_progress(&data->progress_layer,
                              PROG_LAYER_TRANSFORM(data->percent_complete));

  if ((data->progress_source != PROGRESS_UI_SOURCE_FW_UPDATE) &&
        (data->percent_complete >= 100)) {
    prv_handle_finished(data, true /* success */);
  }
}

static void prv_refresh_progress(void *data_in) {
  ProgressUIData *data = (ProgressUIData*) data_in;
  if (!data) {
    // Sanity check
    return;
  }

  // Overwrite the old timer handle, it's no longer valid
  data->timer = app_timer_register(UPDATE_FREQ_MS, prv_refresh_progress, data);

  prv_update_progress(data);
}

////////////////////////////////////////////////////////////
// Window loading, unloading, initializing

static void prv_dialog_unloaded(void *context) {
  ProgressUIData *data = context;
  // Schedule a super quick timer to pop all windows. Can't call it here directly
  // since we would actually try popping the dialog window too, causing a fault.
  data->timer = app_timer_register(10, prv_quit, data);
}

static void prv_window_unload_handler(Window* window) {
  ProgressUIData *data = window_get_user_data(window);
  if (data) {
    i18n_free_all(data);
    if (data->timer) {
      app_timer_cancel(data->timer);
    }
    app_free(data);
  }
}

static void prv_push_factory_reset_dialog(ProgressUIData *data) {
  simple_dialog_init(&data->finished_dialog, "Factory Reset");
  Dialog *dialog = simple_dialog_get_dialog(&data->finished_dialog);
  dialog_set_text(dialog, i18n_get("Resetting...", data));
  dialog_set_icon(dialog, RESOURCE_ID_GENERIC_WARNING_LARGE);
  dialog_set_timeout(dialog, DIALOG_TIMEOUT_INFINITE);
  dialog_set_destroy_on_pop(dialog, false);
  simple_dialog_set_buttons_enabled(&data->finished_dialog, false);
  simple_dialog_set_icon_animated(&data->finished_dialog, false);

  simple_dialog_push(&data->finished_dialog, app_state_get_window_stack());
}

static void prv_window_load_handler(Window* window) {
  ProgressUIData *data = window_get_user_data(window);

  const ProgressUIAppArgs* app_args = app_manager_get_task_context()->args;
  data->progress_source = app_args->progress_source;

  if (data->progress_source == PROGRESS_UI_SOURCE_FACTORY_RESET) {
    prv_push_factory_reset_dialog(data);
    return;
  }

  simple_dialog_init(&data->finished_dialog, "Update Completed Dialog");
  dialog_set_callbacks(&data->finished_dialog.dialog, &(DialogCallbacks) {
    .unload = prv_dialog_unloaded,
  }, data);
  dialog_set_destroy_on_pop(&data->finished_dialog.dialog, false);

  const int16_t load_bar_length = 108;
  const int16_t load_bar_height = 8;
  const int16_t x_offset = (window->layer.bounds.size.w - load_bar_length) / 2;
  const int16_t y_offset_progress = (window->layer.bounds.size.h - load_bar_height) / 2;
  const int16_t y_offset_text = y_offset_progress - 38;
  const GRect progress_bounds = GRect(x_offset, y_offset_progress, load_bar_length, load_bar_height);
  ProgressLayer *progress_layer = &data->progress_layer;
  progress_layer_init(progress_layer, &progress_bounds);
  progress_layer_set_corner_radius(progress_layer, 3);
  layer_add_child(&window->layer, &progress_layer->layer);

  TextLayer *percent_done_text_layer = &data->percent_done_text_layer;
  text_layer_init_with_parameters(percent_done_text_layer,
                                  &GRect(0, y_offset_text, window->layer.bounds.size.w, 30),
                                  data->percent_done_text_buffer,
                                  fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD),
                                  GColorBlack, GColorClear, GTextAlignmentCenter,
                                  GTextOverflowModeTrailingEllipsis);
  layer_add_child(&window->layer, &percent_done_text_layer->layer);

  data->timer = app_timer_register(UPDATE_FREQ_MS, prv_refresh_progress, data);
  prv_update_progress(data);
}

static void prv_progress_ui_window_push(void) {
  ProgressUIData *data = app_zalloc_check(sizeof(ProgressUIData));

  Window* window = &data->window;
  window_init(window, WINDOW_NAME("Progress UI App"));
  window_set_user_data(window, data);
  window_set_overrides_back_button(window, true);
  window_set_background_color(window, PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite));
  window_set_window_handlers(window, &(WindowHandlers){
    .load = prv_window_load_handler,
    .unload = prv_window_unload_handler,
  });
  app_window_stack_push(window, false);
}

static void prv_main(void) {
  if (!app_manager_get_task_context()->args) {
    PBL_LOG_WRN("Progress UI App must be launched with args");
    return;
  }

  launcher_block_popups(true);

  prv_progress_ui_window_push();

  app_event_loop();

  launcher_block_popups(false);
}

////////////////////////////////////////////////////////////
// Public functions

const PebbleProcessMd* progress_ui_app_get_info() {
  static const PebbleProcessMdSystem s_app_info = {
    .common = {
      .main_func = &prv_main,
      .visibility = ProcessVisibilityHidden,
      // UUID: f29f18ac-bbec-452b-9262-49c4f6e5c920
      .uuid = {0xf2, 0x9f, 0x18, 0xac, 0xbb, 0xec, 0x45, 0x2b,
               0x92, 0x62, 0x49, 0xc4, 0xf6, 0xe5, 0xc9, 0x20},
    },
    .name = "Progress UI",
    .run_level = ProcessAppRunLevelSystem,
  };
  return (const PebbleProcessMd*) &s_app_info;
}
