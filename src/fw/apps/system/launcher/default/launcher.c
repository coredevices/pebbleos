/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "launcher.h"

#include "menu_layer.h"

#include "applib/app.h"
#include "applib/app_focus_service.h"
#include "applib/ui/app_window_stack.h"
#include "kernel/pbl_malloc.h"
#include "shell/normal/app_idle_timeout.h"
#include "shell/prefs.h"
#include "process_state/app_state/app_state.h"
#include "pbl/util/attributes.h"

typedef struct LauncherAppWindowData {
  Window window;
  LauncherMenuLayer launcher_menu_layer;
  AppMenuDataSource app_menu_data_source;
} LauncherAppWindowData;

typedef struct LauncherAppPersistedData {
  bool valid;
  RtcTicks leave_time;
  LauncherMenuLayerSelectionState selection_state;
  LauncherDrawState draw_state;
} LauncherAppPersistedData;

static LauncherAppPersistedData s_launcher_app_persisted_data;

/////////////////////////////
// AppFocusService handlers

static void prv_did_focus(bool in_focus) {
  LauncherAppWindowData *data = app_state_get_user_data();
  if (in_focus) {
    launcher_menu_layer_set_selection_animations_enabled(&data->launcher_menu_layer, true);
  }
}

static void prv_will_focus(bool in_focus) {
  LauncherAppWindowData *data = app_state_get_user_data();
  if (!in_focus) {
    launcher_menu_layer_set_selection_animations_enabled(&data->launcher_menu_layer, false);
  }
}

////////////////////////////////
// AppMenuDataSource callbacks

static bool prv_app_filter_callback(PBL_UNUSED AppMenuDataSource *source, AppInstallEntry *entry) {
  // Skip watchfaces and hidden apps
  return (!app_install_entry_is_watchface(entry) && !app_install_entry_is_hidden((entry)));
}

static void prv_data_changed(void *context) {
  LauncherAppWindowData *data = context;
  launcher_menu_layer_reload_data(&data->launcher_menu_layer);
}

//! We're not 100% sure of the order of the launcher list yet, so use this function to transform
//! the row index to achieve the desired list ordering
static uint16_t prv_transform_index(AppMenuDataSource *data_source, uint16_t original_index,
                                    void *context) {
#ifdef CONFIG_SHELL_SDK
  // We want the newest installed developer app to appear at the top
  // This works at the moment because there is only one system app, Watchfaces
  return app_menu_data_source_get_count(data_source) - 1 - original_index;
#else
  return original_index;
#endif
}

/////////////////////
// Window callbacks

static void prv_window_load(Window *window) {
  LauncherAppWindowData *data = window_get_user_data(window);

  Layer *window_root_layer = window_get_root_layer(window);

  AppMenuDataSource *data_source = &data->app_menu_data_source;
  app_menu_data_source_init(data_source, &(AppMenuDataSourceCallbacks) {
    .changed = prv_data_changed,
    .filter = prv_app_filter_callback,
    .transform_index = prv_transform_index,
  }, data);

  LauncherMenuLayer *launcher_menu_layer = &data->launcher_menu_layer;
  launcher_menu_layer_init(launcher_menu_layer, data_source);
  launcher_menu_layer_set_click_config_onto_window(launcher_menu_layer, window);
  layer_add_child(window_root_layer, launcher_menu_layer_get_layer(launcher_menu_layer));

  // If we have a saved launcher selection state, restore it
  if (s_launcher_app_persisted_data.valid) {
    launcher_menu_layer_set_selection_state(launcher_menu_layer,
                                            &s_launcher_app_persisted_data.selection_state);
  }

  app_focus_service_subscribe_handlers((AppFocusHandlers) {
    .did_focus = prv_did_focus,
    .will_focus = prv_will_focus,
  });
}

static void prv_window_unload(Window *window) {
  LauncherAppWindowData *data = window_get_user_data(window);

  // Capture the vertical range of the selection rectangle for compositor transition animations
  GRangeVertical launcher_selection_vertical_range;
  launcher_menu_layer_get_selection_vertical_range(&data->launcher_menu_layer,
                                                   &launcher_selection_vertical_range);

  // Save the current state of the launcher so we can know its draw state and restore it later
  s_launcher_app_persisted_data = (LauncherAppPersistedData) {
    .valid = true,
    .leave_time = rtc_get_ticks(),
    .draw_state.selection_vertical_range = launcher_selection_vertical_range,
    .draw_state.selection_background_color = shell_prefs_get_theme_highlight_color(),
  };
  launcher_menu_layer_get_selection_state(&data->launcher_menu_layer,
                                          &s_launcher_app_persisted_data.selection_state);

  app_focus_service_unsubscribe();
  launcher_menu_layer_deinit(&data->launcher_menu_layer);
  app_menu_data_source_deinit(&data->app_menu_data_source);
}

////////////////////
// App boilerplate

static void prv_launcher_menu_window_push(void) {
  LauncherAppWindowData *data = app_zalloc_check(sizeof(*data));
  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, WINDOW_NAME("Launcher Menu"));
  window_set_user_data(window, data);
  window_set_window_handlers(window, &(WindowHandlers) {
    .load = prv_window_load,
    .unload = prv_window_unload,
  });

  const bool animated = false;
  app_window_stack_push(window, animated);
}

static void prv_main(void) {
  const LauncherMenuArgs *args = (const LauncherMenuArgs *)app_manager_get_task_context()->args;
  // Reset the selection state of the launcher if we're visiting it for the first time or if
  // it has been more than RETURN_TIMEOUT_TICKS since we were last in the launcher
  if (args && args->reset_scroll) {
    if ((s_launcher_app_persisted_data.leave_time + RETURN_TIMEOUT_TICKS) <= rtc_get_ticks()) {
      s_launcher_app_persisted_data.valid = false;
    }
  }

  prv_launcher_menu_window_push();

  app_idle_timeout_start();

  app_event_loop();
}

const PebbleProcessMd *launcher_menu_app_get_app_info(void) {
  static const PebbleProcessMdSystem s_launcher_menu_app_info = {
    .common = {
      .main_func = prv_main,
      // UUID: dec0424c-0625-4878-b1f2-147e57e83688
      .uuid = {0xde, 0xc0, 0x42, 0x4c, 0x06, 0x25, 0x48, 0x78,
               0xb1, 0xf2, 0x14, 0x7e, 0x57, 0xe8, 0x36, 0x88},
      .visibility = ProcessVisibilityHidden
    },
    .name = "Launcher",
  };
  return (const PebbleProcessMd *)&s_launcher_menu_app_info;
}

const LauncherDrawState *launcher_app_get_draw_state(void) {
  return &s_launcher_app_persisted_data.draw_state;
}
