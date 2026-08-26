/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

//! This file generates a menu that lets the user select a quicklaunch app
//! The menu that is generated is the same as the "main menu" but with a
//! title

#include "quick_launch_app_menu.h"
#include "quick_launch_setup_menu.h"
#include "quick_launch.h"
#include "menu.h"
#include "option_menu.h"

#include "applib/graphics/graphics.h"
#include "applib/graphics/text.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/option_menu_window.h"
#include "applib/ui/window_stack.h"
#include "kernel/pbl_malloc.h"
#include "pbl/services/i18n/i18n.h"
#include "process_management/app_install_manager.h"
#include "apps/system/timeline/timeline.h"
#include "process_management/app_menu_data_source.h"
#include "resource/resource_ids.auto.h"
#include "shell/prefs.h"

typedef struct {
  AppMenuDataSource data_source;
  ButtonId button;
  bool is_tap;
  int16_t selected;
  OptionMenu *option_menu;
} QuickLaunchAppMenuData;

#define NUM_CUSTOM_CELLS 1


/* Callback Functions */

static bool prv_app_filter_callback(struct AppMenuDataSource *source, AppInstallEntry *entry) {
  QuickLaunchAppMenuData *data = (QuickLaunchAppMenuData *)source->callback_context;
  const Uuid timeline_future_uuid = TIMELINE_UUID_INIT;
  const Uuid timeline_past_uuid = TIMELINE_PAST_UUID_INIT;
  const Uuid timeline_full_uuid = TIMELINE_FULL_UUID_INIT;
  
  if (app_install_entry_is_watchface(entry)) {
    return false; // Skip watchfaces
  }
  if (app_install_entry_is_hidden(entry) &&
      !app_install_entry_is_quick_launch_visible_only(entry)) {
    return false; // Skip hidden apps unless they are quick launch visible
  }
  
  // For tap buttons, filter Timeline apps based on button
  if (data->is_tap) {
    if (data->button == BUTTON_ID_UP) {
      // Tap Up: Only show Timeline Past, hide Timeline Future and Timeline Full
      if (uuid_equal(&entry->uuid, &timeline_future_uuid)) {
        return false;
      }
      if (uuid_equal(&entry->uuid, &timeline_full_uuid)) {
        return false;
      }
    } else if (data->button == BUTTON_ID_DOWN) {
      // Tap Down: Only show Timeline Future, hide Timeline Past and Timeline Full
      if (uuid_equal(&entry->uuid, &timeline_past_uuid)) {
        return false;
      }
      if (uuid_equal(&entry->uuid, &timeline_full_uuid)) {
        return false;
      }
    }
  }
  
  return true;
}

static uint16_t prv_menu_get_num_rows(OptionMenu *option_menu, void *context) {
  QuickLaunchAppMenuData *data = context;
  return app_menu_data_source_get_count(&data->data_source) + NUM_CUSTOM_CELLS;
}

static void prv_menu_draw_row(OptionMenu *option_menu, GContext* ctx, const Layer *cell_layer,
                              const GRect *text_frame, uint32_t row, bool selected, void *context) {

  QuickLaunchAppMenuData *data = context;
  const char *text = NULL;
  if (row == 0) {
    text = i18n_get("Disable", data);
  } else {
    AppMenuNode *node = app_menu_data_source_get_node_at_index(&data->data_source,
                                                               row - NUM_CUSTOM_CELLS);
    text = node->name;
  }
  option_menu_system_draw_row(option_menu, ctx, cell_layer, text_frame, text, selected, context);
}

static void prv_menu_select(OptionMenu *option_menu, int selection, void *context) {
  window_set_click_config_provider(&option_menu->window, NULL);

  QuickLaunchAppMenuData *data = context;
  if (selection == 0) {
    if (data->is_tap) {
      quick_launch_single_click_set_app(data->button, INSTALL_ID_INVALID);
      quick_launch_single_click_set_enabled(data->button, false);
    } else {
      quick_launch_set_app(data->button, INSTALL_ID_INVALID);
      quick_launch_set_enabled(data->button, false);
    }
    app_window_stack_pop(true);
  } else {
    AppMenuNode* app_menu_node =
        app_menu_data_source_get_node_at_index(&data->data_source, selection - NUM_CUSTOM_CELLS);
    if (data->is_tap) {
      quick_launch_single_click_set_app(data->button, app_menu_node->install_id);
    } else {
      quick_launch_set_app(data->button, app_menu_node->install_id);
    }
    app_window_stack_pop(true);
  }
}

static void prv_menu_reload_data(void *context) {
  QuickLaunchAppMenuData *data = context;
  option_menu_reload_data(data->option_menu);
}

static void prv_menu_unload(OptionMenu *option_menu, void *context) {
  QuickLaunchAppMenuData *data = context;

  option_menu_destroy(option_menu);
  app_menu_data_source_deinit(&data->data_source);
  i18n_free_all(data);
  app_free(data);
}

void quick_launch_app_menu_window_push(ButtonId button, bool is_tap) {
  QuickLaunchAppMenuData *data = app_zalloc_check(sizeof(*data));
  data->button = button;
  data->is_tap = is_tap;

  OptionMenu *option_menu = option_menu_create();
  data->option_menu = option_menu;

  app_menu_data_source_init(&data->data_source, &(AppMenuDataSourceCallbacks) {
    .changed = prv_menu_reload_data,
    .filter = prv_app_filter_callback,
  }, data);

  const AppInstallId install_id = is_tap ? quick_launch_single_click_get_app(button)
                                          : quick_launch_get_app(button);
  const int app_index = app_menu_data_source_get_index_of_app_with_install_id(&data->data_source,
                                                                              install_id);

  GColor highlight_bg = shell_prefs_get_theme_highlight_color();
  const OptionMenuConfig config = {
    .title = i18n_get(i18n_noop("Quick Launch"), data),
    .choice = (install_id == INSTALL_ID_INVALID) ? 0 : (app_index + NUM_CUSTOM_CELLS),
    .status_colors = { system_theme_get_bg_color(), system_theme_get_fg_color() },
    .highlight_colors = { highlight_bg, gcolor_legible_over(highlight_bg) },
    .icons_enabled = true,
  };
  option_menu_configure(option_menu, &config);
  option_menu_set_callbacks(option_menu, &(OptionMenuCallbacks) {
    .select = prv_menu_select,
    .get_num_rows = prv_menu_get_num_rows,
    .draw_row = prv_menu_draw_row,
    .unload = prv_menu_unload,
  }, data);

  const bool animated = true;
  app_window_stack_push(&option_menu->window, animated);
}
