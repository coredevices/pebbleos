/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "settings.h"
#include "activity_tracker.h"
#include "system.h"
#include "bluetooth.h"
#include "display.h"
#include "menu.h"
#include "notifications.h"
#include "quick_launch.h"
#include "remote.h"
#include "time.h"
#include "window.h"

#include "applib/app.h"
#include "applib/battery_state_service.h"
#include "applib/event_service_client.h"
#include "applib/fonts/fonts.h"
#include "applib/ui/menu_layer.h"
#include "applib/ui/option_menu_window.h"
#include "applib/ui/ui.h"
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "kernel/util/fw_reset.h"
#include "process_management/app_manager.h"
#include "process_state/app_state/app_state.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/system_task.h"
#include "system/bootbits.h"
#include "shell/system_theme.h"
#include "system/passert.h"
#include "shell/prefs.h"

#include <stdio.h>
#include <string.h>

typedef struct SettingsData {
  Window window;
  StatusBarLayer status_layer;
  MenuLayer menu_layer;

  SettingsMenuItem current_category; //!< SettingsMenuItem_Invalid if not currently in a category.

  //! Optional explicit title (used by nested submenus). When NULL, the title
  //! falls back to the category's status name.
  const char *title_override;

  //! Previous app-state user_data value stashed at window creation; restored
  //! on unload so nested settings windows don't leak pointers to freed data.
  void *prev_app_user_data;

  const char *title;
  SettingsCallbacks *callbacks;

  ClickConfigProvider menu_layer_click_config; //! HACK: Used to register a back click.

  EventServiceInfo pref_change_event_info; //!< Subscription for pref change notifications
} SettingsData;


// Pref change handler
///////////////////////

static void prv_refresh_theme_colors(SettingsData *data) {
  const GColor bg = system_theme_get_bg_color();
  const GColor fg = system_theme_get_fg_color();
  window_set_background_color(&data->window, bg);
  status_bar_layer_set_colors(&data->status_layer, bg, fg);
  menu_layer_set_normal_colors(&data->menu_layer, bg, fg);
  layer_mark_dirty(menu_layer_get_layer(&data->menu_layer));
}

static void prv_pref_change_handler(PebbleEvent *event, void *context) {
  SettingsData *data = context;
  prv_refresh_theme_colors(data);
}

// Filter category helpers
//////////////////////////

static SettingsCallbacks *prv_get_current_callbacks(SettingsData *data) {
  PBL_ASSERTN(data && data->callbacks);
  return data->callbacks;
}

// Menu appearance helpers
//////////////////////////

static void prv_set_sub_menu_colors(GContext *ctx, const Layer *cell_layer, bool highlight) {
  if (highlight) {
    GColor highlight_bg = shell_prefs_get_theme_highlight_color();
    graphics_context_set_fill_color(ctx, highlight_bg);
    graphics_context_set_text_color(ctx, gcolor_legible_over(highlight_bg));
  } else {
    graphics_context_set_fill_color(ctx, system_theme_get_bg_color());
    graphics_context_set_text_color(ctx, system_theme_get_fg_color());
  }
  graphics_fill_rect(ctx, &cell_layer->bounds);
}

// Menu Layer Handling
//////////////////////

static void prv_select_callback(MenuLayer *menu_layer, MenuIndex *cell_index, void *context) {
  SettingsData *data = context;

  const uint16_t row = cell_index->row;
  SettingsCallbacks *callbacks = prv_get_current_callbacks(data);
  if (callbacks->select_click) {
    callbacks->select_click(callbacks, row);
  }
}

static void prv_selection_changed_callback(MenuLayer *menu_layer, MenuIndex new_index,
                                           MenuIndex old_index, void *context) {
  SettingsData *data = context;

  const uint16_t new_row = new_index.row;
  const uint16_t old_row = old_index.row;

  SettingsCallbacks *callbacks = prv_get_current_callbacks(data);
  if (callbacks->selection_changed) {
    callbacks->selection_changed(callbacks, new_row, old_row);
  }
}

static void prv_selection_will_change_callback(MenuLayer *menu_layer, MenuIndex *new_index,
                                               MenuIndex old_index, void *context) {
  SettingsData *data = context;

  const uint16_t old_row = old_index.row;

  SettingsCallbacks *callbacks = prv_get_current_callbacks(data);
  if (callbacks->selection_will_change) {
    callbacks->selection_will_change(callbacks, &new_index->row, old_row);
  }
}

static void prv_draw_row_callback(GContext *ctx, const Layer *cell_layer,
                                  MenuIndex *cell_index, void *context) {
  SettingsData *data = context;

  uint16_t row = cell_index->row;
  const uint16_t section = cell_index->section;
  PBL_ASSERTN(section < SettingsMenuItem_Count);

  bool highlight = menu_cell_layer_is_highlighted(cell_layer);

  SettingsCallbacks *callbacks = prv_get_current_callbacks(data);
  prv_set_sub_menu_colors(ctx, cell_layer, highlight);
  if (callbacks->draw_row) {
    const bool is_selected = menu_cell_layer_is_highlighted(cell_layer);
    callbacks->draw_row(callbacks, ctx, cell_layer, row, is_selected);
  }
}

static uint16_t prv_get_num_rows_callback(MenuLayer *menu_layer,
                                          uint16_t section_index, void *context) {
  PBL_ASSERTN(section_index < SettingsMenuItem_Count);
  SettingsData *data = context;

  SettingsCallbacks *callbacks = prv_get_current_callbacks(data);
  return callbacks->num_rows ? callbacks->num_rows(callbacks) : (uint16_t)0;
}

static int16_t prv_get_cell_height_callback(MenuLayer *menu_layer,
                                            MenuIndex *cell_index, void *context) {
  PBL_ASSERTN(cell_index->section < SettingsMenuItem_Count);
  SettingsData *data = context;

  const uint16_t row = cell_index->row;
  SettingsCallbacks *callbacks = prv_get_current_callbacks(data);
  const bool is_selected = menu_layer_is_index_selected(menu_layer, cell_index);
  return (callbacks->row_height) ?
      callbacks->row_height(callbacks, row, is_selected) :
      PBL_IF_RECT_ELSE(menu_cell_basic_cell_height(),
                       (is_selected ? MENU_CELL_ROUND_FOCUSED_SHORT_CELL_HEIGHT :
                                      MENU_CELL_ROUND_UNFOCUSED_TALL_CELL_HEIGHT));
}

// Settings Window:
////////////////////////

static void prv_settings_window_load(Window *window) {
  SettingsData *data = window_get_user_data(window);

  StatusBarLayer *status_layer = &data->status_layer;
  status_bar_layer_init(status_layer);
  const char *title = data->title_override
      ? data->title_override
      : settings_menu_get_status_name(data->current_category);
  status_bar_layer_set_title(status_layer, i18n_get(title, data), false, false);
  status_bar_layer_set_colors(status_layer, system_theme_get_bg_color(), system_theme_get_fg_color());
  status_bar_layer_set_separator_mode(status_layer, OPTION_MENU_STATUS_SEPARATOR_MODE);
  layer_add_child(&data->window.layer, status_bar_layer_get_layer(status_layer));

  GRect bounds = grect_inset(data->window.layer.bounds, (GEdgeInsets) {
    .top = STATUS_BAR_LAYER_HEIGHT,
    .bottom = PBL_IF_RECT_ELSE(0, STATUS_BAR_LAYER_HEIGHT),
  });

  // Create the menu
  MenuLayer *menu_layer = &data->menu_layer;
  menu_layer_init(menu_layer, &bounds);
  menu_layer_set_callbacks(menu_layer, data, &(MenuLayerCallbacks) {
    .get_num_rows = prv_get_num_rows_callback,
    .get_cell_height = prv_get_cell_height_callback,
    .draw_row = prv_draw_row_callback,
    .select_click = prv_select_callback,
    .selection_changed = prv_selection_changed_callback,
    .selection_will_change = prv_selection_will_change_callback,
  });
  menu_layer_set_click_config_onto_window(menu_layer, &data->window);
  menu_layer_set_scroll_wrap_around(menu_layer, shell_prefs_get_menu_scroll_wrap_around_enable());
  menu_layer_set_scroll_vibe_on_wrap(menu_layer, shell_prefs_get_menu_scroll_vibe_behavior() == MenuScrollVibeOnWrapAround);
  menu_layer_set_scroll_vibe_on_blocked(menu_layer, shell_prefs_get_menu_scroll_vibe_behavior() == MenuScrollVibeOnLocked);
  layer_add_child(&data->window.layer, menu_layer_get_layer(menu_layer));

  SettingsCallbacks *callbacks = prv_get_current_callbacks(data);
  if (callbacks->get_initial_selection) {
    const uint16_t selected_row = callbacks->get_initial_selection(data->callbacks);
    menu_layer_set_selected_index(menu_layer, MenuIndex(0, selected_row), MenuRowAlignCenter,
                                  false /* animated */);
  }

  if (callbacks->expand) {
    callbacks->expand(data->callbacks);
  }

  // Subscribe to pref change events to auto-refresh when settings change remotely
  data->pref_change_event_info = (EventServiceInfo) {
    .type = PEBBLE_PREF_CHANGE_EVENT,
    .handler = prv_pref_change_handler,
    .context = data,
  };
  event_service_client_subscribe(&data->pref_change_event_info);
}

static void prv_settings_window_appear(Window *window) {
  SettingsData *data = window_get_user_data(window);
  // Refresh colors in case the theme changed while this window was hidden
  prv_refresh_theme_colors(data);
  SettingsCallbacks *callbacks = prv_get_current_callbacks(data);
  if (callbacks->appear) {
    callbacks->appear(data->callbacks);
  }
}

static void prv_settings_window_unload(Window *window) {
  SettingsData *data = window_get_user_data(window);

  // Unsubscribe from pref change events
  event_service_client_unsubscribe(&data->pref_change_event_info);

  // Call the hide callback for the currently open category.
  SettingsCallbacks *callbacks = prv_get_current_callbacks(data);
  if (callbacks->hide) {
    callbacks->hide(callbacks);
  }
  i18n_free_all(data);
  menu_layer_deinit(&data->menu_layer);
  status_bar_layer_deinit(&data->status_layer);
  settings_window_destroy(window);
}

static Window *prv_create(SettingsMenuItem category, const char *title_override,
                          SettingsCallbacks *callbacks) {
  PBL_ASSERTN(callbacks && (category < SettingsMenuItem_Count));

  SettingsData *data = app_zalloc_check(sizeof(*data));

  data->current_category = category;
  data->title = settings_menu_get_submodule_info(category)->name;
  data->title_override = title_override;
  data->callbacks = callbacks;

  // Stash the previously-active settings data so we can restore it on unload
  // and not leave app-state user_data pointing at freed memory after a nested
  // submenu is popped.
  data->prev_app_user_data = app_state_get_user_data();
  app_state_set_user_data(data);

  window_init(&data->window, WINDOW_NAME("Settings Window"));
  window_set_user_data(&data->window, data);
  window_set_window_handlers(&data->window, &(WindowHandlers){
    .load = prv_settings_window_load,
    .appear = prv_settings_window_appear,
    .unload = prv_settings_window_unload,
  });

  return &data->window;
}

Window *settings_window_create(SettingsMenuItem category, SettingsCallbacks *callbacks) {
  return prv_create(category, NULL, callbacks);
}

Window *settings_window_create_with_title(SettingsMenuItem category, const char *title,
                                          SettingsCallbacks *callbacks) {
  return prv_create(category, title, callbacks);
}

void settings_window_destroy(Window *window) {
  SettingsData *data = window_get_user_data(window);

  SettingsCallbacks *callbacks = prv_get_current_callbacks(data);
  if (callbacks->deinit) {
    callbacks->deinit(data->callbacks);
  }

  // Restore whatever was in app-state user_data before we took over.
  app_state_set_user_data(data->prev_app_user_data);

  i18n_free_all(data);
  app_free(data);
}

void settings_menu_mark_dirty(SettingsMenuItem category) {
  SettingsData *data = app_state_get_user_data();
  if (data->current_category == category) {
    prv_refresh_theme_colors(data);
  }
}

void settings_menu_reload_data(SettingsMenuItem category) {
  SettingsData *data = app_state_get_user_data();
  if (data->current_category == category) {
    menu_layer_reload_data(&data->menu_layer);
  }
}

int16_t settings_menu_get_selected_row(SettingsMenuItem category) {
  SettingsData *data = app_state_get_user_data();
  if (data->current_category == category) {
    return menu_layer_get_selected_index(&data->menu_layer).row;
  }
  return 0; // say first row is selected if all else fails
}
