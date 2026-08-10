/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "watchfaces.h"

#include "applib/app.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/dialogs/simple_dialog.h"
#include "applib/ui/kino/kino_reel.h"
#include "applib/ui/menu_layer.h"
#include "applib/ui/window.h"
#include "applib/ui/window_stack.h"
#include "applib/graphics/graphics.h"
#include "kernel/pbl_malloc.h"
#include "process_management/app_manager.h"
#include "process_management/app_menu_data_source.h"
#include "shell/normal/app_idle_timeout.h"
#include "shell/normal/watchface.h"
#include "process_state/app_state/app_state.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"
#include "shell/prefs.h"
#include "system/passert.h"

#include <stdio.h>
#include <string.h>
#include <stdio.h>

typedef struct SettingsWatchfacesData {
  Window window;
  MenuLayer menu_layer;
  AppMenuDataSource data_source;
  AppInstallId active_watchface_id;
} SettingsWatchfacesData;

////////////////////
// AppMenuDataSource callbacks

static bool prv_app_filter_callback(struct AppMenuDataSource *source, AppInstallEntry *entry) {
  if (app_install_entry_is_hidden(entry)) {
    return false;
  }
  if (app_install_entry_is_watchface(entry)) {
    return true; // Only watchfaces
  }
  return false;
}

//////////////
// MenuLayer callbacks

static uint16_t prv_transform_index(AppMenuDataSource *data_source, uint16_t original_index,
                                    void *context) {
#ifdef CONFIG_SHELL_SDK
  // We want the newest installed developer app to appear at the top
  // This works at the moment because there is only one system watchface, TicToc
  return app_menu_data_source_get_count(data_source) - 1 - original_index;
#else
  return original_index;
#endif
}

static void select_callback(MenuLayer *menu_layer, MenuIndex *cell_index, SettingsWatchfacesData *data) {
  const AppMenuNode* app_node =
      app_menu_data_source_get_node_at_index(&data->data_source, cell_index->row);

  // NOTE: The default watchface is not set here in case the app fetch fails.
  menu_layer_reload_data(menu_layer);
  app_manager_put_launch_app_event(&(AppLaunchEventConfig) {
    .id = app_node->install_id,
    .common.reason = APP_LAUNCH_USER,
    .common.button = BUTTON_ID_SELECT,
  });
}

#if PBL_ROUND
static int16_t get_cell_height_callback(struct MenuLayer *menu_layer, MenuIndex *cell_index,
                                        SettingsWatchfacesData *data) {
  return menu_layer_is_index_selected(menu_layer, cell_index) ?
         MENU_CELL_ROUND_FOCUSED_TALL_CELL_HEIGHT : MENU_CELL_ROUND_UNFOCUSED_SHORT_CELL_HEIGHT;
}
#endif

static uint16_t get_num_rows_callback(struct MenuLayer *menu_layer, uint16_t section_index, SettingsWatchfacesData *data) {
  return app_menu_data_source_get_count(&data->data_source);
}

static void draw_row_callback(GContext* ctx, const Layer *cell_layer, MenuIndex *cell_index,
                              SettingsWatchfacesData *data) {
  AppMenuNode *node = app_menu_data_source_get_node_at_index(&data->data_source, cell_index->row);
  GBitmap *bitmap = app_menu_data_source_get_node_icon(&data->data_source, node);
  const char *subtitle = (data->active_watchface_id == node->install_id) ?
      i18n_get("Active", data) : NULL;

  const GCompOp op = (gbitmap_get_format(bitmap) == GBitmapFormat1Bit) ? GCompOpTint : GCompOpSet;
  graphics_context_set_compositing_mode(ctx, op);

  // TODO: PBL-22652 extract common way to configure simple lists on S4
  PBL_UNUSED const bool selected = (cell_index->row == data->menu_layer.selection.index.row);
  // used for a fish-eye effect in the menus, also conveniently prevents us from clipping
  // during the animation
  GFont const title_font = fonts_get_system_font(
      PBL_IF_RECT_ELSE(FONT_KEY_GOTHIC_24_BOLD,
                       selected ? FONT_KEY_GOTHIC_24_BOLD : FONT_KEY_GOTHIC_18_BOLD));
  GFont const subtitle_font = fonts_get_system_font(FONT_KEY_GOTHIC_18);
  menu_cell_basic_draw_custom(ctx, cell_layer, title_font, node->name, NULL, NULL, subtitle_font,
                              subtitle, bitmap, false, GTextOverflowModeTrailingEllipsis);
}

///////////////////
// Window callbacks

static void prv_window_appear(Window *window) {
  SettingsWatchfacesData* data = (SettingsWatchfacesData*)window_get_user_data(window);

  // Select the currently active watchface:
  data->active_watchface_id = watchface_get_default_install_id();
  const uint16_t row =
      app_menu_data_source_get_index_of_app_with_install_id(&data->data_source,
                                                            data->active_watchface_id);
  const bool animated = false;
  menu_layer_set_selected_index(&data->menu_layer, MenuIndex(0, row), MenuRowAlignCenter, animated);
}

static void prv_reload_menu_data(void *data) {
  menu_layer_reload_data(data);
}

static void prv_window_load(Window *window) {
  SettingsWatchfacesData *data = window_get_user_data(window);

  MenuLayer *menu_layer = &data->menu_layer;
  const GRect menu_layer_frame =
    PBL_IF_RECT_ELSE(window->layer.bounds, grect_inset_internal(window->layer.bounds,
                                                                0, STATUS_BAR_LAYER_HEIGHT));
  menu_layer_init(menu_layer, &menu_layer_frame);
  app_menu_data_source_init(&data->data_source, &(AppMenuDataSourceCallbacks) {
    .changed = prv_reload_menu_data,
    .filter = prv_app_filter_callback,
    .transform_index = prv_transform_index,
  }, &data->menu_layer);

  app_menu_data_source_enable_icons(&data->data_source,
                                    RESOURCE_ID_MENU_LAYER_GENERIC_WATCHFACE_ICON);

  menu_layer_set_callbacks(menu_layer, data, &(MenuLayerCallbacks) {
#if PBL_ROUND
    .get_cell_height = (MenuLayerGetCellHeightCallback) get_cell_height_callback,
#endif
    .get_num_rows = (MenuLayerGetNumberOfRowsInSectionsCallback) get_num_rows_callback,
    .draw_row = (MenuLayerDrawRowCallback) draw_row_callback,
    .select_click = (MenuLayerSelectCallback) select_callback,
  });
  menu_layer_set_highlight_colors(&data->menu_layer,
                                  PBL_IF_COLOR_ELSE(GColorJazzberryJam, GColorBlack),
                                  GColorWhite);
  menu_layer_set_click_config_onto_window(menu_layer, window);
  menu_layer_set_scroll_wrap_around(menu_layer, shell_prefs_get_menu_scroll_wrap_around_enable());
  menu_layer_set_scroll_vibe_on_wrap(menu_layer, shell_prefs_get_menu_scroll_vibe_behavior() == MenuScrollVibeOnWrapAround);
  menu_layer_set_scroll_vibe_on_blocked(menu_layer, shell_prefs_get_menu_scroll_vibe_behavior() == MenuScrollVibeOnLocked);
  layer_add_child(&window->layer, menu_layer_get_layer(menu_layer));
}

static void prv_window_unload(Window *window) {
  SettingsWatchfacesData *data = window_get_user_data(window);
  menu_layer_deinit(&data->menu_layer);
  app_menu_data_source_deinit(&data->data_source);

  i18n_free_all(data);
}

static void handle_init(void) {
  SettingsWatchfacesData *data = app_malloc_check(sizeof(SettingsWatchfacesData));

  *data = (SettingsWatchfacesData){};
  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, WINDOW_NAME("Watchfaces"));
  window_set_user_data(window, data);
  window_set_window_handlers(window, &(WindowHandlers) {
    .load = prv_window_load,
    .appear = prv_window_appear,
    .unload = prv_window_unload,
  });
  const bool animated = true;
  app_window_stack_push(window, animated);
}

////////////////////
// App boilerplate

static void s_main(void) {
  handle_init();

  app_idle_timeout_start();
  app_event_loop();
  app_idle_timeout_stop();
}

const PebbleProcessMd* watchfaces_get_app_info() {
  static const PebbleProcessMdSystem s_app_md = {
    .common = {
      .main_func = s_main,
      // UUID: 18e443ce-38fd-47c8-84d5-6d0c775fbe55
      .uuid = {0x18, 0xe4, 0x43, 0xce, 0x38, 0xfd, 0x47, 0xc8,
               0x84, 0xd5, 0x6d, 0x0c, 0x77, 0x5f, 0xbe, 0x55},
    },
    .name = i18n_noop("Watchfaces"),
    .icon_resource_id = RESOURCE_ID_WATCHFACES_APP_GLANCE,
  };
  return (const PebbleProcessMd*) &s_app_md;
}

