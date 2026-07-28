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
#include "process_management/process_manager.h"
#include "shell/normal/watchface.h"
#include "process_state/app_state/app_state.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"
#include "shell/prefs.h"
#include "system/passert.h"
#ifdef CONFIG_TOUCH
#include "applib/app_launch_reason.h"
#include "applib/app_timer.h"
#include "applib/touch_service.h"
#include "applib/ui/animation.h"
#include "applib/ui/content_indicator.h"
#include "applib/ui/property_animation.h"
#include "applib/ui/vibes.h"
#include <pbl/util/math.h>
#endif

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

#ifdef CONFIG_TOUCH
////////////////////////////////////////////////////////////////////////////
// Carousel selector, entered by long-pressing the touchscreen on a watchface

#define SELECTOR_SWIPE_MIN_PX (30)
#define SELECTOR_SLIDE_DURATION_MS (150)
//! The finger that long-pressed to open the selector may still be down when we
//! subscribe and gets re-reported as a fresh touchdown; ignore contacts that
//! start within this window so releasing that finger isn't taken as a tap.
#define SELECTOR_ENTRY_GRACE_MS (500)

typedef struct WatchfacesSelectorData {
  Window window;
  Layer chevron_layer;
  Layer content_layer;
  AppMenuDataSource data_source;
  AppInstallId active_watchface_id;
  uint16_t index;
  int16_t touch_down_x;
  int16_t touch_down_y;
  bool touch_tracking;
  bool entry_grace;
  AppTimer *entry_grace_timer;
  PropertyAnimation *slide_animation;
} WatchfacesSelectorData;

static void prv_selector_content_update_proc(Layer *layer, GContext *ctx) {
  WatchfacesSelectorData *data = window_get_user_data(layer_get_window(layer));
  const uint16_t count = app_menu_data_source_get_count(&data->data_source);
  if (count == 0) {
    return;
  }
  AppMenuNode *node = app_menu_data_source_get_node_at_index(&data->data_source, data->index);

  const GRect bounds = layer->bounds;
  graphics_context_set_text_color(ctx, GColorWhite);

  // Watchface icon (falls back to the generic watchface icon), centered above the name
  GBitmap *icon = app_menu_data_source_get_node_icon(&data->data_source, node);
  if (icon) {
    const GSize icon_size = gbitmap_get_bounds(icon).size;
    const GCompOp op = (gbitmap_get_format(icon) == GBitmapFormat1Bit) ? GCompOpTint : GCompOpSet;
    graphics_context_set_compositing_mode(ctx, op);
    const GRect icon_rect = GRect((bounds.size.w - icon_size.w) / 2,
                                  bounds.size.h / 2 - 48 - icon_size.h, icon_size.w, icon_size.h);
    graphics_draw_bitmap_in_rect(ctx, icon, &icon_rect);
  }

  // Watchface name, centered
  GFont name_font = fonts_get_system_font(FONT_KEY_GOTHIC_28_BOLD);
  const GRect name_rect = GRect(10, bounds.size.h / 2 - 36, bounds.size.w - 20, 64);
  graphics_draw_text(ctx, node->name, name_font, name_rect, GTextOverflowModeTrailingEllipsis,
                     GTextAlignmentCenter, NULL);

  GFont small_font = fonts_get_system_font(FONT_KEY_GOTHIC_18);

  // "Active" badge under the name for the current default
  if (node->install_id == data->active_watchface_id) {
    const GRect active_rect = GRect(10, bounds.size.h / 2 + 30, bounds.size.w - 20, 24);
    graphics_draw_text(ctx, i18n_get("Active", data), small_font, active_rect,
                       GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
  }

  // Position counter below the Active badge
  char counter[12];
  snprintf(counter, sizeof(counter), "%d/%d", data->index + 1, count);
  const GRect counter_rect = GRect(10, bounds.size.h / 2 + 58, bounds.size.w - 20, 24);
  graphics_draw_text(ctx, counter, small_font, counter_rect, GTextOverflowModeTrailingEllipsis,
                     GTextAlignmentCenter, NULL);
}

static void prv_selector_chevron_update_proc(Layer *layer, GContext *ctx) {
  WatchfacesSelectorData *data = window_get_user_data(layer_get_window(layer));
  if (app_menu_data_source_get_count(&data->data_source) < 2) {
    return;
  }
  const GRect bounds = layer->bounds;
  const GRect top_rect = GRect(0, 8, bounds.size.w, 16);
  const GRect bottom_rect = GRect(0, bounds.size.h - 24, bounds.size.w, 16);
  content_indicator_draw_arrow(ctx, &top_rect, ContentIndicatorDirectionUp, GColorWhite,
                               GColorBlack, GAlignCenter);
  content_indicator_draw_arrow(ctx, &bottom_rect, ContentIndicatorDirectionDown, GColorWhite,
                               GColorBlack, GAlignCenter);
}

static void prv_selector_slide(WatchfacesSelectorData *data, int direction) {
  if (data->slide_animation) {
    animation_unschedule(property_animation_get_animation(data->slide_animation));
    data->slide_animation = NULL;
  }
  GRect to = data->window.layer.bounds;
  GRect from = to;
  from.origin.y = (direction > 0) ? to.size.h : -to.size.h;
  layer_set_frame(&data->content_layer, &from);
  data->slide_animation = property_animation_create_layer_frame(&data->content_layer, &from, &to);
  if (data->slide_animation) {
    Animation *animation = property_animation_get_animation(data->slide_animation);
    animation_set_duration(animation, SELECTOR_SLIDE_DURATION_MS);
    animation_set_curve(animation, AnimationCurveEaseOut);
    animation_schedule(animation);
  }
  layer_mark_dirty(&data->content_layer);
}

//! direction: +1 = next (slide in from the bottom), -1 = previous
static void prv_selector_navigate(WatchfacesSelectorData *data, int direction) {
  const uint16_t count = app_menu_data_source_get_count(&data->data_source);
  if (count < 2) {
    return;
  }
  data->index = (data->index + count + direction) % count;
  // Swiping selects: the face landed on becomes the default, so backing out
  // of the selector returns to it.
  AppMenuNode *node = app_menu_data_source_get_node_at_index(&data->data_source, data->index);
  watchface_set_default_install_id(node->install_id);
  data->active_watchface_id = node->install_id;
  prv_selector_slide(data, direction);
}

static void prv_selector_select(WatchfacesSelectorData *data) {
  const uint16_t count = app_menu_data_source_get_count(&data->data_source);
  if (count == 0) {
    return;
  }
  AppMenuNode *node = app_menu_data_source_get_node_at_index(&data->data_source, data->index);
  // Launching with APP_LAUNCH_USER makes app_manager persist this watchface
  // as the new default once it starts successfully.
  app_manager_put_launch_app_event(&(AppLaunchEventConfig) {
    .id = node->install_id,
    .common.reason = APP_LAUNCH_USER,
    .common.button = BUTTON_ID_SELECT,
  });
}

static void prv_selector_touch_handler(const TouchEvent *event, void *context) {
  WatchfacesSelectorData *data = context;
  switch (event->type) {
    case TouchEvent_Touchdown:
      if (data->entry_grace) {
        break;
      }
      data->touch_tracking = true;
      data->touch_down_x = event->x;
      data->touch_down_y = event->y;
      break;
    case TouchEvent_Liftoff: {
      // Ignore the liftoff of the long press that opened the selector
      if (!data->touch_tracking) {
        break;
      }
      data->touch_tracking = false;
      const int16_t dx = event->x - data->touch_down_x;
      const int16_t dy = event->y - data->touch_down_y;
      if (ABS(dy) >= SELECTOR_SWIPE_MIN_PX && ABS(dy) > ABS(dx)) {
        // Swipe up moves forward, swipe down moves back; horizontal swipes
        // are ignored.
        prv_selector_navigate(data, (dy < 0) ? 1 : -1);
      } else if (ABS(dx) < SELECTOR_SWIPE_MIN_PX && ABS(dy) < SELECTOR_SWIPE_MIN_PX) {
        prv_selector_select(data);
      }
      break;
    }
    case TouchEvent_PositionUpdate:
      break;
  }
}

static void prv_selector_up_click_handler(ClickRecognizerRef recognizer, void *context) {
  prv_selector_navigate(context, -1);
}

static void prv_selector_down_click_handler(ClickRecognizerRef recognizer, void *context) {
  prv_selector_navigate(context, 1);
}

static void prv_selector_select_click_handler(ClickRecognizerRef recognizer, void *context) {
  prv_selector_select(context);
}

static void prv_selector_click_config_provider(void *context) {
  window_single_click_subscribe(BUTTON_ID_UP, prv_selector_up_click_handler);
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_selector_down_click_handler);
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_selector_select_click_handler);
  window_set_click_context(BUTTON_ID_UP, context);
  window_set_click_context(BUTTON_ID_DOWN, context);
  window_set_click_context(BUTTON_ID_SELECT, context);
}

static void prv_selector_data_source_changed(void *context) {
  WatchfacesSelectorData *data = context;
  const uint16_t count = app_menu_data_source_get_count(&data->data_source);
  if (count > 0 && data->index >= count) {
    data->index = count - 1;
  }
  layer_mark_dirty(&data->content_layer);
  layer_mark_dirty(&data->chevron_layer);
}

static void prv_selector_entry_grace_timer_callback(void *context) {
  WatchfacesSelectorData *data = context;
  data->entry_grace_timer = NULL;
  data->entry_grace = false;
}

static void prv_selector_window_load(Window *window) {
  WatchfacesSelectorData *data = window_get_user_data(window);

  app_menu_data_source_init(&data->data_source, &(AppMenuDataSourceCallbacks) {
    .changed = prv_selector_data_source_changed,
    .filter = prv_app_filter_callback,
  }, data);
  app_menu_data_source_enable_icons(&data->data_source,
                                    RESOURCE_ID_MENU_LAYER_GENERIC_WATCHFACE_ICON);

  data->active_watchface_id = watchface_get_default_install_id();
  const uint16_t row = app_menu_data_source_get_index_of_app_with_install_id(
      &data->data_source, data->active_watchface_id);
  data->index = (row == (uint16_t)-1) ? 0 : row;

  layer_init(&data->chevron_layer, &window->layer.bounds);
  layer_set_update_proc(&data->chevron_layer, prv_selector_chevron_update_proc);
  layer_add_child(&window->layer, &data->chevron_layer);

  layer_init(&data->content_layer, &window->layer.bounds);
  layer_set_update_proc(&data->content_layer, prv_selector_content_update_proc);
  layer_add_child(&window->layer, &data->content_layer);

  data->entry_grace = true;
  data->entry_grace_timer = app_timer_register(SELECTOR_ENTRY_GRACE_MS,
                                               prv_selector_entry_grace_timer_callback, data);
  touch_service_subscribe(prv_selector_touch_handler, data);
  vibes_short_pulse();
}

static void prv_selector_window_unload(Window *window) {
  WatchfacesSelectorData *data = window_get_user_data(window);
  if (data->entry_grace_timer) {
    app_timer_cancel(data->entry_grace_timer);
    data->entry_grace_timer = NULL;
  }
  if (data->slide_animation) {
    animation_unschedule(property_animation_get_animation(data->slide_animation));
    data->slide_animation = NULL;
  }
  touch_service_unsubscribe();
  layer_deinit(&data->content_layer);
  layer_deinit(&data->chevron_layer);
  app_menu_data_source_deinit(&data->data_source);
  i18n_free_all(data);
}

static void prv_selector_init(void) {
  WatchfacesSelectorData *data = app_malloc_check(sizeof(WatchfacesSelectorData));
  *data = (WatchfacesSelectorData){};
  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, WINDOW_NAME("Watchface Selector"));
  window_set_user_data(window, data);
  window_set_background_color(window, GColorBlack);
  window_set_click_config_provider_with_context(window, prv_selector_click_config_provider, data);
  window_set_window_handlers(window, &(WindowHandlers) {
    .load = prv_selector_window_load,
    .unload = prv_selector_window_unload,
  });
  app_window_stack_push(window, true /* animated */);
}
#endif // CONFIG_TOUCH

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
#ifdef CONFIG_TOUCH
  // Only the shell's long-press path launches with APP_LAUNCH_SYSTEM and
  // WatchfacesLaunchArgs; quick launch reuses args for small integer action
  // codes, so the reason gate must come before the args dereference.
  const WatchfacesLaunchArgs *args =
      (app_launch_reason() == APP_LAUNCH_SYSTEM) ?
          process_manager_get_current_process_args() : NULL;
  if (args && args->selector_mode) {
    prv_selector_init();
  } else {
    handle_init();
  }
#else
  handle_init();
#endif

  app_event_loop();
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

