/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "selection.h"
#include "utils.h"

#include "applib/app.h"
#include "applib/ui/ui.h"
#include "kernel/pbl_malloc.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"
#include "shell/prefs.h"
#include <pbl/logging/logging.h>

#include <stdio.h>

typedef enum {
  WorkoutType_Run,
  WorkoutType_Walk,
  WorkoutType_OpenWorkout,
  WorkoutTypeCount,
} WorkoutType;

typedef struct WorkoutSelectionWindow {
  Window window;
  MenuLayer menu_layer;
  GBitmap workout_icons[WorkoutTypeCount];
  SelectWorkoutCallback select_workout_cb;
} WorkoutSelectionWindow;

////////////////////////////////////////////////////////////////////////////////////////////////////
//! Helpers

static uint32_t prv_get_icon_resource_id(WorkoutType workout_type) {
  switch (workout_type) {
    case WorkoutType_Run:
      return RESOURCE_ID_WORKOUT_APP_RUN_SMALL;
    case WorkoutType_Walk:
      return RESOURCE_ID_WORKOUT_APP_WALK_SMALL;
    case WorkoutType_OpenWorkout:
      return RESOURCE_ID_WORKOUT_APP_WORKOUT_SMALL;
    default:
      return 0;
  }
}

static ActivitySessionType prv_get_activity_type(WorkoutType workout_type) {
  switch (workout_type) {
    case WorkoutType_Run:
      return ActivitySessionType_Run;
    case WorkoutType_Walk:
      return ActivitySessionType_Walk;
    case WorkoutType_OpenWorkout:
      return ActivitySessionType_Open;
    default:
      return ActivitySessionType_Invalid;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//! Menu Layer Callbacks

static uint16_t prv_get_num_rows_callback(MenuLayer *menu_layer, uint16_t section_index,
                                          void *context) {
  return WorkoutTypeCount;
}

static int16_t prv_get_cell_height_callback(MenuLayer *menu_layer, MenuIndex *cell_index,
                                            void *context) {
#if PBL_RECT
  return 56;
#else
  return menu_layer_is_index_selected(menu_layer, cell_index) ? 84 : 38;
#endif
}

static void prv_draw_row_callback(GContext *ctx, const Layer *cell_layer, MenuIndex *cell_index,
                                  void *context) {
  WorkoutSelectionWindow *selection_window = (WorkoutSelectionWindow *)context;

  ActivitySessionType activity_type = prv_get_activity_type(cell_index->row);

  const char *title = workout_utils_get_name_for_activity(activity_type);
  const GBitmap *icon = &selection_window->workout_icons[cell_index->row];

  const int icon_top_padding = 11;
  const int title_top_padding = PBL_IF_RECT_ELSE(11, cell_layer->is_highlighted ? 40 : 2);
  const int max_icon_w = PBL_IF_RECT_ELSE(55, cell_layer->bounds.size.w);
  const int title_origin_x = PBL_IF_RECT_ELSE(max_icon_w, 0);
  const GTextAlignment title_alignment = PBL_IF_RECT_ELSE(GTextAlignmentLeft, GTextAlignmentCenter);
  const GFont title_font = PBL_IF_RECT_ELSE(fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD),
      cell_layer->is_highlighted ? fonts_get_system_font(FONT_KEY_GOTHIC_28_BOLD)
                                 : fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
  const int title_height = fonts_get_font_height(title_font);

  GRect image_bounds = gbitmap_get_bounds(icon);
  image_bounds.origin.x = (max_icon_w / 2) - (image_bounds.size.w / 2);
  image_bounds.origin.y = icon_top_padding;

#if PBL_COLOR
  GCompOp compositing_mode = GCompOpSet;
#else
  GCompOp compositing_mode = cell_layer->is_highlighted ? GCompOpTintLuminance : GCompOpSet;
  graphics_context_set_tint_color(ctx, (cell_layer->is_highlighted ? GColorWhite : GColorBlack));
#endif

  graphics_context_set_compositing_mode(ctx, compositing_mode);

#if PBL_ROUND
  if (cell_layer->is_highlighted) {
    graphics_draw_bitmap_in_rect(ctx, icon, &image_bounds);
  }
#else
  graphics_draw_bitmap_in_rect(ctx, icon, &image_bounds);
#endif

  GRect title_bounds = cell_layer->bounds;
  title_bounds.origin.x = title_origin_x;
  title_bounds.origin.y = title_top_padding;
  title_bounds.size.w -= title_origin_x;
  title_bounds.size.h = title_height;

  graphics_draw_text(ctx, i18n_get(title, selection_window), title_font, title_bounds,
                     GTextOverflowModeFill, title_alignment, NULL);
}

static void prv_select_callback(MenuLayer *menu_layer, MenuIndex *cell_index, void *context) {
  WorkoutSelectionWindow *selection_window = (WorkoutSelectionWindow *)context;

  ActivitySessionType activity_type = prv_get_activity_type(cell_index->row);
  selection_window->select_workout_cb(activity_type);
  window_stack_remove(&selection_window->window, true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//! Handlers

static void prv_window_unload_handler(Window *window) {
  WorkoutSelectionWindow *selection_window = window_get_user_data(window);
  if (selection_window) {
    for (int i = 0; i < WorkoutTypeCount; i++) {
      gbitmap_deinit(&selection_window->workout_icons[i]);
    }
    menu_layer_deinit(&selection_window->menu_layer);
    window_deinit(&selection_window->window);
    i18n_free_all(selection_window);
    app_free(selection_window);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//! Public API

WorkoutSelectionWindow *workout_selection_push(SelectWorkoutCallback select_workout_cb) {
  WorkoutSelectionWindow *selection_window = app_zalloc_check(sizeof(WorkoutSelectionWindow));

  selection_window->select_workout_cb = select_workout_cb;

  Window *window = &selection_window->window;
  window_init(window, WINDOW_NAME("Workout Selection"));
  window_set_user_data(window, selection_window);
  window_set_window_handlers(window, &(WindowHandlers){
    .unload = prv_window_unload_handler,
  });

  for (int i = 0; i < WorkoutTypeCount; i++) {
    gbitmap_init_with_resource(&selection_window->workout_icons[i], prv_get_icon_resource_id(i));
  }

  MenuLayer *menu_layer = &selection_window->menu_layer;
  menu_layer_init(menu_layer, &window->layer.bounds);
  menu_layer_pad_bottom_enable(menu_layer, false);
  menu_layer_set_callbacks(menu_layer, selection_window, &(MenuLayerCallbacks){
    .get_num_rows = prv_get_num_rows_callback,
    .get_cell_height = prv_get_cell_height_callback,
    .draw_row = prv_draw_row_callback,
    .select_click = prv_select_callback,
  });
  menu_layer_set_normal_colors(menu_layer, system_theme_get_bg_color(), system_theme_get_fg_color());
  menu_layer_set_highlight_colors(menu_layer,
                                  PBL_IF_COLOR_ELSE(GColorYellow, GColorBlack),
                                  PBL_IF_COLOR_ELSE(GColorBlack, GColorWhite));
  menu_layer_set_click_config_onto_window(menu_layer, &selection_window->window);
  menu_layer_set_scroll_wrap_around(menu_layer, shell_prefs_get_menu_scroll_wrap_around_enable());
  menu_layer_set_scroll_vibe_on_wrap(menu_layer, shell_prefs_get_menu_scroll_vibe_behavior() == MenuScrollVibeOnWrapAround);
  menu_layer_set_scroll_vibe_on_blocked(menu_layer, shell_prefs_get_menu_scroll_vibe_behavior() == MenuScrollVibeOnLocked);
  layer_add_child(&selection_window->window.layer, menu_layer_get_layer(menu_layer));

  app_window_stack_push(&selection_window->window, true);

  return selection_window;
}
