/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "summary.h"
#include "countdown.h"
#include "selection.h"
#include "utils.h"

#include "applib/ui/action_menu_window.h"
#include "applib/ui/kino/kino_layer.h"
#include "applib/ui/ui.h"
#include "kernel/pbl_malloc.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"

#define BACKGROUND_COLOR PBL_IF_COLOR_ELSE(GColorYellow, GColorWhite)
#define TEXT_COLOR (gcolor_legible_over(BACKGROUND_COLOR))

typedef struct WorkoutSummaryWindow {
  Window window;
  ActionBarLayer action_bar;
  StatusBarLayer status_layer;
  Layer base_layer;

  GBitmap *action_bar_start;
  GBitmap *action_bar_more;

  ActivitySessionType activity_type;

  KinoReel *icon;
  const char *name;

  StartWorkoutCallback start_workout_cb;
  SelectWorkoutCallback select_workout_cb;
} WorkoutSummaryWindow;

////////////////////////////////////////////////////////////////////////////////////////////////////
//! Helpers

static KinoReel* prv_get_icon_for_activity(ActivitySessionType type) {
  switch (type) {
    case ActivitySessionType_Open:
      return kino_reel_create_with_resource(RESOURCE_ID_WORKOUT_APP_WORKOUT);
    case ActivitySessionType_Walk:
      return kino_reel_create_with_resource(RESOURCE_ID_WORKOUT_APP_WALK);
    case ActivitySessionType_Run:
    default:
      return kino_reel_create_with_resource(RESOURCE_ID_WORKOUT_APP_RUN);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//! Drawing

static void prv_render_activity_type(GContext *ctx, Layer *layer, KinoReel *icon,
                                     const char *name) {
  const GFont font = fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD);
  const GTextOverflowMode overflow_mode = GTextOverflowModeWordWrap;
  const GTextAlignment alignment = GTextAlignmentCenter;
  const int16_t rl_margin = PBL_IF_RECT_ELSE(4, 16);

  GRect drawing_rect = grect_inset(layer->bounds, GEdgeInsets(0, rl_margin));

  const GSize icon_size = kino_reel_get_size(icon);
  const int icon_x = drawing_rect.origin.x + PBL_IF_RECT_ELSE(0, (rl_margin / 2))
                   + (drawing_rect.size.w / 2) - (icon_size.w / 2);
  const int icon_y = PBL_IF_RECT_ELSE(45, 49);
  kino_reel_draw(icon, ctx, GPoint(icon_x, icon_y));

  const int name_x = drawing_rect.origin.x + PBL_IF_RECT_ELSE(0, (rl_margin / 2));
  const int name_y = PBL_IF_RECT_ELSE(107, 109);
  GRect name_rect = GRect(name_x, name_y, drawing_rect.size.w, 32);

  graphics_context_set_text_color(ctx, TEXT_COLOR);
  graphics_draw_text(ctx, name, font, name_rect, overflow_mode, alignment, NULL);
}

static void prv_base_layer_update_proc(struct Layer *layer, GContext *ctx) {
  WorkoutSummaryWindow *summary_window = window_get_user_data(layer_get_window(layer));

  prv_render_activity_type(ctx, layer, summary_window->icon, summary_window->name);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//! Handlers

static void prv_up_click_handler(ClickRecognizerRef recognizer, void *context) {
  WorkoutSummaryWindow *summary_window = context;

  workout_countdown_start(summary_window->activity_type, summary_window->start_workout_cb);

  window_stack_remove(&summary_window->window, false);
}

static void prv_down_click_handler(ClickRecognizerRef recognizer, void *context) {
  WorkoutSummaryWindow *summary_window = context;

  workout_selection_push(summary_window->select_workout_cb);
}

static void prv_click_config_provider(void *context) {
  window_set_click_context(BUTTON_ID_UP, context);
  window_set_click_context(BUTTON_ID_DOWN, context);
  window_single_click_subscribe(BUTTON_ID_UP, prv_up_click_handler);
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_down_click_handler);
}

static void prv_window_unload_handler(Window *window) {
  WorkoutSummaryWindow *summary_window = window_get_user_data(window);
  if (summary_window) {
    kino_reel_destroy(summary_window->icon);
    gbitmap_destroy(summary_window->action_bar_more);
    gbitmap_destroy(summary_window->action_bar_start);
    action_bar_layer_deinit(&summary_window->action_bar);
    status_bar_layer_deinit(&summary_window->status_layer);
    layer_deinit(&summary_window->base_layer);
    window_deinit(&summary_window->window);
    i18n_free_all(summary_window);
    app_free(summary_window);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//! Public API

WorkoutSummaryWindow *workout_summary_window_create(ActivitySessionType activity_type,
                                                    StartWorkoutCallback start_workout_cb,
                                                    SelectWorkoutCallback select_workout_cb) {
  WorkoutSummaryWindow *summary_window = app_zalloc_check(sizeof(WorkoutSummaryWindow));

  summary_window->start_workout_cb = start_workout_cb;
  summary_window->select_workout_cb = select_workout_cb;

  Window *window = &summary_window->window;
  window_init(window, WINDOW_NAME("Workout Summary"));
  window_set_user_data(window, summary_window);
  window_set_background_color(window, BACKGROUND_COLOR);
  window_set_window_handlers(window, &(WindowHandlers){
    .unload = prv_window_unload_handler,
  });

  GRect layer_bounds = window->layer.bounds;
  layer_bounds.size.w -= ACTION_BAR_WIDTH;

  layer_init(&summary_window->base_layer, &layer_bounds);
  layer_set_update_proc(&summary_window->base_layer, prv_base_layer_update_proc);
  layer_add_child(&window->layer, &summary_window->base_layer);

  StatusBarLayer *status_layer = &summary_window->status_layer;
  status_bar_layer_init(status_layer);
  status_bar_layer_set_colors(status_layer, GColorClear, TEXT_COLOR);
  layer_add_child(&window->layer, status_bar_layer_get_layer(status_layer));

#if PBL_RECT
  GRect status_layer_bounds = window->layer.bounds;
  status_layer_bounds.size.w -= ACTION_BAR_WIDTH;
  layer_set_frame(&status_layer->layer, &status_layer_bounds);
#endif

  ActionBarLayer *action_bar = &summary_window->action_bar;
  action_bar_layer_init(action_bar);
  action_bar_layer_set_context(action_bar, summary_window);
  action_bar_layer_set_click_config_provider(action_bar, prv_click_config_provider);
  action_bar_layer_add_to_window(action_bar, window);

  summary_window->action_bar_start =
      gbitmap_create_with_resource(RESOURCE_ID_ACTION_BAR_ICON_START);
  summary_window->action_bar_more =
      gbitmap_create_with_resource(RESOURCE_ID_ACTION_BAR_ICON_MORE);

  action_bar_layer_set_icon(action_bar, BUTTON_ID_UP, summary_window->action_bar_start);
  action_bar_layer_set_icon(action_bar, BUTTON_ID_DOWN, summary_window->action_bar_more);

  workout_summary_update_activity_type(summary_window, activity_type);

  return summary_window;
}

void workout_summary_window_push(WorkoutSummaryWindow *summary_window) {
  app_window_stack_push(&summary_window->window, true);
}

void workout_summary_update_activity_type(WorkoutSummaryWindow *summary_window,
                                          ActivitySessionType activity_type) {
  summary_window->activity_type = activity_type;
  summary_window->icon = prv_get_icon_for_activity(activity_type);
  summary_window->name = workout_utils_get_name_for_activity(activity_type);
}
