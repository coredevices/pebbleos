/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "active.h"
#include "dialog.h"
#include "summary.h"
#include "workout.h"

#include "applib/app.h"
#include "board/display.h"
#include "applib/ui/action_bar_layer.h"
#include "applib/ui/action_menu_hierarchy.h"
#include "applib/ui/action_menu_window.h"
#include "applib/ui/kino/kino_layer.h"
#include "applib/ui/ui.h"
#include "applib/ui/window_manager.h"
#include "apps/system/timeline/text_node.h"
#include "kernel/pbl_malloc.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/clock.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/activity/activity_private.h"
#include "pbl/services/activity/health_util.h"
#include "pbl/services/activity/hr_util.h"
#include "pbl/services/activity/workout_service.h"
#include <pbl/logging/logging.h>
#include "pbl/util/size.h"

#include <stdio.h>

#define TEXT_COLOR (GColorBlack)
#define TEXT_ALIGNMENT (PBL_IF_RECT_ELSE(GTextAlignmentLeft, GTextAlignmentRight))
#define BACKGROUND_COLOR PBL_IF_COLOR_ELSE(GColorYellow, GColorWhite)

typedef enum WorkoutLayout {
  WorkoutLayout_SingleMetric,
  WorkoutLayout_StaticAndScrollable,
  WorkoutLayout_TwoStaticAndScrollable,
} WorkoutLayout;

typedef struct WorkoutActiveWindow {
  Window window;
  ActionBarLayer action_bar;
  StatusBarLayer status_layer;
  Layer base_layer;
  Layer top_metric_layer;
  Layer middle_metric_layer;
  Layer scrollable_metric_layer;
  WorkoutDialog end_workout_dialog;

  ButtonId pause_button;

  WorkoutController *workout_controller;
  void *workout_data;

  WorkoutLayout layout;

  WorkoutMetricType top_metric;
  WorkoutMetricType middle_metric;

  int num_scrollable_metrics;
  int current_scrollable_metric;
  WorkoutMetricType scrollable_metrics[WorkoutMetricTypeCount];

  GBitmap *heart_icon;
  GBitmap *hr_measuring_icon;

  GBitmap *action_bar_start;
  GBitmap *action_bar_pause;
  GBitmap *action_bar_stop;
  GBitmap *action_bar_more;
  GBitmap *action_bar_next;

  AppTimer *update_timer;
  AppTimer *hr_measuring_timer;

  int cur_hr_measuring_width_idx;
} WorkoutActiveWindow;

static const int s_hr_measuring_widths[] = {36, 0, 24, 28, 32};

static void prv_draw_heart_node_callback(GContext *ctx, const GRect *box,
                                         const GTextNodeDrawConfig *config, bool render,
                                         GSize *size_out, void *user_data);

static void prv_draw_hr_measuring_node_callback(GContext *ctx, const GRect *box,
                                                const GTextNodeDrawConfig *config, bool render,
                                                GSize *size_out, void *user_data);

////////////////////////////////////////////////////////////////////////////////////////////////////
//! Helpers

static void prv_add_scrollable_metrics(WorkoutActiveWindow *active_window,
                                       int num_scrollable_metrics,
                                       WorkoutMetricType *metrics) {
  for (int i = 0; i < num_scrollable_metrics; i++) {
    active_window->scrollable_metrics[active_window->num_scrollable_metrics++] = metrics[i];
  }
}

static const char* prv_get_label_for_hr_metric(int bpm) {
  switch (hr_util_get_hr_zone(bpm)) {
    case HRZone_Zone1:
      /// Zone 1 HR Label
      return i18n_noop("FAT BURN");
    case HRZone_Zone2:
      /// Zone 2 HR Label
      return i18n_noop("ENDURANCE");
    case HRZone_Zone3:
      /// Zone 3 HR Label
      return i18n_noop("PERFORMANCE");
    default:
      /// Default/Zone 0 HR Label
      return i18n_noop("HEART RATE");
  }
}

static const char* prv_get_label_for_metric(WorkoutMetricType metric_type,
                                            WorkoutActiveWindow *active_window) {
  switch (metric_type) {
    case WorkoutMetricType_Hr:
    {
      int bpm = active_window->workout_controller->get_metric_value(WorkoutMetricType_Hr,
                                                                    active_window->workout_data);
      return prv_get_label_for_hr_metric(bpm);
    }
    case WorkoutMetricType_Custom:
      /// Custom Label from Sports App
      return active_window->workout_controller->get_custom_metric_label_string();
    case WorkoutMetricType_Duration:
      /// Duration Label
      return i18n_noop("DURATION");
    case WorkoutMetricType_AvgPace:
#if PBL_RECT
      /// Average Pace Label
      return i18n_noop("AVG PACE");
#else
      /// Average Pace Label with units
      return active_window->workout_controller->get_distance_string(i18n_noop("AVG PACE (/MI)"),
                                                                    i18n_noop("AVG PACE (/KM)"));
#endif
    case WorkoutMetricType_Pace:
#if PBL_RECT
      /// Pace Label
      return i18n_noop("PACE");
#else
      /// Pace Label with units
      return active_window->workout_controller->get_distance_string(i18n_noop("PACE (/MI)"),
                                                                    i18n_noop("PACE (/KM)"));
#endif
    case WorkoutMetricType_Speed:
#if PBL_RECT
      /// Speed Label
      return i18n_noop("SPEED");
#else
      /// Speed Label with units
      return active_window->workout_controller->get_distance_string(i18n_noop("SPEED (MPH)"),
                                                                    i18n_noop("SPEED (KM/H)"));
#endif
    case WorkoutMetricType_Distance:
#if PBL_RECT
      /// Distance Label
      return i18n_noop("DISTANCE");
#else
      /// Distance Label with units
      return active_window->workout_controller->get_distance_string(i18n_noop("DISTANCE (MI)"),
                                                                    i18n_noop("DISTANCE (KM)"));
#endif
    case WorkoutMetricType_Steps:
      /// Steps Label
      return i18n_noop("STEPS");
    default:
      return "";
  }
}

static GColor prv_get_bg_color_for_metric(WorkoutMetricType metric_type,
                                          WorkoutActiveWindow *active_window,
                                          bool is_scrollable) {
#if PBL_BW
  return GColorWhite;
#else
  if (metric_type == WorkoutMetricType_Hr) {
    switch (hr_util_get_hr_zone(active_window->workout_controller->get_metric_value(
        metric_type, active_window->workout_data))) {
      case HRZone_Zone0:
        return GColorWhite;
      case HRZone_Zone1:
        return GColorMelon;
      case HRZone_Zone2:
        return GColorChromeYellow;
      case HRZone_Zone3:
        return GColorOrange;
      default:
        return BACKGROUND_COLOR;
    }
  } else {
    return is_scrollable ? GColorPastelYellow : BACKGROUND_COLOR;
  }
#endif
}

static GFont prv_get_number_font(bool prefer_larger_font) {
#if PBL_DISPLAY_HEIGHT >= 200
  return prefer_larger_font ? fonts_get_system_font(FONT_KEY_LECO_60_NUMBERS_AM_PM)
                            : fonts_get_system_font(FONT_KEY_LECO_42_NUMBERS);
#else
  return prefer_larger_font ? fonts_get_system_font(FONT_KEY_LECO_38_BOLD_NUMBERS)
                            : fonts_get_system_font(FONT_KEY_LECO_26_BOLD_NUMBERS_AM_PM);
#endif
}

//! Font for durations that include hours ("HH:MM:SS"). On a narrow rectangular
//! display the regular reduced number font (LECO_42) is too wide for 8 chars and
//! the seconds get clipped, so use a narrower font there.
static GFont prv_get_duration_font(void) {
#if PBL_DISPLAY_HEIGHT >= 200 && PBL_RECT
  return fonts_get_system_font(FONT_KEY_LECO_36_BOLD_NUMBERS);
#else
  return prv_get_number_font(false);
#endif
}

static GTextNode* prv_create_text_node(WorkoutActiveWindow *active_window,
                                       WorkoutMetricType metric_type,
                                       bool prefer_larger_font,
                                       void *i18n_owner) {
  GTextNodeHorizontal *horiz_container = graphics_text_node_create_horizontal(MAX_TEXT_NODES);
  GTextNodeContainer *container = &horiz_container->container;
  horiz_container->horizontal_alignment = TEXT_ALIGNMENT;

  const GFont number_font = prv_get_number_font(prefer_larger_font);
  const GFont units_font = fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD);

  const int units_offset_y = fonts_get_font_height(number_font) - fonts_get_font_height(units_font);

  switch (metric_type) {
    case WorkoutMetricType_Hr: {
      GPoint heart_node_offset = GPoint(2, prefer_larger_font ? 5 : 0);
      GTextNodeCustom *heart_node;
      if (active_window->workout_controller->get_metric_value(
          metric_type, active_window->workout_data) > 0) {
        const size_t buffer_size = sizeof("000");
        GTextNodeText *number_text_node = health_util_create_text_node(buffer_size, number_font,
                                                                       TEXT_COLOR, container);
        active_window->workout_controller->metric_to_string(metric_type,
                                                            (char *) number_text_node->text,
                                                            buffer_size, i18n_owner,
                                                            active_window->workout_data);
        heart_node_offset.y += fonts_get_font_cap_offset(number_font);
        heart_node = graphics_text_node_create_custom(prv_draw_heart_node_callback,
                                                      active_window);
      } else {
        // if metric value is 0, we draw another icon that needs different offset
        heart_node_offset.x += 2;
        heart_node_offset.y += 7;
        heart_node = graphics_text_node_create_custom(prv_draw_hr_measuring_node_callback,
                                                      active_window);
      }
      heart_node->node.offset = heart_node_offset;
      graphics_text_node_container_add_child(container, &heart_node->node);
      break;
    }
    case WorkoutMetricType_Steps: {
      const size_t buffer_size = sizeof("000000");
      GTextNodeText *number_text_node = health_util_create_text_node(buffer_size, number_font,
                                                                     TEXT_COLOR, container);
      active_window->workout_controller->metric_to_string(metric_type,
                                                          (char *) number_text_node->text,
                                                          buffer_size, i18n_owner,
                                                          active_window->workout_data);
      break;
    }
    case WorkoutMetricType_Distance: {
      GTextNodeText *number_text_node = health_util_create_text_node(
          HEALTH_WHOLE_AND_DECIMAL_LENGTH, number_font, TEXT_COLOR, container);
      active_window->workout_controller->metric_to_string(metric_type,
                                                          (char *) number_text_node->text,
                                                          HEALTH_WHOLE_AND_DECIMAL_LENGTH,
                                                          i18n_owner,
                                                          active_window->workout_data);

#if PBL_RECT
      /// MI/KM units string
      const char *units_string = active_window->workout_controller->get_distance_string(
          i18n_noop("MI"), i18n_noop("KM"));
      GTextNodeText *units_text_node = health_util_create_text_node_with_text(
          i18n_get(units_string, i18n_owner), units_font, TEXT_COLOR, container);
      units_text_node->node.offset.y = units_offset_y;
#endif
      break;
    }
    case WorkoutMetricType_Custom:
    {
      const size_t buffer_size = 20;
      GTextNodeText *number_text_node = health_util_create_text_node(buffer_size, number_font,
                                                                     TEXT_COLOR, container);
      number_text_node->overflow = GTextOverflowModeTrailingEllipsis;
      active_window->workout_controller->metric_to_string(metric_type,
                                                          (char *) number_text_node->text,
                                                          buffer_size, i18n_owner,
                                                          active_window->workout_data);
      if (strlen(number_text_node->text) > 5) {
        number_text_node->font = prv_get_number_font(false);
      }
      break;
    }
    case WorkoutMetricType_Duration:
    {
      const size_t buffer_size = sizeof("00:00:00");
      GTextNodeText *number_text_node = health_util_create_text_node(buffer_size, number_font,
                                                                     TEXT_COLOR, container);
      active_window->workout_controller->metric_to_string(metric_type,
          (char *)number_text_node->text, buffer_size, i18n_owner, active_window->workout_data);

      if (strlen(number_text_node->text) > 5) {
        // text is long (includes hours) so use a font that fits the seconds
        number_text_node->font = prv_get_duration_font();
      }
      break;
    }
    case WorkoutMetricType_Pace:
    case WorkoutMetricType_AvgPace:
    {
      if (active_window->workout_controller->get_metric_value(
          metric_type, active_window->workout_data) >= SECONDS_PER_HOUR) {
         GTextNodeText *text_node =
            health_util_create_text_node_with_text(EM_DASH, units_font, TEXT_COLOR, container);
            text_node->node.offset.x += 1;
            text_node->node.offset.y = units_offset_y;
       } else {
        const size_t buffer_size = sizeof("00:00:00");
        GTextNodeText *number_text_node = health_util_create_text_node(buffer_size, number_font,
                                                                       TEXT_COLOR, container);
        active_window->workout_controller->metric_to_string(metric_type,
            (char *)number_text_node->text, buffer_size, i18n_owner, active_window->workout_data);

#if PBL_RECT
        GTextNodeText *divider_text_node = health_util_create_text_node_with_text(
            "/", units_font, TEXT_COLOR, container);
        divider_text_node->node.offset.y = units_offset_y;

        /// MI/KM units string
        const char *units_string =
            active_window->workout_controller->get_distance_string(i18n_noop("MI"),
                                                                   i18n_noop("KM"));
        GTextNodeText *units_text_node = health_util_create_text_node_with_text(
            i18n_get(units_string, i18n_owner), units_font, TEXT_COLOR, container);
        units_text_node->node.offset.y = units_offset_y;
#endif
      }
      break;
    }
    case WorkoutMetricType_Speed:
    {
      const size_t buffer_size = sizeof("00:00:00");
      GTextNodeText *number_text_node = health_util_create_text_node(buffer_size, number_font,
                                                                     TEXT_COLOR, container);
      active_window->workout_controller->metric_to_string(metric_type,
          (char *)number_text_node->text, buffer_size, i18n_owner, active_window->workout_data);

#if PBL_RECT
      /// MI/KM units string
      const char *units_string =
          active_window->workout_controller->get_distance_string(i18n_noop("MPH"),
                                                                 i18n_noop("KM/H"));
      GTextNodeText *units_text_node = health_util_create_text_node_with_text(
          i18n_get(units_string, i18n_owner), units_font, TEXT_COLOR, container);
      units_text_node->node.offset.y = units_offset_y;
#endif
      break;
    }
    // don't have default here so when we have a new type, we don't forget to add it here
    case WorkoutMetricType_None:
    case WorkoutMetricTypeCount:
      break;
  }

  return &container->node;
}

static void prv_set_action_bar_icons(WorkoutActiveWindow *active_window) {
  ActionBarLayer *action_bar = &active_window->action_bar;
  bool is_paused = false;
  bool can_stop = false;
  if (active_window->workout_controller) {
    is_paused = active_window->workout_controller->is_paused();
    can_stop = active_window->workout_controller->stop != NULL;
  }

  if (is_paused) {
    action_bar_layer_set_icon(action_bar, active_window->pause_button,
                              active_window->action_bar_start);
    if (can_stop) {
      action_bar_layer_set_icon(action_bar, BUTTON_ID_SELECT, active_window->action_bar_stop);
    }
  } else {
    action_bar_layer_clear_icon(action_bar, BUTTON_ID_SELECT);
    action_bar_layer_set_icon(action_bar, active_window->pause_button,
                              active_window->action_bar_pause);
  }

  if (active_window->num_scrollable_metrics > 1) {
    action_bar_layer_set_icon(action_bar, BUTTON_ID_DOWN, active_window->action_bar_next);
  }
}

static void prv_update_ui(WorkoutActiveWindow *active_window) {
  if (window_manager_is_window_visible(&active_window->window)) {
    layer_mark_dirty(&active_window->base_layer);

    // Update the action bar in case another client updated the workout's status
    prv_set_action_bar_icons(active_window);
  }
}

static void prv_hr_measuring_timer_callback(void *data) {
  WorkoutActiveWindow *active_window = data;

  active_window->cur_hr_measuring_width_idx = (active_window->cur_hr_measuring_width_idx + 1)
                                              % ARRAY_LENGTH(s_hr_measuring_widths);

  prv_update_ui(active_window);

  if (active_window->workout_controller->get_metric_value(
      WorkoutMetricType_Hr, active_window->workout_data) == 0) {
    int timeout_ms = (active_window->cur_hr_measuring_width_idx == 0) ? 800 : 200;
    active_window->hr_measuring_timer =
        app_timer_register(timeout_ms, prv_hr_measuring_timer_callback, active_window);
  } else {
    active_window->hr_measuring_timer = NULL;
  }
}

static void prv_update_timer_callback(void *data) {
  WorkoutActiveWindow *active_window = data;

  if (active_window->workout_controller) {
    active_window->workout_controller->update_data(active_window->workout_data);
  }

  prv_update_ui(active_window);
  active_window->update_timer = app_timer_register(1000, prv_update_timer_callback, active_window);

  const int bpm = active_window->workout_controller->get_metric_value(
      WorkoutMetricType_Hr, active_window->workout_data);
  if (bpm == 0 && !active_window->hr_measuring_timer) {
    active_window->cur_hr_measuring_width_idx = 0;
    prv_hr_measuring_timer_callback(active_window);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//! Drawing

static void prv_draw_heart_icon(GContext *ctx, GBitmap *icon, const GRect *rect,
                                bool render, GSize *size_out) {
  if (render) {
    graphics_context_set_compositing_mode(ctx, GCompOpSet);
    graphics_draw_bitmap_in_rect(ctx, icon, rect);
  }
  if (size_out) {
    *size_out = rect->size;
  }
}

static void prv_draw_heart_node_callback(GContext *ctx, const GRect *box,
                                         const GTextNodeDrawConfig *config, bool render,
                                         GSize *size_out, void *user_data) {
  WorkoutActiveWindow *active_window = user_data;
  GRect bounds = gbitmap_get_bounds(active_window->heart_icon);
  bounds.origin = box->origin;
  prv_draw_heart_icon(ctx, active_window->heart_icon, &bounds, render, size_out);
}

static void prv_draw_hr_measuring_node_callback(GContext *ctx, const GRect *box,
                                                const GTextNodeDrawConfig *config, bool render,
                                                GSize *size_out, void *user_data) {
  WorkoutActiveWindow *active_window = user_data;
  GRect bounds = gbitmap_get_bounds(active_window->hr_measuring_icon);
  bounds.origin = box->origin;
  bounds.size.w = s_hr_measuring_widths[active_window->cur_hr_measuring_width_idx];
  prv_draw_heart_icon(ctx, active_window->hr_measuring_icon, &bounds, render, size_out);
}

static void prv_render_separator(GContext *ctx, Layer *layer) {
  graphics_context_set_stroke_color(ctx, GColorBlack);
  graphics_draw_horizontal_line_dotted(ctx, GPoint(0, layer->bounds.size.h - 1),
                                       layer->bounds.size.w);
}

static void prv_render_bg_color(GContext *ctx, GRect *bounds, GColor color) {
  graphics_context_set_fill_color(ctx, color);
  graphics_fill_rect(ctx, bounds);
}

static void prv_render_metric_label(GContext *ctx, GRect *box, WorkoutMetricType metric_type,
                                    WorkoutActiveWindow *active_window, void *i18n_owner) {
  GRect label_box = *box;
  GTextOverflowMode overflow_mode = GTextOverflowModeWordWrap;
  if (metric_type == WorkoutMetricType_Custom) {
    // I seriously have no idea why the height is hardcoded to 40 and overflow is set to word
    // wrap when there's a note that says the height is being set to 40 to avoid wrapping. Also,
    // with a font size of 18, I don't know how it wouldn't wrap. This fixes the inconsistent
    // magic number problem for the WorkoutMetricType_Custom only
    label_box.size.h = 20;
    overflow_mode = GTextOverflowModeTrailingEllipsis;
  }


  graphics_context_set_text_color(ctx, TEXT_COLOR);
  graphics_draw_text(ctx,
                     i18n_get(prv_get_label_for_metric(metric_type, active_window), i18n_owner),
                     fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD),
                     label_box,
                     overflow_mode,
                     TEXT_ALIGNMENT,
                     NULL);
}

static void prv_render_hr_zones(GContext *ctx, GRect *box, WorkoutActiveWindow *active_window) {
  graphics_context_set_stroke_color(ctx, GColorBlack);
  graphics_context_set_fill_color(ctx, GColorBlack);

  GRect zone_rect = *box;
  zone_rect.origin.x += PBL_IF_RECT_ELSE(1, 69);
  // add some padding after the label
  zone_rect.origin.y += 10;
  // size of a zone rect
  zone_rect.size = GSize(20, 8);

  const int zone_padding = 2;

  for (HRZone i = HRZone_Zone1; i < HRZoneCount; i++) {
    if (i <= hr_util_get_hr_zone(active_window->workout_controller->get_metric_value(
        WorkoutMetricType_Hr, active_window->workout_data))) {
      graphics_fill_rect(ctx, &zone_rect);
    } else {
      // drawing it twice to draw a 2px border
      GRect inner_rect = grect_inset(zone_rect, GEdgeInsets(1));
      graphics_draw_rect(ctx, &zone_rect);
      graphics_draw_rect(ctx, &inner_rect);
    }
    // increment x to draw more zones
    zone_rect.origin.x += zone_rect.size.w + zone_padding;
  }
}

static void prv_render_metric(GContext *ctx, WorkoutMetricType metric_type, Layer *layer,
                              GColor bg_color, bool draw_hr_zones, bool prefer_larger_font) {
  WorkoutActiveWindow *active_window = window_get_user_data(layer_get_window(layer));

  prv_render_bg_color(ctx, &layer->bounds, bg_color);

  const int16_t rl_margin = PBL_IF_RECT_ELSE(5, 23);

  GRect rect = grect_inset(layer->bounds, GEdgeInsets(0, rl_margin));

  // set rect y depending on layout, primary metric and display shape
  if (active_window->layout == WorkoutLayout_SingleMetric) {
    rect.origin.y = PBL_IF_RECT_ELSE(35, 41);
  } else if (active_window->layout == WorkoutLayout_StaticAndScrollable) {
    rect.origin.y = prefer_larger_font ? PBL_IF_RECT_ELSE(2, 13) : PBL_IF_RECT_ELSE(5, 1);
  } else if (active_window->layout == WorkoutLayout_TwoStaticAndScrollable) {
    rect.origin.y = (&active_window->scrollable_metric_layer == layer) ?
        PBL_IF_RECT_ELSE(-2, 0) : PBL_IF_RECT_ELSE(-4, -2);
  }

  // set the rect height so we don't wrap text to the next line
  rect.size.h = 40;

#if PBL_ROUND
  if (draw_hr_zones) {
    // padding between text and zones is less on round
    rect.origin.y -= 10;
  }
  rect.origin.x -= 24;
#endif

  prv_render_metric_label(ctx, &rect, metric_type, active_window, layer);

  // update rect y for the label height
  if (active_window->layout == WorkoutLayout_TwoStaticAndScrollable) {
    rect.origin.y += prefer_larger_font ? 11 : 15;
  } else {
    rect.origin.y += prefer_larger_font ? 12 : 15;
  }

  if (draw_hr_zones) {
    prv_render_hr_zones(ctx, &rect, active_window);
    // update rect y for the zones height
    rect.origin.y += PBL_IF_RECT_ELSE(18, 15);
  }

  // adjust rect for drawing the text node
  rect.origin.x -= PBL_IF_RECT_ELSE(1, 46);
  rect.size.w += (rl_margin * 2);

  GTextNode *text_node = prv_create_text_node(active_window, metric_type,
                                              prefer_larger_font, layer);
  graphics_text_node_draw(text_node, ctx, &rect, NULL, NULL);
  graphics_text_node_destroy(text_node);
}

static void prv_static_layer_update_proc(struct Layer *layer, GContext *ctx) {
  WorkoutActiveWindow *active_window = window_get_user_data(layer_get_window(layer));

  WorkoutMetricType metric_type = WorkoutMetricType_None;
  if (layer == &active_window->top_metric_layer) {
    metric_type = active_window->top_metric;
  } else if (layer == &active_window->middle_metric_layer) {
    metric_type = active_window->middle_metric;
  }

  GColor bg_color = prv_get_bg_color_for_metric(metric_type, active_window, false);

  HRZone hr_zone = hr_util_get_hr_zone(active_window->workout_controller->get_metric_value(
      metric_type, active_window->workout_data));
  const bool draw_zones = (metric_type == WorkoutMetricType_Hr) && hr_zone > HRZone_Zone0;
  const bool prefer_larger_font = active_window->layout == WorkoutLayout_SingleMetric ||
                                  active_window->layout == WorkoutLayout_StaticAndScrollable;

  prv_render_metric(ctx, metric_type, layer, bg_color, draw_zones, prefer_larger_font);

  if (layer == &active_window->top_metric_layer) {
    status_bar_layer_set_colors(&active_window->status_layer, bg_color, GColorBlack);
  }

  if (active_window->layout == WorkoutLayout_StaticAndScrollable ||
      (active_window->layout == WorkoutLayout_TwoStaticAndScrollable &&
       layer == &active_window->middle_metric_layer)) {
    prv_render_separator(ctx, layer);
  }
}

static void prv_scrollable_layer_update_proc(struct Layer *layer, GContext *ctx) {
  WorkoutActiveWindow *active_window = window_get_user_data(layer_get_window(layer));

  if (!active_window->num_scrollable_metrics) {
    return;
  }

  WorkoutMetricType metric_type =
      active_window->scrollable_metrics[active_window->current_scrollable_metric];

  GColor bg_color = prv_get_bg_color_for_metric(metric_type, active_window, true);

  const bool draw_hr_zones = false;
  const bool prefer_larger_font = false;
  prv_render_metric(ctx, metric_type, layer, bg_color, draw_hr_zones, prefer_larger_font);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//! End Workout

static void prv_end_workout_up_click_handler(ClickRecognizerRef recognizer, void *context) {
  WorkoutActiveWindow *active_window = context;

  if (active_window->workout_controller) {
    active_window->workout_controller->stop();
  }

  workout_push_summary_window();

  workout_dialog_pop(&active_window->end_workout_dialog);
  app_window_stack_remove(&active_window->window, false);
}

static void prv_end_workout_down_click_handler(ClickRecognizerRef recognizer, void *context) {
  WorkoutActiveWindow *active_window = context;

  workout_dialog_pop(&active_window->end_workout_dialog);
}

static void prv_end_workout_click_config_provider(void *context) {
  window_single_click_subscribe(BUTTON_ID_UP, prv_end_workout_up_click_handler);
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_end_workout_down_click_handler);
}

static void prv_end_workout(void *context) {
  WorkoutActiveWindow *active_window = context;

  WorkoutDialog *workout_dialog = &active_window->end_workout_dialog;

  workout_dialog_init(workout_dialog, "Workout End");
  Dialog *dialog = workout_dialog_get_dialog(workout_dialog);

  dialog_show_status_bar_layer(dialog, true);
  dialog_set_fullscreen(dialog, true);
  dialog_set_text(dialog, i18n_get("End Workout?", workout_dialog));
  dialog_set_background_color(dialog, BACKGROUND_COLOR);
  dialog_set_text_color(dialog, TEXT_COLOR);
  dialog_set_icon(dialog, RESOURCE_ID_WORKOUT_APP_END);
  dialog_set_icon_animate_direction(dialog, DialogIconAnimateNone);
  dialog_set_destroy_on_pop(dialog, false);

  i18n_free_all(workout_dialog);

  workout_dialog_set_click_config_provider(workout_dialog, prv_end_workout_click_config_provider);
  workout_dialog_set_click_config_context(workout_dialog, context);

  app_workout_dialog_push(workout_dialog);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//! Handlers

static void prv_handle_pause_button(WorkoutActiveWindow *active_window) {
  bool is_paused = false;
  if (active_window->workout_controller) {
    is_paused = active_window->workout_controller->is_paused();
  }

  if (active_window->workout_controller) {
     active_window->workout_controller->pause(!is_paused);
  }

  prv_update_ui(active_window);
}

static void prv_handle_stop_button(WorkoutActiveWindow *active_window) {
  bool is_paused = false;
  bool can_stop = false;
  if (active_window->workout_controller) {
    is_paused = active_window->workout_controller->is_paused();
    can_stop = active_window->workout_controller->stop != NULL;
  }

  if (!is_paused || !can_stop) {
    return;
  }

  prv_end_workout(active_window);
}

static void prv_up_click_handler(ClickRecognizerRef recognizer, void *context) {
  WorkoutActiveWindow *active_window = context;

  if (active_window->pause_button == BUTTON_ID_UP) {
    prv_handle_pause_button(active_window);
  }
}

static void prv_select_click_handler(ClickRecognizerRef recognizer, void *context) {
  WorkoutActiveWindow *active_window = context;

  if (active_window->pause_button == BUTTON_ID_SELECT) {
    prv_handle_pause_button(active_window);
  } else {
    prv_handle_stop_button(active_window);
  }
}

static void prv_set_pause_button(WorkoutActiveWindow *active_window) {
  bool can_stop = active_window->workout_controller->stop != NULL;
  if (can_stop || active_window->num_scrollable_metrics > 1) {
    active_window->pause_button = BUTTON_ID_UP;
  } else {
    active_window->pause_button = BUTTON_ID_SELECT;
  }
}

T_STATIC void prv_cycle_scrollable_metrics(WorkoutActiveWindow *active_window) {
  active_window->current_scrollable_metric =
      (active_window->current_scrollable_metric + 1) % active_window->num_scrollable_metrics;
}

static void prv_down_click_handler(ClickRecognizerRef recognizer, void *context) {
  WorkoutActiveWindow *active_window = context;
  prv_cycle_scrollable_metrics(active_window);
}

static void prv_click_config_provider(void *context) {
  window_set_click_context(BUTTON_ID_UP, context);
  window_set_click_context(BUTTON_ID_SELECT, context);
  window_set_click_context(BUTTON_ID_DOWN, context);
  window_single_click_subscribe(BUTTON_ID_UP, prv_up_click_handler);
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_select_click_handler);
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_down_click_handler);
}

static void prv_window_unload_handler(Window *window) {
  WorkoutActiveWindow *active_window = window_get_user_data(window);
  if (active_window) {
    app_timer_cancel(active_window->update_timer);
    app_timer_cancel(active_window->hr_measuring_timer);
    gbitmap_destroy(active_window->action_bar_start);
    gbitmap_destroy(active_window->action_bar_pause);
    gbitmap_destroy(active_window->action_bar_stop);
    gbitmap_destroy(active_window->action_bar_more);
    gbitmap_destroy(active_window->action_bar_next);
    gbitmap_destroy(active_window->heart_icon);
    gbitmap_destroy(active_window->hr_measuring_icon);
    action_bar_layer_deinit(&active_window->action_bar);
    status_bar_layer_deinit(&active_window->status_layer);
    layer_deinit(&active_window->top_metric_layer);
    layer_deinit(&active_window->scrollable_metric_layer);
    layer_deinit(&active_window->base_layer);
    window_deinit(&active_window->window);
    i18n_free_all(active_window);
    app_free(active_window);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//! Common Setup
static void prv_create_window_common(WorkoutActiveWindow *active_window,
                                     void *workout_data,
                                     WorkoutController *workout_controller) {
  active_window->workout_data = workout_data;
  active_window->workout_controller = workout_controller;

  Window *window = &active_window->window;
  window_init(window, WINDOW_NAME("Workout Active Info"));
  window_set_user_data(window, active_window);
  window_set_background_color(window, BACKGROUND_COLOR);
  window_set_window_handlers(window, &(WindowHandlers){
    .unload = prv_window_unload_handler,
  });

  GRect base_layer_bounds = window->layer.bounds;
#if PBL_RECT
  base_layer_bounds = action_bar_layer_inset_bounds(base_layer_bounds);
#endif

  base_layer_bounds.origin.y = STATUS_BAR_LAYER_HEIGHT;
  layer_init(&active_window->base_layer, &base_layer_bounds);
  layer_add_child(&window->layer, &active_window->base_layer);
  base_layer_bounds.origin.y = 0;

  if (active_window->layout == WorkoutLayout_SingleMetric) {
    // Only 1 metric to show. It can have the whole screen
    GRect metric_bounds = base_layer_bounds;
    layer_init(&active_window->top_metric_layer, &metric_bounds);
    layer_set_update_proc(&active_window->top_metric_layer, prv_static_layer_update_proc);
    layer_add_child(&active_window->base_layer, &active_window->top_metric_layer);
  } else if (active_window->layout == WorkoutLayout_StaticAndScrollable) {
    // Two metrics. 1 big static metric above a smaller scrollable metric
    GRect top_metric_bounds = base_layer_bounds;
#if PBL_DISPLAY_HEIGHT >= 200
    top_metric_bounds.size.h = PBL_IF_RECT_ELSE(105, 115);
#else
    top_metric_bounds.size.h = PBL_IF_RECT_ELSE(90, 77);
#endif
    layer_init(&active_window->top_metric_layer, &top_metric_bounds);
    layer_set_update_proc(&active_window->top_metric_layer, prv_static_layer_update_proc);
    layer_add_child(&active_window->base_layer, &active_window->top_metric_layer);

    GRect scrollable_metric_bounds = top_metric_bounds;
    scrollable_metric_bounds.origin.y = scrollable_metric_bounds.size.h;
    scrollable_metric_bounds.size.h = window->layer.bounds.size.h -
                                      scrollable_metric_bounds.origin.y;
    layer_init(&active_window->scrollable_metric_layer, &scrollable_metric_bounds);
    layer_set_update_proc(&active_window->scrollable_metric_layer,
                          prv_scrollable_layer_update_proc);
    layer_add_child(&active_window->base_layer, &active_window->scrollable_metric_layer);
  } else if (active_window->layout == WorkoutLayout_TwoStaticAndScrollable) {
    // Three metrics. Two static metrics above a scrollable metric
#if PBL_DISPLAY_HEIGHT >= 200
    const int layer_height = 68;
#else
    const int layer_height = 51;
#endif
    GRect top_metric_bounds = base_layer_bounds;
    top_metric_bounds.size.h = layer_height;
    layer_init(&active_window->top_metric_layer, &top_metric_bounds);
    layer_set_update_proc(&active_window->top_metric_layer, prv_static_layer_update_proc);
    layer_add_child(&active_window->base_layer, &active_window->top_metric_layer);

    GRect middle_metric_bounds = top_metric_bounds;
    middle_metric_bounds.origin.y = top_metric_bounds.size.h;
    middle_metric_bounds.size.h = layer_height - 2;
    layer_init(&active_window->middle_metric_layer, &middle_metric_bounds);
    layer_set_update_proc(&active_window->middle_metric_layer, prv_static_layer_update_proc);
    layer_add_child(&active_window->base_layer, &active_window->middle_metric_layer);

    GRect scrollable_metric_bounds = middle_metric_bounds;
    scrollable_metric_bounds.origin.y = top_metric_bounds.size.h + middle_metric_bounds.size.h;
    scrollable_metric_bounds.size.h = layer_height + 10;
    layer_init(&active_window->scrollable_metric_layer, &scrollable_metric_bounds);
    layer_set_update_proc(&active_window->scrollable_metric_layer,
                          prv_scrollable_layer_update_proc);
    layer_add_child(&active_window->base_layer, &active_window->scrollable_metric_layer);
  }

  StatusBarLayer *status_layer = &active_window->status_layer;
  status_bar_layer_init(status_layer);
  status_bar_layer_set_colors(status_layer, GColorClear, GColorBlack);
  layer_add_child(&window->layer, status_bar_layer_get_layer(status_layer));

#if PBL_RECT
  GRect status_layer_bounds = action_bar_layer_inset_bounds(window->layer.bounds);
  layer_set_frame(&status_layer->layer, &status_layer_bounds);
#endif

  ActionBarLayer *action_bar = &active_window->action_bar;
  action_bar_layer_init(action_bar);
  action_bar_layer_set_context(action_bar, active_window);
  action_bar_layer_set_click_config_provider(action_bar, prv_click_config_provider);
  action_bar_layer_add_to_window(action_bar, window);

  active_window->heart_icon = gbitmap_create_with_resource(RESOURCE_ID_WORKOUT_APP_HEART),
  active_window->hr_measuring_icon =
      gbitmap_create_with_resource(RESOURCE_ID_WORKOUT_APP_MEASURING_HR),

  active_window->action_bar_start =
      gbitmap_create_with_resource(RESOURCE_ID_ACTION_BAR_ICON_START);
  active_window->action_bar_pause =
      gbitmap_create_with_resource(RESOURCE_ID_ACTION_BAR_ICON_PAUSE);
  active_window->action_bar_stop =
      gbitmap_create_with_resource(RESOURCE_ID_ACTION_BAR_ICON_STOP);
  active_window->action_bar_more =
      gbitmap_create_with_resource(RESOURCE_ID_ACTION_BAR_ICON_MORE);
  active_window->action_bar_next =
      gbitmap_create_with_resource(RESOURCE_ID_ACTION_BAR_ICON_TOGGLE);

  prv_set_pause_button(active_window);
  prv_set_action_bar_icons(active_window);

  active_window->update_timer = app_timer_register(1000, prv_update_timer_callback, active_window);
}


////////////////////////////////////////////////////////////////////////////////////////////////////
//! Public API

WorkoutActiveWindow *workout_active_create_single_layout(WorkoutMetricType metric,
                                                         void *workout_data,
                                                         WorkoutController *workout_controller) {
  if (metric == WorkoutMetricType_None) {
    PBL_LOG_ERR("Invalid argument");
    return NULL;
  }

  WorkoutActiveWindow *active_window = app_zalloc_check(sizeof(WorkoutActiveWindow));
  active_window->layout = WorkoutLayout_SingleMetric;

  active_window->top_metric = metric;

  prv_create_window_common(active_window, workout_data, workout_controller);

  return active_window;
}

WorkoutActiveWindow *workout_active_create_double_layout(WorkoutMetricType top_metric,
                                                         int num_scrollable_metrics,
                                                         WorkoutMetricType *scrollable_metrics,
                                                         void *workout_data,
                                                         WorkoutController *workout_controller) {
  if (top_metric == WorkoutMetricType_None || num_scrollable_metrics == 0 || !scrollable_metrics) {
    PBL_LOG_ERR("Invalid argument(s)");
    return NULL;
  }

  WorkoutActiveWindow *active_window = app_zalloc_check(sizeof(WorkoutActiveWindow));
  active_window->layout = WorkoutLayout_StaticAndScrollable;

  active_window->top_metric = top_metric;
  prv_add_scrollable_metrics(active_window, num_scrollable_metrics, scrollable_metrics);

  prv_create_window_common(active_window, workout_data, workout_controller);

  return active_window;
}

WorkoutActiveWindow *workout_active_create_tripple_layout(WorkoutMetricType top_metric,
                                                          WorkoutMetricType middle_metric,
                                                          int num_scrollable_metrics,
                                                          WorkoutMetricType *scrollable_metrics,
                                                          void *workout_data,
                                                          WorkoutController *workout_controller) {
  if (top_metric == WorkoutMetricType_None || middle_metric == WorkoutMetricType_None ||
      (num_scrollable_metrics != 0 && !scrollable_metrics)) {
    PBL_LOG_ERR("Invalid argument(s)");
    return NULL;
  }

  WorkoutActiveWindow *active_window = app_zalloc_check(sizeof(WorkoutActiveWindow));
  active_window->layout = WorkoutLayout_TwoStaticAndScrollable;

  active_window->top_metric = top_metric;
  active_window->middle_metric = middle_metric;
  prv_add_scrollable_metrics(active_window, num_scrollable_metrics, scrollable_metrics);

  prv_create_window_common(active_window, workout_data, workout_controller);

  return active_window;
}

WorkoutActiveWindow *workout_active_create_for_activity_type(ActivitySessionType type,
    void *workout_data, WorkoutController *workout_controller) {
  const bool hrm_is_available = activity_is_hrm_present() && activity_prefs_heart_rate_is_enabled();

  switch (type) {
    case ActivitySessionType_Open:
    {
      if (hrm_is_available) {
        WorkoutMetricType scrollable_metrics[] = {WorkoutMetricType_Duration};
        return workout_active_create_double_layout(WorkoutMetricType_Hr,
                                                   ARRAY_LENGTH(scrollable_metrics),
                                                   scrollable_metrics,
                                                   workout_data,
                                                   workout_controller);
      } else {
        return workout_active_create_single_layout(WorkoutMetricType_Duration,
                                                   workout_data,
                                                   workout_controller);
      }
    }
    case ActivitySessionType_Walk:
    {
      if (hrm_is_available) {
        WorkoutMetricType top_metric = WorkoutMetricType_Hr;
        WorkoutMetricType scrollable_metrics[] = {WorkoutMetricType_Duration,
                                                  WorkoutMetricType_Distance,
                                                  WorkoutMetricType_AvgPace,
                                                  WorkoutMetricType_Steps};
        return workout_active_create_double_layout(top_metric,
                                                   ARRAY_LENGTH(scrollable_metrics),
                                                   scrollable_metrics,
                                                   workout_data,
                                                   workout_controller);
      } else {
        WorkoutMetricType top_metric = WorkoutMetricType_Duration;
        WorkoutMetricType scrollable_metrics[] = {WorkoutMetricType_Distance,
                                                  WorkoutMetricType_AvgPace,
                                                  WorkoutMetricType_Steps};
        return workout_active_create_double_layout(top_metric,
                                                   ARRAY_LENGTH(scrollable_metrics),
                                                   scrollable_metrics,
                                                   workout_data,
                                                   workout_controller);
      }
    }
    case ActivitySessionType_Run:
    {
      if (hrm_is_available) {
        WorkoutMetricType top_metric = WorkoutMetricType_Hr;
        WorkoutMetricType scrollable_metrics[] = {WorkoutMetricType_Duration,
                                                  WorkoutMetricType_AvgPace,
                                                  WorkoutMetricType_Distance};
        return workout_active_create_double_layout(top_metric,
                                                   ARRAY_LENGTH(scrollable_metrics),
                                                   scrollable_metrics,
                                                   workout_data,
                                                   workout_controller);
      } else {
        WorkoutMetricType top_metric = WorkoutMetricType_Duration;
        WorkoutMetricType scrollable_metrics[] = {WorkoutMetricType_AvgPace,
                                                  WorkoutMetricType_Distance};
        return workout_active_create_double_layout(top_metric,
                                                   ARRAY_LENGTH(scrollable_metrics),
                                                   scrollable_metrics,
                                                   workout_data,
                                                   workout_controller);
      }
    }
    default:
      return NULL;
  }
}

void workout_active_window_push(WorkoutActiveWindow *active_window) {
  app_window_stack_push(&active_window->window, true);
}

void workout_active_update_scrollable_metrics(WorkoutActiveWindow *active_window,
                                              int num_scrollable_metrics,
                                              WorkoutMetricType *scrollable_metrics) {
  active_window->num_scrollable_metrics = 0;
  prv_add_scrollable_metrics(active_window, num_scrollable_metrics, scrollable_metrics);

  prv_set_pause_button(active_window);

  if (active_window->current_scrollable_metric >= active_window->num_scrollable_metrics) {
    active_window->current_scrollable_metric = 0;
  }

  prv_update_ui(active_window);
}
