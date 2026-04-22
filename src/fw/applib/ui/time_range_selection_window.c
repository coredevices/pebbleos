/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "time_range_selection_window.h"
#include "date_time_selection_window_private.h"

#include "process_management/process_manager.h"
#include "pbl/services/clock.h"
#include "pbl/services/i18n/i18n.h"
#include "shell/system_theme.h"

#include <stdio.h>

typedef struct TimeSelectionSizeConfig {
  int16_t cell_width;
  int16_t cell_padding;

  int16_t top_origin;
  int16_t start_end_y_offset;
  int16_t selection_y_offset;
} TimeSelectionSizeConfig;

static const TimeSelectionSizeConfig s_time_selection_config_medium = {
  .cell_width = 40,
  .cell_padding = 4,

  .top_origin = 10,
  .start_end_y_offset = 69,
  .selection_y_offset = 32,
};

static const TimeSelectionSizeConfig s_time_selection_config_large = {
  .cell_width = 56,
  .cell_padding = 6,

  .top_origin = 11,
  .start_end_y_offset = 105,
  .selection_y_offset = 37,
};

static const TimeSelectionSizeConfig *s_time_selection_configs[NumPreferredContentSizes] = {
  [PreferredContentSizeSmall] = &s_time_selection_config_medium,
  [PreferredContentSizeMedium] = &s_time_selection_config_medium,
  [PreferredContentSizeLarge] = &s_time_selection_config_large,
  [PreferredContentSizeExtraLarge] = &s_time_selection_config_large,
};

static const TimeSelectionSizeConfig *prv_selection_config(void) {
  const PreferredContentSize runtime_platform_default_size =
      system_theme_get_default_content_size_for_runtime_platform();
  return s_time_selection_configs[runtime_platform_default_size];
}

// FROM selection layer callbacks
static char* prv_handle_from_get_text(unsigned index, void *context) {
  TimeRangeSelectionWindowData *data = context;
  return date_time_selection_get_text(&data->from, index, data->buf);
}

static void prv_handle_from_complete(void *context) {
  TimeRangeSelectionWindowData *data = context;
  selection_layer_set_active(&data->from_selection_layer, false);
  selection_layer_set_active(&data->to_selection_layer, true);
  selection_layer_set_click_config_onto_window(&data->to_selection_layer, &data->window);
}

static void prv_handle_from_inc(unsigned index, void *context) {
  TimeRangeSelectionWindowData *data = context;
  date_time_handle_time_change(&data->from, index, 1);
}

static void prv_handle_from_dec(unsigned index, void *context) {
  TimeRangeSelectionWindowData *data = context;
  date_time_handle_time_change(&data->from, index, -1);
}

// TO selection layer callbacks
static char* prv_handle_to_get_text(unsigned index, void *context) {
  TimeRangeSelectionWindowData *data = context;
  return date_time_selection_get_text(&data->to, index, data->buf);
}

static void prv_handle_to_complete(void *context) {
  TimeRangeSelectionWindowData *data = context;
  i18n_free_all(&data->window);
  data->complete_callback(data, data->callback_context);
}

static void prv_handle_to_inc(unsigned index, void *context) {
  TimeRangeSelectionWindowData *data = context;
  date_time_handle_time_change(&data->to, index, 1);
}

static void prv_handle_to_dec(unsigned index, void *context) {
  TimeRangeSelectionWindowData *data = context;
  date_time_handle_time_change(&data->to, index, -1);
}

static void prv_text_layer_init(Window *window, TextLayer *text_layer, GRect *rect,
                                const char *i18n_str) {
  const GFont subtitle_font = system_theme_get_font_for_default_size(TextStyleFont_Subtitle);
  text_layer_init_with_parameters(text_layer, rect, i18n_get(i18n_str, window),
                                  subtitle_font, system_theme_get_fg_color(), GColorClear, GTextAlignmentCenter,
                                  GTextOverflowModeTrailingEllipsis);
  layer_add_child(&window->layer, &text_layer->layer);
}

// Window Setup
void time_range_selection_window_init(TimeRangeSelectionWindowData *time_range_selection_window,
    GColor color, TimeRangeSelectionCompleteCallback complete_callback, void *callback_context) {
  // General window setup
  *time_range_selection_window = (TimeRangeSelectionWindowData) {
    .complete_callback = complete_callback,
    .callback_context = callback_context,
  };

  Window *window = &time_range_selection_window->window;
  window_init(window, WINDOW_NAME("Time Range Selection Window"));
  window_set_user_data(window, time_range_selection_window);

  const TimeSelectionSizeConfig * const config = prv_selection_config();

  // Selection layer variables
  const int cell_width = config->cell_width;
  const int num_cells = clock_is_24h_style() ? 2 : 3;
  const int left_offset = clock_is_24h_style() ? 29 : 8;
  const int width = window->layer.bounds.size.w - 2 * left_offset;
  const int padding = config->cell_padding;
  const int from_top_offset = config->top_origin + config->selection_y_offset;
  const int to_top_offset = from_top_offset + config->start_end_y_offset;

  // FROM selection layer steup
  SelectionLayer *from_selection_layer = &time_range_selection_window->from_selection_layer;
  GRect frame = GRect(left_offset, from_top_offset, width, selection_layer_default_cell_height());
  selection_layer_init(from_selection_layer, &frame, num_cells);
  for (int i = 0; i < num_cells; i++) {
    selection_layer_set_cell_width(from_selection_layer, i, cell_width);
  }
  selection_layer_set_cell_padding(from_selection_layer, padding);
  selection_layer_set_active_bg_color(from_selection_layer, color);
  selection_layer_set_inactive_bg_color(from_selection_layer, GColorDarkGray);
  selection_layer_set_callbacks(from_selection_layer, time_range_selection_window,
      (SelectionLayerCallbacks) {
    .get_cell_text = prv_handle_from_get_text,
    .complete = prv_handle_from_complete,
    .increment = prv_handle_from_inc,
    .decrement = prv_handle_from_dec,
  });

  // TO selection layer steup
  SelectionLayer *to_selection_layer = &time_range_selection_window->to_selection_layer;
  frame.origin.y = to_top_offset;
  selection_layer_init(to_selection_layer, &frame, num_cells);
  for (int i = 0; i < num_cells; i++) {
    selection_layer_set_cell_width(to_selection_layer, i, cell_width);
  }
  selection_layer_set_cell_padding(to_selection_layer, padding);
  selection_layer_set_active_bg_color(to_selection_layer, color);
  selection_layer_set_inactive_bg_color(to_selection_layer, GColorDarkGray);
  selection_layer_set_callbacks(to_selection_layer, time_range_selection_window,
      (SelectionLayerCallbacks) {
    .get_cell_text = prv_handle_to_get_text,
    .complete = prv_handle_to_complete,
    .increment = prv_handle_to_inc,
    .decrement = prv_handle_to_dec,
  });

  selection_layer_set_click_config_onto_window(from_selection_layer, window);
  selection_layer_set_active(to_selection_layer, false);

  layer_add_child(&window->layer, &from_selection_layer->layer);
  layer_add_child(&window->layer, &to_selection_layer->layer);

  // Label setup
  TextLayer *from_text_layer = &time_range_selection_window->from_text_layer;
  GRect text_rect = GRect(0, config->top_origin, window->layer.bounds.size.w, 30);
  prv_text_layer_init(window, from_text_layer, &text_rect, i18n_noop("Start"));

  TextLayer *to_text_layer = &time_range_selection_window->to_text_layer;
  text_rect.origin.y += config->start_end_y_offset;
  prv_text_layer_init(window, to_text_layer, &text_rect, i18n_noop("End"));
}

void time_range_selection_window_deinit(TimeRangeSelectionWindowData *time_range_selection_window) {
  if (time_range_selection_window) {
    selection_layer_deinit(&time_range_selection_window->from_selection_layer);
    selection_layer_deinit(&time_range_selection_window->to_selection_layer);
  }
}
