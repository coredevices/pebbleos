/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "time_selection_window.h"

#include "applib/ui/option_menu_window.h"
#include "pbl/services/clock.h"
#include "shell/system_theme.h"

#include <stdio.h>

#pragma GCC diagnostic ignored "-Wformat-truncation"

typedef struct TimeSelectionSizeConfig {
  const char *subtitle_font_key;

  int16_t cell_width;
  int16_t ampm_cell_width;
  int16_t cell_padding;

  int16_t top_offset_with_label;
  int16_t top_offset_without_label;
  int16_t label_origin_y;
  int16_t range_origin_y;
} TimeSelectionSizeConfig;

static const TimeSelectionSizeConfig s_time_selection_config_medium = {
  .subtitle_font_key = FONT_KEY_GOTHIC_14_BOLD,

  .cell_width = 40,
  .ampm_cell_width = PBL_IF_RECT_ELSE(40, 50),
  .cell_padding = 4,

  .top_offset_with_label = 75,
  .top_offset_without_label = 67,
  .label_origin_y = PBL_IF_RECT_ELSE(33, 38),
  .range_origin_y = 119,
};

static const TimeSelectionSizeConfig s_time_selection_config_large = {
  .subtitle_font_key = FONT_KEY_GOTHIC_18_BOLD,

  .cell_width = 56,
  .ampm_cell_width = 56,
  .cell_padding = 6,

  .top_offset_with_label = 87,
  .top_offset_without_label = 67, // NOTE: this hasn't been designed, because we don't use it
  .label_origin_y = 33,
  .range_origin_y = 158,
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

static int prv_cell_width(int i, int num_cells) {
  const TimeSelectionSizeConfig * const config = prv_selection_config();
  if (!clock_is_24h_style() && (i == (num_cells - 1))) {
    return config->ampm_cell_width;
  } else {
    return config->cell_width;
  }
}

static void prv_update_selection_layer(TimeSelectionWindowData *time_selection_window) {
  const TimeSelectionSizeConfig * const config = prv_selection_config();
  const int top_offset = time_selection_window->label_text_layer.text ?
                             config->top_offset_with_label : config->top_offset_without_label;
  layer_set_frame(&time_selection_window->selection_layer.layer,
                  &GRect(0, top_offset, time_selection_window->window.layer.bounds.size.w,
                         selection_layer_default_cell_height()));
}

static void prv_vertical_align_text_layer(TimeSelectionWindowData *time_selection_window,
                                          TextLayer *text_layer, int origin_y, int lines,
                                          int extra_line_offset_y) {
  // Supports `lines` or `lines + 1` line vertical centering for i18n
  const int line_height = fonts_get_font_height(text_layer->font);
  GRect frame = GRect(0, origin_y, time_selection_window->window.layer.bounds.size.w,
                      (lines + 1) * line_height + line_height / 2); // 1/2 more for descenders
  layer_set_frame(&text_layer->layer, &frame);
#if PBL_ROUND
  const int inset = 4;
  text_layer_enable_screen_text_flow_and_paging(text_layer, inset);
#endif
  const GSize content_size = app_text_layer_get_content_size(text_layer);
  if (content_size.h > lines * line_height) {
    frame = text_layer->layer.frame;
    frame.origin.y += -line_height / 2 + extra_line_offset_y;
    layer_set_frame(&text_layer->layer, &frame);
  }
}

static void prv_update_label_text_layer(TimeSelectionWindowData *time_selection_window) {
  TextLayer *label_text_layer = &time_selection_window->label_text_layer;
  if (label_text_layer->text) {
    const int label_origin_y = prv_selection_config()->label_origin_y;
    const int lines = 1;
    const int extra_line_offset_y = 0;
    prv_vertical_align_text_layer(time_selection_window, label_text_layer, label_origin_y, lines,
                                  extra_line_offset_y);
  }
}

static void prv_update_range_text_layer(TimeSelectionWindowData *time_selection_window) {
  TextLayer *range_text_layer = &time_selection_window->range_text_layer;
  TextLayer *range_subtitle_text_layer = &time_selection_window->range_subtitle_text_layer;
  if (range_text_layer->layer.hidden || range_subtitle_text_layer->layer.hidden) {
    return;
  }

  // update range_text_layer
  const int hour_end = time_selection_window->time_data.hour;
  const int minute_end = time_selection_window->time_data.minute;
  int hour_start = hour_end;
  int minute_start = minute_end;
  clock_hour_and_minute_add(&hour_start, &minute_start,
                            -time_selection_window->range_duration_m);

  char start_buf[TIME_STRING_TIME_LENGTH];
  char end_buf[TIME_STRING_TIME_LENGTH];
  clock_format_time(start_buf, sizeof(start_buf), hour_start, minute_start, true);
  clock_format_time(end_buf, sizeof(end_buf), hour_end, minute_end, true);

  char *buffer = time_selection_window->range_buf;
  snprintf(buffer, sizeof(time_selection_window->range_buf), "%s - %s", start_buf, end_buf);

  text_layer_set_text(range_text_layer, buffer);

  // update range_subtitle_text_layer
  buffer = time_selection_window->range_subtitle_buf;
  snprintf(buffer, sizeof(time_selection_window->range_subtitle_buf),
           "%s", time_selection_window->range_text);

  text_layer_set_text(range_subtitle_text_layer, buffer);

  TextLayer *top_layer = PBL_IF_RECT_ELSE(range_subtitle_text_layer, range_text_layer);
  TextLayer *bottom_layer = PBL_IF_RECT_ELSE(range_text_layer, range_subtitle_text_layer);

  const int range_origin_y = prv_selection_config()->range_origin_y;
  const int extra_line_offset_y = PBL_IF_RECT_ELSE(2, 4);
  prv_vertical_align_text_layer(time_selection_window, top_layer, range_origin_y,
                                1, extra_line_offset_y);
  const GSize top_size = app_text_layer_get_content_size(top_layer);
  const int range_bottom_origin_y = range_origin_y + top_size.h;
  prv_vertical_align_text_layer(time_selection_window, bottom_layer, range_bottom_origin_y,
                                1, extra_line_offset_y);
}

static void prv_update_layer_placement(TimeSelectionWindowData *time_selection_window) {
  prv_update_selection_layer(time_selection_window);
  prv_update_label_text_layer(time_selection_window);
  prv_update_range_text_layer(time_selection_window);
}

// FROM selection layer callbacks
static char* prv_handle_from_get_text(unsigned index, void *context) {
  TimeSelectionWindowData *data = context;
  return date_time_selection_get_text(&data->time_data, index, data->cell_buf);
}

// Selection layer callbacks
static void prv_handle_complete(void *context) {
  TimeSelectionWindowData *data = context;
  data->complete_callback(data, data->callback_context);
}

static void prv_handle_inc(unsigned index, void *context) {
  TimeSelectionWindowData *data = context;
  date_time_handle_time_change(&data->time_data, index, 1);
  prv_update_range_text_layer(data);
}

static void prv_handle_dec(unsigned index, void *context) {
  TimeSelectionWindowData *data = context;
  date_time_handle_time_change(&data->time_data, index, -1);
  prv_update_range_text_layer(data);
}

// Public Functions
void time_selection_window_set_to_current_time(TimeSelectionWindowData *time_selection_window) {
  struct tm current_time;
  clock_get_time_tm(&current_time);
  time_selection_window->time_data.hour = current_time.tm_hour;
  time_selection_window->time_data.minute = current_time.tm_min;
}

void time_selection_window_configure(TimeSelectionWindowData *time_selection_window,
                                     const TimeSelectionWindowConfig *config) {
  text_layer_set_text(&time_selection_window->label_text_layer, config->label);
  if (config->color.a) {
    selection_layer_set_active_bg_color(&time_selection_window->selection_layer, config->color);
  }
  if (config->range.update) {
    time_selection_window->range_text = config->range.text;
    time_selection_window->range_duration_m = config->range.duration_m;
    layer_set_hidden(&time_selection_window->range_text_layer.layer, !config->range.enabled);
    layer_set_hidden(&time_selection_window->range_subtitle_text_layer.layer,
                     !config->range.enabled);
  }
  if (config->callback.update) {
    time_selection_window->complete_callback = config->callback.complete;
    time_selection_window->callback_context = config->callback.context;
  }
  prv_update_layer_placement(time_selection_window);
}

static void prv_text_layer_init(Layer *window_layer, TextLayer *text_layer, const GFont font) {
  text_layer_init_with_parameters(text_layer, &GRectZero, NULL, font, system_theme_get_fg_color(), GColorClear,
                                  GTextAlignmentCenter, GTextOverflowModeTrailingEllipsis);
  layer_add_child(window_layer, &text_layer->layer);
  layer_set_hidden(&text_layer->layer, true);
}

void time_selection_window_init(TimeSelectionWindowData *time_selection_window,
                                const TimeSelectionWindowConfig *config) {
  *time_selection_window = (TimeSelectionWindowData) {};

  // General window setup
  Window *window = &time_selection_window->window;
  window_init(window, WINDOW_NAME("Time Selection Window"));
  window_set_user_data(window, time_selection_window);

  // Selection layer setup
  const TimeSelectionSizeConfig * const size_config = prv_selection_config();
  const int num_cells = clock_is_24h_style() ? 2 : 3;
  const int padding = size_config->cell_padding;
  SelectionLayer *selection_layer = &time_selection_window->selection_layer;
  selection_layer_init(selection_layer, &GRectZero, num_cells);
  for (int i = 0; i < num_cells; i++) {
    const unsigned int cell_width = prv_cell_width(i, num_cells);
    selection_layer_set_cell_width(selection_layer, i, cell_width);
  }

  selection_layer_set_cell_padding(selection_layer, padding);
  selection_layer_set_inactive_bg_color(selection_layer, GColorDarkGray);
  selection_layer_set_click_config_onto_window(selection_layer, window);
  selection_layer_set_callbacks(&time_selection_window->selection_layer, time_selection_window,
                                (SelectionLayerCallbacks) {
    .get_cell_text = prv_handle_from_get_text,
    .complete = prv_handle_complete,
    .increment = prv_handle_inc,
    .decrement = prv_handle_dec,
  });
  layer_add_child(&window->layer, &time_selection_window->selection_layer.layer);

  // Label setup
  const GFont header_font = system_theme_get_font_for_default_size(TextStyleFont_Header);

  TextLayer *label_text_layer = &time_selection_window->label_text_layer;
  prv_text_layer_init(&window->layer, label_text_layer, header_font);
  layer_set_hidden(&label_text_layer->layer, false);

  // Range setup
  TextLayer *range_text_layer = &time_selection_window->range_text_layer;
  prv_text_layer_init(&window->layer, range_text_layer, header_font);

  // Range subtitle setup
  TextLayer *range_subtitle_text_layer = &time_selection_window->range_subtitle_text_layer;
  prv_text_layer_init(&window->layer, range_subtitle_text_layer,
                      fonts_get_system_font(size_config->subtitle_font_key));

  // Status setup
  status_bar_layer_init(&time_selection_window->status_layer);
  status_bar_layer_set_separator_mode(&time_selection_window->status_layer,
                                      PBL_IF_COLOR_ELSE(OPTION_MENU_STATUS_SEPARATOR_MODE,
                                                        StatusBarLayerSeparatorModeNone));
  layer_add_child(&time_selection_window->window.layer,
                  &time_selection_window->status_layer.layer);

  time_selection_window_configure(time_selection_window, config);
}

void time_selection_window_deinit(TimeSelectionWindowData *time_selection_window) {
  if (time_selection_window) {
    status_bar_layer_deinit(&time_selection_window->status_layer);
    selection_layer_deinit(&time_selection_window->selection_layer);
  }
}
