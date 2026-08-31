/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "graph_card.h"

#include "applib/graphics/graphics.h"
#include "applib/graphics/text.h"
#include "board/display.h"
#include <pbl/drivers/rtc.h>
#include "kernel/pbl_malloc.h"
#include "pbl/services/clock.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/util/math.h"
#include "pbl/util/string.h"
#include "util/time/time.h"

// Compile-time display offset calculations
#define DISPLAY_Y_OFFSET ((DISP_ROWS - LEGACY_2X_DISP_ROWS) / 2)

//! Marks where the graph begins - base values for 168px height
#define GRAPH_OFFSET_Y_BASE PBL_IF_RECT_ELSE(38, 48)

//! Marks where the graph ends and where the labels begin - base values for 168px height
#define LABEL_OFFSET_Y_BASE PBL_IF_RECT_ELSE(118, 113)
#define LABEL_HEIGHT 27
#define GRAPH_OFFSET_Y (GRAPH_OFFSET_Y_BASE + DISPLAY_Y_OFFSET)
#define LABEL_OFFSET_Y (LABEL_OFFSET_Y_BASE + DISPLAY_Y_OFFSET)

#define GRAPH_HEIGHT (LABEL_OFFSET_Y - GRAPH_OFFSET_Y)

// Compile-time bar width calculation
#define HEALTH_BAR_WIDTH (23 + (DISP_COLS - LEGACY_2X_DISP_COLS) / 19)
#define HEALTH_BAR_INSET 3
#define HEALTH_TOTAL_BAR_WIDTHS PBL_IF_RECT_ELSE((HEALTH_BAR_WIDTH * 7 + 3) - (HEALTH_BAR_INSET * 6), 141)

// Compile-time avg line width calculations
#define HEALTH_WEEKDAY_WIDTH_BASE PBL_IF_RECT_ELSE(103, 119)
#define HEALTH_WEEKEND_WIDTH_BASE PBL_IF_RECT_ELSE(38, 58)
#define HEALTH_WEEKDAY_WIDTH (HEALTH_WEEKDAY_WIDTH_BASE + (DISP_COLS - LEGACY_2X_DISP_COLS) * HEALTH_WEEKDAY_WIDTH_BASE / LEGACY_2X_DISP_COLS)
#define HEALTH_WEEKEND_WIDTH (HEALTH_WEEKEND_WIDTH_BASE + (DISP_COLS - LEGACY_2X_DISP_COLS) * HEALTH_WEEKEND_WIDTH_BASE / LEGACY_2X_DISP_COLS)

#define AVG_LINE_HEIGHT 4
#define AVG_LINE_LEGEND_WIDTH 10
#define AVG_LINE_COLOR GColorYellow

#define INFO_PADDING_BOTTOM 6

//! Get the current day in the standard tm format. Sunday is 0
static uint8_t prv_get_weekday(time_t timestamp) {
  return time_util_get_day_in_week(timestamp);
}

static void prv_draw_title(HealthGraphCard *graph_card, GContext *ctx) {
  const GRect *bounds = &graph_card->layer.bounds;
  graphics_context_set_text_color(ctx, GColorBlack);
  const int title_height = 60;
  GRect drawing_box = GRect(0, 0, bounds->size.w, title_height);

#if PBL_ROUND
  // inset the drawing bounds if on round to account for the bezel
  drawing_box = grect_inset(drawing_box, GEdgeInsets(8));

  const GSize text_size = graphics_text_layout_get_max_used_size(ctx, graph_card->title,
      graph_card->title_font, drawing_box, GTextOverflowModeWordWrap, GTextAlignmentCenter, NULL);

  // increase drawing box y offset if we're only drawing one line of text
  if (text_size.h < 30) {
    drawing_box.origin.y += 10;
  }
#endif

  graphics_draw_text(ctx, graph_card->title, graph_card->title_font, drawing_box,
                     GTextOverflowModeWordWrap, GTextAlignmentCenter, NULL);
}

static void prv_draw_day_labels_background(HealthGraphCard *graph_card, GContext *ctx) {
  const GRect *bounds = &graph_card->layer.bounds;
  GRect box = {{ .y = LABEL_OFFSET_Y }, { bounds->size.w, LABEL_HEIGHT }};
  graphics_context_set_fill_color(ctx, GColorBlack);
  graphics_fill_rect(ctx, &box);
  const int border_width = 3;
  box = grect_inset(box, GEdgeInsets(border_width, 0));
  const GColor label_background_color = GColorWhite;
  graphics_context_set_fill_color(ctx, label_background_color);
  graphics_fill_rect(ctx, &box);
}

//! Get the corresponding data point for a weekday.
//! Sunday is 0, and the day data begins with today and continues into the past.
static int32_t prv_get_day_point(HealthGraphCard *graph_card, int weekday) {
  const int index = positive_modulo(graph_card->current_day - weekday, DAYS_PER_WEEK);
  return graph_card->day_data[index];
}

static int32_t prv_convert_to_graph_height(HealthGraphCard *graph_card, int32_t point) {
  // Round up in order to show the minimum stub bar once progress begins
  int bar_height = (point * GRAPH_HEIGHT + graph_card->data_max - 1) / graph_card->data_max;
  const int minimum_stub_height = 5;
  if (bar_height > 0 && bar_height < minimum_stub_height) {
    // Show the minimum stub bar if progress just began
    bar_height = minimum_stub_height;
  }
  return bar_height;
}

static void prv_setup_day_bar_box(int weekday, GRect *box, int16_t bar_height) {
  const int w = HEALTH_BAR_WIDTH;
#if PBL_RECT
  // The center bars are slightly wider than the other bars
  // Note that Thursday is the center bar, not Wednesday since drawing begins with Monday
  //                                      S  M  T    W      T      F    S
  const int bar_widths[DAYS_PER_WEEK] = { w, w, w, w + 1, w + 1, w + 1, w };
  const int bar_width = bar_widths[weekday];
#else
  const int bar_width = w;
#endif
  box->origin.y = LABEL_OFFSET_Y - bar_height;
  box->size = GSize(bar_width, bar_height);
}

static void prv_draw_day_bar_wide(GContext *ctx, const GRect *box, const GRect *box_inset,
                                  GColor bar_color) {
  const GColor border_color = GColorBlack;
  graphics_context_set_fill_color(ctx, border_color);
  graphics_fill_rect(ctx, box);
  graphics_context_set_fill_color(ctx, bar_color);
  graphics_fill_rect(ctx, box_inset);
}

static void prv_draw_day_bar_thin(GContext *ctx, const GRect *box, int weekday,
                                  GColor bar_color) {
  GRect thin_box = *box;
  // Nudge the bars before Thursday (inclusive). Note that Sunday is on the right side, at the end
  const int thin_offset_x = WITHIN(weekday, Monday, Thursday) ? 1 : 0;
  const int thin_width = 5;
  thin_box.origin.x += thin_offset_x + (box->size.w - thin_width) / 2;
  thin_box.size.w = thin_width;
  graphics_context_set_fill_color(ctx, bar_color);
  graphics_fill_rect(ctx, &thin_box);
}

static int16_t prv_draw_day_bar(GContext *ctx, int weekday, const GRect *box,
                                GColor bar_color, bool wide_bar) {
  const int bar_inset = 3;
  GRect box_inset = grect_inset(*box, GEdgeInsets(bar_inset, bar_inset, 0, bar_inset));
  if (wide_bar) {
    prv_draw_day_bar_wide(ctx, box, &box_inset, bar_color);
  } else {
    prv_draw_day_bar_thin(ctx, box, weekday, bar_color);
  }
  // The borders of the boxes caused by the inset need to overlap each other
  return box->origin.x + box->size.w - bar_inset;
}

static bool prv_bar_should_be_wide(int draw_weekday, int current_weekday) {
  // The graph begins on Monday, so all bars from Monday until current (inclusive) should be wide
  return (positive_modulo(draw_weekday - Monday, DAYS_PER_WEEK) <=
          positive_modulo(current_weekday - Monday, DAYS_PER_WEEK));
}

static GColor prv_get_bar_color(HealthGraphCard *graph_card, bool is_active, bool is_wide) {
  const GColor active_color = GColorWhite;
  const GColor inactive_wide_color = GColorDarkGray;
  const GColor inactive_thin_color = graph_card->inactive_color;
  return (is_active ? active_color : (is_wide ? inactive_wide_color : inactive_thin_color));
}

static void prv_draw_day_bars(HealthGraphCard *graph_card, GContext *ctx) {
  // With values from prv_setup_day_bar_box and prv_draw_day_bar,
  // total_bar_width is sum(bar_widths) - (bar_inset * (DAYS_PER_WEEK - 1))
  const int total_bar_widths = HEALTH_TOTAL_BAR_WIDTHS;
  const int legend_line_height = fonts_get_font_height(graph_card->legend_font);
  const GRect *bounds = &graph_card->layer.bounds;
  GRect box = { .origin.x = (bounds->size.w - total_bar_widths) / 2, .origin.y = LABEL_OFFSET_Y };
  // The first day to draw is Monday, and draw a week's worth of bars
  for (int i = Monday, draw_count = 0;
       draw_count < DAYS_PER_WEEK;
       draw_count++, i = (i + 1) % DAYS_PER_WEEK) {
    // Setup the dimensions and color of the day bar
    const int32_t day_point = prv_get_day_point(graph_card, i);
    const int bar_height = prv_convert_to_graph_height(graph_card, day_point);

    const bool is_active = (graph_card->selection == i);
    if (graph_card->current_day == i) {
      // Draw last week's bar as a thin bar behind this bar
      const int32_t last_bar_height =
          prv_convert_to_graph_height(graph_card, graph_card->day_data[DAYS_PER_WEEK]);
      prv_setup_day_bar_box(i, &box, last_bar_height);
      const GColor bar_color = prv_get_bar_color(graph_card, is_active, false /* wide bar */);
      prv_draw_day_bar(ctx, i, &box, bar_color, false /* wide bar */);
    }

    // Draw the day bar
    prv_setup_day_bar_box(i, &box, bar_height);
    const bool is_wide = prv_bar_should_be_wide(i, graph_card->current_day);
    const GColor bar_color = prv_get_bar_color(graph_card, is_active, is_wide);
    const int16_t next_x = prv_draw_day_bar(ctx, i, &box, bar_color, is_wide);

    // Draw the day character legend
    const int char_offset_y = 1;
    box.origin.y = LABEL_OFFSET_Y + char_offset_y;
    box.size.h = legend_line_height;
    char char_buffer[] = { graph_card->day_chars[i], '\0' };
    const GColor active_legend_color = GColorRed;
    const GColor inactive_legend_color = GColorBlack;
    graphics_context_set_text_color(ctx, is_active ? active_legend_color : inactive_legend_color);
    graphics_draw_text(ctx, char_buffer, graph_card->legend_font, box,
                       GTextOverflowModeWordWrap, GTextAlignmentCenter, NULL);

    // Move the box cursor to the next bar
    box.origin.x = next_x;
  }
}

static void prv_draw_avg_line(HealthGraphCard *graph_card, GContext *ctx, int32_t avg,
                              int16_t offset_x, int16_t width) {
  if (avg == 0) {
    return;
  }
  const int offset_y = LABEL_OFFSET_Y - MAX(prv_convert_to_graph_height(graph_card, avg),
                                            AVG_LINE_HEIGHT / 2);
  graphics_context_set_fill_color(ctx, AVG_LINE_COLOR);
  graphics_fill_rect(ctx, &(GRect) {{ offset_x, offset_y - AVG_LINE_HEIGHT / 2 },
                                    { width, AVG_LINE_HEIGHT }});
}

static void prv_draw_avg_lines(HealthGraphCard *graph_card, GContext *ctx) {
  const GRect *bounds = &graph_card->layer.bounds;
  prv_draw_avg_line(graph_card, ctx, graph_card->stats.weekday.avg, 0, HEALTH_WEEKDAY_WIDTH);
  prv_draw_avg_line(graph_card, ctx, graph_card->stats.weekend.avg, bounds->size.w - HEALTH_WEEKEND_WIDTH,
                    HEALTH_WEEKEND_WIDTH);
}

static int32_t prv_get_info_data_point(HealthGraphCard *graph_card) {
  // Show today's data point if the selection is a day of the week, otherwise show the weekday
  // average if the current day is a weekday or weekend average if the current day is on the weekend
  if (graph_card->selection == HealthGraphIndex_Average) {
    return IS_WEEKDAY(graph_card->current_day) ? graph_card->stats.weekday.avg :
                                                 graph_card->stats.weekend.avg;
  }
  int day_point = prv_get_day_point(graph_card, graph_card->selection);
  if (graph_card->selection == graph_card->current_day && day_point == 0) {
    // If today has no progress, use the info from last week
    day_point = graph_card->day_data[DAYS_PER_WEEK];
  }
  return day_point;
}

static void prv_draw_avg_line_legend(HealthGraphCard *graph_card, GContext *ctx, int offset_x,
                                     int info_offset_y, GSize custom_text_size) {
  const int info_line_height = fonts_get_font_height(graph_card->legend_font);
  const int avg_line_offset_y = -1;
  const GRect avg_line_box = {
    .origin.x = offset_x,
    // Position vertically centered with the text
    .origin.y = info_offset_y + (info_line_height + INFO_PADDING_BOTTOM) / 2 + avg_line_offset_y,
    .size = { AVG_LINE_LEGEND_WIDTH, AVG_LINE_HEIGHT },
  };
  graphics_context_set_fill_color(ctx, AVG_LINE_COLOR);
  graphics_fill_rect(ctx, &avg_line_box);
}

static void prv_draw_avg_info_text(HealthGraphCard *graph_card, GContext *ctx, int offset_x,
                                   int offset_y, int height) {
  const GRect *bounds = &graph_card->layer.bounds;
  const GRect avg_text_box = {{ offset_x, offset_y }, { bounds->size.w, height }};
  graphics_draw_text(ctx, graph_card->info_avg, graph_card->legend_font, avg_text_box,
                     GTextOverflowModeWordWrap, GTextAlignmentLeft, NULL);
}

static void prv_draw_custom_info_text(HealthGraphCard *graph_card, GContext *ctx, char *text,
                                      int offset_x, int info_offset_y, int info_height) {
  const GRect *bounds = &graph_card->layer.bounds;
  const GRect info_text_box = {{ offset_x, info_offset_y }, { bounds->size.w, info_height }};
  graphics_context_set_text_color(ctx, GColorBlack);
  graphics_draw_text(ctx, text, graph_card->legend_font, info_text_box,
                     GTextOverflowModeWordWrap, GTextAlignmentLeft, NULL);
}

static bool prv_is_selection_last_weekday(HealthGraphCard *graph_card) {
  // If the selection is today, the selection is last week's only if today has no progress
  // Else if today is Sunday, the entire graph represents the current week
  // Otherwise the selection is last week if either the selection is Sunday
  // or if the selection is greater than the current day
  return (((int)graph_card->current_day == graph_card->selection && graph_card->day_data[0] == 0) ||
          ((graph_card->current_day == Sunday) ? false :
           ((int)graph_card->selection == Sunday ||
            (int)graph_card->selection > graph_card->current_day)));
}

size_t health_graph_format_weekday_prefix(HealthGraphCard *graph_card, char *buffer,
                                          size_t buffer_size) {
  if (prv_is_selection_last_weekday(graph_card)) {
    // The graph starts on Monday, so wrap around the selection and current_day for Sunday
    const time_t selection_time =
        ((positive_modulo(graph_card->selection - Monday, DAYS_PER_WEEK) -
          positive_modulo(graph_card->current_day - Monday, DAYS_PER_WEEK) - DAYS_PER_WEEK) *
         SECONDS_PER_DAY) + graph_card->data_timestamp;
    const int pos = clock_get_month_named_abbrev_date(buffer, buffer_size, selection_time);
    strncat(buffer, i18n_get(": ", graph_card), buffer_size - pos - 1);
    return strlen(buffer);
  } else {
    struct tm local_tm = (struct tm) {
      .tm_wday = positive_modulo(graph_card->selection, DAYS_PER_WEEK),
    };
    return strftime(buffer, buffer_size, i18n_get("%a: ", graph_card), &local_tm);
  }
}

static void prv_draw_info_with_text(HealthGraphCard *graph_card, GContext *ctx, char *text) {
  const GRect *bounds = &graph_card->layer.bounds;
  // Calculate the custom info text size
  GSize custom_text_size;
  const TextLayoutExtended text_layout = {};
  graphics_text_layout_get_max_used_size(ctx, text, graph_card->legend_font, *bounds,
                                         GTextOverflowModeWordWrap, GTextAlignmentLeft,
                                         (GTextAttributes *)&text_layout);
  custom_text_size = text_layout.max_used_size;
  GSize avg_text_size = GSizeZero;
  int total_width = custom_text_size.w;

  const int info_padding_top = PBL_IF_RECT_ELSE(-1, 1);
  const int info_offset_y = LABEL_OFFSET_Y + LABEL_HEIGHT + info_padding_top;
  const int info_line_height = fonts_get_font_height(graph_card->legend_font);
  const int info_height = PBL_IF_ROUND_ELSE(2, 1) * info_line_height + INFO_PADDING_BOTTOM;

  int cursor_x = 0;
  if (graph_card->selection == HealthGraphIndex_Average) {
    graphics_text_layout_get_max_used_size(ctx, graph_card->info_avg, graph_card->legend_font,
                                           *bounds, GTextOverflowModeWordWrap,
                                           GTextAlignmentLeft, (GTextAttributes *)&text_layout);
    avg_text_size = text_layout.max_used_size;
    total_width += avg_text_size.w + AVG_LINE_LEGEND_WIDTH;

    // Draw the avg line legend
    cursor_x = (bounds->size.w - total_width) / 2;
    prv_draw_avg_line_legend(graph_card, ctx, cursor_x, info_offset_y, custom_text_size);
    cursor_x += AVG_LINE_LEGEND_WIDTH;

    // Draw the avg info text
    prv_draw_avg_info_text(graph_card, ctx, cursor_x, info_offset_y, info_height);
    cursor_x += avg_text_size.w;
  } else {
    // Center the custom text
    cursor_x = (bounds->size.w - total_width) / 2;
  }

  // Draw the custom info text
  prv_draw_custom_info_text(graph_card, ctx, text, cursor_x, info_offset_y, info_height);
}

static void prv_draw_info(HealthGraphCard *graph_card, GContext *ctx) {
  if (!graph_card->info_buffer_size) {
    return;
  }
  char buffer[graph_card->info_buffer_size];
  memset(buffer, 0, sizeof(buffer));
  if (graph_card->info_update) {
    const int32_t day_point = prv_get_info_data_point(graph_card);
    graph_card->info_update(graph_card, day_point, buffer, sizeof(buffer));
  }
  if (IS_EMPTY_STRING(buffer)) {
    return;
  }
  prv_draw_info_with_text(graph_card, ctx, buffer);
}

static void prv_health_graph_layer_update_proc(Layer *layer, GContext *ctx) {
  HealthGraphCard *graph_card = (HealthGraphCard *)layer;

  prv_draw_title(graph_card, ctx);
  prv_draw_day_labels_background(graph_card, ctx);
  prv_draw_day_bars(graph_card, ctx);
  prv_draw_avg_lines(graph_card, ctx);
  prv_draw_info(graph_card, ctx);
}

HealthGraphCard *health_graph_card_create(const HealthGraphCardConfig *config) {
  HealthGraphCard *graph_card = app_zalloc_check(sizeof(HealthGraphCard));
  if (graph_card) {
    layer_init(&graph_card->layer, &GRectZero);
    layer_set_update_proc(&graph_card->layer, prv_health_graph_layer_update_proc);
    health_graph_card_configure(graph_card, config);
    graph_card->title_font = fonts_get_system_font(PBL_IF_RECT_ELSE(FONT_KEY_GOTHIC_24_BOLD,
                                                                    FONT_KEY_GOTHIC_18_BOLD));
    graph_card->legend_font = fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD);
    graph_card->current_day = prv_get_weekday(graph_card->data_timestamp);
    // The day characters in standard tm weekday order
    graph_card->day_chars = i18n_get("SMTWTFS", graph_card);
    graph_card->selection = HealthGraphIndex_Average;
  }
  return graph_card;
}

void health_graph_card_destroy(HealthGraphCard *graph_card) {
  if (!graph_card) {
    return;
  }
  layer_deinit(&graph_card->layer);
  i18n_free_all(graph_card);
  app_free(graph_card);
}

void health_graph_card_configure(HealthGraphCard *graph_card, const HealthGraphCardConfig *config) {
  if (!graph_card || !config) {
    return;
  }
  if (config->title) {
    graph_card->title = i18n_get(config->title, graph_card);
  }
  if (config->info_avg) {
    graph_card->info_avg = i18n_get(config->info_avg, graph_card);
  }
  if (config->graph_data) {
    graph_card->stats = config->graph_data->stats;
    memcpy(graph_card->day_data, config->graph_data->day_data, sizeof(graph_card->day_data));
    graph_card->data_timestamp = config->graph_data->timestamp;
    graph_card->data_max = MAX(config->graph_data->default_max,
                               config->graph_data->stats.daily.max);
  }
  if (config->info_update) {
    graph_card->info_update = config->info_update;
  }
  if (config->info_buffer_size) {
    graph_card->info_buffer_size = config->info_buffer_size;
  }
  if (!gcolor_equal(config->inactive_color, GColorClear)) {
    graph_card->inactive_color = config->inactive_color;
  }
}

void health_graph_card_cycle_selected(HealthGraphCard *graph_card) {
  if (graph_card->selection == HealthGraphIndex_Sunday) {
    // Sunday is the last day in the graph, show the average next
    graph_card->selection = HealthGraphIndex_Average;
  } else if (graph_card->selection == HealthGraphIndex_Average) {
    // Monday is the first day in the graph, show Monday after showing the average
    graph_card->selection = HealthGraphIndex_Monday;
  } else {
    // Otherwise progress through the weekdays normally
    graph_card->selection = (graph_card->selection + 1) % DAYS_PER_WEEK;
  }
}
