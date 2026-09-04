/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "sleep_summary_card.h"
#include "sleep_summary_card_segments.h"
#include "sleep_detail_card.h"
#include "progress.h"
#include "ui.h"
#include "pbl/services/activity/health_util.h"

#include "applib/pbl_std/pbl_std.h"
#include "applib/ui/kino/kino_layer.h"
#include "board/display.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/util/size.h"

// Compile-time display offset calculations
#define HEALTH_X_OFFSET ((DISP_COLS - LEGACY_2X_DISP_COLS) / 2)
#define HEALTH_Y_OFFSET ((DISP_ROWS - LEGACY_2X_DISP_ROWS) / 2)

typedef struct HealthSleepSummaryCardData {
  HealthData *health_data;
  HealthProgressBar progress_bar;
  KinoReel *icon;

  GFont number_font;
  GFont unit_font;
  GFont typical_font;
  GFont em_dash_font;
} HealthSleepSummaryCardData;

#define PROGRESS_CURRENT_COLOR (PBL_IF_COLOR_ELSE(GColorVividCerulean, GColorDarkGray))
#define PROGRESS_SECONDARY_COLOR (PBL_IF_COLOR_ELSE(GColorVeryLightBlue, GColorBlack))
#define PROGRESS_TYPICAL_COLOR (PBL_IF_COLOR_ELSE(GColorYellow, GColorBlack))
#define PROGRESS_BACKGROUND_COLOR (PBL_IF_COLOR_ELSE(GColorDarkGray, GColorClear))
#define PROGRESS_OUTLINE_COLOR (PBL_IF_COLOR_ELSE(GColorClear, GColorBlack))

#define CURRENT_TEXT_COLOR (PBL_IF_COLOR_ELSE(GColorVividCerulean, GColorBlack))
#define TYPICAL_TEXT_COLOR (PBL_IF_COLOR_ELSE(GColorBlack, GColorWhite))
#define NO_DATA_TEXT_COLOR (PBL_IF_COLOR_ELSE(GColorWhite, GColorBlack))
#define CARD_BACKGROUND_COLOR (PBL_IF_COLOR_ELSE(GColorOxfordBlue, GColorWhite))

#define TWELVE_HOURS (SECONDS_PER_HOUR * 12)


static void prv_render_sleep_sessions(GContext *ctx, HealthSleepSummaryCardData *data) {
  const int num_sessions = health_data_sleep_get_num_sessions(data->health_data);
  ActivitySession *sessions = health_data_sleep_get_sessions(data->health_data);
  for (int i = 0; i < num_sessions; i++) {
    ActivitySession *session = &sessions[i];
    GColor fill_color = GColorClear;

    if (session->type == ActivitySessionType_Sleep) {
      fill_color = PROGRESS_CURRENT_COLOR;
    } else if (session->type == ActivitySessionType_RestfulSleep) {
      fill_color = PROGRESS_SECONDARY_COLOR;
    }

    if (gcolor_equal(fill_color, GColorClear)) {
      continue;
    }

    struct tm local_tm;
    localtime_r(&session->start_utc, &local_tm);

    const int session_start_24h = (local_tm.tm_sec +
                                  local_tm.tm_min * SECONDS_PER_MINUTE +
                                  local_tm.tm_hour * SECONDS_PER_HOUR);
    const int session_end_24h = session_start_24h + (session->length_min * SECONDS_PER_MINUTE);

    const int session_start_12h = session_start_24h % TWELVE_HOURS;
    const int session_end_12h = session_end_24h % TWELVE_HOURS;

    const int start = (session_start_12h * HEALTH_PROGRESS_BAR_MAX_VALUE / TWELVE_HOURS);
    const int end = (session_end_12h * HEALTH_PROGRESS_BAR_MAX_VALUE / TWELVE_HOURS);

    health_progress_bar_fill(ctx, &data->progress_bar, fill_color, start, end);
  }
}

static void prv_render_typical_markers(GContext *ctx, HealthSleepSummaryCardData *data) {
  // Some time fuzz is applied to a couple values to ensure that typical fill touches the sleep
  // sessions (needed because of how our fill algorithms work)
  const int sleep_start_24h = health_data_sleep_get_start_time(data->health_data);

  const int sleep_end_24h = health_data_sleep_get_end_time(data->health_data);

  if (sleep_start_24h || sleep_end_24h) {
#if PBL_COLOR
    const int time_fuzz = (2 * SECONDS_PER_MINUTE);
    const int sleep_start_12h = (sleep_start_24h) % TWELVE_HOURS;
    const int sleep_end_12h = (sleep_end_24h - time_fuzz) % TWELVE_HOURS;
    const int sleep_start = (sleep_start_12h * HEALTH_PROGRESS_BAR_MAX_VALUE / TWELVE_HOURS);
    const int sleep_end = (sleep_end_12h * HEALTH_PROGRESS_BAR_MAX_VALUE / TWELVE_HOURS);
#endif

    const int typical_sleep_start_24h = health_data_sleep_get_typical_start_time(data->health_data);
    const int typical_sleep_start_12h = typical_sleep_start_24h % TWELVE_HOURS;
    const int typical_sleep_end_24h = health_data_sleep_get_typical_end_time(data->health_data);
    const int typical_sleep_end_12h = typical_sleep_end_24h % TWELVE_HOURS;

    const int typical_start =
        (typical_sleep_start_12h * HEALTH_PROGRESS_BAR_MAX_VALUE / TWELVE_HOURS);
    const int typical_end =
        (typical_sleep_end_12h * HEALTH_PROGRESS_BAR_MAX_VALUE / TWELVE_HOURS);

#if PBL_COLOR
    const bool fell_asleep_late = (typical_sleep_start_24h < sleep_start_24h);
    if (fell_asleep_late) {
      health_progress_bar_fill(ctx, &data->progress_bar, PROGRESS_TYPICAL_COLOR,
                               typical_start, sleep_start);
    } else {
      health_progress_bar_mark(ctx, &data->progress_bar, PROGRESS_TYPICAL_COLOR, typical_start);
    }

    const bool woke_up_early = (typical_sleep_end_24h > sleep_end_24h);
    if (woke_up_early) {
      health_progress_bar_fill(ctx, &data->progress_bar, PROGRESS_TYPICAL_COLOR,
                               sleep_end, typical_end);
    } else {
      health_progress_bar_mark(ctx, &data->progress_bar, PROGRESS_TYPICAL_COLOR, typical_end);
    }
#else
    health_progress_bar_mark(ctx, &data->progress_bar, PROGRESS_TYPICAL_COLOR, typical_start);
    health_progress_bar_mark(ctx, &data->progress_bar, PROGRESS_TYPICAL_COLOR, typical_end);
#endif
  }
}

static void prv_render_progress_bar(GContext *ctx, Layer *base_layer) {
  HealthSleepSummaryCardData *data = layer_get_data(base_layer);

  // Renders the background
  health_progress_bar_fill(ctx, &data->progress_bar, PROGRESS_BACKGROUND_COLOR,
                           0, HEALTH_PROGRESS_BAR_MAX_VALUE);

  prv_render_sleep_sessions(ctx, data);

  prv_render_typical_markers(ctx, data);

  // This is required to get the rounded corners on the outside of the rectangle
  graphics_context_set_stroke_width(ctx, 2);
  graphics_context_set_stroke_color(ctx, CARD_BACKGROUND_COLOR);
  graphics_draw_round_rect(ctx, &s_sleep_summary_masking_rect, 5);

  // This needs to be done after drawing the progress bars or else the progress fill
  // overlaps the outline and things look weird
  health_progress_bar_outline(ctx, &data->progress_bar, PROGRESS_OUTLINE_COLOR);
}

static void prv_render_icon(GContext *ctx, Layer *base_layer) {
  HealthSleepSummaryCardData *data = layer_get_data(base_layer);

  const int y = PBL_IF_RECT_ELSE(PBL_IF_BW_ELSE(37, 32), 39) + HEALTH_Y_OFFSET;
  const int x_center_offset = 17;
  kino_reel_draw(data->icon, ctx, GPoint(base_layer->bounds.size.w / 2 - x_center_offset, y));
}

static void prv_render_current_sleep_text(GContext *ctx, Layer *base_layer) {
  HealthSleepSummaryCardData *data = layer_get_data(base_layer);

  // Mirror the pill's downshift at half the offset so the step count and
  // pill stay visually balanced. Zero on legacy-sized displays where
  // HEALTH_Y_OFFSET itself is 0.
  const int y = PBL_IF_RECT_ELSE(PBL_IF_BW_ELSE(85, 83), 88) + HEALTH_Y_OFFSET
                + HEALTH_Y_OFFSET / 6;
  const GRect rect = GRect(0, y, base_layer->bounds.size.w, 40);

  const int current_sleep = health_data_current_sleep_get(data->health_data);
  if (current_sleep) {
    // Draw the hours slept
    GTextNodeHorizontal *horiz_container = graphics_text_node_create_horizontal(MAX_TEXT_NODES);
    GTextNodeContainer *container = &horiz_container->container;
    horiz_container->horizontal_alignment = GTextAlignmentCenter;
    health_util_duration_to_hours_and_minutes_text_node(current_sleep, base_layer,
                                                        data->number_font,
                                                        data->unit_font,
                                                        CURRENT_TEXT_COLOR, container);
    graphics_text_node_draw(&container->node, ctx, &rect, NULL, NULL);
    graphics_text_node_destroy(&container->node);
  } else {
    char buffer[16];
    const GFont font = data->em_dash_font;
    snprintf(buffer, sizeof(buffer), EM_DASH);
    graphics_context_set_text_color(ctx, CURRENT_TEXT_COLOR);
    graphics_draw_text(ctx, buffer, font, rect, GTextOverflowModeFill, GTextAlignmentCenter, NULL);
  }
}

static void prv_render_typical_sleep_text(GContext *ctx, Layer *base_layer) {
  HealthSleepSummaryCardData *data = layer_get_data(base_layer);

  const int typical_sleep = health_data_sleep_get_cur_wday_average(data->health_data);

  char sleep_text[32];

  if (typical_sleep) {
    health_util_format_hours_and_minutes(sleep_text, sizeof(sleep_text), typical_sleep, base_layer);
  } else {
    snprintf(sleep_text, sizeof(sleep_text), EM_DASH);
  }

  health_ui_render_typical_text_box(ctx, base_layer, sleep_text);
}

static void prv_render_no_sleep_data_text(GContext *ctx, Layer *base_layer) {
  HealthSleepSummaryCardData *data = layer_get_data(base_layer);

  const int y = PBL_IF_RECT_ELSE(91, 100) + HEALTH_Y_OFFSET;
  const GRect rect = GRect(0, y, base_layer->bounds.size.w, 60);

  const char *text = i18n_get("No sleep data,\nwear your watch\nto sleep", base_layer);

  graphics_context_set_text_color(ctx, NO_DATA_TEXT_COLOR);
  graphics_draw_text(ctx, text, data->typical_font,
                     rect, GTextOverflowModeWordWrap, GTextAlignmentCenter, NULL);
}

static bool prv_has_sleep_data(HealthData *health_data) {
  // daily weekly stats doesn't include the first index so we check that separately
  return health_data_current_sleep_get(health_data) ||
         health_data_sleep_get_monthly_average(health_data) > 0;
}

static void prv_base_layer_update_proc(Layer *base_layer, GContext *ctx) {
  HealthSleepSummaryCardData *data = layer_get_data(base_layer);

  prv_render_icon(ctx, base_layer);

  prv_render_progress_bar(ctx, base_layer);

  if (!prv_has_sleep_data(data->health_data)) {
    prv_render_no_sleep_data_text(ctx, base_layer);
    return;
  }

  prv_render_current_sleep_text(ctx, base_layer);

  prv_render_typical_sleep_text(ctx, base_layer);
}

static void prv_sleep_detail_card_unload_callback(Window *window) {
  health_sleep_detail_card_destroy(window);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// API Functions
//

Layer *health_sleep_summary_card_create(HealthData *health_data) {
  // create base layer
  Layer *base_layer = layer_create_with_data(GRectZero, sizeof(HealthSleepSummaryCardData));
  HealthSleepSummaryCardData *health_sleep_summary_card_data = layer_get_data(base_layer);
  layer_set_update_proc(base_layer, prv_base_layer_update_proc);
  // set health data
  *health_sleep_summary_card_data = (HealthSleepSummaryCardData) {
    .icon = kino_reel_create_with_resource(RESOURCE_ID_HEALTH_APP_SLEEP),
    .progress_bar = {
      .num_segments = ARRAY_LENGTH(s_sleep_summary_progress_segments),
      .segments = s_sleep_summary_progress_segments,
    },
    .health_data = health_data,
#if DISP_ROWS > LEGACY_2X_DISP_ROWS
    .number_font = fonts_get_system_font(FONT_KEY_LECO_32_BOLD_NUMBERS),
    .unit_font = fonts_get_system_font(FONT_KEY_LECO_26_BOLD_NUMBERS_AM_PM),
#else
    .number_font = fonts_get_system_font(FONT_KEY_LECO_26_BOLD_NUMBERS_AM_PM),
    .unit_font = fonts_get_system_font(FONT_KEY_LECO_20_BOLD_NUMBERS),
#endif
    .typical_font = fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD),
    .em_dash_font = fonts_get_system_font(FONT_KEY_GOTHIC_28_BOLD),
  };
  return base_layer;
}

void health_sleep_summary_card_select_click_handler(Layer *layer) {
  HealthSleepSummaryCardData *health_sleep_summary_card_data = layer_get_data(layer);
  HealthData *health_data = health_sleep_summary_card_data->health_data;
  if (prv_has_sleep_data(health_data)) {
    Window *window = health_sleep_detail_card_create(health_data);
    window_set_window_handlers(window, &(WindowHandlers) {
      .unload = prv_sleep_detail_card_unload_callback,
    });
    app_window_stack_push(window, true);
  }
}

void health_sleep_summary_card_destroy(Layer *base_layer) {
  HealthSleepSummaryCardData *data = layer_get_data(base_layer);
  i18n_free_all(base_layer);
  kino_reel_destroy(data->icon);
  layer_destroy(base_layer);
}

GColor health_sleep_summary_card_get_bg_color(Layer *layer) {
  return CARD_BACKGROUND_COLOR;
}

bool health_sleep_summary_show_select_indicator(Layer *layer) {
  HealthSleepSummaryCardData *health_sleep_summary_card_data = layer_get_data(layer);
  return prv_has_sleep_data(health_sleep_summary_card_data->health_data);
}
