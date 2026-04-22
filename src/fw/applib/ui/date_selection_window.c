/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "date_selection_window.h"

#include "applib/ui/option_menu_window.h"
#include "pbl/services/clock.h"
#include "shell/system_theme.h"
#include "util/date.h"

#include <stdio.h>

// ---------------------------------------------------------------------------
// Layout configuration
// ---------------------------------------------------------------------------

typedef struct {
  int16_t year_cell_width;
  int16_t month_cell_width;
  int16_t day_cell_width;
  int16_t cell_padding;
  int16_t top_offset;
  int16_t label_origin_y;
} DateSelectionSizeConfig;

static const DateSelectionSizeConfig s_date_selection_config_medium = {
  .year_cell_width = 54,
  .month_cell_width = 32,
  .day_cell_width = 32,
  .cell_padding = 4,
  .top_offset = 75,
  .label_origin_y = PBL_IF_RECT_ELSE(33, 38),
};

static const DateSelectionSizeConfig s_date_selection_config_large = {
  .year_cell_width = 70,
  .month_cell_width = 44,
  .day_cell_width = 44,
  .cell_padding = 6,
  .top_offset = 87,
  .label_origin_y = 33,
};

static const DateSelectionSizeConfig *s_date_selection_configs[NumPreferredContentSizes] = {
  [PreferredContentSizeSmall] = &s_date_selection_config_medium,
  [PreferredContentSizeMedium] = &s_date_selection_config_medium,
  [PreferredContentSizeLarge] = &s_date_selection_config_large,
  [PreferredContentSizeExtraLarge] = &s_date_selection_config_large,
};

static const DateSelectionSizeConfig *prv_config(void) {
  const PreferredContentSize default_size =
      system_theme_get_default_content_size_for_runtime_platform();
  return s_date_selection_configs[default_size];
}

// ---------------------------------------------------------------------------
// Cell text rendering
// ---------------------------------------------------------------------------

static char *prv_get_cell_text(unsigned index, void *context) {
  DateSelectionWindowData *data = context;
  switch ((DateInputIndex)index) {
    case DateInputIndexYear:
      snprintf(data->cell_buf, sizeof(data->cell_buf), "%04d",
               (int)(data->date.year + STDTIME_YEAR_OFFSET));
      return data->cell_buf;
    case DateInputIndexMonth:
      snprintf(data->cell_buf, sizeof(data->cell_buf), "%02d",
               (int)(data->date.month + 1));
      return data->cell_buf;
    case DateInputIndexDay:
      snprintf(data->cell_buf, sizeof(data->cell_buf), "%02d",
               (int)data->date.day);
      return data->cell_buf;
    default:
      return "";
  }
}

// ---------------------------------------------------------------------------
// Selection layer callbacks
// ---------------------------------------------------------------------------

static void prv_handle_complete(void *context) {
  DateSelectionWindowData *data = context;
  if (data->complete_callback) {
    data->complete_callback(data, data->callback_context);
  }
}

static void prv_handle_inc(unsigned index, void *context) {
  DateSelectionWindowData *data = context;
  switch ((DateInputIndex)index) {
    case DateInputIndexYear:
      data->date.year = date_time_selection_step_year(data->date.year, 1);
      data->date.day = date_time_selection_truncate_date(
          data->date.year, data->date.month, data->date.day);
      break;
    case DateInputIndexMonth:
      data->date.month = date_time_selection_step_month(data->date.month, 1);
      data->date.day = date_time_selection_truncate_date(
          data->date.year, data->date.month, data->date.day);
      break;
    case DateInputIndexDay:
      data->date.day = date_time_selection_step_day(
          data->date.year, data->date.month, data->date.day, 1);
      break;
    default:
      break;
  }
}

static void prv_handle_dec(unsigned index, void *context) {
  DateSelectionWindowData *data = context;
  switch ((DateInputIndex)index) {
    case DateInputIndexYear:
      data->date.year = date_time_selection_step_year(data->date.year, -1);
      data->date.day = date_time_selection_truncate_date(
          data->date.year, data->date.month, data->date.day);
      break;
    case DateInputIndexMonth:
      data->date.month = date_time_selection_step_month(data->date.month, -1);
      data->date.day = date_time_selection_truncate_date(
          data->date.year, data->date.month, data->date.day);
      break;
    case DateInputIndexDay:
      data->date.day = date_time_selection_step_day(
          data->date.year, data->date.month, data->date.day, -1);
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// Helper – text layer setup
// ---------------------------------------------------------------------------

static void prv_text_layer_init(Layer *window_layer, TextLayer *text_layer, const GFont font) {
  text_layer_init_with_parameters(text_layer, &GRectZero, NULL, font, system_theme_get_fg_color(), GColorClear,
                                  GTextAlignmentCenter, GTextOverflowModeTrailingEllipsis);
  layer_add_child(window_layer, &text_layer->layer);
  layer_set_hidden(&text_layer->layer, true);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void date_selection_window_set_to_current_date(DateSelectionWindowData *window) {
  struct tm now;
  clock_get_time_tm(&now);
  // Clamp the year to the selectable range so that dates outside the valid
  // window (e.g. year 2000 after an RTC reset) are snapped to the nearest
  // valid value instead of displaying an out-of-range year that jumps on
  // the first button press.
  window->date.year = date_time_selection_step_year(now.tm_year, 0);
  window->date.month = now.tm_mon;
  window->date.day = date_time_selection_truncate_date(
      window->date.year, window->date.month, now.tm_mday);
}

void date_selection_window_init(DateSelectionWindowData *window, const char *label,
                                GColor color, DateSelectionCompleteCallback complete,
                                void *context) {
  *window = (DateSelectionWindowData){};

  window->complete_callback = complete;
  window->callback_context = context;

  // General window setup
  Window *w = &window->window;
  window_init(w, WINDOW_NAME("Date Selection Window"));
  window_set_user_data(w, window);

  // Selection layer setup
  const DateSelectionSizeConfig * const cfg = prv_config();
  const int num_cells = 3;  // Year, Month, Day
  SelectionLayer *sel = &window->selection_layer;
  selection_layer_init(sel, &GRectZero, num_cells);
  selection_layer_set_cell_width(sel, DateInputIndexYear, cfg->year_cell_width);
  selection_layer_set_cell_width(sel, DateInputIndexMonth, cfg->month_cell_width);
  selection_layer_set_cell_width(sel, DateInputIndexDay, cfg->day_cell_width);
  selection_layer_set_cell_padding(sel, cfg->cell_padding);
  selection_layer_set_inactive_bg_color(sel, GColorDarkGray);
  if (color.a) {
    selection_layer_set_active_bg_color(sel, color);
  }

  layer_set_frame(&sel->layer,
                  &GRect(0, cfg->top_offset, w->layer.bounds.size.w,
                         selection_layer_default_cell_height()));

  selection_layer_set_click_config_onto_window(sel, w);
  selection_layer_set_callbacks(sel, window, (SelectionLayerCallbacks){
    .get_cell_text = prv_get_cell_text,
    .complete = prv_handle_complete,
    .increment = prv_handle_inc,
    .decrement = prv_handle_dec,
  });
  layer_add_child(&w->layer, &sel->layer);

  // Label setup
  const GFont header_font = system_theme_get_font_for_default_size(TextStyleFont_Header);
  TextLayer *label_layer = &window->label_text_layer;
  prv_text_layer_init(&w->layer, label_layer, header_font);
  if (label) {
    const int lines = 1;
    const int line_height = fonts_get_font_height(header_font);
    layer_set_frame(&label_layer->layer,
                    &GRect(0, cfg->label_origin_y, w->layer.bounds.size.w,
                           (lines + 1) * line_height + line_height / 2));
#if PBL_ROUND
    text_layer_enable_screen_text_flow_and_paging(label_layer, 4 /* inset */);
#endif
    text_layer_set_text(label_layer, label);
    layer_set_hidden(&label_layer->layer, false);
  }

  // Status bar setup
  status_bar_layer_init(&window->status_layer);
  status_bar_layer_set_separator_mode(&window->status_layer,
                                      PBL_IF_COLOR_ELSE(OPTION_MENU_STATUS_SEPARATOR_MODE,
                                                        StatusBarLayerSeparatorModeNone));
  layer_add_child(&w->layer, &window->status_layer.layer);
}

void date_selection_window_deinit(DateSelectionWindowData *window) {
  if (window) {
    status_bar_layer_deinit(&window->status_layer);
    selection_layer_deinit(&window->selection_layer);
  }
}
