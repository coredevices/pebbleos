/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/app.h"
#include "applib/app_watch_info.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/window.h"
#include "applib/ui/path_layer.h"
#include "applib/ui/text_layer.h"
#include "applib/graphics/graphics.h"
#include "apps/prf/mfg_test_result.h"
#include "kernel/pbl_malloc.h"
#include "mfg/mfg_info.h"
#include "process_state/app_state/app_state.h"
#include "process_management/pebble_process_md.h"
#include "pbl/util/size.h"

#include <stdbool.h>
#include <stdio.h>

#ifdef CONFIG_BOARD_ASTERIX
const char *const s_model = "P2D";
#elif defined(CONFIG_BOARD_OBELIX)
const char *const s_model = "PT2";
#elif defined(CONFIG_BOARD_GETAFIX)
const char *const s_model = "PR2";
#else
const char *const s_model = "Unknown";
#endif

typedef struct {
  WatchInfoColor color;
  const char *name;
  const char *short_name;
} ColorTable;

const ColorTable s_color_table[] = {
#ifdef CONFIG_BOARD_ASTERIX
  {
    .color = WATCH_INFO_COLOR_COREDEVICES_P2D_BLACK,
    .name = "BLACK",
    .short_name = "BK",
  },
  {
    .color = WATCH_INFO_COLOR_COREDEVICES_P2D_WHITE,
    .name = "WHITE",
    .short_name = "WH",
  }
#elif defined(CONFIG_BOARD_OBELIX)
  {
    .color = WATCH_INFO_COLOR_COREDEVICES_PT2_BLACK_GREY,
    .name = "BLACK/GREY",
    .short_name = "BG",
  },
  {
    .color = WATCH_INFO_COLOR_COREDEVICES_PT2_BLACK_RED,
    .name = "BLACK/RED",
    .short_name = "BR",
  },
  {
    .color = WATCH_INFO_COLOR_COREDEVICES_PT2_SILVER_BLUE,
    .name = "SILVER/BLUE",
    .short_name = "SB",
  },
  {
    .color = WATCH_INFO_COLOR_COREDEVICES_PT2_SILVER_GREY,
    .name = "SILVER/GREY",
    .short_name = "SG",
  },
#elif defined(CONFIG_BOARD_GETAFIX)
  {
    .color = WATCH_INFO_COLOR_COREDEVICES_PR2_BLACK_20,
    .name = "BLACK-20MM",
    .short_name = "BK20",
  },
  {
    .color = WATCH_INFO_COLOR_COREDEVICES_PR2_SILVER_14,
    .name = "SILVER-14MM",
    .short_name = "SV14",
  },
  {
    .color = WATCH_INFO_COLOR_COREDEVICES_PR2_SILVER_20,
    .name = "SILVER-20MM",
    .short_name = "SV20",
  },
  {
    .color = WATCH_INFO_COLOR_COREDEVICES_PR2_GOLD_14,
    .name = "GOLD-14MM",
    .short_name = "GD14",
  },
#endif
};

typedef struct {
  Window window;

  TextLayer title;
  TextLayer color;
  TextLayer status;

  PathLayer up_arrow;
  PathLayer down_arrow;

#ifdef PBL_COLOR
  Layer color_preview;
#endif

  int selected_color_index;
  char color_text[32];  // Buffer for "NAME (SN)" format
} AppData;

#ifdef PBL_COLOR
// Helper function to convert WatchInfoColor to GColor for display
static void prv_get_display_colors(WatchInfoColor watch_color, GColor *color1, GColor *color2) {
  // Default to black for unknown colors
  *color1 = GColorBlack;
  *color2 = GColorBlack;

  switch (watch_color) {
    // Single color options (C2D)
    case WATCH_INFO_COLOR_COREDEVICES_P2D_BLACK:
      *color1 = GColorBlack;
      *color2 = GColorBlack;
      break;
    case WATCH_INFO_COLOR_COREDEVICES_P2D_WHITE:
      *color1 = GColorWhite;
      *color2 = GColorWhite;
      break;

    // Two color options (CT2)
    case WATCH_INFO_COLOR_COREDEVICES_PT2_BLACK_GREY:
      *color1 = GColorBlack;
      *color2 = GColorLightGray;
      break;
    case WATCH_INFO_COLOR_COREDEVICES_PT2_BLACK_RED:
      *color1 = GColorBlack;
      *color2 = GColorRed;
      break;
    case WATCH_INFO_COLOR_COREDEVICES_PT2_SILVER_BLUE:
      *color1 = GColorLightGray;  // Silver approximation
      *color2 = GColorBlue;
      break;
    case WATCH_INFO_COLOR_COREDEVICES_PT2_SILVER_GREY:
      *color1 = GColorLightGray;  // Silver approximation
      *color2 = GColorDarkGray;
      break;

    // PR2
    case WATCH_INFO_COLOR_COREDEVICES_PR2_BLACK_20:
      *color1 = GColorBlack;
      *color2 = GColorBlack;
      break;
    case WATCH_INFO_COLOR_COREDEVICES_PR2_SILVER_14:
      *color1 = GColorLightGray;  // Silver approximation
      *color2 = GColorLightGray;
      break;
    case WATCH_INFO_COLOR_COREDEVICES_PR2_SILVER_20:
      *color1 = GColorLightGray;  // Silver approximation
      *color2 = GColorLightGray;
      break;
    case WATCH_INFO_COLOR_COREDEVICES_PR2_GOLD_14:
      *color1 = GColorYellow;  // Gold approximation
      *color2 = GColorYellow;
      break;

    default:
      break;
  }
}

static void prv_color_preview_update_proc(Layer *layer, GContext *ctx) {
  AppData *app_data = app_state_get_user_data();

  if (app_data->selected_color_index < 0 ||
      app_data->selected_color_index >= (int)ARRAY_LENGTH(s_color_table)) {
    return;
  }

  GRect bounds;
  layer_get_bounds(layer, &bounds);
  WatchInfoColor watch_color = s_color_table[app_data->selected_color_index].color;

  GColor color1, color2;
  prv_get_display_colors(watch_color, &color1, &color2);

  // Draw a square split diagonally if two different colors
  if (gcolor_equal(color1, color2)) {
    // Single color - fill entire square
    graphics_context_set_fill_color(ctx, color1);
    graphics_fill_rect(ctx, &bounds);
  } else {
    // Two colors - split diagonally
    // Top triangle (color1)
    graphics_context_set_fill_color(ctx, color1);
    for (int y = 0; y < bounds.size.h; y++) {
      GRect top_rect = GRect(bounds.origin.x, bounds.origin.y + y,
                             bounds.size.w - y, 1);
      graphics_fill_rect(ctx, &top_rect);
    }

    // Bottom triangle (color2)
    graphics_context_set_fill_color(ctx, color2);
    for (int y = 0; y < bounds.size.h; y++) {
      GRect bottom_rect = GRect(bounds.origin.x + bounds.size.w - y,
                                bounds.origin.y + y, y, 1);
      graphics_fill_rect(ctx, &bottom_rect);
    }
  }

  // Draw border
  graphics_context_set_stroke_color(ctx, GColorBlack);
  graphics_draw_rect(ctx, &bounds);
}
#endif

static void prv_up_click_handler(ClickRecognizerRef recognizer, void *data) {
  AppData *app_data = app_state_get_user_data();

  if (ARRAY_LENGTH(s_color_table) == 0) {
    return;
  }

  if (app_data->selected_color_index == 0) {
    app_data->selected_color_index = ARRAY_LENGTH(s_color_table) - 1;
  } else {
    app_data->selected_color_index--;
  }
  snprintf(app_data->color_text, sizeof(app_data->color_text), "%s (%s)",
           s_color_table[app_data->selected_color_index].name,
           s_color_table[app_data->selected_color_index].short_name);
  text_layer_set_text(&app_data->color, app_data->color_text);
#ifdef PBL_COLOR
  layer_mark_dirty(&app_data->color_preview);
#endif
}

static void prv_down_click_handler(ClickRecognizerRef recognizer, void *data) {
  AppData *app_data = app_state_get_user_data();

  if (ARRAY_LENGTH(s_color_table) == 0) {
    return;
  }

  if (app_data->selected_color_index == (int)ARRAY_LENGTH(s_color_table) - 1) {
    app_data->selected_color_index = 0;
  } else {
    app_data->selected_color_index++;
  }
  snprintf(app_data->color_text, sizeof(app_data->color_text), "%s (%s)",
           s_color_table[app_data->selected_color_index].name,
           s_color_table[app_data->selected_color_index].short_name);
  text_layer_set_text(&app_data->color, app_data->color_text);
#ifdef PBL_COLOR
  layer_mark_dirty(&app_data->color_preview);
#endif
}

static void prv_close_timer_callback(void *cb_data) {
  app_window_stack_pop(false);
}

static void prv_select_click_handler(ClickRecognizerRef recognizer, void *data) {
  AppData *app_data = app_state_get_user_data();
  char model[MFG_INFO_MODEL_STRING_LENGTH];

  if (ARRAY_LENGTH(s_color_table) == 0 || app_data->selected_color_index == -1) {
    return;
  }

  snprintf(model, sizeof(model), "%s-%s",
           s_model, s_color_table[app_data->selected_color_index].short_name);

  mfg_info_set_model(model);
  mfg_info_set_watch_color(s_color_table[app_data->selected_color_index].color);

  mfg_test_result_report(MfgTestId_ProgramColor, true,
                         s_color_table[app_data->selected_color_index].color);
  text_layer_set_text(&app_data->status, "PROGRAMMED!");
  app_timer_register(3000, prv_close_timer_callback, NULL);
}

static void prv_config_provider(void *data) {
  window_single_click_subscribe(BUTTON_ID_UP, prv_up_click_handler);
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_select_click_handler);
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_down_click_handler);
}

static void prv_handle_init(void) {
  AppData *data = app_malloc_check(sizeof(AppData));
  *data = (AppData) {
    .selected_color_index = -1,
  };

  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, "");
  window_set_fullscreen(window, true);
  window_set_click_config_provider(window, prv_config_provider);

  TextLayer *title = &data->title;
  text_layer_init(title, &window->layer.bounds);
  text_layer_set_font(title, fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
  text_layer_set_text_alignment(title, GTextAlignmentCenter);
  text_layer_set_text(title, "PROGRAM COLOR");
  layer_add_child(&window->layer, &title->layer);

  // Create up arrow (pointing up)
  static GPoint UP_ARROW_POINTS[] = {{0, 10}, {7, 0}, {14, 10}};
  static const GPathInfo UP_ARROW_PATH_INFO = {
    .num_points = ARRAY_LENGTH(UP_ARROW_POINTS),
    .points = UP_ARROW_POINTS
  };
  PathLayer *up_arrow = &data->up_arrow;
  path_layer_init(up_arrow, &UP_ARROW_PATH_INFO);
  path_layer_set_fill_color(up_arrow, GColorBlack);
  path_layer_set_stroke_color(up_arrow, GColorBlack);
  layer_set_frame(&up_arrow->layer,
                  &GRect((window->layer.bounds.size.w / 2) - 7, 40, 14, 10));
  layer_add_child(&window->layer, &up_arrow->layer);

#ifdef PBL_COLOR
  // Create color preview square above the color text
  Layer *color_preview = &data->color_preview;
  const int preview_size = 40;
  layer_init(color_preview,
             &GRect((window->layer.bounds.size.w / 2) - (preview_size / 2), 55,
                    preview_size, preview_size));
  layer_set_update_proc(color_preview, prv_color_preview_update_proc);
  layer_add_child(&window->layer, color_preview);
#endif

  TextLayer *color = &data->color;
  text_layer_init(color,
                  &GRect(5, 100,
                         window->layer.bounds.size.w - 10, 28));
  text_layer_set_font(color, fonts_get_system_font(FONT_KEY_GOTHIC_24));
  text_layer_set_text_alignment(color, GTextAlignmentCenter);
  if (ARRAY_LENGTH(s_color_table) > 0) {
    data->selected_color_index = 0;
    snprintf(data->color_text, sizeof(data->color_text), "%s (%s)",
             s_color_table[0].name, s_color_table[0].short_name);
    text_layer_set_text(color, data->color_text);
  } else {
    text_layer_set_text(color, "NO COLORS AVAILABLE");
  }
  layer_add_child(&window->layer, &color->layer);

  // Create down arrow (pointing down)
  static GPoint DOWN_ARROW_POINTS[] = {{0, 0}, {7, 10}, {14, 0}};
  static const GPathInfo DOWN_ARROW_PATH_INFO = {
    .num_points = ARRAY_LENGTH(DOWN_ARROW_POINTS),
    .points = DOWN_ARROW_POINTS
  };
  PathLayer *down_arrow = &data->down_arrow;
  path_layer_init(down_arrow, &DOWN_ARROW_PATH_INFO);
  path_layer_set_fill_color(down_arrow, GColorBlack);
  path_layer_set_stroke_color(down_arrow, GColorBlack);
  layer_set_frame(&down_arrow->layer,
                  &GRect((window->layer.bounds.size.w / 2) - 7, 133, 14, 10));
  layer_add_child(&window->layer, &down_arrow->layer);

  TextLayer *status = &data->status;
  text_layer_init(status,
                  &GRect(5, 148,
                         window->layer.bounds.size.w - 5, window->layer.bounds.size.h - 148));
  text_layer_set_font(status, fonts_get_system_font(FONT_KEY_GOTHIC_24));
  text_layer_set_text_alignment(status, GTextAlignmentCenter);
  layer_add_child(&window->layer, &status->layer);

  app_window_stack_push(window, true /* Animated */);
}

static void s_main(void) {
  prv_handle_init();

  app_event_loop();
}

const PebbleProcessMd* mfg_program_color_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common.main_func = &s_main,
    // UUID: d5f0a47d-e570-499d-bcaa-fc6d56230038
    .common.uuid = { 0xd5, 0xf0, 0xa4, 0x7d, 0xe5, 0x70, 0x49, 0x9d,
                     0xbc, 0xaa, 0xfc, 0x6d, 0x56, 0x23, 0x00, 0x38 },
    .name = "MfgProgramColor",
  };
  return (const PebbleProcessMd*) &s_app_info;
}

