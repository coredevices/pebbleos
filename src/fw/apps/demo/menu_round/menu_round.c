/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "menu_round.h"

#include "applib/app.h"
#include "applib/graphics/gdraw_command_image.h"
#include "applib/ui/ui.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "resource/resource_ids.auto.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/size.h"

#include <stdio.h>

// Menu Detail
//////////////////

typedef enum {
  MenuLayerStyleTitle = 0,
  MenuLayerStyleTitleAndSubtitle,
  MenuLayerStyleTitleAndIconOnRight,
  MenuLayerStyleTitleAndSubtitleAndValue,
  MenuLayerStyleTitleAndSubtitleAndIcon,
} MenuLayerStyle;

typedef struct {
  char *title;
  char *subtitle;
  char *value;
} MenuDetailRowData;

typedef struct {
  Window window;
  MenuLayer menu_layer;
  StatusBarLayer status_bar_layer;
  MenuLayerStyle style;
} MenuDetailWindowData;


static const MenuDetailRowData menu_detail_row_data_notifications[] = {
  {"Liron Damir", "Late again. Sorry, I'll be on time in the future.", NULL},
  {"Angela Tam", "Late again? Can you be on time for once?", NULL},
  {"Eric Migicovsky", "Friday meeting will be held in the big room.", NULL},
  {"Instagram", "Keep scrolling down.", NULL},
  {"Liron Levak", "That's not my name.", NULL},
  {"Kimberly North West Kardashian", "I broke the Internet again.", NULL},
  {"Henry Damir", "That's not my name.", NULL},
  {"Kevin Conley", "Wubalubadubdub!", NULL},
};

static const MenuDetailRowData menu_detail_row_data_days[] = {
  {"Monday", NULL, NULL},
  {"Tuesday", NULL, NULL},
  {"Wednesday", NULL, NULL},
  {"Thursday", NULL, NULL},
  {"Friday", NULL, NULL},
  {"Saturday", NULL, NULL},
  {"Sunday", NULL, NULL},
};

static const MenuDetailRowData menu_detail_row_data_alarms[] = {
  {"8:00 AM", "Workdays", "ON"},
  {"10:00 AM", "Sat, Sun, Mon", "OFF"},
  {"11:30 AM", "Weekends", "ON"},
  {"5:00 PM", "Weekdays", "ON"},
};

typedef struct {
  const MenuDetailRowData *rows;
  uint16_t num_rows;
  int16_t selected_cell_height;
  int16_t unselected_cell_height;
  GColor highlight_background_color;
} MenuDetailInfo;

static MenuDetailInfo prv_get_row_details_for_style(MenuLayerStyle style) {
  switch (style) {
    case MenuLayerStyleTitle:
      return (MenuDetailInfo) {
        .rows = menu_detail_row_data_notifications,
        .num_rows = ARRAY_LENGTH(menu_detail_row_data_notifications),
        .selected_cell_height = MENU_CELL_ROUND_FOCUSED_SHORT_CELL_HEIGHT,
        .unselected_cell_height = MENU_CELL_ROUND_UNFOCUSED_TALL_CELL_HEIGHT,
        .highlight_background_color = GColorFolly,
      };
    case MenuLayerStyleTitleAndSubtitle:
      return (MenuDetailInfo) {
        .rows = menu_detail_row_data_notifications,
        .num_rows = ARRAY_LENGTH(menu_detail_row_data_notifications),
        .selected_cell_height = MENU_CELL_ROUND_FOCUSED_SHORT_CELL_HEIGHT,
        .unselected_cell_height = MENU_CELL_ROUND_UNFOCUSED_TALL_CELL_HEIGHT,
        .highlight_background_color = GColorIslamicGreen,
      };
    case MenuLayerStyleTitleAndSubtitleAndIcon:
      return (MenuDetailInfo) {
        .rows = menu_detail_row_data_notifications,
        .num_rows = ARRAY_LENGTH(menu_detail_row_data_notifications),
        .selected_cell_height = MENU_CELL_ROUND_FOCUSED_TALL_CELL_HEIGHT,
        .unselected_cell_height = MENU_CELL_ROUND_UNFOCUSED_SHORT_CELL_HEIGHT,
        .highlight_background_color = GColorFolly,
      };
    case MenuLayerStyleTitleAndIconOnRight:
      return (MenuDetailInfo) {
        .rows = menu_detail_row_data_days,
        .num_rows = ARRAY_LENGTH(menu_detail_row_data_days),
        .selected_cell_height = menu_cell_basic_cell_height(),
        .unselected_cell_height = MENU_CELL_ROUND_UNFOCUSED_TALL_CELL_HEIGHT,
        .highlight_background_color = GColorIslamicGreen,
      };
    case MenuLayerStyleTitleAndSubtitleAndValue:
      return (MenuDetailInfo) {
        .rows = menu_detail_row_data_alarms,
        .num_rows = ARRAY_LENGTH(menu_detail_row_data_alarms),
        .selected_cell_height = MENU_CELL_ROUND_FOCUSED_SHORT_CELL_HEIGHT,
        .unselected_cell_height = MENU_CELL_ROUND_UNFOCUSED_TALL_CELL_HEIGHT,
        .highlight_background_color = GColorIslamicGreen,
      };
    default:
      WTF;
  }
}

static int16_t prv_get_cell_height_for_menu_layer(MenuLayer *menu_layer, MenuIndex *cell_index,
                                                  MenuLayerStyle style) {
  const MenuDetailInfo row_details = prv_get_row_details_for_style(style);
  return menu_layer_is_index_selected(menu_layer, cell_index) ?
         row_details.selected_cell_height :
         row_details.unselected_cell_height;
}

static int16_t prv_menu_detail_get_cell_height(struct MenuLayer *menu_layer, MenuIndex *cell_index,
                                               void *context) {
  MenuDetailWindowData *data = context;
  return prv_get_cell_height_for_menu_layer(menu_layer, cell_index, data->style);
}
static uint16_t prv_menu_detail_get_num_rows_callback(MenuLayer *menu_layer,
                                                      uint16_t section_index,
                                                      void *context) {
  MenuDetailWindowData *data = context;
  return prv_get_row_details_for_style(data->style).num_rows;
}

static void prv_menu_detail_draw_row(GContext *ctx, const Layer *cell_layer,
                                     MenuDetailRowData *row_data, MenuLayerStyle style) {
  const GFont title_font = fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD);
  switch (style) {
    case MenuLayerStyleTitle: {
      menu_cell_basic_draw_custom(ctx, cell_layer, title_font, row_data->title, title_font, NULL,
                                  NULL, NULL, NULL, false /* icon_on_right */,
                                  GTextOverflowModeWordWrap);
      break;
    }
    case MenuLayerStyleTitleAndSubtitle: {
      char *subtitle = menu_cell_layer_is_highlighted(cell_layer) ? row_data->subtitle : NULL;
      menu_cell_basic_draw(ctx, cell_layer, row_data->title, subtitle, NULL);
      break;
    }
    case MenuLayerStyleTitleAndIconOnRight: {
      GBitmap *radio_button = gbitmap_create_with_resource(RESOURCE_ID_CHECKED_RADIO_BUTTON);
      menu_cell_basic_draw_icon_right(ctx, cell_layer, row_data->title, row_data->subtitle,
                                      radio_button);
      gbitmap_destroy(radio_button);
      break;
    }
    case MenuLayerStyleTitleAndSubtitleAndValue: {
      const GFont subtitle_font = fonts_get_system_font(FONT_KEY_GOTHIC_14);
      menu_cell_basic_draw_custom(ctx, cell_layer, title_font, row_data->title, title_font,
                                  row_data->value, subtitle_font, row_data->subtitle, NULL, false,
                                  GTextOverflowModeFill);
      break;
    }
    case MenuLayerStyleTitleAndSubtitleAndIcon: {
      GBitmap *icon_bitmap = gbitmap_create_with_resource(RESOURCE_ID_MENU_ICON_TICTOC_WATCH);
      menu_cell_basic_draw(ctx, cell_layer, row_data->title, row_data->subtitle, icon_bitmap);
      gbitmap_destroy(icon_bitmap);
      break;
    }
    default:
      WTF;
  }
}

static void prv_menu_detail_draw_row_callback(GContext* ctx, const Layer *cell_layer,
                                               MenuIndex *cell_index, void *context) {
  MenuDetailWindowData *data = context;
  const MenuDetailInfo menu_info = prv_get_row_details_for_style(data->style);
  MenuDetailRowData row_data = menu_info.rows[cell_index->row];
  prv_menu_detail_draw_row(ctx, cell_layer, &row_data, data->style);
}

static void prv_detail_window_load(Window *window) {
  MenuDetailWindowData *data = window_get_user_data(window);

  MenuLayer *menu_layer = &data->menu_layer;
  const GRect menu_layer_frame = grect_inset_internal(window->layer.bounds, 0,
                                                      STATUS_BAR_LAYER_HEIGHT);
  menu_layer_init(menu_layer, &menu_layer_frame);
  menu_layer_set_callbacks(menu_layer, data, &(MenuLayerCallbacks) {
    .get_cell_height = prv_menu_detail_get_cell_height,
    .get_num_rows = prv_menu_detail_get_num_rows_callback,
    .draw_row = prv_menu_detail_draw_row_callback,
  });
  menu_layer_set_click_config_onto_window(menu_layer, window);
  menu_layer_set_selected_index(menu_layer, MenuIndex(0, 1), MenuRowAlignCenter, false);
  const MenuDetailInfo menu_info = prv_get_row_details_for_style(data->style);
  menu_layer_set_highlight_colors(menu_layer, menu_info.highlight_background_color, GColorWhite);
  layer_add_child(&window->layer, menu_layer_get_layer(menu_layer));

  StatusBarLayer *status_bar = &data->status_bar_layer;
  status_bar_layer_init(status_bar);
  status_bar_layer_set_colors(status_bar, GColorClear, GColorBlack);
  layer_add_child(&window->layer, &status_bar->layer);
}

static void prv_detail_window_unload(Window *window) {
  MenuDetailWindowData *data = window_get_user_data(window);
  menu_layer_deinit(&data->menu_layer);
  app_free(data);
}

static void prv_push_detail_window(MenuLayerStyle menu_layer_style) {
  MenuDetailWindowData *data = app_zalloc_check(sizeof(MenuDetailWindowData));
  data->style = menu_layer_style;

  Window *window = &data->window;
  window_init(window, WINDOW_NAME("MenuLayer Round Demo Detail Menu"));
  window_set_user_data(window, data);
  window_set_window_handlers(window, &(WindowHandlers) {
    .load = prv_detail_window_load,
    .unload = prv_detail_window_unload,
  });
  const bool animated = true;
  app_window_stack_push(window, animated);
}

// Menu Chooser
//////////////////

typedef struct {
  Window window;
  MenuLayer menu_layer;
  StatusBarLayer status_bar_layer;
} MenuChooserData;

typedef struct {
  char *title;
  MenuLayerStyle style;
} MenuChooserRowData;

static const MenuChooserRowData menu_chooser_row_data[] = {
  {"Title Only", MenuLayerStyleTitle},
  {"Title & Subtitle", MenuLayerStyleTitleAndSubtitle},
  {"Title & Right Icon", MenuLayerStyleTitleAndIconOnRight},
  {"Title, Sub, Value", MenuLayerStyleTitleAndSubtitleAndValue},
  {"Title, Sub, Icon", MenuLayerStyleTitleAndSubtitleAndIcon},
};

static int16_t prv_menu_chooser_get_cell_height(struct MenuLayer *menu_layer, MenuIndex *cell_index,
                                                void *context) {
  return prv_get_cell_height_for_menu_layer(menu_layer, cell_index, MenuLayerStyleTitle);
}

static uint16_t prv_menu_chooser_get_num_rows_callback(struct MenuLayer *menu_layer,
                                                       uint16_t section_index,
                                                       void *context) {
  return ARRAY_LENGTH(menu_chooser_row_data);
}

static void prv_menu_chooser_draw_row_callback(GContext* ctx, const Layer *cell_layer,
                                               MenuIndex *cell_index, void *context) {
  MenuChooserRowData row_data = menu_chooser_row_data[cell_index->row];
  menu_cell_basic_draw(ctx, cell_layer, row_data.title, NULL, NULL);
}

static void prv_menu_chooser_select_callback(MenuLayer *menu_layer, MenuIndex *cell_index,
                                             void *context) {
  prv_push_detail_window(menu_chooser_row_data[cell_index->row].style);
}

static void prv_window_load(Window *window) {
  MenuChooserData *data = window_get_user_data(window);

  MenuLayer *menu_layer = &data->menu_layer;
  const GRect menu_layer_frame = grect_inset_internal(window->layer.bounds, 0,
                                                      STATUS_BAR_LAYER_HEIGHT);
  menu_layer_init(menu_layer, &menu_layer_frame);
  menu_layer_set_callbacks(menu_layer, data, &(MenuLayerCallbacks) {
    .get_cell_height = prv_menu_chooser_get_cell_height,
    .get_num_rows = prv_menu_chooser_get_num_rows_callback,
    .draw_row = prv_menu_chooser_draw_row_callback,
    .select_click = prv_menu_chooser_select_callback,
  });
  menu_layer_set_click_config_onto_window(menu_layer, window);
  menu_layer_set_selected_index(menu_layer, MenuIndex(0, 1), MenuRowAlignCenter, false);
  menu_layer_set_highlight_colors(menu_layer, GColorPictonBlue, GColorWhite);
  layer_add_child(&window->layer, menu_layer_get_layer(menu_layer));

  StatusBarLayer *status_bar = &data->status_bar_layer;
  status_bar_layer_init(status_bar);
  status_bar_layer_set_colors(status_bar, GColorClear, GColorBlack);
  layer_add_child(&window->layer, &status_bar->layer);
}

static void prv_window_unload(Window *window) {
  MenuChooserData *data = window_get_user_data(window);
  menu_layer_deinit(&data->menu_layer);
}

// App boilerplate
////////////////////

static void prv_init(void) {
  MenuChooserData *data = app_zalloc_check(sizeof(MenuChooserData));

  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, WINDOW_NAME("MenuLayer Round Demo Chooser Menu"));
  window_set_user_data(window, data);
  window_set_window_handlers(window, &(WindowHandlers) {
    .load = prv_window_load,
    .unload = prv_window_unload,
  });
  const bool animated = true;
  app_window_stack_push(window, animated);
}

static void prv_deinit(void) {
  MenuChooserData *data = app_state_get_user_data();
  app_free(data);
}

static void s_main(void) {
  prv_init();
  app_event_loop();
  prv_deinit();
}

const PebbleProcessMd* menu_round_app_get_info() {
  static const PebbleProcessMdSystem s_app_info = {
    .common.main_func = &s_main,
    .name = "MenuLayer Round Demo"
  };
  return (const PebbleProcessMd*) &s_app_info;
}
