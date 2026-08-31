/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "menu_layer_legacy2.h"

#include "kernel/pbl_malloc.h"
#include "system/passert.h"

extern void menu_layer_init_scroll_layer_callbacks(MenuLayer *menu_layer);

void menu_layer_legacy2_init(MenuLayer *menu_layer, const GRect *frame) {
  *menu_layer = (MenuLayer){};

  ScrollLayer *scroll_layer = &menu_layer->scroll_layer;
  scroll_layer_init(scroll_layer, frame);
  menu_layer_init_scroll_layer_callbacks(menu_layer);
  scroll_layer_set_context(scroll_layer, menu_layer);

  menu_layer_set_normal_colors(menu_layer, GColorWhite, GColorBlack);
  menu_layer_set_highlight_colors(menu_layer, GColorBlack, GColorWhite);

  InverterLayer *inverter = &menu_layer->inverter;
  inverter_layer_init(inverter, &GRectZero);
  scroll_layer_add_child(scroll_layer, &inverter->layer);
}

MenuLayer* menu_layer_legacy2_create(GRect frame) {
  MenuLayer *layer = task_malloc(sizeof(MenuLayer));
  if (layer) {
    menu_layer_legacy2_init(layer, &frame);
  }
  return layer;
}

void menu_layer_legacy2_set_callbacks(MenuLayer *menu_layer,
                                      void *callback_context,
                                      MenuLayerCallbacksLegacy2 callbacks) {
  menu_layer_set_callbacks(menu_layer, callback_context, &(MenuLayerCallbacks) {
        .get_num_sections = callbacks.get_num_sections,
        .get_num_rows = callbacks.get_num_rows,
        .get_cell_height = callbacks.get_cell_height,
        .get_header_height = callbacks.get_header_height,
        .draw_row = callbacks.draw_row,
        .draw_header = callbacks.draw_header,
        .select_click = callbacks.select_click,
        .select_long_click = callbacks.select_long_click,
        .selection_changed = callbacks.selection_changed,
        .get_separator_height = callbacks.get_separator_height,
        .draw_separator = callbacks.draw_separator,
      });
}

void menu_layer_legacy2_set_callbacks__deprecated(MenuLayer *menu_layer,
                                                  void *callback_context,
                                                  MenuLayerCallbacksLegacy2__deprecated callbacks) {
  menu_layer_set_callbacks(menu_layer, callback_context, &(MenuLayerCallbacks) {
        .get_num_sections = callbacks.get_num_sections,
        .get_num_rows = callbacks.get_num_rows,
        .get_cell_height = callbacks.get_cell_height,
        .get_header_height = callbacks.get_header_height,
        .draw_row = callbacks.draw_row,
        .draw_header = callbacks.draw_header,
        .select_click = callbacks.select_click,
        .select_long_click = callbacks.select_long_click,
        .selection_changed = callbacks.selection_changed,
      });
}
