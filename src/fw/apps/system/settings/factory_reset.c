/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "factory_reset.h"

#include "applib/app_timer.h"
#include "applib/fonts/fonts.h"
#include "applib/ui/action_bar_layer.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/window_stack_private.h"
#include "applib/ui/ui.h"
#include "apps/system/timeline/peek_layer.h"
#include "kernel/pbl_malloc.h"
#include "kernel/ui/kernel_ui.h"
#include "kernel/ui/system_icons.h"
#include "kernel/util/factory_reset.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/system_task.h"
#include "bluetooth.h"
#include "system/logging.h"

#define MESSAGE_BUF_SIZE 96

typedef struct ConfirmUIData {
  Window window;
  ActionBarLayer action_bar;
  TextLayer msg_text_layer;
  TextLayer forget_text_layer;
  PeekLayer resetting_layer;
  char msg_text_layer_buffer[MESSAGE_BUF_SIZE];
  GBitmap *action_bar_icon_check;
  GBitmap *action_bar_icon_x;
} ConfirmUIData;

//! Wipe registry + Reboot
static void start_factory_reset(void *data) {
  factory_reset(false /* should_shutdown */);
}

static void prv_lockout_back_button(Window *window) {
  window_set_overrides_back_button(window, true);
  window_set_click_config_provider(window, NULL);
}

static void confirm_click_handler(ClickRecognizerRef recognizer, Window *window) {
  ConfirmUIData *data = window_get_user_data(window);

  // Need to lock-out inputs after starting the factory reset.
  prv_lockout_back_button(window);

  PeekLayer *peek_layer = &data->resetting_layer;
  peek_layer_init(peek_layer, &window->layer.bounds);
  peek_layer_set_title_font(peek_layer, fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
  TimelineResourceInfo timeline_res = {
    .res_id = TIMELINE_RESOURCE_GENERIC_WARNING,
  };
  peek_layer_set_icon(peek_layer, &timeline_res);
  peek_layer_set_title(peek_layer, i18n_get("Resetting...", data));
  peek_layer_set_background_color(peek_layer, PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite));
  peek_layer_play(peek_layer);
  layer_add_child(&window->layer, &peek_layer->layer);

  // give it a chance to animate
  const uint32_t factory_reset_start_delay = 100;
  app_timer_register(PEEK_LAYER_UNFOLD_DURATION + factory_reset_start_delay,
                     start_factory_reset, NULL);
}

//! Wipe registry + Enter Standby (for factory)
static void confirm_long_click_handler(ClickRecognizerRef recognizer, Window *window) {
  // Need to lock-out inputs after starting the factory reset.
  prv_lockout_back_button(window);

  factory_reset(true /* should_shutdown */);
}

static void decline_click_handler(ClickRecognizerRef recognizer, Window *window) {
  const bool animated = true;
  app_window_stack_pop(animated);
  (void)recognizer;
  (void)window;
}

static void config_provider(Window *window) {
  window_single_click_subscribe(BUTTON_ID_UP, (ClickHandler) confirm_click_handler);
  window_long_click_subscribe(BUTTON_ID_UP, 1200, (ClickHandler) confirm_long_click_handler, NULL);
  window_single_click_subscribe(BUTTON_ID_DOWN, (ClickHandler) decline_click_handler);
  (void)window;
}

static void prv_window_load(Window *window) {
  ConfirmUIData *data = window_get_user_data(window);
  const GRect *root_layer_bounds = &window_get_root_layer(window)->bounds;
  const int16_t width = root_layer_bounds->size.w - ACTION_BAR_WIDTH;
  const uint16_t content_x_start = action_bar_is_on_right() ? 0 : ACTION_BAR_WIDTH;

  const uint16_t x_margin_px = PBL_IF_ROUND_ELSE(6, 3);
  const uint16_t msg_text_y_offset_px = PBL_IF_ROUND_ELSE(15, 0);
  const uint16_t msg_text_max_height_px = root_layer_bounds->size.h - msg_text_y_offset_px;

  const GTextAlignment alignment = PBL_IF_ROUND_ELSE(
      action_bar_is_on_right() ? GTextAlignmentRight : GTextAlignmentLeft, GTextAlignmentLeft);
  const GTextOverflowMode overflow_mode = GTextOverflowModeTrailingEllipsis;
  const GColor text_color = PBL_IF_COLOR_ELSE(GColorWhite, GColorBlack);

  TextLayer *msg_text_layer = &data->msg_text_layer;
  GRect msg_text_frame = (GRect) {
    .origin = GPoint(content_x_start + x_margin_px, msg_text_y_offset_px),
    .size =  GSize(width - (2 * x_margin_px), msg_text_max_height_px)
  };
  text_layer_init_with_parameters(msg_text_layer, &msg_text_frame,
                                  i18n_get("Perform factory reset?", data),
                                  fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD), text_color,
                                  GColorClear, alignment, overflow_mode);
  layer_add_child(&window->layer, &msg_text_layer->layer);
#if PBL_ROUND
  const uint8_t text_flow_inset = 8;
  text_layer_enable_screen_text_flow_and_paging(msg_text_layer, text_flow_inset);
#endif

  // handle different title heights gracefully
  GContext *ctx = graphics_context_get_current_context();
  const uint16_t msg_text_height_px = text_layer_get_content_size(ctx, msg_text_layer).h;
  const int text_spacing = 7;
  const uint16_t forget_text_y_offset_px = msg_text_y_offset_px + msg_text_height_px + text_spacing;

  TextLayer *forget_text_layer = &data->forget_text_layer;
  const GRect forget_text_frame = (GRect) {
    .origin = GPoint(content_x_start + x_margin_px, forget_text_y_offset_px),
    .size =  GSize(width - (2 * x_margin_px), root_layer_bounds->size.h - forget_text_y_offset_px)
  };
  text_layer_init_with_parameters(forget_text_layer, &forget_text_frame,
                                  i18n_get(BT_FORGET_PAIRING_STR, data),
                                  fonts_get_system_font(FONT_KEY_GOTHIC_18), text_color,
                                  GColorClear, alignment, overflow_mode);
  layer_add_child(&window->layer, &forget_text_layer->layer);
#if PBL_ROUND
  text_layer_enable_screen_text_flow_and_paging(forget_text_layer, text_flow_inset);
#endif

  // Action bar:
  ActionBarLayer *action_bar = &data->action_bar;
  action_bar_layer_init(action_bar);
  action_bar_layer_set_context(action_bar, window);
  action_bar_layer_add_to_window(action_bar, window);
  action_bar_layer_set_click_config_provider(action_bar, (ClickConfigProvider) config_provider);
  action_bar_layer_set_icon(action_bar, BUTTON_ID_UP, data->action_bar_icon_check);
  action_bar_layer_set_icon(action_bar, BUTTON_ID_DOWN, data->action_bar_icon_x);
}

static void prv_window_unload(Window *window) {
  ConfirmUIData *data = window_get_user_data(window);
  gbitmap_destroy(data->action_bar_icon_check);
  gbitmap_destroy(data->action_bar_icon_x);
  i18n_free_all(data);
  app_free(data);
}

void settings_factory_reset_window_push(void) {
  ConfirmUIData *data = (ConfirmUIData*)app_malloc_check(sizeof(ConfirmUIData));
  *data = (ConfirmUIData){};

  Window *window = &data->window;
  window_init(window, "Settings Factory Reset");
  window_set_window_handlers(window, &(WindowHandlers) {
    .load = prv_window_load,
    .unload = prv_window_unload,
  });
#if PBL_COLOR
  window_set_background_color(window, GColorCobaltBlue);
#endif
  window_set_user_data(window, data);
  data->action_bar_icon_check = gbitmap_create_with_resource(RESOURCE_ID_ACTION_BAR_ICON_CHECK);
  data->action_bar_icon_x = gbitmap_create_with_resource(RESOURCE_ID_ACTION_BAR_ICON_X);
  const bool animated = true;
  app_window_stack_push(window, animated);
}

#undef MESSAGE_BUF_SIZE
