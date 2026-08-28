/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "actionable_dialog.h"

#include "applib/applib_malloc.auto.h"
#include "applib/fonts/fonts.h"
#include "applib/ui/bitmap_layer.h"
#include "applib/ui/dialogs/dialog.h"
#include "applib/ui/dialogs/dialog_private.h"
#include "applib/ui/kino/kino_reel/scale_segmented.h"
#include "applib/ui/layer.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window.h"
#include "kernel/ui/kernel_ui.h"
#include "resource/resource_ids.auto.h"
#include "system/passert.h"

#include <limits.h>
#include <string.h>

static void prv_actionable_dialog_load(Window *window) {
  ActionableDialog *actionable_dialog = window_get_user_data(window);
  Dialog *dialog = &actionable_dialog->dialog;
  Layer *window_root_layer = window_get_root_layer(window);

  // Ownership of icon is taken over by KinoLayer in dialog_init_icon_layer() call below
  KinoReel *icon = dialog_create_icon(dialog);
  const GSize icon_size = icon ? kino_reel_get_size(icon) : GSizeZero;

  const GRect *bounds = &window_root_layer->bounds;
  const uint16_t left_margin_px = PBL_IF_RECT_ELSE(5, 0);
  const uint16_t content_and_action_bar_horizontal_spacing = PBL_IF_RECT_ELSE(5, 7);
  const uint16_t content_x_start = action_bar_is_on_right() ? 0 : ACTION_BAR_WIDTH;
  const uint16_t right_margin_px = ACTION_BAR_WIDTH +
                                              content_and_action_bar_horizontal_spacing;
  const GFont dialog_text_font = fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD);
  const int max_text_line_height_px = 2 * fonts_get_font_height(dialog_text_font) + 8;
  const uint16_t icon_text_spacing_px = 4;

  uint16_t x = 0;
  uint16_t y = 0;
  uint16_t w = PBL_IF_RECT_ELSE(bounds->size.w - ACTION_BAR_WIDTH, bounds->size.w);
  uint16_t h = STATUS_BAR_LAYER_HEIGHT;

  if (dialog->show_status_layer) {
    dialog_add_status_bar_layer(dialog, &GRect(x, y, w, h));
  }

  x = content_x_start + left_margin_px;
  w = bounds->size.w - left_margin_px - right_margin_px;

  GTextAttributes *text_attributes = NULL;
#if PBL_ROUND
  // Create a GTextAttributes for the TextLayer. Note that the matching
  // graphics_text_attributes_destroy() will not need to be called here, as the ownership
  // of text_attributes will be transferred to the TextLayer we assign it to.
  text_attributes = graphics_text_attributes_create();
  graphics_text_attributes_enable_screen_text_flow(text_attributes, 8);
#endif

  // Probe text height to vertically center icon + text in the available space
  GContext *ctx = graphics_context_get_current_context();
  const GTextAlignment text_alignment = PBL_IF_RECT_ELSE(GTextAlignmentCenter,
      action_bar_is_on_right() ? GTextAlignmentRight : GTextAlignmentLeft);
  const GRect probe_rect = GRect(x, 0, w, max_text_line_height_px);
  const uint16_t text_height = graphics_text_layout_get_max_used_size(ctx,
                                                                      dialog->buffer,
                                                                      dialog_text_font,
                                                                      probe_rect,
                                                                      GTextOverflowModeWordWrap,
                                                                      text_alignment,
                                                                      text_attributes).h;

  const uint16_t status_bar_height = dialog->show_status_layer ? STATUS_BAR_LAYER_HEIGHT : 0;
  const uint16_t available_height = bounds->size.h - status_bar_height;
  const uint16_t spacing = icon ? icon_text_spacing_px : 0;
  const uint16_t content_height = icon_size.h + spacing + text_height;
  uint16_t icon_top_margin_px = status_bar_height +
      (available_height > content_height ? (available_height - content_height) / 2 : 0);
  uint16_t text_top_margin_px = icon_top_margin_px + icon_size.h + spacing;

  y = text_top_margin_px;
  h = bounds->size.h - y;

  // Set up the text.
  TextLayer *text_layer = &dialog->text_layer;
  text_layer_init_with_parameters(text_layer, &GRect(x, y, w, h),
                                  dialog->buffer, dialog_text_font,
                                  dialog->text_color, GColorClear, text_alignment,
                                  GTextOverflowModeWordWrap);
  if (text_attributes) {
    text_layer->should_cache_layout = true;
    text_layer->layout_cache = text_attributes;
  }

  layer_add_child(&window->layer, &text_layer->layer);

  // Action bar. If the user hasn't given a custom action bar, we'll create one of the preset
  // types.
  if (actionable_dialog->action_bar_type != DialogActionBarCustom) {
    actionable_dialog->action_bar = action_bar_layer_create();
    action_bar_layer_set_click_config_provider(actionable_dialog->action_bar,
                                               actionable_dialog->config_provider);
  }
  ActionBarLayer *action_bar = actionable_dialog->action_bar;
  if (actionable_dialog->action_bar_type == DialogActionBarConfirm) {
#if !defined(CONFIG_RECOVERY_FW)
    actionable_dialog->select_icon = gbitmap_create_with_resource(
        RESOURCE_ID_ACTION_BAR_ICON_CHECK);
#endif
    action_bar_layer_set_context(action_bar, window);
    action_bar_layer_set_icon(action_bar, BUTTON_ID_SELECT, actionable_dialog->select_icon);
  } else if (actionable_dialog->action_bar_type == DialogActionBarDecline) {
#if !defined(CONFIG_RECOVERY_FW)
    actionable_dialog->select_icon = gbitmap_create_with_resource(RESOURCE_ID_ACTION_BAR_ICON_X);
#endif
    action_bar_layer_set_context(action_bar, window);
    action_bar_layer_set_icon(action_bar, BUTTON_ID_SELECT, actionable_dialog->select_icon);
  } else if (actionable_dialog->action_bar_type == DialogActionBarConfirmDecline) {
#if !defined(CONFIG_RECOVERY_FW)
    actionable_dialog->up_icon = gbitmap_create_with_resource(RESOURCE_ID_ACTION_BAR_ICON_CHECK);
    actionable_dialog->down_icon = gbitmap_create_with_resource(RESOURCE_ID_ACTION_BAR_ICON_X);
#endif
    action_bar_layer_set_icon(action_bar, BUTTON_ID_UP, actionable_dialog->up_icon);
    action_bar_layer_set_icon(action_bar, BUTTON_ID_DOWN, actionable_dialog->down_icon);
    action_bar_layer_set_context(action_bar, window);
  }
  action_bar_layer_add_to_window(action_bar, window);

  // Icon
  // On rectangular displays we just center it horizontally b/w the left edge of the display and
  // the left edge of the action bar
#if PBL_RECT
  x = content_x_start + (grect_get_max_x(bounds) - ACTION_BAR_WIDTH - icon_size.w) / 2;
#else
  // On round displays we align it adjacent to the action bar
  x = action_bar_is_on_right()
      ? (grect_get_max_x(bounds) - ACTION_BAR_WIDTH - content_and_action_bar_horizontal_spacing -
         icon_size.w)
      : (content_x_start + content_and_action_bar_horizontal_spacing);
#endif

  y = icon_top_margin_px;

  if (dialog_init_icon_layer(dialog, icon, GPoint(x, y), true)) {
    layer_add_child(window_root_layer, &dialog->icon_layer.layer);
  }

  dialog_load(&actionable_dialog->dialog);
}

static void prv_actionable_dialog_appear(Window *window) {
  ActionableDialog *actionable_dialog = window_get_user_data(window);
  Dialog *dialog = actionable_dialog_get_dialog(actionable_dialog);
  dialog_appear(dialog);
}

static void prv_actionable_dialog_unload(Window *window) {
  ActionableDialog *actionable_dialog = window_get_user_data(window);
  dialog_unload(&actionable_dialog->dialog);

  // Destroy action bar if it was a predefined type. If the action bar is custom, it is the user's
  // responsibility to free it.
  if (actionable_dialog->action_bar_type != DialogActionBarCustom) {
    action_bar_layer_destroy(actionable_dialog->action_bar);
    if (actionable_dialog->action_bar_type == DialogActionBarConfirmDecline) {
      gbitmap_destroy(actionable_dialog->up_icon);
      gbitmap_destroy(actionable_dialog->down_icon);
    } else { // DialogActionBarConfirm || DialogActionBarDecline
      gbitmap_destroy(actionable_dialog->select_icon);
    }
  }
  if (actionable_dialog->dialog.destroy_on_pop) {
    applib_free(actionable_dialog);
  }
}

Dialog *actionable_dialog_get_dialog(ActionableDialog *actionable_dialog) {
  return &actionable_dialog->dialog;
}

void actionable_dialog_push(ActionableDialog *actionable_dialog, WindowStack *window_stack) {
  dialog_push(&actionable_dialog->dialog, window_stack);
}

void app_actionable_dialog_push(ActionableDialog *actionable_dialog) {
  app_dialog_push(&actionable_dialog->dialog);
}

void actionable_dialog_pop(ActionableDialog *actionable_dialog) {
  dialog_pop(&actionable_dialog->dialog);
}

void actionable_dialog_init(ActionableDialog *actionable_dialog, const char *dialog_name) {
  PBL_ASSERTN(actionable_dialog);
  *actionable_dialog = (ActionableDialog){};

  dialog_init(&actionable_dialog->dialog, dialog_name);
  Window *window = &actionable_dialog->dialog.window;
  window_set_window_handlers(window, &(WindowHandlers) {
    .load = prv_actionable_dialog_load,
    .unload = prv_actionable_dialog_unload,
    .appear = prv_actionable_dialog_appear,
  });
  window_set_user_data(window, actionable_dialog);
}

ActionableDialog *actionable_dialog_create(const char *dialog_name) {
  // Note: Not exported so no need for padding.
  ActionableDialog *actionable_dialog = applib_malloc(sizeof(ActionableDialog));
  if (actionable_dialog) {
    actionable_dialog_init(actionable_dialog, dialog_name);
  }
  return actionable_dialog;
}

void actionable_dialog_set_action_bar_type(ActionableDialog *actionable_dialog,
                                           DialogActionBarType action_bar_type,
                                           ActionBarLayer *action_bar) {
  if (action_bar_type == DialogActionBarCustom) {
    PBL_ASSERTN(action_bar); // Action bar must not be NULL if it is a custom type.
    actionable_dialog->action_bar = action_bar;
  } else {
    actionable_dialog->action_bar = NULL;
  }
  actionable_dialog->action_bar_type = action_bar_type;
}

void actionable_dialog_set_click_config_provider(ActionableDialog *actionable_dialog,
                                                 ClickConfigProvider click_config_provider) {
  actionable_dialog->config_provider = click_config_provider;
}

void actionable_dialog_set_user_data(ActionableDialog *actionable_dialog, void *user_data) {
  actionable_dialog->user_data = user_data;
}

void *actionable_dialog_get_user_data(ActionableDialog *actionable_dialog) {
  return actionable_dialog->user_data;
}
