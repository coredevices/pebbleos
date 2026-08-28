/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "dialog.h"

#include "applib/applib_malloc.auto.h"
#include "applib/graphics/gtypes.h"
#include "applib/ui/action_bar_layer_private.h"
#include "applib/ui/dialogs/dialog_private.h"
#include "kernel/ui/kernel_ui.h"
#include "kernel/pbl_malloc.h"
#include "resource/resource_ids.auto.h"
#include "system/passert.h"

#define TEXT_FLOW_INSET_PX (PBL_IF_RECT_ELSE(0, 8))

static void prv_workout_dialog_load(Window *window) {
  WorkoutDialog *workout_dialog = window_get_user_data(window);
  Dialog *dialog = &workout_dialog->dialog;
  Layer *window_root_layer = window_get_root_layer(window);

  // Ownership of icon is taken over by KinoLayer in dialog_init_icon_layer() call below
  KinoReel *icon = dialog_create_icon(dialog);
  const GSize icon_size = icon ? kino_reel_get_size(icon) : GSizeZero;

  const bool show_action_bar = !workout_dialog->hide_action_bar;

  const GRect *bounds = &window_root_layer->bounds;
  const uint16_t icon_single_line_text_offset_px = 9;
  const uint16_t small_icon_offset = (icon_size.h < 60) ? 7 : 0;
  const uint16_t left_margin_px = PBL_IF_RECT_ELSE(5, 0);
  const uint16_t action_bar_width = show_action_bar ? ACTION_BAR_WIDTH : 0;
  const uint16_t content_x_start = (show_action_bar && !action_bar_layer_is_on_right())
      ? action_bar_width : 0;
#if PBL_ROUND
  const bool action_bar_on_right = action_bar_layer_is_on_right();
#endif
  const uint16_t content_and_action_bar_horizontal_spacing =
      PBL_IF_RECT_ELSE(left_margin_px, show_action_bar ? 11 : left_margin_px);
  const uint16_t right_margin_px = action_bar_width + content_and_action_bar_horizontal_spacing;
  const uint16_t text_single_line_text_offset_px = 17;
  const int16_t text_layer_line_spacing_delta = -4;
  const GFont dialog_text_font = fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD);
  const GFont dialog_subtext_font = fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD);
  const int single_line_text_height_px = fonts_get_font_height(dialog_text_font);
  const int max_text_line_height_px = 2 * single_line_text_height_px + 8;

  const uint16_t status_layer_offset = dialog->show_status_layer ? 6 : 0;
  uint16_t text_top_margin_px = icon ? icon_size.h + 22 : 6;
  uint16_t subtext_top_margin_px = text_top_margin_px + single_line_text_height_px;
  uint16_t icon_top_margin_px = PBL_IF_RECT_ELSE(18, 22);
  uint16_t text_height;
  uint16_t x = content_x_start;
  uint16_t y = 0;
  uint16_t w = PBL_IF_RECT_ELSE(bounds->size.w - action_bar_width, bounds->size.w);
  uint16_t h = STATUS_BAR_LAYER_HEIGHT;

  if (dialog->show_status_layer) {
    dialog_add_status_bar_layer(dialog, &GRect(x, y, w, h));
  }

  x = content_x_start + (action_bar_layer_is_on_right()
                         ? left_margin_px : content_and_action_bar_horizontal_spacing);
  w = bounds->size.w - left_margin_px - right_margin_px;

  GTextAttributes *text_attributes = NULL;
#if PBL_ROUND
  // Create a GTextAttributes for the TextLayer. Note that the matching
  // graphics_text_attributes_destroy() will not need to be called here, as the ownership
  // of text_attributes will be transferred to the TextLayer we assign it to.
  text_attributes = graphics_text_attributes_create();
  graphics_text_attributes_enable_screen_text_flow(text_attributes, TEXT_FLOW_INSET_PX);
#endif
  // Check if the text takes up more than one line. If the dialog has a single line of text,
  // the icon and line of text are positioned lower so as to be more vertically centered.
  GContext *ctx = graphics_context_get_current_context();
  const GTextAlignment text_alignment = PBL_IF_RECT_ELSE(GTextAlignmentCenter,
      show_action_bar ? (action_bar_on_right ? GTextAlignmentRight : GTextAlignmentLeft)
                      : GTextAlignmentCenter);
  {
    // do all this in a block so we enforce that nobody uses these variables outside of the block
    // when dealing with round displays, sizes change depending on location.
    const GRect probe_rect = GRect(x, y + text_single_line_text_offset_px,
                                   w, max_text_line_height_px);
    text_height = graphics_text_layout_get_max_used_size(ctx,
                                                         dialog->buffer,
                                                         dialog_text_font,
                                                         probe_rect,
                                                         GTextOverflowModeWordWrap,
                                                         text_alignment,
                                                         text_attributes).h;
    if (text_height <= single_line_text_height_px) {
      text_top_margin_px += text_single_line_text_offset_px;
      icon_top_margin_px += icon_single_line_text_offset_px;
    } else {
      text_top_margin_px += status_layer_offset + small_icon_offset + 2;
      icon_top_margin_px += status_layer_offset + small_icon_offset;
    }
    subtext_top_margin_px = text_top_margin_px + text_height + text_layer_line_spacing_delta;
  }

  y = text_top_margin_px;
  h = text_height;

  // Set up the text.
  TextLayer *text_layer = &dialog->text_layer;
  text_layer_init_with_parameters(text_layer, &GRect(x, y, w, h),
                                  dialog->buffer, dialog_text_font,
                                  dialog->text_color, GColorClear, text_alignment,
                                  GTextOverflowModeWordWrap);
#if PBL_ROUND
  text_layer_enable_screen_text_flow_and_paging(text_layer, TEXT_FLOW_INSET_PX);
#endif
  text_layer_set_line_spacing_delta(text_layer, text_layer_line_spacing_delta);

  layer_add_child(&window->layer, &text_layer->layer);

  if (workout_dialog->subtext_buffer) {
    y = subtext_top_margin_px;

    TextLayer *subtext_layer = &workout_dialog->subtext_layer;
    text_layer_init_with_parameters(subtext_layer, &GRect(x, y, w, h),
                                    workout_dialog->subtext_buffer, dialog_subtext_font,
                                    dialog->text_color, GColorClear, text_alignment,
                                    GTextOverflowModeWordWrap);
#if PBL_ROUND
    text_layer_enable_screen_text_flow_and_paging(subtext_layer, TEXT_FLOW_INSET_PX);
#endif

    layer_add_child(&window->layer, &subtext_layer->layer);
  }

  if (show_action_bar) {
    action_bar_layer_add_to_window(&workout_dialog->action_bar, window);
  }

  // Icon
  // On rectangular displays we just center it horizontally b/w the left edge of the display and
  // the left edge of the action bar
#if PBL_RECT
  x = content_x_start + (grect_get_max_x(bounds) - action_bar_width - icon_size.w) / 2;
#else
  if (show_action_bar) {
    x = action_bar_on_right
        ? (grect_get_max_x(bounds) - action_bar_width - content_and_action_bar_horizontal_spacing -
           icon_size.w)
        : (content_x_start + content_and_action_bar_horizontal_spacing);
  } else {
    x = (grect_get_max_x(bounds) - action_bar_width - icon_size.w) / 2;
  }
#endif

  y = icon_top_margin_px;

  if (dialog_init_icon_layer(dialog, icon, GPoint(x, y), false)) {
    layer_add_child(window_root_layer, &dialog->icon_layer.layer);
  }

  dialog_load(dialog);
}

static void prv_workout_dialog_appear(Window *window) {
  WorkoutDialog *workout_dialog = window_get_user_data(window);
  dialog_appear(&workout_dialog->dialog);
}

static void prv_workout_dialog_unload(Window *window) {
  WorkoutDialog *workout_dialog = window_get_user_data(window);
  dialog_unload(&workout_dialog->dialog);

  action_bar_layer_remove_from_window(&workout_dialog->action_bar);
  action_bar_layer_deinit(&workout_dialog->action_bar);

  gbitmap_deinit(&workout_dialog->confirm_icon);
  gbitmap_deinit(&workout_dialog->decline_icon);

  if (workout_dialog->subtext_buffer) {
    applib_free(workout_dialog->subtext_buffer);
  }

  if (workout_dialog->dialog.destroy_on_pop) {
    applib_free(workout_dialog);
  }
}

void workout_dialog_init(WorkoutDialog *workout_dialog, const char *dialog_name) {
  PBL_ASSERTN(workout_dialog);
  *workout_dialog = (WorkoutDialog){};

  dialog_init(&workout_dialog->dialog, dialog_name);
  Window *window = &workout_dialog->dialog.window;
  window_set_window_handlers(window, &(WindowHandlers) {
    .load = prv_workout_dialog_load,
    .unload = prv_workout_dialog_unload,
    .appear = prv_workout_dialog_appear,
  });
  window_set_user_data(window, workout_dialog);

  gbitmap_init_with_resource(&workout_dialog->confirm_icon, RESOURCE_ID_ACTION_BAR_ICON_CHECK);
  gbitmap_init_with_resource(&workout_dialog->decline_icon, RESOURCE_ID_ACTION_BAR_ICON_X);

  ActionBarLayer *action_bar = &workout_dialog->action_bar;
  action_bar_layer_init(action_bar);
  action_bar_layer_set_icon(action_bar, BUTTON_ID_UP, &workout_dialog->confirm_icon);
  action_bar_layer_set_icon(action_bar, BUTTON_ID_DOWN, &workout_dialog->decline_icon);
  action_bar_layer_set_background_color(action_bar, GColorBlack);
  action_bar_layer_set_context(action_bar, workout_dialog);
}

WorkoutDialog *workout_dialog_create(const char *dialog_name) {
  // Note: Not exported so no need for padding.
  WorkoutDialog *workout_dialog = applib_malloc(sizeof(WorkoutDialog));
  if (workout_dialog) {
    workout_dialog_init(workout_dialog, dialog_name);
  }
  return workout_dialog;
}

Dialog *workout_dialog_get_dialog(WorkoutDialog *workout_dialog) {
  return &workout_dialog->dialog;
}

ActionBarLayer *workout_dialog_get_action_bar(WorkoutDialog *workout_dialog) {
  return &workout_dialog->action_bar;
}

void workout_dialog_set_click_config_provider(WorkoutDialog *workout_dialog,
                                              ClickConfigProvider click_config_provider) {
  if (workout_dialog == NULL) {
    return;
  }
  ActionBarLayer *action_bar = &workout_dialog->action_bar;
  action_bar_layer_set_click_config_provider(action_bar, click_config_provider);
}

void workout_dialog_set_click_config_context(WorkoutDialog *workout_dialog, void *context) {
  if (workout_dialog == NULL) {
    return;
  }
  ActionBarLayer *action_bar = &workout_dialog->action_bar;
  action_bar_layer_set_context(action_bar, context);
}

void workout_dialog_push(WorkoutDialog *workout_dialog, WindowStack *window_stack) {
  dialog_push(&workout_dialog->dialog, window_stack);
}

void app_workout_dialog_push(WorkoutDialog *workout_dialog) {
  app_dialog_push(&workout_dialog->dialog);
}

void workout_dialog_pop(WorkoutDialog *workout_dialog) {
  dialog_pop(&workout_dialog->dialog);
}

void workout_dialog_set_text(WorkoutDialog *workout_dialog, const char *text) {
  dialog_set_text(&workout_dialog->dialog, text);
}

void workout_dialog_set_subtext(WorkoutDialog *workout_dialog, const char *text) {
  if (workout_dialog->subtext_buffer) {
    applib_free(workout_dialog->subtext_buffer);
  }
  uint16_t len = strlen(text);
  workout_dialog->subtext_buffer = applib_malloc(len + 1);
  strncpy(workout_dialog->subtext_buffer, text, len + 1);
}

void workout_dialog_set_action_bar_hidden(WorkoutDialog *workout_dialog, bool should_hide) {
  workout_dialog->hide_action_bar = should_hide;
}
