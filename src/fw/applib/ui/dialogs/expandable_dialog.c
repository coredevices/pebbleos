/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "expandable_dialog.h"

#include "applib/applib_malloc.auto.h"
#include "applib/fonts/fonts.h"
#include "applib/graphics/gtypes.h"
#include "applib/graphics/text.h"
#include "applib/ui/bitmap_layer.h"
#include "applib/ui/dialogs/dialog_private.h"
#include "applib/ui/layer.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window.h"
#include "applib/ui/window_private.h"
#include "kernel/ui/kernel_ui.h"
#include "resource/resource.h"
#include "resource/resource_ids.auto.h"
#include "system/passert.h"

#include <limits.h>
#include <stdint.h>
#include <string.h>

static void prv_show_action_bar_icon(ExpandableDialog *expandable_dialog, ButtonId button_id) {
  ActionBarLayer *action_bar = &expandable_dialog->action_bar;
  const GBitmap *icon = (button_id ==
      BUTTON_ID_UP) ? expandable_dialog->up_icon : expandable_dialog->down_icon;
  action_bar_layer_set_icon_animated(action_bar, button_id, icon,
      expandable_dialog->show_action_icon_animated);

  ActionBarLayerIconPressAnimation animation = (button_id == BUTTON_ID_UP)
      ? ActionBarLayerIconPressAnimationMoveUp
      : ActionBarLayerIconPressAnimationMoveDown;
  action_bar_layer_set_icon_press_animation(action_bar, button_id, animation);
}

// Manually scrolls the scroll layer up or down. The manual scrolling is required so that the
// click handlers of the scroll layer and the action bar play nicely together.
static void prv_manual_scroll(ScrollLayer *scroll_layer, int8_t dir) {
  scroll_layer_scroll(scroll_layer, dir, true);
}

static void prv_offset_changed_handler(ScrollLayer *scroll_layer, void *context) {
  ExpandableDialog *expandable_dialog = context;
  ActionBarLayer *action_bar = &expandable_dialog->action_bar;
  GPoint offset = scroll_layer_get_content_offset(scroll_layer);

  if (!expandable_dialog->show_action_bar) {
    // Prematurely return if we are not showing the action bar.
    return;
  }

  if (offset.y < 0) {
    // We have scrolled down, so we want to display the up arrow.
    prv_show_action_bar_icon(expandable_dialog, BUTTON_ID_UP);
  } else if (offset.y == 0) {
    // Hide the up arrow as we've reached the top.
    action_bar_layer_clear_icon(action_bar, BUTTON_ID_UP);
  }

  Layer *layer = scroll_layer_get_layer(scroll_layer);
  const GRect *bounds = &layer->bounds;
  GSize content_size = scroll_layer_get_content_size(scroll_layer);

  if (offset.y + content_size.h > bounds->size.h) {
    // We have scrolled up, so we want to display the down arrow.
    prv_show_action_bar_icon(expandable_dialog, BUTTON_ID_DOWN);
  } else if (offset.y + content_size.h <= bounds->size.h) {
    // Hide the down arrow as we've reached the bottom.
    action_bar_layer_clear_icon(action_bar, BUTTON_ID_DOWN);
  }
}

static void prv_up_click_handler(ClickRecognizerRef recognizer, void *context) {
  ExpandableDialog *expandable_dialog = context;
  prv_manual_scroll(&expandable_dialog->scroll_layer, 1);
}

static void prv_down_click_handler(ClickRecognizerRef recognizer, void *context) {
  ExpandableDialog *expandable_dialog = context;
  prv_manual_scroll(&expandable_dialog->scroll_layer, -1);
}

static void prv_config_provider(void *context) {
  ExpandableDialog *expandable_dialog = context;
  window_single_repeating_click_subscribe(BUTTON_ID_UP, 100, prv_up_click_handler);
  window_single_repeating_click_subscribe(BUTTON_ID_DOWN, 100, prv_down_click_handler);
  if (expandable_dialog->select_click_handler) {
    window_single_click_subscribe(BUTTON_ID_SELECT, expandable_dialog->select_click_handler);
  }
}

static void prv_expandable_dialog_load(Window *window) {
  ExpandableDialog *expandable_dialog = window_get_user_data(window);
  Dialog *dialog = &expandable_dialog->dialog;

  GRect frame = window->layer.bounds;
  static const uint16_t ICON_TOP_MARGIN_PX = 16;
  static const uint16_t BOTTOM_MARGIN_PX = 6;
  static const uint16_t CONTENT_DOWN_ARROW_HEIGHT = PBL_IF_RECT_ELSE(16, 10);

  // Small margin is shown when we have an action bar to fit more text on the line.
  const uint16_t SM_LEFT_MARGIN_PX = 4;
  // Normal margin shown when there is no action bar in the expandable dialog.
  const uint16_t NM_LEFT_MARGIN_PX = 10;

  bool show_action_bar = expandable_dialog->show_action_bar;
  bool action_bar_on_right = action_bar_is_on_right();

  uint16_t left_margin_px = show_action_bar ? SM_LEFT_MARGIN_PX : NM_LEFT_MARGIN_PX;
  uint16_t right_margin_px = left_margin_px;

  bool has_header = *expandable_dialog->header ? true : false;

  const GFont header_font = expandable_dialog->header_font;
  int32_t header_content_height = has_header ? DISP_ROWS : 0;

  uint16_t status_layer_offset = dialog->show_status_layer * STATUS_BAR_LAYER_HEIGHT;
  uint16_t action_bar_offset = show_action_bar * ACTION_BAR_WIDTH;
  uint16_t content_x_start = action_bar_on_right ? 0 : action_bar_offset;

  uint16_t x = content_x_start;
  uint16_t y = 0;
  uint16_t w = PBL_IF_RECT_ELSE(frame.size.w - action_bar_offset, frame.size.w);
  uint16_t h = STATUS_BAR_LAYER_HEIGHT;

  if (dialog->show_status_layer) {
    dialog_add_status_bar_layer(dialog, &GRect(x, y, w, h));
  }
  GContext *ctx = graphics_context_get_current_context();

  // Ownership of icon is taken over by KinoLayer in dialog_init_icon_layer() call below
  KinoReel *icon = dialog_create_icon(dialog);
  const GSize icon_size = icon ? kino_reel_get_size(icon) : GSizeZero;
  uint16_t icon_offset = (icon ? ICON_TOP_MARGIN_PX - status_layer_offset : 0);


  x = 0;
  y = status_layer_offset;
  w = frame.size.w;
  h = frame.size.h - y;

  ScrollLayer *scroll_layer = &expandable_dialog->scroll_layer;
  scroll_layer_init(scroll_layer, &GRect(x, y, w, h));
  layer_add_child(&window->layer, &scroll_layer->layer);


  // Set up the header if this dialog is set to have one.
  GTextAlignment alignment = PBL_IF_RECT_ELSE(GTextAlignmentLeft,
                                              (show_action_bar ?
                                               (action_bar_on_right ?
                                                GTextAlignmentRight : GTextAlignmentLeft) :
                                               GTextAlignmentCenter));
  uint16_t right_aligned_box_reduction = PBL_IF_RECT_ELSE(0, show_action_bar ? 10 : 0);
  if (has_header) {
    const uint16_t HEADER_OFFSET = 6;
#if PBL_RECT
    x = content_x_start + left_margin_px;
    w = frame.size.w - right_margin_px - left_margin_px - action_bar_offset
        - right_aligned_box_reduction;
#else
    x = content_x_start;
    w = frame.size.w - right_margin_px - action_bar_offset - right_aligned_box_reduction;
#endif
    y = icon ? icon_offset + icon_size.h : -HEADER_OFFSET;

    TextLayer *header_layer = &expandable_dialog->header_layer;
    text_layer_init_with_parameters(header_layer, &GRect(x, y, w, header_content_height),
                                    expandable_dialog->header, header_font, dialog->text_color,
                                    GColorClear, alignment, GTextOverflowModeWordWrap);
    // layer must be added immediately to scroll layer for perimeter and paging
    scroll_layer_add_child(scroll_layer, &header_layer->layer);

#if PBL_ROUND
    text_layer_enable_screen_text_flow_and_paging(header_layer, 8);
#endif

    // We account for a header that may be larger than the available space
    // by adjusting our height variable that is passed to the body layer
    GSize header_size = text_layer_get_content_size(ctx, header_layer);
    header_size.h += 4; // See PBL-1741
    header_size.w = w;
    header_content_height = header_size.h;
    text_layer_set_size(header_layer, header_size);
  }

  // Set up the text.
  const uint16_t TEXT_OFFSET = 6;
  x = content_x_start + left_margin_px;
  y = (icon ? icon_offset + icon_size.h : -TEXT_OFFSET) + header_content_height;
  w = frame.size.w - right_margin_px - left_margin_px - action_bar_offset
      - right_aligned_box_reduction;
  h = INT16_MAX;  // height is clamped to content size
  GFont font = expandable_dialog->body_font;

  TextLayer *text_layer = &dialog->text_layer;
  text_layer_init_with_parameters(text_layer, &GRect(x, y, w, h), dialog->buffer, font,
                                  dialog->text_color, GColorClear,
                                  alignment, GTextOverflowModeWordWrap);
  // layer must be added immediately to scroll layer for perimeter and paging
  scroll_layer_add_child(scroll_layer, &text_layer->layer);

#if PBL_ROUND
  text_layer_set_line_spacing_delta(text_layer, -1);
  text_layer_enable_screen_text_flow_and_paging(text_layer, 8);
#endif

  int32_t text_content_height = text_layer_get_content_size(ctx, text_layer).h;
  text_content_height += 4; // See PBL-1741
  text_layer_set_size(text_layer, GSize(w, text_content_height));

  uint16_t scroll_height = icon_offset + icon_size.h +
      header_content_height + text_content_height + (icon ? BOTTOM_MARGIN_PX : 0);

  scroll_layer_set_content_size(scroll_layer, GSize(frame.size.w, scroll_height));
  scroll_layer_set_shadow_hidden(scroll_layer, true);
  scroll_layer_set_callbacks(scroll_layer, (ScrollLayerCallbacks) {
        .content_offset_changed_handler = prv_offset_changed_handler
      });
  scroll_layer_set_context(scroll_layer, expandable_dialog);

#if PBL_ROUND
  scroll_layer_set_paging(scroll_layer, true);
#endif

  if (show_action_bar) {
    // Icons for up and down on the action bar.
#ifndef CONFIG_RECOVERY_FW
    expandable_dialog->up_icon = gbitmap_create_with_resource_system(SYSTEM_APP,
        RESOURCE_ID_ACTION_BAR_ICON_UP);
    expandable_dialog->down_icon = gbitmap_create_with_resource_system(SYSTEM_APP,
        RESOURCE_ID_ACTION_BAR_ICON_DOWN);
    PBL_ASSERTN(expandable_dialog->down_icon && expandable_dialog->up_icon);
#endif

    // Set up the Action bar.
    ActionBarLayer *action_bar = &expandable_dialog->action_bar;
    action_bar_layer_init(action_bar);
    if (expandable_dialog->action_bar_background_color.a != 0) {
      action_bar_layer_set_background_color(action_bar,
                                            expandable_dialog->action_bar_background_color);
    }
    if (expandable_dialog->select_icon) {
      action_bar_layer_set_icon_animated(action_bar, BUTTON_ID_SELECT,
          expandable_dialog->select_icon, expandable_dialog->show_action_icon_animated);
    }
    action_bar_layer_set_context(action_bar, expandable_dialog);
    action_bar_layer_set_click_config_provider(action_bar, prv_config_provider);
    action_bar_layer_add_to_window(action_bar, window);
  } else {
    window_set_click_config_provider_with_context(window, prv_config_provider, expandable_dialog);
  }

  x = PBL_IF_RECT_ELSE(content_x_start + left_margin_px, (show_action_bar) ?
      (action_bar_on_right ?
       (frame.size.w - right_margin_px - left_margin_px -
        action_bar_offset - right_aligned_box_reduction - icon_size.h) :
       (content_x_start + left_margin_px)) :
      (90 - icon_size.h / 2));
  y = icon_offset + PBL_IF_RECT_ELSE(0, 5);
  if (dialog_init_icon_layer(dialog, icon, GPoint(x, y), false /* not animated */)) {
    scroll_layer_add_child(scroll_layer, &dialog->icon_layer.layer);
  }

  // Check if we should show the down arrow by checking if we have enough content to warrant
  // the scroll layer scrolling.
  if (scroll_height > frame.size.h) {
    if (show_action_bar) {
      prv_show_action_bar_icon(expandable_dialog, BUTTON_ID_DOWN);
    } else {
      // If there isn't an action bar and there is more content than fits the screen
      // setup the status layer and content_down_arrow_layer
      ContentIndicator *indicator = scroll_layer_get_content_indicator(scroll_layer);
      content_indicator_configure_direction(
          indicator, ContentIndicatorDirectionUp,
          &(ContentIndicatorConfig) {
            .layer = &expandable_dialog->dialog.status_layer.layer,
            .times_out = true,
            .colors.foreground = dialog->text_color,
            .colors.background = dialog->window.background_color,
      });
      layer_init(&expandable_dialog->content_down_arrow_layer, &GRect(
                 content_x_start, frame.size.h - CONTENT_DOWN_ARROW_HEIGHT,
                 PBL_IF_RECT_ELSE(frame.size.w - action_bar_offset, frame.size.w),
                 CONTENT_DOWN_ARROW_HEIGHT));
      layer_add_child(&window->layer, &expandable_dialog->content_down_arrow_layer);
      content_indicator_configure_direction(
          indicator, ContentIndicatorDirectionDown,
          &(ContentIndicatorConfig) {
            .layer = &expandable_dialog->content_down_arrow_layer,
            .times_out = false,
            .alignment = PBL_IF_RECT_ELSE(GAlignCenter, GAlignTop),
            .colors.foreground = dialog->text_color,
            .colors.background = dialog->window.background_color,
      });
    }
  }

  dialog_load(dialog);
}

static void prv_expandable_dialog_appear(Window *window) {
  ExpandableDialog *expandable_dialog = window_get_user_data(window);
  Dialog *dialog = expandable_dialog_get_dialog(expandable_dialog);
  scroll_layer_update_content_indicator(&expandable_dialog->scroll_layer);
  dialog_appear(dialog);
}

static void prv_expandable_dialog_unload(Window *window) {
  ExpandableDialog *expandable_dialog = window_get_user_data(window);
  dialog_unload(&expandable_dialog->dialog);
  if (expandable_dialog->show_action_bar) {
    action_bar_layer_deinit(&expandable_dialog->action_bar);
  }
  gbitmap_destroy(expandable_dialog->up_icon);
  gbitmap_destroy(expandable_dialog->down_icon);
  if (*expandable_dialog->header) {
    text_layer_deinit(&expandable_dialog->header_layer);
  }
  scroll_layer_deinit(&expandable_dialog->scroll_layer);
  if (expandable_dialog->dialog.destroy_on_pop) {
    applib_free(expandable_dialog);
  }
}

Dialog *expandable_dialog_get_dialog(ExpandableDialog *expandable_dialog) {
  return &expandable_dialog->dialog;
}

void expandable_dialog_init(ExpandableDialog *expandable_dialog, const char *dialog_name) {
  PBL_ASSERTN(expandable_dialog);
  *expandable_dialog = (ExpandableDialog) {
    .header_font = fonts_get_system_font(FONT_KEY_GOTHIC_28_BOLD),
    .body_font = fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD),
  };

  dialog_init(&expandable_dialog->dialog, dialog_name);
  Window *window = &expandable_dialog->dialog.window;
  window_set_window_handlers(window, &(WindowHandlers) {
    .load = prv_expandable_dialog_load,
    .unload = prv_expandable_dialog_unload,
    .appear = prv_expandable_dialog_appear,
  });
  expandable_dialog->show_action_bar = true;
  window_set_user_data(window, expandable_dialog);
}

ExpandableDialog *expandable_dialog_create(const char *dialog_name) {
  // Note: Note exported so no padding necessary
  ExpandableDialog *expandable_dialog = applib_type_malloc(ExpandableDialog);
  if (expandable_dialog) {
    expandable_dialog_init(expandable_dialog, dialog_name);
  }
  return expandable_dialog;
}

void expandable_dialog_close_cb(ClickRecognizerRef recognizer, void *e_dialog) {
  expandable_dialog_pop(e_dialog);
}

ExpandableDialog *expandable_dialog_create_with_params(const char *dialog_name, ResourceId icon,
                                                       const char *text, GColor text_color,
                                                       GColor background_color,
                                                       DialogCallbacks *callbacks,
                                                       ResourceId select_icon,
                                                       ClickHandler select_click_handler) {
  ExpandableDialog *expandable_dialog = expandable_dialog_create(dialog_name);
  if (expandable_dialog) {
    expandable_dialog_set_select_action(expandable_dialog, select_icon, select_click_handler);

    Dialog *dialog = expandable_dialog_get_dialog(expandable_dialog);
    dialog_set_icon(dialog, icon);
    dialog_set_text(dialog, text);
    dialog_set_background_color(dialog, background_color);
    dialog_set_text_color(dialog, gcolor_legible_over(background_color));
    dialog_set_callbacks(dialog, callbacks, dialog);
  }
  return expandable_dialog;
}

void expandable_dialog_show_action_bar(ExpandableDialog *expandable_dialog,
                                       bool show_action_bar) {
  expandable_dialog->show_action_bar = show_action_bar;
}

void expandable_dialog_set_action_icon_animated(ExpandableDialog *expandable_dialog,
                                                bool animated) {
  expandable_dialog->show_action_icon_animated = animated;
}

void expandable_dialog_set_action_bar_background_color(ExpandableDialog *expandable_dialog,
                                                       GColor background_color) {
  expandable_dialog->action_bar_background_color = background_color;
}

void expandable_dialog_set_header(ExpandableDialog *expandable_dialog, const char *header) {
  if (!header) {
    expandable_dialog->header[0] = 0;
    return;
  }
  strncpy(expandable_dialog->header, header, DIALOG_MAX_HEADER_LEN);
  expandable_dialog->header[DIALOG_MAX_HEADER_LEN] = '\0';
}

void expandable_dialog_set_header_font(ExpandableDialog *expandable_dialog, GFont header_font) {
  expandable_dialog->header_font = header_font;
}

void expandable_dialog_set_body_font(ExpandableDialog *expandable_dialog, GFont body_font) {
  expandable_dialog->body_font = body_font;
}

void expandable_dialog_set_select_action(ExpandableDialog *expandable_dialog,
                                         uint32_t resource_id,
                                         ClickHandler select_click_handler) {
  if (expandable_dialog->select_icon) {
    gbitmap_destroy(expandable_dialog->select_icon);
    expandable_dialog->select_icon = NULL;
  }

  if (resource_id != RESOURCE_ID_INVALID) {
    expandable_dialog->select_icon = gbitmap_create_with_resource_system(SYSTEM_APP, resource_id);
  }
  expandable_dialog->select_click_handler = select_click_handler;
}

void expandable_dialog_push(ExpandableDialog *expandable_dialog, WindowStack *window_stack) {
  dialog_push(&expandable_dialog->dialog, window_stack);
}

void app_expandable_dialog_push(ExpandableDialog *expandable_dialog) {
  app_dialog_push(&expandable_dialog->dialog);
}

void expandable_dialog_pop(ExpandableDialog *expandable_dialog) {
  dialog_pop(&expandable_dialog->dialog);
}
