/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "action_bar_layer_legacy2.h"

#include "applib/graphics/graphics.h"
#include "applib/ui/window_private.h"
#include "kernel/pbl_malloc.h"
#include "system/passert.h"

inline static bool action_bar_legacy2_is_highlighted(ActionBarLayerLegacy2 *action_bar,
                                                     uint8_t index) {
  PBL_ASSERTN(index < NUM_ACTION_BAR_LEGACY2_ITEMS);
  return (bool) (action_bar->is_highlighted & (1 << index));
}

inline static void action_bar_legacy2_set_highlighted(ActionBarLayerLegacy2 *action_bar,
                                                      uint8_t index, bool highlighted) {
  PBL_ASSERT(index < NUM_ACTION_BAR_LEGACY2_ITEMS, "Index: %"PRIu8, index);

  const uint8_t bit = (1 << index);
  if (highlighted) {
    action_bar->is_highlighted |= bit;
  } else {
    action_bar->is_highlighted &= ~bit;
  }
}

static void action_bar_legacy2_changed_proc(ActionBarLayerLegacy2 *action_bar, GContext* ctx) {
  if (action_bar->layer.window && action_bar->layer.window->on_screen == false) {
    // clear first, fixes issue of returning from other page while highlighted
    for (int i = 0; i < NUM_ACTION_BAR_LEGACY2_ITEMS; i++) {
      action_bar_legacy2_set_highlighted(action_bar, i, false);
    }
  }
}

static void action_bar_legacy2_update_proc(ActionBarLayerLegacy2 *action_bar, GContext* ctx) {
  const GColor bg_color = get_native_color(action_bar->background_color);
  graphics_context_set_fill_color(ctx, bg_color);
  const uint8_t radius = 3;
  const uint8_t margin = 1;
  graphics_fill_round_rect(ctx, &action_bar->layer.bounds, radius,
                           GCornerTopLeft | GCornerBottomLeft);
  GRect rect = action_bar->layer.bounds;
  rect.origin.x += margin;
  rect.origin.y += margin;
  rect.size.w -= margin;
  rect.size.h -= 2 * margin;
  rect.size.h /= NUM_ACTION_BAR_LEGACY2_ITEMS;
  const bool is_white = gcolor_equal(bg_color, GColorWhite);
  const GColor highlighted_color = (is_white) ? GColorBlack : GColorWhite;
  for (unsigned int index = 0; index < NUM_ACTION_BAR_LEGACY2_ITEMS; ++index) {
    const GBitmap *icon = action_bar->icons[index];
    if (icon) {
      const bool is_highlighted = action_bar_legacy2_is_highlighted(action_bar, index);
      if (is_highlighted) {
        graphics_context_set_fill_color(ctx, highlighted_color);
        GCornerMask corner;
        switch (index) {
          case 0: corner = GCornerTopLeft; break;
          case NUM_ACTION_BAR_LEGACY2_ITEMS - 1: corner = GCornerBottomLeft; break;
          default: corner = GCornerNone; break;
        }
        graphics_fill_round_rect(ctx, &rect, radius - margin, corner);
      }
      GRect icon_rect = icon->bounds;
      const bool clip = true;
      grect_align(&icon_rect, &rect, GAlignCenter, clip);
      const GCompOp op = (is_white != is_highlighted) ? GCompOpAssign : GCompOpAssignInverted;
      graphics_context_set_compositing_mode(ctx, op);
      graphics_draw_bitmap_in_rect(ctx, (GBitmap*)icon, &icon_rect);
    }
    rect.origin.y += rect.size.h;
  }
}

void action_bar_layer_legacy2_init(ActionBarLayerLegacy2 *action_bar) {
  *action_bar = (ActionBarLayerLegacy2){};
  layer_init(&action_bar->layer, &GRectZero);
  action_bar->layer.update_proc = (LayerUpdateProc) action_bar_legacy2_update_proc;
  action_bar->layer.property_changed_proc =
      (PropertyChangedProc) (void *) action_bar_legacy2_changed_proc;
  action_bar->background_color = GColor2Black;
}

ActionBarLayerLegacy2 *action_bar_layer_legacy2_create(void) {
  ActionBarLayerLegacy2 * layer = task_malloc(sizeof(ActionBarLayerLegacy2));
  if (layer) {
    action_bar_layer_legacy2_init(layer);
  }
  return layer;
}

void action_bar_layer_legacy2_deinit(ActionBarLayerLegacy2 *action_bar_layer) {
  layer_deinit(&action_bar_layer->layer);
}

void action_bar_layer_legacy2_destroy(ActionBarLayerLegacy2 *action_bar_layer) {
  if (action_bar_layer == NULL) {
    return;
  }
  action_bar_layer_legacy2_deinit(action_bar_layer);
  task_free(action_bar_layer);
}

Layer* action_bar_layer_legacy2_get_layer(ActionBarLayerLegacy2 *action_bar_layer) {
  return &action_bar_layer->layer;
}

inline static void* action_bar_legacy2_get_context(ActionBarLayerLegacy2 *action_bar) {
  return action_bar->context ? action_bar->context : action_bar;
}

void action_bar_layer_legacy2_set_context(ActionBarLayerLegacy2 *action_bar, void *context) {
  action_bar->context = context;
}

static void action_bar_legacy2_raw_up_down_handler(ClickRecognizerRef recognizer,
                                                   ActionBarLayerLegacy2 *action_bar,
                                                   bool is_highlighted) {
  const ButtonId button_id = click_recognizer_get_button_id(recognizer);
  const uint8_t index = button_id - 1;
  const GBitmap *icon = action_bar->icons[index];

  // is_highlighted will cause the icon in the action bar to render normal or inverted:
  action_bar_legacy2_set_highlighted(action_bar, index, is_highlighted);
  if (icon == NULL) {
    return;
  } else {
    layer_mark_dirty(&action_bar->layer);
  }
}

static void action_bar_legacy2_raw_up_handler(ClickRecognizerRef recognizer, void *context) {
  ActionBarLayerLegacy2 *action_bar = (ActionBarLayerLegacy2 *)context;
  action_bar_legacy2_raw_up_down_handler(recognizer, action_bar, false);
}

static void action_bar_legacy2_raw_down_handler(ClickRecognizerRef recognizer, void *context) {
  ActionBarLayerLegacy2 *action_bar = (ActionBarLayerLegacy2 *)context;
  action_bar_legacy2_raw_up_down_handler(recognizer, action_bar, true);
}

static void action_bar_legacy2_click_config_provider(ActionBarLayerLegacy2 *action_bar) {
  void *context = action_bar_legacy2_get_context(action_bar);
  // For UP, SELECT and DOWN, setup the raw handler and assign the user specified context:
  for (ButtonId button_id = BUTTON_ID_UP; button_id < NUM_BUTTONS; ++button_id) {
    window_raw_click_subscribe(button_id, action_bar_legacy2_raw_down_handler,
                               action_bar_legacy2_raw_up_handler, action_bar);
    window_set_click_context(button_id, context);
  }
  // If back button is overridden, set context of BACK click recognizer as well:
  if (action_bar->window && action_bar->window->overrides_back_button) {
    window_set_click_context(BUTTON_ID_BACK, context);
  }
  if (action_bar->click_config_provider) {
    action_bar->click_config_provider(context);
  }
}

inline static void action_bar_legacy2_update_click_config_provider(
    ActionBarLayerLegacy2 *action_bar) {
  if (action_bar->window) {
    window_set_click_config_provider_with_context(
        action_bar->window,
        (ClickConfigProvider) action_bar_legacy2_click_config_provider,
        action_bar);
  }
}

void action_bar_layer_legacy2_set_click_config_provider(ActionBarLayerLegacy2 *action_bar,
                                                        ClickConfigProvider click_config_provider) {
  action_bar->click_config_provider = click_config_provider;
  action_bar_legacy2_update_click_config_provider(action_bar);
}

void action_bar_layer_legacy2_set_icon(ActionBarLayerLegacy2 *action_bar, ButtonId button_id,
                                       const GBitmap *icon) {
  if (button_id < BUTTON_ID_UP || button_id >= NUM_BUTTONS) {
    return;
  }
  if (action_bar->icons[button_id - 1] == icon) {
    return;
  }
  action_bar->icons[button_id - 1] = icon;
  layer_mark_dirty(&action_bar->layer);
}

void action_bar_layer_legacy2_clear_icon(ActionBarLayerLegacy2 *action_bar, ButtonId button_id) {
  action_bar_layer_legacy2_set_icon(action_bar, button_id, NULL);
}

void action_bar_layer_legacy2_add_to_window(ActionBarLayerLegacy2 *action_bar,
                                            struct Window *window) {
  const uint8_t vertical_margin = 3;
  const GRect *window_bounds = &window->layer.bounds;
  GRect rect = GRect(0, 0, ACTION_BAR_LEGACY2_WIDTH,
                     window_bounds->size.h - (vertical_margin * 2));
  layer_set_bounds(&action_bar->layer, &rect);
  rect.origin.x = window_bounds->size.w - ACTION_BAR_LEGACY2_WIDTH;
  rect.origin.y = vertical_margin;
  layer_set_frame(&action_bar->layer, &rect);
  layer_add_child(&window->layer, &action_bar->layer);

  action_bar->window = window;
  action_bar_legacy2_update_click_config_provider(action_bar);
}

void action_bar_layer_legacy2_remove_from_window(ActionBarLayerLegacy2 *action_bar) {
  if (action_bar == NULL || action_bar->window == NULL) {
    return;
  }
  layer_remove_from_parent(&action_bar->layer);
  window_set_click_config_provider_with_context(action_bar->window, NULL, NULL);
  action_bar->window = NULL;
}

void action_bar_layer_legacy2_set_background_color_2bit(ActionBarLayerLegacy2 *action_bar,
    GColor2 background_color) {
  GColor native_background_color = get_native_color(background_color);
  if (gcolor_equal(native_background_color, get_native_color(action_bar->background_color))) {
    return;
  }
  action_bar->background_color = get_closest_gcolor2(native_background_color);
  layer_mark_dirty(&(action_bar->layer));
}
