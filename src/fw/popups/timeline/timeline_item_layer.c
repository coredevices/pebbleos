/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "timeline_item_layer.h"

#include "applib/graphics/graphics.h"
#include "applib/ui/action_menu_window.h"
#include "applib/ui/window.h"
#include "applib/ui/window_manager.h"
#include "apps/system/timeline/timeline.h"
#include "kernel/pebble_tasks.h"
#include "kernel/ui/kernel_ui.h"
#include "kernel/ui/modals/modal_manager.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/timeline/layout_layer.h"
#include "pbl/services/timeline/timeline_actions.h"
#include "system/passert.h"
#include "pbl/util/math.h"

#include <stdint.h>
#include <time.h>

///////////////////////////////////////////////////////////
// Drawing functions
///////////////////////////////////////////////////////////

static GSize prv_get_frame_size(TimelineItemLayer *item_layer) {
  return item_layer->layer.bounds.size;
}

static int16_t prv_get_height(TimelineItemLayer *item_layer) {
  if (item_layer->timeline_layout) {
    GSize size = layout_get_size(graphics_context_get_current_context(),
                                 (LayoutLayer *)item_layer->timeline_layout);
    return size.h;
  } else {
    return 0;
  }
}

static void prv_update_item(GContext *ctx, TimelineItemLayer *item_layer) {
  if (item_layer->timeline_layout) {
    GRect bounds = item_layer->timeline_layout->layout_layer.layer.bounds;
    bounds.origin.y = 0 - item_layer->scroll_offset_pixels;
    layer_set_bounds((Layer *)item_layer->timeline_layout, &bounds);
  }
}

///////////////////////////////////////////////////////////
// Scrolling related functions
///////////////////////////////////////////////////////////

static int16_t prv_get_first_scroll_offset(TimelineItemLayer *item_layer) {
  if (!item_layer->timeline_layout->has_page_break) {
    return 0;
  }
  return MAX(prv_get_frame_size(item_layer).h, 0);
}

static int16_t prv_get_min_scroll_offset(TimelineItemLayer *item_layer) {
  return 0;
}

static int16_t prv_get_max_scroll_offset(TimelineItemLayer *item_layer) {
  int16_t max_scroll = prv_get_height(item_layer) - prv_get_frame_size(item_layer).h;
  if (max_scroll > 0) {
    int16_t first_scroll = prv_get_first_scroll_offset(item_layer);
    return MAX(first_scroll, max_scroll);
  } else {
    return MAX(max_scroll, 0);
  }
}

static void prv_scroll_offset_setter(TimelineItemLayer *item_layer, int16_t value) {
  item_layer->scroll_offset_pixels = value;
  layer_mark_dirty(&item_layer->layer);
}

static int16_t prv_scroll_offset_getter(TimelineItemLayer *item_layer) {
  return item_layer->scroll_offset_pixels;
}

static void prv_update_scroll_offset(TimelineItemLayer *item_layer, int16_t new_offset,
                                     bool is_first_scroll) {
  static const PropertyAnimationImplementation implementation = {
    .base = {
      .update = (AnimationUpdateImplementation) property_animation_update_int16,
    },
    .accessors = {
      .setter = { .int16 = (const Int16Setter) prv_scroll_offset_setter, },
      .getter = { .int16 = (const Int16Getter) prv_scroll_offset_getter},
    },
  };

  // If we're already at that position, don't bother scheduling an animation
  if (item_layer->scroll_offset_pixels == new_offset) {
    return;
  }

  if (item_layer->animation
      && animation_is_scheduled(property_animation_get_animation(item_layer->animation))) {
    // Don't do anything if we're already animating to this position from our current position
    int16_t offset;
    property_animation_get_to_int16(item_layer->animation, &offset);
    if (offset == new_offset) {
      return;
    }
    animation_unschedule(property_animation_get_animation(item_layer->animation));
  }

  if (item_layer->animation) {
    property_animation_init(item_layer->animation, &implementation, item_layer, NULL, &new_offset);
  } else {
    item_layer->animation = property_animation_create(&implementation,
        item_layer, NULL, &new_offset);
    PBL_ASSERTN(item_layer->animation);
    animation_set_auto_destroy(property_animation_get_animation(item_layer->animation), false);
  }
  Animation *animation = property_animation_get_animation(item_layer->animation);
  if (is_first_scroll) {
    animation_set_duration(animation, interpolate_moook_duration());
    animation_set_custom_interpolation(animation, interpolate_moook);
  } else {
    animation_set_curve(animation, AnimationCurveEaseOut);
  }
  animation_schedule(animation);
}

//! Maybe make this part of the style and smaller for smaller text sizes?
static const int SCROLL_AMOUNT = PBL_IF_RECT_ELSE(48, DISP_ROWS - STATUS_BAR_LAYER_HEIGHT);
static const int SCROLL_FUDGE_AMOUNT = PBL_IF_RECT_ELSE(10, 0);

/////////////////////////////////////////
// Click Config
/////////////////////////////////////////

T_STATIC void prv_handle_down_click(ClickRecognizerRef recognizer, void *context) {
  TimelineItemLayer *item_layer = (TimelineItemLayer *)context;
  int16_t max_scroll = prv_get_max_scroll_offset(item_layer);
  const int16_t first_scroll = prv_get_first_scroll_offset(item_layer);
  int16_t current_scroll = item_layer->scroll_offset_pixels;
#if PBL_ROUND  // align current_scroll with paging for text flow
  current_scroll = ROUND_TO_MOD_CEIL(current_scroll, SCROLL_AMOUNT);
#endif

  if (max_scroll >= first_scroll && current_scroll < first_scroll) {
    prv_update_scroll_offset(item_layer, first_scroll, true);
  } else if (current_scroll + SCROLL_AMOUNT + SCROLL_FUDGE_AMOUNT >= max_scroll) {
#if PBL_ROUND
    // scroll down to page aligned end of content
    max_scroll = ROUND_TO_MOD_CEIL(max_scroll, DISP_ROWS - STATUS_BAR_LAYER_HEIGHT);
#endif
    prv_update_scroll_offset(item_layer, max_scroll, false);
  } else {
    prv_update_scroll_offset(item_layer, current_scroll + SCROLL_AMOUNT, false);
  }
  layer_mark_dirty(&item_layer->layer);
}

static void prv_handle_select_click(ClickRecognizerRef recognizer, void *context) {
  TimelineItemLayer *item_layer = context;
  TimelineItemActionGroup *action_group = &item_layer->item->action_group;
  const uint8_t num_actions = action_group->num_actions;
  ActionMenuLevel *root_level = timeline_actions_create_action_menu_root_level(
      num_actions, 0, TimelineItemActionSourceTimeline);
  for (int i = 0; i < num_actions; i++) {
    timeline_actions_add_action_to_root_level(&action_group->actions[i], root_level);
  }
  const LayoutColors *colors = layout_get_colors((LayoutLayer *)item_layer->timeline_layout);
  ActionMenuConfig config = {
    .root_level = root_level,
    .context = item_layer->item,
    .colors.background = colors->bg_color,
    .colors.foreground = colors->primary_color,
  };
  timeline_actions_push_action_menu(
      &config, window_manager_get_window_stack(ModalPriorityNotification));
}

static void prv_handle_up_click(ClickRecognizerRef recognizer, void *context) {
  TimelineItemLayer *item_layer = (TimelineItemLayer *)context;
  const int16_t min_scroll = prv_get_min_scroll_offset(item_layer);
  const int16_t first_scroll = prv_get_first_scroll_offset(item_layer);
  int16_t current_scroll = item_layer->scroll_offset_pixels;
#if PBL_ROUND  // align current_scroll with paging for text flow
  current_scroll = ROUND_TO_MOD_CEIL(current_scroll, SCROLL_AMOUNT);
#endif

  if (current_scroll <= first_scroll) {
    prv_update_scroll_offset(item_layer, min_scroll, true);
#if PBL_RECT  // fudge breaks ROUND display paging
  } else if (current_scroll - (SCROLL_AMOUNT + SCROLL_FUDGE_AMOUNT) < first_scroll) {
    prv_update_scroll_offset(item_layer, first_scroll, false);
#endif
  } else {
    prv_update_scroll_offset(item_layer, current_scroll - SCROLL_AMOUNT, false);
  }
  layer_mark_dirty(&item_layer->layer);
}

static void prv_handle_back_click(ClickRecognizerRef recognizer, void *context) {
  timeline_animate_back_from_card();
}

static void timeline_item_layer_click_config_provider(void *context) {
  window_single_repeating_click_subscribe(BUTTON_ID_UP, 100, prv_handle_up_click);
  window_single_repeating_click_subscribe(BUTTON_ID_DOWN, 100, prv_handle_down_click);
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_handle_select_click);
  window_set_click_context(BUTTON_ID_UP, context);
  window_set_click_context(BUTTON_ID_DOWN, context);
  window_set_click_context(BUTTON_ID_SELECT, context);
  window_set_click_context(BUTTON_ID_BACK, context);

  if (pebble_task_get_current() == PebbleTask_App) {
    // only override the back button when we're in the app
    window_single_click_subscribe(BUTTON_ID_BACK, prv_handle_back_click);
  }
}

void timeline_item_layer_set_click_config_onto_window(TimelineItemLayer *item_layer,
    struct Window *window) {
  window_set_click_config_provider_with_context(window,
                                                timeline_item_layer_click_config_provider,
                                                item_layer);
}

/////////////////////////////////////////
// Public functions
/////////////////////////////////////////

void timeline_item_layer_update_proc(Layer* layer, GContext* ctx) {
  //! Fill background with white to hide layers below
  TimelineItemLayer* item_layer = (TimelineItemLayer *)layer;
  const LayoutColors *colors = layout_get_colors((LayoutLayer *)item_layer->timeline_layout);
  graphics_context_set_fill_color(ctx, colors->bg_color);
  graphics_fill_rect(ctx, &layer->bounds);
  prv_update_item(ctx, item_layer);
}

void timeline_item_layer_init(TimelineItemLayer *item_layer, const GRect *frame) {
  *item_layer = (TimelineItemLayer){};
  layer_init(&item_layer->layer, frame);
  layer_set_update_proc(&item_layer->layer, timeline_item_layer_update_proc);
  layer_set_clips(&item_layer->layer, false);
}

void timeline_item_layer_deinit(TimelineItemLayer *item_layer) {
  property_animation_destroy(item_layer->animation);
  layer_deinit(&item_layer->layer);
  if (item_layer->timeline_layout) {
    layout_destroy((LayoutLayer *)item_layer->timeline_layout);
    item_layer->timeline_layout = NULL;
  }
}

void timeline_item_layer_set_item(TimelineItemLayer *item_layer, TimelineItem *item,
    TimelineLayoutInfo *info) {
  item_layer->item = item;
  if (item_layer->timeline_layout) {
    layer_remove_from_parent((Layer *)item_layer->timeline_layout);
    layout_destroy((LayoutLayer *)item_layer->timeline_layout);
  }
  const LayoutLayerConfig config = {
    .frame = &(GRect) { GPointZero, item_layer->layer.frame.size },
    .attributes = &item_layer->item->attr_list,
    .mode = LayoutLayerModeCard,
    .app_id = &item->header.parent_id,
    .context = info,
  };
  item_layer->timeline_layout =
      (TimelineLayout *)layout_create(item_layer->item->header.layout, &config);
  layer_add_child(&item_layer->layer, (Layer *)item_layer->timeline_layout);
}
