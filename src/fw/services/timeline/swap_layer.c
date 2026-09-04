/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/timeline/swap_layer.h"

#include "applib/applib_malloc.auto.h"
#include "applib/graphics/graphics.h"
#include "applib/graphics/gtypes.h"
#include "applib/ui/property_animation.h"
#include "applib/ui/status_bar_layer.h"
#include "applib/ui/window.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/timeline/layout_layer.h"
#include "pbl/services/timeline/notification_layout.h"
#include "pbl/services/notifications/alerts_preferences_private.h"
#include "kernel/ui/kernel_ui.h"
#include "process_state/app_state/app_state.h"
#include "pbl/util/math.h"

#ifdef CONFIG_TOUCH
#include "applib/ui/recognizer/touch_nav.h"
#include "kernel/pebble_tasks.h"
#endif

// Initial pixel scroll amount, paging_height (LAYOUT_HEIGHT) for circular display
#define INITIAL_SCROLL_PX PBL_IF_RECT_ELSE(LAYOUT_BANNER_HEIGHT_RECT, LAYOUT_HEIGHT)

// Max pixel scroll amount, paging height (LAYOUT_HEIGHT) for circular display
#define SCROLL_PX PBL_IF_RECT_ELSE(48, LAYOUT_HEIGHT)

// Max pixel scroll for repeating scrolls (button is held)
#define REPEATING_SCROLL_PX 24

// Scroll animation speed
// Same as the normal moook duration, but one frame shorter
#define SCROLL_MS PBL_IF_RECT_ELSE(200, interpolate_moook_duration() - \
                                   ANIMATION_TARGET_FRAME_INTERVAL_MS)

// Swap animation speed
// Adding ANIMATION_TARGET_FRAME_INTERVAL_MS doesn't actually add a frame, because plain moook
// has a fixed number of frames. Adding it in just adds more time between the frames
#define SWAP_MS PBL_IF_RECT_ELSE(200, 150)

// Pixel peek amount for next layout
#define PEEK_PX PBL_IF_RECT_ELSE(LAYOUT_BANNER_HEIGHT_RECT, 0)

// Get within FUDGE_PX to the end of the layout on a scroll, and will
// scroll the rest of the way.
// Uses 0 for circular display to support paging
#define FUDGE_PX PBL_IF_RECT_ELSE(PEEK_PX, 0)

// Delay for the next scroll down to happen
#define SCROLL_REPEAT_MS 200

//! Delay for when button is held at message edge. In multiples of 100ms
//! (based on repeating click handler).
#define MESSAGE_SWAP_DELAY 3

typedef enum {
  ScrollAnimationCurveKind_Interpolator,
  ScrollAnimationCurveKind_Curve
} ScrollAnimationCurveKind;

typedef struct {
  ScrollAnimationCurveKind swap_curve_kind;
  union {
    struct { InterpolateInt64Function interpolator; };
    struct { AnimationCurve curve; };
  };
} ScrollAnimationCurve;

///////////////////////
// HELPER FUNCTIONS
///////////////////////

//! Helper to prevent casting all over the place.
static void prv_layout_set_frame(LayoutLayer *layout, const GRect *frame) {
  layer_set_frame((Layer *)layout, frame);
}

static void prv_finish_animation(SwapLayer *swap_layer) {
  if (animation_is_scheduled(swap_layer->animation)) {
    animation_set_elapsed(swap_layer->animation,
                          animation_get_duration(swap_layer->animation, true, true));
    animation_unschedule(swap_layer->animation);
  }
}

//! Calls the remove callback and deinits the layout.
//! The client is responsible to removing any data that is associated with the layout, including
//! itself
static void prv_remove_old_layout(SwapLayer *swap_layer, LayoutLayer *layout) {
  if (!layout) {
    return;
  }

  layer_remove_from_parent((Layer *)layout);

  if (layout && swap_layer->callbacks.layout_removed_handler) {
    swap_layer->callbacks.layout_removed_handler(swap_layer, layout, swap_layer->context);
  }
}

//! Calls the fetch layout callback and gets the layout that is at an offset of rel_change from the
//! current index (which is stored and kept track of by the client)
static LayoutLayer *prv_fetch_next_layout(SwapLayer *swap_layer, int8_t rel_change) {
  if (swap_layer->callbacks.get_layout_handler) {
    LayoutLayer *layout = swap_layer->callbacks.get_layout_handler(swap_layer, rel_change,
                                                                   swap_layer->context);
    // if there is no layout, we return NULL
    if (!layout) {
      return NULL;
    }

    // Calculate the size of the layout we were given and set the frame.
    GSize size = layout_get_size(graphics_context_get_current_context(), layout);
    prv_layout_set_frame(layout, &(GRect) {
      .size.w = MAX(swap_layer->layer.frame.size.w, size.w),
      .size.h = MAX(swap_layer->layer.frame.size.h, size.h),
    });
    return layout;
  }
  return NULL;
}

static void prv_announce_layout_will_appear(SwapLayer *swap_layer, LayoutLayer *layout) {
  swap_layer->swap_in_progress = true;
  if (swap_layer->callbacks.layout_will_appear_handler) {
    return swap_layer->callbacks.layout_will_appear_handler(swap_layer, layout,
                                                            swap_layer->context);
  }
}

static void prv_announce_layout_did_appear(SwapLayer *swap_layer, LayoutLayer *layout,
                                           int8_t rel_change) {
  swap_layer->swap_in_progress = false;
  if (swap_layer->callbacks.layout_did_appear_handler) {
    swap_layer->callbacks.layout_did_appear_handler(swap_layer, layout, rel_change,
                                                   swap_layer->context);
  }
}

static void prv_update_colors(SwapLayer *swap_layer, GColor bg_color,
                              bool status_bar_filled) {
  if (swap_layer->callbacks.update_colors_handler) {
    swap_layer->callbacks.update_colors_handler(swap_layer, bg_color, status_bar_filled,
                                                swap_layer->context);
  }
}

static void prv_announce_interaction(SwapLayer *swap_layer) {
  if (swap_layer->callbacks.interaction_handler) {
    swap_layer->callbacks.interaction_handler(swap_layer, swap_layer->context);
  }
}

static void prv_refresh_next_layer(SwapLayer *swap_layer) {
  prv_remove_old_layout(swap_layer, swap_layer->next);
  LayoutLayer *new_next = prv_fetch_next_layout(swap_layer, +1);

  swap_layer->next = new_next;

  if (new_next) {
    const GRect *current_frame = &swap_layer->current->layer.frame;
    GRect next_frame = swap_layer->next->layer.frame;

    next_frame.origin.y = current_frame->origin.y + current_frame->size.h;
    prv_layout_set_frame(swap_layer->next, &next_frame);
    layer_add_child(&swap_layer->layer, (Layer *)swap_layer->next);
    layer_insert_below_sibling((Layer *)swap_layer->next, &swap_layer->arrow_layer.layer);
  }
}

//////////////////////////
// STATUS LAYER FUNCTIONS
//////////////////////////

static void prv_update_status_bar_color(SwapLayer *swap_layer) {
  // Set initial values to assume failure
  bool color_status_bar = false;
  GColor bg_color = GColorClear;

  // If there is a current layout, then fetch the color and whether or not the status bar
  // should be colored
  if (swap_layer->current) {
    const GRect *cur_frame = &swap_layer->current->layer.frame;
#if PBL_RECT
    // PBL-23115 status_bar can be off, so detect within range, updated a frame later.
    color_status_bar = WITHIN(cur_frame->origin.y, -(3 * LAYOUT_BANNER_HEIGHT_RECT/ 2) - 1,
                              LAYOUT_BANNER_HEIGHT_RECT / 2 - 1);
#else
    color_status_bar = WITHIN(cur_frame->origin.y, -96, 66) || swap_layer->swap_in_progress;
#endif
    bg_color = layout_get_colors(swap_layer->current)->bg_color;

#if PBL_BW
    // On a black and white display, use the appropriate status bar color based on design style
    const bool use_alternative_design = alerts_preferences_get_notification_alternative_design();
    bg_color = use_alternative_design ? GColorBlack : GColorLightGray;
#endif
  }

  prv_update_colors(swap_layer, bg_color, color_status_bar);
}

/////////////////////////
// ARROW LAYER FUNCTIONS
/////////////////////////

//! Hides or shows the arrow (or two arrows, on S4) depending on where the layouts sit within the
//! primary layer.
static void prv_update_arrow(SwapLayer *swap_layer) {
  bool hide_it = true;

  // If there is a current layout, then we will need to compute whether to show the arrow
  // If there is no current layout, then we should hide it.
  if (swap_layer->current) {
    const GRect *cur_frame = &swap_layer->current->layer.frame;
    const GRect *layer_frame = &swap_layer->layer.frame;
    const bool viewing_entire_notif = (cur_frame->size.h == layer_frame->size.h);
#if PBL_ROUND
    const bool at_bottom = (cur_frame->origin.y < (-cur_frame->size.h + DISP_ROWS));
    const bool text_visible =
        WITHIN(cur_frame->origin.y, TEXT_VISIBLE_LOWER_THRESHOLD(cur_frame->size.h),
               TEXT_VISIBLE_UPPER_THRESHOLD);
    if (!viewing_entire_notif && !at_bottom && text_visible) {
      hide_it = false;
    }
#else
    const bool at_top = (cur_frame->origin.y == 0);
    if (at_top && (!viewing_entire_notif || swap_layer->next)) {
      hide_it = false;
    }
#endif
  }

  layer_set_hidden(&swap_layer->arrow_layer.layer, hide_it);
}

static void prv_arrow_layer_update_proc(Layer *layer, GContext* ctx) {
  ArrowLayer *arrow_layer = (ArrowLayer *)layer;

  const GRect *layer_bounds = &layer->bounds;

#if PBL_RECT
  graphics_context_set_fill_color(ctx, GColorWhite);
  graphics_fill_rect(ctx, layer_bounds);
#endif

  GRect arrow_bounds = arrow_layer->arrow_bitmap.bounds;
  const GAlign arrow_alignment = PBL_IF_RECT_ELSE(GAlignTop, GAlignBottom);
  grect_align(&arrow_bounds, layer_bounds, arrow_alignment, false /* clip */);
  const int16_t arrow_nudge_y = PBL_IF_RECT_ELSE(7, -8);
  arrow_bounds.origin.y += arrow_nudge_y;

  // FIXME PBL-43428:
  // For some reason the down arrow bitmap is drawn as all-black in the test_notification_window
  // unit test on Silk unless we draw it with GCompOpSet, yet this results in the arrow being drawn
  // as all-white on a real Silk watch/QEMU; choosing the compositing mode this way ensures the
  // arrow is drawn correctly in both environments
#if UNITTEST
  const GCompOp compositing_mode = GCompOpSet;
#else
  const GCompOp compositing_mode = PBL_IF_COLOR_ELSE(GCompOpSet, GCompOpAssign);
#endif
  graphics_context_set_compositing_mode(ctx, compositing_mode);

  graphics_draw_bitmap_in_rect(ctx, &arrow_layer->arrow_bitmap, &arrow_bounds);
}

////////////////////////////////
// SWAPPING/ANIMATION FUNCTIONS
////////////////////////////////

//! Returns the current offset of the notification
//! Always returns a positive number.
static int16_t prv_get_current_notification_offset(SwapLayer *swap_layer) {
  Layer *current = (Layer *)swap_layer->current;
  // produce positive representation of the offset of the layer from the top of the frame.
  int16_t offset = - current->frame.origin.y;
  return offset;
}

static void prv_frame_scroll_complete(Animation *animation, bool finished, void *context) {
  if (!finished) {
    GRect end;
    PropertyAnimation *property_animation = (PropertyAnimation *)animation;
    Layer *layer = context;
    property_animation_get_to_grect(property_animation, &end);
    layer_set_frame(layer, &end);
  }
}

//! Creates an animation that moves a layer from its original frame to another frame that is offset
//! dy (change in origin.y).
static Animation *prv_create_anim_frame_scroll(Layer *layer, uint32_t duration, int16_t dy,
                                               ScrollAnimationCurve *curve) {
  if (!layer) {
    return NULL;
  }

  GRect to_origin = layer->frame;
  to_origin.origin.y += dy;
  Animation *result = (Animation *) property_animation_create_layer_frame(
      layer, NULL, &to_origin);
  animation_set_handlers(result, (AnimationHandlers) {
    .stopped = prv_frame_scroll_complete,
  }, layer);
  animation_set_duration(result, duration);
#if PBL_ROUND
  if (curve) {
    switch (curve->swap_curve_kind) {
      case ScrollAnimationCurveKind_Curve:
        animation_set_curve(result, curve->curve);
        break;
      case ScrollAnimationCurveKind_Interpolator:
        animation_set_custom_interpolation(result, curve->interpolator);
        break;
    }
  }
#endif
  return result;
}

static void prv_swap_up_start(Animation *animation, void *context) {
  SwapLayer *swap_layer = (SwapLayer *)context;
  prv_announce_layout_will_appear(swap_layer, swap_layer->previous);
}

static void prv_swap_up_complete(Animation *animation, bool finished, void *context) {
  SwapLayer *swap_layer = (SwapLayer *) context;
  if (swap_layer->is_deiniting) {
    return;
  }

  // remove the layout
  prv_remove_old_layout(swap_layer, swap_layer->next);

  // shift all of the indexes
  swap_layer->next = swap_layer->current;
  swap_layer->current = swap_layer->previous;
  swap_layer->previous = NULL;

  // let the client know we have moved to another layer
  prv_announce_layout_did_appear(swap_layer, swap_layer->current, -1);

  // refresh the layer down below in case we have jumped around in our data model
  prv_refresh_next_layer(swap_layer);
}

static Animation *prv_create_swap_up_animation(SwapLayer *swap_layer, bool full_swap) {
  int16_t dy;

  const GRect *prev_frame = &swap_layer->previous->layer.frame;
  const GRect *current_frame = &swap_layer->current->layer.frame;

  if (full_swap) {
    dy = prev_frame->size.h - current_frame->origin.y;
  } else {
    dy = swap_layer->layer.frame.size.h - PEEK_PX;
  }

  Animation *prev_down = prv_create_anim_frame_scroll((Layer *)swap_layer->previous, SWAP_MS, dy,
                                                      NULL);
  Animation *current_down = prv_create_anim_frame_scroll((Layer *)swap_layer->current, SWAP_MS, dy,
                                                         NULL);
  Animation *next_down = prv_create_anim_frame_scroll((Layer *)swap_layer->next, SWAP_MS, dy,
                                                      NULL);
  return animation_spawn_create(prev_down, current_down, next_down, NULL);
}

static bool prv_setup_swap_up(SwapLayer *swap_layer) {
  LayoutLayer *new_previous = prv_fetch_next_layout(swap_layer, -1);

  // if there is no layout to swap up to, abort
  if (!new_previous) {
    return false;
  }

  swap_layer->previous = new_previous;

  GRect prev_frame = swap_layer->previous->layer.frame;
  const GRect *current_frame = &swap_layer->current->layer.frame;

  // set relative offset for the previous layer
  prev_frame.origin.y = -prev_frame.size.h + current_frame->origin.y;
  prv_layout_set_frame(swap_layer->previous, &prev_frame);
  layer_add_child(&swap_layer->layer, (Layer *)swap_layer->previous);
  layer_insert_below_sibling((Layer *)swap_layer->previous, &swap_layer->arrow_layer.layer);

  return true;
}

static void prv_swap_down_start(Animation *animation, void *context) {
  SwapLayer *swap_layer = (SwapLayer *)context;
  prv_announce_layout_will_appear(swap_layer, swap_layer->current);
}

static void prv_swap_down_complete(Animation *animation, bool finished, void *context) {
  SwapLayer *swap_layer = (SwapLayer *) context;
  if (swap_layer->is_deiniting) {
    return;
  }

  // remove the previous layer
  prv_remove_old_layout(swap_layer, swap_layer->previous);
  swap_layer->previous = NULL;

  // let the client know we have moved to another layer
  prv_announce_layout_did_appear(swap_layer, swap_layer->current, +1);

  // refresh the layer down below in case we have jumped around in our data model
  prv_refresh_next_layer(swap_layer);
}

static Animation *prv_create_swap_down_animation(SwapLayer *swap_layer) {
  // Compute the animation distance
  const GRect *prev_frame = &swap_layer->previous->layer.frame;

  int16_t dy = -(prev_frame->origin.y + prev_frame->size.h);
  ScrollAnimationCurve swap_down_scroll_curve = (ScrollAnimationCurve) {
      .swap_curve_kind = ScrollAnimationCurveKind_Curve,
      .curve = AnimationCurveEaseOut
  };
  Animation *prev_up = prv_create_anim_frame_scroll((Layer *)swap_layer->previous, SWAP_MS, dy,
                                                    &swap_down_scroll_curve);
  Animation *current_up = prv_create_anim_frame_scroll((Layer *)swap_layer->current, SWAP_MS, dy,
                                                       &swap_down_scroll_curve);
  // next might return NULL if there is no next. That's OK since animation_spawn CAN take two NULL's
  // and perform the creation correctly.
  Animation *next_up = prv_create_anim_frame_scroll((Layer *)swap_layer->next, SWAP_MS, dy,
                                                    &swap_down_scroll_curve);
  return animation_spawn_create(prev_up, current_up, next_up, NULL);
}

static bool prv_setup_swap_down(SwapLayer *swap_layer) {
  // if there is no layout to swap down to, return
  if (!swap_layer->next) {
    return false;
  }

  // shift all of the indexes. No need to fetch the new next layer, since we will do that
  // in the swap down complete
  swap_layer->previous = swap_layer->current;
  swap_layer->current = swap_layer->next;
  swap_layer->next = NULL;

  return true;
}

static void prv_scroll(SwapLayer *swap_layer, int16_t dy, AnimationCurve curve) {
  if (dy == 0) {
    return;
  }
  ScrollAnimationCurve moook_scroll_curve = (ScrollAnimationCurve) {
      .swap_curve_kind = ScrollAnimationCurveKind_Interpolator,
      .interpolator = interpolate_moook
  };
  Animation *current = prv_create_anim_frame_scroll((Layer *)swap_layer->current, SCROLL_MS, dy,
                                                    &moook_scroll_curve);
#if PBL_RECT
  animation_set_curve(current, curve);
#endif
  Animation *animation;
  if (swap_layer->next) {
    Animation *next = prv_create_anim_frame_scroll((Layer *)swap_layer->next, SCROLL_MS, dy, NULL);
#if PBL_RECT
    animation_set_curve(next, curve);
#endif
    animation = animation_spawn_create(current, next, NULL);
  } else {
    animation = current;
  }
  swap_layer->animation = animation;
  animation_schedule(animation);
}

// scroll to top of current notification
static void prv_scroll_to_top(SwapLayer *swap_layer) {
  int16_t offset = prv_get_current_notification_offset(swap_layer);
  prv_scroll(swap_layer, offset, AnimationCurveEaseOut);
}

// scroll to bottom of current notification
static void prv_scroll_to_bottom(SwapLayer *swap_layer) {
  const GRect *current_frame = &swap_layer->current->layer.frame;
  const GRect *layer_frame = &swap_layer->layer.frame;
  // don't allow scrolling up
  if (current_frame->size.h < layer_frame->size.h) {
    prv_scroll(swap_layer, -current_frame->size.h - current_frame->origin.y + layer_frame->size.h,
               AnimationCurveEaseOut);
  }
}

//! Attempt to swap up or down. 'full_swap' is for swapping up, and it is used when wanting to go
//! to the top of the previous notification.
static bool prv_attempt_swap(SwapLayer *swap_layer, ScrollDirection direction, bool full_swap) {
  prv_finish_animation(swap_layer);

  Animation *animation;
  if (direction == ScrollDirectionDown) {
    if (!prv_setup_swap_down(swap_layer)) {
      // failed to swap, just scroll
      prv_scroll_to_bottom(swap_layer);
      return false;
    }

    animation = prv_create_swap_down_animation(swap_layer);
  } else {  // ScrollDirectionUp
    if (!prv_setup_swap_up(swap_layer)) {
      // failed to swap, just scroll
      prv_scroll_to_top(swap_layer);
      return false;
    }

    animation = prv_create_swap_up_animation(swap_layer, full_swap);
  }

  animation_set_handlers(animation, (AnimationHandlers) {
    .started = (direction == ScrollDirectionDown) ? prv_swap_down_start    : prv_swap_up_start,
    .stopped = (direction == ScrollDirectionDown) ? prv_swap_down_complete : prv_swap_up_complete,
  }, swap_layer);
  swap_layer->animation = animation;
  animation_schedule(animation);
#if PBL_ROUND
  // Skip the animation on round, because it looks bad
  if (full_swap) {
    animation_set_elapsed(animation, animation_get_duration(animation, true, true));
  }
#endif
  return true;
}

//! Computes the amount the current layer frame can scroll until a swap is necessary.
//! Always returns a positive number
static int16_t prv_get_max_scroll_dy(SwapLayer *swap_layer) {
  int16_t max_dy = swap_layer->current->layer.frame.size.h - swap_layer->layer.frame.size.h;

  // we are peeking the next notification if we have a next.
  if (swap_layer->next != NULL) {
    max_dy += PEEK_PX;
  } else {
#if PBL_ROUND
    // The last notification has to be able to scroll past content to stay page aligned
    max_dy = ROUND_TO_MOD_CEIL(max_dy, LAYOUT_HEIGHT);
#endif
  }

  max_dy = MAX(max_dy, 0);
  return max_dy;
}

static void prv_handle_swap_attempt(SwapLayer *swap_layer, ScrollDirection direction,
                                    bool is_repeating) {
  // if this is a regular click or we have waited long enough
  if (!is_repeating || swap_layer->swap_delay_remaining <= 0) {
    prv_attempt_swap(swap_layer, direction, false /* to_top */);
    swap_layer->swap_delay_remaining = MESSAGE_SWAP_DELAY;
  } else {
    swap_layer->swap_delay_remaining--;
  }
}

T_STATIC void prv_attempt_scroll(SwapLayer *swap_layer, ScrollDirection direction,
                                 bool is_repeating) {
  prv_finish_animation(swap_layer);

#if PBL_ROUND
  is_repeating = false;
#endif

  int16_t offset = prv_get_current_notification_offset(swap_layer);
  int16_t max_dy = prv_get_max_scroll_dy(swap_layer);

  // distance to scroll
  int16_t dy = 0;

  // check if we are going to go off screen, if so get a new layer and set it up, then animate.
  switch (direction) {
    case ScrollDirectionUp:
    {
      if (offset == 0) {
        // we are at the topmost part of the notification, swap up
        prv_handle_swap_attempt(swap_layer, direction, is_repeating);
        return;
      } else if ((offset - FUDGE_PX) < SCROLL_PX) {
        // we have a little room between the top of notification and top of frame
        dy = offset;
      } else {
        // so much roooooooooom, scroll up normal amount
        dy = (is_repeating) ? REPEATING_SCROLL_PX : SCROLL_PX;
      }
      break;
    }
    case ScrollDirectionDown:
    {
#if PBL_RECT
      if (max_dy == offset) {
        // if we have already scrolled our maximum amount for this notification, we should swap
        prv_handle_swap_attempt(swap_layer, direction, is_repeating);
        return;
      }
#else
      if (offset >= swap_layer->current->layer.bounds.size.h - DISP_ROWS) {
        prv_handle_swap_attempt(swap_layer, direction, is_repeating);
        return;
      }
#endif

      // pause at the top of a notification
      if ((offset == 0) &&
          (is_repeating) &&
          (swap_layer->swap_delay_remaining > 0)) {
        swap_layer->swap_delay_remaining--;
        return;
      }

      swap_layer->swap_delay_remaining = MESSAGE_SWAP_DELAY;
      if ((max_dy - offset - FUDGE_PX) < SCROLL_PX) {
        // if we can scroll a little more, let's do that instead.
        // negative because we need to scroll down
        dy = -(max_dy - offset);
      } else {
        // if we have a lot of scrolling leg room, let's scroll down the full amount.
        const GRect *cur_frame = &swap_layer->current->layer.frame;

        // scroll only the banner when it's the first scroll on a notification
        if (cur_frame->origin.y == 0) {
          dy = -INITIAL_SCROLL_PX;
        } else {
          dy = (is_repeating) ? -REPEATING_SCROLL_PX : -SCROLL_PX;
        }
      }
      break;
    }
    default:
      return;
  }

  const AnimationCurve curve = (is_repeating) ? AnimationCurveLinear : AnimationCurveEaseOut;
  prv_scroll(swap_layer, dy, curve);
}

///////////////////////
// CLICK HANDLERS
///////////////////////

ScrollDirection prv_direction_for_recognizer(ClickRecognizerRef recognizer) {
  ButtonId button_id = click_recognizer_get_button_id(recognizer);
  return (button_id == BUTTON_ID_UP) ? ScrollDirectionUp : ScrollDirectionDown;
}

static void prv_single_click_handler(ClickRecognizerRef recognizer, void *context) {
  SwapLayer *swap_layer = (SwapLayer *)context;
  if (click_recognizer_is_repeating(recognizer)) {
    prv_announce_interaction(swap_layer);
    const bool is_repeating = true;
    prv_attempt_scroll(swap_layer, prv_direction_for_recognizer(recognizer), is_repeating);
  }
}

static void prv_up_multi_click_handler(ClickRecognizerRef recognizer, void *context) {
  SwapLayer *swap_layer = (SwapLayer *)context;
  prv_finish_animation(swap_layer);

  prv_announce_interaction(swap_layer);
  // If our first click caused a swap, then just scroll to the top of the current notification
  int16_t offset = prv_get_current_notification_offset(swap_layer);
  const GRect *layer_frame = &swap_layer->layer.frame;
  const GRect *current_frame = &swap_layer->current->layer.frame;
  if ((current_frame->size.h - layer_frame->size.h - offset) ==
      -PBL_IF_RECT_ELSE(LAYOUT_BANNER_HEIGHT_RECT, LAYOUT_TOP_BANNER_HEIGHT_ROUND)) {
    prv_scroll_to_top(swap_layer);
  } else {
    prv_attempt_swap(swap_layer, ScrollDirectionUp, true /* to_top */);
  }
}

static void prv_down_multi_click_handler(ClickRecognizerRef recognizer, void *context) {
  SwapLayer *swap_layer = (SwapLayer *)context;
  prv_finish_animation(swap_layer);

  // If our first click caused a swap, then ignore the double click
  int16_t offset = prv_get_current_notification_offset(swap_layer);
  if (offset != 0) {
    prv_announce_interaction(swap_layer);
    prv_attempt_swap(swap_layer, ScrollDirectionDown, true /* to_top */);
  }
}

static void prv_raw_click_handler(ClickRecognizerRef recognizer, void *context) {
  SwapLayer *swap_layer = (SwapLayer *)context;
  // Ignore any press that isn't the start of a new click pattern
  if (click_number_of_clicks_counted(recognizer) < 1) {
    prv_announce_interaction(swap_layer);
    const bool is_repeating = false;
    prv_attempt_scroll(swap_layer, prv_direction_for_recognizer(recognizer), is_repeating);
  }
}

static void prv_swap_layer_click_config_provider(void *context) {
  // Use raw clicks to avoid single click delay which results from having multi-click enabled
  window_raw_click_subscribe(BUTTON_ID_UP, prv_raw_click_handler, NULL, context);
  window_single_repeating_click_subscribe(BUTTON_ID_UP, SCROLL_REPEAT_MS,
                                          prv_single_click_handler);
  window_multi_click_subscribe(BUTTON_ID_UP, 2, 2, 100, false, prv_up_multi_click_handler);

  // Use raw clicks to avoid single click delay which results from having multi-click enabled
  window_raw_click_subscribe(BUTTON_ID_DOWN, prv_raw_click_handler, NULL, context);
  window_single_repeating_click_subscribe(BUTTON_ID_DOWN, SCROLL_REPEAT_MS,
                                          prv_single_click_handler);
  window_multi_click_subscribe(BUTTON_ID_DOWN, 2, 2, 100, false, prv_down_multi_click_handler);

  SwapLayer *swap_layer = context;
#ifdef CONFIG_TOUCH
  // Re-registering here (idempotent) makes the click-config-provider the re-show hook: after a higher
  // modal that released touch is dismissed and this window regains input focus, the swap layer is
  // re-subscribed as the sole Tier-1 touch handler.
  swap_layer_touch_register(swap_layer);
#endif
  if (swap_layer->callbacks.click_config_provider) {
    swap_layer->callbacks.click_config_provider(swap_layer->context);
  }
}

///////////////////////
// MISC FUNCTIONS
///////////////////////

static void prv_swap_layer_update_proc(Layer *layer, GContext* ctx) {
  SwapLayer *swap_layer = (SwapLayer *)layer;
  prv_update_arrow(swap_layer);
  prv_update_status_bar_color(swap_layer);
}

static void prv_swap_layer_reset(SwapLayer *swap_layer) {
  prv_finish_animation(swap_layer);

  prv_remove_old_layout(swap_layer, swap_layer->previous);
  swap_layer->previous = NULL;

  prv_remove_old_layout(swap_layer, swap_layer->current);
  swap_layer->current = NULL;

  prv_remove_old_layout(swap_layer, swap_layer->next);
  swap_layer->next = NULL;
}

void swap_layer_reload_data(SwapLayer *swap_layer) {
  prv_swap_layer_reset(swap_layer);

  LayoutLayer *current = prv_fetch_next_layout(swap_layer, 0);
  if (!current) {
    return;
  }

  GRect current_frame = current->layer.frame;
  current_frame.origin = (GPoint) {0, 0};
  prv_layout_set_frame(current, &current_frame);
  layer_add_child(&swap_layer->layer, (Layer *)current);
  layer_insert_below_sibling((Layer *)current, &swap_layer->arrow_layer.layer);
  swap_layer->current = current;

  prv_announce_layout_will_appear(swap_layer, swap_layer->current);
  prv_announce_layout_did_appear(swap_layer, swap_layer->current, 0);

  LayoutLayer *next = prv_fetch_next_layout(swap_layer, +1);

  if (next) {
    GRect next_frame = next->layer.frame;
    next_frame.origin.y = swap_layer->current->layer.frame.size.h;
    prv_layout_set_frame(next, &next_frame);
    layer_add_child(&swap_layer->layer, (Layer *)next);
    layer_insert_below_sibling((Layer *)next, &swap_layer->arrow_layer.layer);
    swap_layer->next = next;
  }
}

bool swap_layer_attempt_layer_swap(SwapLayer *swap_layer, ScrollDirection direction) {
  return prv_attempt_swap(swap_layer, direction, true /* to_top */);
}

#ifdef CONFIG_TOUCH
///////////////////////
// TIER-1 TOUCH NAVIGATION
//
// A SwapLayer registers itself as a Tier-1 touch widget in swap_layer_init() / the
// click-config-provider and deregisters in swap_layer_deinit() (and, while covered by a higher
// modal, in swap_layer_touch_release()). The recognizers and gesture state are not owned per-widget:
// the unified widget (tap, pan, swipe) set, owned by TouchNavState, drives whichever migrated widget
// the finger lands on through a per-node apply vtable. A SwapLayer supplies that vtable
// (s_swap_touch_nav_ops) at registration; the apply functions below stay the per-swap gesture
// surface (and the unit-test entry points).
//
// A pan drives the content offset live and 1:1 (base captured at pan Start, throttled by the core),
// clamped to [0, max_scroll]. On liftoff a pull past the clamped edge by more than SWAP_OVERPULL_PX
// swaps to the neighbouring notification through the same physical-button path
// (prv_handle_swap_attempt); a normal drag settles. A tap or left swipe emulates SELECT; a right
// swipe emulates BACK. The notification body may not be loaded when the widget registers, so the
// can_start op gates the gesture on a non-NULL current layout (prv_get_current_notification_offset
// dereferences it), declining the whole gesture until a notification is present.
///////////////////////

// Finger travel, in pixels, before a pan counts as a drag. Below this a liftoff is a no-op, so a
// normal scroll to the edge (or a jittery tap) cannot accidentally scroll or swap.
#define DRAG_THRESHOLD_PX 10
// How far the finger must pull PAST the clamped edge to swap to the neighbouring notification. Kept
// separate from DRAG_THRESHOLD_PX so reaching the edge without over-pulling never swaps.
#define SWAP_OVERPULL_PX 40

// Provided by the owning shells; forward-declared to avoid pulling the kernel modal header in.
struct TouchNavState *modal_manager_get_touch_nav_state(void);

_Static_assert(sizeof(((SwapLayer *)0)->touch_nav_node) == sizeof(TouchNavWidgetNode),
               "SwapLayer touch_nav_node must match TouchNavWidgetNode layout");

static bool prv_is_app_task(void) {
  return pebble_task_get_current() == PebbleTask_App;
}

static TouchNavState *prv_task_touch_nav_state(void) {
  return prv_is_app_task() ? app_state_get_touch_nav_state() : modal_manager_get_touch_nav_state();
}

// Test seam (declared in swap_layer.h under CONFIG_TOUCH). The unified widget set holds no per-task
// singleton to reset; the touch-nav state is owned by TouchNavState, so this is a no-op kept for
// source compatibility with tests that call it in their setup.
void swap_layer_touch_nav_reset_all(void) {
}

bool swap_layer_touch_is_gesture_target(const SwapLayer *swap_layer) {
  const TouchNavState *state = prv_task_touch_nav_state();
  return state && state->latched_target && state->latched_target->widget == swap_layer;
}

// ---------------------------------------------------------------------------------------------
// The scroll point and the two-threshold liftoff decision

void swap_layer_touch_scroll_by(SwapLayer *swap_layer, int16_t dy) {
  if (!swap_layer || !swap_layer->current) {
    return;
  }
  prv_finish_animation(swap_layer);

  const int16_t offset = prv_get_current_notification_offset(swap_layer);  // >= 0
  const int16_t max_dy = prv_get_max_scroll_dy(swap_layer);                // >= 0
  // offset == -origin.y; dy is the requested change to origin.y, so the new offset is offset - dy.
  const int16_t new_offset = CLIP(offset - dy, 0, max_dy);

  GRect current_frame = swap_layer->current->layer.frame;
  current_frame.origin.y = -new_offset;
  prv_layout_set_frame(swap_layer->current, &current_frame);

  // Pull the next layout right under the current one so its peek tracks the finger.
  if (swap_layer->next) {
    GRect next_frame = swap_layer->next->layer.frame;
    next_frame.origin.y = current_frame.origin.y + current_frame.size.h;
    prv_layout_set_frame(swap_layer->next, &next_frame);
  }

  // Refresh the auto-close timer: without this a long notification closes under the finger mid-read.
  prv_announce_interaction(swap_layer);
}

SwapTouchLiftoffAction swap_layer_touch_liftoff_action(int16_t base_offset, int16_t delta_y,
                                                       int16_t max_dy) {
  if (ABS(delta_y) < DRAG_THRESHOLD_PX) {
    // Sub-threshold: not a drag, so neither scroll nor swap.
    return SwapTouchLiftoff_None;
  }
  // Content-scroll convention: finger up (delta_y < 0) scrolls into the content (offset increases).
  const int16_t requested_offset = base_offset - delta_y;
  if (requested_offset < -SWAP_OVERPULL_PX) {
    return SwapTouchLiftoff_SwapPrev;  // over-pull at the top (finger down) -> previous notification
  }
  if (requested_offset > max_dy + SWAP_OVERPULL_PX) {
    return SwapTouchLiftoff_SwapNext;  // over-pull at the bottom (finger up) -> next notification
  }
  return SwapTouchLiftoff_Settle;
}

int16_t swap_layer_touch_settle_offset(int16_t base_offset, int16_t delta_y, int16_t max_dy,
                                       int16_t page_h) {
  int16_t target = CLIP(base_offset - delta_y, 0, max_dy);
  if (page_h > 0) {
    // Land on the nearest page boundary, never past the last one (max_dy may include the
    // next-notification peek, which is not a rest position).
    const int16_t last_page = (int16_t)((max_dy / page_h) * page_h);
    target = (int16_t)(((target + page_h / 2) / page_h) * page_h);
    target = MIN(target, last_page);
  }
  return target;
}

// Apply a live drag: move to clamp(base_offset - delta_y). Sub-threshold movement is a no-op so the
// notification does not creep on a tap.
static void prv_swap_touch_apply_drag(SwapLayer *swap_layer, int16_t base_offset, int16_t delta_y) {
  if (!swap_layer->current || ABS(delta_y) < DRAG_THRESHOLD_PX) {
    // No notification loaded yet (registration can happen before the layout loads), or sub-threshold.
    return;
  }
  const int16_t requested_offset = base_offset - delta_y;
  const int16_t cur = prv_get_current_notification_offset(swap_layer);
  // dy is in origin.y units so the resulting offset lands on requested_offset (before the point's
  // own clamp handles an over-pull).
  swap_layer_touch_scroll_by(swap_layer, cur - requested_offset);
}

// Liftoff: swap to a neighbour on over-pull (same path as the physical UP/DOWN buttons), otherwise
// settle (animated) to the clamped offset.
static void prv_swap_touch_liftoff(SwapLayer *swap_layer, int16_t base_offset, int16_t delta_y) {
  if (!swap_layer->current) {
    return;
  }
  const int16_t max_dy = prv_get_max_scroll_dy(swap_layer);
  switch (swap_layer_touch_liftoff_action(base_offset, delta_y, max_dy)) {
    case SwapTouchLiftoff_SwapPrev:
      prv_handle_swap_attempt(swap_layer, ScrollDirectionUp, false /* is_repeating */);
      break;
    case SwapTouchLiftoff_SwapNext:
      prv_handle_swap_attempt(swap_layer, ScrollDirectionDown, false /* is_repeating */);
      break;
    case SwapTouchLiftoff_Settle: {
      // Round text flow is computed for page-aligned rest positions (the paging fold in
      // prv_walk_lines_down), so a touch settle must land on a page boundary or the line chords
      // no longer match where the content actually sits on the circle. Rect settles freely.
      const int16_t target_offset = swap_layer_touch_settle_offset(
          base_offset, delta_y, max_dy, PBL_IF_RECT_ELSE(0, LAYOUT_HEIGHT));
      const int16_t cur = prv_get_current_notification_offset(swap_layer);
      prv_scroll(swap_layer, cur - target_offset, AnimationCurveEaseOut);
      break;
    }
    case SwapTouchLiftoff_None:
      break;
  }
  prv_announce_interaction(swap_layer);
}

// ---------------------------------------------------------------------------------------------
// Button emulation for tap / swipe, mirroring the Tier-2 bridge through the task's TouchNavOps.

static void prv_swap_touch_emit(SwapLayer *swap_layer, ButtonId button) {
  const TouchNavState *state = prv_task_touch_nav_state();
  if (!state || !state->ops) {
    return;
  }
  const TouchNavOps *ops = state->ops;
  // A gesture that lands mid-transition is dropped, matching the bridge.
  if (ops->is_animating && ops->is_animating(ops->ctx)) {
    return;
  }
  if (button == BUTTON_ID_BACK &&
      !(ops->top_overrides_back && ops->top_overrides_back(ops->ctx))) {
    // BACK on a window with no back handler pops the stack rather than feeding the click recognizer.
    if (ops->pop_top) {
      ops->pop_top(ops->ctx);
    }
  } else if (ops->emit_button) {
    ops->emit_button(ops->ctx, button);
  }
  prv_announce_interaction(swap_layer);
}

// ---------------------------------------------------------------------------------------------
// Unified widget-set ops
//
// Thin void*->SwapLayer* wrappers over the apply functions above. The unified widget set (owned by
// TouchNavState) drives the swap layer through these; direct assignment of the apply functions would
// not compile because their first parameter is SwapLayer*, not void*. Swap's content axis is
// vertical, so the GPoint base/delta carried by the vtable use only the .y component.

// The notification body may not be loaded yet (registration can happen before the layout loads) and
// prv_get_current_notification_offset dereferences current, so gate the whole gesture on a present
// layout. A declined gesture never Starts the pan, so no base offset is latched or applied.
static bool prv_swap_ops_can_start(void *w) {
  return ((SwapLayer *)w)->current != NULL;
}

static void prv_swap_ops_pan_started(void *w) {
  SwapLayer *swap_layer = w;
  // Stop any running swap/scroll animation so the finger takes over, and refresh the auto-close timer
  // so a long notification cannot close under the finger while it is being dragged. The base offset
  // is latched by the core via get_base_offset next.
  prv_finish_animation(swap_layer);
  prv_announce_interaction(swap_layer);
}

static GPointReturn prv_swap_ops_get_base_offset(void *w) {
  // Vertical-only: carry the notification scroll offset in .y (current is non-NULL, gated by
  // can_start above).
  return GPoint(0, prv_get_current_notification_offset((SwapLayer *)w));
}

static void prv_swap_ops_pan_update(void *w, GPoint base, GPoint delta) {
  prv_swap_touch_apply_drag((SwapLayer *)w, base.y, delta.y);
}

static void prv_swap_ops_pan_snap(void *w, GPoint base, GPoint final_delta, GPoint velocity) {
  // The notification swap has page semantics; the liftoff decision is displacement-based.
  (void)velocity;
  prv_swap_touch_liftoff((SwapLayer *)w, base.y, final_delta.y);
}

static void prv_swap_ops_pan_cancel(void *w) {
  // Settle to the clamped offset with no swap (the live drag already left it clamped). Per the pan
  // engine this does not fire on pre-emption (the pan recognizer never emits Cancelled there), which
  // preserves the pre-refactor behaviour; it is reached only on an explicit mid-gesture cancel.
  SwapLayer *swap_layer = w;
  if (!swap_layer->current) {
    return;
  }
  const int16_t cur = prv_get_current_notification_offset(swap_layer);
  const int16_t max_dy = prv_get_max_scroll_dy(swap_layer);
  // Same page alignment as the liftoff settle: a cancelled pan must not rest off-grid on round.
  const int16_t target = swap_layer_touch_settle_offset(cur, 0, max_dy,
                                                        PBL_IF_RECT_ELSE(0, LAYOUT_HEIGHT));
  prv_scroll(swap_layer, cur - target, AnimationCurveEaseOut);
}

static void prv_swap_ops_tap(void *w, GPoint point_on_screen) {
  // A tap on the notification body intentionally does nothing. The action menu is opened by a left
  // swipe (prv_swap_ops_swipe -> SELECT); a bare tap must not open it by accident. The handler is
  // kept in the vtable (rather than NULL) so the tap gesture is still consumed here instead of
  // leaking to another handler.
  (void)w;
  (void)point_on_screen;
}

static void prv_swap_ops_swipe(void *w, SwipeDirection dir) {
  SwapLayer *swap_layer = w;
  // As with tap: no notification loaded -> the gesture is inert.
  if (!swap_layer->current) {
    return;
  }
  // Called only for horizontal swipes (the unified set masks vertical swipes into pans): swiping
  // right = BACK, left = SELECT.
  if (dir == SwipeDirection_Right) {
    prv_swap_touch_emit(swap_layer, BUTTON_ID_BACK);
  } else if (dir == SwipeDirection_Left) {
    prv_swap_touch_emit(swap_layer, BUTTON_ID_SELECT);
  }
}

static const TouchNavWidgetOps s_swap_touch_nav_ops = {
  .can_start = prv_swap_ops_can_start,
  .pan_started = prv_swap_ops_pan_started,
  .get_base_offset = prv_swap_ops_get_base_offset,
  .pan_update = prv_swap_ops_pan_update,
  .pan_snap = prv_swap_ops_pan_snap,
  .pan_cancel = prv_swap_ops_pan_cancel,
  .tap = prv_swap_ops_tap,
  .swipe = prv_swap_ops_swipe,
};

// ---------------------------------------------------------------------------------------------
// Registration lifecycle

void swap_layer_touch_register(SwapLayer *swap_layer) {
  if (!swap_layer || swap_layer->touch_registered) {
    return;
  }
  TouchNavState *state = prv_task_touch_nav_state();
  if (!state || !state->manager) {
    // Touch is not available on this task (nav disabled / no manager): stay on the button path.
    return;
  }

  // Register the swap layer as a migrated Tier-1 widget: the unified widget set drives it through
  // s_swap_touch_nav_ops. The registry add dedups by address and re-applies the ops/layer, so a
  // repeated init/re-show on the same swap layer (no intervening deinit) keeps routing to and
  // driving it.
  touch_nav_registry_add(state, TouchNavWidgetType_Swap,
                         (TouchNavWidgetNode *)&swap_layer->touch_nav_node, &swap_layer->layer,
                         &s_swap_touch_nav_ops, swap_layer);
  swap_layer->touch_registered = true;
}

void swap_layer_touch_deregister(SwapLayer *swap_layer) {
  if (!swap_layer || !swap_layer->touch_registered) {
    return;
  }
  TouchNavState *state = prv_task_touch_nav_state();
  if (state) {
    // If this swap layer is the live gesture target and is about to go away under a live window,
    // cancel the gesture with NO client callbacks so a settle cannot reach freed state.
    // touch_nav_registry_remove clears the latched target BEFORE we cancel (its UAF hook), so the
    // resulting Cancelled dispatch finds no target and re-enters nothing.
    const bool was_target = swap_layer_touch_is_gesture_target(swap_layer);
    // Idempotent: removing a node that is not in the registry (double deinit) is a safe no-op.
    touch_nav_registry_remove(state, TouchNavWidgetType_Swap,
                              (TouchNavWidgetNode *)&swap_layer->touch_nav_node);
    if (was_target && state->manager) {
      recognizer_manager_cancel_and_reset(state->manager);
    }
  }
  swap_layer->touch_registered = false;
}

void swap_layer_touch_release(SwapLayer *swap_layer) {
  // Covered by a higher modal: deregister from the Tier-1 registry (removing the swap node and
  // clearing the gesture latch). In our system-slot architecture the bridge lives on the
  // touch-service system handler and the unified widget set is owned by TouchNavState, so removing
  // the widget from the registry is what stops touch from routing into the now-hidden notification
  // body; the now-focused modal owns touch through the same system-slot bridge.
  swap_layer_touch_deregister(swap_layer);
}
#endif  // CONFIG_TOUCH

///////////////////////
// ACCESSOR FUNCTIONS
///////////////////////

void swap_layer_set_click_config_onto_window(SwapLayer *swap_layer, struct Window *window) {
  window_set_click_config_provider_with_context(window, prv_swap_layer_click_config_provider,
                                                swap_layer);
}

void swap_layer_set_callbacks(SwapLayer *swap_layer, void *callback_context,
                              SwapLayerCallbacks callbacks) {
  swap_layer->context = callback_context;
  swap_layer->callbacks = callbacks;

  swap_layer_reload_data(swap_layer);
}

Layer* swap_layer_get_layer(const SwapLayer *swap_layer) {
  return &((SwapLayer *)swap_layer)->layer;
}

LayoutLayer* swap_layer_get_current_layout(const SwapLayer *swap_layer) {
  return swap_layer->current;
}

///////////////////////
// INIT FUNCTIONS
///////////////////////

void swap_layer_init(SwapLayer *swap_layer, const GRect *frame) {
  *swap_layer = (SwapLayer){};

  Layer *layer = &swap_layer->layer;
  layer_init(layer, frame);
  layer_set_update_proc(layer, prv_swap_layer_update_proc);

  gbitmap_init_with_resource(&swap_layer->arrow_layer.arrow_bitmap, RESOURCE_ID_ARROW_DOWN);

  const GRect arrow_frame = GRect(
      0, frame->size.h - LAYOUT_ARROW_HEIGHT, frame->size.w, LAYOUT_ARROW_HEIGHT);
  layer_init(&swap_layer->arrow_layer.layer, &arrow_frame);
  layer_set_update_proc(&swap_layer->arrow_layer.layer, prv_arrow_layer_update_proc);
  layer_add_child(layer, &swap_layer->arrow_layer.layer);

#ifdef CONFIG_TOUCH
  swap_layer_touch_register(swap_layer);
#endif
}

void swap_layer_deinit(SwapLayer *swap_layer) {
  swap_layer->is_deiniting = true;
#ifdef CONFIG_TOUCH
  // Deinit-safety: deregister removes the swap node from the Tier-1 registry, detaches the shared
  // recognizer set, and clears the active-gesture pointer (cancelling the in-flight gesture) if it
  // targets this layer, so a torn-down widget cannot be reached by a later touch event.
  swap_layer_touch_deregister(swap_layer);
#endif
  gbitmap_deinit(&swap_layer->arrow_layer.arrow_bitmap);
  layer_deinit(&swap_layer->arrow_layer.layer);
  prv_swap_layer_reset(swap_layer);
  layer_deinit(&swap_layer->layer);
}

SwapLayer* swap_layer_create(GRect frame) {
  // Note: Not yet exported for 3rd party apps so no padding is necessary
  SwapLayer *layer = applib_malloc(sizeof(SwapLayer));
  if (layer) {
    swap_layer_init(layer, &frame);
  }
  return layer;
}

void swap_layer_destroy(SwapLayer *swap_layer) {
  if (swap_layer == NULL) {
    return;
  }
  swap_layer_deinit(swap_layer);
  applib_free(swap_layer);
}
