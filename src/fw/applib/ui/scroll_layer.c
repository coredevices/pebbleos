/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "scroll_layer.h"
#include "scroll_layer_private.h"

#include "applib/applib_malloc.auto.h"
#include "applib/graphics/gtypes.h"
#include "applib/graphics/graphics.h"
#include "applib/ui/content_indicator_private.h"
#include "applib/ui/shadows.h"
#include "applib/ui/window.h"
#include "process_management/app_manager.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/math.h"

#include "animation_timing.h"

#include <string.h>

#ifdef CONFIG_TOUCH
#include "applib/ui/recognizer/touch_nav.h"
#include "applib/ui/recognizer/recognizer_manager.h"
#include "applib/ui/recognizer/pan.h"
#include "applib/ui/recognizer/swipe.h"
#include "kernel/pebble_tasks.h"
#include "pbl/drivers/rtc.h"

// Provided by the owning task; forward-declared (as in menu_layer.c) to avoid pulling kernel
// app-state / modal-manager headers into this applib translation unit.
struct TouchNavState *app_state_get_touch_nav_state(void);
struct TouchNavState *modal_manager_get_touch_nav_state(void);
#endif

T_STATIC bool prv_scroll_layer_is_paging_enabled(ScrollLayer *scroll_layer) {
  PBL_ASSERTN(scroll_layer);
  if (process_manager_compiled_with_legacy2_sdk() || !scroll_layer->shadow_sublayer.hidden) {
    return false;
  }
  return !scroll_layer->paging.paging_disabled;
}

T_STATIC uint16_t prv_scroll_layer_get_paging_height(ScrollLayer *scroll_layer) {
  if (!prv_scroll_layer_is_paging_enabled(scroll_layer)) {
    return 0;
  }
  return MAX(0, scroll_layer->layer.frame.size.h);
}

//! Return callback_context or if NULL,
inline static void* get_callback_context(ScrollLayer *scroll_layer) {
  return scroll_layer->context ? scroll_layer->context : scroll_layer;
}

void scroll_layer_draw_shadow_sublayer(Layer *shadow_sublayer, GContext* ctx) {
  ScrollLayer *scroll_layer = (ScrollLayer *)(((uint8_t*)shadow_sublayer) - offsetof(ScrollLayer, shadow_sublayer));
  const GPoint content_offset = scroll_layer_get_content_offset(scroll_layer);
  const GSize content_size = scroll_layer_get_content_size(scroll_layer);
  const GSize frame_size = scroll_layer->layer.frame.size;
  GBitmap *shadow_top = shadow_get_top();
  GBitmap *shadow_bottom = shadow_get_bottom();

  graphics_context_set_compositing_mode(ctx, GCompOpClear);

  // Draw top shadow, if (partially) visible:
  const int16_t layer_w = shadow_sublayer->bounds.size.w;
  const int16_t layer_h = shadow_sublayer->bounds.size.h;
  const int16_t shadow_top_bitmap_h = shadow_top->bounds.size.h;
  const int16_t shadow_top_y_offset = -shadow_top_bitmap_h - CLIP(content_offset.y, -shadow_top_bitmap_h, 0);
  if (shadow_top_y_offset >= -shadow_top_bitmap_h) {
    graphics_draw_bitmap_in_rect(ctx, shadow_top, &GRect(0, shadow_top_y_offset,
                                                         layer_w, shadow_top_bitmap_h));
  }
  // Draw bottom shadow, if (partially) visible:
  const int16_t shadow_bottom_bitmap_h = shadow_top->bounds.size.h;
  const int16_t bottom_clipped_height = (content_size.h + content_offset.y) - frame_size.h;
  const int16_t shadow_bottom_y_offset = - CLIP(bottom_clipped_height, 0, shadow_bottom_bitmap_h);
  if (shadow_bottom_y_offset < 0) {
    graphics_draw_bitmap_in_rect(ctx, shadow_bottom, &GRect(0, layer_h + shadow_bottom_y_offset,
                                                            layer_w, shadow_bottom_bitmap_h));
  }
}

static void prv_setup_shadow_layer(ScrollLayer *scroll_layer) {
  layer_init(&scroll_layer->shadow_sublayer, &scroll_layer->layer.bounds);
  layer_set_clips(&scroll_layer->shadow_sublayer, true);
  scroll_layer->shadow_sublayer.update_proc = scroll_layer_draw_shadow_sublayer;
  layer_add_child(&scroll_layer->layer, &scroll_layer->shadow_sublayer);
}

static void scroll_layer_property_changed_proc(Layer *layer) {
  ScrollLayer *scroll_layer = (ScrollLayer*)layer;
  const GRect internal_rect = (GRect) { GPointZero, scroll_layer->layer.frame.size };

  // If shadow_sublayer initialized (opposite of paging_enabled)
  if (!prv_scroll_layer_is_paging_enabled(scroll_layer)) {
    scroll_layer->shadow_sublayer.frame = internal_rect;
    scroll_layer->shadow_sublayer.bounds = internal_rect;
  }

  layer_set_frame(&scroll_layer->content_sublayer, &internal_rect);
}

#ifdef CONFIG_TOUCH
// ---------------------------------------------------------------------------------------------
// Tier-1 touch navigation
//
// A ScrollLayer registers itself as a Tier-1 touch widget in scroll_layer_init(). Like MenuLayer,
// the recognizers and gesture state are NOT owned per-widget: the unified widget (tap, pan, swipe)
// set, owned by TouchNavState, drives whichever migrated widget the finger lands on through a
// per-node apply vtable. A ScrollLayer supplies that vtable (s_scroll_touch_nav_ops) at
// registration; the apply functions below stay the public per-scroll gesture surface (and the
// unit-test entry points). A ScrollLayer is a pure scroll container (no selection model): a vertical
// pan drags the content 1:1 and a horizontal swipe navigates (right = BACK, left = SELECT); it has
// no tap action (tap op is NULL, so a tap on it is dropped, never a bridge SELECT). A ScrollLayer
// embedded in a composite that drives its own scrolling (e.g. MenuLayer) deregisters via
// scroll_layer_touch_nav_deregister() so the same layer never has two gesture drivers.

_Static_assert(sizeof(((ScrollLayer *)0)->touch_nav_node) == sizeof(TouchNavWidgetNode),
               "ScrollLayer touch_nav_node must match TouchNavWidgetNode layout");

static bool prv_is_app_task(void) {
  return pebble_task_get_current() == PebbleTask_App;
}

static TouchNavState *prv_task_touch_nav_state(void) {
  return prv_is_app_task() ? app_state_get_touch_nav_state() : modal_manager_get_touch_nav_state();
}

// Test seam (declared in scroll_layer_private.h under CONFIG_TOUCH). The unified widget set holds no
// per-task singleton to reset; the touch-nav state is owned by TouchNavState, so this is a no-op
// kept for source compatibility with tests that call it in their setup.
void scroll_layer_touch_nav_reset_all(void) {
}

bool scroll_layer_touch_is_gesture_target(const ScrollLayer *scroll_layer) {
  const TouchNavState *state = prv_task_touch_nav_state();
  return state && state->latched_target && state->latched_target->widget == scroll_layer;
}

// Coarse clamp: [min(frame_h - content_h, 0), 0]. With content shorter than the viewport the lower
// bound collapses to 0 (via the min()), so a short page cannot be dragged off its top.
static int16_t prv_scroll_touch_clamp_offset_y(ScrollLayer *scroll_layer, int16_t y) {
  const int16_t frame_h = scroll_layer->layer.frame.size.h;
  const int16_t content_h = scroll_layer_get_content_size(scroll_layer).h;
  const int16_t min_y = MIN((int16_t)(frame_h - content_h), (int16_t)0);
  return CLIP(y, min_y, (int16_t)0);
}

// ---------------------------------------------------------------------------------------------
// Gesture handlers (also the unit-test entry surface)

void scroll_layer_touch_handle_pan_update(ScrollLayer *scroll_layer, GPoint base,
                                          GPoint delta_since_start) {
  const int16_t new_y = prv_scroll_touch_clamp_offset_y(scroll_layer, base.y + delta_since_start.y);
  scroll_layer_set_content_offset(scroll_layer, GPoint(0, new_y), false);
}

static void prv_scroll_touch_fling_stopped(Animation *animation, bool finished, void *context) {
  (void)animation;
  (void)finished;
  scroll_layer_touch_fling_cleanup((ScrollLayer *)context);
}

void scroll_layer_touch_handle_snap(ScrollLayer *scroll_layer, GPoint base, GPoint final_delta,
                                    GPoint velocity) {
  const int16_t released_y = prv_scroll_touch_clamp_offset_y(scroll_layer,
                                                             base.y + final_delta.y);
  // Coast: project where the finger's velocity carries the content and decelerate there. The
  // coast starts from the current (last throttled) offset, so the unthrottled residual is
  // absorbed into the animation instead of jumping instantly at liftoff. Paging keeps its snap
  // semantics (a coast would break page alignment).
  const int32_t v = CLIP((int32_t)velocity.y, -TOUCH_FLING_MAX_VELOCITY_PX_S,
                         TOUCH_FLING_MAX_VELOCITY_PX_S);
  if (ABS(v) >= TOUCH_FLING_MIN_VELOCITY_PX_S && !scroll_layer_get_paging(scroll_layer)) {
    const int32_t projected_y = released_y + (v * TOUCH_FLING_TAU_MS) / 1000;
    const int16_t target_y = prv_scroll_touch_clamp_offset_y(
        scroll_layer, (int16_t)CLIP(projected_y, INT16_MIN, INT16_MAX));
    if (scroll_layer_touch_fling_start(scroll_layer, target_y, (int16_t)v,
                                       prv_scroll_touch_fling_stopped, scroll_layer)) {
      return;
    }
  }
  // Slow liftoff (or a coast too short to schedule): settle the final offset, as before.
  scroll_layer_set_content_offset(scroll_layer, GPoint(0, released_y), false);
}

void scroll_layer_touch_handle_swipe(ScrollLayer *scroll_layer, SwipeDirection direction) {
  (void)scroll_layer;
  // Horizontal swipe navigation, mirroring the Tier-2 bridge and MenuLayer: right = BACK (pop the
  // top window when it has no back handler), left = SELECT. Guarded against a mid-transition drop.
  const TouchNavState *state = prv_task_touch_nav_state();
  if (!state || !state->ops) {
    return;
  }
  const TouchNavOps *ops = state->ops;
  if (ops->is_animating && ops->is_animating(ops->ctx)) {
    return;
  }
  switch (direction) {
    case SwipeDirection_Right:
      if (!(ops->top_overrides_back && ops->top_overrides_back(ops->ctx))) {
        if (ops->pop_top) {
          ops->pop_top(ops->ctx);
        }
      } else if (ops->emit_button) {
        ops->emit_button(ops->ctx, BUTTON_ID_BACK);
      }
      break;
    case SwipeDirection_Left:
      if (ops->emit_button) {
        ops->emit_button(ops->ctx, BUTTON_ID_SELECT);
      }
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------------------------
// Unified widget-set ops

// Thin void*->ScrollLayer* wrappers over the public apply functions. The unified widget set (owned
// by TouchNavState) drives the scroll layer through these; direct assignment of the apply functions
// would not compile because their first parameter is ScrollLayer*, not void*.

void scroll_layer_touch_handle_touchdown(ScrollLayer *scroll_layer) {
  // Finger down: stop any running scroll animation immediately so a coasting fling (or a settle)
  // is caught at touchdown, not at the pan threshold.
  Animation *anim = property_animation_get_animation(scroll_layer->animation);
  if (anim && animation_is_scheduled(anim)) {
    animation_unschedule(anim);
  }
  // A fling caught before its first frame never ran its stopped handler; restore explicitly.
  scroll_layer_touch_fling_cleanup(scroll_layer);
}

static void prv_scroll_ops_touchdown(void *w) {
  scroll_layer_touch_handle_touchdown((ScrollLayer *)w);
}

static void prv_scroll_ops_pan_started(void *w) {
  // The touchdown op already stopped any running animation; this covers an animation that started
  // between the Touchdown and the pan threshold (e.g. a button-driven scroll) so the finger takes
  // over cleanly. The base offset is latched by the core via get_base_offset next.
  scroll_layer_touch_handle_touchdown((ScrollLayer *)w);
}

static GPointReturn prv_scroll_ops_get_base_offset(void *w) {
  return scroll_layer_get_content_offset((ScrollLayer *)w);
}

static void prv_scroll_ops_pan_update(void *w, GPoint base, GPoint delta) {
  scroll_layer_touch_handle_pan_update((ScrollLayer *)w, base, delta);
}

static void prv_scroll_ops_pan_snap(void *w, GPoint base, GPoint final_delta, GPoint velocity) {
  scroll_layer_touch_handle_snap((ScrollLayer *)w, base, final_delta, velocity);
}

static void prv_scroll_ops_pan_cancel(void *w) {
  // The content is already settled from the last Updated; a cancelled pan has nothing to undo.
  (void)w;
}

static void prv_scroll_ops_swipe(void *w, SwipeDirection dir) {
  scroll_layer_touch_handle_swipe((ScrollLayer *)w, dir);
}

// can_start is NULL: a ScrollLayer's content is always valid, so a pan may always start. tap is
// NULL: a ScrollLayer has no tap action, so a tap on it is dropped (never a bridge SELECT).
static const TouchNavWidgetOps s_scroll_touch_nav_ops = {
  .can_start = NULL,
  .touchdown = prv_scroll_ops_touchdown,
  .pan_started = prv_scroll_ops_pan_started,
  .get_base_offset = prv_scroll_ops_get_base_offset,
  .pan_update = prv_scroll_ops_pan_update,
  .pan_snap = prv_scroll_ops_pan_snap,
  .pan_cancel = prv_scroll_ops_pan_cancel,
  .tap = NULL,
  .swipe = prv_scroll_ops_swipe,
};

static void prv_scroll_touch_nav_register(ScrollLayer *scroll_layer) {
  TouchNavState *state = prv_task_touch_nav_state();
  if (!state || !state->manager) {
    return;
  }

  // Register the scroll layer as a migrated Tier-1 widget: the unified widget set drives it through
  // s_scroll_touch_nav_ops. The registry add dedups by address and re-applies the ops/layer, so a
  // repeated init on the same scroll layer (no intervening deinit) keeps routing to and driving it.
  touch_nav_registry_add(state, TouchNavWidgetType_Scroll,
                         (TouchNavWidgetNode *)&scroll_layer->touch_nav_node, &scroll_layer->layer,
                         &s_scroll_touch_nav_ops, scroll_layer);
}

void scroll_layer_touch_nav_deregister(ScrollLayer *scroll_layer) {
  TouchNavState *state = prv_task_touch_nav_state();
  if (!state) {
    return;
  }
  // If this scroll layer is the live gesture target and is about to go away under a live window,
  // cancel the gesture with NO client callbacks so a settle cannot reach freed state.
  // touch_nav_registry_remove clears the latched target BEFORE we cancel (its UAF hook), so the
  // resulting Cancelled dispatch finds no target and re-enters nothing.
  const bool was_target = scroll_layer_touch_is_gesture_target(scroll_layer);
  // Idempotent: removing a node that is not in the registry (double deinit, or a never-registered
  // layer) is a safe no-op.
  touch_nav_registry_remove(state, TouchNavWidgetType_Scroll,
                            (TouchNavWidgetNode *)&scroll_layer->touch_nav_node);
  if (was_target && state->manager) {
    recognizer_manager_cancel_and_reset(state->manager);
  }
}
#endif  // CONFIG_TOUCH

void scroll_layer_init(ScrollLayer *scroll_layer, const GRect *frame) {
  *scroll_layer = (ScrollLayer){};

  layer_init(&scroll_layer->layer, frame);
  const GRect *bounds = &scroll_layer->layer.bounds;
  scroll_layer->layer.property_changed_proc = scroll_layer_property_changed_proc;

  layer_init(&scroll_layer->content_sublayer, bounds);
  layer_add_child(&scroll_layer->layer, &scroll_layer->content_sublayer);

  prv_setup_shadow_layer(scroll_layer);

#ifdef CONFIG_TOUCH
  prv_scroll_touch_nav_register(scroll_layer);
#endif
}

ScrollLayer* scroll_layer_create(GRect frame) {
  ScrollLayer *layer = applib_type_malloc(ScrollLayer);
  if (layer) {
    scroll_layer_init(layer, &frame);
  }
  return layer;
}

bool scroll_layer_is_instance(const Layer *layer) {
  return layer && layer->property_changed_proc == scroll_layer_property_changed_proc;
}

void scroll_layer_deinit(ScrollLayer *scroll_layer) {
#ifdef CONFIG_TOUCH
  // Deregister from the Tier-1 touch registry first so a gesture in flight on this widget is
  // cancelled with no client callbacks before its state is torn down (double-deinit safe).
  scroll_layer_touch_nav_deregister(scroll_layer);
#endif
  animation_destroy(property_animation_get_animation(scroll_layer->animation));
  content_indicator_destroy_for_scroll_layer(scroll_layer);
  layer_deinit(&scroll_layer->layer);
}

void scroll_layer_destroy(ScrollLayer *scroll_layer) {
  if (scroll_layer == NULL) {
    return;
  }
  scroll_layer_deinit(scroll_layer);
  applib_free(scroll_layer);
}

Layer* scroll_layer_get_layer(const ScrollLayer *scroll_layer) {
  return &((ScrollLayer *)scroll_layer)->layer;
}

void scroll_layer_set_frame(ScrollLayer *scroll_layer, GRect rect) {
  scroll_layer->layer.frame = rect;
  layer_mark_dirty(&scroll_layer->layer);
}

void scroll_layer_add_child(ScrollLayer *scroll_layer, Layer *child) {
  layer_add_child(&scroll_layer->content_sublayer, child);
}


GPoint scroll_layer_get_content_offset(ScrollLayer *scroll_layer) {
  return scroll_layer->content_sublayer.bounds.origin;
}

T_STATIC void prv_scroll_layer_set_content_offset_internal(
    ScrollLayer *scroll_layer, GPoint offset) {
  const GSize frame_size = scroll_layer->layer.frame.size;
  GRect bounds = scroll_layer->content_sublayer.bounds;
  const GPoint old_offset = bounds.origin;
  const int16_t min_x_offset = frame_size.w - bounds.size.w;
  int16_t min_y_offset = frame_size.h - bounds.size.h;

  if (prv_scroll_layer_is_paging_enabled(scroll_layer)) {
    uint16_t page_height = prv_scroll_layer_get_paging_height(scroll_layer);
    if (page_height) {
      // showing full page-aligned contents of last page
      min_y_offset = ROUND_TO_MOD_CEIL(min_y_offset, page_height);
    }
  }

  if (scroll_layer_get_clips_content_offset(scroll_layer)) {
    bounds.origin.x = CLIP(offset.x, MIN(min_x_offset, 0), 0);
    bounds.origin.y = CLIP(offset.y, MIN(min_y_offset, 0), 0);
  } else {
    bounds.origin = offset;
  }

  if (gpoint_equal(&old_offset, &bounds.origin)) {
    // Not changed.
    // still, call update_content indicator to refresh potential timers
    scroll_layer_update_content_indicator(scroll_layer);

    return;
  }

  layer_set_bounds(&scroll_layer->content_sublayer, &bounds);
  scroll_layer_update_content_indicator(scroll_layer);

  if (scroll_layer->callbacks.content_offset_changed_handler) {
    scroll_layer->callbacks.content_offset_changed_handler(scroll_layer, get_callback_context(scroll_layer));
  }
}

static const PropertyAnimationImplementation s_content_offset_animation_impl = {
  .base = {
    .update = (AnimationUpdateImplementation) property_animation_update_gpoint,
  },
  .accessors = {
    .setter = { .grect = (const GRectSetter) (void *) prv_scroll_layer_set_content_offset_internal, },
    .getter = { .grect = (const GRectGetter) (void *) scroll_layer_get_content_offset, },
  },
};

void scroll_layer_set_content_offset(ScrollLayer *scroll_layer, GPoint offset, bool animated) {
  // Note: animation_is_scheduled() returns false and property_animation_destroy does nothing
  // if the argument is NULL
  Animation *animation = property_animation_get_animation(scroll_layer->animation);
  bool was_running = false;
  if (animation) {
    was_running = animation_is_scheduled(animation);
    if (was_running) {
      animation_unschedule(animation);
    }
  }
  if (animated) {
    if (animation) {
      property_animation_init(scroll_layer->animation, &s_content_offset_animation_impl,
                              scroll_layer, NULL, &offset);
      if (was_running && !scroll_layer_get_paging(scroll_layer)) {
        animation_set_curve(animation, AnimationCurveEaseOut);
      }
    } else {
      scroll_layer->animation = property_animation_create(&s_content_offset_animation_impl,
                                                          scroll_layer, NULL, &offset);
      animation = property_animation_get_animation(scroll_layer->animation);
      if (scroll_layer_get_paging(scroll_layer)) {
        animation_set_custom_interpolation(animation, interpolate_moook);
        animation_set_duration(animation, interpolate_moook_duration());
      }
      animation_set_auto_destroy(animation, false);
    }
    animation_schedule(animation);
  } else {
    prv_scroll_layer_set_content_offset_internal(scroll_layer, offset);
  }
}

#ifdef CONFIG_TOUCH
// Cubic ease-out in AnimationProgress space: f(t) = 1 - (1 - t)^3. Compared to the stock
// quadratic EaseOut it brakes harder early and glides out longer, so a fast coast tapers off
// instead of stopping dead.
static AnimationProgress prv_fling_ease_out_cubic(AnimationProgress progress) {
  const int64_t remaining = ANIMATION_NORMALIZED_MAX - progress;
  return (AnimationProgress)(ANIMATION_NORMALIZED_MAX -
      (remaining * remaining * remaining) /
          ((int64_t)ANIMATION_NORMALIZED_MAX * ANIMATION_NORMALIZED_MAX));
}

bool scroll_layer_touch_fling_start(ScrollLayer *scroll_layer, int16_t target_y,
                                    int16_t velocity_y, AnimationStoppedHandler stopped,
                                    void *stopped_context) {
  const int16_t current_y = scroll_layer_get_content_offset(scroll_layer).y;
  const int32_t distance = (int32_t)target_y - current_y;
  if (ABS(distance) < TOUCH_FLING_MIN_DISTANCE_PX || velocity_y == 0) {
    return false;
  }
  const GPoint target = GPoint(0, target_y);
  Animation *animation = property_animation_get_animation(scroll_layer->animation);
  if (animation) {
    if (animation_is_scheduled(animation)) {
      animation_unschedule(animation);  // fires a previous coast's stopped handler, if any
    }
    property_animation_init(scroll_layer->animation, &s_content_offset_animation_impl,
                            scroll_layer, NULL, (void *)&target);
  } else {
    scroll_layer->animation = property_animation_create(&s_content_offset_animation_impl,
                                                        scroll_layer, NULL, (void *)&target);
    if (!scroll_layer->animation) {
      return false;
    }
    animation = property_animation_get_animation(scroll_layer->animation);
    animation_set_auto_destroy(animation, false);
  }
  // The cubic ease-out launches at slope 3 (f(t) = 1 - (1 - t)^3), so T = 3000 * |d| / |v| makes
  // the animated launch velocity equal the finger's liftoff velocity, with a long soft tail. An
  // edge-truncated distance shortens T proportionally, preserving the launch speed instead of
  // crawling into the clamp.
  const int32_t duration_ms = CLIP((3000 * ABS(distance)) / ABS((int32_t)velocity_y),
                                   (int32_t)TOUCH_FLING_MIN_DURATION_MS,
                                   (int32_t)TOUCH_FLING_MAX_DURATION_MS);
  animation_set_duration(animation, (uint32_t)duration_ms);
  animation_set_custom_curve(animation, prv_fling_ease_out_cubic);
  animation_set_handlers(animation, (AnimationHandlers) { .stopped = stopped }, stopped_context);
  animation_schedule(animation);
  return true;
}

void scroll_layer_touch_fling_cleanup(ScrollLayer *scroll_layer) {
  Animation *animation = property_animation_get_animation(scroll_layer->animation);
  if (!animation || animation_is_scheduled(animation)) {
    return;
  }
  animation_set_duration(animation, ANIMATION_DEFAULT_DURATION_MS);
  animation_set_curve(animation, AnimationCurveDefault);
  animation_set_handlers(animation, (AnimationHandlers) { 0 }, NULL);
}
#endif

void scroll_layer_set_content_size(ScrollLayer *scroll_layer, GSize size) {
  GRect bounds = scroll_layer->content_sublayer.bounds;
  bounds.size = size;
  layer_set_bounds(&scroll_layer->content_sublayer, &bounds);
  // Ensure our content offset is clipped to the new size.
  // We call prv_scroll_layer_set_content_offset_internal() directly and
  // keep potential animations running – since some 3rd-party apps do change the content size
  // frequently (e.g. in an update_proc) and would otherwise implicitly stop scroll animations.
  // It's fine to keep scroll animations running as they clip the offset to valid bounds.
  prv_scroll_layer_set_content_offset_internal(
      scroll_layer, scroll_layer_get_content_offset(scroll_layer));
}

GSize scroll_layer_get_content_size(const ScrollLayer *scroll_layer) {
  return scroll_layer->content_sublayer.bounds.size;
}

void scroll_layer_scroll(ScrollLayer *scroll_layer, ScrollDirection direction, bool animated) {
  GPoint offset = scroll_layer_get_content_offset(scroll_layer);
  int32_t scroll_height = 32;

  // If process is 3.x and has enabled paging
  if (prv_scroll_layer_is_paging_enabled(scroll_layer)) {
    uint16_t page_height = prv_scroll_layer_get_paging_height(scroll_layer);
    if (page_height) {
      // Force offset to start (and stay) page aligned
      offset.y = ROUND_TO_MOD_CEIL(offset.y, page_height);
      scroll_height = page_height;
    }
  }

  switch (direction) {
    case ScrollDirectionUp:
      offset.y += scroll_height;
      break;
    case ScrollDirectionDown:
      offset.y -= scroll_height;
      break;

    default: return;
  }
  scroll_layer_set_content_offset(scroll_layer, offset, animated);
}

void scroll_layer_scroll_up_click_handler(ClickRecognizerRef recognizer, void *context) {
  ScrollLayer *scroll_layer = (ScrollLayer *)context;
  scroll_layer_scroll(scroll_layer, ScrollDirectionUp, true);
  (void)recognizer;
}

void scroll_layer_scroll_down_click_handler(ClickRecognizerRef recognizer, void *context) {
  ScrollLayer *scroll_layer = (ScrollLayer *)context;
  scroll_layer_scroll(scroll_layer, ScrollDirectionDown, true);
  (void)recognizer;
}

static void scroll_layer_click_config_provider(ScrollLayer *scroll_layer) {
  // Config UP / DOWN button behavior:
  window_single_repeating_click_subscribe(BUTTON_ID_UP, 100, scroll_layer_scroll_up_click_handler);
  window_single_repeating_click_subscribe(BUTTON_ID_DOWN, 100, scroll_layer_scroll_down_click_handler);
  // Set the context for the SELECT button:
  window_set_click_context(BUTTON_ID_SELECT, get_callback_context(scroll_layer));

  // Callback to provide the client to setup the SELECT button:
  if (scroll_layer->callbacks.click_config_provider) {
    scroll_layer->callbacks.click_config_provider(get_callback_context(scroll_layer));
  }
}

void scroll_layer_set_click_config_onto_window(ScrollLayer *scroll_layer, struct Window *window) {
  window_set_click_config_provider_with_context(window, (ClickConfigProvider) scroll_layer_click_config_provider, scroll_layer);
}

void scroll_layer_set_callbacks(ScrollLayer *scroll_layer, ScrollLayerCallbacks callbacks) {
  scroll_layer->callbacks = callbacks;
}

void scroll_layer_set_context(ScrollLayer *scroll_layer, void *context) {
  scroll_layer->context = context;
}

void scroll_layer_set_shadow_hidden(ScrollLayer *scroll_layer, bool hidden) {
  PBL_ASSERTN(scroll_layer);

  // paging and shadow_sublayer are mutually exclusive
  // so init shadow_sublayer if it was paging data
  if (prv_scroll_layer_is_paging_enabled(scroll_layer) && hidden == false) {
    prv_setup_shadow_layer(scroll_layer);
  }

  scroll_layer_property_changed_proc((Layer*)scroll_layer);
  layer_set_hidden(&scroll_layer->shadow_sublayer, hidden);
}

bool scroll_layer_get_shadow_hidden(const ScrollLayer *scroll_layer) {
  return layer_get_hidden(&scroll_layer->shadow_sublayer);
}

void scroll_layer_set_paging(ScrollLayer *scroll_layer, bool paging_enabled) {
  PBL_ASSERTN(scroll_layer);
  if (paging_enabled) {
    // Deinit shadow_sublayer to enable paging
    if (!prv_scroll_layer_is_paging_enabled(scroll_layer)) {
      layer_deinit(&scroll_layer->shadow_sublayer);
    }

    // paging and shadow_sublayer are mutually exclusive
    scroll_layer->paging.shadow_hidden = true;
    scroll_layer->paging.paging_disabled = false;
  } else {
    if (prv_scroll_layer_is_paging_enabled(scroll_layer)) {
      prv_setup_shadow_layer(scroll_layer);
      // still require explicit un-hiding of shadow
      scroll_layer_set_shadow_hidden(scroll_layer, true);
    }
  }
}

bool scroll_layer_get_paging(ScrollLayer* scroll_layer) {
  return scroll_layer && prv_scroll_layer_is_paging_enabled(scroll_layer);
}

ContentIndicator *scroll_layer_get_content_indicator(ScrollLayer *scroll_layer) {
  return content_indicator_get_or_create_for_scroll_layer(scroll_layer);
}

void scroll_layer_update_content_indicator(ScrollLayer *scroll_layer) {
  ContentIndicator *content_indicator = content_indicator_get_for_scroll_layer(scroll_layer);
  if (!content_indicator) {
    return;
  }

  const GSize scroll_layer_frame_size = scroll_layer_get_layer(scroll_layer)->frame.size;
  const GSize scroll_layer_content_size = scroll_layer_get_content_size(scroll_layer);
  const int16_t scroll_layer_content_offset_y = scroll_layer_get_content_offset(scroll_layer).y;

  const bool content_available_up = (scroll_layer_content_offset_y < 0);
  content_indicator_set_content_available(content_indicator,
                                          ContentIndicatorDirectionUp,
                                          content_available_up);
  const bool content_available_down =
    (scroll_layer_frame_size.h - scroll_layer_content_offset_y < scroll_layer_content_size.h);
  content_indicator_set_content_available(content_indicator,
                                          ContentIndicatorDirectionDown,
                                          content_available_down);
}

void scroll_layer_set_clips_content_offset(ScrollLayer *scroll_layer, bool clips) {
  scroll_layer->content_sublayer.clips = clips;
  scroll_layer_set_content_offset(scroll_layer,
                                  scroll_layer_get_content_offset(scroll_layer), false);
}

bool scroll_layer_get_clips_content_offset(ScrollLayer *scroll_layer) {
  return scroll_layer->content_sublayer.clips;
}
