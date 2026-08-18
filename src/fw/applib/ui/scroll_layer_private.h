/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "scroll_layer.h"

#ifdef CONFIG_TOUCH
#include "applib/ui/recognizer/swipe.h"

//! @internal
//! Touch-navigation (Tier-1) gesture handlers for a bare ScrollLayer, split out so they can be unit
//! tested directly without driving the full recognizer stack. A ScrollLayer is a pure scroll
//! container (no selection model), so a vertical pan drags the content 1:1 and a horizontal swipe
//! navigates (right = BACK, left = SELECT), mirroring MenuLayer's Tier-1 wiring. See scroll_layer.c
//! for the wiring into the per-task recognizer set.

//! Finger down: stop any running scroll animation (catch-to-stop for a coasting fling) and
//! restore the shared animation's fling defaults.
void scroll_layer_touch_handle_touchdown(ScrollLayer *scroll_layer);

//! Live scroll during a pan: move the content to \a base + \a delta_since_start on the y axis,
//! clamped to [min(frame_h - content_h, 0), 0].
void scroll_layer_touch_handle_pan_update(ScrollLayer *scroll_layer, GPoint base,
                                          GPoint delta_since_start);

//! Liftoff: settle the final (unthrottled) offset, same clamp as the live pan, then coast
//! (inertial fling) when \a velocity (px/s, see \ref pan_recognizer_get_velocity) is above the
//! fling threshold. Paging scroll layers never fling (a coast would break page alignment).
void scroll_layer_touch_handle_snap(ScrollLayer *scroll_layer, GPoint base, GPoint final_delta,
                                    GPoint velocity);

//! Inertial-fling tuning. All integer math; see scroll_layer_touch_fling_start(). A liftoff slower
//! than the MIN velocity settles like before; faster liftoffs coast toward
//! target = offset + velocity * TAU / 1000 with a cubic ease-out deceleration whose launch
//! velocity matches the finger (launch slope 3, so duration = 3000 * |distance| / |velocity|,
//! i.e. 3 * TAU unclamped). The cubic tail brakes hard early and then glides out, so a fast fling
//! ends softly instead of stopping dead.
#define TOUCH_FLING_MIN_VELOCITY_PX_S 250
#define TOUCH_FLING_MAX_VELOCITY_PX_S 3600
#define TOUCH_FLING_TAU_MS 240
#define TOUCH_FLING_MIN_DURATION_MS 100
#define TOUCH_FLING_MAX_DURATION_MS (3 * TOUCH_FLING_TAU_MS)
#define TOUCH_FLING_MIN_DISTANCE_PX 3

//! @internal
//! Start an inertial coast on the shared scroll animation toward \a target_y (already clamped by
//! the caller). \a velocity_y is the liftoff velocity in px/s (already range-checked by the
//! caller). \a stopped runs when the coast ends (finished or unscheduled) and MUST call
//! scroll_layer_touch_fling_cleanup() so the shared animation's defaults are restored.
//! @return false (and moves nothing) when the remaining distance is below the fling minimum.
bool scroll_layer_touch_fling_start(ScrollLayer *scroll_layer, int16_t target_y,
                                    int16_t velocity_y, AnimationStoppedHandler stopped,
                                    void *stopped_context);

//! @internal
//! Restore the shared scroll animation's defaults after a fling (duration, curve, handlers).
//! Idempotent; a no-op while the animation is scheduled. Callers that unschedule a fling that may
//! not have run its first frame yet (the stopped handler only fires for started animations) must
//! call this explicitly after unscheduling.
void scroll_layer_touch_fling_cleanup(ScrollLayer *scroll_layer);

//! Horizontal swipe navigation through the touch bridge: right emits BACK (pop when the top window
//! has no back handler), left emits SELECT. A no-op when the bridge is mid-transition.
void scroll_layer_touch_handle_swipe(ScrollLayer *scroll_layer, SwipeDirection direction);

//! @internal
//! Deregister a ScrollLayer from the Tier-1 Scroll registry without tearing down the rest of the
//! layer. Used by MenuLayer to drop its embedded scroll layer's bare-Scroll registration (the menu
//! drives scrolling through its own Menu registration), so the same layer never has two gesture
//! drivers. Idempotent: a layer that is not registered is a safe no-op.
void scroll_layer_touch_nav_deregister(ScrollLayer *scroll_layer);

//! @internal Test seam: zero the per-task Tier-1 gesture singletons for cross-test isolation.
void scroll_layer_touch_nav_reset_all(void);

//! @internal Test seam: whether \a scroll_layer is the current per-task gesture target.
bool scroll_layer_touch_is_gesture_target(const ScrollLayer *scroll_layer);
#endif
