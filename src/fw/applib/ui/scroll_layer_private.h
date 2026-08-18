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

//! Live scroll during a pan: move the content to \a base + \a delta_since_start on the y axis,
//! clamped to [min(frame_h - content_h, 0), 0].
void scroll_layer_touch_handle_pan_update(ScrollLayer *scroll_layer, GPoint base,
                                          GPoint delta_since_start);

//! Liftoff: settle the final (unthrottled) offset, same clamp as the live pan. \a velocity is the
//! liftoff velocity in px/s (see \ref pan_recognizer_get_velocity).
void scroll_layer_touch_handle_snap(ScrollLayer *scroll_layer, GPoint base, GPoint final_delta,
                                    GPoint velocity);

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
