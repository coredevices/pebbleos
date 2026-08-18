/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "menu_layer.h"

struct MenuIterator;

typedef void (*MenuIteratorCallback)(struct MenuIterator *it);

typedef struct MenuIterator {
  MenuLayer * menu_layer;
  MenuCellSpan cursor;
  int16_t cell_bottom_y;
  MenuIteratorCallback row_callback_before_geometry;
  MenuIteratorCallback row_callback_after_geometry;
  MenuIteratorCallback section_callback;
  bool should_continue; // callback can set this to false if the row-loop should be exited.
} MenuIterator;

typedef struct MenuRenderIterator {
  MenuIterator it;
  GContext* ctx;
  int16_t content_top_y;
  int16_t content_bottom_y;
  bool cache_set:1;
  bool cursor_in_frame:1;
  MenuCellSpan new_cache;
  Layer cell_layer;
} MenuRenderIterator;

#ifdef CONFIG_TOUCH
#include "applib/ui/recognizer/swipe.h"

//! @internal
//! Touch-navigation (Tier-1) gesture handlers, split out so they can be unit tested directly
//! without driving the full recognizer stack. All coordinates that flow in are already resolved to
//! the scroll layer's frame (screen) space; the handlers convert to content space internally using
//! the live content offset. See menu_layer.c for the wiring into the per-task recognizer set.

//! Locate the selectable row whose cell spans \a content_y (content-space y). Section headers and
//! separators are not selectable and are skipped. Walks downward then upward from the render anchor.
//! @return true and fills \a index_out if a row is hit; false if \a content_y lands on a header/gap.
bool menu_layer_touch_find_row_at_content_y(MenuLayer *menu_layer, int16_t content_y,
                                            MenuIndex *index_out);

//! Live scroll during a pan: move the content to \a base + \a delta_since_start, coarsely clamped.
//! On a plain menu the selection index is intentionally left unchanged. On a center-focused menu
//! the focus stays pinned at the viewport centre: the row crossing the centre becomes the
//! selection live, through the full selection_will_change contract (veto/redirect honoured).
void menu_layer_touch_handle_pan_update(MenuLayer *menu_layer, GPoint base, GPoint delta_since_start);

//! Liftoff. Applies the last (unthrottled) pan delta. A plain menu only settles the offset — the
//! selection never moves on a pan. A center-focused menu re-tracks the row under the centre and
//! then glides it to the exact centre (animated). \a velocity is the liftoff velocity in px/s.
void menu_layer_touch_handle_snap(MenuLayer *menu_layer, GPoint base, GPoint final_delta,
                                  GPoint velocity);

//! One-step tap activation through the same contract: map \a point_on_screen into the scroll
//! layer's frame, hit-test it, run selection_will_change, and activate (select_click) only if the
//! final selection is the tapped row. A veto (final == old) does nothing; a redirect to a third row
//! selects it without activating.
void menu_layer_touch_handle_tap(MenuLayer *menu_layer, GPoint point_on_screen);

//! Horizontal swipe: right activates the selected row, left emits BACK through the touch bridge.
void menu_layer_touch_handle_swipe(MenuLayer *menu_layer, SwipeDirection direction);

//! Cancelled gesture: a plain menu has nothing to settle; a center-focused menu re-centres its
//! selection synchronously (no animation) so the handover leaves a stable state.
void menu_layer_touch_handle_cancel(MenuLayer *menu_layer);

//! @internal Test seam: zero the per-task Tier-1 gesture singletons for cross-test isolation.
void menu_layer_touch_nav_reset_all(void);

//! @internal Test seam: whether \a menu_layer is the current per-task gesture target.
bool menu_layer_touch_is_gesture_target(const MenuLayer *menu_layer);
#endif
