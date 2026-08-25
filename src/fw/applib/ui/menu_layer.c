/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "menu_layer.h"
#include "menu_layer_private.h"

#include "applib/applib_malloc.auto.h"
#include "applib/preferred_content_size.h"
#include "applib/graphics/graphics.h"
#include "applib/graphics/text.h"
#include "pbl/util/trig.h"
#include "applib/fonts/fonts.h"
#include "applib/ui/animation_timing.h"
#include "applib/ui/click.h"
#include "applib/ui/window.h"
#include "applib/pbl_std/pbl_std.h"
#include "applib/legacy2/ui/menu_layer_legacy2.h"
#include "kernel/pbl_malloc.h"
#include "process_management/process_manager.h"
#include "shell/system_theme.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"
#include "vibes.h"

#include <string.h>

#ifdef CONFIG_TOUCH
#include "applib/ui/recognizer/touch_nav.h"
#include "applib/ui/recognizer/recognizer_manager.h"
#include "applib/ui/recognizer/pan.h"
#include "applib/ui/recognizer/swipe.h"
#include "applib/ui/recognizer/tap.h"
#include "applib/ui/scroll_layer_private.h"
#include "kernel/pebble_tasks.h"
#include "pbl/drivers/rtc.h"
#include "syscall/syscall.h"

// The per-task touch-nav state lives in the app state (app task) or the modal manager (KernelMain).
// Forward-declared here to avoid pulling those heavy kernel/app headers into this applib module.
struct TouchNavState *app_state_get_touch_nav_state(void);
struct TouchNavState *modal_manager_get_touch_nav_state(void);

static void prv_menu_touch_nav_register(MenuLayer *menu_layer);
static void prv_menu_touch_nav_deregister(MenuLayer *menu_layer);
static void prv_menu_touch_track_center_row(MenuLayer *menu_layer);
static void prv_menu_update_overscroll_stretch(MenuLayer *menu_layer);
#endif

//! @return True if there was an animation to cancel, false otherwise
static bool prv_cancel_selection_animation(MenuLayer *menu_layer);

//////////////////////
// Menu Layer
//
// NOTES: The MenuLayer is built on top of ScrollLayer. It uses ScrollLayer's scrolling and clipping features.
// Since it easily becomes to costly in terms of RAM to hold a layer for each row in the menu in memory,
// the MenuLayer does not use layers for its rows and headers. When a row is about to be displayed,
// it will call out to the client using a callback to get that row drawn.
// Inside the MenuLayer's update_proc (Layer drawing callback), it will call out to its client for each row
// that needs to be drawn, until all visible rows have been drawn.

//! How long the scrollbar overlay stays visible after the last touch scroll movement.
#define MENU_LAYER_SCROLLBAR_HIDE_TIMEOUT_MS 1000
#define MENU_LAYER_SCROLLBAR_WIDTH 3
#define MENU_LAYER_SCROLLBAR_MARGIN 2
#define MENU_LAYER_SCROLLBAR_MIN_THUMB_HEIGHT 8

static void prv_scrollbar_cancel_hide_timer(MenuLayer *menu_layer) {
  if (menu_layer->scrollbar_hide_timer) {
    app_timer_cancel(menu_layer->scrollbar_hide_timer);
    menu_layer->scrollbar_hide_timer = NULL;
  }
}

#ifdef CONFIG_TOUCH
static void prv_scrollbar_hide_timer_cb(void *data) {
  MenuLayer *menu_layer = data;
  menu_layer->scrollbar_hide_timer = NULL;
  menu_layer->scrollbar_visible = false;
  layer_mark_dirty(&menu_layer->scroll_layer.layer);
}

//! Shows the scrollbar overlay and (re)arms the timer that hides it again. Called only from the
//! touch scrolling paths: button scrolling steps the selection and doesn't need the position cue.
static void prv_scrollbar_kick(MenuLayer *menu_layer) {
  if (menu_layer->scrollbar_hidden || process_manager_compiled_with_legacy2_sdk()) {
    return;
  }
  const int16_t frame_h = menu_layer->scroll_layer.layer.frame.size.h;
  const int16_t content_h = scroll_layer_get_content_size(&menu_layer->scroll_layer).h;
  if (content_h <= frame_h) {
    return;
  }
  menu_layer->scrollbar_visible = true;
  if (!menu_layer->scrollbar_hide_timer ||
      !app_timer_reschedule(menu_layer->scrollbar_hide_timer,
                            MENU_LAYER_SCROLLBAR_HIDE_TIMEOUT_MS)) {
    menu_layer->scrollbar_hide_timer = app_timer_register(
        MENU_LAYER_SCROLLBAR_HIDE_TIMEOUT_MS, prv_scrollbar_hide_timer_cb, menu_layer);
  }
}
#endif  // CONFIG_TOUCH

#if PBL_ROUND
//! Curved scrollbar (Wear OS style): the track is an arc hugging the bezel, spanning
//! ±30° around the 3 o'clock position.
#define MENU_LAYER_SCROLLBAR_TRACK_SWEEP DEG_TO_TRIGANGLE(60)
#define MENU_LAYER_SCROLLBAR_MIN_THUMB_SWEEP DEG_TO_TRIGANGLE(10)

//! Thumb start/end angles for the curved scrollbar; false if the content doesn't scroll.
T_STATIC bool prv_scrollbar_thumb_angles(MenuLayer *menu_layer, int16_t content_top_y,
                                         int32_t *angle_start, int32_t *angle_end) {
  const GSize frame_size = menu_layer->scroll_layer.layer.frame.size;
  const int16_t content_h = scroll_layer_get_content_size(&menu_layer->scroll_layer).h;
  const int16_t scrollable_h = content_h - frame_size.h;
  if (scrollable_h <= 0) {
    return false;
  }
  const int32_t track_sweep = MENU_LAYER_SCROLLBAR_TRACK_SWEEP;
  const int32_t thumb_sweep = CLIP((track_sweep * frame_size.h) / content_h,
                                   MENU_LAYER_SCROLLBAR_MIN_THUMB_SWEEP, track_sweep);
  // Center-focused menus over-scroll past the ends (offset clipping disabled); clamp to the track.
  const int32_t progress_y = CLIP(content_top_y, 0, scrollable_h);
  const int32_t track_start = DEG_TO_TRIGANGLE(90) - (track_sweep / 2);
  *angle_start = track_start + (((track_sweep - thumb_sweep) * progress_y) / scrollable_h);
  *angle_end = *angle_start + thumb_sweep;
  return true;
}

//! Fills an arc band with rounded end caps (graphics_fill_radial ends are cut flat).
static void prv_scrollbar_fill_capped_arc(GContext *ctx, GPoint center, uint16_t radius_outer,
                                          uint16_t thickness, int32_t angle_start,
                                          int32_t angle_end) {
  graphics_fill_radial_internal(ctx, center, radius_outer - thickness, radius_outer,
                                angle_start, angle_end);
  const int32_t radius_mid = radius_outer - (thickness / 2);
  const int32_t angles[2] = { angle_start, angle_end };
  for (size_t i = 0; i < ARRAY_LENGTH(angles); i++) {
    const GPoint cap = GPoint(
        center.x + (int16_t)((radius_mid * sin_lookup(angles[i])) / TRIG_MAX_RATIO),
        center.y - (int16_t)((radius_mid * cos_lookup(angles[i])) / TRIG_MAX_RATIO));
    graphics_fill_circle(ctx, cap, thickness / 2);
  }
}

static void prv_scrollbar_draw(MenuLayer *menu_layer, GContext *ctx, int16_t content_top_y) {
  int32_t angle_start, angle_end;
  if (!prv_scrollbar_thumb_angles(menu_layer, content_top_y, &angle_start, &angle_end)) {
    return;
  }
  const GSize frame_size = menu_layer->scroll_layer.layer.frame.size;
  // Drawing happens in content space: the viewport center is offset by content_top_y.
  const GPoint center = GPoint(frame_size.w / 2, content_top_y + (frame_size.h / 2));
  const uint16_t radius_outer =
      (MIN(frame_size.w, frame_size.h) / 2) - MENU_LAYER_SCROLLBAR_MARGIN;
  // 1px halo around the thumb (rounded caps included) so it stays visible over both normal and
  // highlighted cells
  graphics_context_set_fill_color(ctx, menu_layer->normal_colors[MenuLayerColorBackground]);
  prv_scrollbar_fill_capped_arc(ctx, center, radius_outer + 1, MENU_LAYER_SCROLLBAR_WIDTH + 2,
                                angle_start, angle_end);
  graphics_context_set_fill_color(ctx, menu_layer->normal_colors[MenuLayerColorForeground]);
  prv_scrollbar_fill_capped_arc(ctx, center, radius_outer, MENU_LAYER_SCROLLBAR_WIDTH,
                                angle_start, angle_end);
}
#else
//! Scrollbar thumb rect in content-space coordinates (the space menu_layer_update_proc draws in).
T_STATIC GRect prv_scrollbar_thumb_rect(MenuLayer *menu_layer, int16_t content_top_y) {
  const GSize frame_size = menu_layer->scroll_layer.layer.frame.size;
  const int16_t content_h = scroll_layer_get_content_size(&menu_layer->scroll_layer).h;
  const int16_t scrollable_h = content_h - frame_size.h;
  const int16_t track_h = frame_size.h - (2 * MENU_LAYER_SCROLLBAR_MARGIN);
  if (scrollable_h <= 0 || track_h <= MENU_LAYER_SCROLLBAR_MIN_THUMB_HEIGHT) {
    return GRectZero;
  }
  const int16_t thumb_h = CLIP((int16_t)(((int32_t)track_h * frame_size.h) / content_h),
                               MENU_LAYER_SCROLLBAR_MIN_THUMB_HEIGHT, track_h);
  // Center-focused menus over-scroll past the ends (offset clipping disabled); clamp to the track.
  const int16_t progress_y = CLIP(content_top_y, 0, scrollable_h);
  const int16_t thumb_y = MENU_LAYER_SCROLLBAR_MARGIN +
      (((int32_t)(track_h - thumb_h) * progress_y) / scrollable_h);
  return GRect(frame_size.w - MENU_LAYER_SCROLLBAR_MARGIN - MENU_LAYER_SCROLLBAR_WIDTH,
               content_top_y + thumb_y, MENU_LAYER_SCROLLBAR_WIDTH, thumb_h);
}

static void prv_scrollbar_draw(MenuLayer *menu_layer, GContext *ctx, int16_t content_top_y) {
  const GRect thumb = prv_scrollbar_thumb_rect(menu_layer, content_top_y);
  if (grect_is_empty(&thumb)) {
    return;
  }
  // 1px halo around the thumb so it stays visible over both normal and highlighted cells
  const GRect halo = grect_inset_internal(thumb, -1, -1);
  graphics_context_set_fill_color(ctx, menu_layer->normal_colors[MenuLayerColorBackground]);
  graphics_fill_round_rect(ctx, &halo, 2, GCornersAll);
  graphics_context_set_fill_color(ctx, menu_layer->normal_colors[MenuLayerColorForeground]);
  graphics_fill_round_rect(ctx, &thumb, 1, GCornersAll);
}
#endif  // PBL_ROUND

static void prv_menu_scroll_offset_changed_handler(ScrollLayer *scroll_layer,
                                                   MenuLayer *menu_layer) {
#ifdef CONFIG_TOUCH
  prv_menu_update_overscroll_stretch(menu_layer);
  if (menu_layer->touch_fling_active) {
    // Keep the scrollbar alive while an inertial coast drives the offset.
    prv_scrollbar_kick(menu_layer);
    // During an inertial coast on a carousel the row crossing the centre becomes the selection
    // live, one row at a time, exactly as during a finger pan (same selection_will_change
    // contract).
    if (menu_layer->center_focused) {
      prv_menu_touch_track_center_row(menu_layer);
    }
  }
#endif
}

static void prv_menu_select_click_handler(ClickRecognizerRef recognizer, MenuLayer *menu_layer) {
  // If the selection animation is running, complete it. Note that 2.x apps don't have a selection
  // animation.
  if (menu_layer->animation.animation) {
    animation_set_elapsed(menu_layer->animation.animation,
                          animation_get_duration(menu_layer->animation.animation, true, true));
  }

  // If we're in the middle of scrolling, finish scrolling immediately before handling the select
  // click. We do this to make a transition animation have a consistent position to animate from.
  // Note that animation_set_elapsed isn't supported on 2.x animations. Just skip this step, as
  // no 2.x transitions interact directly with menu layer state.
  if (!process_manager_compiled_with_legacy2_sdk() && menu_layer->scroll_layer.animation) {
    Animation *scroll_layer_animation =
        property_animation_get_animation(menu_layer->scroll_layer.animation);
    animation_set_elapsed(scroll_layer_animation,
                          animation_get_duration(scroll_layer_animation, true, true));
  }

  // Actually handle the click
  if (menu_layer->callbacks.select_click) {
    menu_layer->callbacks.select_click(menu_layer, &menu_layer->selection.index,
                                       menu_layer->callback_context);
  }
}

static void prv_menu_select_long_click_handler(ClickRecognizerRef recognizer,
    MenuLayer *menu_layer) {
  if (menu_layer->callbacks.select_long_click) {
    menu_layer->callbacks.select_long_click(menu_layer, &menu_layer->selection.index,
        menu_layer->callback_context);
  }
}

static inline uint16_t prv_menu_layer_get_num_sections(MenuLayer *menu_layer);
static inline uint16_t prv_menu_layer_get_num_rows(MenuLayer *menu_layer, uint16_t section_index);

static bool prv_menu_index_is_first_index(MenuLayer *menu_layer, const MenuIndex *index) {
  (void)menu_layer;

  MenuIndex first_index = MenuIndex(0, 0);
  return menu_index_compare(index, &first_index) == 0;
}

static bool prv_menu_index_is_last_index(MenuLayer *menu_layer, const MenuIndex *index) {
  int last_index_section = prv_menu_layer_get_num_sections(menu_layer) - 1;
  int last_index_row = prv_menu_layer_get_num_rows(menu_layer, last_index_section) - 1;
  MenuIndex last_index = MenuIndex(last_index_section, last_index_row);
  return menu_index_compare(index, &last_index) == 0;
}

static void prv_vibe_pulse(void) {
  uint32_t const segments[] = { 50 };
  VibePattern pat = {
    .durations = segments,
    .num_segments = ARRAY_LENGTH(segments),
  };
  vibes_enqueue_custom_pattern(pat);
}

//! Handle the menu scroll wrap around
//! @param menu_layer reference to the current MenuLayer
//! @param recognizer reference to the ClickRecognizer struct
//! @param scrolling_up `true` if scrolling up, `false` if scrolling down
//! @return `true` if a wrap around has been applied
static bool prv_menu_scroll_handle_wrap_around(MenuLayer *menu_layer, ClickRecognizerRef recognizer, bool scrolling_up) {
  const uint8_t current_scroll_action = scrolling_up ? MenuLayerRepeatScrollingUp : MenuLayerRepeatScrollingDown;
  const bool is_repeating = click_recognizer_is_repeating(recognizer);

  if (is_repeating) {
    menu_layer->cache.button_repeat_scrolling = current_scroll_action;
    if (!menu_layer->scroll_force_wrap_on_repeat) {
      return false;
    }
  }
  menu_layer->cache.button_repeat_scrolling = MenuLayerNoRepeatScrolling;

  MenuIndex current_index = menu_layer->selection.index;
  int last_index_section = prv_menu_layer_get_num_sections(menu_layer) - 1;
  int last_index_row = prv_menu_layer_get_num_rows(menu_layer, last_index_section) - 1;
  MenuIndex first_index = MenuIndex(0, 0);
  MenuIndex last_index = MenuIndex(last_index_section, last_index_row);
  MenuIndex *wraparound_dest_index;
  if ((menu_index_compare(&current_index, &first_index) == 0) && scrolling_up) {
    wraparound_dest_index = &last_index;
  } else if ((menu_index_compare(&current_index, &last_index) == 0) && !scrolling_up) {
    wraparound_dest_index = &first_index;
  } else {
    return false;
  }

  // Honor selection_will_change, like normal scrolling does, so the wrap
  // destination can be redirected away from non-selectable rows.
  MenuLayerSelectionWillChangeCallback will_change_cb =
      menu_layer->callbacks.selection_will_change;
  if (will_change_cb) {
    MenuIndex new_index = *wraparound_dest_index;
    will_change_cb(menu_layer, &new_index, current_index, menu_layer->callback_context);
    if (menu_index_compare(&new_index, &current_index) == 0) {
      // Callback locked the selection in place; don't wrap.
      return false;
    }
    *wraparound_dest_index = new_index;
  }

  const bool animated = true;
  menu_layer_set_selected_index(menu_layer, *wraparound_dest_index, MenuRowAlignCenter, animated);
  if (menu_layer->scroll_vibe_on_wrap_around) {
    prv_vibe_pulse();
  }
  return true;
}

#ifdef CONFIG_TOUCH
// After a free pixel scroll (touch), the logical selection can drift off-screen while the visible
// content is elsewhere. Stepping UP/DOWN from that off-screen selection teleports it across the list.
// If the selection is not visible in the viewport, reselect the row nearest the viewport centre first
// (no activation, MenuRowAlignNone leaves a free-scrolled list where it is; center_focused menus keep
// their centre invariant regardless); the caller then steps from there.
// When the selection is already visible, this is a no-op so on-screen menus behave exactly as before.
static void prv_menu_reconcile_selection_before_step(MenuLayer *menu_layer) {
  // Only a free pixel scroll (pan) leaves the selection off-screen with a SETTLED offset. A button or
  // programmatic step animates the SCROLL offset toward the selection; while that scroll animation is
  // in flight the offset legitimately lags the selection, and pulling to centre would fight the
  // settling step. Gate on the scroll animation specifically, not the highlight animation: a pan
  // leaves the scroll offset settled even while a just-tapped row's highlight animation is still
  // running, and that case must still reconcile. animation_is_scheduled() is NULL-safe (false).
  Animation *scroll_animation = (Animation *) menu_layer->scroll_layer.animation;
  if (animation_is_scheduled(scroll_animation)) {
    return;
  }
  const int16_t offset_y = scroll_layer_get_content_offset(&menu_layer->scroll_layer).y;
  const int16_t frame_h = menu_layer->scroll_layer.layer.frame.size.h;

  // A content point content_y appears at frame_y = content_y + offset_y; the selected row is visible
  // iff its frame-y range overlaps [0, frame_h].
  const bool selection_visible =
      (menu_layer->selection.y + offset_y + menu_layer->selection.h > 0) &&
      (menu_layer->selection.y + offset_y < frame_h);
  if (selection_visible) {
    return;
  }

  MenuIndex center_idx;
  if (!menu_layer_touch_find_row_at_content_y(menu_layer, frame_h / 2 - offset_y, &center_idx)) {
    // Viewport centre landed on a header/gap: do not reselect, just let the caller step as today.
    return;
  }
  menu_layer_set_selected_index(menu_layer, center_idx, MenuRowAlignNone, false);
}
#endif  // CONFIG_TOUCH

void menu_up_click_handler(ClickRecognizerRef recognizer, MenuLayer *menu_layer) {
  const bool up = true;
  if (menu_layer->scroll_wrap_around && prv_menu_scroll_handle_wrap_around(menu_layer, recognizer, up)) {
    return;
  }
#ifdef CONFIG_TOUCH
  prv_menu_reconcile_selection_before_step(menu_layer);
#endif

  MenuIndex prev_index = menu_layer->selection.index;
  const bool animated = true;
  menu_layer_set_selected_next(menu_layer, up, MenuRowAlignCenter, animated);
  MenuIndex current_index = menu_layer->selection.index;
  if ((menu_layer->scroll_vibe_on_blocked) &&
    (menu_index_compare(&current_index, &prev_index) == 0) &&
    (prv_menu_index_is_first_index(menu_layer, &current_index))) {
      prv_vibe_pulse();
  }
}

void menu_down_click_handler(ClickRecognizerRef recognizer, MenuLayer *menu_layer) {
  const bool up = false;
  if (menu_layer->scroll_wrap_around && prv_menu_scroll_handle_wrap_around(menu_layer, recognizer, up)) {
    return;
  }
#ifdef CONFIG_TOUCH
  prv_menu_reconcile_selection_before_step(menu_layer);
#endif

  MenuIndex prev_index = menu_layer->selection.index;
  const bool animated = true;
  menu_layer_set_selected_next(menu_layer, up, MenuRowAlignCenter, animated);
  MenuIndex current_index = menu_layer->selection.index;
  if ((menu_layer->scroll_vibe_on_blocked) &&
    (menu_index_compare(&current_index, &prev_index) == 0) &&
    (prv_menu_index_is_last_index(menu_layer, &current_index))) {
      prv_vibe_pulse();
  }
}

static void prv_menu_click_config_provider(MenuLayer *menu_layer) {
  // The config that gets passed in, has already the UP and DOWN buttons configured
  // we're overriding the default behavior here:
  window_single_repeating_click_subscribe(BUTTON_ID_UP, 100 /*ms*/,
      (ClickHandler)menu_up_click_handler);
  if (menu_layer->callbacks.select_click) {
    window_single_click_subscribe(BUTTON_ID_SELECT, (ClickHandler)prv_menu_select_click_handler);
  }
  if (menu_layer->callbacks.select_long_click) {
    window_long_click_subscribe(BUTTON_ID_SELECT, 0,
        (ClickHandler)prv_menu_select_long_click_handler, NULL);
  }
  window_single_repeating_click_subscribe(BUTTON_ID_DOWN, 100 /*ms*/,
      (ClickHandler)menu_down_click_handler);
}

static inline uint16_t prv_menu_layer_get_num_sections(MenuLayer *menu_layer) {
  if (menu_layer->callbacks.get_num_sections) {
    return menu_layer->callbacks.get_num_sections(menu_layer, menu_layer->callback_context);
  } else {
    return 1; // default
  }
}

static inline uint16_t prv_menu_layer_get_num_rows(MenuLayer *menu_layer, uint16_t section_index) {
  if (section_index == MENU_INDEX_NOT_FOUND) {
    return 0;
  }

  if (menu_layer->callbacks.get_num_rows) {
    return menu_layer->callbacks.get_num_rows(menu_layer, section_index,
        menu_layer->callback_context);
  } else {
    return 1;  // default
  }
}

static inline int16_t prv_menu_layer_get_separator_height(MenuLayer *menu_layer,
    MenuIndex *cell_index) {
  if (menu_layer->callbacks.get_separator_height) {
    return menu_layer->callbacks.get_separator_height(menu_layer, cell_index, menu_layer->callback_context);
  } else if (process_manager_compiled_with_legacy2_sdk()) {
    return MENU_CELL_LEGACY2_BASIC_SEPARATOR_HEIGHT;
  } else {
    return MENU_CELL_BASIC_SEPARATOR_HEIGHT;
  }
}

static inline int16_t prv_menu_layer_get_header_height(MenuLayer *menu_layer,
    uint16_t section_index) {
  if (menu_layer->callbacks.get_header_height) {
    return menu_layer->callbacks.get_header_height(menu_layer, section_index, menu_layer->callback_context);
  } else {
    return 0; // default
  }
}

static inline int16_t prv_menu_layer_get_cell_height(MenuLayer *menu_layer, MenuIndex
    *cell_index, bool provide_correct_selection_index) {
  if (menu_layer->callbacks.get_cell_height) {
    const MenuIndex prev_selection_index = menu_layer->selection.index;
    if (!provide_correct_selection_index) {
      menu_layer->selection.index.section = MENU_INDEX_NOT_FOUND;
    }
    const int16_t result = menu_layer->callbacks.get_cell_height(menu_layer, cell_index,
                                                                 menu_layer->callback_context);

    menu_layer->selection.index = prev_selection_index;
    return result;
  } else {
    return menu_cell_basic_cell_height();  // default
  }
}

static inline void prv_menu_layer_draw_separator(MenuLayer *menu_layer, Layer *cell_layer,
    MenuCellSpan *cursor, GContext* ctx) {
  const int16_t y = cursor->y - cursor->sep;
  if (menu_layer->callbacks.draw_separator) {
    // Save current drawing state:
    GDrawState prev_state = graphics_context_get_drawing_state(ctx);
    GRect prev_bounds = cell_layer->bounds;
    GRect new_bounds = prev_bounds;

    // Translate the drawing_box to the bounds of the layer:
    ctx->draw_state.drawing_box.origin.y += y;
    ctx->draw_state.drawing_box.size.h = cursor->h;

    // Set the height appropriately on the cell layer
    new_bounds.size.h = cursor->sep;
    layer_set_bounds(cell_layer, &new_bounds);

    // Call the client, to ask to draw the separator:
    menu_layer->callbacks.draw_separator(ctx, cell_layer, &cursor->index, menu_layer->callback_context);

    // Restore current drawing state:
    graphics_context_set_drawing_state(ctx, prev_state);

    // Restore the layer bounds:
    layer_set_bounds(cell_layer, &prev_bounds);
  } else {
    graphics_fill_rect(
        ctx, &GRect(0, y, menu_layer->scroll_layer.layer.bounds.size.w, cursor->sep));
  }
}

static void prv_prepare_row(GContext *ctx, MenuLayer *menu_layer,
                            Layer *cell_layer, bool highlight) {
  if (!process_manager_compiled_with_legacy2_sdk()) {
    GColor *colors = (highlight) ? menu_layer->highlight_colors : menu_layer->normal_colors;
    ctx->draw_state.fill_color = colors[MenuLayerColorBackground];
    ctx->draw_state.text_color = colors[MenuLayerColorForeground];
    ctx->draw_state.tint_color = colors[MenuLayerColorForeground];
    if (!gcolor_is_transparent(ctx->draw_state.fill_color)) {
      graphics_fill_rect(ctx, &cell_layer->bounds);
    }
  }
  cell_layer->is_highlighted = highlight;
}

static void prv_prepare_and_draw_row(GContext *ctx, MenuLayer *menu_layer,
                                     Layer *cell_layer, MenuCellSpan *cursor, bool highlight) {
  prv_prepare_row(ctx, menu_layer, cell_layer, highlight);
  const GRect prev_bounds = cell_layer->bounds;

  // in theory, we could decrement the origin by cell_content_origin_offset_y after the call
  // in practice once shouldn't trust the draw_row implementation
  const int16_t draw_box_origin_y = ctx->draw_state.drawing_box.origin.y;
  ctx->draw_state.drawing_box.origin.y += menu_layer->animation.cell_content_origin_offset_y;

  // Call the client, to ask to draw the row:
  menu_layer->callbacks.draw_row(ctx, cell_layer, &cursor->index, menu_layer->callback_context);

  ctx->draw_state.drawing_box.origin.y = draw_box_origin_y;
  cell_layer->bounds = prev_bounds;
}

static inline void prv_menu_layer_draw_row(MenuLayer *menu_layer, Layer *cell_layer,
    MenuCellSpan *cursor, GContext* ctx) {
  if (cursor->h == 0) {
    // cell has height 0, no need to draw anything.
    return;
  }

  cell_layer->bounds.size.h = cursor->h;
  cell_layer->frame.size.h = cursor->h;
  cell_layer->frame.origin.y = cursor->y;

  // Save current drawing state:
  GDrawState prev_state = graphics_context_get_drawing_state(ctx);

  // Translate the drawing_box to the bounds of the layer:
  ctx->draw_state.drawing_box.origin.y += cursor->y;
  ctx->draw_state.drawing_box.size.h = cursor->h;

  // Use the drawing_box as a clipper to force the content to only use
  // the space available to it and remove overflow
  const GRect *const rect_clipper = (const GRect *const)&ctx->draw_state.drawing_box;
  grect_clip((GRect *const)&ctx->draw_state.clip_box, rect_clipper);

  const bool fully_covered = grect_equal(&cell_layer->frame, &menu_layer->inverter.layer.frame);
  const bool partial = grect_overlaps_grect(&cell_layer->frame, &menu_layer->inverter.layer.frame);

  if (fully_covered || !partial) {
    prv_prepare_and_draw_row(ctx, menu_layer, cell_layer, cursor, fully_covered);
  } else {
    // Render the full cell without highlight
    prv_prepare_and_draw_row(ctx, menu_layer, cell_layer, cursor, false);

    // Set clipper to the inverter layer in clipping box coordinates
    GRect selection_clipper;
    layer_get_global_frame(&menu_layer->inverter.layer, &selection_clipper);
    grect_clip((GRect *const)&ctx->draw_state.clip_box, &selection_clipper);

    // Render with highlight
    prv_prepare_and_draw_row(ctx, menu_layer, cell_layer, cursor, true);
  }

  // Restore current drawing state:
  graphics_context_set_drawing_state(ctx, prev_state);
}

static inline void prv_menu_layer_draw_section_header(MenuLayer *menu_layer, Layer *cell_layer,
    MenuCellSpan *cursor, GContext* ctx) {
  cell_layer->bounds.size.h = cursor->h;
  cell_layer->frame.size.h = cursor->h;
  cell_layer->frame.origin.y = cursor->y;

  // Callback to get the shared cell instance filled with data:

  // Save current drawing state:
  GDrawState prev_state = graphics_context_get_drawing_state(ctx);

  // Translate the drawing_box to the bounds of the layer:
  ctx->draw_state.drawing_box.origin.y += cursor->y;
  ctx->draw_state.drawing_box.size.h = cursor->h;

  const GRect *const rect_clipper = (const GRect *const)&ctx->draw_state.drawing_box;
  grect_clip((GRect *const)&ctx->draw_state.clip_box, rect_clipper);

  prv_prepare_row(ctx, menu_layer, cell_layer, false);

  // Call the client, to ask to draw the section:
  menu_layer->callbacks.draw_header(ctx, cell_layer, cursor->index.section, menu_layer->callback_context);

  // Restore current drawing state:
  graphics_context_set_drawing_state(ctx, prev_state);
}

static void prv_menu_layer_render_section_from_iterator(MenuIterator *iterator) {
  MenuRenderIterator *it = (MenuRenderIterator*)iterator;
  const int16_t top_diff = it->it.cursor.y - it->content_top_y;
  const bool is_header_in_frame = (top_diff >= 0 && it->it.cursor.y <= it->content_bottom_y) ||
      (it->it.cell_bottom_y >= it->content_top_y && it->it.cell_bottom_y <= it->content_bottom_y);
  if (is_header_in_frame) {
    // Draw section header:
    prv_menu_layer_draw_section_header(it->it.menu_layer, &it->cell_layer, &it->it.cursor, it->ctx);
    // Draw the separator on top of the cell:
    if (top_diff >= it->it.cursor.sep) {
      prv_menu_layer_draw_separator(it->it.menu_layer, &it->cell_layer, &it->it.cursor, it->ctx);
    }
  }
}

static void prv_menu_layer_render_row_from_iterator(MenuIterator *iterator) {
  MenuRenderIterator *it = (MenuRenderIterator*)iterator;
  const int16_t iter_y = it->it.cursor.y;

  const int16_t top_diff = it->it.cursor.y - it->content_top_y;
  const bool is_row_in_frame = (top_diff >= 0 && it->it.cursor.y <= it->content_bottom_y) ||
      (it->it.cell_bottom_y >= it->content_top_y && it->it.cell_bottom_y <= it->content_bottom_y);
  if (is_row_in_frame) {
    it->cursor_in_frame = true;
    // Draw the cell
    prv_menu_layer_draw_row(it->it.menu_layer, &it->cell_layer, &it->it.cursor, it->ctx);
    // Draw the separator on top of the cell
    if (top_diff >= it->it.cursor.sep) {
      prv_menu_layer_draw_separator(it->it.menu_layer, &it->cell_layer, &it->it.cursor, it->ctx);
    }
    // Update the cache with the center-most row
    it->it.cursor.y = iter_y;
    if (false == it->cache_set) {
      it->new_cache = it->it.cursor;
      it->cache_set = true;
    }
  } else {
    if (it->cursor_in_frame) {
      it->it.should_continue = false;
    }
  }
  it->it.cursor.y = iter_y;
}

// NOTE: The following two iteration functions are asymmetrical!
// In other words, even one is going downward and the other upward, there are some subtle
// differences. Most importantly: the downward function calls the row_callback_after_geometry for
// the row the iterator's cursor is currently set to, while the upward function skips over the
// current row.
// Secondly, section_callback is only called when a sections is encountered while walking.
// For example, if the current index is (section: 0, row: 0), the section_callback for section 0
// will only be called when walking upward.

static void prv_menu_layer_walk_downward_from_iterator(MenuIterator *it) {
  const uint16_t num_sections = prv_menu_layer_get_num_sections(it->menu_layer);
  it->should_continue = true;
  for (;;) { // sections
    const uint16_t num_rows_in_section = prv_menu_layer_get_num_rows(it->menu_layer,
        it->cursor.index.section);
    for (;;) { // rows
      if (it->cursor.index.row >= num_rows_in_section) {
        // Reached last row
        break;
      }

      if (it->row_callback_before_geometry) {
        it->row_callback_before_geometry(it);
      }

      it->cursor.h = prv_menu_layer_get_cell_height(it->menu_layer, &it->cursor.index, true);
      it->cell_bottom_y = it->cursor.y + it->cursor.h;

      // ROW
      if (it->row_callback_after_geometry) {
        it->row_callback_after_geometry(it);
      }
      if (it->should_continue == false) {
        return;
      }

      // Next row:
      it->cursor.sep = prv_menu_layer_get_separator_height(it->menu_layer, &it->cursor.index);
      it->cursor.y = it->cell_bottom_y; // Bottom of previous cell is y of the next cell

      // Don't leave space for the seperator for the (non-existent) row after the last row.
      // This doesn't impact cell drawing in this loop (this condition will only trip on the last run).
      // But, other parts of the system rely on the cursor being set properly at the end of this iteration.
      if (it->cursor.index.row < num_rows_in_section - 1 || it->cursor.index.section < num_sections - 1) {
        it->cursor.y += it->cursor.sep;
      }
      ++(it->cursor.index.row);
    } // for() rows

    // Next section:
    ++(it->cursor.index.section);
    if (it->cursor.index.section >= num_sections) {
      break;
      // Reached last section
    }
    it->cursor.index.row = 0;
    it->cursor.h = prv_menu_layer_get_header_height(it->menu_layer, it->cursor.index.section);
    it->cell_bottom_y = it->cursor.y + it->cursor.h;

    // SECTION
    if (it->cursor.h > 0) {
      it->section_callback(it);
      it->cursor.sep = prv_menu_layer_get_separator_height(it->menu_layer, &it->cursor.index);
      it->cursor.y = it->cell_bottom_y + it->cursor.sep;
    }

    if (it->should_continue == false) {
      return;
    }

  } // for() sections
}

static void prv_menu_layer_walk_upward_from_iterator(MenuIterator *it) {
  it->should_continue = true;
  for (;;) { // sections
    for (;;) { // rows
      // Previous row
      if (it->cursor.index.row == 0) {
        // Reached top-most row in current section
        break;
      }
      --(it->cursor.index.row);

      if (it->row_callback_before_geometry) {
        it->row_callback_before_geometry(it);
      }

      // when walking upwards, selected_index isn't set yet here
      // hence, the heights are the sizes as they were before the selection changed
      it->cursor.h = prv_menu_layer_get_cell_height(it->menu_layer, &it->cursor.index, false);
      it->cursor.sep = prv_menu_layer_get_separator_height(it->menu_layer, &it->cursor.index);
      it->cursor.y -= it->cursor.h + it->cursor.sep;
      it->cell_bottom_y = it->cursor.y + it->cursor.h;

      // ask for height again, this time with correct selection status
      it->cursor.h = prv_menu_layer_get_cell_height(it->menu_layer, &it->cursor.index, true);

      // ROW
      if (it->row_callback_after_geometry) {
        it->row_callback_after_geometry(it);
      }

      if (it->should_continue == false) {
        break;
      }
    } // for() rows

    if (it->cursor.index.row == 0) {
      // If top-most row, layout the section header
      it->cursor.h = prv_menu_layer_get_header_height(it->menu_layer, it->cursor.index.section);
      it->cursor.sep = prv_menu_layer_get_separator_height(it->menu_layer, &it->cursor.index);

      if (it->cursor.h > 0) {
        // Bottom of previous cell is y of the next cell
        const int16_t total_height = it->cursor.h + it->cursor.sep;
        if (total_height > it->cursor.y) {
          // If the total height is greater than the cursor y, don't
          // add in space to accodomate the separator as the downwards callback
          // will add it for us.
          it->cursor.y -= it->cursor.h;
        } else {
          it->cursor.y -= total_height;
        }
        it->cell_bottom_y = it->cursor.y + it->cursor.h;

        // SECTION
        it->section_callback(it);
      }
    }

    if (it->should_continue == false) {
      return;
    }

    // Previous section:
    if (it->cursor.index.section == 0) {
      // Reached top
      break;
    }
    --(it->cursor.index.section);
    // -1 will happen when entering for() rows
    it->cursor.index.row = it->menu_layer->callbacks.get_num_rows(it->menu_layer,
        it->cursor.index.section, it->menu_layer->callback_context);

  } // for() sections
}

static void NOINLINE prv_draw_background(MenuLayer *menu_layer, GContext *ctx,
                                Layer *bg_layer, bool highlight) {
  GDrawState prev_state = graphics_context_get_drawing_state(ctx);

  const GRect *bounds = &bg_layer->bounds;
  ctx->draw_state.drawing_box.origin.y = bounds->origin.y;
  ctx->draw_state.drawing_box.size.h = bounds->size.h;

  MenuLayerDrawBackgroundCallback draw_background_cb = menu_layer->callbacks.draw_background;
  if (draw_background_cb) {
    draw_background_cb(ctx, bg_layer, false, menu_layer->callback_context);
  } else if (highlight) {
    ctx->draw_state.fill_color = menu_layer->highlight_colors[MenuLayerColorBackground];
    graphics_fill_rect(ctx, bounds);
  } else {
    ctx->draw_state.fill_color = menu_layer->normal_colors[MenuLayerColorBackground];
    graphics_fill_rect(ctx, bounds);
  }

  graphics_context_set_drawing_state(ctx, prev_state);
}

void menu_layer_update_proc(Layer *scroll_content_layer, GContext* ctx) {
  MenuLayer *menu_layer = (MenuLayer*)(((uint8_t*)scroll_content_layer) -
      offsetof(MenuLayer, scroll_layer.content_sublayer));
  const GSize frame_size = menu_layer->scroll_layer.layer.frame.size;
  const int16_t content_top_y = -scroll_layer_get_content_offset(&menu_layer->scroll_layer).y;
  const int16_t content_bottom_y = content_top_y + frame_size.h;

  if (!process_manager_compiled_with_legacy2_sdk()) {
    prv_draw_background(menu_layer, ctx, &menu_layer->scroll_layer.layer, false);
  }

  if (menu_layer->overscroll_stretched) {
    // The overscroll-stretched selection background: cells paint over their own part, so only
    // the gap beyond the content edge shows it.
    graphics_context_set_fill_color(ctx, menu_layer->highlight_colors[MenuLayerColorBackground]);
    graphics_fill_rect(ctx, &menu_layer->inverter.layer.frame);
  }

  MenuRenderIterator *render_iter = applib_type_malloc(MenuRenderIterator);
  PBL_ASSERTN(render_iter);

  if (menu_layer->center_focused) {
    // in this mode, the selected row is always the best candidate for the cache
    menu_layer->cache.cursor = menu_layer->selection;
  }

  *render_iter = (MenuRenderIterator) {
    .it = {
      .menu_layer = menu_layer,
      .cursor = menu_layer->cache.cursor,
      .row_callback_after_geometry = prv_menu_layer_render_row_from_iterator,
      .section_callback = prv_menu_layer_render_section_from_iterator,
    },
    .ctx = ctx,
    .content_top_y = content_top_y,
    .content_bottom_y = content_bottom_y,
    .cache_set = false,
    .cursor_in_frame = false,
    .cell_layer = {
      .bounds = {
        .size = {
          .w = frame_size.w,
        },
      },
      .frame = {
        .size = {
          .w = frame_size.w,
        },
      },
    },
  };
  layer_add_child(&menu_layer->scroll_layer.content_sublayer, &render_iter->cell_layer);

  // Set separator color
  graphics_context_set_fill_color(ctx, GColorBlack);

  // We're caching the y-coord and index of the one row, as our "anchor" point in the menu.
  // We'll be walking downward and upward from that index until the rows fall off the screen.
  const int16_t content_center_y = (content_top_y + content_bottom_y) / 2;
  if (content_center_y >= menu_layer->cache.cursor.y) {
    // Walk downward from cache.cursor, then upward
    prv_menu_layer_walk_downward_from_iterator(&render_iter->it);
    render_iter->it.cursor = menu_layer->cache.cursor;
    prv_menu_layer_walk_upward_from_iterator(&render_iter->it);
  } else {
    // Walk upward from cache.cursor, then downward
    prv_menu_layer_walk_upward_from_iterator(&render_iter->it);
    render_iter->it.cursor = menu_layer->cache.cursor;
    prv_menu_layer_walk_downward_from_iterator(&render_iter->it);
  }
  layer_remove_from_parent(&render_iter->cell_layer);

  // Assign the new cache:
  menu_layer->cache.cursor = render_iter->new_cache;

  task_free(render_iter);

  if (menu_layer->scrollbar_visible) {
    prv_scrollbar_draw(menu_layer, ctx, content_top_y);
  }
}

void menu_layer_init_scroll_layer_callbacks(MenuLayer *menu_layer) {
  ScrollLayer *scroll_layer = &menu_layer->scroll_layer;
  scroll_layer_set_callbacks(scroll_layer, (ScrollLayerCallbacks) {
    .click_config_provider = (ClickConfigProvider)prv_menu_click_config_provider,
    .content_offset_changed_handler = (ScrollLayerCallback)prv_menu_scroll_offset_changed_handler,
  });
  scroll_layer->content_sublayer.update_proc = (LayerUpdateProc)menu_layer_update_proc;
}

static void prv_set_center_focused(MenuLayer *menu_layer, bool center_focused) {
  menu_layer->center_focused = center_focused;
  scroll_layer_set_clips_content_offset(&menu_layer->scroll_layer, !center_focused);
}

void menu_layer_init(MenuLayer *menu_layer, const GRect *frame) {
  *menu_layer = (MenuLayer) {
    .pad_bottom = true,
  };

  ScrollLayer *scroll_layer = &menu_layer->scroll_layer;
  scroll_layer_init(scroll_layer, frame);
#ifdef CONFIG_TOUCH
  // scroll_layer_init() registered the embedded scroll layer as a bare Tier-1 Scroll widget. The
  // MenuLayer drives scrolling through its own Menu registration, so drop the redundant Scroll node
  // to avoid two gesture drivers competing on the same layer.
  scroll_layer_touch_nav_deregister(scroll_layer);
#endif
  menu_layer_init_scroll_layer_callbacks(menu_layer);
  scroll_layer_set_shadow_hidden(scroll_layer, true);
  scroll_layer_set_context(scroll_layer, menu_layer);

  menu_layer_set_normal_colors(menu_layer, GColorWhite, GColorBlack);
  menu_layer_set_highlight_colors(menu_layer, GColorBlack, GColorWhite);

  InverterLayer *inverter = &menu_layer->inverter;
  inverter_layer_init(inverter, &GRectZero);
  scroll_layer_add_child(scroll_layer, &inverter->layer);

  // Hide inverter layer by default for 3.0 apps
  layer_set_hidden(inverter_layer_get_layer(&menu_layer->inverter), true);

#if PBL_ROUND
  prv_set_center_focused(menu_layer, true);
#endif

#ifdef CONFIG_TOUCH
  prv_menu_touch_nav_register(menu_layer);
#endif
}

MenuLayer* menu_layer_create(GRect frame) {
  MenuLayer *layer = applib_type_malloc(MenuLayer);
  if (layer) {
    menu_layer_init(layer, &frame);
  }
  return layer;
}

void menu_layer_pad_bottom_enable(MenuLayer *menu_layer, bool enable) {
  menu_layer->pad_bottom = enable;
}

void menu_layer_deinit(MenuLayer *menu_layer) {
#ifdef CONFIG_TOUCH
  // Deregister from the Tier-1 touch registry first so a gesture in flight on this widget is
  // cancelled with no client callbacks before its client state is torn down (double-deinit safe).
  prv_menu_touch_nav_deregister(menu_layer);
#endif
  prv_cancel_selection_animation(menu_layer);
  prv_scrollbar_cancel_hide_timer(menu_layer);
  layer_deinit(&menu_layer->inverter.layer);
  scroll_layer_deinit(&menu_layer->scroll_layer);
}

void menu_layer_destroy(MenuLayer* menu_layer) {
  if (menu_layer == NULL) {
    return;
  }
  menu_layer_deinit(menu_layer);
  applib_free(menu_layer);
}

Layer* menu_layer_get_layer(const MenuLayer *menu_layer) {
  return &((MenuLayer *)menu_layer)->scroll_layer.layer;
}

ScrollLayer* menu_layer_get_scroll_layer(const MenuLayer *menu_layer) {
  return &((MenuLayer *)menu_layer)->scroll_layer;
}

typedef struct MenuPrimeCacheIterator {
  MenuIterator it;
  bool cache_set;
} MenuPrimeCacheIterator;

static void prv_menu_layer_iterator_noop_callback(MenuIterator *it) {
  (void)it;
}

static void prv_menu_layer_iterator_prime_cache_callback(MenuIterator *iterator) {
  MenuPrimeCacheIterator *it = (MenuPrimeCacheIterator*)iterator;
  if (false == it->cache_set) {
    // Prime the cursor cache:
    it->it.menu_layer->cache.cursor = it->it.cursor;
    // Set initial selection too:
    it->it.menu_layer->selection = it->it.cursor;
    it->cache_set = true;
  }
}

//! Calculate the total height of all row cells and section headers,
//! and assign the appropriate content size to the scroll_layer.
//! Also prime the offset cache on the fly.
void menu_layer_update_caches(MenuLayer *menu_layer) {
  // Save the currently selected cell index.
  MenuIndex selected_index = menu_layer_get_selected_index(menu_layer);
  MenuPrimeCacheIterator it = {
    .it = {
      .menu_layer = menu_layer,
      .row_callback_after_geometry = prv_menu_layer_iterator_prime_cache_callback,
      .section_callback = prv_menu_layer_iterator_noop_callback,
      .should_continue = true,
      .cursor = {
        // Section header of current section (0) is not part of the walk down, set it "manually"
        .y = prv_menu_layer_get_header_height(menu_layer, 0),
        .sep = prv_menu_layer_get_separator_height(menu_layer, 0)
      },
    },
    .cache_set = false,
  };

  if (prv_menu_layer_get_header_height(menu_layer, 0) != 0) {
    // We have to add the separator height, as when drawing down -> up, we render the separator
    // for the row above before proceeding down. We only render this separator at the top if we
    // have headers on the first section.
    it.it.cursor.y += it.it.cursor.sep;
  }

  // handle special case of just one row so that calls for menu_layer_get_selected_index()
  // will already answer correctly
  if (prv_menu_layer_get_num_sections(menu_layer) == 1 &&
      prv_menu_layer_get_num_rows(menu_layer, 0) == 1) {
    menu_layer->selection.index = MenuIndex(0, 0);
  }

  prv_menu_layer_walk_downward_from_iterator(&it.it);
  int16_t total_height = it.it.cursor.y;
  if (menu_layer->pad_bottom) {
    total_height += MENU_LAYER_BOTTOM_PADDING;
  }

  // Set the content size on the scroll layer, so all the rows will fit onto the content layer:
  const GSize frame_size = menu_layer->scroll_layer.layer.frame.size;
  scroll_layer_set_content_size(&menu_layer->scroll_layer, GSize(frame_size.w, total_height));

  // Set the selected cell again:
  const bool animated = false;
  menu_layer_set_selected_index(menu_layer, selected_index, MenuRowAlignNone, animated);
}

void menu_layer_set_callbacks(MenuLayer *menu_layer, void *callback_context,
                            const MenuLayerCallbacks *callbacks) {
  if (callbacks) {
    menu_layer->callbacks = *callbacks;
    PBL_ASSERTN(menu_layer->callbacks.draw_row);
    PBL_ASSERTN(menu_layer->callbacks.get_num_rows);
  }

  menu_layer->callback_context = callback_context;

  menu_layer_reload_data(menu_layer);
}

void menu_layer_set_callbacks_by_value(MenuLayer *menu_layer, void *callback_context,
                                       MenuLayerCallbacks callbacks) {
  menu_layer_set_callbacks(menu_layer, callback_context, &callbacks);
}

void menu_layer_set_click_config_onto_window(MenuLayer *menu_layer, struct Window *window) {
  // Delegate this directly to the scroll layer:
  scroll_layer_set_click_config_onto_window(&menu_layer->scroll_layer, window);
}

//! @returns 0 if A and B are equal, 1 if A has a higher section & row combination than B or else -1
int16_t menu_index_compare(const MenuIndex *a, const MenuIndex *b) {
  const int16_t max_rows = MAX(a->row, b->row) + 1;
  const int32_t a_abs = ((a->section * max_rows) + a->row);
  const int32_t b_abs = ((b->section * max_rows) + b->row);
  if (a_abs > b_abs) {
    return 1;
  } else if (a_abs < b_abs) {
    return -1;
  } else {
    return 0;
  }
}

static void prv_selection_complete(Animation *animation, bool finished, void *context) {
  MenuLayer *menu_layer = (MenuLayer *) context;
  menu_layer->animation.animation = NULL;
}

static bool prv_cancel_selection_animation(MenuLayer *menu_layer) {
  const bool result = animation_is_scheduled(menu_layer->animation.animation);
  if (result) {
    animation_unschedule(menu_layer->animation.animation);
  }
  menu_layer->animation.animation = NULL;
  return result;
}

#define TOP_DOWN_PX  7
#define BOTTOM_DOWN_PX 10
static void prv_setup_selection_animation(MenuLayer *menu_layer, bool up) {
  // Move selection inverter layer:
  const int16_t w = menu_layer->scroll_layer.layer.frame.size.w;
  const GSize size = GSize(w, menu_layer->selection.h);

  // Step 1. Bring down TOP of cell by TOP_DOWN_PX.
  GRect from;
  if (menu_layer->animation.animation) {
    from = menu_layer->animation.target;
    prv_cancel_selection_animation(menu_layer);
  } else {
    from = menu_layer->inverter.layer.frame;
  }
  GRect target = (GRect) {
    .origin = {
      .x = 0,
      .y = from.origin.y + ((up) ? 0 : TOP_DOWN_PX),
    },
    .size = {
      .w = size.w,
      .h = size.h - TOP_DOWN_PX,
    }
  };

  Animation *a1 = (Animation *) property_animation_create_layer_frame(&menu_layer->inverter.layer,
                                                                      &from, &target);
  animation_set_duration(a1, 100);
  animation_set_curve(a1, AnimationCurveEaseOut);
  animation_set_auto_destroy(a1, true);

  // Step 2. Skip the top of the highlight down to the top of the newly selected cell,
  // and have the selection BOTTOM_DOWN_PX below the selected cell.
  from.origin.y = menu_layer->selection.y - ((up) ? BOTTOM_DOWN_PX : 0);
  from.size.h = size.h + BOTTOM_DOWN_PX;

  // Step 3. Bring up the bottom of the highlight to only cover the selected cell.
  target.origin.y = menu_layer->selection.y;
  target.size = size;

  Animation *a2 = (Animation *) property_animation_create_layer_frame(&menu_layer->inverter.layer,
                                                                      &from, &target);
  animation_set_duration(a2, 250);
  animation_set_curve(a2, AnimationCurveEaseOut);
  animation_set_auto_destroy(a2, true);

  Animation *a = animation_sequence_create(a1, a2, NULL);

  animation_set_auto_destroy(a, true); // [MJ] false?
  animation_set_handlers(a, (AnimationHandlers) { .stopped = prv_selection_complete }, menu_layer);

  menu_layer->animation.animation = a;
  menu_layer->animation.target = target;
  animation_schedule(a);
}


static void prv_menu_layer_update_selection_highlight(MenuLayer *menu_layer, bool up,
                                                      bool animated,
                                                      bool change_ongoing_animation) {
  if (menu_layer->center_focused || menu_layer->selection_animation_disabled) {
    // animation on center_focused will not happen by moving the selection
    // see prv_schedule_center_focus_animation()
    animated = false;
  }

  Animation *scroll_animation = (Animation *) menu_layer->scroll_layer.animation;
  if (change_ongoing_animation && animation_is_scheduled(scroll_animation)) {
    animation_unschedule(scroll_animation);
  }
  if (change_ongoing_animation && animated && !process_manager_compiled_with_legacy2_sdk()) {
    prv_setup_selection_animation(menu_layer, up);
  } else {
    if (change_ongoing_animation) {
      prv_cancel_selection_animation(menu_layer);
    }
    // Move selection inverter layer:
    const int16_t w = menu_layer->scroll_layer.layer.frame.size.w;
    const GSize size = GSize(w, menu_layer->selection.h);
    menu_layer->inverter.layer.bounds = (GRect) {
      .origin = { 0, 0 },
      .size = size,
    };
    menu_layer->inverter.layer.frame = (GRect) {
      .origin = {
        .x = 0,
        .y = menu_layer->selection.y,
      },
      .size = size,
    };
    layer_mark_dirty(&menu_layer->inverter.layer);
  }
}

static MenuRowAlign prv_corrected_scroll_align(MenuLayer *menu_layer, MenuRowAlign align) {
  if (menu_layer->center_focused) {
    return MenuRowAlignCenter;
  }
  return align;
}

static void prv_menu_layer_update_selection_scroll_position(MenuLayer *menu_layer,
                                                            MenuRowAlign scroll_align,
                                                            bool animated) {
  scroll_align = prv_corrected_scroll_align(menu_layer, scroll_align);

  if (scroll_align != MenuRowAlignNone) {
    int16_t y;
    const GSize frame_size = menu_layer->scroll_layer.layer.frame.size;
    // Scroll to the right position:
    switch (scroll_align) {
      case MenuRowAlignTop:
        y = - menu_layer->selection.y;
        break;

      case MenuRowAlignBottom:
        y = frame_size.h - menu_layer->selection.y - menu_layer->selection.h;
        break;

      default:
      case MenuRowAlignCenter:
        y = (frame_size.h / 2) - menu_layer->selection.y - (menu_layer->selection.h / 2);
        break;
    }

    if (menu_layer->center_focused) {
      // animation on center_focus will not happen via scrolling
      // see prv_schedule_center_focus_animation()
      animated = false;
    }
    // scroll layer will take care of clipping if necessary
    scroll_layer_set_content_offset(&menu_layer->scroll_layer, GPoint(0, y), animated);
  }
}

typedef struct MenuSelectIndexIterator {
  MenuIterator it;
  MenuCellSpan selection;
  bool did_change_selection:1;
} MenuSelectIndexIterator;

static void prv_menu_layer_iterator_selection_index_callback(MenuIterator *iterator) {
  MenuSelectIndexIterator *it = (MenuSelectIndexIterator*)iterator;
  if (!menu_index_compare(&it->it.cursor.index, &it->selection.index)) {
    it->it.menu_layer->selection = it->it.cursor;
    it->it.should_continue = false;
    it->did_change_selection = true;
  }
}

static void prv_menu_layer_iterator_update_selection(MenuIterator *iterator) {
  MenuLayer *menu_layer = iterator->menu_layer;
  if (menu_index_compare(&iterator->cursor.index, &menu_layer->selection.index) == 0) {
    menu_layer->selection = iterator->cursor;
  }
}

static void prv_walk_with_iterator(const int8_t direction, MenuIterator *it) {
  MenuLayer *menu_layer = it->menu_layer;
  const int16_t prev_selection_height = menu_layer->selection.h;
  const MenuIndex prev_selection_index = menu_layer->selection.index;

  if (menu_layer->center_focused) {
    it->row_callback_before_geometry = it->row_callback_after_geometry;
    it->row_callback_after_geometry = prv_menu_layer_iterator_update_selection;

    // invalidate current selection while iterating
    menu_layer->selection.index.section = MENU_INDEX_NOT_FOUND;
  }

  if (direction < 0) {
    // new index comes before current selection
    prv_menu_layer_walk_upward_from_iterator(it);
  } else if (direction > 0) {
    // new index comes after current selection
    prv_menu_layer_walk_downward_from_iterator(it);
  }

  // potentially restore previous state of selection
  if (menu_layer->selection.index.section == MENU_INDEX_NOT_FOUND) {
    menu_layer->selection.index = prev_selection_index;
    menu_layer->selection.h = prev_selection_height;
  }
}

typedef struct {
  MenuLayer *menu_layer;
  bool up;
} CenterFocusSelectionAnimationState;

static CenterFocusSelectionAnimationState prv_center_focus_animation_state(Animation *animation) {
  PropertyAnimation *prop_anim = (PropertyAnimation *)animation;
  CenterFocusSelectionAnimationState result = {};
  property_animation_get_subject(prop_anim, (void **)&result.menu_layer);
  property_animation_to(prop_anim, &result.up, sizeof(result.up), false);
  return result;
}

static void prv_center_focus_animation_setup(Animation *animation) {
  CenterFocusSelectionAnimationState state = prv_center_focus_animation_state(animation);
  state.menu_layer->animation.cell_content_origin_offset_y = 0;
  state.menu_layer->animation.selection_extend_top = 0;
  state.menu_layer->animation.selection_extend_bottom = 0;
}

static void prv_announce_selection_changed(MenuLayer *menu_layer, MenuIndex prev_index) {
  if (!menu_layer->callbacks.selection_changed) {
    return;
  }

  menu_layer->callbacks.selection_changed(menu_layer, menu_layer->selection.index,
                                          prev_index, menu_layer->callback_context);
}

void prv_center_focus_animation_update_impl(Animation *animation,
                                            bool second_half,
                                            AnimationProgress adjusted_progress) {
  CenterFocusSelectionAnimationState state = prv_center_focus_animation_state(animation);

  // values as seen in the design videos
  const int16_t move_in_dist = 16;
  const int16_t move_out_dist = 4;
  const int16_t abs_content_offset = second_half ?
                                     interpolate_int16(adjusted_progress, move_out_dist, 0) :
                                     interpolate_int16(adjusted_progress, 0, move_in_dist);
  const int16_t content_offset = (state.up ? abs_content_offset : -abs_content_offset) / 2;
  state.menu_layer->animation.cell_content_origin_offset_y = content_offset;

  const bool reached_second_half_before = menu_index_compare(
      &state.menu_layer->selection.index, &state.menu_layer->animation.new_selection.index) == 0;

  if (second_half) {
    if (!reached_second_half_before) {
      const MenuIndex prev_index = state.menu_layer->selection.index;
      state.menu_layer->selection = state.menu_layer->animation.new_selection;
      prv_announce_selection_changed(state.menu_layer, prev_index);
    }
    // this favors robustness over efficiency - the functions might be called multiple times
    // but instead of keeping track (which is more difficult that it seems)
    // we simply call them too often
    prv_menu_layer_update_selection_scroll_position(state.menu_layer, MenuRowAlignCenter, false);
    prv_menu_layer_update_selection_highlight(state.menu_layer, state.up, false, false);
    state.menu_layer->inverter.layer.frame.size.h += abs_content_offset;
    state.menu_layer->inverter.layer.bounds.size = state.menu_layer->inverter.layer.frame.size;

    // when scrolling up, bounce back at the top (otherwise at the bottom)
    if (!state.up) {
      state.menu_layer->inverter.layer.frame.origin.y -= abs_content_offset;
    }
  }
  layer_mark_dirty(&state.menu_layer->scroll_layer.layer);
}

void prv_center_focus_animation_update_in_and_out(Animation *animation,
                                                  const AnimationProgress progress) {
  const AnimationProgress half_progress = ANIMATION_NORMALIZED_MAX / 2;
  const bool second_half = progress >= half_progress;
  const AnimationProgress adjusted_progress = second_half ?
      animation_timing_scaled(progress, half_progress, ANIMATION_NORMALIZED_MAX) :
      animation_timing_scaled(progress, 0, half_progress);
  prv_center_focus_animation_update_impl(animation, second_half, adjusted_progress);
}

void prv_center_focus_animation_update_out_only(Animation *animation,
                                                const AnimationProgress progress) {
  // anwalys only render the bounce back
  prv_center_focus_animation_update_impl(animation, true, progress);
}

static void prv_center_focus_animation_teardown(Animation *animation) {
  // usually a "redundant" call. Just in case the animation gets cancelled before finish
  prv_center_focus_animation_update_in_and_out(animation, ANIMATION_NORMALIZED_MAX);
}

static void prv_schedule_center_focus_animation(MenuLayer *menu_layer, bool up,
                                                const MenuCellSpan *prev_selection,
                                                bool was_animating) {
  // we reconfigure the current index to be the previous index so that all parties in the ongoing
  // animation will continue to reply with the proper values with respect to the selection
  // half-way through the animation we then switch (back) to the new index
  menu_layer->animation.new_selection = menu_layer->selection;
  menu_layer->selection = *prev_selection;

  // force selection + scrolling to be at the right spot, not animated since the actual animation
  // for center_focused is done via rendering offset below
  const bool selection_animated = false;
  prv_menu_layer_update_selection_highlight(menu_layer, up, selection_animated,
                                            true /* change_ongoing_animation */);
  prv_menu_layer_update_selection_scroll_position(menu_layer, MenuRowAlignNone, selection_animated);

  static const PropertyAnimationImplementation s_center_focus_selection_animation_in_out_impl = {
    .base = {
      .setup = prv_center_focus_animation_setup,
      .update = prv_center_focus_animation_update_in_and_out,
      .teardown = prv_center_focus_animation_teardown,
    }
  };
  static const PropertyAnimationImplementation s_center_focus_selection_animation_out_only_impl = {
    .base = {
      .setup = prv_center_focus_animation_setup,
      .update = prv_center_focus_animation_update_out_only,
      .teardown = prv_center_focus_animation_teardown,
    }
  };
  // when we were animating already, use the implementation that's only showing the bounce back
  const PropertyAnimationImplementation *impl = was_animating ?
                                                &s_center_focus_selection_animation_out_only_impl :
                                                &s_center_focus_selection_animation_in_out_impl;
  PropertyAnimation *const prop_anim = property_animation_create(impl, menu_layer, NULL, NULL);
  // we're (ab)using the .to value to store the direction, see prv_center_focus_animation_state()
  property_animation_to(prop_anim, &up, sizeof(up), true);
  Animation *const anim = property_animation_get_animation(prop_anim);
  menu_layer->animation.animation = anim;

  // number of frames measured in the video
  const uint32_t full_duration_ms = ANIMATION_TARGET_FRAME_INTERVAL_MS * 7;
  uint32_t duration = full_duration_ms;
  if (was_animating) {
    // only show second half of animation if uses presses repetitive
    // as it's only the bounce back, then
    duration /= 2;
    animation_set_delay(anim, duration);
  }
  animation_set_duration(anim, duration);
  animation_set_curve(anim, AnimationCurveEaseInOut);
  animation_schedule(anim);

  if (was_animating) {
    // create visual state that's already reflecting the beginning of the "out" animation
    prv_center_focus_animation_update_out_only(anim, 0);
  }
}

static void prv_apply_selection_change(MenuLayer *menu_layer, MenuRowAlign scroll_align, bool up,
                                       bool did_change, const MenuCellSpan *prev_selection,
                                       bool was_animating, bool animated) {
#ifdef CONFIG_TOUCH
  // Single selection choke point for button-nav and programmatic changes: any non-tap selection
  // move invalidates the double-tap window, so a stale recorded row cannot be activated (or, if it
  // was deleted, passed out-of-range to select_click). The tap handler re-arms after this returns.
  menu_layer->double_tap_armed = false;
#endif
  if (menu_layer->center_focused && animated) {
    prv_schedule_center_focus_animation(menu_layer, up, prev_selection, was_animating);
  } else {
    prv_menu_layer_update_selection_highlight(menu_layer, up, animated, true);
    prv_menu_layer_update_selection_scroll_position(menu_layer, scroll_align, animated);

    // only call this here, on animated center focus, the announcement will happen in-between
    // as we change the selection index for real
    if (did_change) {
      prv_announce_selection_changed(menu_layer, prev_selection->index);
    }
  }
}

typedef struct {
  bool was_animating;
  MenuCellSpan prev_selection;
} MenuLayerBeforeSelectionChangeState;

static MenuLayerBeforeSelectionChangeState prv_capture_state_and_cancel_center_focus_animation(
    MenuLayer *menu_layer) {
  // it's critical to cancel the animation for center focus here so that any potential in-between
  // selection state will be cleaned up
  const bool was_animating = menu_layer->center_focused ?
                             prv_cancel_selection_animation(menu_layer) :
                             false;
  return (MenuLayerBeforeSelectionChangeState) {
    .was_animating = was_animating,
    .prev_selection = menu_layer->selection,
  };
}

void menu_layer_set_selected_index(MenuLayer *menu_layer, MenuIndex index, MenuRowAlign scroll_align, bool animated) {
  const MenuLayerBeforeSelectionChangeState before_state =
      prv_capture_state_and_cancel_center_focus_animation(menu_layer);

  // Keep the selection within a valid range
  const uint16_t num_sections = prv_menu_layer_get_num_sections(menu_layer);
  if (index.section >= num_sections) {
    index.section = num_sections - 1;
  }
  // check to make sure this callback has been set, return early if not
  if (menu_layer->callbacks.get_num_rows == NULL) {
    PBL_LOG_ERR("Please set menu layer callbacks before running menu_layer_set_selected_index.");
    return;
  }

  const uint16_t num_rows = menu_layer->callbacks.get_num_rows(menu_layer, index.section, menu_layer->callback_context);
  if (index.row >= num_rows) {
    index.row = num_rows - 1;
  }

  // when called from iteration triggered by menu_layer_set_selected_next() the
  // selection.index.section could be MENU_INDEX_NOT_FOUND (a very large value)
  // in this case, walk forward from {0, 0| to avoid a very long loop run
  const bool is_invalid_section = menu_layer->selection.index.section == MENU_INDEX_NOT_FOUND;
  const int16_t comp = is_invalid_section ? 1 :
                       menu_index_compare(&index, &menu_layer->selection.index);
  MenuSelectIndexIterator it = {
    .it = {
      .menu_layer = menu_layer,
      .row_callback_after_geometry = prv_menu_layer_iterator_selection_index_callback,
      .section_callback = prv_menu_layer_iterator_noop_callback,
      .should_continue = true,
      .cursor = is_invalid_section ? (MenuCellSpan){} : menu_layer->selection,
    },
    .selection = {
      .index = index,
    },
    .did_change_selection = false,
  };

  prv_walk_with_iterator((int8_t)comp, &it.it);

  const bool up = (comp == -1);
  prv_apply_selection_change(menu_layer, scroll_align, up, it.did_change_selection,
                             &before_state.prev_selection, before_state.was_animating, animated);
}

typedef struct MenuSelectNextIterator {
  MenuIterator it;
  uint8_t count;
  bool did_change_selection:1;
} MenuSelectNextIterator;

static void prv_menu_layer_iterator_selection_next_callback(MenuIterator *iterator) {
  MenuSelectNextIterator *it = (MenuSelectNextIterator*)iterator;
  MenuLayer *menu_layer = it->it.menu_layer;
  if (it->count == 1) {
    MenuLayerSelectionWillChangeCallback cb = menu_layer->callbacks.selection_will_change;
    it->it.should_continue = false;
    it->did_change_selection = true;
    if (cb) {
      MenuIndex new_index = it->it.cursor.index;
      cb(menu_layer, &new_index, menu_layer->selection.index, menu_layer->callback_context);
      if (menu_index_compare(&new_index, &menu_layer->selection.index) == 0) {
        // locked into old index
      } else if (menu_index_compare(&new_index, &it->it.cursor.index) == 0) {
        // new index is the index we wanted to select
        menu_layer->selection = it->it.cursor;
      } else {
        // when center focused, animation will be scheduled at the very end
        // see prv_apply_selection_change()
        const bool animated = !menu_layer->center_focused;
        // Specified an alternate index
        // This is safe since menu_layer_set_selected_index will not trigger the
        // SelectionWillChangeCallback again.
        menu_layer_set_selected_index(menu_layer, new_index, MenuRowAlignNone, animated);
        it->did_change_selection = false;
      }
    } else {
      menu_layer->selection = it->it.cursor;
    }
  } else {
    ++it->count;
  }
}

void menu_layer_set_selected_next(MenuLayer *menu_layer, bool up,
                                  MenuRowAlign scroll_align, bool animated) {
  const MenuLayerBeforeSelectionChangeState before_state =
      prv_capture_state_and_cancel_center_focus_animation(menu_layer);

  MenuSelectNextIterator it = {
    .it = {
      .menu_layer = menu_layer,
      .row_callback_after_geometry = prv_menu_layer_iterator_selection_next_callback,
      .section_callback = prv_menu_layer_iterator_noop_callback,
      .should_continue = true,
      .cursor = menu_layer->selection,
    },
    .count = up ? 1 : 0, // see asymmetry note with menu_layer_walk_downward_from_iterator()
    .did_change_selection = false,
  };

  prv_walk_with_iterator((int8_t)(up ? -1 : 1), &it.it);

  prv_apply_selection_change(menu_layer, scroll_align, up, it.did_change_selection,
                             &before_state.prev_selection, before_state.was_animating, animated);
}

MenuIndex menu_layer_get_selected_index(const MenuLayer *menu_layer) {
  return menu_layer->selection.index;
}

bool menu_layer_is_index_selected(const MenuLayer *menu_layer, MenuIndex *index) {
  MenuIndex selected_index = menu_layer_get_selected_index(menu_layer);
  return menu_index_compare(&selected_index, index) == 0;
}

//! indicates that the data behind the menu has changed and needs a re-draw
void menu_layer_reload_data(MenuLayer *menu_layer) {
#ifdef CONFIG_TOUCH
  // A reload may delete the recorded row, so disarm the double-tap window to avoid activating a
  // now out-of-range index.
  menu_layer->double_tap_armed = false;
#endif
  menu_layer_update_caches(menu_layer);
}

bool menu_cell_layer_is_highlighted(const Layer *cell_layer) {
  return cell_layer->is_highlighted;
}

void menu_layer_set_normal_colors(MenuLayer *menu_layer, GColor background, GColor foreground) {
  menu_layer->normal_colors[MenuLayerColorBackground] = background;
  menu_layer->normal_colors[MenuLayerColorForeground] = foreground;
}

void menu_layer_set_highlight_colors(MenuLayer *menu_layer, GColor background, GColor foreground) {
  menu_layer->highlight_colors[MenuLayerColorBackground] = background;
  menu_layer->highlight_colors[MenuLayerColorForeground] = foreground;
}

bool menu_layer_get_center_focused(MenuLayer *menu_layer) {
  return menu_layer->center_focused;
}

void menu_layer_set_center_focused(MenuLayer *menu_layer, bool center_focused) {
  if (!menu_layer) {
    return;
  }
  prv_set_center_focused(menu_layer, center_focused);
  menu_layer_update_caches(menu_layer);
}

void menu_layer_set_scrollbar_hidden(MenuLayer *menu_layer, bool hidden) {
  if (!menu_layer) {
    return;
  }
  menu_layer->scrollbar_hidden = hidden;
  if (hidden && menu_layer->scrollbar_visible) {
    prv_scrollbar_cancel_hide_timer(menu_layer);
    menu_layer->scrollbar_visible = false;
    layer_mark_dirty(&menu_layer->scroll_layer.layer);
  }
}

void menu_layer_set_tap_select_only(MenuLayer *menu_layer, bool tap_select_only) {
  if (!menu_layer) {
    return;
  }
  menu_layer->tap_select_only = tap_select_only;
}

bool menu_layer_get_scroll_wrap_around(MenuLayer *menu_layer) {
  return menu_layer->scroll_wrap_around;
}

void menu_layer_set_scroll_wrap_around(MenuLayer *menu_layer, bool scroll_wrap_around) {
  if (!menu_layer) {
    return;
  }
  menu_layer->scroll_wrap_around = scroll_wrap_around;
}

uint8_t menu_layer_get_scroll_vibe_behavior(MenuLayer *menu_layer) {
  if (menu_layer->scroll_vibe_on_blocked) {
    return 2;
  } else if (menu_layer->scroll_vibe_on_wrap_around) {
    return 1;
  } else {
    return 0;
  }
}

void menu_layer_set_scroll_vibe_on_wrap(MenuLayer *menu_layer, bool scroll_vibe_on_wrap) {
  if (!menu_layer) {
    return;
  }

  if (scroll_vibe_on_wrap) {
    menu_layer->scroll_vibe_on_blocked = false;
  }
  menu_layer->scroll_vibe_on_wrap_around = scroll_vibe_on_wrap;
}

void menu_layer_set_scroll_vibe_on_blocked(MenuLayer *menu_layer, bool scroll_vibe_on_blocked) {
  if (!menu_layer) {
    return;
  }
  
  if (scroll_vibe_on_blocked) {
    menu_layer->scroll_vibe_on_wrap_around = false;
  }
  menu_layer->scroll_vibe_on_blocked = scroll_vibe_on_blocked;
}

#ifdef CONFIG_TOUCH
// ---------------------------------------------------------------------------------------------
// Tier-1 touch navigation
//
// A MenuLayer registers itself as a Tier-1 touch widget in menu_layer_init() (never the legacy-2.x
// path, which falls back to the Tier-2 button bridge). The recognizers and gesture state are NOT
// owned per-widget: the unified widget (tap, pan, swipe) set, owned by TouchNavState, drives
// whichever migrated widget the finger lands on through a per-node apply vtable. A MenuLayer
// supplies that vtable (s_menu_touch_nav_ops) at registration; the apply functions below stay the
// public per-menu gesture surface (and the unit-test entry points).
//
// Live scrolling happens on pan Updated. On a plain (non-center-focused) menu the selection is
// frozen for the whole pan and never changes on liftoff — a pan only scrolls. A center-focused
// menu is a carousel instead: the focus stays pinned at the viewport centre, so the row crossing
// the centre becomes the selection live during the pan (through the full selection_will_change
// contract), and liftoff settles the selected row to the exact centre.
//
// Taps differ by shape. A plain menu behaves like a phone list: a single tap on any row both
// selects it (selection_will_change honoured) and activates it — one tap opens the item. A
// carousel keeps the two-step model (a tap on an off-centre row centres it, a tap on the centred
// row activates), bridged by the double-tap window below.

// Two independent taps within this window on the same carousel count as a "double tap" and activate
// the last tap-selected row (there is no dedicated double-tap recognizer). Plain menus activate on
// the first tap, so the window only ever arms on center-focused menus. CALIBRATION: ~300ms is the
// usual comfortable double-tap spacing; tune on hardware if it feels too eager/sluggish.
#define DOUBLE_TAP_WINDOW_MS 300

_Static_assert(sizeof(((MenuLayer *)0)->touch_nav_node) == sizeof(TouchNavWidgetNode),
               "MenuLayer touch_nav_node must match TouchNavWidgetNode layout");

static bool prv_is_app_task(void) {
  return pebble_task_get_current() == PebbleTask_App;
}

static TouchNavState *prv_task_touch_nav_state(void) {
  return prv_is_app_task() ? app_state_get_touch_nav_state() : modal_manager_get_touch_nav_state();
}

// Test seam (declared in menu_layer_private.h under CONFIG_TOUCH). The unified widget set holds no
// per-task singleton to reset; the touch-nav state is owned by TouchNavState, so this is a no-op
// kept for source compatibility with tests that call it in their setup.
void menu_layer_touch_nav_reset_all(void) {
}

bool menu_layer_touch_is_gesture_target(const MenuLayer *menu_layer) {
  const TouchNavState *state = prv_task_touch_nav_state();
  return state && state->latched_target && state->latched_target->widget == menu_layer;
}

// ---------------------------------------------------------------------------------------------
// Coordinate helpers and hit-test

static void prv_menu_touch_offset_bounds(MenuLayer *menu_layer, int16_t *min_y, int16_t *max_y) {
  // Coarse bounds: [min(frame_h - content_h, 0), 0]. With content shorter than the viewport the
  // lower bound is 0 (via the min()). center_focused widens both ends by half a frame so the first
  // and last rows can reach the centre (the scroll layer itself does not clip in that mode).
  const int16_t frame_h = menu_layer->scroll_layer.layer.frame.size.h;
  const int16_t content_h = scroll_layer_get_content_size(&menu_layer->scroll_layer).h;
  *min_y = MIN((int16_t)(frame_h - content_h), (int16_t)0);
  *max_y = 0;
  if (menu_layer->center_focused) {
    const int16_t widen = frame_h / 2;
    *min_y -= widen;
    *max_y += widen;
  }
}

static int16_t prv_menu_touch_clamp_offset_y(MenuLayer *menu_layer, int16_t y) {
  int16_t min_y, max_y;
  prv_menu_touch_offset_bounds(menu_layer, &min_y, &max_y);
  return CLIP(y, min_y, max_y);
}

//! Derive the overscroll highlight stretch from the current (possibly rubber-banded) offset:
//! with the edge row selected, the selection background extends to the viewport edge so the
//! exposed gap reads as the selected cell stretching -- the touch counterpart of the
//! blocked-button selection bounce -- and never paints over a neighbouring row (the neighbours
//! moved away with the content). Purely offset-derived, so the live pan, the release
//! spring-back, and a cancelled gesture all keep it in sync for free.
static void prv_menu_update_overscroll_stretch(MenuLayer *menu_layer) {
  GRect frame = (GRect) {
    .origin = { 0, menu_layer->selection.y },
    .size = { menu_layer->scroll_layer.layer.frame.size.w, menu_layer->selection.h },
  };
  bool stretched = false;
  if (!menu_layer->center_focused && !menu_layer->selection_animation_disabled &&
      !process_manager_compiled_with_legacy2_sdk()) {
    const int16_t frame_h = menu_layer->scroll_layer.layer.frame.size.h;
    const int16_t content_h = scroll_layer_get_content_size(&menu_layer->scroll_layer).h;
    const int16_t min_y = MIN((int16_t)(frame_h - content_h), (int16_t)0);
    const int16_t offset_y = scroll_layer_get_content_offset(&menu_layer->scroll_layer).y;
    if (content_h > frame_h && offset_y > 0 && menu_layer->selection.y == 0) {
      // Pulled past the top with the first cell selected: extend up to the viewport top.
      frame.origin.y = (int16_t)-offset_y;
      frame.size.h += offset_y;
      stretched = true;
    } else if (content_h > frame_h && offset_y < min_y &&
               prv_menu_index_is_last_index(menu_layer, &menu_layer->selection.index)) {
      // Pulled past the bottom with the last cell selected: extend down to the viewport bottom
      // (covering the bottom padding on the way).
      frame.size.h = (int16_t)(frame_h - offset_y) - frame.origin.y;
      stretched = true;
    }
  }
  if (!stretched && !menu_layer->overscroll_stretched) {
    return;  // never disturb the inverter (selection animations own it) unless we stretched it
  }
  if (stretched && !menu_layer->overscroll_stretched) {
    prv_cancel_selection_animation(menu_layer);  // the pull owns the highlight now
  }
  menu_layer->overscroll_stretched = stretched;
  if (!grect_equal(&menu_layer->inverter.layer.frame, &frame)) {
    menu_layer->inverter.layer.frame = frame;
    menu_layer->inverter.layer.bounds.size = frame.size;
    layer_mark_dirty(&menu_layer->inverter.layer);
  }
}

typedef struct MenuHitTestIterator {
  MenuIterator it;
  int16_t target_y;
  bool found;
  MenuIndex found_index;
} MenuHitTestIterator;

static void prv_menu_hit_test_row_callback(MenuIterator *iterator) {
  MenuHitTestIterator *it = (MenuHitTestIterator *)iterator;
  const int16_t top = it->it.cursor.y;
  const int16_t bottom = top + it->it.cursor.h;
  if (it->target_y >= top && it->target_y < bottom) {
    it->found = true;
    it->found_index = it->it.cursor.index;
    it->it.should_continue = false;
  }
}

static void prv_menu_hit_test_section_callback(MenuIterator *iterator) {
  // Section headers are not selectable; nothing to do.
  (void)iterator;
}

bool menu_layer_touch_find_row_at_content_y(MenuLayer *menu_layer, int16_t content_y,
                                            MenuIndex *index_out) {
  // Walk downward from the render anchor, then upward, mirroring menu_layer_update_proc so the same
  // section-header/separator geometry is honoured. The downward walk includes the anchor row; the
  // upward walk covers everything above it.
  MenuHitTestIterator it = {
    .it = {
      .menu_layer = menu_layer,
      .cursor = menu_layer->cache.cursor,
      .row_callback_after_geometry = prv_menu_hit_test_row_callback,
      .section_callback = prv_menu_hit_test_section_callback,
      .should_continue = true,
    },
    .target_y = content_y,
    .found = false,
  };
  prv_menu_layer_walk_downward_from_iterator(&it.it);
  if (!it.found) {
    it.it.cursor = menu_layer->cache.cursor;
    it.it.should_continue = true;
    prv_menu_layer_walk_upward_from_iterator(&it.it);
  }
  if (it.found && index_out) {
    *index_out = it.found_index;
  }
  return it.found;
}

// ---------------------------------------------------------------------------------------------
// Selection commit through the will_change contract

static void prv_menu_activate_index(MenuLayer *menu_layer, MenuIndex *index) {
  if (menu_layer->callbacks.select_click) {
    menu_layer->callbacks.select_click(menu_layer, index, menu_layer->callback_context);
  }
}

static void prv_menu_activate_selected(MenuLayer *menu_layer) {
  prv_menu_activate_index(menu_layer, &menu_layer->selection.index);
}

// Reassemble/store the split-half tap timestamp (see the field comment in menu_layer.h).
static RtcTicks prv_menu_get_last_select_ticks(const MenuLayer *menu_layer) {
  return ((RtcTicks)menu_layer->last_select_ticks_hi << 32) | menu_layer->last_select_ticks_lo;
}

static void prv_menu_set_last_select_ticks(MenuLayer *menu_layer, RtcTicks ticks) {
  menu_layer->last_select_ticks_hi = (uint32_t)(ticks >> 32);
  menu_layer->last_select_ticks_lo = (uint32_t)ticks;
}

// Run the client's selection_will_change for \a candidate (relative to the current selection) and
// return the resolved final index. The client may veto (leave it at the old index) or redirect it.
static MenuIndex prv_menu_run_will_change(MenuLayer *menu_layer, MenuIndex candidate) {
  MenuIndex final = candidate;
  if (menu_layer->callbacks.selection_will_change) {
    menu_layer->callbacks.selection_will_change(menu_layer, &final, menu_layer->selection.index,
                                                menu_layer->callback_context);
  }
  return final;
}

// ---------------------------------------------------------------------------------------------
// Center-focused (carousel) pan: selection tracks the viewport centre

// Move the selection to \a index WITHOUT repositioning the scroll — the finger owns the offset.
// menu_layer_set_selected_index cannot be used mid-pan: on a center-focused menu it always snaps
// the offset so the new selection is centred (prv_corrected_scroll_align), which would fight the
// finger on every crossing.
static void prv_menu_touch_reselect_row(MenuLayer *menu_layer, MenuIndex index) {
  const MenuCellSpan prev_selection = menu_layer->selection;
  const int16_t comp = menu_index_compare(&index, &menu_layer->selection.index);
  if (comp == 0) {
    return;
  }
  MenuSelectIndexIterator it = {
    .it = {
      .menu_layer = menu_layer,
      .row_callback_after_geometry = prv_menu_layer_iterator_selection_index_callback,
      .section_callback = prv_menu_layer_iterator_noop_callback,
      .should_continue = true,
      .cursor = menu_layer->selection,
    },
    .selection = {
      .index = index,
    },
    .did_change_selection = false,
  };
  prv_walk_with_iterator((int8_t)comp, &it.it);
  if (!it.did_change_selection) {
    return;
  }
  // Any selection move invalidates the double-tap window (same rule as prv_apply_selection_change).
  menu_layer->double_tap_armed = false;
  const bool up = (comp < 0);
  // During an inertial coast the scroll animation IS the ongoing motion: change_ongoing_animation
  // would unschedule it on the first row crossing and kill the coast dead. During a finger pan no
  // scroll animation is scheduled, so the flag is inert either way.
  prv_menu_layer_update_selection_highlight(menu_layer, up, false /* animated */,
                                            !menu_layer->touch_fling_active);
  prv_announce_selection_changed(menu_layer, prev_selection.index);
}

// Adopt the row under the viewport centre as the selection, through the will_change contract: a
// veto keeps the old row focused (the content still scrolls), a redirect focuses the redirected
// row. The newly focused row inflates toward the centre (its height reflow is absorbed by its
// centre-facing edge), so the content under the finger does not jump; the inflation also gives the
// boundary natural hysteresis.
static void prv_menu_touch_track_center_row(MenuLayer *menu_layer) {
  const int16_t frame_h = menu_layer->scroll_layer.layer.frame.size.h;
  const int16_t offset_y = scroll_layer_get_content_offset(&menu_layer->scroll_layer).y;
  MenuIndex candidate;
  if (!menu_layer_touch_find_row_at_content_y(menu_layer, frame_h / 2 - offset_y, &candidate)) {
    // Centre is past the content ends (the clamp widens by half a frame) or on a header/gap: keep
    // the current focus.
    return;
  }
  if (menu_index_compare(&candidate, &menu_layer->selection.index) == 0) {
    return;
  }
  prv_menu_touch_reselect_row(menu_layer, prv_menu_run_will_change(menu_layer, candidate));
}

// Scroll so the current selection sits exactly at the viewport centre. Mirrors the
// MenuRowAlignCenter case of prv_menu_layer_update_selection_scroll_position, which cannot be used
// here because it forces animated=false on center-focused menus.
static void prv_menu_touch_settle_to_center(MenuLayer *menu_layer, bool animated) {
  const int16_t frame_h = menu_layer->scroll_layer.layer.frame.size.h;
  const int16_t y =
      (int16_t)(frame_h / 2 - menu_layer->selection.y - menu_layer->selection.h / 2);
  scroll_layer_set_content_offset(&menu_layer->scroll_layer, GPoint(0, y), animated);
}

// ---------------------------------------------------------------------------------------------
// Gesture handlers (also the unit-test entry surface)

void menu_layer_touch_handle_touchdown(MenuLayer *menu_layer) {
  // Finger down while the content is coasting: catch it. The tap this gesture may become is then a
  // stop, not a select. The flag is ASSIGNED (never OR-ed) so it always reflects this gesture: a
  // caught-then-abandoned gesture's leftover is overwritten by the next Touchdown.
  const bool coasting = menu_layer->touch_fling_active;
  menu_layer->touch_tap_swallow = coasting;
  if (!coasting) {
    return;
  }
  Animation *anim = property_animation_get_animation(menu_layer->scroll_layer.animation);
  if (anim && animation_is_scheduled(anim)) {
    animation_unschedule(anim);
  }
  menu_layer->touch_fling_active = false;   // a coast caught pre-first-frame skips its handler
  scroll_layer_touch_fling_cleanup(&menu_layer->scroll_layer);
  if (menu_layer->center_focused) {
    // A dead stop could freeze the carousel off-grid forever on a catch-and-hold (a >300 ms hold
    // fails the tap, so no later callback would re-centre). Glide into the nearest row centre
    // instead; a pan taking over unschedules this settle with the base latched mid-flight.
    prv_menu_touch_track_center_row(menu_layer);
    prv_menu_touch_settle_to_center(menu_layer, true /* animated */);
  }
}

void menu_layer_touch_handle_pan_update(MenuLayer *menu_layer, GPoint base,
                                        GPoint delta_since_start) {
  int16_t min_y, max_y;
  prv_menu_touch_offset_bounds(menu_layer, &min_y, &max_y);
  const int32_t raw_y = (int32_t)base.y + delta_since_start.y;
  const int16_t frame_h = menu_layer->scroll_layer.layer.frame.size.h;
  // Past an edge the pan rubber-bands: the excess is damped instead of hard-clamped. Content
  // that fits the frame keeps the hard clamp (nothing to scroll, nothing to bounce).
  const bool scrollable = scroll_layer_get_content_size(&menu_layer->scroll_layer).h > frame_h;
  const int16_t new_y = scrollable
      ? scroll_layer_touch_overscroll_damp(raw_y, min_y, max_y, frame_h)
      : (int16_t)CLIP(raw_y, min_y, max_y);
  prv_scrollbar_kick(menu_layer);
  scroll_layer_touch_set_content_offset_overscrolled(&menu_layer->scroll_layer, new_y);
  if (menu_layer->center_focused) {
    prv_menu_touch_track_center_row(menu_layer);
  }
  // Plain menus: selection intentionally NOT touched — a pan scrolls, a tap selects.
}

static void prv_menu_touch_spring_back_stopped(Animation *animation, bool finished,
                                               void *context) {
  (void)animation;
  (void)finished;
  scroll_layer_touch_fling_cleanup(&((MenuLayer *)context)->scroll_layer);
}

static void prv_menu_touch_fling_stopped(Animation *animation, bool finished, void *context) {
  (void)animation;
  MenuLayer *menu_layer = context;
  menu_layer->touch_fling_active = false;
  // Restore the shared animation defaults first so a settle scheduled below runs with them.
  scroll_layer_touch_fling_cleanup(&menu_layer->scroll_layer);
  if (finished && menu_layer->center_focused) {
    // The coast ended off-grid: adopt the row under the centre and glide it to the exact centre
    // (this also reconciles a mid-coast veto). A not-finished stop means something else took over
    // the offset -- whoever unscheduled owns the settle.
    prv_menu_touch_track_center_row(menu_layer);
    prv_menu_touch_settle_to_center(menu_layer, true /* animated */);
  }
}

// Coast toward the velocity projection, clamped by the menu's own (possibly widened) clamp.
static void prv_menu_touch_fling(MenuLayer *menu_layer, int16_t released_y, int32_t velocity_y) {
  const int32_t projected_y = released_y + (velocity_y * TOUCH_FLING_TAU_MS) / 1000;
  const int16_t target_y = prv_menu_touch_clamp_offset_y(
      menu_layer, (int16_t)CLIP(projected_y, INT16_MIN, INT16_MAX));
  if (scroll_layer_touch_fling_start(&menu_layer->scroll_layer, target_y, (int16_t)velocity_y,
                                     prv_menu_touch_fling_stopped, menu_layer)) {
    menu_layer->touch_fling_active = true;
  }
}

void menu_layer_touch_handle_snap(MenuLayer *menu_layer, GPoint base, GPoint final_delta,
                                  GPoint velocity) {
  const int16_t new_y = prv_menu_touch_clamp_offset_y(menu_layer, base.y + final_delta.y);
  prv_scrollbar_kick(menu_layer);
  const int16_t current_y = scroll_layer_get_content_offset(&menu_layer->scroll_layer).y;
  if (!menu_layer->center_focused &&
      current_y != prv_menu_touch_clamp_offset_y(menu_layer, current_y)) {
    // Rubber-banded past an edge: glide back to it regardless of velocity (a coast from out of
    // bounds would be clipped to the edge on its first frame by the standard animation setter).
    // Center-focused menus don't take this path: their settle-to-center below glides them back.
    scroll_layer_touch_overscroll_spring_back(&menu_layer->scroll_layer, new_y,
                                              prv_menu_touch_spring_back_stopped, menu_layer);
    return;
  }
  const int32_t v = CLIP((int32_t)velocity.y, -TOUCH_FLING_MAX_VELOCITY_PX_S,
                         TOUCH_FLING_MAX_VELOCITY_PX_S);
  if (ABS(v) >= TOUCH_FLING_MIN_VELOCITY_PX_S) {
    // Fast liftoff: coast toward the projection from the released offset. The coast starts from
    // the current (last throttled) offset, so the unthrottled residual is absorbed into the
    // animation instead of jumping instantly at liftoff. Plain menus coast the content only (the
    // selection is intentionally left where it was — a pan scrolls, a tap selects); on a carousel
    // rows step through the centre live via the offset-changed handler, and the coast's stopped
    // handler settles the final row to the exact centre.
    prv_menu_touch_fling(menu_layer, new_y, v);
    if (menu_layer->touch_fling_active) {
      return;
    }
  }
  // Slow liftoff (or a coast too short to schedule): settle the final (unthrottled) offset.
  scroll_layer_set_content_offset(&menu_layer->scroll_layer, GPoint(0, new_y), false);
  if (!menu_layer->center_focused) {
    return;
  }
  // Carousel: the final delta may have crossed one more boundary than the last throttled pan
  // update saw, so re-track, then glide the focused row into the exact centre.
  prv_menu_touch_track_center_row(menu_layer);
  prv_menu_touch_settle_to_center(menu_layer, true /* animated */);
}

void menu_layer_touch_handle_cancel(MenuLayer *menu_layer) {
  if (!menu_layer->center_focused) {
    // The selection never moved during the pan and the content is already settled; only a
    // rubber-banded offset needs restoring -- instantly, since a cancel is a handover that must
    // not leave an animation running.
    const int16_t y = scroll_layer_get_content_offset(&menu_layer->scroll_layer).y;
    const int16_t clamped_y = prv_menu_touch_clamp_offset_y(menu_layer, y);
    if (y != clamped_y) {
      scroll_layer_set_content_offset(&menu_layer->scroll_layer, GPoint(0, clamped_y), false);
    }
    return;
  }
  // A cancel means something else took over (gesture steal, window change): leave the carousel in
  // a stable state immediately, without an animation that could outlive the handover.
  prv_menu_touch_settle_to_center(menu_layer, false /* animated */);
}

void menu_layer_touch_handle_tap(MenuLayer *menu_layer, GPoint point_on_screen) {
  if (menu_layer->touch_tap_swallow) {
    // This tap caught (stopped) a coast at its Touchdown: it is a stop, not a select.
    menu_layer->touch_tap_swallow = false;
    return;
  }
  // The recognizer reports the tap in screen coordinates; map it into the scroll layer's frame
  // first. Skipping this mis-hits by the frame's screen offset, so a menu inset below the status
  // bar resolves the bottom of every row to the next row down.
  GRect scroll_frame;
  layer_get_global_frame(&menu_layer->scroll_layer.layer, &scroll_frame);
  const int16_t frame_y = point_on_screen.y - scroll_frame.origin.y;
  const int16_t offset_y = scroll_layer_get_content_offset(&menu_layer->scroll_layer).y;
  const int16_t content_y = frame_y - offset_y;

  MenuIndex candidate;
  if (!menu_layer_touch_find_row_at_content_y(menu_layer, content_y, &candidate)) {
    // Tap landed on a header/gap: nothing to select or activate, and the double-tap window is left
    // untouched (a stray tap on a gap must not arm or spend it).
    return;
  }

  const RtcTicks now = sys_get_ticks();
  const RtcTicks window_ticks = (RtcTicks)DOUBLE_TAP_WINDOW_MS * RTC_TICKS_HZ / 1000;

  // Priority 1 — fast double tap: a second tap within the window after a tap-select activates the
  // row that select committed, WITHOUT re-hit-testing. The just-selected row may have animated to
  // the centre, so the current tap can hit-test a neighbour; the recorded index is the source of
  // truth. Guarded by an explicit arm flag so it never fires on the first tap.
  if (menu_layer->double_tap_armed &&
      (RtcTicks)(now - prv_menu_get_last_select_ticks(menu_layer)) <= window_ticks) {
    prv_cancel_selection_animation(menu_layer);
    prv_menu_activate_index(menu_layer, &menu_layer->last_selected_index);
    menu_layer->double_tap_armed = false;  // disarm so a third tap does not re-fire
    return;
  }

  // Priority 2 — deliberate second tap on the already-centred selection activates it.
  if (menu_index_compare(&candidate, &menu_layer->selection.index) == 0) {
    prv_cancel_selection_animation(menu_layer);
    prv_menu_activate_selected(menu_layer);
    menu_layer->double_tap_armed = false;
    return;
  }

  // Priority 3 — the tapped row is not the selection: run the full will_change contract
  // (veto/redirect honoured). A veto (final == old selection) must change nothing: no select, no
  // activate, no re-centre of the old selection (that mini-snap is exactly what Fix 1 removed from
  // pans), no animation cancel, and the double-tap window stays as it was — cancelling would abort
  // an in-flight highlight animation and violate "veto changes nothing", so the cancel lives past
  // the veto check.
  const MenuIndex old_index = menu_layer->selection.index;
  const MenuIndex final = prv_menu_run_will_change(menu_layer, candidate);
  if (menu_index_compare(&final, &old_index) == 0) {
    return;
  }
  prv_cancel_selection_animation(menu_layer);

  if (!menu_layer->center_focused) {
    // Plain menus behave like a phone list: the tap selects AND opens the row in one gesture.
    // Commit without scrolling — the row is already visible under the finger, and a re-centre
    // would visibly shift the content just as the activated window pushes. Activate only when the
    // contract kept the tapped row: a redirect selects the redirected row without opening it
    // (opening a row the finger never touched would be a misfire), so there activation stays a
    // deliberate second tap (priority 2). tap_select_only menus (short-item action menus, whose
    // rows hold several columns the row-granular hit-test cannot tell apart) keep that two-step
    // model for every row.
    menu_layer_set_selected_index(menu_layer, final, MenuRowAlignNone, false);
    if (!menu_layer->tap_select_only &&
        menu_index_compare(&menu_layer->selection.index, &candidate) == 0) {
      prv_menu_activate_selected(menu_layer);
    }
    return;
  }

  // Carousel: select the tapped row and centre it WITHOUT activating — the same jump/bounce as a
  // button step (the nudge-in, half-way snap and bounce-out of
  // prv_schedule_center_focus_animation); opening is the second tap (or the double-tap window).
  menu_layer_set_selected_index(menu_layer, final, MenuRowAlignCenter, true);
  // Record the committed (range-clamped) selection, not the raw will_change output: a client that
  // redirects to an out-of-range index would otherwise be handed that OOB index by a fast double tap
  // (priority 1), diverging from priority 2 which activates the clamped selection.index. The index
  // commit is deferred to the half-way point of the jump animation; until then the committed
  // target lives in animation.new_selection (a priority-1 tap during the jump cancels it, which
  // drives the animation to its end state and commits that same target).
  const bool jump_in_flight = animation_is_scheduled(menu_layer->animation.animation);
  menu_layer->last_selected_index = jump_in_flight ? menu_layer->animation.new_selection.index
                                                   : menu_layer->selection.index;
  prv_menu_set_last_select_ticks(menu_layer, now);
  menu_layer->double_tap_armed = true;
}

// Emit BACK through the bridge ops, mirroring the Tier-2 bridge: pop the window when it has no back
// handler, otherwise synthesise the button. Guarded against a mid-transition drop.
static void prv_menu_touch_emit_back(void) {
  const TouchNavState *state = prv_task_touch_nav_state();
  if (!state || !state->ops) {
    return;
  }
  const TouchNavOps *ops = state->ops;
  if (ops->is_animating && ops->is_animating(ops->ctx)) {
    return;
  }
  if (!(ops->top_overrides_back && ops->top_overrides_back(ops->ctx))) {
    if (ops->pop_top) {
      ops->pop_top(ops->ctx);
    }
  } else if (ops->emit_button) {
    ops->emit_button(ops->ctx, BUTTON_ID_BACK);
  }
}

void menu_layer_touch_handle_swipe(MenuLayer *menu_layer, SwipeDirection direction) {
  // Only the back swipe is honored: right = BACK. Activate-on-swipe (left) is intentionally
  // disabled, so a menu swipe can never trigger the selected item.
  switch (direction) {
    case SwipeDirection_Right:
      prv_menu_touch_emit_back();
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------------------------
// Unified widget-set ops

// Thin void*->MenuLayer* wrappers over the public apply functions. The unified widget set (owned by
// TouchNavState) drives the menu through these; direct assignment of the apply functions would not
// compile because their first parameter is MenuLayer*, not void*.

static void prv_menu_ops_touchdown(void *w) {
  menu_layer_touch_handle_touchdown((MenuLayer *)w);
}

static void prv_menu_ops_pan_started(void *w) {
  // Touchdown-equivalent for the pan: stop any in-flight selection animation so the finger takes
  // over. The base offset is latched by the core via get_base_offset immediately after.
  MenuLayer *menu_layer = w;
  prv_cancel_selection_animation(menu_layer);
  // Also stop a running settle/coast on the scroll offset itself; otherwise it and the first
  // throttled pan update both write the offset for up to one frame. A coast caught before its
  // first frame never ran its stopped handler, so clear its state explicitly.
  Animation *anim = property_animation_get_animation(menu_layer->scroll_layer.animation);
  if (anim && animation_is_scheduled(anim)) {
    animation_unschedule(anim);
  }
  menu_layer->touch_fling_active = false;
  scroll_layer_touch_fling_cleanup(&menu_layer->scroll_layer);
}

static GPointReturn prv_menu_ops_get_base_offset(void *w) {
  return scroll_layer_get_content_offset(&((MenuLayer *)w)->scroll_layer);
}

static void prv_menu_ops_pan_update(void *w, GPoint base, GPoint delta) {
  menu_layer_touch_handle_pan_update((MenuLayer *)w, base, delta);
}

static void prv_menu_ops_pan_snap(void *w, GPoint base, GPoint final_delta, GPoint velocity) {
  menu_layer_touch_handle_snap((MenuLayer *)w, base, final_delta, velocity);
}

static void prv_menu_ops_pan_cancel(void *w) {
  menu_layer_touch_handle_cancel((MenuLayer *)w);
}

static void prv_menu_ops_tap(void *w, GPoint point_on_screen) {
  // Keeps the full selection_will_change veto/redirect/double-tap contract.
  menu_layer_touch_handle_tap((MenuLayer *)w, point_on_screen);
}

static void prv_menu_ops_swipe(void *w, SwipeDirection dir) {
  menu_layer_touch_handle_swipe((MenuLayer *)w, dir);
}

// can_start is NULL: a MenuLayer is always ready to start a pan.
static const TouchNavWidgetOps s_menu_touch_nav_ops = {
  .can_start = NULL,
  .touchdown = prv_menu_ops_touchdown,
  .pan_started = prv_menu_ops_pan_started,
  .get_base_offset = prv_menu_ops_get_base_offset,
  .pan_update = prv_menu_ops_pan_update,
  .pan_snap = prv_menu_ops_pan_snap,
  .pan_cancel = prv_menu_ops_pan_cancel,
  .tap = prv_menu_ops_tap,
  .swipe = prv_menu_ops_swipe,
};

static void prv_menu_touch_nav_register(MenuLayer *menu_layer) {
  // The legacy-2.x MenuLayer path is not a Tier-1 widget; it falls back to the Tier-2 bridge.
  if (process_manager_compiled_with_legacy2_sdk()) {
    return;
  }
  TouchNavState *state = prv_task_touch_nav_state();
  if (!state || !state->manager) {
    return;
  }

  // Register the menu as a migrated Tier-1 widget: the unified widget set drives it through
  // s_menu_touch_nav_ops. The registry add dedups by address and re-applies the ops/layer, so a
  // repeated init on the same menu (no intervening deinit) keeps routing to and driving it.
  Layer *layer = menu_layer_get_layer(menu_layer);
  touch_nav_registry_add(state, TouchNavWidgetType_Menu,
                         (TouchNavWidgetNode *)&menu_layer->touch_nav_node, layer,
                         &s_menu_touch_nav_ops, menu_layer);
}

static void prv_menu_touch_nav_deregister(MenuLayer *menu_layer) {
  TouchNavState *state = prv_task_touch_nav_state();
  if (!state) {
    return;
  }
  // If this menu is the live gesture target and is about to be destroyed under a live window, cancel
  // the gesture with NO client callbacks so a snap cannot reach freed client state.
  // touch_nav_registry_remove clears the latched target BEFORE we cancel (its UAF hook), so the
  // resulting Cancelled dispatch finds no target and re-enters nothing.
  const bool was_target = menu_layer_touch_is_gesture_target(menu_layer);
  // Idempotent: removing a node that is not in the registry (double deinit, or a never-registered
  // legacy-2.x menu) is a safe no-op.
  touch_nav_registry_remove(state, TouchNavWidgetType_Menu,
                            (TouchNavWidgetNode *)&menu_layer->touch_nav_node);
  if (was_target && state->manager) {
    recognizer_manager_cancel_and_reset(state->manager);
  }
}
#endif  // CONFIG_TOUCH