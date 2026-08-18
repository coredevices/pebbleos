/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "pbl/services/timeline/swap_layer.h"
#include "pbl/services/timeline/layout_layer.h"

#include "applib/ui/layer.h"
#include "applib/ui/window.h"
#include "applib/ui/property_animation.h"
#include "applib/ui/recognizer/recognizer.h"
#include "applib/ui/recognizer/recognizer_list.h"
#include "applib/ui/recognizer/recognizer_manager.h"
#include "applib/ui/recognizer/touch_nav.h"
#include "applib/ui/recognizer/pan.h"

#include "pbl/drivers/rtc.h"

#include "fake_rtc.h"

// Stubs
#include "stubs_app_state.h"
#include "stubs_click.h"
#include "stubs_logging.h"
#include "stubs_passert.h"
#include "stubs_pebble_tasks.h"
#include "stubs_process_manager.h"
#include "stubs_unobstructed_area.h"

// ---------------------------------------------------------------------------------------------
// Light replacements for the heavy swap_layer.c collaborators that these unit tests never exercise
// (no notification is ever loaded, so the layout callbacks are not driven and the animated settle
// produces no real animation). The animation_* primitives come from tests/fakes/fake_animation.c.

GContext *graphics_context_get_current_context(void) { return NULL; }

GSize layout_get_size(GContext *ctx, LayoutLayer *layout) { return GSizeZero; }
const LayoutColors *layout_get_colors(const LayoutLayer *layout) { return NULL; }
void *layout_get_context(LayoutLayer *layout) { return NULL; }

bool gbitmap_init_with_resource(GBitmap *bitmap, uint32_t resource_id) { return true; }
void gbitmap_deinit(GBitmap *bitmap) {}

void *applib_malloc(size_t bytes) { return NULL; }
void applib_free(void *ptr) {}

int64_t interpolate_moook(int32_t normalized, int64_t from, int64_t to) { return to; }
uint32_t interpolate_moook_duration(void) { return 0; }

// The animated settle/scroll path records how many times it ran, but produces no real animation.
static int s_scroll_animations;
PropertyAnimation *property_animation_create_layer_frame(struct Layer *layer, GRect *from,
                                                         GRect *to) {
  s_scroll_animations++;
  return NULL;
}
bool property_animation_to(PropertyAnimation *property_animation, void *to, size_t size, bool set) {
  return false;
}
bool animation_set_duration(Animation *animation, uint32_t duration) { return true; }
bool animation_set_curve(Animation *animation, AnimationCurve curve) { return true; }
bool animation_set_custom_interpolation(Animation *animation,
                                        InterpolateInt64Function interpolate) { return true; }

// Graphics and click/window-subscribe collaborators pulled in by swap_layer.c's render/click paths,
// none of which these tests drive.
void graphics_context_set_fill_color(GContext *ctx, GColor color) {}
void graphics_context_set_compositing_mode(GContext *ctx, GCompOp mode) {}
void graphics_fill_rect(GContext *ctx, const GRect *rect) {}
void graphics_draw_bitmap_in_rect(GContext *ctx, const GBitmap *bitmap, const GRect *rect) {}
bool graphics_release_frame_buffer(GContext *ctx, GBitmap *buffer) { return true; }

void window_schedule_render(Window *window) {}
void window_set_click_config_provider_with_context(Window *window,
                                                   ClickConfigProvider provider, void *context) {}
void window_raw_click_subscribe(ButtonId button_id, ClickHandler down_handler,
                                ClickHandler up_handler, void *context) {}
void window_single_repeating_click_subscribe(ButtonId button_id, uint16_t repeat_interval_ms,
                                             ClickHandler handler) {}
void window_multi_click_subscribe(ButtonId button_id, uint8_t min_clicks, uint8_t max_clicks,
                                  uint16_t timeout, bool last_click_only, ClickHandler handler) {}

// ---------------------------------------------------------------------------------------------
// Touch-nav harness. swap_layer.c resolves the per-task touch-nav state through these accessors and
// the master gate; the recognizer manager needs a couple of window/layer collaborators to link.

static bool s_nav_enabled = true;
bool sys_touch_nav_enabled(void) { return s_nav_enabled; }
bool sys_touch_app_nav_active(void) { return false; }

static TouchNavState s_touch_nav_state;
struct TouchNavState *app_state_get_touch_nav_state(void) { return &s_touch_nav_state; }
struct TouchNavState *modal_manager_get_touch_nav_state(void) { return &s_touch_nav_state; }

// Counts interaction_handler firings: this is the #1266 auto-close-timer extension. Every touch
// scroll must refresh the timer so a long notification cannot close under the finger mid-read.
static int s_interaction_count;
static void prv_count_interaction(SwapLayer *swap_layer, void *context) { s_interaction_count++; }

static Layer *s_active_layer;
Layer *layer_find_layer_containing_point(const Layer *node, const GPoint *point) {
  return s_active_layer;
}

static Window s_window;
static Layer s_root_layer;
static RecognizerManager s_manager;
static RecognizerList s_global_list;

struct Layer *window_get_root_layer(const Window *window) { return &s_root_layer; }
RecognizerList *window_get_recognizer_list(Window *window) { return NULL; }
RecognizerManager *window_get_recognizer_manager(Window *window) { return &s_manager; }

static int s_emit_count;
static ButtonId s_last_emit;
static int s_pop_count;
static bool s_ops_animating;
static bool s_overrides_back;
static bool prv_ops_is_animating(void *ctx) { return s_ops_animating; }
static bool prv_ops_overrides_back(void *ctx) { return s_overrides_back; }
static void prv_ops_pop_top(void *ctx) { s_pop_count++; }
static void prv_ops_emit_button(void *ctx, ButtonId button) {
  s_emit_count++;
  s_last_emit = button;
}
static TouchNavOps s_ops;

// Bring up a live per-task touch-nav state with a real recognizer manager so a registered SwapLayer
// threads onto a live registry and is driven end to end through touch_nav_dispatch. Unlike the
// pre-Ф3 harness, this now initialises the FULL touch-nav state (touch_nav_state_init): both the
// Tier-2 system set and the unified widget set are attached to the global list, and the unified set
// drives the migrated SwapLayer through its ops vtable.
static void prv_live_state_setup(void) {
  layer_init(&s_root_layer, &GRect(0, 0, 200, 200));
  recognizer_list_init(&s_global_list);
  recognizer_manager_init(&s_manager);
  s_manager.window = &s_window;
  s_manager.global_list = &s_global_list;

  s_ops = (TouchNavOps){
    .is_animating = prv_ops_is_animating,
    .top_overrides_back = prv_ops_overrides_back,
    .pop_top = prv_ops_pop_top,
    .emit_button = prv_ops_emit_button,
  };
  touch_nav_state_init(&s_touch_nav_state, &s_manager, &s_ops);
}

// Drive a touch event through the dispatcher (the unified routing entry point), not the recognizer
// manager directly, so tests exercise the same Touchdown-latch / tier-exclusion path as production.
static void prv_drive(TouchEventType type, int16_t x, int16_t y) {
  const TouchEvent e = { .type = type, .x = x, .y = y, .non_navigational = false };
  touch_nav_dispatch(&e, &s_touch_nav_state);
}

static void prv_advance_ms(uint32_t ms) {
  fake_rtc_increment_ticks((RtcTicks)ms * RTC_TICKS_HZ / 1000);
}

void test_swap_layer_touch__initialize(void) {
  fake_rtc_init(0, 0);
  s_nav_enabled = true;
  s_active_layer = NULL;
  s_scroll_animations = 0;
  s_interaction_count = 0;
  s_emit_count = 0;
  s_last_emit = NUM_BUTTONS;
  s_pop_count = 0;
  s_ops_animating = false;
  s_overrides_back = false;
  // A zeroed state has a NULL manager, so a register call is inert unless a test opts into the live
  // harness via prv_live_state_setup().
  s_touch_nav_state = (TouchNavState){0};
  swap_layer_touch_nav_reset_all();
}

void test_swap_layer_touch__cleanup(void) {
  swap_layer_touch_nav_reset_all();
}

// ---------------------------------------------------------------------------------------------
// A hand-built SwapLayer: a viewport frame plus current/next layouts, without loading a real
// notification (which would need the whole timeline layout stack).

typedef struct FakeSwap {
  SwapLayer swap;
  LayoutLayer current;
  LayoutLayer next;
} FakeSwap;

// viewport_h: the swap layer frame height. content_h: the current notification height. If has_next,
// a next layout is attached so the peek/max-scroll accounting includes it.
static void prv_build_swap(FakeSwap *fs, int16_t viewport_h, int16_t content_h, bool has_next) {
  *fs = (FakeSwap){0};
  layer_init(&fs->swap.layer, &GRect(0, 0, 144, viewport_h));
  // Wire the interaction handler so prv_announce_interaction() is observable (the #1266 timer).
  fs->swap.callbacks.interaction_handler = prv_count_interaction;

  layer_init(&fs->current.layer, &GRect(0, 0, 144, content_h));
  fs->swap.current = &fs->current;

  if (has_next) {
    layer_init(&fs->next.layer, &GRect(0, content_h, 144, viewport_h));
    fs->swap.next = &fs->next;
  }
}

static int16_t prv_offset(const FakeSwap *fs) {
  return -fs->current.layer.frame.origin.y;
}

// =============================================================================================
// The scroll point: 1:1 offset with clamp, and the next-layout peek tracking.
// =============================================================================================

void test_swap_layer_touch__scroll_clamp_top(void) {
  FakeSwap fs;
  prv_build_swap(&fs, 168, 400, false);
  // Already at the top (offset 0). A positive dy scrolls further towards the top: clamp holds it.
  swap_layer_touch_scroll_by(&fs.swap, 50);
  cl_assert_equal_i(prv_offset(&fs), 0);
}

void test_swap_layer_touch__scroll_clamp_bottom(void) {
  FakeSwap fs;
  prv_build_swap(&fs, 168, 400, false);
  const int16_t max_dy = 400 - 168;  // no next => max scroll is content_h - viewport_h
  // Scroll well past the bottom (negative dy scrolls into the content): clamp holds it at max.
  swap_layer_touch_scroll_by(&fs.swap, -1000);
  cl_assert_equal_i(prv_offset(&fs), max_dy);
  // A further push into the content does not move past the clamped edge.
  swap_layer_touch_scroll_by(&fs.swap, -50);
  cl_assert_equal_i(prv_offset(&fs), max_dy);
}

void test_swap_layer_touch__scroll_1to1_and_next_peek(void) {
  FakeSwap fs;
  prv_build_swap(&fs, 168, 400, true /* has_next */);
  // A negative dy scrolls into the content 1:1; the offset moves by exactly |dy|.
  swap_layer_touch_scroll_by(&fs.swap, -30);
  cl_assert_equal_i(prv_offset(&fs), 30);
  // The next layout is pulled right under the current one so its peek tracks the finger.
  cl_assert_equal_i(fs.next.layer.frame.origin.y,
                    fs.current.layer.frame.origin.y + fs.current.layer.frame.size.h);
}

// =============================================================================================
// The two-threshold liftoff decision (DRAG_THRESHOLD_PX = 10, SWAP_OVERPULL_PX = 40).
// =============================================================================================

// max_dy for a mid-length notification used across the liftoff tests.
#define TEST_MAX_DY 200

void test_swap_layer_touch__liftoff_drag_down_settles(void) {
  // Finger down (positive delta) from the middle, within the edges: a normal scroll -> settle.
  const int16_t base_offset = 100;
  cl_assert_equal_i(swap_layer_touch_liftoff_action(base_offset, 30, TEST_MAX_DY),
                    SwapTouchLiftoff_Settle);
}

void test_swap_layer_touch__liftoff_drag_up_settles(void) {
  // Finger up (negative delta) from the middle, within the edges: a normal scroll -> settle.
  const int16_t base_offset = 100;
  cl_assert_equal_i(swap_layer_touch_liftoff_action(base_offset, -30, TEST_MAX_DY),
                    SwapTouchLiftoff_Settle);
}

void test_swap_layer_touch__liftoff_subthreshold_noop(void) {
  // Below DRAG_THRESHOLD_PX (10): not a drag at all, so neither scroll nor swap. This must hold even
  // at the very edge where an over-pull could otherwise trigger.
  cl_assert_equal_i(swap_layer_touch_liftoff_action(0, -9, TEST_MAX_DY), SwapTouchLiftoff_None);
  cl_assert_equal_i(swap_layer_touch_liftoff_action(0, 9, TEST_MAX_DY), SwapTouchLiftoff_None);
  // A sub-threshold pull at the top, far enough to clear SWAP_OVERPULL_PX in magnitude were the drag
  // threshold neutered, still stays a no-op because the drag threshold gates first.
  cl_assert_equal_i(swap_layer_touch_liftoff_action(0, 5, TEST_MAX_DY), SwapTouchLiftoff_None);
  // At EXACTLY the threshold it IS a drag (pins the < vs <= edge that the below-threshold cases
  // leave open): requested = 0 - 10 = -10, within range, so it settles rather than no-ops.
  cl_assert_equal_i(swap_layer_touch_liftoff_action(0, 10, TEST_MAX_DY), SwapTouchLiftoff_Settle);
}

void test_swap_layer_touch__liftoff_overpull_prev(void) {
  // At the top (offset 0), finger down far enough that requested_offset < -SWAP_OVERPULL_PX.
  // requested_offset = base_offset - delta_y = 0 - 50 = -50 < -40 -> swap to previous.
  cl_assert_equal_i(swap_layer_touch_liftoff_action(0, 50, TEST_MAX_DY),
                    SwapTouchLiftoff_SwapPrev);
}

void test_swap_layer_touch__liftoff_overpull_next(void) {
  // At the bottom (offset == max_dy), finger up far enough that
  // requested_offset > max_dy + SWAP_OVERPULL_PX. requested_offset = 200 - (-50) = 250 > 240.
  cl_assert_equal_i(swap_layer_touch_liftoff_action(TEST_MAX_DY, -50, TEST_MAX_DY),
                    SwapTouchLiftoff_SwapNext);
}

void test_swap_layer_touch__liftoff_reaching_edge_without_overpull_no_swap(void) {
  // Reaching the bottom edge WITHOUT over-pulling past SWAP_OVERPULL_PX must NOT swap: a plain
  // scroll to (and a touch beyond) the edge settles. requested_offset = 200 - (-40) = 240, which is
  // exactly max_dy + SWAP_OVERPULL_PX and therefore not strictly greater -> settle, not swap.
  cl_assert_equal_i(swap_layer_touch_liftoff_action(TEST_MAX_DY, -40, TEST_MAX_DY),
                    SwapTouchLiftoff_Settle);
  // Symmetrically at the top: requested_offset = 0 - 40 = -40 == -SWAP_OVERPULL_PX, not strictly
  // less -> settle.
  cl_assert_equal_i(swap_layer_touch_liftoff_action(0, 40, TEST_MAX_DY), SwapTouchLiftoff_Settle);
  // Just PAST the threshold DOES swap (pins the just-past side the -50/250 cases leave open):
  // requested = 0 - 41 = -41 < -40; and 200 - (-41) = 241 > 240.
  cl_assert_equal_i(swap_layer_touch_liftoff_action(0, 41, TEST_MAX_DY), SwapTouchLiftoff_SwapPrev);
  cl_assert_equal_i(swap_layer_touch_liftoff_action(TEST_MAX_DY, -41, TEST_MAX_DY),
                    SwapTouchLiftoff_SwapNext);
}

// =============================================================================================
// The settle target (page quantization for round text flow).
// =============================================================================================

void test_swap_layer_touch__settle_offset_free_when_no_page(void) {
  // page_h = 0 (rect): the settle target is the plain clamped offset.
  cl_assert_equal_i(swap_layer_touch_settle_offset(100, 30, TEST_MAX_DY, 0), 70);
  cl_assert_equal_i(swap_layer_touch_settle_offset(100, -30, TEST_MAX_DY, 0), 130);
  // Clamped at both edges.
  cl_assert_equal_i(swap_layer_touch_settle_offset(0, 30, TEST_MAX_DY, 0), 0);
  cl_assert_equal_i(swap_layer_touch_settle_offset(TEST_MAX_DY, -30, TEST_MAX_DY, 0), TEST_MAX_DY);
}

void test_swap_layer_touch__settle_offset_quantizes_to_pages(void) {
  // page_h = 100 (round): the settle target lands on the nearest page boundary, so the circular
  // text flow — computed for page-aligned rest positions — matches the content again.
  const int16_t page = 100;
  // Below half a page rounds back to the current page; at/above half advances.
  cl_assert_equal_i(swap_layer_touch_settle_offset(0, -49, TEST_MAX_DY, page), 0);
  cl_assert_equal_i(swap_layer_touch_settle_offset(0, -50, TEST_MAX_DY, page), 100);
  cl_assert_equal_i(swap_layer_touch_settle_offset(0, -149, TEST_MAX_DY, page), 100);
  cl_assert_equal_i(swap_layer_touch_settle_offset(0, -150, TEST_MAX_DY, page), 200);
  // Quantization applies from any base, not only page-aligned ones (a cancelled pan settles from
  // wherever the finger left the content).
  cl_assert_equal_i(swap_layer_touch_settle_offset(130, 0, TEST_MAX_DY, page), 100);
  cl_assert_equal_i(swap_layer_touch_settle_offset(170, 0, TEST_MAX_DY, page), 200);
}

void test_swap_layer_touch__settle_offset_never_rests_in_peek(void) {
  // max_dy may include the next-notification peek (not a page multiple). A target in the peek zone
  // must land on the last whole page below max_dy, never at the off-grid clamped maximum.
  const int16_t page = 100;
  const int16_t max_dy_with_peek = 230;  // two whole pages + 30px peek
  cl_assert_equal_i(swap_layer_touch_settle_offset(200, -25, max_dy_with_peek, page), 200);
  // Even the hardest pull that still settles (just short of the over-pull swap threshold, clamped
  // to max_dy and past the nearest-page midpoint) stays on the last page.
  cl_assert_equal_i(swap_layer_touch_settle_offset(200, -70, max_dy_with_peek, page), 200);
}

// =============================================================================================
// Registration lifecycle.
// =============================================================================================

void test_swap_layer_touch__touch_disabled_not_registered(void) {
  // The default (zeroed) state has a NULL manager: touch is unavailable, so a register is inert and
  // the widget stays on the button path.
  FakeSwap fs;
  prv_build_swap(&fs, 168, 400, false);
  swap_layer_touch_register(&fs.swap);
  cl_assert(s_touch_nav_state.swap_head == NULL);
  cl_assert(!fs.swap.touch_registered);
}

void test_swap_layer_touch__registered_and_deregistered(void) {
  prv_live_state_setup();
  FakeSwap fs;
  prv_build_swap(&fs, 168, 400, false);

  swap_layer_touch_register(&fs.swap);
  cl_assert(s_touch_nav_state.swap_head != NULL);
  cl_assert(fs.swap.touch_registered);

  // Re-registering is idempotent: no duplicate node, no second recognizer set.
  swap_layer_touch_register(&fs.swap);
  cl_assert(s_touch_nav_state.swap_head->next == NULL);

  swap_layer_touch_deregister(&fs.swap);
  cl_assert(s_touch_nav_state.swap_head == NULL);
  cl_assert(!fs.swap.touch_registered);
}

void test_swap_layer_touch__release_deregisters(void) {
  prv_live_state_setup();
  FakeSwap fs;
  prv_build_swap(&fs, 168, 400, false);
  swap_layer_touch_register(&fs.swap);

  // Covered by a higher modal: release drops the registry membership (which, in the system-slot
  // bridge architecture, is what releases touch — there is no raw subscription to clear).
  swap_layer_touch_release(&fs.swap);
  cl_assert(s_touch_nav_state.swap_head == NULL);
  cl_assert(!fs.swap.touch_registered);
}

// Criterion 2 (the #1266 fix): a touch scroll must refresh the auto-close timer via the interaction
// handler, so a long notification cannot close under the finger mid-read.
void test_swap_layer_touch__scroll_extends_autoclose(void) {
  FakeSwap fs;
  prv_build_swap(&fs, 168, 400, false);
  cl_assert_equal_i(s_interaction_count, 0);
  swap_layer_touch_scroll_by(&fs.swap, -30);
  cl_assert(s_interaction_count > 0);
  const int after_first = s_interaction_count;
  swap_layer_touch_scroll_by(&fs.swap, -30);
  cl_assert(s_interaction_count > after_first);
}

// =============================================================================================
// Recognizer-driven integration THROUGH touch_nav_dispatch: the unified widget set drives the
// migrated SwapLayer via its ops vtable (Touchdown latch, tier-exclusion, throttled live pan).
// =============================================================================================

// Parent the current layout under the swap layer and the swap layer under the window root, point the
// hit-test at it, and register it so the dispatcher resolves it as the migrated widget target.
static void prv_attach_and_register(FakeSwap *fs, int16_t viewport_h, int16_t content_h,
                                    bool has_next) {
  prv_build_swap(fs, viewport_h, content_h, has_next);
  layer_add_child(&fs->swap.layer, &fs->current.layer);
  layer_add_child(&s_root_layer, &fs->swap.layer);
  s_active_layer = &fs->swap.layer;
  swap_layer_touch_register(&fs->swap);
}

void test_swap_layer_touch__dispatch_pan_scrolls_and_post_liftoff_ignored(void) {
  prv_live_state_setup();
  FakeSwap fs;
  prv_attach_and_register(&fs, 168, 400, false);

  // The Touchdown latches the swap layer as the unified gesture target (routing fixed for the
  // gesture). The pan Starts on the first update (latch + base) and live-scrolls on the next.
  prv_drive(TouchEvent_Touchdown, 72, 120);
  cl_assert(swap_layer_touch_is_gesture_target(&fs.swap));
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 72, 90);   // 30px up -> pan Started (base latched)
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 72, 60);   // -> Updated -> live scroll
  const int16_t scrolled_offset = prv_offset(&fs);
  // The content scrolled into view (offset increased from 0) as the finger moved up.
  cl_assert(scrolled_offset > 0);
  // The pan refreshed the auto-close timer while the finger was still down (the #1266 fix).
  cl_assert(s_interaction_count > 0);

  prv_drive(TouchEvent_Liftoff, 0, 0);
  // After liftoff the gesture target is cleared and no button was emulated by the pan.
  cl_assert(!swap_layer_touch_is_gesture_target(&fs.swap));
  cl_assert_equal_i(s_emit_count, 0);

  // A stray position update after liftoff is ignored: the offset does not change further.
  const int16_t after_liftoff = prv_offset(&fs);
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 72, 20);
  cl_assert_equal_i(prv_offset(&fs), after_liftoff);

  swap_layer_touch_deregister(&fs.swap);
}

// CRITICAL (round-2/3 F1): a pan on a swap layer whose current notification is not loaded
// (current == NULL) must NOT dereference it. can_start declines the whole gesture BEFORE
// pan_started / get_base_offset (which read current and would crash), so nothing scrolls, no button
// is emulated, and nothing crashes.
void test_swap_layer_touch__pan_declined_when_no_current(void) {
  prv_live_state_setup();
  FakeSwap fs = {0};
  layer_init(&fs.swap.layer, &GRect(0, 0, 144, 168));
  fs.swap.callbacks.interaction_handler = prv_count_interaction;
  cl_assert(fs.swap.current == NULL);   // no notification loaded yet (registration before layout)
  layer_add_child(&s_root_layer, &fs.swap.layer);
  s_active_layer = &fs.swap.layer;
  swap_layer_touch_register(&fs.swap);

  prv_drive(TouchEvent_Touchdown, 72, 120);
  cl_assert(swap_layer_touch_is_gesture_target(&fs.swap));  // the node is latched...
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 72, 80);   // 40px up -> pan Starts -> can_start() == false
  cl_assert(s_touch_nav_state.declined);          // ...but the gesture is declined, not driven
  cl_assert_equal_i(s_interaction_count, 0);      // pan_started (announce_interaction) never ran
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 72, 40);   // gated by `declined`: drives nothing (no deref)
  prv_drive(TouchEvent_Liftoff, 0, 0);
  cl_assert(!s_touch_nav_state.declined);         // reset for the next gesture
  cl_assert(!swap_layer_touch_is_gesture_target(&fs.swap));
  cl_assert_equal_i(s_emit_count, 0);

  swap_layer_touch_deregister(&fs.swap);
}

// A tap on the notification body is intentionally inert: it must NOT emit SELECT / open the action
// menu. The action menu is reached by a left swipe instead (dispatch_swipe_left_emits_select), so a
// bare tap does nothing (no button emitted, no pop).
void test_swap_layer_touch__dispatch_tap_is_inert(void) {
  prv_live_state_setup();
  FakeSwap fs;
  prv_attach_and_register(&fs, 168, 400, false);

  prv_drive(TouchEvent_Touchdown, 72, 120);
  prv_advance_ms(30);
  prv_drive(TouchEvent_Liftoff, 0, 0);
  cl_assert_equal_i(s_emit_count, 0);
  cl_assert_equal_i(s_pop_count, 0);
  cl_assert(!swap_layer_touch_is_gesture_target(&fs.swap));

  swap_layer_touch_deregister(&fs.swap);
}

// Drive a straight fast flick from (sx, sy) to (ex, ey) through the dispatcher.
static void prv_swipe(int16_t sx, int16_t sy, int16_t ex, int16_t ey) {
  prv_drive(TouchEvent_Touchdown, sx, sy);
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, (int16_t)((sx + ex) / 2), (int16_t)((sy + ey) / 2));
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, ex, ey);
  prv_advance_ms(20);
  prv_drive(TouchEvent_Liftoff, 0, 0);
}

// A horizontal swipe on the notification body navigates, matching the bridge convention:
// right = BACK, which with no back-override pops the window.
void test_swap_layer_touch__dispatch_swipe_right_emits_back(void) {
  prv_live_state_setup();
  s_overrides_back = false;      // no back handler -> BACK pops the window
  FakeSwap fs;
  prv_attach_and_register(&fs, 168, 400, false);

  prv_swipe(20, 120, 90, 120);   // left-to-right = right swipe
  cl_assert_equal_i(s_pop_count, 1);
  cl_assert_equal_i(s_emit_count, 0);

  swap_layer_touch_deregister(&fs.swap);
}

// A horizontal swipe on the notification body navigates, matching the bridge convention:
// left = SELECT.
void test_swap_layer_touch__dispatch_swipe_left_emits_select(void) {
  prv_live_state_setup();
  FakeSwap fs;
  prv_attach_and_register(&fs, 168, 400, false);

  prv_swipe(90, 120, 20, 120);   // right-to-left = left swipe
  cl_assert_equal_i(s_emit_count, 1);
  cl_assert_equal_i(s_last_emit, BUTTON_ID_SELECT);
  cl_assert_equal_i(s_pop_count, 0);

  swap_layer_touch_deregister(&fs.swap);
}

// The "no notification loaded" invariant for tap and swipe (symmetric to
// pan_declined_when_no_current). can_start only gates the pan (tap/swipe have no Started), so the tap
// and swipe ops must themselves early-return on current == NULL: a tap or horizontal swipe on a swap
// with no notification loaded must emulate NO button (would otherwise feed a click to a not-yet-loaded
// notification), matching the pre-refactor behaviour of doing nothing.
void test_swap_layer_touch__tap_and_swipe_inert_when_no_current(void) {
  prv_live_state_setup();
  FakeSwap fs = {0};
  layer_init(&fs.swap.layer, &GRect(0, 0, 144, 168));
  fs.swap.callbacks.interaction_handler = prv_count_interaction;
  cl_assert(fs.swap.current == NULL);   // no notification loaded yet
  layer_add_child(&s_root_layer, &fs.swap.layer);
  s_active_layer = &fs.swap.layer;
  swap_layer_touch_register(&fs.swap);

  // A tap emulates nothing.
  prv_drive(TouchEvent_Touchdown, 72, 120);
  prv_advance_ms(30);
  prv_drive(TouchEvent_Liftoff, 0, 0);
  cl_assert_equal_i(s_emit_count, 0);
  cl_assert_equal_i(s_pop_count, 0);

  // A right swipe (would be BACK -> pop) emulates nothing.
  prv_swipe(20, 120, 90, 120);
  cl_assert_equal_i(s_emit_count, 0);
  cl_assert_equal_i(s_pop_count, 0);

  // A left swipe (would be SELECT) emulates nothing.
  prv_swipe(90, 120, 20, 120);
  cl_assert_equal_i(s_emit_count, 0);
  cl_assert_equal_i(s_pop_count, 0);

  swap_layer_touch_deregister(&fs.swap);
}

// UAF-safety: deregistering a swap layer WHILE it is the live gesture target (before liftoff), e.g.
// the notification is torn down under a live window, clears the latched target before cancelling the
// recognizers (the touch_nav_registry_remove UAF hook), so a synchronous Cancelled callback cannot
// re-enter the freed swap layer.
void test_swap_layer_touch__deregister_mid_gesture_clears_target(void) {
  prv_live_state_setup();
  FakeSwap fs;
  prv_attach_and_register(&fs, 168, 400, false);

  // Finger down + a move so the swap layer becomes the live gesture target.
  prv_drive(TouchEvent_Touchdown, 72, 120);
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 72, 90);
  cl_assert(swap_layer_touch_is_gesture_target(&fs.swap));

  // Deregister mid-gesture: the target is cleared (before the recognizer cancel), and the node is
  // gone from the registry.
  swap_layer_touch_deregister(&fs.swap);
  cl_assert(!swap_layer_touch_is_gesture_target(&fs.swap));
  cl_assert(s_touch_nav_state.swap_head == NULL);
}
