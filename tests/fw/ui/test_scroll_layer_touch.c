/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"
#include "pebble_asserts.h"

#include "applib/ui/scroll_layer.h"
#include "applib/ui/scroll_layer_private.h"
#include "applib/ui/recognizer/recognizer.h"
#include "applib/ui/recognizer/recognizer_list.h"
#include "applib/ui/recognizer/recognizer_manager.h"
#include "applib/ui/recognizer/swipe.h"
#include "applib/ui/recognizer/touch_nav.h"

#include "fake_rtc.h"
#include "pbl/drivers/rtc.h"

// Stubs
/////////////////////
#include "stubs_app_state.h"
#include "stubs_compiled_with_legacy2_sdk.h"
#include "stubs_content_indicator.h"
#include "stubs_heap.h"
#include "stubs_logging.h"
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"
#include "stubs_pebble_tasks.h"
#include "stubs_resources.h"
#include "stubs_syscalls.h"
#include "stubs_unobstructed_area.h"

// ---------------------------------------------------------------------------------------------
// Touch-navigation harness (CONFIG_TOUCH). scroll_layer.c resolves the per-task touch-nav state
// through these accessors; the recognizer manager needs a few window/layer collaborators to link.

static bool s_nav_enabled = true;
bool sys_touch_nav_enabled(void) { return s_nav_enabled; }
bool sys_touch_app_nav_active(void) { return false; }

static TouchNavState s_touch_nav_state;
struct TouchNavState *app_state_get_touch_nav_state(void) { return &s_touch_nav_state; }
struct TouchNavState *modal_manager_get_touch_nav_state(void) { return &s_touch_nav_state; }

static Layer s_root_layer;               // window root, holds the scroll layer while driving pans
static RecognizerManager s_recognizer_manager;
static RecognizerList s_global_list;

struct Layer *window_get_root_layer(const Window *window) { return &s_root_layer; }
RecognizerList *window_get_recognizer_list(Window *window) { return NULL; }
RecognizerManager *window_get_recognizer_manager(Window *window) { return &s_recognizer_manager; }

// Fake bridge ops so swipe-right BACK / swipe-left SELECT are observable.
typedef struct FakeBridgeOps {
  bool overrides_back;
  bool animating;
  int pop_count;
  int emit_count;
  ButtonId last_emit;
} FakeBridgeOps;
static FakeBridgeOps s_bridge;
static bool prv_bridge_is_animating(void *ctx) { return ((FakeBridgeOps *)ctx)->animating; }
static bool prv_bridge_top_overrides_back(void *ctx) {
  return ((FakeBridgeOps *)ctx)->overrides_back;
}
static void prv_bridge_pop_top(void *ctx) { ((FakeBridgeOps *)ctx)->pop_count++; }
static void prv_bridge_emit_button(void *ctx, ButtonId b) {
  FakeBridgeOps *o = ctx; o->emit_count++; o->last_emit = b;
}
static TouchNavOps s_bridge_ops;

// Bring up a real per-task touch-nav state so scroll_layer_init() registers into a live registry.
static void prv_touch_nav_setup(void) {
  s_bridge = (FakeBridgeOps){0};
  s_bridge_ops = (TouchNavOps){
    .is_animating = prv_bridge_is_animating,
    .top_overrides_back = prv_bridge_top_overrides_back,
    .pop_top = prv_bridge_pop_top,
    .emit_button = prv_bridge_emit_button,
    .ctx = &s_bridge,
  };
  layer_init(&s_root_layer, &GRect(0, 0, 200, 400));
  recognizer_list_init(&s_global_list);
  recognizer_manager_init(&s_recognizer_manager);
  s_recognizer_manager.window = (Window *)&s_root_layer;  // non-NULL sentinel
  s_recognizer_manager.global_list = &s_global_list;
  touch_nav_state_init(&s_touch_nav_state, &s_recognizer_manager, &s_bridge_ops);
}

// Fakes
////////////////////
void graphics_context_set_compositing_mode(GContext* ctx, GCompOp mode) {}
void graphics_draw_bitmap_in_rect(GContext *ctx, const GBitmap *bitmap, const GRect *rect) {}
GDrawState graphics_context_get_drawing_state(GContext* ctx) { return (GDrawState){}; }
void graphics_context_set_drawing_state(GContext* ctx, GDrawState draw_state) {}

bool graphics_release_frame_buffer(GContext *ctx, GBitmap *buffer) { return false; }
void window_set_click_config_provider_with_context(struct Window *window,
                                                   ClickConfigProvider click_config_provider,
                                                   void *context) {}
void window_set_click_context(ButtonId button_id, void *context) {}
void window_schedule_render(struct Window *window) {}
void window_single_repeating_click_subscribe(ButtonId button_id, uint16_t repeat_interval_ms,
                                             ClickHandler handler) {}

// ---------------------------------------------------------------------------------------------
// Test lifecycle

void test_scroll_layer_touch__initialize(void) {
  fake_rtc_init(0, 0);
  s_nav_enabled = true;
  // A zeroed state has a NULL manager, so scroll_layer_init() registration is inert for tests that
  // do not opt into the touch-nav harness (prv_touch_nav_setup()).
  s_touch_nav_state = (TouchNavState){0};
  scroll_layer_touch_nav_reset_all();
}

void test_scroll_layer_touch__cleanup(void) {}

// Build a scroll layer with content taller than its frame so there is room to drag.
static void prv_make_tall_scroll(ScrollLayer *sl, GRect frame, int16_t content_h) {
  scroll_layer_init(sl, &frame);
  scroll_layer_set_content_size(sl, GSize(frame.size.w, content_h));
}

static void prv_drive(TouchEventType type, int16_t x, int16_t y) {
  const TouchEvent e = { .type = type, .x = x, .y = y, .non_navigational = false };
  touch_nav_dispatch(&e, &s_touch_nav_state);
}

static void prv_advance_ms(uint32_t ms) {
  fake_rtc_increment_ticks((RtcTicks)ms * RTC_TICKS_HZ / 1000);
}

// ---------------------------------------------------------------------------------------------
// Registration

void test_scroll_layer_touch__registered_and_deregistered(void) {
  prv_touch_nav_setup();
  ScrollLayer sl;
  prv_make_tall_scroll(&sl, GRect(0, 0, 200, 300), 900);
  cl_assert(s_touch_nav_state.scroll_head != NULL);  // Tier-1 Scroll widget
  cl_assert_equal_p(s_touch_nav_state.scroll_head->layer, scroll_layer_get_layer(&sl));
  scroll_layer_deinit(&sl);
  cl_assert(s_touch_nav_state.scroll_head == NULL);
}

// Init->init without deinit stays a single entry (dedup by address); double deinit is a no-op.
void test_scroll_layer_touch__double_init_and_double_deinit(void) {
  prv_touch_nav_setup();
  ScrollLayer sl;
  prv_make_tall_scroll(&sl, GRect(0, 0, 200, 300), 900);
  scroll_layer_init(&sl, &GRect(0, 0, 200, 300));  // re-init without deinit
  scroll_layer_set_content_size(&sl, GSize(200, 900));
  cl_assert(s_touch_nav_state.scroll_head != NULL);
  cl_assert(s_touch_nav_state.scroll_head->next == NULL);
  cl_assert_equal_p(s_touch_nav_state.scroll_head->layer, scroll_layer_get_layer(&sl));
  scroll_layer_deinit(&sl);
  cl_assert(s_touch_nav_state.scroll_head == NULL);
  scroll_layer_deinit(&sl);  // safe no-op
  cl_assert(s_touch_nav_state.scroll_head == NULL);
}

// ---------------------------------------------------------------------------------------------
// Criterion (a): a pan drags the content 1:1 and clamps at both ends.

void test_scroll_layer_touch__pan_drags_content_1to1_and_clamps(void) {
  ScrollLayer sl;
  prv_make_tall_scroll(&sl, GRect(0, 0, 200, 300), 900);  // min offset = 300 - 900 = -600

  // A downward-content pan of -120 moves the offset exactly -120 (1:1).
  scroll_layer_set_content_offset(&sl, GPoint(0, 0), false);
  scroll_layer_touch_handle_pan_update(&sl, GPoint(0, 0), GPoint(0, -120));
  cl_assert_equal_i(scroll_layer_get_content_offset(&sl).y, -120);

  // A further pan continues from the passed base 1:1.
  scroll_layer_touch_handle_pan_update(&sl, GPoint(0, -120), GPoint(0, -80));
  cl_assert_equal_i(scroll_layer_get_content_offset(&sl).y, -200);

  // Clamp at the bottom: a huge negative delta stops at min(frame_h - content_h, 0) = -600.
  scroll_layer_touch_handle_pan_update(&sl, GPoint(0, 0), GPoint(0, -5000));
  cl_assert_equal_i(scroll_layer_get_content_offset(&sl).y, -600);

  // Clamp at the top: a positive delta cannot pull the content past 0.
  scroll_layer_touch_handle_pan_update(&sl, GPoint(0, -100), GPoint(0, 5000));
  cl_assert_equal_i(scroll_layer_get_content_offset(&sl).y, 0);

  scroll_layer_deinit(&sl);
}

// Content shorter than the frame cannot be dragged off its top (lower bound collapses to 0).
void test_scroll_layer_touch__short_content_does_not_scroll(void) {
  ScrollLayer sl;
  prv_make_tall_scroll(&sl, GRect(0, 0, 200, 300), 120);  // content shorter than frame
  scroll_layer_touch_handle_pan_update(&sl, GPoint(0, 0), GPoint(0, -300));
  cl_assert_equal_i(scroll_layer_get_content_offset(&sl).y, 0);
  scroll_layer_deinit(&sl);
}

// ---------------------------------------------------------------------------------------------
// Criterion (b): a sub-threshold pan (finger barely moves) does not scroll; a real pan does.

void test_scroll_layer_touch__subthreshold_pan_does_not_scroll(void) {
  prv_touch_nav_setup();
  ScrollLayer sl;
  prv_make_tall_scroll(&sl, GRect(0, 0, 200, 300), 900);
  layer_add_child(&s_root_layer, scroll_layer_get_layer(&sl));

  // Touchdown then a 3px move: below PAN_START_THRESHOLD_PX (10) so the pan never Starts. The
  // Touchdown latches the scroll layer as the unified target (routing is fixed for the gesture), but
  // with the pan never Starting no content moves.
  prv_drive(TouchEvent_Touchdown, 100, 150);
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 147);
  cl_assert_equal_i(scroll_layer_get_content_offset(&sl).y, 0);   // no scroll
  prv_drive(TouchEvent_Liftoff, 100, 147);

  // A real pan (well past threshold) does scroll, confirming the widget is otherwise live. The pan
  // Starts on the first update (latch only) and live-scrolls on the second (Updated) event.
  prv_drive(TouchEvent_Touchdown, 100, 150);
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 110);   // 40px up -> pan Started
  cl_assert(scroll_layer_touch_is_gesture_target(&sl));
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 90);    // -> Updated -> live scroll
  cl_assert(scroll_layer_get_content_offset(&sl).y < 0);   // content scrolled

  scroll_layer_deinit(&sl);
}

// ---------------------------------------------------------------------------------------------
// Criterion (c): touchdown/pan-start cancels a running scroll animation so the finger takes over.

void test_scroll_layer_touch__pan_start_cancels_running_animation(void) {
  prv_touch_nav_setup();
  ScrollLayer sl;
  prv_make_tall_scroll(&sl, GRect(0, 0, 200, 300), 900);
  layer_add_child(&s_root_layer, scroll_layer_get_layer(&sl));

  // Kick off an animated scroll; the animation is now scheduled.
  scroll_layer_set_content_offset(&sl, GPoint(0, -400), true);
  Animation *anim = property_animation_get_animation(sl.animation);
  cl_assert(anim != NULL);
  cl_assert(animation_is_scheduled(anim));

  // A pan grabs the content: on pan Start the running animation is unscheduled.
  prv_drive(TouchEvent_Touchdown, 100, 150);
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 100);   // 50px -> pan Started
  cl_assert(scroll_layer_touch_is_gesture_target(&sl));
  cl_assert(!animation_is_scheduled(anim));          // finger cancelled the fling

  scroll_layer_deinit(&sl);
}

// ---------------------------------------------------------------------------------------------
// Criterion (d): a non-participating third-party app is inert even with the master pref ON.
//
// The opt-in gate is inherited: the app-task twin installs the system touch handler (and thus the
// dispatcher that routes to the Scroll registry) only when the pref is on AND the app participates.
// A non-participating app never reaches the dispatcher, so its registered ScrollLayer never scrolls.

// Drive a full pan through the dispatcher only if the app twin would be subscribed. The pan Starts
// on the first update (latch) and live-scrolls on the second (Updated) before liftoff.
static void prv_twin_pan(bool participating, int16_t sy, int16_t my, int16_t ey) {
  if (!touch_nav_app_twin_active(s_nav_enabled, false /* master */, participating,
                                 false /* opted_in */)) {
    return;  // twin not subscribed: the dispatcher is never wired, registry stays inert
  }
  prv_drive(TouchEvent_Touchdown, 100, sy);
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, my);
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, ey);
  prv_advance_ms(20);
  prv_drive(TouchEvent_Liftoff, 100, ey);
}

void test_scroll_layer_touch__third_party_inert_with_pref_on(void) {
  prv_touch_nav_setup();
  s_nav_enabled = true;                 // master pref ON
  ScrollLayer sl;
  prv_make_tall_scroll(&sl, GRect(0, 0, 200, 300), 900);
  layer_add_child(&s_root_layer, scroll_layer_get_layer(&sl));

  // Third-party app, no opt-in: the twin is inactive despite the pref, so the pan never routes.
  const bool participating = false;
  cl_assert(!touch_nav_app_twin_active(s_nav_enabled, false, participating, false));
  prv_twin_pan(participating, 150, 110, 60);
  cl_assert_equal_i(scroll_layer_get_content_offset(&sl).y, 0);   // inert
  cl_assert_equal_i(s_bridge.emit_count, 0);
  cl_assert_equal_i(s_bridge.pop_count, 0);

  // A participating (system / opted-in) app on the very same registry does scroll.
  const bool sys = true;
  cl_assert(touch_nav_app_twin_active(s_nav_enabled, false, sys, false));
  prv_twin_pan(sys, 150, 110, 60);
  cl_assert(scroll_layer_get_content_offset(&sl).y < 0);

  scroll_layer_deinit(&sl);
}

// ---------------------------------------------------------------------------------------------
// Criterion (e): deinit while this scroll layer is the active gesture target cancels cleanly and
// the next gesture still works.

void test_scroll_layer_touch__deinit_mid_gesture_cancels(void) {
  prv_touch_nav_setup();
  ScrollLayer sl;
  prv_make_tall_scroll(&sl, GRect(0, 0, 200, 300), 900);
  layer_add_child(&s_root_layer, scroll_layer_get_layer(&sl));

  // Drive a vertical pan to Started so this scroll layer becomes the gesture target.
  prv_drive(TouchEvent_Touchdown, 100, 150);
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 110);
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 70);
  cl_assert(scroll_layer_touch_is_gesture_target(&sl));

  // Destroy the widget under a live window: the gesture is cancelled and the target cleared.
  scroll_layer_deinit(&sl);
  cl_assert(!scroll_layer_touch_is_gesture_target(&sl));
  cl_assert(s_touch_nav_state.scroll_head == NULL);

  // A fresh scroll layer + touch still works (routing recovered).
  ScrollLayer sl2;
  prv_make_tall_scroll(&sl2, GRect(0, 0, 200, 300), 900);
  layer_add_child(&s_root_layer, scroll_layer_get_layer(&sl2));
  prv_drive(TouchEvent_Touchdown, 100, 150);
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 110);   // pan Started
  cl_assert(scroll_layer_touch_is_gesture_target(&sl2));
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 80);    // Updated -> scroll
  cl_assert(scroll_layer_get_content_offset(&sl2).y < 0);

  scroll_layer_deinit(&sl2);
}

// ---------------------------------------------------------------------------------------------
// Horizontal swipe navigation: right = BACK (pop), left = SELECT.

void test_scroll_layer_touch__swipe_right_emits_back(void) {
  prv_touch_nav_setup();
  s_bridge.overrides_back = false;   // no back handler => the bridge pops the window
  ScrollLayer sl;
  prv_make_tall_scroll(&sl, GRect(0, 0, 200, 300), 900);
  scroll_layer_touch_handle_swipe(&sl, SwipeDirection_Right);
  cl_assert_equal_i(s_bridge.pop_count, 1);
  cl_assert_equal_i(s_bridge.emit_count, 0);
  scroll_layer_deinit(&sl);
}

void test_scroll_layer_touch__swipe_left_emits_select(void) {
  prv_touch_nav_setup();
  ScrollLayer sl;
  prv_make_tall_scroll(&sl, GRect(0, 0, 200, 300), 900);
  scroll_layer_touch_handle_swipe(&sl, SwipeDirection_Left);
  cl_assert_equal_i(s_bridge.emit_count, 1);
  cl_assert_equal_i(s_bridge.last_emit, BUTTON_ID_SELECT);
  cl_assert_equal_i(s_bridge.pop_count, 0);
  scroll_layer_deinit(&sl);
}

// A swipe mid window-transition is dropped (guarded against a double action).
void test_scroll_layer_touch__swipe_dropped_while_animating(void) {
  prv_touch_nav_setup();
  s_bridge.animating = true;
  ScrollLayer sl;
  prv_make_tall_scroll(&sl, GRect(0, 0, 200, 300), 900);
  scroll_layer_touch_handle_swipe(&sl, SwipeDirection_Right);
  scroll_layer_touch_handle_swipe(&sl, SwipeDirection_Left);
  cl_assert_equal_i(s_bridge.pop_count, 0);
  cl_assert_equal_i(s_bridge.emit_count, 0);
  scroll_layer_deinit(&sl);
}

// =============================================================================================
// Unified widget set: gestures driven THROUGH touch_nav_dispatch (not the apply functions).
// These exercise the Ф2 vtable path end to end: Touchdown latches the ScrollLayer as the migrated
// widget target, and the unified (tap, pan, swipe) set drives it via s_scroll_touch_nav_ops.

// A quick, stationary tap through the dispatcher (Touchdown then Liftoff, no movement).
static void prv_scroll_dispatch_tap(int16_t x, int16_t y) {
  prv_drive(TouchEvent_Touchdown, x, y);
  prv_advance_ms(30);
  prv_drive(TouchEvent_Liftoff, x, y);
}

// ---- A bare pan through the dispatcher scrolls 1:1 and settles on liftoff ----

void test_scroll_layer_touch__dispatch_pan_scrolls_1to1_and_snaps(void) {
  prv_touch_nav_setup();
  ScrollLayer sl;
  prv_make_tall_scroll(&sl, GRect(0, 0, 200, 300), 900);  // min offset = 300 - 900 = -600
  layer_add_child(&s_root_layer, scroll_layer_get_layer(&sl));

  // Touchdown, then a first update 40px up crosses the pan threshold: the pan Starts and only latches
  // the base (offset unchanged). delta_since_start is re-anchored to (0,0) at Start.
  prv_drive(TouchEvent_Touchdown, 100, 200);
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 160);   // pan Started at y=160, base latched
  cl_assert(scroll_layer_touch_is_gesture_target(&sl));
  cl_assert_equal_i(scroll_layer_get_content_offset(&sl).y, 0);

  // A second update 30px further up live-scrolls the content 1:1 (base 0 + delta -30).
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 130);
  cl_assert_equal_i(scroll_layer_get_content_offset(&sl).y, -30);

  // Liftoff settles to the final (unthrottled) offset, still -30, and clears the latch.
  prv_drive(TouchEvent_Liftoff, 100, 130);
  cl_assert_equal_i(scroll_layer_get_content_offset(&sl).y, -30);
  cl_assert(!scroll_layer_touch_is_gesture_target(&sl));
  cl_assert_equal_i(s_bridge.emit_count, 0);   // a pan is never a bridge button
  cl_assert_equal_i(s_bridge.pop_count, 0);

  scroll_layer_deinit(&sl);
}

// ---- A tap on a bare ScrollLayer does NOTHING (tap op is NULL, no bridge SELECT) ----

void test_scroll_layer_touch__dispatch_tap_does_nothing(void) {
  prv_touch_nav_setup();
  ScrollLayer sl;
  prv_make_tall_scroll(&sl, GRect(0, 0, 200, 300), 900);
  layer_add_child(&s_root_layer, scroll_layer_get_layer(&sl));

  // A stationary tap on the scroll routes to the unified set (the bridge set is failed on the Tier-1
  // route). ScrollLayer's tap op is NULL, so the tap is dropped: no scroll, and crucially NO bridge
  // SELECT (the tap must not fall through to the Tier-2 bridge).
  prv_scroll_dispatch_tap(100, 150);
  cl_assert_equal_i(scroll_layer_get_content_offset(&sl).y, 0);   // no scroll
  cl_assert_equal_i(s_bridge.emit_count, 0);                      // no SELECT emitted
  cl_assert_equal_i(s_bridge.pop_count, 0);
  cl_assert(!scroll_layer_touch_is_gesture_target(&sl));          // latch cleared on completion
  // The manager returns to idle after the dropped tap.
  cl_assert_equal_i(s_recognizer_manager.state, RecognizerManagerState_WaitForTouchdown);

  scroll_layer_deinit(&sl);
}

// ---- A no-trigger gesture on a scroll-only route leaves the manager idle ----
//
// The app's only widget is a ScrollLayer, but the finger lands off it: the resolver finds no
// migrated widget under the active layer, so the unified set is failed at the Touchdown latch
// (symmetric to the bridge exclusion). A gesture that then triggers nothing leaves no recognizer
// stuck Possible -- the manager is idle by construction after liftoff.
void test_scroll_layer_touch__dispatch_no_trigger_leaves_manager_idle(void) {
  prv_touch_nav_setup();
  ScrollLayer sl;
  // Register the scroll away from the top status-bar dead zone so a dead-zone Touchdown does not get
  // routed to it as the sole widget.
  scroll_layer_init(&sl, &GRect(0, 100, 200, 200));
  scroll_layer_set_content_size(&sl, GSize(200, 600));
  layer_add_child(&s_root_layer, scroll_layer_get_layer(&sl));

  // A Touchdown in the top dead zone resolves no widget under the active layer: the unified set is
  // failed at the latch.
  prv_drive(TouchEvent_Touchdown, 100, 4);
  cl_assert(!scroll_layer_touch_is_gesture_target(&sl));
  cl_assert_equal_i(recognizer_get_state(s_touch_nav_state.widget_tap), RecognizerState_Failed);
  cl_assert_equal_i(recognizer_get_state(s_touch_nav_state.widget_pan), RecognizerState_Failed);
  cl_assert_equal_i(recognizer_get_state(s_touch_nav_state.widget_swipe), RecognizerState_Failed);

  // Liftoff with nothing triggered returns the manager to idle: no stuck Possible recognizer.
  prv_drive(TouchEvent_Liftoff, 100, 4);
  cl_assert_equal_i(s_recognizer_manager.state, RecognizerManagerState_WaitForTouchdown);

  scroll_layer_deinit(&sl);
}

