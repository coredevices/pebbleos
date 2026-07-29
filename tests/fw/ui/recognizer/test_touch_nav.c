/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "applib/ui/layer.h"
#include "applib/ui/window.h"
#include "applib/ui/recognizer/recognizer.h"
#include "applib/ui/recognizer/recognizer_impl.h"
#include "applib/ui/recognizer/recognizer_list.h"
#include "applib/ui/recognizer/recognizer_manager.h"
#include "applib/ui/recognizer/pan.h"
#include "applib/ui/recognizer/recognizer_private.h"
#include "applib/ui/recognizer/swipe.h"
#include "applib/ui/recognizer/tap.h"
#include "applib/ui/recognizer/touch_nav.h"

#include "pbl/drivers/rtc.h"

// Fakes
#include "fake_rtc.h"

// Stubs
#include "stubs_app_install_manager.h"
#include "stubs_app_state.h"
#include "stubs_gbitmap.h"
#include "stubs_graphics.h"
#include "stubs_heap.h"
#include "stubs_logging.h"
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"
#include "stubs_pebble_tasks.h"
#include "stubs_process_manager.h"
#include "stubs_ui_window.h"
#include "stubs_unobstructed_area.h"

// ---------------------------------------------------------------------------------------------
// Collaborator stubs the recognizer manager needs (mirrors test_recognizer_manager.c).

static Layer *s_active_layer;
static RecognizerManager *s_manager;

RecognizerList *window_get_recognizer_list(Window *window) {
  // The window carries no window-level recognizers in these tests; only the task-global list.
  return NULL;
}

RecognizerManager *window_get_recognizer_manager(Window *window) {
  return s_manager;
}

struct Layer *window_get_root_layer(const Window *window) {
  if (!window) {
    return NULL;
  }
  return &((Window *)window)->layer;
}

Layer *layer_find_layer_containing_point(const Layer *node, const GPoint *point) {
  return s_active_layer;
}

// ---------------------------------------------------------------------------------------------
// Master-pref gate, controlled by the tests.

static bool s_nav_enabled;

bool touch_nav_enabled(void) {
  return s_nav_enabled;
}

bool touch_app_nav_active(void) { return false; }

// ---------------------------------------------------------------------------------------------
// Fake TouchNavOps, recording every effect.

typedef struct FakeOps {
  bool animating;
  bool overrides_back;
  bool bridge_disabled;         // the top window opted out (window_set_touch_bridge_disabled)
  bool app_has_raw_subscriber;  // the app installed its own raw touch subscriber
  int pop_count;
  int idle_refresh_count;
  int emit_count;
  ButtonId last_emit;
} FakeOps;

static FakeOps s_fake;

static bool prv_is_animating(void *ctx) { return ((FakeOps *)ctx)->animating; }
static bool prv_top_overrides_back(void *ctx) { return ((FakeOps *)ctx)->overrides_back; }
static bool prv_top_bridge_disabled(void *ctx) {
  // Exercise the real app-task gate helper (window opt-out OR app raw subscriber), exactly as
  // app_state.c's top_bridge_disabled op does.
  const FakeOps *f = ctx;
  return touch_nav_app_bridge_disabled(f->bridge_disabled, f->app_has_raw_subscriber);
}
static void prv_pop_top(void *ctx) { ((FakeOps *)ctx)->pop_count++; }
static void prv_idle_refresh(void *ctx) { ((FakeOps *)ctx)->idle_refresh_count++; }
static void prv_emit_button(void *ctx, ButtonId button) {
  FakeOps *ops = ctx;
  ops->emit_count++;
  ops->last_emit = button;
}

static TouchNavOps s_ops;

// ---------------------------------------------------------------------------------------------
// Fake TouchNavTwinOps for the app-twin subscribe/reconcile state machine (the real logic app_state
// delegates to). Records install/remove and reports a controllable master pref.

typedef struct FakeTwin {
  bool pref;    // effective SYSTEM nav (master AND sub-pref)
  bool master;  // master pref alone
  int install_count;
  int remove_count;
} FakeTwin;

static FakeTwin s_twin;
static TouchNavTwinOps s_twin_ops;

static bool prv_twin_pref(void *ctx) { return ((FakeTwin *)ctx)->pref; }
static bool prv_twin_master(void *ctx) { return ((FakeTwin *)ctx)->master; }
static void prv_twin_install(void *ctx) { ((FakeTwin *)ctx)->install_count++; }
static void prv_twin_remove(void *ctx) { ((FakeTwin *)ctx)->remove_count++; }

// ---------------------------------------------------------------------------------------------
// Fixture

static Window s_window;
static Layer s_child_layer;
static RecognizerList s_global_list;
static RecognizerManager s_recognizer_manager;
static TouchNavState s_state;

void test_touch_nav__initialize(void) {
  fake_rtc_init(0, 0);
  s_nav_enabled = true;
  s_active_layer = NULL;
  s_fake = (FakeOps){0};
  s_ops = (TouchNavOps){
    .is_animating = prv_is_animating,
    .top_overrides_back = prv_top_overrides_back,
    .top_bridge_disabled = prv_top_bridge_disabled,
    .pop_top = prv_pop_top,
    .emit_button = prv_emit_button,
    .idle_refresh = prv_idle_refresh,
    .ctx = &s_fake,
  };

  s_twin = (FakeTwin){0};
  s_twin_ops = (TouchNavTwinOps){
    .pref_enabled = prv_twin_pref,
    .master_enabled = prv_twin_master,
    .install_handler = prv_twin_install,
    .remove_handler = prv_twin_remove,
    .ctx = &s_twin,
  };

  s_window = (Window){};
  layer_init(&s_window.layer, &GRectZero);
  layer_init(&s_child_layer, &GRectZero);
  layer_add_child(&s_window.layer, &s_child_layer);

  recognizer_list_init(&s_global_list);
  recognizer_manager_init(&s_recognizer_manager);
  s_recognizer_manager.window = &s_window;
  s_recognizer_manager.global_list = &s_global_list;
  s_manager = &s_recognizer_manager;

  touch_nav_state_init(&s_state, &s_recognizer_manager, &s_ops);
}

void test_touch_nav__cleanup(void) {
  touch_nav_state_deinit(&s_state);
}

// ---------------------------------------------------------------------------------------------
// Helpers

static void prv_dispatch(TouchEventType type, int16_t x, int16_t y, bool non_navigational) {
  const TouchEvent e = {
    .type = type,
    .x = x,
    .y = y,
    .non_navigational = non_navigational,
  };
  touch_nav_dispatch(&e, &s_state);
}

static void prv_advance_ms(uint32_t ms) {
  fake_rtc_increment_ticks((RtcTicks)ms * RTC_TICKS_HZ / MS_PER_SECOND);
}

// Drive a straight fast flick from (sx, sy) to (ex, ey) through the dispatcher. The route is
// resolved on the Touchdown.
static void prv_swipe(int16_t sx, int16_t sy, int16_t ex, int16_t ey) {
  prv_dispatch(TouchEvent_Touchdown, sx, sy, false);
  prv_advance_ms(20);
  prv_dispatch(TouchEvent_PositionUpdate, (int16_t)((sx + ex) / 2), (int16_t)((sy + ey) / 2), false);
  prv_advance_ms(20);
  prv_dispatch(TouchEvent_PositionUpdate, ex, ey, false);
  prv_advance_ms(20);
  prv_dispatch(TouchEvent_Liftoff, 0, 0, false);
}

// A quick, stationary tap through the dispatcher.
static void prv_tap(int16_t x, int16_t y) {
  prv_dispatch(TouchEvent_Touchdown, x, y, false);
  prv_advance_ms(30);
  prv_dispatch(TouchEvent_Liftoff, 0, 0, false);
}

static RecognizerState prv_state(Recognizer *r) {
  return recognizer_get_state(r);
}

// ---------------------------------------------------------------------------------------------
// Criterion 7: Tier-2 → pan Failed immediately; swipe up → Completed.
// Content-scroll convention (criterion 1/7): a swipe up emulates the DOWN button.

void test_touch_nav__tier2_fails_pan_and_completes_swipe(void) {
  // Touchdown below the dead zone, no Tier-1 widget, bridge enabled → Tier-2 route.
  prv_dispatch(TouchEvent_Touchdown, 90, 90, false);
  cl_assert_equal_i(s_state.route, TouchNavRoute_Tier2);
  // pan is failed at routing time so it cannot Start at threshold and fail the swipe.
  cl_assert_equal_i(prv_state(s_state.pan), RecognizerState_Failed);
  cl_assert_equal_i(prv_state(s_state.tap), RecognizerState_Possible);
  cl_assert_equal_i(prv_state(s_state.swipe), RecognizerState_Possible);

  prv_advance_ms(20);
  prv_dispatch(TouchEvent_PositionUpdate, 90, 60, false);
  prv_advance_ms(20);
  prv_dispatch(TouchEvent_PositionUpdate, 90, 30, false);
  prv_advance_ms(20);
  prv_dispatch(TouchEvent_Liftoff, 0, 0, false);

  // The swipe Completed (observed via the bridge emulation and the completed counter). The manager
  // resets the set to Possible immediately after completion, so state is asserted through the emit.
  cl_assert_equal_i(s_state.counters.completed, 1);
  cl_assert_equal_i(s_fake.emit_count, 1);
  cl_assert_equal_i(s_fake.last_emit, BUTTON_ID_DOWN);
}

// ---------------------------------------------------------------------------------------------
// Criterion 1: direction mapping + BACK branch for a window with no BACK handler.
// Content-scroll convention: the emulated button opposes finger travel (swipe up -> DOWN button,
// swipe down -> UP button), matching native MenuLayer/ScrollLayer touch.

void test_touch_nav__swipe_up_maps_to_down(void) {
  prv_swipe(50, 90, 50, 20);
  cl_assert_equal_i(s_fake.emit_count, 1);
  cl_assert_equal_i(s_fake.last_emit, BUTTON_ID_DOWN);
  cl_assert_equal_i(s_fake.pop_count, 0);
}

void test_touch_nav__swipe_down_maps_to_up(void) {
  prv_swipe(50, 20, 50, 90);
  cl_assert_equal_i(s_fake.emit_count, 1);
  cl_assert_equal_i(s_fake.last_emit, BUTTON_ID_UP);
}

void test_touch_nav__swipe_right_no_back_handler_pops(void) {
  s_fake.overrides_back = false;
  prv_swipe(20, 90, 90, 90);
  // Left-to-right (right) maps to BACK; with no back handler it pops the stack rather than
  // feeding the click recognizer.
  cl_assert_equal_i(s_fake.pop_count, 1);
  cl_assert_equal_i(s_fake.emit_count, 0);
}

void test_touch_nav__swipe_right_with_back_handler_emits_click(void) {
  s_fake.overrides_back = true;
  prv_swipe(20, 90, 90, 90);
  // The window handles BACK itself, so synthesize the button rather than popping.
  cl_assert_equal_i(s_fake.pop_count, 0);
  cl_assert_equal_i(s_fake.emit_count, 1);
  cl_assert_equal_i(s_fake.last_emit, BUTTON_ID_BACK);
}

void test_touch_nav__tap_maps_to_select(void) {
  prv_tap(50, 90);
  cl_assert_equal_i(s_fake.emit_count, 1);
  cl_assert_equal_i(s_fake.last_emit, BUTTON_ID_SELECT);
}

void test_touch_nav__swipe_left_maps_to_select(void) {
  // Right-to-left (left) maps to SELECT; it never pops.
  prv_swipe(90, 90, 20, 90);
  cl_assert_equal_i(s_fake.emit_count, 1);
  cl_assert_equal_i(s_fake.last_emit, BUTTON_ID_SELECT);
  cl_assert_equal_i(s_fake.pop_count, 0);
}

// ---------------------------------------------------------------------------------------------
// Criterion 2: a gesture completing during window_stack_is_animating is dropped.

void test_touch_nav__completion_during_animation_dropped(void) {
  s_fake.animating = true;
  prv_swipe(50, 90, 50, 20);
  cl_assert_equal_i(s_fake.emit_count, 0);
  cl_assert_equal_i(s_fake.pop_count, 0);
  cl_assert_equal_i(s_state.counters.dropped, 1);
}

// ---------------------------------------------------------------------------------------------
// Criterion 3: a non_navigational Touchdown never reaches the bridge.

void test_touch_nav__non_navigational_never_bridges(void) {
  prv_dispatch(TouchEvent_Touchdown, 50, 90, true /* non_navigational */);
  cl_assert_equal_i(s_state.counters.gated, 1);
  // Feed the rest of a would-be swipe; nothing must be emulated.
  prv_advance_ms(20);
  prv_dispatch(TouchEvent_PositionUpdate, 50, 60, true);
  prv_advance_ms(20);
  prv_dispatch(TouchEvent_PositionUpdate, 50, 20, true);
  prv_advance_ms(20);
  prv_dispatch(TouchEvent_Liftoff, 0, 0, true);
  cl_assert_equal_i(s_fake.emit_count, 0);
  cl_assert_equal_i(s_fake.pop_count, 0);
}

// ---------------------------------------------------------------------------------------------
// Criterion 4: a touch_bridge_disabled window with no Tier-1 widget fails all three system
// recognizers after the Touchdown.

void test_touch_nav__bridge_disabled_fails_all(void) {
  s_fake.bridge_disabled = true;
  prv_dispatch(TouchEvent_Touchdown, 50, 90, false);
  cl_assert_equal_i(s_state.route, TouchNavRoute_None);
  cl_assert_equal_i(prv_state(s_state.tap), RecognizerState_Failed);
  cl_assert_equal_i(prv_state(s_state.pan), RecognizerState_Failed);
  cl_assert_equal_i(prv_state(s_state.swipe), RecognizerState_Failed);
}

// ---------------------------------------------------------------------------------------------
// П5: anti-double arbitration — "one gesture → exactly one action, never double". The fake
// emit_button op counts every emulation so we can assert exactly-one (never 0 or 2).

// (a) A bridge-disabled window (the app owns touch): a full gesture emulates nothing.
void test_touch_nav__arbitration_bridge_disabled_no_emit(void) {
  s_fake.bridge_disabled = true;
  prv_swipe(50, 90, 50, 20);
  cl_assert_equal_i(s_fake.emit_count, 0);
  cl_assert_equal_i(s_fake.pop_count, 0);
}

// (b) The persistent-sensor timing window: the window becomes bridge-disabled BETWEEN the Touchdown
// (which latched Tier-2) and the Liftoff. The Liftoff re-check must catch the changed arbitration
// and drop, so the bridge does not fire alongside the app -> no double action.
void test_touch_nav__arbitration_persistent_sensor_timing_window(void) {
  s_fake.bridge_disabled = false;
  prv_dispatch(TouchEvent_Touchdown, 50, 90, false);
  cl_assert_equal_i(s_state.route, TouchNavRoute_Tier2);

  // The app takes over touch after the route already latched Tier-2.
  s_fake.bridge_disabled = true;

  prv_advance_ms(20);
  prv_dispatch(TouchEvent_PositionUpdate, 50, 60, false);
  prv_advance_ms(20);
  prv_dispatch(TouchEvent_PositionUpdate, 50, 20, false);
  prv_advance_ms(20);
  prv_dispatch(TouchEvent_Liftoff, 0, 0, false);

  // The swipe completed but the Liftoff re-check dropped it: exactly zero emulations.
  cl_assert_equal_i(s_state.counters.completed, 1);
  cl_assert_equal_i(s_fake.emit_count, 0);
  cl_assert_equal_i(s_fake.pop_count, 0);
  cl_assert_equal_i(s_state.counters.dropped, 1);
}

// (c) A normal Tier-2 gesture emulates EXACTLY ONE button — never zero, never twice.
void test_touch_nav__arbitration_single_gesture_single_emit(void) {
  prv_swipe(50, 90, 50, 20);
  cl_assert_equal_i(s_fake.emit_count, 1);
}

// ---------------------------------------------------------------------------------------------
// Raw-subscriber arbitration gate: an app that installed its own raw touch subscriber
// (touch_service_subscribe) must keep its raw handler AND never receive synthesized buttons —
// touch nav must not change existing runtime behavior for a third-party app. The app-task gate
// (app_state.c's top_bridge_disabled op) routes such an app to TouchNavRoute_None. Tier-1 native
// widgets still win before the gate is consulted.

// The pure gate helper: window opt-out OR a genuine app raw subscriber disables the Tier-2 bridge.
void test_touch_nav__app_bridge_disabled_helper(void) {
  cl_assert(!touch_nav_app_bridge_disabled(false, false));  // neither -> bridge active (Tier-2)
  cl_assert(touch_nav_app_bridge_disabled(true, false));    // window opted out -> disabled
  cl_assert(touch_nav_app_bridge_disabled(false, true));    // app owns raw touch -> disabled
  cl_assert(touch_nav_app_bridge_disabled(true, true));     // both -> disabled
}

// (a) A participating app WITH a raw touch subscriber, master pref ON: a full swipe and a tap both
// synthesize NOTHING (route None), so the app's own raw handler is the only thing that runs.
void test_touch_nav__raw_subscriber_suppresses_synthesis(void) {
  s_nav_enabled = true;
  s_fake.bridge_disabled = false;      // window did NOT call window_set_touch_bridge_disabled
  s_fake.app_has_raw_subscriber = true;  // but the app called touch_service_subscribe

  prv_dispatch(TouchEvent_Touchdown, 50, 90, false);
  cl_assert_equal_i(s_state.route, TouchNavRoute_None);
  prv_dispatch(TouchEvent_Liftoff, 0, 0, false);

  prv_swipe(50, 90, 50, 20);
  prv_tap(50, 90);
  cl_assert_equal_i(s_fake.emit_count, 0);
  cl_assert_equal_i(s_fake.pop_count, 0);
}

// (b) A participating app WITHOUT a raw subscriber synthesizes as before (Tier-2 unchanged).
void test_touch_nav__no_raw_subscriber_synthesizes_as_before(void) {
  s_nav_enabled = true;
  s_fake.bridge_disabled = false;
  s_fake.app_has_raw_subscriber = false;

  // A swipe up routes Tier-2 on the Touchdown and completes to exactly one synthesized DOWN.
  prv_dispatch(TouchEvent_Touchdown, 50, 90, false);
  cl_assert_equal_i(s_state.route, TouchNavRoute_Tier2);
  prv_advance_ms(20);
  prv_dispatch(TouchEvent_PositionUpdate, 50, 55, false);
  prv_advance_ms(20);
  prv_dispatch(TouchEvent_PositionUpdate, 50, 20, false);
  prv_advance_ms(20);
  prv_dispatch(TouchEvent_Liftoff, 0, 0, false);
  cl_assert_equal_i(s_fake.emit_count, 1);
  cl_assert_equal_i(s_fake.last_emit, BUTTON_ID_DOWN);
}

// (c) A Tier-1 native widget wins before the raw-subscriber gate is ever consulted: even with a raw
// subscriber installed, the registered widget owns the gesture (Tier-1, not None).
void test_touch_nav__tier1_wins_over_raw_subscriber_gate(void) {
  s_fake.app_has_raw_subscriber = true;
  static TouchNavWidgetNode node;
  node = (TouchNavWidgetNode){0};
  touch_nav_registry_add(&s_state, TouchNavWidgetType_Menu, &node, &s_child_layer, NULL, NULL);
  s_active_layer = &s_child_layer;

  prv_dispatch(TouchEvent_Touchdown, 50, 90, false);
  cl_assert_equal_i(s_state.route, TouchNavRoute_Tier1);
  cl_assert_equal_i(prv_state(s_state.tap), RecognizerState_Failed);
  cl_assert_equal_i(prv_state(s_state.pan), RecognizerState_Failed);
  cl_assert_equal_i(prv_state(s_state.swipe), RecognizerState_Failed);

  touch_nav_registry_remove(&s_state, TouchNavWidgetType_Menu, &node);
}

// (d) Backlight / nav system-hold alone must NOT count as an app raw subscriber. Those holds live
// in the touch service layer and the app twin installs into the separate system slot; only
// touch_service_subscribe sets the raw handler. So with app_has_raw_subscriber=false the bridge
// stays active and synthesis proceeds — no false suppression.
void test_touch_nav__system_hold_alone_does_not_suppress(void) {
  s_nav_enabled = true;
  s_fake.bridge_disabled = false;
  s_fake.app_has_raw_subscriber = false;  // twin/backlight/system-hold do not set the raw slot
  cl_assert(!touch_nav_app_bridge_disabled(false, false));

  prv_swipe(50, 90, 50, 20);
  cl_assert_equal_i(s_fake.emit_count, 1);
  cl_assert_equal_i(s_fake.last_emit, BUTTON_ID_DOWN);
}

// ---------------------------------------------------------------------------------------------
// Criterion 5: pref OFF → recognizers not activated, bridge synthesizes nothing even with a stale
// subscription (the state remains subscribed but nav is disabled).

void test_touch_nav__pref_off_is_inert(void) {
  s_nav_enabled = false;
  prv_swipe(50, 90, 50, 20);
  // The manager never activated: the set stays Possible and nothing is emulated.
  cl_assert_equal_i(s_recognizer_manager.state, RecognizerManagerState_WaitForTouchdown);
  cl_assert_equal_i(prv_state(s_state.tap), RecognizerState_Possible);
  cl_assert_equal_i(prv_state(s_state.pan), RecognizerState_Possible);
  cl_assert_equal_i(prv_state(s_state.swipe), RecognizerState_Possible);
  cl_assert_equal_i(s_fake.emit_count, 0);
  cl_assert_equal_i(s_fake.pop_count, 0);
}

// ---------------------------------------------------------------------------------------------
// App-twin gate: a participating app (system apps by default) rides the SYSTEM nav state (master
// AND the Touch Navigation sub-pref), while an app that explicitly opted in follows the master
// pref alone. Modeled here by driving the dispatcher only when the twin is active, exactly as
// app_touch_nav_subscribe() would gate the handler installation.

// Drive a full swipe through the dispatcher only if the app twin would be subscribed. An inert twin
// (handler never installed) means the dispatcher is never reached, so nothing is emulated.
static void prv_twin_swipe(bool participating, int16_t sx, int16_t sy, int16_t ex, int16_t ey) {
  if (!touch_nav_app_twin_active(s_nav_enabled, false /* master */, participating,
                                 false /* opted_in */)) {
    return;
  }
  prv_swipe(sx, sy, ex, ey);
}

// (a) A third-party app (does not participate) with system nav ON is inert: no emulation.
void test_touch_nav__app_twin_third_party_inert_with_pref_on(void) {
  s_nav_enabled = true;
  const bool participating = false;  // third-party app, no opt-in
  cl_assert(!touch_nav_app_twin_active(s_nav_enabled, false, participating, false));
  prv_twin_swipe(participating, 50, 90, 50, 20);
  cl_assert_equal_i(s_fake.emit_count, 0);
  cl_assert_equal_i(s_fake.pop_count, 0);
}

// (b) A system app (participates by default) with system nav ON gets touch nav.
void test_touch_nav__app_twin_system_app_active_with_pref_on(void) {
  s_nav_enabled = true;
  const bool participating = true;  // system app
  cl_assert(touch_nav_app_twin_active(s_nav_enabled, false, participating, false));
  prv_twin_swipe(participating, 50, 90, 50, 20);
  cl_assert_equal_i(s_fake.emit_count, 1);
  cl_assert_equal_i(s_fake.last_emit, BUTTON_ID_DOWN);
}

// (c) A third-party app that opts in gets touch nav while system nav is on.
void test_touch_nav__app_twin_third_party_optin_activates(void) {
  s_nav_enabled = true;
  bool participating = false;  // third-party app, initially inert
  cl_assert(!touch_nav_app_twin_active(s_nav_enabled, false, participating, false));
  prv_twin_swipe(participating, 50, 90, 50, 20);
  cl_assert_equal_i(s_fake.emit_count, 0);

  // app_touch_navigation_enable(true) flips participation.
  participating = true;
  cl_assert(touch_nav_app_twin_active(s_nav_enabled, false, participating, false));
  prv_twin_swipe(participating, 50, 90, 50, 20);
  cl_assert_equal_i(s_fake.emit_count, 1);
  cl_assert_equal_i(s_fake.last_emit, BUTTON_ID_DOWN);
}

// System nav off still gates mere participation: a system app is inert when either pref is off.
void test_touch_nav__app_twin_pref_off_dominates_participation(void) {
  s_nav_enabled = false;
  cl_assert(!touch_nav_app_twin_active(s_nav_enabled, false, true /* participating */, false));
  prv_twin_swipe(true, 50, 90, 50, 20);
  cl_assert_equal_i(s_fake.emit_count, 0);
}

// An EXPLICIT opt-in follows the master pref alone: active with system nav off but master on,
// inert once the master goes off too. Mere participation never rides the master by itself.
void test_touch_nav__app_twin_opted_in_follows_master(void) {
  // System nav off (Touch Navigation sub-pref off), master on: opted-in app is active...
  cl_assert(touch_nav_app_twin_active(false, true, true, true /* opted_in */));
  // ...a merely-participating (system) app is not...
  cl_assert(!touch_nav_app_twin_active(false, true, true, false));
  // ...and with the master off everything is inert, opted in or not.
  cl_assert(!touch_nav_app_twin_active(false, false, true, true));
  // Opt-in without the master is nothing either.
  cl_assert(!touch_nav_app_twin_active(false, false, false, true));
}

// ---------------------------------------------------------------------------------------------
// App-twin subscribe/reconcile state machine — the REAL logic app_state.c delegates to
// (touch_nav_app_twin_subscribe / touch_nav_app_twin_reconcile), driven through fake install/remove
// ops so the actual wiring (not just the predicate) is exercised.

// (a) subscribe installs the system handler only when the gate passes.
void test_touch_nav__app_twin_subscribe_gate(void) {
  // System nav off + participating -> no install.
  s_twin.pref = false;
  touch_nav_app_twin_subscribe(&s_twin_ops, true, false);
  cl_assert_equal_i(s_twin.install_count, 0);

  // System nav on + not participating -> no install (third-party default).
  s_twin.pref = true;
  touch_nav_app_twin_subscribe(&s_twin_ops, false, false);
  cl_assert_equal_i(s_twin.install_count, 0);

  // System nav on + participating -> install exactly once.
  touch_nav_app_twin_subscribe(&s_twin_ops, true, false);
  cl_assert_equal_i(s_twin.install_count, 1);
  cl_assert_equal_i(s_twin.remove_count, 0);

  // Opted in + master on (system nav off) -> installs too: opt-ins ride the master alone.
  s_twin = (FakeTwin){.pref = false, .master = true};
  touch_nav_app_twin_subscribe(&s_twin_ops, true, true);
  cl_assert_equal_i(s_twin.install_count, 1);
}

// (b) The launch-time classification (participate iff a system/negative install id) leaves an
// INSTALL_ID_INVALID (unresolved) app non-participating, so the twin never installs for it.
void test_touch_nav__app_twin_invalid_install_id_is_inert(void) {
  // The exact rule app_state_init uses: app_install_id_from_system(app_manager_get_current_app_id()).
  cl_assert(!app_install_id_from_system(INSTALL_ID_INVALID));  // 0 -> not a system app
  cl_assert(!app_install_id_from_system(5));                   // third-party app-DB id -> not system
  cl_assert(app_install_id_from_system(-3));                   // system/built-in -> participates

  s_twin.pref = true;
  const bool participating = app_install_id_from_system(INSTALL_ID_INVALID);
  touch_nav_app_twin_subscribe(&s_twin_ops, participating, false);
  cl_assert_equal_i(s_twin.install_count, 0);
}

// (c1) reconcile is idempotent, honors the gate, and never double-subscribes.
void test_touch_nav__app_twin_reconcile_subscribe_rules(void) {
  bool participating, opted_in;

  // No transition (already true) -> no install, no remove: never double-subscribe.
  s_twin = (FakeTwin){.pref = true};
  participating = true;
  opted_in = true;
  touch_nav_app_twin_reconcile(&s_twin_ops, &participating, &opted_in, true);
  cl_assert(participating && opted_in);
  cl_assert_equal_i(s_twin.install_count, 0);
  cl_assert_equal_i(s_twin.remove_count, 0);

  // Opt in while both prefs are off: the flags flip but nothing installs yet.
  s_twin = (FakeTwin){.pref = false, .master = false};
  participating = false;
  opted_in = false;
  touch_nav_app_twin_reconcile(&s_twin_ops, &participating, &opted_in, true);
  cl_assert(participating && opted_in);
  cl_assert_equal_i(s_twin.install_count, 0);

  // Opt in while system nav is on: installs exactly once; a second enable(true) is a no-op.
  s_twin = (FakeTwin){.pref = true};
  participating = false;
  opted_in = false;
  touch_nav_app_twin_reconcile(&s_twin_ops, &participating, &opted_in, true);
  cl_assert_equal_i(s_twin.install_count, 1);
  touch_nav_app_twin_reconcile(&s_twin_ops, &participating, &opted_in, true);
  cl_assert_equal_i(s_twin.install_count, 1);

  // Opt in while only the MASTER is on (Touch Navigation sub-pref off): installs -- an explicit
  // opt-in follows the master pref alone.
  s_twin = (FakeTwin){.pref = false, .master = true};
  participating = false;
  opted_in = false;
  touch_nav_app_twin_reconcile(&s_twin_ops, &participating, &opted_in, true);
  cl_assert_equal_i(s_twin.install_count, 1);
}

// (c2) reconcile only unsubscribes on a real opt-out, never when it was never subscribed.
void test_touch_nav__app_twin_reconcile_unsubscribe_rules(void) {
  bool participating, opted_in;

  // Disable from an already-disabled state: no transition -> no unsubscribe.
  s_twin = (FakeTwin){.pref = true};
  participating = false;
  opted_in = false;
  touch_nav_app_twin_reconcile(&s_twin_ops, &participating, &opted_in, false);
  cl_assert(!participating && !opted_in);
  cl_assert_equal_i(s_twin.remove_count, 0);

  // Opt out from enabled: exactly one remove.
  s_twin = (FakeTwin){.pref = true};
  participating = true;
  opted_in = true;
  touch_nav_app_twin_reconcile(&s_twin_ops, &participating, &opted_in, false);
  cl_assert(!participating && !opted_in);
  cl_assert_equal_i(s_twin.remove_count, 1);
}

// (d) resync after a pref flip: keeps an opted-in app installed when only the sub-pref went off,
// removes it once the master goes off, and removes a merely-participating app when system nav is
// off.
void test_touch_nav__app_twin_resync_follows_gate(void) {
  // Opted-in app, sub-pref turns off (system nav off, master still on): stays installed.
  s_twin = (FakeTwin){.pref = false, .master = true};
  touch_nav_app_twin_resync(&s_twin_ops, true, true);
  cl_assert_equal_i(s_twin.install_count, 1);
  cl_assert_equal_i(s_twin.remove_count, 0);

  // Master turns off too: removed.
  s_twin = (FakeTwin){.pref = false, .master = false};
  touch_nav_app_twin_resync(&s_twin_ops, true, true);
  cl_assert_equal_i(s_twin.install_count, 0);
  cl_assert_equal_i(s_twin.remove_count, 1);

  // A merely-participating (system) app with system nav off: removed even with the master on.
  s_twin = (FakeTwin){.pref = false, .master = true};
  touch_nav_app_twin_resync(&s_twin_ops, true, false);
  cl_assert_equal_i(s_twin.install_count, 0);
  cl_assert_equal_i(s_twin.remove_count, 1);
}

// ---------------------------------------------------------------------------------------------
// Criterion 8: a gated Touchdown skips routing/set_failed, leaves the set Possible, ticks `gated`,
// and the next navigational Touchdown works from its first event.

void test_touch_nav__gated_then_navigational(void) {
  prv_dispatch(TouchEvent_Touchdown, 50, 90, true /* non_navigational */);
  cl_assert_equal_i(s_state.counters.gated, 1);
  // Routing/set_failed were skipped: the set is still Possible.
  cl_assert_equal_i(prv_state(s_state.tap), RecognizerState_Possible);
  cl_assert_equal_i(prv_state(s_state.pan), RecognizerState_Possible);
  cl_assert_equal_i(prv_state(s_state.swipe), RecognizerState_Possible);
  cl_assert_equal_i(s_recognizer_manager.state, RecognizerManagerState_WaitForTouchdown);

  // The next navigational Touchdown routes normally, from its very first event.
  prv_dispatch(TouchEvent_Touchdown, 50, 90, false);
  cl_assert_equal_i(s_state.route, TouchNavRoute_Tier2);
  cl_assert_equal_i(prv_state(s_state.pan), RecognizerState_Failed);
  cl_assert_equal_i(s_recognizer_manager.state, RecognizerManagerState_RecognizersActive);
}

// ---------------------------------------------------------------------------------------------
// Tier-1 registry: a registered widget under the touched layer wins the route (all three system
// recognizers are failed so the system set does not emulate).

void test_touch_nav__tier1_widget_wins(void) {
  static TouchNavWidgetNode node;
  node = (TouchNavWidgetNode){0};
  touch_nav_registry_add(&s_state, TouchNavWidgetType_Menu, &node, &s_child_layer, NULL, NULL);
  s_active_layer = &s_child_layer;

  prv_dispatch(TouchEvent_Touchdown, 50, 90, false);
  cl_assert_equal_i(s_state.route, TouchNavRoute_Tier1);
  cl_assert_equal_i(prv_state(s_state.tap), RecognizerState_Failed);
  cl_assert_equal_i(prv_state(s_state.pan), RecognizerState_Failed);
  cl_assert_equal_i(prv_state(s_state.swipe), RecognizerState_Failed);
}

// Registry dedup: re-adding the same node is a no-op (still a single entry), and remove unlinks it.
void test_touch_nav__registry_dedup_and_remove(void) {
  static TouchNavWidgetNode node;
  node = (TouchNavWidgetNode){0};
  touch_nav_registry_add(&s_state, TouchNavWidgetType_Menu, &node, &s_child_layer, NULL, NULL);
  touch_nav_registry_add(&s_state, TouchNavWidgetType_Menu, &node, &s_child_layer, NULL, NULL);
  // Sole widget only if exactly one is registered — proves the re-add did not double-insert.
  s_active_layer = &s_child_layer;
  prv_dispatch(TouchEvent_Touchdown, 50, 90, false);
  cl_assert_equal_i(s_state.route, TouchNavRoute_Tier1);

  // Removing a never-added node is a safe no-op.
  static TouchNavWidgetNode other;
  other = (TouchNavWidgetNode){0};
  touch_nav_registry_remove(&s_state, TouchNavWidgetType_Menu, &other);

  touch_nav_registry_remove(&s_state, TouchNavWidgetType_Menu, &node);
  // With the widget gone the same Touchdown falls through to Tier-2.
  prv_dispatch(TouchEvent_Liftoff, 0, 0, false);
  s_active_layer = NULL;
  prv_dispatch(TouchEvent_Touchdown, 50, 90, false);
  cl_assert_equal_i(s_state.route, TouchNavRoute_Tier2);
}

// Status-bar dead zone with no sole widget → dropped.
void test_touch_nav__dead_zone_dropped(void) {
  prv_dispatch(TouchEvent_Touchdown, 50, 4 /* inside the dead zone */, false);
  cl_assert_equal_i(s_state.route, TouchNavRoute_Dropped);
  cl_assert_equal_i(prv_state(s_state.tap), RecognizerState_Failed);
  cl_assert_equal_i(prv_state(s_state.pan), RecognizerState_Failed);
  cl_assert_equal_i(prv_state(s_state.swipe), RecognizerState_Failed);
}

// Status-bar dead zone with exactly one registered widget (via the Swap registry) that is not
// under the active layer routes to that sole widget (Tier-1), not Dropped. Pins the sole-widget
// branch and the Swap-type registry walk that the y=90 registry tests never reach (they match on
// the parent walk before the dead-zone check).
void test_touch_nav__dead_zone_sole_widget_routes_tier1(void) {
  static TouchNavWidgetNode node;
  node = (TouchNavWidgetNode){0};
  touch_nav_registry_add(&s_state, TouchNavWidgetType_Swap, &node, &s_child_layer, NULL, NULL);
  s_active_layer = NULL;  // the parent walk finds nothing, so the dead-zone branch runs

  prv_dispatch(TouchEvent_Touchdown, 50, 4 /* inside the dead zone */, false);
  cl_assert_equal_i(s_state.route, TouchNavRoute_Tier1);

  touch_nav_registry_remove(&s_state, TouchNavWidgetType_Swap, &node);
}

// The dead-zone boundary is exclusive at TOUCH_NAV_STATUS_BAR_DEAD_ZONE_PX: one pixel inside drops,
// exactly at the threshold is a normal (Tier-2) route.
void test_touch_nav__dead_zone_boundary(void) {
  prv_dispatch(TouchEvent_Touchdown, 50, TOUCH_NAV_STATUS_BAR_DEAD_ZONE_PX - 1 /* inside */, false);
  cl_assert_equal_i(s_state.route, TouchNavRoute_Dropped);
  prv_dispatch(TouchEvent_Liftoff, 0, 0, false);

  prv_dispatch(TouchEvent_Touchdown, 50, TOUCH_NAV_STATUS_BAR_DEAD_ZONE_PX /* at threshold */, false);
  cl_assert_equal_i(s_state.route, TouchNavRoute_Tier2);
}

// The idle timeout is refreshed on a navigational Touchdown, but not on a gated one.
void test_touch_nav__idle_refresh_gated_by_navigation(void) {
  prv_dispatch(TouchEvent_Touchdown, 50, 90, false);
  cl_assert_equal_i(s_fake.idle_refresh_count, 1);
  prv_dispatch(TouchEvent_Liftoff, 0, 0, false);
  prv_dispatch(TouchEvent_Touchdown, 50, 90, true /* gated */);
  cl_assert_equal_i(s_fake.idle_refresh_count, 1);  // unchanged
}

// ---------------------------------------------------------------------------------------------
// П1: ActionBarLayer tap zoning. The bar frame is in global coords; a tap is split vertically into
// three equal zones (top = UP, middle = SELECT, bottom = DOWN). A zone with no icon, a tap outside
// the bar, or an absent snapshot all fall back to SELECT. Swipes are never zoned.

// Pure zone-math + fallback, tested directly on the helper (no dispatcher).
void test_touch_nav__action_bar_zone_math(void) {
  // Bar at x[100,140), y[20,110); h = 90 -> zones of 30px: [20,50) UP, [50,80) SELECT, [80,110) DOWN.
  const TouchNavActionBar bar = {
    .frame = GRect(100, 20, 40, 90),
    .icon_mask = 0x7,  // all three zones have icons
    .present = true,
  };
  // Top zone -> UP.
  cl_assert_equal_i(touch_nav_action_bar_zone_button(&bar, GPoint(120, 20)), BUTTON_ID_UP);
  cl_assert_equal_i(touch_nav_action_bar_zone_button(&bar, GPoint(120, 49)), BUTTON_ID_UP);
  // Middle zone -> SELECT (half-open boundary at 50).
  cl_assert_equal_i(touch_nav_action_bar_zone_button(&bar, GPoint(120, 50)), BUTTON_ID_SELECT);
  cl_assert_equal_i(touch_nav_action_bar_zone_button(&bar, GPoint(120, 79)), BUTTON_ID_SELECT);
  // Bottom zone -> DOWN (half-open boundary at 80; last pixel 109 inside).
  cl_assert_equal_i(touch_nav_action_bar_zone_button(&bar, GPoint(120, 80)), BUTTON_ID_DOWN);
  cl_assert_equal_i(touch_nav_action_bar_zone_button(&bar, GPoint(120, 109)), BUTTON_ID_DOWN);
}

// A zone whose icon bit is clear falls back to SELECT even though the point is in that zone.
void test_touch_nav__action_bar_zone_without_icon_is_select(void) {
  const TouchNavActionBar bar = {
    .frame = GRect(100, 20, 40, 90),
    .icon_mask = 0x6,  // SELECT (bit1) + DOWN (bit2), but no UP (bit0)
    .present = true,
  };
  // UP zone has no icon -> SELECT.
  cl_assert_equal_i(touch_nav_action_bar_zone_button(&bar, GPoint(120, 30)), BUTTON_ID_SELECT);
  // DOWN zone still has its icon.
  cl_assert_equal_i(touch_nav_action_bar_zone_button(&bar, GPoint(120, 95)), BUTTON_ID_DOWN);
}

// A tap outside the bar frame, or on an absent snapshot, is a plain SELECT.
void test_touch_nav__action_bar_outside_and_absent_are_select(void) {
  const TouchNavActionBar bar = {
    .frame = GRect(100, 20, 40, 90),
    .icon_mask = 0x7,
    .present = true,
  };
  // Left of the bar (x < 100) -> SELECT.
  cl_assert_equal_i(touch_nav_action_bar_zone_button(&bar, GPoint(50, 30)), BUTTON_ID_SELECT);
  // Above the bar (y < 20) -> SELECT.
  cl_assert_equal_i(touch_nav_action_bar_zone_button(&bar, GPoint(120, 10)), BUTTON_ID_SELECT);
  // Absent snapshot -> SELECT regardless of the point.
  const TouchNavActionBar absent = { .present = false };
  cl_assert_equal_i(touch_nav_action_bar_zone_button(&absent, GPoint(120, 30)), BUTTON_ID_SELECT);
}

// Through the dispatcher: with a bar snapshot set, a tap inside a zone emulates that zone's button.
void test_touch_nav__action_bar_tap_zones_through_dispatcher(void) {
  const GRect frame = GRect(100, 20, 40, 90);
  touch_nav_set_action_bar(&s_state, &frame, 0x7);

  // Tap in the UP zone.
  prv_tap(120, 30);
  cl_assert_equal_i(s_fake.emit_count, 1);
  cl_assert_equal_i(s_fake.last_emit, BUTTON_ID_UP);

  // Tap in the SELECT zone.
  prv_tap(120, 65);
  cl_assert_equal_i(s_fake.emit_count, 2);
  cl_assert_equal_i(s_fake.last_emit, BUTTON_ID_SELECT);

  // Tap in the DOWN zone.
  prv_tap(120, 95);
  cl_assert_equal_i(s_fake.emit_count, 3);
  cl_assert_equal_i(s_fake.last_emit, BUTTON_ID_DOWN);

  // Tap outside the bar -> plain SELECT.
  prv_tap(50, 65);
  cl_assert_equal_i(s_fake.emit_count, 4);
  cl_assert_equal_i(s_fake.last_emit, BUTTON_ID_SELECT);
}

// With no snapshot present, a tap is a plain SELECT (baseline, unchanged from the bar-less path).
void test_touch_nav__action_bar_absent_tap_is_select(void) {
  // No touch_nav_set_action_bar call: the snapshot is absent from init.
  prv_tap(120, 30);
  cl_assert_equal_i(s_fake.emit_count, 1);
  cl_assert_equal_i(s_fake.last_emit, BUTTON_ID_SELECT);
}

// A swipe over the bar is NOT zoned: it stays a full-screen content-scroll (swipe up -> DOWN).
void test_touch_nav__action_bar_swipe_stays_fullscreen(void) {
  const GRect frame = GRect(100, 20, 40, 90);
  touch_nav_set_action_bar(&s_state, &frame, 0x7);

  // Swipe up entirely within the bar's x-column. Vertical swipe maps by content-scroll, not zone.
  prv_swipe(120, 95, 120, 30);
  cl_assert_equal_i(s_fake.emit_count, 1);
  cl_assert_equal_i(s_fake.last_emit, BUTTON_ID_DOWN);
}

// Clearing the snapshot (bar removed) reverts taps to plain SELECT even inside the old frame.
void test_touch_nav__action_bar_cleared_reverts_to_select(void) {
  const GRect frame = GRect(100, 20, 40, 90);
  touch_nav_set_action_bar(&s_state, &frame, 0x7);
  prv_tap(120, 30);
  cl_assert_equal_i(s_fake.last_emit, BUTTON_ID_UP);

  // Clear (as remove_from_window would): a tap in the old UP zone is now a plain SELECT.
  touch_nav_set_action_bar(&s_state, NULL, 0);
  prv_tap(120, 30);
  cl_assert_equal_i(s_fake.emit_count, 2);
  cl_assert_equal_i(s_fake.last_emit, BUTTON_ID_SELECT);
}

// ---------------------------------------------------------------------------------------------
// Criterion 9: the disable transaction cancel_and_resets both managers, drops the task refcount,
// and runs in the mandated order (2)-(5). The order is recorded as a step sequence.

typedef enum TxnStep {
  Step_Persist,
  Step_KernelSubscribe,
  Step_TakeHold,
  Step_SynthLiftoff,
  Step_KernelCancelResetUnsub,
  Step_AppUnsubscribe,
  Step_ReleaseHold,
} TxnStep;

static TxnStep s_steps[16];
static int s_step_count;
static bool s_last_persist_enable;

static void prv_step(TxnStep step) { s_steps[s_step_count++] = step; }
static void prv_txn_persist(void *ctx, bool enable) {
  s_last_persist_enable = enable;
  prv_step(Step_Persist);
}
static void prv_txn_kernel_subscribe(void *ctx) { prv_step(Step_KernelSubscribe); }
static void prv_txn_take_hold(void *ctx) { prv_step(Step_TakeHold); }
static void prv_txn_synth_liftoff(void *ctx) { prv_step(Step_SynthLiftoff); }
static void prv_txn_kernel_cancel(void *ctx) { prv_step(Step_KernelCancelResetUnsub); }
static void prv_txn_app_unsub(void *ctx) { prv_step(Step_AppUnsubscribe); }
static void prv_txn_release_hold(void *ctx) { prv_step(Step_ReleaseHold); }

static const TouchNavTxnOps s_txn_ops = {
  .persist = prv_txn_persist,
  .kernel_subscribe = prv_txn_kernel_subscribe,
  .take_system_hold = prv_txn_take_hold,
  .synthesize_liftoff = prv_txn_synth_liftoff,
  .kernel_cancel_reset_unsub = prv_txn_kernel_cancel,
  .app_unsubscribe = prv_txn_app_unsub,
  .release_system_hold = prv_txn_release_hold,
};

void test_touch_nav__enable_transaction_order(void) {
  s_step_count = 0;
  touch_nav_transaction_apply(&s_txn_ops, true);
  cl_assert_equal_i(s_step_count, 3);
  cl_assert_equal_i(s_steps[0], Step_Persist);
  cl_assert(s_last_persist_enable);
  cl_assert_equal_i(s_steps[1], Step_KernelSubscribe);
  cl_assert_equal_i(s_steps[2], Step_TakeHold);
}

void test_touch_nav__disable_transaction_order(void) {
  s_step_count = 0;
  touch_nav_transaction_apply(&s_txn_ops, false);
  cl_assert_equal_i(s_step_count, 5);
  cl_assert_equal_i(s_steps[0], Step_Persist);
  cl_assert(!s_last_persist_enable);
  // The mandated (2)-(5) order: synth Liftoff, kernel cancel+unsub, app unsubscribe, release hold.
  cl_assert_equal_i(s_steps[1], Step_SynthLiftoff);
  cl_assert_equal_i(s_steps[2], Step_KernelCancelResetUnsub);
  cl_assert_equal_i(s_steps[3], Step_AppUnsubscribe);
  cl_assert_equal_i(s_steps[4], Step_ReleaseHold);
}

// ---------------------------------------------------------------------------------------------
// Unified widget set: the can_start / declined path (round-3 F1) and gated-pre-emption pan_cancel.
// A fake migrated widget records every vtable call through counters so the exact call sequence is
// observable end to end through touch_nav_dispatch.

typedef struct FakeWidget {
  bool can_start_result;
  int can_start_calls;
  int pan_started_calls;
  int get_base_offset_calls;
  int pan_update_calls;
  int pan_snap_calls;
  int pan_cancel_calls;
  int tap_calls;
  int swipe_calls;
  GPoint base;
} FakeWidget;

static FakeWidget s_widget;

static bool prv_w_can_start(void *w) {
  FakeWidget *fw = w;
  fw->can_start_calls++;
  return fw->can_start_result;
}
static void prv_w_pan_started(void *w) { ((FakeWidget *)w)->pan_started_calls++; }
static GPointReturn prv_w_get_base_offset(void *w) {
  FakeWidget *fw = w;
  fw->get_base_offset_calls++;
  return fw->base;
}
static void prv_w_pan_update(void *w, GPoint base, GPoint delta) {
  ((FakeWidget *)w)->pan_update_calls++;
}
static void prv_w_pan_snap(void *w, GPoint base, GPoint final_delta) {
  ((FakeWidget *)w)->pan_snap_calls++;
}
static void prv_w_pan_cancel(void *w) { ((FakeWidget *)w)->pan_cancel_calls++; }
static void prv_w_tap(void *w, GPoint pt) { ((FakeWidget *)w)->tap_calls++; }
static void prv_w_swipe(void *w, SwipeDirection dir) { ((FakeWidget *)w)->swipe_calls++; }

static const TouchNavWidgetOps s_fake_widget_ops = {
  .can_start = prv_w_can_start,
  .pan_started = prv_w_pan_started,
  .get_base_offset = prv_w_get_base_offset,
  .pan_update = prv_w_pan_update,
  .pan_snap = prv_w_pan_snap,
  .pan_cancel = prv_w_pan_cancel,
  .tap = prv_w_tap,
  .swipe = prv_w_swipe,
};

// Register the fake widget as a migrated (ops-bearing) node under the touched layer.
static void prv_register_fake_widget(TouchNavWidgetNode *node) {
  s_widget = (FakeWidget){0};
  *node = (TouchNavWidgetNode){0};
  touch_nav_registry_add(&s_state, TouchNavWidgetType_Menu, node, &s_child_layer, &s_fake_widget_ops,
                         &s_widget);
  s_active_layer = &s_child_layer;
}

// A Touchdown that woke the screen (wake_touch stamped): the gesture must not tap or swipe, but a
// follow-on drag is deliberate, now-visible input — the widget pan drives scrolling in the SAME
// gesture instead of demanding a lift and re-touch.
void test_touch_nav__wake_touchdown_pans_but_never_taps(void) {
  static TouchNavWidgetNode node;
  prv_register_fake_widget(&node);
  s_widget.can_start_result = true;

  const TouchEvent td = {.type = TouchEvent_Touchdown, .x = 50, .y = 150, .wake_touch = true};
  touch_nav_dispatch(&td, &s_state);
  cl_assert_equal_i(s_state.route, TouchNavRoute_Tier1);
  // Taps and swipes are dead for this gesture, in both recognizer sets.
  cl_assert_equal_i(prv_state(s_state.tap), RecognizerState_Failed);
  cl_assert_equal_i(prv_state(s_state.swipe), RecognizerState_Failed);
  cl_assert_equal_i(prv_state(s_state.widget_tap), RecognizerState_Failed);
  cl_assert_equal_i(prv_state(s_state.widget_swipe), RecognizerState_Failed);
  // The widget pan is alive and a drag drives the widget.
  cl_assert(prv_state(s_state.widget_pan) != RecognizerState_Failed);
  prv_advance_ms(20);
  const TouchEvent pu = {.type = TouchEvent_PositionUpdate, .x = 50, .y = 100, .wake_touch = true};
  touch_nav_dispatch(&pu, &s_state);
  cl_assert_equal_i(s_widget.pan_started_calls, 1);
  const TouchEvent lo = {.type = TouchEvent_Liftoff, .wake_touch = true};
  touch_nav_dispatch(&lo, &s_state);
  cl_assert_equal_i(s_widget.pan_snap_calls, 1);
  cl_assert_equal_i(s_widget.tap_calls, 0);
  cl_assert_equal_i(s_widget.swipe_calls, 0);
  cl_assert_equal_i(s_fake.emit_count, 0);
}

// A stationary wake tap completes nothing anywhere: no widget tap, no bridge emulation.
void test_touch_nav__wake_tap_is_inert(void) {
  static TouchNavWidgetNode node;
  prv_register_fake_widget(&node);
  s_widget.can_start_result = true;

  const TouchEvent td = {.type = TouchEvent_Touchdown, .x = 50, .y = 90, .wake_touch = true};
  touch_nav_dispatch(&td, &s_state);
  prv_advance_ms(30);
  const TouchEvent lo = {.type = TouchEvent_Liftoff, .wake_touch = true};
  touch_nav_dispatch(&lo, &s_state);
  cl_assert_equal_i(s_widget.tap_calls, 0);
  cl_assert_equal_i(s_fake.emit_count, 0);
  cl_assert_equal_i(s_fake.pop_count, 0);
}

// can_start returning false declines the WHOLE gesture: no pan_started/get_base_offset, the unified
// pan is NOT set_failed/cancelled, later events drive nothing and never re-latch, and `declined`
// resets on the next Touchdown so a following can_start=true gesture drives the widget normally.
void test_touch_nav__widget_can_start_decline_then_accept(void) {
  static TouchNavWidgetNode node;
  prv_register_fake_widget(&node);

  // Gesture 1: the widget refuses to start.
  s_widget.can_start_result = false;
  prv_dispatch(TouchEvent_Touchdown, 50, 90, false);
  cl_assert_equal_i(s_state.route, TouchNavRoute_Tier1);
  cl_assert_equal_p(s_state.latched_target, &node);
  cl_assert(!s_state.declined);

  prv_advance_ms(20);
  prv_dispatch(TouchEvent_PositionUpdate, 50, 55, false);  // 35px up -> pan Starts -> can_start()
  cl_assert(s_state.declined);
  cl_assert_equal_i(s_widget.can_start_calls, 1);
  // (a) no set_failed / cancel of the unified pan: it is still live (Started/Updated), not Failed.
  cl_assert(prv_state(s_state.widget_pan) != RecognizerState_Failed);
  // (b) no pan_started / get_base_offset ran (declined before either).
  cl_assert_equal_i(s_widget.pan_started_calls, 0);
  cl_assert_equal_i(s_widget.get_base_offset_calls, 0);

  prv_advance_ms(20);
  prv_dispatch(TouchEvent_PositionUpdate, 50, 25, false);  // Updated: gated by `declined`
  // (b) no pan_update; (c) the target is NOT re-resolved/re-latched (no teleport).
  cl_assert_equal_i(s_widget.pan_update_calls, 0);
  cl_assert_equal_p(s_state.latched_target, &node);

  prv_dispatch(TouchEvent_Liftoff, 0, 0, false);           // Completed: dropped
  cl_assert_equal_i(s_widget.pan_snap_calls, 0);
  cl_assert_equal_i(s_widget.pan_cancel_calls, 0);
  cl_assert_equal_p(s_state.latched_target, NULL);         // cleared on completion
  cl_assert(!s_state.declined);                            // reset for the next gesture

  // (d) Gesture 2: can_start now accepts -> a full pan drives the widget.
  s_widget.can_start_result = true;
  s_widget.base = GPoint(0, -5);
  prv_dispatch(TouchEvent_Touchdown, 50, 90, false);
  cl_assert(!s_state.declined);
  prv_advance_ms(20);
  prv_dispatch(TouchEvent_PositionUpdate, 50, 55, false);  // pan Starts -> accepted
  cl_assert_equal_i(s_widget.can_start_calls, 2);
  cl_assert_equal_i(s_widget.pan_started_calls, 1);
  cl_assert_equal_i(s_widget.get_base_offset_calls, 1);
  prv_advance_ms(20);
  prv_dispatch(TouchEvent_PositionUpdate, 50, 25, false);  // Updated -> live pan_update
  cl_assert_equal_i(s_widget.pan_update_calls, 1);
  prv_dispatch(TouchEvent_Liftoff, 0, 0, false);           // Completed -> pan_snap
  cl_assert_equal_i(s_widget.pan_snap_calls, 1);

  touch_nav_registry_remove(&s_state, TouchNavWidgetType_Menu, &node);
}

// A non-navigational (wake / DnD) Touchdown pre-empting an in-flight pan drops the gesture cleanly:
// the latch and `declined` are cleared and NO snap runs, so the pan is not committed. It does NOT run
// ops->pan_cancel -- the pan recognizer's cancel op returns false (pan.c) and it never
// self-transitions to Cancelled, so cancel_and_reset delivers no Cancelled event to the unified pan;
// this matches the per-widget behaviour before the refactor (Ф1 is no-behaviour-change). The next
// gesture still drives the widget, proving routing recovered.
void test_touch_nav__gated_preemption_drops_active_pan(void) {
  static TouchNavWidgetNode node;
  prv_register_fake_widget(&node);
  s_widget.can_start_result = true;

  prv_dispatch(TouchEvent_Touchdown, 50, 90, false);
  prv_advance_ms(20);
  prv_dispatch(TouchEvent_PositionUpdate, 50, 55, false);  // pan Started, widget is the live target
  cl_assert_equal_i(s_widget.pan_started_calls, 1);
  cl_assert_equal_p(s_state.latched_target, &node);

  // Gated pre-emption mid-pan: the gesture is unwound with no snap and no (unreachable) pan_cancel.
  prv_dispatch(TouchEvent_Touchdown, 50, 90, true /* non_navigational */);
  cl_assert_equal_i(s_widget.pan_snap_calls, 0);
  cl_assert_equal_i(s_widget.pan_cancel_calls, 0);
  cl_assert_equal_p(s_state.latched_target, NULL);  // latch cleared
  cl_assert(!s_state.declined);
  cl_assert_equal_i(s_recognizer_manager.state, RecognizerManagerState_WaitForTouchdown);

  // Routing recovered: a fresh gesture drives the widget normally.
  prv_dispatch(TouchEvent_Touchdown, 50, 90, false);
  prv_advance_ms(20);
  prv_dispatch(TouchEvent_PositionUpdate, 50, 55, false);  // pan Started again
  cl_assert_equal_i(s_widget.pan_started_calls, 2);
  prv_advance_ms(20);
  prv_dispatch(TouchEvent_PositionUpdate, 50, 25, false);  // Updated -> live pan
  cl_assert_equal_i(s_widget.pan_update_calls, 1);
  prv_dispatch(TouchEvent_Liftoff, 0, 0, false);
  cl_assert_equal_i(s_widget.pan_snap_calls, 1);

  touch_nav_registry_remove(&s_state, TouchNavWidgetType_Menu, &node);
}

// The typed getters are SDK-exported: handed a NULL or wrong-type recognizer (all three types live
// on this state) they must return a neutral value, never deref the mismatched impl data.
void test_touch_nav__typed_getters_reject_wrong_type(void) {
  const GPoint zero = GPointZero;

  GPoint p = tap_recognizer_get_tap_point(s_state.pan);
  cl_assert_equal_i(p.x, zero.x);
  cl_assert_equal_i(p.y, zero.y);
  p = pan_recognizer_get_total_delta(s_state.tap);
  cl_assert_equal_i(p.x, zero.x);
  cl_assert_equal_i(p.y, zero.y);
  p = pan_recognizer_get_delta_since_start(NULL);
  cl_assert_equal_i(p.x, zero.x);
  cl_assert_equal_i(p.y, zero.y);
  p = pan_recognizer_get_velocity(s_state.swipe);
  cl_assert_equal_i(p.x, zero.x);
  cl_assert_equal_i(p.y, zero.y);
  cl_assert_equal_i(swipe_recognizer_get_direction(s_state.tap), SwipeDirection_None);
  p = swipe_recognizer_get_velocity(NULL);
  cl_assert_equal_i(p.x, zero.x);
  cl_assert_equal_i(p.y, zero.y);
  p = tap_recognizer_get_tap_point(NULL);
  cl_assert_equal_i(p.x, zero.x);
  cl_assert_equal_i(p.y, zero.y);
}
