/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "kernel/events.h"
#include "kernel/pebble_tasks.h"
#include "pbl/services/event_service.h"
#include "pbl/services/touch/touch.h"
#include "pbl/services/touch/touch_event.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fake_events.h"

// Stubs
#include "stubs_analytics.h"
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"

void kernel_free(void *p) {}

// Declared in syscall/syscall.h; in the test build DEFINE_SYSCALL is a plain function.
void sys_touch_set_raw_subscribed(bool subscribed);

// sys_touch_set_raw_subscribed marks the calling task; make it settable.
static PebbleTask s_current_task = PebbleTask_App;
PebbleTask pebble_task_get_current(void) {
  return s_current_task;
}

static EventServiceAddSubscriberCallback s_add_subscriber_cb;
static EventServiceRemoveSubscriberCallback s_remove_subscriber_cb;

void event_service_init(PebbleEventType type, EventServiceAddSubscriberCallback add_cb,
                        EventServiceRemoveSubscriberCallback remove_cb) {
  cl_assert(type == PEBBLE_TOUCH_EVENT || type == PEBBLE_GESTURE_EVENT);
  s_add_subscriber_cb = add_cb;
  s_remove_subscriber_cb = remove_cb;
}

static int s_touch_sensor_enable_count;
static int s_touch_sensor_disable_count;
static bool s_touch_sensor_enabled;

void touch_sensor_set_enabled(bool enabled) {
  if (enabled) {
    s_touch_sensor_enable_count++;
  } else {
    s_touch_sensor_disable_count++;
  }
  s_touch_sensor_enabled = enabled;
}

// setup and teardown
void test_touch__initialize(void) {
  fake_event_init();
  s_add_subscriber_cb = NULL;
  s_remove_subscriber_cb = NULL;
  s_touch_sensor_enable_count = 0;
  s_touch_sensor_disable_count = 0;
  s_touch_sensor_enabled = false;
  touch_init();
  touch_reset();
  // Make sure the global kill switch is reset between tests — it's a module
  // static in touch.c and a failed test could otherwise leak its state.
  touch_service_set_globally_enabled(true);
  // Nav pref is a module static too; default it off between tests.
  touch_set_nav_enabled(false);
  // The raw-slot mark is a module static; clear the App task's bit between tests.
  s_current_task = PebbleTask_App;
  sys_touch_set_raw_subscribed(false);
}

void test_touch__cleanup(void) {
}

static void prv_assert_touch_event(TouchEventType type, int16_t x, int16_t y) {
  PebbleEvent event = fake_event_get_last();
  cl_assert_equal_i(event.type, PEBBLE_TOUCH_EVENT);
  cl_assert_equal_i(event.touch.event.type, type);
  cl_assert_equal_i(event.touch.event.x, x);
  cl_assert_equal_i(event.touch.event.y, y);
}

// tests
void test_touch__touchdown(void) {
  touch_handle_update(TouchState_FingerDown, 15, 100);
  cl_assert_equal_i(fake_event_get_count(), 1);
  prv_assert_touch_event(TouchEvent_Touchdown, 15, 100);
}

void test_touch__liftoff(void) {
  touch_handle_update(TouchState_FingerDown, 15, 100);
  touch_handle_update(TouchState_FingerUp, 20, 120);
  cl_assert_equal_i(fake_event_get_count(), 2);
  prv_assert_touch_event(TouchEvent_Liftoff, 20, 120);
}

void test_touch__position_update(void) {
  touch_handle_update(TouchState_FingerDown, 10, 10);
  touch_handle_update(TouchState_FingerDown, 13, 13);
  cl_assert_equal_i(fake_event_get_count(), 2);
  prv_assert_touch_event(TouchEvent_PositionUpdate, 13, 13);

  touch_handle_update(TouchState_FingerDown, 18, 5);
  cl_assert_equal_i(fake_event_get_count(), 3);
  prv_assert_touch_event(TouchEvent_PositionUpdate, 18, 5);
}

void test_touch__position_stationary(void) {
  touch_handle_update(TouchState_FingerDown, 10, 10);
  fake_event_reset_count();
  // No event generated when position is unchanged
  touch_handle_update(TouchState_FingerDown, 10, 10);
  cl_assert_equal_i(fake_event_get_count(), 0);
}

void test_touch__no_event_when_idle(void) {
  // Liftoff while already up produces no event
  touch_handle_update(TouchState_FingerUp, 0, 0);
  cl_assert_equal_i(fake_event_get_count(), 0);
}

void test_touch__reset_clears_state(void) {
  touch_handle_update(TouchState_FingerDown, 10, 10);
  touch_reset();
  fake_event_reset_count();

  // After reset, a FingerDown update should emit a Touchdown (not PositionUpdate)
  touch_handle_update(TouchState_FingerDown, 50, 50);
  cl_assert_equal_i(fake_event_get_count(), 1);
  prv_assert_touch_event(TouchEvent_Touchdown, 50, 50);
}

void test_touch__subscriber_enables_sensor(void) {
  cl_assert(s_add_subscriber_cb != NULL);
  cl_assert(s_remove_subscriber_cb != NULL);

  s_add_subscriber_cb(PebbleTask_App);
  cl_assert_equal_i(s_touch_sensor_enable_count, 1);
  cl_assert(s_touch_sensor_enabled);

  // Additional subscribers don't re-enable the sensor
  s_add_subscriber_cb(PebbleTask_App);
  cl_assert_equal_i(s_touch_sensor_enable_count, 1);

  // Sensor stays enabled until the last subscriber leaves
  s_remove_subscriber_cb(PebbleTask_App);
  cl_assert_equal_i(s_touch_sensor_disable_count, 0);
  cl_assert(s_touch_sensor_enabled);

  s_remove_subscriber_cb(PebbleTask_App);
  cl_assert_equal_i(s_touch_sensor_disable_count, 1);
  cl_assert(!s_touch_sensor_enabled);
}

void test_touch__backlight_toggles_sensor(void) {
  touch_set_backlight_enabled(true);
  cl_assert_equal_i(s_touch_sensor_enable_count, 1);
  cl_assert(s_touch_sensor_enabled);

  // Idempotent: enabling again is a no-op
  touch_set_backlight_enabled(true);
  cl_assert_equal_i(s_touch_sensor_enable_count, 1);

  touch_set_backlight_enabled(false);
  cl_assert_equal_i(s_touch_sensor_disable_count, 1);
  cl_assert(!s_touch_sensor_enabled);

  // Idempotent: disabling again is a no-op
  touch_set_backlight_enabled(false);
  cl_assert_equal_i(s_touch_sensor_disable_count, 1);
}

void test_touch__backlight_and_app_share_sensor(void) {
  touch_set_backlight_enabled(true);
  cl_assert_equal_i(s_touch_sensor_enable_count, 1);

  // App subscription while backlight already holds the sensor: no extra enable
  s_add_subscriber_cb(PebbleTask_App);
  cl_assert_equal_i(s_touch_sensor_enable_count, 1);

  // Dropping the backlight while an app subscriber remains keeps the sensor on
  touch_set_backlight_enabled(false);
  cl_assert_equal_i(s_touch_sensor_disable_count, 0);
  cl_assert(s_touch_sensor_enabled);

  s_remove_subscriber_cb(PebbleTask_App);
  cl_assert_equal_i(s_touch_sensor_disable_count, 1);
  cl_assert(!s_touch_sensor_enabled);
}

void test_touch__has_app_subscribers_app(void) {
  cl_assert(!touch_has_app_subscribers());

  // Only the explicit raw-slot mark (touch_service_subscribe) counts.
  sys_touch_set_raw_subscribed(true);
  cl_assert(touch_has_app_subscribers());

  sys_touch_set_raw_subscribed(false);
  cl_assert(!touch_has_app_subscribers());
}

void test_touch__has_app_subscribers_ignores_shared_subscriptions(void) {
  // The nav twins' system-slot handlers subscribe through the same per-task
  // event-service subscription as raw handlers; those subscriptions must NOT
  // read as app subscribers, or wake-on-every-touch comes back with menu
  // gestures off.
  s_add_subscriber_cb(PebbleTask_KernelMain);  // modal twin
  s_add_subscriber_cb(PebbleTask_App);         // app twin
  cl_assert(!touch_has_app_subscribers());
  s_remove_subscriber_cb(PebbleTask_App);
  s_remove_subscriber_cb(PebbleTask_KernelMain);
}

void test_touch__has_app_subscribers_cleared_on_subscription_death(void) {
  // A dead app cannot unsubscribe: when its shared subscription is torn down
  // (event-service task cleanup), the raw-slot mark must be dropped with it.
  sys_touch_set_raw_subscribed(true);
  s_add_subscriber_cb(PebbleTask_App);
  cl_assert(touch_has_app_subscribers());

  s_remove_subscriber_cb(PebbleTask_App);
  cl_assert(!touch_has_app_subscribers());
}

void test_touch__has_app_subscribers_backlight(void) {
  // The backlight subscription must not register as an app subscriber: the
  // event loop uses touch_has_app_subscribers() as an override that fires the
  // backlight on any gesture, so counting the backlight's own subscription
  // would defeat the wake-gesture filter.
  cl_assert(!touch_has_app_subscribers());

  touch_set_backlight_enabled(true);
  cl_assert(!touch_has_app_subscribers());

  // With an app also subscribed, the call reflects the app, regardless of the
  // backlight subscription state.
  sys_touch_set_raw_subscribed(true);
  cl_assert(touch_has_app_subscribers());

  touch_set_backlight_enabled(false);
  cl_assert(touch_has_app_subscribers());

  sys_touch_set_raw_subscribed(false);
  cl_assert(!touch_has_app_subscribers());
}

void test_touch__has_app_subscribers_nav(void) {
  // Nav off + only the backlight hold: the backlight subscription must not
  // register as an app subscriber.
  touch_set_backlight_enabled(true);
  cl_assert(!touch_has_app_subscribers());

  // The nav gate reports app subscribers regardless of the actual subscriber
  // set. The shell only raises it when nav is effectively on (master pref AND
  // the touch-navigation sub-pref).
  touch_set_nav_enabled(true);
  cl_assert(touch_has_app_subscribers());

  // Back off, and with only the backlight hold it's false again.
  touch_set_nav_enabled(false);
  cl_assert(!touch_has_app_subscribers());

  // Nav off + the nav system hold (on top of backlight): the hold must be
  // excluded too, otherwise tap-to-wake is wrongly suppressed once the shell
  // decouples the pref from the hold.
  touch_set_system_hold(true);
  cl_assert(!touch_has_app_subscribers());
  touch_set_system_hold(false);

  // Nav off + a real raw subscriber → true.
  sys_touch_set_raw_subscribed(true);
  cl_assert(touch_has_app_subscribers());

  sys_touch_set_raw_subscribed(false);
  touch_set_backlight_enabled(false);
}

void test_touch__system_hold_holds_sensor(void) {
  // The permanent hold powers the sensor directly, without an event-service
  // subscription.
  touch_set_system_hold(true);
  cl_assert_equal_i(s_touch_sensor_enable_count, 1);
  cl_assert(s_touch_sensor_enabled);

  // Idempotent: holding again is a no-op.
  touch_set_system_hold(true);
  cl_assert_equal_i(s_touch_sensor_enable_count, 1);

  touch_set_system_hold(false);
  cl_assert_equal_i(s_touch_sensor_disable_count, 1);
  cl_assert(!s_touch_sensor_enabled);

  // Idempotent: releasing again is a no-op.
  touch_set_system_hold(false);
  cl_assert_equal_i(s_touch_sensor_disable_count, 1);
}

void test_touch__globally_enabled_default_true(void) {
  // Default state after init is enabled; no setter call required.
  cl_assert(touch_service_is_globally_enabled());
}

void test_touch__global_disable_drops_events(void) {
  touch_service_set_globally_enabled(false);
  touch_handle_update(TouchState_FingerDown, 10, 20);
  cl_assert_equal_i(fake_event_get_count(), 0);

  touch_handle_update(TouchState_FingerUp, 10, 20);
  cl_assert_equal_i(fake_event_get_count(), 0);

  touch_service_set_globally_enabled(true);
}

void test_touch__global_disable_powers_down_sensor(void) {
  // Subscriber brings the sensor up.
  s_add_subscriber_cb(PebbleTask_App);
  cl_assert(s_touch_sensor_enabled);
  cl_assert_equal_i(s_touch_sensor_enable_count, 1);

  // Disabling globally powers the sensor down even though a subscriber remains.
  touch_service_set_globally_enabled(false);
  cl_assert(!s_touch_sensor_enabled);
  cl_assert_equal_i(s_touch_sensor_disable_count, 1);

  // Re-enabling brings it back up for the existing subscriber.
  touch_service_set_globally_enabled(true);
  cl_assert(s_touch_sensor_enabled);
  cl_assert_equal_i(s_touch_sensor_enable_count, 2);

  s_remove_subscriber_cb(PebbleTask_App);
}

void test_touch__subscribe_while_disabled_keeps_sensor_off(void) {
  touch_service_set_globally_enabled(false);

  // Subscribing while disabled must not power the sensor up.
  s_add_subscriber_cb(PebbleTask_App);
  cl_assert(!s_touch_sensor_enabled);
  cl_assert_equal_i(s_touch_sensor_enable_count, 0);

  // Re-enabling globally powers it up for the pre-existing subscriber.
  touch_service_set_globally_enabled(true);
  cl_assert(s_touch_sensor_enabled);
  cl_assert_equal_i(s_touch_sensor_enable_count, 1);

  s_remove_subscriber_cb(PebbleTask_App);
}

void test_touch__global_disable_drops_gestures(void) {
  // Gestures (tap/double-tap) must honor the kill switch too, not just the
  // finger up/down updates.
  touch_service_set_globally_enabled(false);

  touch_handle_gesture(TouchGesture_Tap, 10, 20);
  cl_assert_equal_i(fake_event_get_count(), 0);

  touch_handle_gesture(TouchGesture_DoubleTap, 30, 40);
  cl_assert_equal_i(fake_event_get_count(), 0);

  touch_service_set_globally_enabled(true);
}

void test_touch__gestures_delivered_when_enabled(void) {
  touch_handle_gesture(TouchGesture_Tap, 10, 20);
  cl_assert_equal_i(fake_event_get_count(), 1);

  PebbleEvent event = fake_event_get_last();
  cl_assert_equal_i(event.type, PEBBLE_GESTURE_EVENT);
  cl_assert_equal_i(event.gesture.event.type, GestureEvent_Tap);
}

void test_touch__global_disable_sleeps_unsubscribed_sensor(void) {
  // The driver can re-arm the sensor outside the service's subscriber
  // bookkeeping (e.g. its I2C error-recovery path). Disabling globally must
  // still put the chip to sleep, even when s_subscriber_count is 0.
  s_touch_sensor_enabled = true;

  touch_service_set_globally_enabled(false);
  cl_assert(!s_touch_sensor_enabled);
  cl_assert(s_touch_sensor_disable_count >= 1);

  touch_service_set_globally_enabled(true);
}

void test_touch__wake_gate_formula(void) {
  // (before=F, after=T) -> woke the screen -> latch.
  cl_assert(touch_wake_gate_on_touchdown(true, false, false, true).latch);
  // Already on (T, T) -> navigation.
  cl_assert(!touch_wake_gate_on_touchdown(true, false, true, true).latch);
  // Off and stayed off (F, F), no DnD (day/ALS) -> navigation.
  cl_assert(!touch_wake_gate_on_touchdown(true, false, false, false).latch);
  // DnD suppressed the wake while off -> still a wake tap -> latch.
  cl_assert(touch_wake_gate_on_touchdown(true, true, false, false).latch);
  // DnD but screen already on -> navigation.
  cl_assert(!touch_wake_gate_on_touchdown(true, true, true, true).latch);
}

void test_touch__wake_gate_guard_matrix(void) {
  // No backlight driver (gesture-wake mode): a touch that begins with the
  // screen off can only be a wake attempt -- it must not act invisibly on the
  // UI, so it latches non-navigational regardless of `after`/DnD.
  cl_assert(touch_wake_gate_on_touchdown(false, false, false, false).latch);
  cl_assert(touch_wake_gate_on_touchdown(false, false, false, true).latch);
  cl_assert(touch_wake_gate_on_touchdown(false, true, false, false).latch);
  // Screen already on: the touch navigates.
  cl_assert(!touch_wake_gate_on_touchdown(false, false, true, true).latch);
  cl_assert(!touch_wake_gate_on_touchdown(false, true, true, true).latch);

  // Driven, no DnD: latch strictly by the formula.
  TouchWakeGateResult driven = touch_wake_gate_on_touchdown(true, false, false, true);
  cl_assert(driven.latch);

  // Driven, DnD: latch == !before.
  cl_assert(touch_wake_gate_on_touchdown(true, true, false, false).latch);
  cl_assert(!touch_wake_gate_on_touchdown(true, true, true, true).latch);
}

void test_touch__wake_gate_latches_across_gesture(void) {
  // A wake Touchdown stamps non_navigational and latches it for the gesture.
  TouchWakeGateResult woke = touch_wake_gate_on_touchdown(true, false, false, true);
  TouchEvent td = {.type = TouchEvent_Touchdown};
  touch_wake_gate_stamp(&td, woke);
  cl_assert(td.non_navigational);

  // PositionUpdate and Liftoff carry the latch, regardless of their gate arg.
  TouchEvent pu = {.type = TouchEvent_PositionUpdate};
  touch_wake_gate_stamp(&pu, (TouchWakeGateResult){0});
  cl_assert(pu.non_navigational);

  TouchEvent lo = {.type = TouchEvent_Liftoff};
  touch_wake_gate_stamp(&lo, (TouchWakeGateResult){0});
  cl_assert(lo.non_navigational);

  // A fresh navigational Touchdown clears the latch for the next gesture.
  TouchWakeGateResult nav = touch_wake_gate_on_touchdown(true, false, false, false);
  TouchEvent td2 = {.type = TouchEvent_Touchdown};
  touch_wake_gate_stamp(&td2, nav);
  cl_assert(!td2.non_navigational);

  TouchEvent pu2 = {.type = TouchEvent_PositionUpdate};
  touch_wake_gate_stamp(&pu2, (TouchWakeGateResult){0});
  cl_assert(!pu2.non_navigational);
}

void test_touch__toggle_off_with_finger_down_emits_liftoff(void) {
  // Finger down, then the global toggle goes off: a Liftoff must be synthesized
  // with the last coordinates (not zeros) so the backlight hold unwinds.
  touch_handle_update(TouchState_FingerDown, 30, 40);
  fake_event_reset_count();

  touch_service_set_globally_enabled(false);
  cl_assert_equal_i(fake_event_get_count(), 1);
  prv_assert_touch_event(TouchEvent_Liftoff, 30, 40);

  touch_service_set_globally_enabled(true);
}

void test_touch__toggle_off_without_finger_no_liftoff(void) {
  // No finger down: toggling off must not fabricate a Liftoff.
  fake_event_reset_count();
  touch_service_set_globally_enabled(false);
  cl_assert_equal_i(fake_event_get_count(), 0);
  touch_service_set_globally_enabled(true);
}

void test_touch__event_abi_unchanged(void) {
  // The wake flag rides in the padding after type:8; x/y offsets and the
  // overall size must not move, keeping the SDK struct app-compatible.
  cl_assert_equal_i(offsetof(TouchEvent, x), 2);
  cl_assert_equal_i(offsetof(TouchEvent, y), 4);
  cl_assert(sizeof(TouchEvent) <= 9);
}
