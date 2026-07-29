/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "applib/touch_service.h"
#include "applib/touch_service_private.h"
#include "kernel/events.h"
#include "kernel/pebble_tasks.h"

#include <stdbool.h>
#include <stdint.h>

// Stubs
#include "stubs_logging.h"
#include "stubs_passert.h"

// Fake event service
//////////////////////////////////////////
static int s_subscribe_count;
static int s_unsubscribe_count;
static EventServiceInfo *s_subscribed_info;

void event_service_client_subscribe(EventServiceInfo *info) {
  s_subscribe_count++;
  s_subscribed_info = info;
}

void event_service_client_unsubscribe(EventServiceInfo *info) {
  s_unsubscribe_count++;
  cl_assert_equal_p(info, s_subscribed_info);
  s_subscribed_info = NULL;
}

// Task / state plumbing so prv_get_state() resolves to our test state
//////////////////////////////////////////
static TouchServiceState s_state;

PebbleTask pebble_task_get_current(void) {
  return PebbleTask_App;
}

bool sys_app_is_watchface(void) {
  return false;
}

TouchServiceState *app_state_get_touch_service_state(void) {
  return &s_state;
}

TouchServiceState *kernel_applib_get_touch_service_state(void) {
  return &s_state;
}

static int s_touch_reset_count;
void sys_touch_reset(void) {
  s_touch_reset_count++;
}

static bool s_raw_subscribed;
void sys_touch_set_raw_subscribed(bool subscribed) {
  s_raw_subscribed = subscribed;
}

bool sys_touch_service_is_enabled(void) {
  return true;
}

void sys_app_touch_navigation_enable(bool enable) {
  (void)enable;
}

// Handler bookkeeping
//////////////////////////////////////////
typedef struct HandlerRecord {
  int calls;
  TouchEvent last_event;
  void *last_context;
} HandlerRecord;

static HandlerRecord s_system_rec;
static HandlerRecord s_raw_rec;

static int s_system_marker;
static int s_raw_marker;

static void prv_system_handler(const TouchEvent *event, void *context) {
  s_system_rec.calls++;
  s_system_rec.last_event = *event;
  s_system_rec.last_context = context;
}

static void prv_raw_handler(const TouchEvent *event, void *context) {
  s_raw_rec.calls++;
  s_raw_rec.last_event = *event;
  s_raw_rec.last_context = context;
}

// Ordering probes: record the global sequence position at which each fires.
static int s_seq;
static int s_system_seq;
static int s_raw_seq;

static void prv_order_system_handler(const TouchEvent *event, void *context) {
  s_system_seq = ++s_seq;
}

static void prv_order_raw_handler(const TouchEvent *event, void *context) {
  s_raw_seq = ++s_seq;
}

static void prv_deliver_touch(TouchEventType type, int16_t x, int16_t y) {
  cl_assert(s_subscribed_info != NULL);
  PebbleEvent e = {
    .type = PEBBLE_TOUCH_EVENT,
    .touch = {
      .event = {
        .type = type,
        .x = x,
        .y = y,
      },
    },
  };
  s_subscribed_info->handler(&e, s_subscribed_info->context);
}

// setup / teardown
//////////////////////////////////////////
void test_touch_service__initialize(void) {
  s_subscribe_count = 0;
  s_unsubscribe_count = 0;
  s_subscribed_info = NULL;
  s_touch_reset_count = 0;
  s_system_rec = (HandlerRecord){ 0 };
  s_raw_rec = (HandlerRecord){ 0 };
  touch_service_state_init(&s_state);
}

void test_touch_service__cleanup(void) {
}

// tests
//////////////////////////////////////////

// 1. Both slots receive one event, each with its own context.
void test_touch_service__both_slots_receive_event(void) {
  touch_service_set_system_handler(prv_system_handler, &s_system_marker);
  touch_service_subscribe(prv_raw_handler, &s_raw_marker);

  prv_deliver_touch(TouchEvent_Touchdown, 12, 34);

  cl_assert_equal_i(s_system_rec.calls, 1);
  cl_assert_equal_i(s_raw_rec.calls, 1);
  cl_assert_equal_p(s_system_rec.last_context, &s_system_marker);
  cl_assert_equal_p(s_raw_rec.last_context, &s_raw_marker);
  cl_assert_equal_i(s_system_rec.last_event.x, 12);
  cl_assert_equal_i(s_raw_rec.last_event.y, 34);
}

// System handler runs before the raw handler.
void test_touch_service__system_runs_before_raw(void) {
  s_seq = 0;
  s_system_seq = 0;
  s_raw_seq = 0;

  touch_service_set_system_handler(prv_order_system_handler, &s_system_marker);
  touch_service_subscribe(prv_order_raw_handler, &s_raw_marker);
  prv_deliver_touch(TouchEvent_Liftoff, 1, 2);

  cl_assert(s_system_seq > 0);
  cl_assert(s_raw_seq > 0);
  cl_assert(s_system_seq < s_raw_seq);
}

// 2. App subscribe/unsubscribe does not touch the system slot and vice-versa.
void test_touch_service__slots_are_independent(void) {
  touch_service_set_system_handler(prv_system_handler, &s_system_marker);
  touch_service_subscribe(prv_raw_handler, &s_raw_marker);

  // Unsubscribing the app clears only the raw slot.
  touch_service_unsubscribe();
  cl_assert(s_state.raw_handler == NULL);
  cl_assert(s_state.system_handler == prv_system_handler);

  // A delivered event now reaches only the system slot.
  prv_deliver_touch(TouchEvent_Touchdown, 5, 6);
  cl_assert_equal_i(s_system_rec.calls, 1);
  cl_assert_equal_i(s_raw_rec.calls, 0);

  // Re-subscribe the app, then clear the system slot: the raw slot survives.
  touch_service_subscribe(prv_raw_handler, &s_raw_marker);
  touch_service_set_system_handler(NULL, NULL);
  cl_assert(s_state.system_handler == NULL);
  cl_assert(s_state.raw_handler == prv_raw_handler);

  prv_deliver_touch(TouchEvent_Touchdown, 7, 8);
  cl_assert_equal_i(s_system_rec.calls, 1);  // unchanged
  cl_assert_equal_i(s_raw_rec.calls, 1);
}

// 3. Event-service subscription is created once and the refcount doesn't leak.
void test_touch_service__subscription_created_once(void) {
  // First occupied slot subscribes.
  touch_service_set_system_handler(prv_system_handler, &s_system_marker);
  cl_assert_equal_i(s_subscribe_count, 1);
  cl_assert_equal_i(s_unsubscribe_count, 0);

  // Second slot does not re-subscribe.
  touch_service_subscribe(prv_raw_handler, &s_raw_marker);
  cl_assert_equal_i(s_subscribe_count, 1);
  cl_assert_equal_i(s_unsubscribe_count, 0);

  // Dropping one slot keeps the shared subscription alive.
  touch_service_unsubscribe();
  cl_assert_equal_i(s_subscribe_count, 1);
  cl_assert_equal_i(s_unsubscribe_count, 0);
  cl_assert(s_state.subscribed);

  // Dropping the last slot unsubscribes exactly once.
  touch_service_set_system_handler(NULL, NULL);
  cl_assert_equal_i(s_subscribe_count, 1);
  cl_assert_equal_i(s_unsubscribe_count, 1);
  cl_assert(!s_state.subscribed);

  // Re-occupying subscribes again (no leak of the previous cycle).
  touch_service_subscribe(prv_raw_handler, &s_raw_marker);
  cl_assert_equal_i(s_subscribe_count, 2);
  cl_assert_equal_i(s_unsubscribe_count, 1);
}

// touch_service_unsubscribe with only the system slot occupied is a no-op on
// the subscription and never clears the system slot.
void test_touch_service__unsubscribe_keeps_system_slot(void) {
  touch_service_set_system_handler(prv_system_handler, &s_system_marker);
  cl_assert_equal_i(s_subscribe_count, 1);

  touch_service_unsubscribe();
  cl_assert(s_state.system_handler == prv_system_handler);
  cl_assert(s_state.subscribed);
  cl_assert_equal_i(s_unsubscribe_count, 0);
}
