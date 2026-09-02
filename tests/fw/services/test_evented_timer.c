/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "kernel/events.h"
#include "pbl/services/evented_timer.h"
#include "kernel/pebble_tasks.h"

#include "stubs_events.h"
#include "stubs_logging.h"
#include "stubs_passert.h"
#include "stubs_mutex.h"
#include "stubs_reboot_reason.h"
#include "stubs_syscall_internal.h"
#include "fake_pbl_malloc.h"
#include "fake_new_timer.h"


// NOTE: This must match the definition of EventedTimer in kernel/services/evented_timer.c
typedef struct EventedTimer {
  ListNode list_node;

  //! The TimerID type used for sys_timers is a non-repeating integer that we also use as our key for finding
  //!  EventedTimers by id. 
  TimerID sys_timer_id;

  EventedTimerCallback callback;
  void* callback_data;

  PebbleTask target_task;

  bool expired; // This gets set when the timer's callback runs
  bool repeating;
} EventedTimer;



// Fakes
///////////////////////////////////////////////////////////
PebbleTask pebble_task_get_current(void) {
  return PebbleTask_App;
}

const char* pebble_task_get_name(PebbleTask task) {
  return "App <Stub>";
}


static PebbleEvent s_last_event;

bool process_manager_send_event_to_process(PebbleTask task, PebbleEvent* e) {
  s_last_event = *e;
  return true;
}


// Tests
///////////////////////////////////////////////////////////
int s_times_callback_executed = 0;

static void stub_evented_timer_callback(void* data) {
  s_times_callback_executed++;
  s_last_event = (PebbleEvent) { 0 };
}


void test_evented_timer__initialize(void) {
  s_times_callback_executed = 0;

  s_last_event = (PebbleEvent) { 0 };
}

void test_evented_timer__cleanup(void) {
  evented_timer_reset();
}

void test_evented_timer__simple(void) {
  EventedTimerID e_timer = evented_timer_register(100, false, stub_evented_timer_callback, 0);

  // NOTE: We are leveraging the fact that we know the system timer ID is the same as the
  // EventedTimer ID
  TimerID sys_timer_id = e_timer;
  cl_assert(stub_new_timer_is_scheduled(sys_timer_id));
  cl_assert_equal_i(s_last_event.type, PEBBLE_NULL_EVENT);
  cl_assert(!s_times_callback_executed);

  // Fire the timer on "the timer task"
  stub_new_timer_fire(sys_timer_id);

  // We know have an event to run on "the app task"
  cl_assert_equal_i(s_last_event.type, PEBBLE_CALLBACK_EVENT);
  cl_assert(!s_times_callback_executed);

  // Run the code on "the app task"
  s_last_event.callback.callback(s_last_event.callback.data);

  // And we're done!
  cl_assert(s_times_callback_executed == 1);


  // Fire again, this time it should fail (not a repeating timer)
  cl_assert(stub_new_timer_fire(sys_timer_id) == false);
  cl_assert_equal_i(s_last_event.type, PEBBLE_NULL_EVENT);
}

void test_evented_timer__repeating(void) {
  EventedTimerID e_timer = evented_timer_register(100, true, stub_evented_timer_callback, 0);

  // NOTE: We are leveraging the fact that we know the system timer ID is the same as the
  // EventedTimer ID
  TimerID sys_timer_id = e_timer;
  cl_assert(stub_new_timer_is_scheduled(sys_timer_id));
  cl_assert_equal_i(s_last_event.type, PEBBLE_NULL_EVENT);
  cl_assert(!s_times_callback_executed);

  for (int i = 0; i < 10; ++i) {
    // Fire the timer on "the timer task"
    stub_new_timer_fire(sys_timer_id);

    // We know have an event to run on "the app task"
    cl_assert_equal_i(s_last_event.type, PEBBLE_CALLBACK_EVENT);
    cl_assert(s_times_callback_executed == i);

    // Run the code on "the app task"
    s_last_event.callback.callback(s_last_event.callback.data);

    // And we're done!
    cl_assert(s_times_callback_executed == i + 1);
  }
}



void test_evented_timer__cancel_during_freertos_timer_cb(void) {
  EventedTimerID timer = evented_timer_register(100, false, stub_evented_timer_callback, 0);

  // We've started the timer, but it hasn't fired yet.
  TimerID sys_timer_id = timer;
  cl_assert(stub_new_timer_is_scheduled(sys_timer_id));
  cl_assert_equal_i(s_last_event.type, PEBBLE_NULL_EVENT);
  cl_assert(!s_times_callback_executed);

  // Now cancel the timer, this should delete the system timer
  stub_new_timer_set_executing(sys_timer_id, true);   // This allows the timer to be cancelled, but not deleted
  evented_timer_cancel(timer);

  cl_assert(!stub_new_timer_is_scheduled(sys_timer_id));

  // However, we want to test the case where we send the delete command but the timer goes off before the
  // command is applied. Run the timer anyway.
  stub_new_timer_fire(sys_timer_id);

  // However, the timer should have been canceled in the evented_timer system, and we shouldn't see an event
  // be generated.
  cl_assert_equal_i(s_last_event.type, PEBBLE_NULL_EVENT);
  cl_assert(!s_times_callback_executed);
}

void test_evented_timer__cancel_during_app_event(void) {
  EventedTimerID timer = evented_timer_register(100, false, stub_evented_timer_callback, 0);

  // We've started the timer, but it hasn't fired yet.
  TimerID sys_timer_id = timer;
  cl_assert(stub_new_timer_is_scheduled(sys_timer_id));
  cl_assert_equal_i(s_last_event.type, PEBBLE_NULL_EVENT);
  cl_assert(!s_times_callback_executed);

  // Fire the timer on "the timer task"
  stub_new_timer_fire(sys_timer_id);

  // We know have an event to run on "the app task"
  cl_assert_equal_i(s_last_event.type, PEBBLE_CALLBACK_EVENT);
  cl_assert(!s_times_callback_executed);

  // Now cancel the timer after the event has been run on the timer task but before it's handled on the app task
  evented_timer_cancel(timer);

  // Run the code on "the app task"
  s_last_event.callback.callback(s_last_event.callback.data);

  // And we're done! Even though we let the timer fire and generate the event, cancelling before the event is
  // handled should stop the registered timer callback from being called.
  cl_assert(!s_times_callback_executed);
}

