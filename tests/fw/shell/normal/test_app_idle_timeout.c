/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "shell/normal/app_idle_timeout.h"

#include "kernel/event_loop.h"
#include "shell/normal/watchface.h"
#include "shell/shell.h"

// Stubs
/////////////////////////////////////////////////////////////////////////
#include "stubs_logging.h"
#include "stubs_passert.h"

#include "fake_new_timer.h"

static int s_watchface_launch_count;

void launcher_task_add_callback(CallbackEventCallback callback, void *data) {
  callback(data);
}

const CompositorTransition *shell_get_watchface_compositor_animation(bool watchface_is_destination) {
  return NULL;
}

void watchface_launch_default(const CompositorTransition *animation) {
  s_watchface_launch_count++;
}

// Module state under test
/////////////////////////////////////////////////////////////////////////
extern TimerID s_timer;
extern bool s_app_paused;
extern bool s_touch_held;

static bool prv_is_scheduled(void) {
  return stub_new_timer_is_scheduled(s_timer);
}

// Tests
/////////////////////////////////////////////////////////////////////////

void test_app_idle_timeout__initialize(void) {
  s_watchface_launch_count = 0;
  app_idle_timeout_stop();
  s_app_paused = false;
  s_touch_held = false;
}

void test_app_idle_timeout__cleanup(void) {
  app_idle_timeout_stop();
  stub_new_timer_cleanup();
}

void test_app_idle_timeout__start_schedules(void) {
  app_idle_timeout_start();
  cl_assert(prv_is_scheduled());
}

void test_app_idle_timeout__touch_hold_halts_until_liftoff(void) {
  app_idle_timeout_start();

  app_idle_timeout_touch_down();
  cl_assert(!prv_is_scheduled());

  app_idle_timeout_touch_up();
  cl_assert(prv_is_scheduled());
}

void test_app_idle_timeout__refresh_during_hold_stays_halted(void) {
  app_idle_timeout_start();
  app_idle_timeout_touch_down();

  app_idle_timeout_refresh();
  cl_assert(!prv_is_scheduled());

  app_idle_timeout_touch_up();
  cl_assert(prv_is_scheduled());
}

void test_app_idle_timeout__hold_composes_with_focus_pause(void) {
  app_idle_timeout_start();

  // Focus lost mid-hold: liftoff must not restart the timer while paused.
  app_idle_timeout_touch_down();
  app_idle_timeout_pause();
  app_idle_timeout_touch_up();
  cl_assert(!prv_is_scheduled());
  app_idle_timeout_resume();
  cl_assert(prv_is_scheduled());

  // Focus regained mid-hold: resume must not restart the timer while held.
  app_idle_timeout_pause();
  app_idle_timeout_touch_down();
  app_idle_timeout_resume();
  cl_assert(!prv_is_scheduled());
  app_idle_timeout_touch_up();
  cl_assert(prv_is_scheduled());
}

void test_app_idle_timeout__liftoff_without_touchdown_is_harmless(void) {
  app_idle_timeout_start();
  app_idle_timeout_touch_up();
  cl_assert(prv_is_scheduled());
}

void test_app_idle_timeout__start_during_hold_waits_for_liftoff(void) {
  app_idle_timeout_touch_down();
  app_idle_timeout_start();
  cl_assert(!prv_is_scheduled());

  app_idle_timeout_touch_up();
  cl_assert(prv_is_scheduled());
}

void test_app_idle_timeout__expiry_launches_watchface(void) {
  app_idle_timeout_start();
  cl_assert(stub_new_timer_fire(s_timer));
  cl_assert_equal_i(s_watchface_launch_count, 1);
}
