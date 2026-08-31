/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "testinfra.h"

#include "console/pulse_internal.h"
#include "pbl/services/new_timer/new_timer.h"
#include <pbl/logging/logging.h>

#if !UNITTEST
static void prv_emit_ready_log(void *unused) {
  pbl_log(LOG_LEVEL_DEBUG, __FILE_NAME__, __LINE__, "Ready for communication.");
}
#endif

void notify_system_ready_for_communication(void) {
#if !UNITTEST
#ifdef CONFIG_PULSE_EVERYWHERE
  static bool s_pulse_started = false;
  if (!s_pulse_started) {
    pulse_start();
    s_pulse_started = true;
  }
#endif
  // Defer the "Ready for communication" log briefly.  pebble-tool's emulator
  // wait loop polls for this string and starts the install immediately when
  // it appears — but at this point the first app's task has only just been
  // *created*; its main_func hasn't run yet and it hasn't pushed its first
  // window.  For third-party watchfaces (whose window names don't match
  // pebble-tool's other wait tokens "<Launcher>" / "<SDK Home>") this means
  // install arrives before the watchface is event-loop-responsive, which
  // wedges the install until the user manually toggles to the launcher.
  // A short timer lets the first app run its main_func and push at least one
  // window before pebble-tool starts pumping protocol messages at us.
  static TimerID s_ready_log_timer = TIMER_INVALID_ID;
  if (s_ready_log_timer == TIMER_INVALID_ID) {
    s_ready_log_timer = new_timer_create();
  }
  new_timer_start(s_ready_log_timer, 500, prv_emit_ready_log, NULL, 0);
#endif
}
