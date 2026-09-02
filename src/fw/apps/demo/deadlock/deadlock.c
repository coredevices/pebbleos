/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "profile_mutexes.h"

#include "applib/app.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/window.h"

#include <pbl/logging/logging.h>
#include "pbl/kernel/mutex.h"
#include "system/profiler.h"

#include "kernel/util/sleep.h"
#include "pbl/services/new_timer/new_timer.h"

static Window *window;
static PBL_MUTEX_DEFINE(s_mutex);
static PBL_MUTEX_DEFINE(s_mutex2);

static void callback(void *data) {
  PBL_LOG_DBG("Locking mutex 2 (new timer)");
  pbl_mutex_lock(&s_mutex2, PBL_FOREVER);
  PBL_LOG_DBG("Locking mutex 1 (new timer)");
  pbl_mutex_lock(&s_mutex, PBL_FOREVER);
}

static void deadlock(void) {
  TimerID timer = new_timer_create();
  new_timer_start(timer, 10, callback, NULL, 0);

  PBL_LOG_DBG("Locking mutex 1");
  pbl_mutex_lock(&s_mutex, PBL_FOREVER);
  psleep(20);
  PBL_LOG_DBG("Locking mutex 2");
  pbl_mutex_lock(&s_mutex2, PBL_FOREVER);
}

static void s_main(void) {
  window = window_create();
  app_window_stack_push(window, true /* Animated */);

  deadlock();

  app_event_loop();
}

const PebbleProcessMd* deadlock_get_app_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common.main_func = &s_main,
    .name = "Deadlock"
  };
  return (const PebbleProcessMd*) &s_app_info;
}
