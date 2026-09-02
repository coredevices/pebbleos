/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "profile_mutexes.h"

#include "applib/app.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/window.h"
#include "applib/ui/window_stack.h"

#include <pbl/logging/logging.h>
#include "pbl/kernel/mutex.h"
#include "system/profiler.h"

static Window *window;
static PBL_MUTEX_DEFINE(s_mutex);
static PBL_MUTEX_DEFINE(s_rmutex);

static void profile_mutexes(void) {
  PBL_LOG_DBG("INITIALIZING PROFILER FOR MUTEXES!");
  PROFILER_INIT;
  PROFILER_START;

  for (int i=0; i < 10000; i++) {
    pbl_mutex_lock(&s_mutex, PBL_FOREVER);
    pbl_mutex_unlock(&s_mutex);
  }

  for (int i=0; i < 10000; i++) {
    pbl_mutex_lock(&s_rmutex, PBL_FOREVER);
  }
  for (int i=0; i < 10000; i++) {
    pbl_mutex_unlock(&s_rmutex);
  }

  PROFILER_STOP;
  PROFILER_PRINT_STATS;
}

static void s_main(void) {
  window = window_create();
  app_window_stack_push(window, true /* Animated */);

  profile_mutexes();

  app_event_loop();
}

const PebbleProcessMd* profile_mutexes_get_app_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common.main_func = &s_main,
    .name = "Profile Mutexes"
  };
  return (const PebbleProcessMd*) &s_app_info;
}
