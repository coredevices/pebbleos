/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/ui/animation_private.h"
#include "applib/app_logging.h"

#include "kernel/events.h"
#include "kernel/kernel_applib_state.h"

#include "process_management/process_manager.h"
#include "process_state/app_state/app_state.h"

#include "pbl/services/new_timer/new_timer.h"

#include "system/passert.h"

#include "syscall/syscall_internal.h"


// The timer ID used for each task that we support
static TimerID s_kernel_main_timer_id = TIMER_INVALID_ID;
static TimerID s_app_timer_id = TIMER_INVALID_ID;

static bool s_kernel_main_event_pending;
static bool s_app_event_pending;

// ------------------------------------------------------------------------------------------
void animation_service_cleanup(PebbleTask task) {
  PBL_ASSERT_TASK(PebbleTask_KernelMain);

  if (task == PebbleTask_KernelMain) {
    if (s_kernel_main_timer_id != TIMER_INVALID_ID) {
      new_timer_delete(s_kernel_main_timer_id);
      s_kernel_main_timer_id = TIMER_INVALID_ID;
    }
    s_kernel_main_event_pending = false;
  } else if (task == PebbleTask_App) {
    if (s_app_timer_id != TIMER_INVALID_ID) {
      new_timer_delete(s_app_timer_id);
      s_app_timer_id = TIMER_INVALID_ID;
    }
    s_app_event_pending = false;
  }
}


// ------------------------------------------------------------------------------------------
static void prv_timer_callback(void * context) {
  PebbleTask task = (PebbleTask)context;

  PebbleEvent e = {
    .type = PEBBLE_CALLBACK_EVENT,
    .callback = {
      .callback = animation_private_timer_callback
    }
  };

  switch (task) {
    case PebbleTask_KernelMain:
      if (!s_kernel_main_event_pending) {
        s_kernel_main_event_pending = true;
        e.callback.data = (void *)kernel_applib_get_animation_state();
        event_put(&e);
      }
      break;
    case PebbleTask_App:
      if (!s_app_event_pending) {
        e.callback.data = (void *)app_state_get_animation_state();
        s_app_event_pending = process_manager_send_event_to_process(task, &e);
      }
      break;
    default:
      PBL_CROAK("Invalid task %s", pebble_task_get_name(pebble_task_get_current()));
    }
}


// ------------------------------------------------------------------------------------------
DEFINE_SYSCALL(void, animation_service_timer_event_received, void) {
  PebbleTask task = pebble_task_get_current();

  if (task == PebbleTask_KernelMain) {
    s_kernel_main_event_pending = false;
  } else if (task == PebbleTask_App) {
    s_app_event_pending = false;
  } else {
    if (PRIVILEGE_WAS_ELEVATED) {
      syscall_failed();
    }
    return;
  }
}


// ------------------------------------------------------------------------------------------
DEFINE_SYSCALL(void, animation_service_timer_schedule, uint32_t ms) {
  PebbleTask task = pebble_task_get_current();
  TimerID *timer_id;

  if (task == PebbleTask_KernelMain) {
    timer_id = &s_kernel_main_timer_id;
  } else if (task == PebbleTask_App) {
    timer_id = &s_app_timer_id;
  } else {
    if (PRIVILEGE_WAS_ELEVATED) {
      syscall_failed();
    }
    return;
  }

  // Need to create the timer?
  bool success = false;
  if (*timer_id == TIMER_INVALID_ID) {
    *timer_id = new_timer_create();
  }

  // Schedule/reschedule it
  if (*timer_id != TIMER_INVALID_ID) {
    success = new_timer_start(*timer_id, ms, prv_timer_callback, (void *)(uintptr_t)task,
                    0 /*flags */);
  }
  if (!success) {
    APP_LOG(APP_LOG_LEVEL_ERROR, "Error scheduling timer");
  }
}


// ---------------------------------------------------------------------------
// Used for unit tests only
TimerID animation_service_test_get_timer_id(void) {
  return s_kernel_main_timer_id;
}
