/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/system_task.h"

#include <pbl/logging/logging.h>

#include <pbl/drivers/task_watchdog.h>
#include "kernel/pebble_tasks.h"
#include "kernel/util/task_init.h"
#include "pbl/mcu/fpu.h"
#include "pbl/kernel/types.h"
#include "pbl/services/regular_timer.h"

#include "pbl/kernel/msgq.h"
#include "pbl/kernel/poll.h"
#include "pbl/kernel/thread.h"

PBL_LOG_MODULE_DEFINE(service_system_task, CONFIG_SERVICE_SYSTEM_TASK_LOG_LEVEL);

#define SYSTEM_TASK_PRIORITY (PBL_PRIO_IDLE + 1)

typedef struct {
  SystemTaskEventCallback cb;
  void *data;
} SystemTaskEvent;

#define SYSTEM_TASK_QUEUE_LENGTH 30
#define FROM_APP_SYSTEM_TASK_QUEUE_LENGTH 8

static PBL_MSGQ_DEFINE(s_system_task_queue, sizeof(SystemTaskEvent), SYSTEM_TASK_QUEUE_LENGTH);
static PBL_MSGQ_DEFINE(s_from_app_system_task_queue, sizeof(SystemTaskEvent),
                       FROM_APP_SYSTEM_TASK_QUEUE_LENGTH);
static PBL_POLL_GROUP_DEFINE(s_system_task_queue_set);
static bool s_initialized;

static SystemTaskEventCallback s_current_cb;

static bool s_system_task_idle = true;
static bool s_should_block_callbacks = false;

static bool prv_is_accepting_callbacks() {
  return s_initialized && !s_should_block_callbacks;
}

static void system_task_idle_timer_callback(void* data) {
  if (s_system_task_idle && pbl_poll_group_is_empty(&s_system_task_queue_set)) {
    system_task_watchdog_feed();
  }
}

static void system_task_main(void* paramater) {
  task_watchdog_mask_set(PebbleTask_KernelBackground);
  task_init();

  while (true) {
    s_system_task_idle = true;

    SystemTaskEvent event;

    struct pbl_msgq *activated_queue = pbl_poll_group_wait(&s_system_task_queue_set, PBL_FOREVER);

    // Get event from the activated queue
    bool result = (pbl_msgq_get(activated_queue, &event, PBL_NO_WAIT) == 0);

    // I believe its possible that we just reset the queue and accidently
    // pended an extra event to the queue set so handle that case gracefully
    if (result) {
      s_system_task_idle = false;
      s_current_cb = event.cb;
      event.cb(event.data);
      mcu_fpu_cleanup();
      s_current_cb = NULL;
    }

    // Refresh the watchdog immediately, just in case that cb() took awhile to run.
    system_task_watchdog_feed();
  }
}

void system_task_init(void) {
  pbl_poll_group_add(&s_system_task_queue_set, &s_system_task_queue);
  pbl_poll_group_add(&s_system_task_queue_set, &s_from_app_system_task_queue);
  s_initialized = true;

  extern uint32_t __kernel_bg_stack_start__[];
  extern uint32_t __kernel_bg_stack_size__[];
  extern uint32_t __stack_guard_size__[];

  struct pbl_thread_attr attr = {
    .name = "KernelBG",
    .entry = system_task_main,
    .prio = SYSTEM_TASK_PRIORITY,
    .privileged = true,
    .stack = (void *)((uintptr_t)__kernel_bg_stack_start__ + (uintptr_t)__stack_guard_size__),
    .stack_size = (uintptr_t)__kernel_bg_stack_size__ - (uintptr_t)__stack_guard_size__,
  };

  pebble_task_create(PebbleTask_KernelBackground, &attr);
}

void system_task_timer_init(void) {
  // Register a regular timer to kick the watchdog while we're waiting for something
  // to do. The other way to do this is to have the queue wait in system_task_main time out
  // occasionally, but that isn't necessarily second aligned and will require the watch
  // to wakeup from sleep just to kick the watchdog. This way it's kicked at the same time as
  // all the other regular tasks. Note that the system_task_idle_timer_callback only kicks
  // the watchdog if we're currently waiting for work to do on the system_task. If we're in the
  // middle of something we won't kick it.
  static RegularTimerInfo idle_watchdog_timer = {
    .cb = system_task_idle_timer_callback
  };
  regular_timer_add_seconds_callback(&idle_watchdog_timer);
}

void system_task_watchdog_feed(void) {
  task_watchdog_bit_set(PebbleTask_KernelBackground);
}

static void handle_system_task_send_failure(SystemTaskEventCallback cb, uintptr_t caller_lr) {
  PBL_LOG_ERR("System task queue full. Dropped cb: %p, current cb: %p", cb, s_current_cb);

  RebootReason reason = {
    .code = RebootReasonCode_EventQueueFull,
    .event_queue = {
      .push_lr = (uint32_t) caller_lr,
      .current_event = (uint32_t) s_current_cb,
      .dropped_event = (uint32_t) cb
    }
  };
  reboot_reason_set(&reason);

  reset_due_to_software_failure();
}

static bool prv_send_to_queue_from_isr(SystemTaskEventCallback cb, void *data,
                                       bool *should_context_switch) {
  SystemTaskEvent event = {
    .cb = cb,
    .data = data,
  };

  bool success = (pbl_msgq_put(&s_system_task_queue, &event, PBL_NO_WAIT) == 0);
  *should_context_switch = false;

  return success;
}

bool system_task_add_callback_from_isr(SystemTaskEventCallback cb, void *data, bool* should_context_switch) {
  // Capture caller LR at entry; reading from a deeper helper is unreliable.
  uintptr_t caller_lr = (uintptr_t)__builtin_return_address(0);
  if (!prv_is_accepting_callbacks()) {
    return false;
  }

  bool success = prv_send_to_queue_from_isr(cb, data, should_context_switch);
  if (!success) {
    handle_system_task_send_failure(cb, caller_lr);
  }

  return success;
}

bool system_task_add_callback_from_isr_droppable(SystemTaskEventCallback cb, void *data,
                                                 bool *should_context_switch) {
  if (!prv_is_accepting_callbacks()) {
    return false;
  }

  return prv_send_to_queue_from_isr(cb, data, should_context_switch);
}

bool system_task_add_callback(SystemTaskEventCallback cb, void *data) {
  uintptr_t caller_lr = (uintptr_t)__builtin_return_address(0);
  if (!prv_is_accepting_callbacks()) {
    return false;
  }

  SystemTaskEvent event = {
    .cb = cb,
    .data = data,
  };

  if (pebble_task_get_current() == PebbleTask_App) {
    // If we're the app and we've filled up our system task, the app just gets to wait.
    // FIXME: In the future when we want to bound the amount of time a syscall can take this will have to change.
    pbl_msgq_put(&s_from_app_system_task_queue, &event, PBL_FOREVER);
    return true;
  } else {
    // Back ourselves up and wait a reasonable amount of time before failing. If the queue is really backed up
    // we want to fall through to the handle_system_task_send_failure and not just get killed by the watchdog.
    bool success = (pbl_msgq_put(&s_system_task_queue, &event, PBL_MSEC(3000)) == 0);
    if (!success) {
      handle_system_task_send_failure(cb, caller_lr);
    }
  }

  return true;
}

void system_task_block_callbacks(bool block) {
  s_should_block_callbacks = block;
}

uint32_t system_task_get_available_space(void) {
  const bool is_app = pebble_task_get_current() == PebbleTask_App;
  return pbl_msgq_num_free(is_app ? &s_from_app_system_task_queue : &s_system_task_queue);
}

void* system_task_get_current_callback(void) {
  return s_current_cb;
}

void system_task_enable_raised_priority(bool is_raised) {
  const pbl_prio_t raised_priority_level = PBL_PRIO_IDLE + 3; // Same as KernelMain / BT tasks
  pbl_thread_prio_set(pebble_task_get_thread(PebbleTask_KernelBackground),
                      is_raised ? raised_priority_level : SYSTEM_TASK_PRIORITY);
}

bool system_task_is_ready_to_run(void) {
  // check if system task is ready to go (instead of e.g. waiting for a mutex)
  return pbl_thread_state(pebble_task_get_thread(PebbleTask_KernelBackground)) == PBL_THREAD_READY;
}
