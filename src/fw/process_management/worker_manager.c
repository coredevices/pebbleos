/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "process_manager.h"
#include "worker_manager.h"
#include "process_loader.h"

// Pebble stuff
#include "kernel/pbl_malloc.h"
#include "kernel/util/segment.h"
#include "kernel/util/task_init.h"
#include "pbl/mcu/privilege.h"
#include "popups/crashed_ui.h"
#include "process_management/app_install_manager.h"
#include "process_management/app_manager.h"
#include "process_management/process_heap.h"
#include "process_state/worker_state/worker_state.h"
#include "shell/prefs.h"

#include "syscall/syscall.h"
#include "syscall/syscall_internal.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

// FreeRTOS stuff

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MAX_TO_WORKER_EVENTS 8
static ProcessContext s_worker_task_context;
static PBL_MSGQ_DEFINE(s_to_worker_event_queue, sizeof(PebbleEvent), MAX_TO_WORKER_EVENTS);

extern char __WORKER_RAM__[];
extern char __WORKER_RAM_end__[];
extern char __stack_guard_size__[];

//! Used by the "pebble gdb" command to locate the loaded worker in memory.
void * volatile g_worker_load_address;

typedef struct NextWorker {
  const PebbleProcessMd *md;
  const void *args;
} NextWorker;

static NextWorker s_next_worker;

static bool s_workers_enabled = true;

static AppInstallId s_last_worker_crashed_install_id;
static time_t s_last_worker_crash_timestamp;
static bool s_worker_crash_relaunches_disabled;

// ---------------------------------------------------------------------------------------------
void worker_manager_init(void) {
}

// ---------------------------------------------------------------------------------------------
// This is the wrapper function for the worker. It's not allowed to return as it's the top frame on the stack
// created for the application.
static void prv_worker_task_main(void *entry_point) {
  // Init worker state variables
  worker_state_init();
  task_init();

  // about to start the worker in earnest. No longer safe to kill.
  s_worker_task_context.safe_to_kill = false;

  // Enter unprivileged mode!
  const bool is_unprivileged = s_worker_task_context.app_md->is_unprivileged;
  if (is_unprivileged) {
    mcu_state_set_thread_privilege(false);
  }

  const PebbleMain main_func = entry_point;
  main_func();

  // Clean up after the worker.  Remember to put only non-critical cleanup here,
  // as the worker may crash or otherwise misbehave. If something really needs to
  // be cleaned up, make it so the kernel can do it on the worker's behalf and put
  // the call at the bottom of prv_worker_cleanup.
  worker_state_deinit();

  sys_exit();
}

//! Heap locking function for our app heap. Our process heaps don't actually
//! have to be locked because they're the sole property of the process and no
//! other tasks should be touching it. All this function does is verify that
//! this condition is met before continuing without locking.
static void prv_heap_lock(void* unused) {
  PBL_ASSERT_TASK(PebbleTask_Worker);
}

static size_t prv_get_worker_segment_size(const PebbleProcessMd *app_md) {
  // 12 KiB - 640 bytes workerlib static = 11648 bytes
  return 11648;
}

static size_t prv_get_worker_stack_size(const PebbleProcessMd *app_md) {
  return 1400;
}

// ------------------------------------------------------------------------------------------------
bool worker_manager_launch_new_worker_with_args(const PebbleProcessMd *app_md, const void *args) {
  PBL_ASSERT_TASK(PebbleTask_KernelMain);

  // Don't launch workers in recovery mode to reduce the chance of crashes
#ifdef CONFIG_RECOVERY_FW
  return false;
#endif

  // If workers are disabled, don't launch
  if (!s_workers_enabled) {
    PBL_LOG_WRN("Workers disabled");
    return false;
  }

  // if we are trying to start another worker, then we want to enable relaunches on crashes.
  s_worker_crash_relaunches_disabled = false;

  // If there is a different worker currently running, tell it to quit first. When it sees s_next_worker
  // set, it will call us again once it finishes closing the current worker
  if (s_worker_task_context.app_md != NULL &&  s_worker_task_context.app_md != app_md) {
    s_next_worker = (NextWorker) {
      .md = app_md,
      .args = args,
    };
    worker_manager_close_current_worker(true /*graceful*/);
    return true;
  }

  // Clear the next worker settings
  s_next_worker = (NextWorker) {};

  // Error if a worker already launched
  if (pebble_task_get_thread(PebbleTask_Worker) != NULL) {
    PBL_LOG_WRN("Worker already launched");
    return false;
  }

  process_manager_init_context(&s_worker_task_context, app_md, args);
  s_worker_task_context.to_process_event_queue = &s_to_worker_event_queue;

  // Set up the worker's memory and load the binary into it.
  const size_t worker_segment_size = prv_get_worker_segment_size(app_md);
  // The stack guard is counted as part of the app segment size...
  const size_t stack_guard_size = (uintptr_t)__stack_guard_size__;
  // ...and is carved out of the stack.
  const size_t stack_size =
      prv_get_worker_stack_size(app_md) - stack_guard_size;

  MemorySegment worker_ram = { __WORKER_RAM__, __WORKER_RAM_end__ };
  memset((char *)worker_ram.start + stack_guard_size, 0,
         memory_segment_get_size(&worker_ram) - stack_guard_size);

  MemorySegment worker_segment;
  PBL_ASSERTN(memory_segment_split(&worker_ram, &worker_segment,
                                   worker_segment_size));
  PBL_ASSERTN(memory_segment_split(&worker_segment, NULL, stack_guard_size));
  // No (accessible) memory segments can be placed between the top of WORKER_RAM
  // and the end of stack. Stacks always grow towards lower memory addresses, so
  // we want a stack overflow to touch the stack guard region before it begins
  // to clobber actual data. And syscalls assume that the stack is always at the
  // top of WORKER_RAM; violating this assumption will result in syscalls
  // sometimes failing when the worker hasn't done anything wrong.
  void *stack = memory_segment_split(&worker_segment, NULL, stack_size);
  PBL_ASSERTN(stack);
  s_worker_task_context.load_start = worker_segment.start;
  g_worker_load_address = worker_segment.start;
  void *entry_point = process_loader_load(app_md, PebbleTask_Worker,
                                          &worker_segment);
  s_worker_task_context.load_end = worker_segment.start;
  if (!entry_point) {
    PBL_LOG_WRN("Tried to launch an invalid worker in bank %u!",
            process_metadata_get_code_bank_num(app_md));
    return false;
  }

  // The rest of worker_ram is available for worker state to use as it sees fit.
  if (!worker_state_configure(&worker_ram)) {
    PBL_LOG_ERR("Worker state configuration failed");
    return false;
  }
  // The remaining space in worker_segment is assigned to the worker's
  // heap. Worker state needs to be configured before initializing the
  // heap as the WorkerState struct holds the worker heap's Heap object.
  Heap *worker_heap = worker_state_get_heap();
  PBL_LOG_DBG("Worker heap init %p %p",
          worker_segment.start, worker_segment.end);
  heap_init(worker_heap, worker_segment.start, worker_segment.end,
            /* enable_heap_fuzzing */ false);
  heap_set_lock_impl(worker_heap, (HeapLockImpl) {
      .lock_function = prv_heap_lock,
  });
  process_heap_set_exception_handlers(worker_heap, app_md);

  // Init services required for this process before it starts to execute
  process_manager_process_setup(PebbleTask_Worker);

  char task_name[PBL_THREAD_NAME_LEN];
  snprintf(task_name, sizeof(task_name), "Worker <%s>", process_metadata_get_name(s_worker_task_context.app_md));

  struct pbl_thread_attr attr = {
    .name = task_name,
    .entry = prv_worker_task_main,
    .arg = entry_point,
    .prio = PBL_PRIO_IDLE + 1,
    .privileged = true,
    .stack = stack,
    .stack_size = stack_size,
  };

  PBL_LOG_DBG("Starting %s", task_name);

  s_worker_task_context.task_handle = pebble_task_create(PebbleTask_Worker, &attr);

  // If no default yet, set as the default so that it can be relaunched upon system reset
  if (worker_manager_get_default_install_id() == INSTALL_ID_INVALID) {
    worker_manager_set_default_install_id(s_worker_task_context.install_id);
  }

  return true;
}

// ------------------------------------------------------------------------------------------------
// Reset the data we're tracking for workers that crash
static void prv_reset_last_worker_crashed_data(void) {
  // No need to reset s_last_worker_crash_timestamp because we always check the install id before
  // we check the timestamp
  s_last_worker_crashed_install_id = INSTALL_ID_INVALID;
}

// ------------------------------------------------------------------------------------------------
// Launch the next worker, if there is one
void worker_manager_launch_next_worker(AppInstallId previous_worker_install_id) {
  // Is there another worker set to switch to?
  if (s_next_worker.md != NULL) {
    worker_manager_launch_new_worker_with_args(s_next_worker.md, s_next_worker.args);
  } else {
    // Do we have a default worker we should switch to that is different from the previous worker?
    AppInstallId default_id = worker_manager_get_default_install_id();
    if (default_id != INSTALL_ID_INVALID && default_id != previous_worker_install_id) {
      worker_manager_put_launch_worker_event(default_id);
    }
  }
}

// ------------------------------------------------------------------------------------------------
void worker_manager_handle_remove_current_worker(void) {
  s_worker_crash_relaunches_disabled = true;
  worker_manager_close_current_worker(true);
}
// ------------------------------------------------------------------------------------------------
void worker_manager_close_current_worker(bool gracefully) {

  // This method can be called as a result of receiving a PEBBLE_PROCESS_KILL_EVENT notification
  // from an app, telling us that it just finished it's deinit.

  // Shouldn't be called from app. Use process_manager_put_kill_process_event() instead.
  PBL_ASSERT_TASK(PebbleTask_KernelMain);

  // If no worker running, nothing to do
  if (!s_worker_task_context.app_md) {
    return;
  }

  // Make sure the process is safe to kill. If this method returns false, it will have set a timer
  // to post another KILL event in a few seconds, thus giving the process a chance to clean up.
  if (!process_manager_make_process_safe_to_kill(PebbleTask_Worker, gracefully)) {
    // Maybe next time...
    PBL_LOG_DBG("Worker not ready to exit");
    return;
  }

  // Save which worker we are exiting
  AppInstallId closing_worker_install_id = s_worker_task_context.install_id;

  // Perform generic process cleanup
  process_manager_process_cleanup(PebbleTask_Worker);

  // Notify the app install manager that we finally exited
  app_install_notify_worker_closed();

  // If the worker was closed gracefully, launch any next/default worker and return
  if (gracefully) {
    // Reset the data tracking the last worker that crashed since the closing worker did not crash
    prv_reset_last_worker_crashed_data();
    worker_manager_launch_next_worker(closing_worker_install_id);
    return;
  }

  // We arrive here if the worker crashed...

  // If the worker's app is in the foreground, close it
  if (closing_worker_install_id == app_manager_get_current_app_id()) {
    app_manager_force_quit_to_launcher();
  } else {
    const time_t current_time = rtc_get_time();
    const time_t WORKER_CRASH_RESET_TIMEOUT_SECONDS = 60;
    if ((closing_worker_install_id == s_last_worker_crashed_install_id) &&
        ((current_time - s_last_worker_crash_timestamp) <= WORKER_CRASH_RESET_TIMEOUT_SECONDS)) {
      // Reset the data tracking the last worker that crashed since we are going to show crash UI
      prv_reset_last_worker_crashed_data();
      // Show the crash UI, which will ask the user if they want to launch the worker's app
      crashed_ui_show_worker_crash(closing_worker_install_id);
    } else {
      // Record that this worker crashed and what time it crashed
      s_last_worker_crashed_install_id = closing_worker_install_id;
      s_last_worker_crash_timestamp = current_time;
      // Silently restart the worker if we are allowing relaunches of crashed workers
      if (!s_worker_crash_relaunches_disabled) {
        worker_manager_put_launch_worker_event(closing_worker_install_id);
      }
    }
  }
}

// ------------------------------------------------------------------------------------------------
const PebbleProcessMd* worker_manager_get_current_worker_md(void) {
  return s_worker_task_context.app_md;
}

// ------------------------------------------------------------------------------------------------
AppInstallId worker_manager_get_current_worker_id(void) {
  return s_worker_task_context.install_id;
}

// ------------------------------------------------------------------------------------------------
ProcessContext* worker_manager_get_task_context(void) {
  return &s_worker_task_context;
}

// ------------------------------------------------------------------------------------------------
void worker_manager_put_launch_worker_event(AppInstallId id) {
  PBL_ASSERTN(id != INSTALL_ID_INVALID);

  PebbleEvent e = {
    .type = PEBBLE_WORKER_LAUNCH_EVENT,
    .launch_app = {
      .id = id,
    },
  };

  event_put(&e);
}

// ------------------------------------------------------------------------------------------------
AppInstallId worker_manager_get_default_install_id(void) {
  return worker_preferences_get_default_worker();
}

// ------------------------------------------------------------------------------------------------
void worker_manager_set_default_install_id(AppInstallId id) {
  worker_preferences_set_default_worker(id);
}

// ------------------------------------------------------------------------------------------------
void worker_manager_enable(void) {
  if (!s_workers_enabled) {
    s_workers_enabled = true;
    AppInstallId id = worker_manager_get_default_install_id();
    if (id != INSTALL_ID_INVALID) {
      worker_manager_put_launch_worker_event(id);
    }
  }
}

// ------------------------------------------------------------------------------------------------
void worker_manager_disable(void) {
  if (s_workers_enabled) {
    s_workers_enabled = false;
    process_manager_put_kill_process_event(PebbleTask_Worker, true /*graceful*/);
  }
}

// ------------------------------------------------------------------------------------------------
void command_worker_kill(void) {
  process_manager_put_kill_process_event(PebbleTask_Worker, true /*graceful*/);
}

// ------------------------------------------------------------------------------------------------
DEFINE_SYSCALL(AppInstallId, sys_worker_manager_get_current_worker_id, void) {
  return worker_manager_get_current_worker_id();
}

