/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/new_timer/new_timer.h"

#include "kernel/pbl_malloc.h"
#include "kernel/task_timer_manager.h"
#include "kernel/util/task_init.h"
#include "kernel/pebble_tasks.h"
#include <pbl/logging/logging.h>

#include "pbl/kernel/msgq.h"
#include "pbl/kernel/thread.h"
#include "pbl/kernel/sem.h"

PBL_LOG_MODULE_DEFINE(service_new_timer, CONFIG_SERVICE_NEW_TIMER_LOG_LEVEL);

typedef struct {
  NewTimerWorkCallback cb;
  void *data;
} NewTimerWorkItem;

// The timer service loop blocks on this binary semaphore with a timeout waiting for the next timer
// to be ready to fire.
static PBL_SEM_DEFINE(s_wake_srv_loop, 1, 1);

//! Queue of pointers to that should be called on the new_timer thread. This allows very high
//! priority pieces of work to be done on the new_timer thread in between timers.
#define WORK_QUEUE_SIZE 5
#define TASK_STACK_SIZE_BYTES 1380

PBL_THREAD_STACK_DEFINE(s_new_timer_stack, TASK_STACK_SIZE_BYTES);
static PBL_MSGQ_DEFINE(s_work_queue, sizeof(NewTimerWorkItem), WORK_QUEUE_SIZE);

// Used by debugging facility
static void *s_current_work_cb = 0;

static TaskTimerManager s_task_timer_manager;

// =======================================================================================
// Client-side Implementation

// ---------------------------------------------------------------------------------------
// Create a new timer
TimerID new_timer_create(void) {
  return task_timer_create(&s_task_timer_manager);
}

// --------------------------------------------------------------------------------
// Schedule a timer to run. 
bool new_timer_start(TimerID timer_id, uint32_t timeout_ms, NewTimerCallback cb, void *cb_data,
                     uint32_t flags) {
  return task_timer_start(&s_task_timer_manager, timer_id, timeout_ms, cb, cb_data, flags);
}

// --------------------------------------------------------------------------------
// Return scheduled status
bool new_timer_scheduled(TimerID timer_id, uint32_t *expire_ms_p) {
  return task_timer_scheduled(&s_task_timer_manager, timer_id, expire_ms_p);
}

// --------------------------------------------------------------------------------
// Stop a timer. If the timer callback is currently executing, return false, else return true.
bool new_timer_stop(TimerID timer_id) {
  return task_timer_stop(&s_task_timer_manager, timer_id);
}

// --------------------------------------------------------------------------------
// Delete a timer
void new_timer_delete(TimerID timer_id) {
  task_timer_delete(&s_task_timer_manager, timer_id);
}

// ========================================================================================
// Service Implementation
static void new_timer_service_loop(void *data) {
  task_init();

  while (1) {
    TickType_t ticks_to_wait = task_timer_manager_execute_expired_timers(&s_task_timer_manager);

    pbl_sem_take(&s_wake_srv_loop, PBL_TICKS(ticks_to_wait));

    // See if we have any work to do
    NewTimerWorkItem work;
    if (pbl_msgq_get(&s_work_queue, &work, PBL_NO_WAIT) == 0) {
      s_current_work_cb = work.cb;
      work.cb(work.data);
      s_current_work_cb = NULL;
    }
  }
}

// -----------------------------------------------------------------------------------------------
// Used by the watchdog timer logic
void* new_timer_debug_get_current_callback(void) {
  void *timer_cb = task_timer_manager_get_current_cb(&s_task_timer_manager);
  if (timer_cb) {
    return timer_cb;
  }
  return s_current_work_cb;
}

//=========================================================================================
// Initialize the timer service
void new_timer_service_init(void) {
  PBL_LOG_DBG("NT: Initializing");

  task_timer_manager_init(&s_task_timer_manager, &s_wake_srv_loop);

  struct pbl_thread_attr attr = {
    .name = "NewTimer",
    .entry = new_timer_service_loop,
    .prio = PBL_PRIO_MAX,
    .privileged = true,
    .stack = s_new_timer_stack,
    .stack_size = sizeof(s_new_timer_stack),
  };

  pebble_task_create(PebbleTask_NewTimers, &attr);
}

// -----------------------------------------------------------------------------------------------------
// Used by the console command to list timers
bool new_timer_add_work_callback_from_isr(NewTimerWorkCallback cb, void *data) {
  NewTimerWorkItem work = { cb, data };
  pbl_msgq_put(&s_work_queue, &work, PBL_NO_WAIT);

  // Wake up the thread to process the work item we just added.
  pbl_sem_give(&s_wake_srv_loop);

  return false;
}

bool new_timer_add_work_callback(NewTimerWorkCallback cb, void *data) {
  TickType_t TICKS_TO_WAIT = 50;

  NewTimerWorkItem work = { cb, data };
  if (pbl_msgq_put(&s_work_queue, &work, PBL_TICKS(TICKS_TO_WAIT)) == 0) {
    // Wake up the thread to process the work item we just added.
    pbl_sem_give(&s_wake_srv_loop);
    return true;
  }

  return false;
}
