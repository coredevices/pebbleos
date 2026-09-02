/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "task_timer.h"

#include "pbl/kernel/sem.h"
#include "pbl/util/list.h"

//! Internal state object. Each task that wants to execute timers should allocate their own
//! instance of this object.
typedef struct TaskTimerManager {
  struct pbl_mutex mutex;

  //! List of timers that are currently running
  ListNode *running_timers;
  //! List of timers that are allocated but unscheduled
  ListNode *idle_timers;

  //! The next ID to assign to a new timer.
  TaskTimerID next_id;

  //! Externally provided semaphore that is given whenever the next timer to expire has changed.
  struct pbl_sem *semaphore;

  //! The callback we're currently executing, useful for debugging.
  void *current_cb;
} TaskTimerManager;


//! Initialize a passed in manager object.
//! @param[in] manager The manager object to initialize
//! @param[in] semaphore a sempahore the TaskTimerManager should give if the next expiring timer
//!                      has changed. The task event loop should block on this same semphore to
//!                      handle timer updates in a timely fashion.
void task_timer_manager_init(TaskTimerManager *manager, struct pbl_sem *semaphore);

//! Execute any timers that are currently expired.
//! @return the number of ticks until the next timer expires. If there are no timers running,
//!         returns portMAX_DELAY.
TickType_t task_timer_manager_execute_expired_timers(TaskTimerManager *manager);

//! Debugging interface to help understand why the task_timer exuction is stuck and what
//! its stuck on.
//! @return A pointer to the current callback that's running, NULL if no callback
//!         is currently running.
void* task_timer_manager_get_current_cb(const TaskTimerManager *manager);
