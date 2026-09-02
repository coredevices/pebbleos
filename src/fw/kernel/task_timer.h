/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/kernel/mutex.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"

//! task_timer.h
//!
//! This file implements timers in a way that allows any of our tasks to run timers on their own.
//! This code is heavily based on the new_timer codebase. Each task that wants to execute timers
//! should allocate their own TaskTimerManager. Timers can be created using that manager using
//! task_timer_create. Timers created with one manager aren't transferable to any other manager.

//! A handle to a given timer. IDs are used instead of pointers to avoid use-after-free issues.
//! Note that IDs are only unique for a given manager.
typedef uint32_t TaskTimerID;
static const TaskTimerID TASK_TIMER_INVALID_ID = 0;

typedef struct TaskTimerManager TaskTimerManager;

typedef void (*TaskTimerCallback)(void *data);

//! Flags for task_timer_start()
//! TIMER_START_FLAG_REPEATING          make this a repeating timer
//!
//! TIMER_START_FLAG_FAIL_IF_EXECUTING  If the timer callback is currently executing, do not
//! schedule the timer and return false from task_timer_start. This can be helpful in usage patterns
//! where the timer callback might be blocked on a semaphore owned by the task issuing the start.
//!
//! TIMER_START_FLAG_FAIL_IF_SCHEDULED If the timer is already scheduled, do not reschedule it and
//! return false from task_timer_start.
#define TIMER_START_FLAG_REPEATING          0x01
#define TIMER_START_FLAG_FAIL_IF_EXECUTING  0x02
#define TIMER_START_FLAG_FAIL_IF_SCHEDULED  0x04


//! Creates a new timer object. This timer will start out in the stopped state.
//! @return the non-zero timer id or TIMER_INVALID_ID if OOM
TaskTimerID task_timer_create(TaskTimerManager *manager);

//! Schedule an existing timer to execute in timeout_ms. If the timer was already started, it will
//! be rescheduled for the new time.
//! @param[in] manager The manager that owns the timer
//! @param[in] timer ID
//! @param[in] timeout_ms timeout in milliseconds
//! @param[in] cb pointer to the user's callback procedure
//! @param[in] cb_data reference data for the callback
//! @param[in] flags one or more TIMER_START_FLAG_.* flags
//! @return True if succesful, false if timer was not rescheduled. Note that it will never return
//!     false if none of the FAIL_IF_* flags are set.
bool task_timer_start(TaskTimerManager *manager, TaskTimerID timer, uint32_t timeout_ms,
                      TaskTimerCallback cb, void *cb_data, uint32_t flags);

//! Stop a timer. For repeating timers, even if this method returns false (callback is currently
//! executing) the timer will not run again. Safe to call on timers that aren't currently started.
//! @param[in] manager The manager that owns the timer
//! @param[in] timer ID
//! @return False if timer's callback is current executing, true if not.
bool task_timer_stop(TaskTimerManager *manager, TaskTimerID timer);

//! Get scheduled status of a timer
//! @param[in] manager The manager that owns the timer
//! @param[in] timer ID
//! @param[out] expire_ms_p if not NULL, the number of milliseconds until this timer will fire is
//!                         returned in *expire_ms_p. If the timer is not scheduled (return value
//!                         is false), this value should be ignored.
//! @return True if timer is scheduled, false if not
bool task_timer_scheduled(TaskTimerManager *manager, TaskTimerID timer, uint32_t *expire_ms_p);

//! Delete a timer
//! @param[in] manager The manager that owns the timer
//! @param[in] timer ID
void task_timer_delete(TaskTimerManager *manager, TaskTimerID timer);
