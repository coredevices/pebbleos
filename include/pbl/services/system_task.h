/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

//! @file system_task.h
//!
//! This file implements a low priority background task that ISRs and other high priority tasks can
//! marshal units of work onto.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "kernel/pebble_tasks.h"

void system_task_init(void);
void system_task_timer_init(void);

//! If your callback running on the system task takes awhile to run, call this regularly to show that
//! you're still alive.
void system_task_watchdog_feed(void);

typedef void (*SystemTaskEventCallback)(void *data);

//! @param cb Callback function that will later be called from the system task
//! @param data Context pointer passed to the callback
//! @param should_context_switch A boolean that indicates our ISR should context switch at the end instead of
//!                              resuming the previous task. See portEND_SWITCHING_ISR()
bool system_task_add_callback_from_isr(SystemTaskEventCallback cb, void *data, bool* should_context_switch);

//! @param cb Callback function that will later be called from the system task
//! @param data Context pointer passed to the callback
bool system_task_add_callback(SystemTaskEventCallback cb, void *data);

//! Like system_task_add_callback(), but reports a full queue instead of waiting
//! on it. system_task_add_callback() blocks for up to 3 s and then reboots the
//! watch, which is not an option for a caller that holds a lock KernelBG needs,
//! runs on KernelBG itself, or runs on a task the rest of the system waits on.
//! @param cb Callback function that will later be called from the system task
//! @param data Context pointer passed to the callback
//! @return True if the callback was queued. False if the queue is full or
//! callbacks are blocked; the caller owns whatever it was going to hand over.
bool system_task_add_callback_nonblocking(SystemTaskEventCallback cb, void *data);

//! @param block True if callbacks should be rejected, False if they should be let through.
void system_task_block_callbacks(bool block);

//! @return The number callbacks that can be enqueued before the queue is full.
uint32_t system_task_get_available_space(void);

//! Debug! Return the callback we're currently executing.
void* system_task_get_current_callback(void);

//! @param is_raised When true, priority of the KernelBG task is raised to a higher priority. When
//! false, the priority is set to the normal priority.
//! @note WARNING: if you want to use this, implement ref counting internally. Currently only
//! comm/session.c uses this hence we can get away without ref counting.
void system_task_enable_raised_priority(bool is_raised);

//! @return True if the KernelBG task is ready to run (i.e. not blocked by mutex / queue)
bool system_task_is_ready_to_run(void);
