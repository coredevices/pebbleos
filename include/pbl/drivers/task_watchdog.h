/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>

#include "kernel/pebble_tasks.h"

#define TASK_WATCHDOG_PRIORITY 0x1
#define TASK_WATCHDOG_FEED_PERIOD_MS 500

void task_watchdog_init(void);

//! Feed the hardware watchdog. Only needed if task watchdog timer is external to the task watchdog
//! and is not being fed by the task watchdog timer ISR. Call this function periodically
//! with TASK_WATCHDOG_FEED_PERIOD_MS to ensure the hardware watchdog is fed.
void task_watchdog_feed(void);

//! Pause the task watchdog for a certain number of seconds. This is useful if you know
//! you're going to be doing something that will take a long time and you don't want the
//! watchdog to trigger a reboot.
void task_watchdog_pause(unsigned int seconds);

//! Resume the task watchdog after a call to task_watchdog_pause.
void task_watchdog_resume(void);

//! Feed the watchdog for a particular task. If a task doesn't call this function frequently
//! enough and it's mask is set we will eventually trigger a reboot.
void task_watchdog_bit_set(PebbleTask task);

//! Feed all task watchdogs bits. Don't use this unless you have to, as ideally all tasks should be
//! managing their own bits. If you're using this you're probably hacking around something awful.
void task_watchdog_bit_set_all(void);

//! @return bool Wether this task is being tracked by the task watchdog.
bool task_watchdog_mask_get(PebbleTask task);

//! Starts tracking a particular task using the task watchdog. The task must regularly call
//! task_watchdog_bit_set if task_watchdog_mask_set is set.
void task_watchdog_mask_set(PebbleTask task);

//! Removes a task from the task watchdog. This task will no long need to call
//! task_watchdog_bit_set regularly.
void task_watchdog_mask_clear(PebbleTask task);

//! Should only be called if the task_watchdog timer has been halted for some reason
//! (For example, when we are in stop mode)
void task_watchdog_step_elapsed_time_ms(uint32_t elapsed_ms);
