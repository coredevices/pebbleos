/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/util/list.h"

typedef void (*RegularTimerCallback)(void* data);

typedef struct RegularTimerInfo {
  ListNode list_node;
  RegularTimerCallback cb;
  void* cb_data;

  // the following fields are for internal use by the regular timer service and should not be touched
  uint16_t private_reset_count;
  uint16_t private_count;
  bool is_executing;
  bool pending_delete;
} RegularTimerInfo;

void regular_timer_init(void);

//! Add a callback that will be called every second.
void regular_timer_add_seconds_callback(RegularTimerInfo* cb);
//! Add a callback that will be called every n seconds. This can also be called to change the schedule of an existing
//! seconds timer, from inside or outside the callback procedure.
void regular_timer_add_multisecond_callback(RegularTimerInfo* cb, uint16_t seconds);

//! Add a callback that will be called every minute
void regular_timer_add_minutes_callback(RegularTimerInfo* cb);
//! Add a callback that will be called every n minutes. This can also be called to change the schedule of an existing
//! minute timer, from inside or outside the callback procedure.
void regular_timer_add_multiminute_callback(RegularTimerInfo* cb, uint16_t minutes);

//! Remove a callback already registered for either seconds or minutes.
//! WARNING: If you call this from your callback procedure, you are NOT allowed to free up the memory used for
//!  the RegularTimerInfo structure until after your callback exits!
//! @return true iff the timer was successfully stopped (false may indicate no timer was
//!  scheduled at all or the cb is currently executing)
bool regular_timer_remove_callback(RegularTimerInfo* cb);

//! Check if a regular timer is currently scheduled
//! @param cb pointer to the RegularTimerInfo struct for the timer
//! @returns true if scheduled or pending deletion, false otherwise
bool regular_timer_is_scheduled(RegularTimerInfo *cb);

//! Check if a regular timer is pending deletion. This means the timer
//! has been unscheduled but is in the process of executing
//! TODO: It would probably make sense to just fold this into the logic
//!       for _is_scheduled() once we verify no consumers are relying on
//!       this odd behavior
//! @param cb pointer to the RegularTimerInfo struct for the timer
bool regular_timer_pending_deletion(RegularTimerInfo *cb);


// -----------------------------------------------------------------------------
// For testing:

void regular_timer_deinit(void);

//! Fires the second callbacks, for which (seconds_interval % secs) is 0.
void regular_timer_fire_seconds(uint8_t secs);

//! Fires the minutes callbacks, for which (minutes_interval % mins) is 0.
void regular_timer_fire_minutes(uint8_t mins);

//! The number of registered (multi) second callbacks.
uint32_t regular_timer_seconds_count(void);

//! The number of registered (multi) minute callbacks.
uint32_t regular_timer_minutes_count(void);
