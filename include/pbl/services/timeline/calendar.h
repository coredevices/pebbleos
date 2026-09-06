/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "event.h"

//! This module puts events that report the current state of calendar events.
//! The states are:
//! - "no calendar events ongoing"
//! - "one or more calendar events ongoing"
//! Not every calendar event start / stop produces an event, but every transition is guaranteed
//! to put an event.


const TimelineEventImpl *calendar_get_event_service(void);

//! Used to determine if there is currently an event going on, used for Smart DND
bool calendar_event_is_ongoing(void);

#if UNITTEST
#include "pbl/services/new_timer/new_timer.h"
TimerID get_calendar_timer_id(void);
void set_calendar_timer_id(TimerID id);
#endif
