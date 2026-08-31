/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>
#include "kernel/pebble_tasks.h"

//! @file animation_service.h
//! Manage the system resources used by the applib/animation module.

//! Register the timer to fire in N ms. When it fires, the animation_private_timer_callback()
//! will be called and passed the AnimationState for that task.
void animation_service_timer_schedule(uint32_t ms);

//! Acknowledge that we received an event sent by the animation timer
void animation_service_timer_event_received(void);

//! Destroy the animation resoures used by the given task. Called by the process_manager when a
// process exits
void animation_service_cleanup(PebbleTask task);
