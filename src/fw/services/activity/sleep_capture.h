/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/events.h"
#include "pbl/services/accel_manager_types.h"

#include <stdbool.h>
#include <stdint.h>

//! Called once per minute by the Activity service.
void sleep_capture_minute_handler(uint32_t utc_sec, bool heart_rate_enabled, bool sleep_active,
                                  bool enhanced_logging_enabled);

//! True while the HRM subscription must include PPI collection.
bool sleep_capture_is_active(void);

//! Store a BPM or accepted PPI event.
void sleep_capture_handle_hrm_event(const PebbleHRMEvent *event);

//! Summarize accelerometer data into 30-second epochs.
void sleep_capture_handle_accel(const AccelRawData *data, uint32_t num_samples);

//! Flush and close the active DataLogging stream.
void sleep_capture_deinit(void);
