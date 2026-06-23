/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "board/board.h"

#include "pbl/services/hrm/hrm_activity_scene.h"
#include "pbl/services/hrm/hrm_manager.h"

#include <stdbool.h>

//! Initialize the HRM
void hrm_init(HRMDevice *dev);

//! Enable the HRM, sampling the PPG functions needed for the requested features
//! @param dev the HRM device to enable
//! @param features bitmask of HRMFeature values the sensor should collect
//! @param low_latency true for live-display sessions that need prompt updates (workout / foreground
//!   app); false for background daily logging, where the FIFO can be drained less often to save
//!   MCU/I2C wakeups since only the final reading matters
//! @return true if successfully enabled, false if initialization failed
bool hrm_enable(HRMDevice *dev, HRMFeature features, bool low_latency);

//! Disable the HRM
void hrm_disable(HRMDevice *dev);

//! Checks whether or not the HRM is enabled
bool hrm_is_enabled(HRMDevice *dev);

//! Tell the HRM which activity is in progress so its algorithm can use a motion-appropriate mode.
//! Safe to call at any time (it only selects the algorithm's internal scenario); idempotent.
void hrm_set_activity_scene(HRMDevice *dev, HRMActivityScene scene);
