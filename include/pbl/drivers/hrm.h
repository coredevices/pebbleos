/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "board/board.h"

#include "pbl/services/hrm/hrm_manager.h"

#include <stdbool.h>

//! Initialize the HRM
void hrm_init(HRMDevice *dev);

//! Enable the HRM, sampling the PPG functions needed for the requested features
//! @param dev the HRM device to enable
//! @param features bitmask of HRMFeature values the sensor should collect
//! @return true if successfully enabled, false if initialization failed
bool hrm_enable(HRMDevice *dev, HRMFeature features);

//! Disable the HRM
void hrm_disable(HRMDevice *dev);

//! Checks whether or not the HRM is enabled
bool hrm_is_enabled(HRMDevice *dev);
