/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

//! Coarse description of what the user is doing, so the HR sensor's PPG algorithm can switch to a
//! motion-tuned model. The mapping to the underlying algorithm's per-activity modes lives in the
//! driver; callers just describe the activity. Kept board/driver-independent so the (widely
//! included) HRM manager private header can use it without dragging in the HRMDevice type.
typedef enum {
  HRMActivityScene_Default = 0,  //!< Rest / background sampling; algorithm's general-purpose mode.
  HRMActivityScene_Walk,         //!< Walking.
  HRMActivityScene_Run,          //!< Running (incl. high heart rate).
  HRMActivityScene_HighIntensity,  //!< Open / mixed high-intensity exercise.
} HRMActivityScene;
