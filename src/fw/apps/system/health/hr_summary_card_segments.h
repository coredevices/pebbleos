/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "board/display.h"
#include "progress.h"

//! 4 main segments + 4 real corners
//! Each of the 4 non-corner segments get 25% of the total
#define AMOUNT_PER_SEGMENT (HEALTH_PROGRESS_BAR_MAX_VALUE * 25 / 100)

// The shape is the same, but the offsets are different
// Dynamically center based on display size vs legacy 144x168 base
// Round displays need additional adjustment for the bezel
#define X_SHIFT (((DISP_COLS - LEGACY_2X_DISP_COLS) / 2) + PBL_IF_ROUND_ELSE(18, 0))
#define Y_SHIFT (((DISP_ROWS - LEGACY_2X_DISP_ROWS) / 2) + PBL_IF_ROUND_ELSE(6, 0))

static HealthProgressSegment s_hr_summary_progress_segments[] = {
  {
    // Bottom corner
    .type = HealthProgressSegmentType_Corner,
    .points = {{72 + X_SHIFT, 87 + Y_SHIFT}, {65 + X_SHIFT, 94 + Y_SHIFT},
               {72 + X_SHIFT, 101 + Y_SHIFT}, {79 + X_SHIFT, 94 + Y_SHIFT}},
  },
  {
    // Left side bottom
    .amount_of_total = AMOUNT_PER_SEGMENT,
    .type = HealthProgressSegmentType_Vertical,
    .points = {{66 + X_SHIFT, 95 + Y_SHIFT}, {73 + X_SHIFT, 88 + Y_SHIFT},
               {42 + X_SHIFT, 57 + Y_SHIFT}, {35 + X_SHIFT, 64 + Y_SHIFT}},
  },
  {
    // Left corner
    .type = HealthProgressSegmentType_Corner,
    .points = {{43 + X_SHIFT, 58 + Y_SHIFT}, {36 + X_SHIFT, 51 + Y_SHIFT},
               {29 + X_SHIFT, 58 + Y_SHIFT}, {36 + X_SHIFT, 65 + Y_SHIFT}},
  },
  {
    // Left side top
    .amount_of_total = AMOUNT_PER_SEGMENT,
    .type = HealthProgressSegmentType_Vertical,
    .points = {{35 + X_SHIFT, 52 + Y_SHIFT}, {42 + X_SHIFT, 59 + Y_SHIFT},
               {73 + X_SHIFT, 28 + Y_SHIFT}, {66 + X_SHIFT, 21 + Y_SHIFT}},
  },
  {
    // Top corner
    .type = HealthProgressSegmentType_Corner,
    .points = {{72 + X_SHIFT, 29 + Y_SHIFT}, {79 + X_SHIFT, 22 + Y_SHIFT},
               {72 + X_SHIFT, 15 + Y_SHIFT}, {65 + X_SHIFT, 22 + Y_SHIFT}},
  },
  {
    // Right side top
    .amount_of_total = AMOUNT_PER_SEGMENT,
    .type = HealthProgressSegmentType_Vertical,
    .points = {{78 + X_SHIFT, 21 + Y_SHIFT}, {71 + X_SHIFT, 28 + Y_SHIFT},
               {102 + X_SHIFT, 59 + Y_SHIFT}, {109 + X_SHIFT, 52 + Y_SHIFT}},
  },
  {
    // Right corner
    .type = HealthProgressSegmentType_Corner,
    .points = {{101 + X_SHIFT, 58 + Y_SHIFT}, {108 + X_SHIFT, 65 + Y_SHIFT},
               {115 + X_SHIFT, 58 + Y_SHIFT}, {108 + X_SHIFT, 51 + Y_SHIFT}},
  },
  {
    // Right side bottom
    .amount_of_total = AMOUNT_PER_SEGMENT,
    .type = HealthProgressSegmentType_Vertical,
    .points = {{102 + X_SHIFT, 57 + Y_SHIFT}, {109 + X_SHIFT, 64 + Y_SHIFT},
               {78 + X_SHIFT, 95 + Y_SHIFT}, {71 + X_SHIFT, 88 + Y_SHIFT}},
  },
};
