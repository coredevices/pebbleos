/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "metrics.h"

#include "pbl/services/activity/hr_util.h"

#include <stddef.h>
#include <stdint.h>

typedef struct WorkoutData {
  int32_t steps;
  int32_t duration_s;
  int32_t distance_m;
  int32_t avg_pace;
  int32_t bpm;
  HRZone hr_zone;
} WorkoutData;

void workout_data_update(void *workout_data);

void workout_data_fill_metric_value(WorkoutMetricType type, char *buffer,
                                    size_t buffer_size, void *i18n_owner, void *workout_data);

int32_t workout_data_get_metric_value(WorkoutMetricType type, void *workout_data);
