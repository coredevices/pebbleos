/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "data.h"

typedef struct HealthData {
  //!< Current step / activity info
  int32_t step_data[DAYS_PER_WEEK]; //!< Step history for today and the previous 6 days
  int32_t current_distance_meters;
  int32_t current_calories;

  //!< Typical step info
  ActivityMetricAverages step_averages; //!< The step averages for the current day
  int32_t current_step_average; //!< The current step average so far
  int32_t step_average_last_updated_time; //!< The time at which current_step_average was updated

  int32_t monthly_step_average;

  int32_t sleep_data[DAYS_PER_WEEK]; //!< Sleep history for the past week
  int32_t typical_sleep; //! Typical sleep for the current week day
  int32_t deep_sleep; //!< Amount of deep sleep last night

  int32_t sleep_start; //!< When the user went to sleep (seconds after midnight)
  int32_t sleep_end; //!< When the user woke up (seconds after midnight)
  int32_t typical_sleep_start; //!< When the user typically goes to sleep
  int32_t typical_sleep_end; //!< When the user typically wakes up

  int32_t monthly_sleep_average;

  uint32_t num_activity_sessions; //!< Number of activity sessions returned by the API
  ActivitySession activity_sessions[ACTIVITY_MAX_ACTIVITY_SESSIONS_COUNT]; //!< Activity sessions

  int32_t current_hr_bpm; //!< Current BPM
  int32_t resting_hr_bpm; //!< Resting BPM
  time_t hr_last_updated; //!< Time at which HR data was last updated

  int32_t hr_zone1_minutes;
  int32_t hr_zone2_minutes;
  int32_t hr_zone3_minutes;
} HealthData;
