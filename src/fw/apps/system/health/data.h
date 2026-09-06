/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/activity/activity.h"

typedef struct {
  int32_t sum;
  int32_t avg;
  int32_t max;
} BasicHealthStats;

typedef struct {
  BasicHealthStats weekday;
  BasicHealthStats weekend;
  BasicHealthStats daily;
} WeeklyStats;

//! Health data model
typedef struct HealthData HealthData;

////////////////////////////////////////////////////////////////////////////////////////////////////
// API Functions
//

//! Create a health data structure
//! @return A pointer to the new HealthData structure
HealthData *health_data_create(void);

//! Destroy a health data structure
//! @param health_data A pointer to an existing health data structure
void health_data_destroy(HealthData *health_data);

//! Fetch the current activity data from the system
//! @param health_data A pointer to the health data to use
void health_data_update(HealthData *health_data);

//! Fetch only the data required to display the initial card.
//! This helps reduce lag when opening the app
//! @param health_data A pointer to the health data to use
void health_data_update_quick(HealthData *health_data);

//! Fetch the current data for step derived metrics (distance, active time, calories)
//! @param health_data A pointer to the health data to use
void health_data_update_step_derived_metrics(HealthData *health_data);

//! Update the number of steps the user has taken today
//! @param health_data A pointer to the health data to use
//! @param new_steps the new value of the steps for today
void health_data_update_steps(HealthData *health_data, uint32_t new_steps);

//! Update the number of seconds the user has slept today
//! @param health_data A pointer to the health data to use
//! @param new_sleep the new value of the seconds of sleep today
//! @param new_deep_sleep the new value of the seconds of deep sleep today
void health_data_update_sleep(HealthData *health_data, uint32_t new_sleep, uint32_t new_deep_sleep);

//! Update the current HR BPM
//! @param health_data A pointer to the health data to use
void health_data_update_current_bpm(HealthData *health_data);

//! Update the time in HR zones
//! @param health_data A pointer to the health data to use
void health_data_update_hr_zone_minutes(HealthData *health_data);


//! Get the current step count
//! @param health_data A pointer to the health data to use
//! @return the current step count
int32_t health_data_current_steps_get(HealthData *health_data);

//! Get the historical step data
//! @param health_data A pointer to the health data to use
//! @return A pointer to historical step data
int32_t *health_data_steps_get(HealthData *health_data);

//! Get the current distance traveled in meters
//! @param health_data A pointer to the health data to use
//! @return the current distance traveled in meters
int32_t health_data_current_distance_meters_get(HealthData *health_data);

//! Get the current calories
//! @param health_data A pointer to the health data to use
//! @return the current calories
int32_t health_data_current_calories_get(HealthData *health_data);

//! Get current number of steps that should be taken by this time today
//! @param health_data A pointer to the health data to use
//! @return The integer number of steps that should be taken by this time today
int32_t health_data_steps_get_current_average(HealthData *health_data);

//! Get the minute-of-day of the latest completed step-average bin. Reflects the
//! value cached by the most recent health_data_steps_get_current_average() call.
//! @param health_data A pointer to the health data to use
//! @return The minute-of-day (0..1439) of the latest completed step-average bin
int32_t health_data_steps_get_current_average_minute(HealthData *health_data);

//! Get the step average for the current day of the week
//! @param health_data A pointer to the health data to use
//! @return An integer value for the number of steps that are typically taken on this week day
int32_t health_data_steps_get_cur_wday_average(HealthData *health_data);

//! Get the step average over the past month
//! @param health_data A pointer to the health data to use
//! @return An integer value for the average number of steps that we taken over the past month
int32_t health_data_steps_get_monthly_average(HealthData *health_data);

//! Get the historical sleep data
//! @param health_data A pointer to the health data to use
//! @return A pointer to historical sleep data
int32_t *health_data_sleep_get(HealthData *health_data);

//! Get the current sleep length
//! @param health_data A pointer to the health data to use
//! @return the current sleep length
int32_t health_data_current_sleep_get(HealthData *health_data);

//! Gets the typical sleep duration for the current weekday
int32_t health_data_sleep_get_cur_wday_average(HealthData *health_data);

//! Get the current deep sleep data
//! @param health_data A pointer to the health data to use
//! @return the current deep sleep length
int32_t health_data_current_deep_sleep_get(HealthData *health_data);

//! Get the sleep average over the past month
//! @param health_data A pointer to the health data to use
//! @return The average daily sleep over the past month
int32_t health_data_sleep_get_monthly_average(HealthData *health_data);

// Get the sleep start time
int32_t health_data_sleep_get_start_time(HealthData *health_data);

// Get the sleep end time
int32_t health_data_sleep_get_end_time(HealthData *health_data);

// Get the typical sleep start time
int32_t health_data_sleep_get_typical_start_time(HealthData *health_data);

// Get the typical sleep end time
int32_t health_data_sleep_get_typical_end_time(HealthData *health_data);

// Get the number of sleep sessions
int32_t health_data_sleep_get_num_sessions(HealthData *health_data);

// Get today's sleep sessions
ActivitySession *health_data_sleep_get_sessions(HealthData *health_data);

// Get current BPM
uint32_t health_data_hr_get_current_bpm(HealthData *health_data);

// Get resting BPM
uint32_t health_data_hr_get_resting_bpm(HealthData *health_data);

// Get HR last updated timestamp
time_t health_data_hr_get_last_updated_timestamp(HealthData *health_data);

// Get number of minutes in Zone 1
int32_t health_data_hr_get_zone1_minutes(HealthData *health_data);

// Get number of minutes in Zone 2
int32_t health_data_hr_get_zone2_minutes(HealthData *health_data);

// Get number of minutes in Zone 3
int32_t health_data_hr_get_zone3_minutes(HealthData *health_data);
