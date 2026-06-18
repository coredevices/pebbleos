/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/activity/activity_private.h"
#include "pbl/util/attributes.h"

bool WEAK activity_tracking_on(void) {
  return true;
}

bool WEAK activity_get_metric(ActivityMetric metric, uint32_t history_len, int32_t *history) {
  return false;
}

bool WEAK activity_get_sessions(uint32_t *session_entries, ActivitySession *sessions) {
  return false;
}

bool WEAK activity_get_step_averages(DayInWeek day_of_week, ActivityMetricAverages *averages) {
  return false;
}

void WEAK activity_metrics_prv_set_metric(ActivityMetric metric, DayInWeek day, int32_t value) {}

bool WEAK activity_prefs_tracking_is_enabled(void) {
  return false;
}

uint8_t WEAK activity_prefs_get_age_years(void) {
  return 30; // This is our current default
}

bool WEAK activity_prefs_heart_rate_is_enabled(void) {
  return true;
}

bool WEAK activity_prefs_blood_oxygen_is_enabled(void) {
  return false;
}

bool WEAK activity_prefs_blood_oxygen_activity_tracking_is_enabled(void) {
  return false;
}

bool activity_get_metric_typical(ActivityMetric metric, DayInWeek day, int32_t *value_out) {
  return false;
}

bool activity_get_metric_monthly_avg(ActivityMetric metric, int32_t *value_out) {
  return false;
}

uint32_t activity_private_compute_distance_mm(uint32_t steps, uint32_t ms) {
  return 0;
}

uint32_t activity_private_compute_active_calories(uint32_t distance_mm, uint32_t ms) {
  return 0;
}

uint32_t activity_private_compute_resting_calories(uint32_t elapsed_minutes) {
  return 0;
}

uint8_t WEAK activity_prefs_heart_get_resting_hr(void) {
  return 70;
}

uint8_t WEAK activity_prefs_heart_get_elevated_hr(void) {
  return 100;
}

uint8_t WEAK activity_prefs_heart_get_max_hr(void) {
  return 190;
}

uint8_t WEAK activity_prefs_heart_get_zone1_threshold(void) {
  return 130;
}

uint8_t WEAK activity_prefs_heart_get_zone2_threshold(void) {
  return 154;
}

uint8_t WEAK activity_prefs_heart_get_zone3_threshold(void) {
  return 172;
}

bool WEAK activity_is_initialized(void) {
  return true;
}

void WEAK activity_set_enabled(bool enabled) {}

bool WEAK activity_start_tracking(bool test_mode) {
  return true;
}

bool WEAK activity_stop_tracking(void) {
  return true;
}
