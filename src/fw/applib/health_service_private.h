/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "health_service.h"
#include "event_service_client.h"
#include "pbl/services/activity/activity.h"

typedef struct {
  HealthValue totals[ACTIVITY_HISTORY_DAYS];
} HealthServiceDailyHistory;

typedef struct {
  int32_t sum;
  int32_t avg;
  int32_t min;
  int32_t max;
  int32_t count;
} HealthServiceStats;

typedef struct {
  HealthServiceStats weekday;                // weekday stats
  HealthServiceStats weekend;                // weekend stats
  HealthServiceStats weekly;                 // weekly stats
  HealthServiceStats daily;                  // daily stats
} HealthServiceMetricStats;

// The number of session we choose to store is arbitrary and taken from other examples
// today. Typically, there should be less than 10 or so.
#define HEALTH_SERVICE_MAX_ACTIVITY_SESSIONS 16

// Information required to support health metric alerts
typedef struct {
  HealthValue threshold;                      // the threshold
  HealthValue prior_reading;                  // the prior reading
} HealthServiceMetricAlertInfo;

typedef struct {
  uint32_t cur_day_id;                        // Current day ID, used for cache validation

  // These are intraday step averages
  DayInWeek step_averages_day;                // which day in the week the step averages are for
  ActivityMetricAverages step_averages;       // intraday step averages

  // We cache the daily step totals since that metric is very likely to be requested by a
  // client. The other metrics we fetch only on an as-needed basis
  HealthServiceDailyHistory steps_daily;

  // Storage for fetched activity sessions
  ActivitySession sessions[HEALTH_SERVICE_MAX_ACTIVITY_SESSIONS];

  // Storage for fetching minute history
  HealthMinuteData minute_data[MINUTES_PER_HOUR];

  // Metric alert thresholds. 0 if not set.
  HealthServiceMetricAlertInfo alert_threshold_heart_rate;

  union {
    struct {
      uint16_t step_averages_valid:1;
      uint16_t step_daily_valid:1;
      uint16_t reserved:14;
    };
    uint16_t valid_flags;
  };
} HealthServiceCache;

typedef struct HealthServiceState {
  HealthEventHandler event_handler;
  void *context;
  HealthServiceCache *cache;
  EventServiceInfo health_event_service_info;
  uint16_t last_hrv_ppi_ms;
} HealthServiceState;

// initializes all static data, does not allocate a cache
void health_service_state_init(HealthServiceState *state);

// deallocates the cache (if it was allocated)
void health_service_state_deinit(HealthServiceState *state);

// helper struct for representing utc-based ranges on a per-day granularity including fractions
typedef struct {
  uint32_t last_day_idx; // last intersected day of this range (0=today, 1=yesterday, ...)
  uint32_t num_days; // number of intersected days for this range
  uint32_t seconds_first_day; // number of seconds on the oldest intersected day for this range
  uint32_t seconds_last_day; // number of seconds on the youngest intersected day for this range
  uint32_t seconds_total_last_day; // total number of seconds available on the youngest i. day
} HealthServiceTimeRange;


// since we expect clients to allocate this struct on the stack we make sure its size is limited
_Static_assert(sizeof(HealthServiceTimeRange) <= 160, "Helper struct too large for stack");

// Return the daily history of the given metric
bool health_service_private_get_metric_history(HealthMetric metric, uint32_t history_len,
                                               int32_t *history);

// wrapper around sys_activity_get_metric() to simplify migration of FW apps by indirectly
// giving access to ActivityMetricSleepEnterAtSeconds and ActivityMetricSleepExitAtSeconds
bool health_service_private_get_yesterdays_sleep_activity(HealthValue *enter_sec,
                                                          HealthValue *exit_sec);

// Utility callback function that can be used for the stats_calculate_basic() call. This
// particular callback returns true for all non-zero items.
// @param index the index of the daily total we are processing (0 = today, 1 = yesterday, etc.)
// @param value the value of the given daily total
// @tm_weekday_ref the context argument passed to the stats_calculate_basic() call, which
//  in this case is ignored
bool health_service_private_non_zero_filter(int index, int32_t value, void *tm_weekday_ref);

// Utility callback function that can be used for the stats_calculate_basic() call. This
// particular callback returns true for weekdays (Mon-Fri).
// @param index the index of the daily total we are processing (0 = today, 1 = yesterday, etc.)
// @param value the value of the given daily total
// @tm_weekday_ref the context argument passed to the stats_calculate_basic() call, which
//  in this case is the day of the week (Sunday, Monday, etc.) of index 0 in the daily totals,
//  typecast to a ptr.
bool health_service_private_weekday_filter(int index, int32_t value, void *tm_weekday_ref);

// Utility callback function that can be used for the stats_calculate_basic() call. This
// particular callback returns true for weekend days (Sat-Sun).
// @param index the index of the daily total we are processing (0 = today, 1 = yesterday, etc.)
// @param value the value of the given daily total
// @tm_weekday_ref the context argument passed to the stats_calculate_basic() call, which
//  in this case is the day of the week (Sunday, Monday, etc.) of index 0 in the daily totals,
//  typecast to a ptr.
bool health_service_private_weekend_filter(int index, int32_t value, void *tm_weekday_ref);

// Utility callback function that can be used for the stats_calculate_basic() call. This
// particular callback returns true for only days of the week that match tm_weekday_ref
// @param index the index of the daily total we are processing (0 = today, 1 = yesterday, etc.)
// @param value the value of the given daily total
// @tm_weekday_ref the context argument passed to the stats_calculate_basic() call, which
//  in this case is the day of the week (Sunday, Monday, etc.) of index 0 in the daily totals,
//  typecast to a ptr.
bool health_service_private_weekly_filter(int index, int32_t value, void *tm_weekday_ref);
