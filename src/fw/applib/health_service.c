/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "health_service.h"
#include "health_service_private.h"

#include "applib/app.h"
#include "applib/applib_malloc.auto.h"
#include "applib/pbl_std/pbl_std.h"
#include "event_service_client.h"
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "process_state/worker_state/worker_state.h"
#include "pbl/services/event_service.h"
#include "pbl/services/hrm/hrm_manager.h"
#include "pbl/services/activity/activity.h"
#include "shell/prefs_syscalls.h"
#include "syscall/syscall.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"
#include "util/stats.h"


// Fetching minute history can take a while, so we limit the amount of data we will ever access
// in one call to this
#define HS_MAX_MINUTE_DATA_SEC (2 * SECONDS_PER_HOUR)

// The limit to how old an HealthMetricHeartRateBPM sample can be and still return it within
// the peek function.
#define HS_MAX_AGE_HR_SAMPLE (15 * SECONDS_PER_MINUTE)

// ----------------------------------------------------------------------------------------------
static bool prv_is_heart_rate_metric(HealthMetric metric) {
  return (metric == HealthMetricHeartRateBPM) || (metric == HealthMetricHeartRateRawBPM);
}

// ----------------------------------------------------------------------------------------------
// Checks whether the interval between start and end are specifying a time within the past minute.
static bool prv_interval_within_last_minute(time_t now_utc, time_t start, time_t end) {
  const time_t last_minute = (now_utc - SECONDS_PER_MINUTE);
  const bool within_last_minute = ((start <= end) &&
                                   (start >= last_minute) &&
                                   (end <= now_utc));
  return within_last_minute;
}

static HealthAggregation prv_default_aggregation(HealthMetric metric) {
  switch (metric) {
    case HealthMetricStepCount:
    case HealthMetricActiveSeconds:
    case HealthMetricWalkedDistanceMeters:
    case HealthMetricSleepSeconds:
    case HealthMetricSleepRestfulSeconds:
    case HealthMetricRestingKCalories:
    case HealthMetricActiveKCalories:
      return HealthAggregationSum;
    case HealthMetricHeartRateBPM:
    case HealthMetricHeartRateRawBPM:
      return HealthAggregationAvg;
  }
  WTF;
  return 0;
}

// ----------------------------------------------------------------------------------------------
static HealthServiceState* prv_get_state(bool ensure_cache_initialized) {
  PebbleTask task = pebble_task_get_current();

  HealthServiceState *result = NULL;
  if (task == PebbleTask_App) {
    result = app_state_get_health_service_state();
  } else if (task == PebbleTask_Worker) {
    result = worker_state_get_health_service_state();
  } else {
    WTF;
  }

  // clients can free the cache by calling health_service_events_unsubscribe()
  if (result && ensure_cache_initialized && result->cache == NULL) {
    result->cache = applib_type_zalloc(HealthServiceCache);
  }

  return result;
}

// ----------------------------------------------------------------------------------------------
static void prv_health_service_deinit_cache(HealthServiceState *state) {
  if (state) {
    applib_free(state->cache);
    state->cache = NULL;
  }
}

// ----------------------------------------------------------------------------------------------
// returns a time_t of a given time that represents midnight of the given local time.
static time_t prv_get_midnight_of_local_time(time_t now) {
  struct tm *local_tm = pbl_override_gmtime(&now);
  local_tm->tm_hour = 0;
  local_tm->tm_min = 0;
  local_tm->tm_sec = 0;
  return pbl_override_mktime(local_tm);
}

// ----------------------------------------------------------------------------------------
// Return true if the passed in day is a weekend
static bool prv_is_weekend(DayInWeek day) {
  return (day == Sunday) || (day == Saturday);
}

// ----------------------------------------------------------------------------------------------
// Return the activity metric that maps to the given health metric. We separate the two because
// in the future, the health APIs may need to go other services besides just the Activity service
// to get information.
static ActivityMetric prv_get_activity_metric(HealthMetric metric) {
  switch (metric) {
    case HealthMetricStepCount:
      return ActivityMetricStepCount;
    case HealthMetricActiveSeconds:
      return ActivityMetricActiveSeconds;
    case HealthMetricWalkedDistanceMeters:
      return ActivityMetricDistanceMeters;
    case HealthMetricSleepSeconds:
      return ActivityMetricSleepTotalSeconds;
    case HealthMetricSleepRestfulSeconds:
      return ActivityMetricSleepRestfulSeconds;
    case HealthMetricRestingKCalories:
      return ActivityMetricRestingKCalories;
    case HealthMetricActiveKCalories:
      return ActivityMetricActiveKCalories;
    case HealthMetricHeartRateBPM:
      return ActivityMetricHeartRateFilteredBPM;
    case HealthMetricHeartRateRawBPM:
      return ActivityMetricHeartRateRawBPM;
  }
  WTF;
  return 0;
}

// ----------------------------------------------------------------------------------------------
// Return true if this metric is implemented for the given aggregation type
static bool prv_metric_aggregation_implemented(HealthMetric metric, time_t time_start,
                                               time_t time_end, HealthAggregation agg,
                                               HealthServiceTimeScope scope) {
  const time_t now_utc = sys_get_time();

  switch (metric) {
    case HealthMetricStepCount:
    case HealthMetricActiveSeconds:
    case HealthMetricWalkedDistanceMeters:
    case HealthMetricSleepSeconds:
    case HealthMetricSleepRestfulSeconds:
    case HealthMetricRestingKCalories:
    case HealthMetricActiveKCalories:
      // We can only use HealthAggregationSum with accumulating metrics and scope doesn't matter
      return (agg == HealthAggregationSum);
    case HealthMetricHeartRateRawBPM: {
      // Only support querying the current raw heart rate.
      const bool query_cur_minute = prv_interval_within_last_minute(now_utc, time_start, time_end);
      return ((agg == HealthAggregationAvg) && query_cur_minute);
    }
    case HealthMetricHeartRateBPM:
      // For heart rate, we can only support avg, min, max with constraints on time
      switch (agg) {
        case HealthAggregationSum:
          return false;
        case HealthAggregationAvg:
        {
          // We used to unconditionally return true here which was a bug
          // Fixing this bug broke some apps / watchfaces
          Version legacy_version = {.major = 0x5, .minor = 0x54};
          Version app_version = sys_get_current_app_sdk_version();
          if (version_compare(app_version, legacy_version) < 0) {
            return true;
          }
        }
        /* FALLTHRU */
        case HealthAggregationMax:
        case HealthAggregationMin: {
          // Only supported using minute data (short time range, no scope) because
          // we only store a few hours of HR minute data.
          return (scope == HealthServiceTimeScopeOnce)
                 && ((now_utc - time_start) <= HS_MAX_MINUTE_DATA_SEC);
        }
      }
      break;
  }
  WTF;
}

// ----------------------------------------------------------------------------------------------
// Return the daily historical values for the given metric, retrieving from the cache if
// possible.
static bool prv_get_metric_daily_history(HealthServiceState *state, HealthMetric metric,
                                         HealthServiceDailyHistory *daily) {
  // Return cached data if we have it available.
  if (state->cache && (metric == HealthMetricStepCount) && state->cache->step_daily_valid) {
    memcpy(daily, &state->cache->steps_daily, sizeof(*daily));
    // Get updated value for today. Getting only today's value is MUCH faster than getting
    // the historical values
    sys_activity_get_metric(prv_get_activity_metric(metric), 1, &daily->totals[0]);
    return true;
  }

  // Read in the metric history
  if (!sys_activity_get_metric(prv_get_activity_metric(metric),
                               ARRAY_LENGTH(daily->totals), daily->totals)) {
    PBL_LOG_ERR("Error fetching metric data");
    return false;
  }

  // Store in cache if we have space for it.
  if (state->cache && (metric == HealthMetricStepCount)) {
    memcpy(&state->cache->steps_daily, daily, sizeof(*daily));
    state->cache->step_daily_valid = true;
  }
  return true;
}

// ----------------------------------------------------------------------------------------------
// Compute all stats (weekly, daily, weekend, weekday, etc.) for the given metric.
// @param[in] state our API state
// @param[in] metric which metric to compute stats for
// @param[out] stats the stats for this metric are returned here
// @param[in] weekly_day which day of the week to use when computing the weekly stats
static bool prv_get_metric_stats(HealthServiceState *state, HealthMetric metric,
                                 HealthServiceMetricStats *stats, DayInWeek weekly_day) {
  // Get the daily history for this metric
  HealthServiceDailyHistory daily_totals;
  if (!prv_get_metric_daily_history(state, metric, &daily_totals)) {
    return false;
  }

  // What day of the week is it now?
  const time_t now_utc = sys_get_time();
  struct tm *local_tm = pbl_override_localtime(&now_utc);

  // Compute weekly, weekday, and daily stats
  *stats = (HealthServiceMetricStats) {};
  const StatsBasicOp op = (StatsBasicOp_Sum | StatsBasicOp_Average | StatsBasicOp_Count
    | StatsBasicOp_Min | StatsBasicOp_Max);
  stats_calculate_basic(op, daily_totals.totals, ARRAY_LENGTH(daily_totals.totals),
                        health_service_private_weekday_filter, (void *)(uintptr_t)local_tm->tm_wday,
                        &stats->weekday.sum);
  stats_calculate_basic(op, daily_totals.totals, ARRAY_LENGTH(daily_totals.totals),
                        health_service_private_weekend_filter, (void *)(uintptr_t)local_tm->tm_wday,
                        &stats->weekend.sum);
  // We want to sum only the days that are this far from index 0 (which is local_tm.tm_wday)
  int day_offset = local_tm->tm_wday - weekly_day;
  if (day_offset < 0) {
    day_offset += DAYS_PER_WEEK;
  }
  stats_calculate_basic(op, daily_totals.totals, ARRAY_LENGTH(daily_totals.totals),
                        health_service_private_weekly_filter, (void *)(uintptr_t)day_offset,
                        &stats->weekly.sum);

  // If the average is 0 (this can happen if we don't have any history), set the averages based
  // on today's total so far
  time_t seconds_today = now_utc - sys_time_start_of_today();
  HealthValue per_day_default = (daily_totals.totals[0] * SECONDS_PER_DAY)
    / MAX(1, seconds_today);
  if (stats->weekday.sum == 0) {
    stats->weekday = (HealthServiceStats) {
      .sum = per_day_default,
      .avg = per_day_default,
      .min = per_day_default,
      .max = per_day_default,
      .count = 1,
    };
  }
  if (stats->weekend.sum == 0) {
    stats->weekend = (HealthServiceStats) {
      .sum = per_day_default,
      .avg = per_day_default,
      .min = per_day_default,
      .max = per_day_default,
      .count = 1,
    };
  }

  // Daily is just the sum of weekend and weekday
  stats->daily.sum = stats->weekday.sum + stats->weekend.sum;
  stats->daily.count = stats->weekday.count + stats->weekend.count;
  stats->daily.avg = stats->daily.count ? stats->daily.sum / stats->daily.count : 0;
  stats->daily.min = MIN(stats->weekday.min, stats->weekend.min);
  stats->daily.max = MAX(stats->weekday.max, stats->weekend.max);

  return true;
}

// ----------------------------------------------------------------------------------------------
// Return intra-day averages for the given metric
static bool prv_get_intraday_averages(HealthServiceState *state, HealthMetric metric,
                                      ActivityMetricAverages *averages,
                                      DayInWeek day_in_week) {
  // If the cache is valid, return cached data.
  if (state->cache && (metric == HealthMetricStepCount)
    && state->cache->step_averages_valid && (day_in_week == state->cache->step_averages_day)) {
    memcpy(averages, &state->cache->step_averages, sizeof(*averages));
    return true;
  }

  // Fetch the intraday averages, if available
  if (state->cache && (metric == HealthMetricStepCount)) {
    // Fill the cache if this is step count
    sys_activity_get_step_averages(day_in_week, &state->cache->step_averages);
    state->cache->step_averages_day = day_in_week;
    state->cache->step_averages_valid = true;
    memcpy(averages, &state->cache->step_averages, sizeof(*averages));
  } else if (metric == HealthMetricStepCount) {
    // For step count, we have intraday averages available
    sys_activity_get_step_averages(day_in_week, averages);
  } else {
    // For other metrics, we don't.
    memset(averages->average, ACTIVITY_METRIC_AVERAGES_UNKNOWN & 0xFF, sizeof(averages->average));
  }

  // If all metric averages are unknown, we will plug in a default
  bool use_default = true;
  if (metric == HealthMetricStepCount) {
    for (int i = 0; i < ACTIVITY_NUM_METRIC_AVERAGES; i++) {
      if (averages->average[i] != ACTIVITY_METRIC_AVERAGES_UNKNOWN) {
        use_default = false;
        break;
      }
    }
  }

  // Compute the default average value
  uint16_t default_value = 0;
  if (use_default) {
    HealthServiceMetricStats stats;
    if (!prv_get_metric_stats(state, metric, &stats, day_in_week)) {
      return false;
    }

    HealthValue value_per_day = stats.weekly.avg;
    default_value = value_per_day / ACTIVITY_NUM_METRIC_AVERAGES;
  }

  // Plug in the default value for any entries which are unknown.
  for (int i = 0; i < ACTIVITY_NUM_METRIC_AVERAGES; i++) {
    if (averages->average[i] == ACTIVITY_METRIC_AVERAGES_UNKNOWN) {
      averages->average[i] = default_value;
    }
    // If this entry is cached, fix up the cache entry
    if (state->cache && (metric == HealthMetricStepCount)) {
      if (state->cache->step_averages.average[i] == ACTIVITY_METRIC_AVERAGES_UNKNOWN) {
        state->cache->step_averages.average[i] = default_value;
      }
    }
  }
  return true;
}

// ----------------------------------------------------------------------------------------------
// Compute the sum of the chunks in the averages array that comprise the given time range from
// time_start to time_end. The averages array represents all the chunks for a day, and time_start
// to time_end is always <= 1 day.
static HealthValue prv_sum_intraday_averages(ActivityMetricAverages *averages, time_t time_start,
                                             time_t time_end) {
  PBL_ASSERTN((time_end - time_start) <= SECONDS_PER_DAY);
  struct tm *local_tm = pbl_override_localtime(&time_start);

  // Add up the metric averages for the passed in time range
  time_t chunk_start_time = time_start;
  const int k_seconds_per_step_avg = SECONDS_PER_DAY / ACTIVITY_NUM_METRIC_AVERAGES;
  unsigned int second_idx = local_tm->tm_hour * SECONDS_PER_HOUR
    + local_tm->tm_min * SECONDS_PER_MINUTE + local_tm->tm_sec;
  unsigned int chunk_idx = second_idx / k_seconds_per_step_avg;

  HealthValue result = 0;
  while (chunk_start_time < time_end) {
    int seconds_left = time_end - chunk_start_time;
    int seconds_in_chunk = k_seconds_per_step_avg - (second_idx % k_seconds_per_step_avg);
    seconds_in_chunk = MIN(seconds_left, seconds_in_chunk);

    if (averages->average[chunk_idx] != ACTIVITY_METRIC_AVERAGES_UNKNOWN) {
      if (seconds_in_chunk == k_seconds_per_step_avg) {
        result += averages->average[chunk_idx];
      } else {
        result += averages->average[chunk_idx] * seconds_in_chunk / k_seconds_per_step_avg;
      }
    }

    // Increment indices and time to the next chunk
    chunk_start_time += seconds_in_chunk;
    second_idx += seconds_in_chunk;
    second_idx %= SECONDS_PER_DAY;

    chunk_idx++;
    chunk_idx %= ACTIVITY_NUM_METRIC_AVERAGES;
  }

  return result;
}

// ----------------------------------------------------------------------------------------------
// Fills in the range structure based on time_start and time_end. This computes the following
// values:
// * How many whole days of data are needed to include time_start and time_end (range->num_days).
//   This will alway be >= 1.
// * The index of the last day in the range relative to today (0 means today, 1 means yesterday,
//    etc.) (range->last_day_idx).
// * How many seconds from the range we should count from the first day (range->seconds_first_day)
// * How many seconds from the range we should count from the last day (range->seconds_last_day)
// * How many seconds of data we have collected for the last day of the range
//    (range->seconds_total_last_day)
T_STATIC bool prv_calculate_time_range(time_t time_start, time_t time_end,
                                       HealthServiceTimeRange *range) {
  // as the data set from activity_get_metric() uses day boundaries in local time we
  // need to convert the arguments to local time
  const time_t now = sys_time_utc_to_local(sys_get_time());
  time_start = sys_time_utc_to_local(time_start);
  time_end = sys_time_utc_to_local(time_end);

  // we use this value as a reference to calculate the range of valid data entries
  const time_t midnight_after_now = prv_get_midnight_of_local_time(now) + SECONDS_PER_DAY;

  // never work with values in the future
  time_end = MIN(time_end, now);
  // never work with values older than the supported history of data
  time_start = MAX(time_start,
                   midnight_after_now - (SECONDS_PER_DAY * ACTIVITY_HISTORY_DAYS));
  if (time_end < time_start) {
    return false;
  }

  if (range) {
    const time_t midnight_before_start = prv_get_midnight_of_local_time(time_start);
    const time_t midnight_before_end = prv_get_midnight_of_local_time(time_end);
    // we treat time_end as exclusive, if one passes exactly midnight, we don't count that day
    const time_t midnight_after_end = (midnight_before_end == time_end) ?
                                      midnight_before_end :
                                      (midnight_before_end + SECONDS_PER_DAY);

    // no additional range changes (e.g. < 0 or >= ACTIVITY_HISTORY_DAYS needed due to checks above)
    range->last_day_idx = (midnight_after_now - midnight_after_end) / SECONDS_PER_DAY;

    // always positive and <= ACTIVITY_HISTORY_DAYS due to check above
    range->num_days = (midnight_after_end - midnight_before_start) / SECONDS_PER_DAY;

    // we calculate how many seconds are covered on the first/last day of the range to allow
    // clients to do some interpolation.
    // if there's only one day, we return the number of seconds in the total range for both values
    const uint32_t seconds_first_day = SECONDS_PER_DAY - (time_start - midnight_before_start);
    // compensate for cases where time_end is on a day boundary
    const uint32_t seconds_last_day = (time_end == midnight_before_end) ?
                                      SECONDS_PER_DAY : (time_end - midnight_before_end);
    const uint32_t total_seconds = time_end - time_start;

    range->seconds_first_day = (range->num_days == 1) ? total_seconds : seconds_first_day;
    range->seconds_last_day = (range->num_days == 1) ? total_seconds : seconds_last_day;
    range->seconds_total_last_day =
      (range->last_day_idx == 0) ? (now - midnight_before_end) : SECONDS_PER_DAY;
  }

  return true;
}

// ----------------------------------------------------------------------------------------------
// Fill in the time_range and daily_history structures for this metric and time range.
// Returns HealthServiceAccessibilityMaskAvailable if this time span and metric are accessible.
static HealthServiceAccessibilityMask prv_get_range_and_daily_history(
  HealthServiceState *state, HealthMetric metric, time_t time_start, time_t time_end,
  HealthServiceTimeRange *time_range, HealthServiceDailyHistory *daily_history) {
  PBL_ASSERTN((time_range != NULL) && (daily_history != NULL));

  // TODO: PBL-31628 permission system to reply with HealthServiceAccessibilityMaskNoPermission

  if (!prv_get_metric_daily_history(state, metric, daily_history)) {
    return HealthServiceAccessibilityMaskNotAvailable;
  }

  if (!prv_calculate_time_range(time_start, time_end, time_range)) {
    return HealthServiceAccessibilityMaskNotAvailable;
  }

  return HealthServiceAccessibilityMaskAvailable;
}

// ----------------------------------------------------------------------------------------------
// This adjusts the values in the values array that represent the first and last day of
// the given time range. If either of these are not totally included in the time range, we
// decrease their value proportionally to how many seconds in the range overlap them.
T_STATIC void prv_adjust_value_boundaries(HealthValue *values, size_t num_values,
                                          const HealthServiceTimeRange *range) {
  PBL_ASSERTN(values && range && range->seconds_total_last_day > 0);

  if (((range->last_day_idx + range->num_days) > num_values) || (range->num_days < 1)) {
    return;
  }

  // as all indices inside of values[] are relative to range.last_day_idx, we adjust the pointer
  // once here to simplify the following lines
  values += range->last_day_idx;

  // last day might not be complete, yet (as it can be today)
  values[0] =
    (HealthValue)(((int64_t)values[0] * range->seconds_last_day) / range->seconds_total_last_day);

  // only process first day if its in range and does not overlap with the last day
  if ((range->num_days > 1) && (num_values >= range->num_days)) {
    const uint32_t oldest_day_idx = range->num_days - 1;
    values[oldest_day_idx] =
      (HealthValue)(((int64_t)values[oldest_day_idx] * range->seconds_first_day) / SECONDS_PER_DAY);
  }
}


// ----------------------------------------------------------------------------------------------
static HealthValue prv_compute_aggregate_using_daily_totals(
  HealthServiceState *state, HealthMetric metric, time_t time_start, time_t time_end,
  HealthAggregation aggregation) {
  HealthServiceTimeRange time_range = {};
  HealthServiceDailyHistory daily_history = {};

  const HealthServiceAccessibilityMask accessible =
    prv_get_range_and_daily_history(state, metric, time_start, time_end, &time_range,
                                    &daily_history);
  if (accessible != HealthServiceAccessibilityMaskAvailable) {
    return 0;
  }

  // If we are summing, scale the values for the first and last day of the time range. For
  // min, max, and avg scaling does not apply.
  if (aggregation == HealthAggregationSum) {
    prv_adjust_value_boundaries(daily_history.totals, ARRAY_LENGTH(daily_history.totals),
                                &time_range);
  }

  HealthValue result = 0;
  switch (aggregation) {
    case HealthAggregationSum:
    case HealthAggregationAvg:
      for (uint32_t i = 0; i < time_range.num_days; i++) {
        result += daily_history.totals[i + time_range.last_day_idx];
      }
      if (aggregation == HealthAggregationAvg) {
        result = ROUND(result, time_range.num_days);
      }
      break;

    case HealthAggregationMax:
      result = INT32_MIN;
      for (uint32_t i = 0; i < time_range.num_days; i++) {
        result = MAX(result, daily_history.totals[i + time_range.last_day_idx]);
      }
      break;

    case HealthAggregationMin:
      result = INT32_MAX;
      for (uint32_t i = 0; i < time_range.num_days; i++) {
        result = MIN(result, daily_history.totals[i + time_range.last_day_idx]);
      }
      break;
  }
  return result;
}


// ----------------------------------------------------------------------------------------------
// Compute the value of the given metric using aggregation and averaging based on daily history
// values.
static HealthValue prv_compute_aggregate_averaged_using_daily_totals(
  HealthServiceState *state, HealthMetric metric, time_t time_start, time_t time_end,
  HealthAggregation aggregation, HealthServiceTimeScope scope) {
  PBL_ASSERTN(scope != HealthServiceTimeScopeOnce);

  // What day of the week is the scope for? For now, we will use the day of the week that
  // time_start falls on. In the future, we could be better about blending weekday with weekend
  // if the time range spans both
  struct tm *local_tm = pbl_override_localtime(&time_start);
  bool is_weekend = prv_is_weekend(local_tm->tm_wday);

  // Compute all stats
  HealthServiceMetricStats stats;
  if (!prv_get_metric_stats(state, metric, &stats, local_tm->tm_wday)) {
    return 0;
  }

  // Return the appropriate statistic given the scope and aggregation
  HealthValue result = 0;
  HealthServiceStats *which_stats = NULL;
  if (scope == HealthServiceTimeScopeDaily) {
    which_stats = &stats.daily;
  } else if (scope == HealthServiceTimeScopeDailyWeekdayOrWeekend) {
    if (is_weekend) {
      which_stats = &stats.weekend;
    } else {
      which_stats = &stats.weekday;
    }
  } else if (scope == HealthServiceTimeScopeWeekly) {
    which_stats = &stats.weekly;
  } else {
    APP_LOG(APP_LOG_LEVEL_ERROR, "Unsupported scope: %d", (int) scope);
    result = 0;
  }

  // Get the result
  switch (aggregation) {
    case HealthAggregationSum:
      // NOTE: the caller is asking for "sum" aggregation, but we only have one value stored per
      // day, so we just need to compute the average amongst all the days.
      result = which_stats->avg;
      break;
    case HealthAggregationAvg:
      result = which_stats->avg;
      break;
    case HealthAggregationMin:
      result = which_stats->min;
      break;
    case HealthAggregationMax:
      result = which_stats->max;
      break;
  }

  // Scale result by the actual amount of requested time if asked for a sum
  if (aggregation == HealthAggregationSum) {
    result = result * (time_end - time_start) / SECONDS_PER_DAY;
  }
  return result;
}

// ----------------------------------------------------------------------------------------------
// Compute the aggregated value of the given metric using values from minute history
static HealthValue prv_compute_aggregate_using_minute_history(
  HealthServiceState *state, HealthMetric metric, time_t time_start, time_t time_end,
  HealthAggregation aggregation) {
  // Currently only implemented for heart rate BPM
  PBL_ASSERTN(metric == HealthMetricHeartRateBPM);

  // Can't execute this call if no cache
  if (!state->cache) {
    APP_LOG(APP_LOG_LEVEL_ERROR, "Not enough memory for health cache");
    return 0;
  }

  HealthValue value = 0;
  uint32_t num_samples = 0;
  switch (aggregation) {
    case HealthAggregationSum:
      WTF;    // Not supported
      break;
    case HealthAggregationAvg:
      value = 0;
      break;
    case HealthAggregationMin:
      value = INT32_MAX;
      break;
    case HealthAggregationMax:
      value = INT32_MIN;
      break;
  }

  // If the current value is within the time range, incorporate it into the stats
  time_t now_utc = sys_get_time();
  if (time_end > now_utc - SECONDS_PER_MINUTE) {
    HealthValue current_value;
    bool success = sys_activity_get_metric(ActivityMetricHeartRateRawBPM, 1, &current_value);
    if (success && current_value != 0) {
      num_samples++;
      value = current_value;
    }
  }

  HealthMinuteData *minute_data = state->cache->minute_data;
  bool more_data = true;
  while (more_data && (time_start < time_end)) {
    uint32_t num_records = ARRAY_LENGTH(state->cache->minute_data);
    PBL_LOG_DBG("Fetching %"PRIu32" minute records for %d to %d...", num_records,
            (int)time_start, (int)time_end);
    bool success = sys_activity_get_minute_history(minute_data, &num_records, &time_start);
    if (!success) {
      APP_LOG(APP_LOG_LEVEL_WARNING, "Error fetching minute history");
      break;
    }
    PBL_LOG_DBG("   Got %"PRIu32" minute records for %d", num_records,
            (int)time_start);
    if (num_records == 0) {
      // No more data available
      more_data = false;
      break;
    }

    // Update the metric from this new batch of data
    for (unsigned i = 0; (i < num_records) && (time_start < time_end);
         i++, time_start += SECONDS_PER_MINUTE) {
      if (minute_data[i].heart_rate_bpm == 0) {
        // Ignore minutes that have no heart rate BPM
        continue;
      }
      num_samples++;
      switch (aggregation) {
        case HealthAggregationAvg:
          value += minute_data[i].heart_rate_bpm;
          break;
        case HealthAggregationMax:
          value = MAX(value, minute_data[i].heart_rate_bpm);
          break;
        case HealthAggregationMin:
          value = MIN(value, minute_data[i].heart_rate_bpm);
          break;
        case HealthAggregationSum:
          WTF;
          break;
      }
    }
  }

  // Post-process the metric if necessary
  if (aggregation == HealthAggregationAvg) {
    if (num_samples > 0) {
      value = ROUND(value, num_samples);
    }
  }
  if (num_samples == 0) {
    // Error case: no samples
    value = 0;
  }
  return value;
}

// ---------------------------------------------------------------------------------------------
// Init a metric alert info structure
static void prv_init_metric_alert(HealthServiceState *state, HealthMetric metric,
                                  HealthValue threshold, HealthServiceMetricAlertInfo *info) {
  int32_t value = 0;
  sys_activity_get_metric(prv_get_activity_metric(metric), 1, &value);
  info->prior_reading = value;
  info->threshold = threshold;
}

#if !defined(CONFIG_RECOVERY_FW)
// ---------------------------------------------------------------------------------------------
// Determine if we should generate a health metric alert event
static void prv_check_and_generate_metric_alert(HealthServiceState *state, HealthMetric metric,
                                                HealthServiceMetricAlertInfo *info) {
  if (info->threshold == 0) {
    // No threshold set
    return;
  }
  int32_t value;
  bool success = sys_activity_get_metric(prv_get_activity_metric(metric), 1, &value);
  if (!success) {
    return;
  }
  bool went_above = ((value > info->threshold) && (info->prior_reading < info->threshold));
  bool went_below = ((value < info->threshold) && (info->prior_reading > info->threshold));
  if (went_above || went_below) {
    state->event_handler(HealthEventMetricAlert, state->context);
    info->prior_reading = value;
  }
}
#endif

// ----------------------------------------------------------------------------------------------
uint16_t health_service_peek_hrv_ppi_ms(void) {
  HealthServiceState *state = prv_get_state(false);
  if (!state) {
    return 0;
  }
  return state->last_hrv_ppi_ms;
}

// ----------------------------------------------------------------------------------------------
T_STATIC void prv_health_event_handler(PebbleEvent *e, void *context) {
#if !defined(CONFIG_RECOVERY_FW)
  HealthServiceState *state = prv_get_state(true);
  PBL_ASSERTN(state && state->event_handler != NULL);

  // If this is a significant update event, invalidate our cache
  if (e->health_event.type == HealthEventHRVUpdate) {
    state->last_hrv_ppi_ms = e->health_event.data.hrv_update.ppi_ms;
  }
  else if (e->health_event.type == HealthEventSignificantUpdate) {
    if (state->cache) {
      state->cache->valid_flags = 0;
    }
  }

  // If this is a step update, update the cached value for today
  else if (e->health_event.type == HealthEventMovementUpdate) {
    if (state->cache) {
      state->cache->steps_daily.totals[0] = e->health_event.data.movement_update.steps;
    }
  }

  state->event_handler(e->health_event.type, state->context);

  // If we crossed an alert threshold, generate a metric alert event
  if (state->cache) {
    prv_check_and_generate_metric_alert(state, HealthMetricHeartRateBPM,
                                        &state->cache->alert_threshold_heart_rate);
  }
#endif // !defined(CONFIG_RECOVERY_FW)
}

// ----------------------------------------------------------------------------------------------
T_STATIC bool prv_activity_session_matches(const ActivitySession *session, HealthActivityMask mask,
                                           time_t time_start, time_t time_end) {
  PBL_ASSERTN(session);

  const bool type_matches =
       (session->type == ActivitySessionType_Sleep && ((mask & HealthActivitySleep) > 0))
    || (session->type == ActivitySessionType_Nap && ((mask & HealthActivitySleep) > 0))
    || (session->type == ActivitySessionType_RestfulSleep && ((mask & HealthActivityRestfulSleep) > 0))
    || (session->type == ActivitySessionType_RestfulNap && ((mask & HealthActivityRestfulSleep) > 0))
    || (session->type == ActivitySessionType_Walk && ((mask & HealthActivityWalk) > 0))
    || (session->type == ActivitySessionType_Run && ((mask & HealthActivityRun) > 0))
    || (session->type == ActivitySessionType_Open && ((mask & HealthActivityOpenWorkout) > 0));
  if (!type_matches) {
    return false;
  }

  unsigned int length_sec = session->length_min * SECONDS_PER_MINUTE;
  const bool time_matches = session->start_utc < time_end &&
    (time_t)(session->start_utc + length_sec) > time_start;
  return time_matches;
}

// ----------------------------------------------------------------------------------------------
T_STATIC int64_t prv_session_compare(const ActivitySession *a, const ActivitySession *b,
                                     HealthIterationDirection direction) {
  PBL_ASSERTN(a && b);

  switch (direction) {
    case HealthIterationDirectionPast:
      // sessions that end later come first
      return (b->start_utc + (b->length_min * SECONDS_PER_MINUTE))
        - (a->start_utc + (a->length_min * SECONDS_PER_MINUTE));
    case HealthIterationDirectionFuture:
      // sessions that start earlier come first
      return a->start_utc - b->start_utc;
    default:
      WTF;
  }
}

// ----------------------------------------------------------------------------------------------
static void prv_sessions_sort(ActivitySession *sessions, const uint32_t num_sessions,
                              HealthIterationDirection direction) {
  // as the number of sessions is expected to be small (<=16), and we don't seem to have a generic
  // sort implementation, we do a simple bubble sort here
  for (uint32_t i = 0; i + 1 < num_sessions; i++) {
    for (uint32_t j = i + 1; j < num_sessions; j++) {
      if (prv_session_compare(&sessions[i], &sessions[j], direction) > 0) {
        ActivitySession temp = sessions[i];
        sessions[i] = sessions[j];
        sessions[j] = temp;
      }
    }
  }
}

// ----------------------------------------------------------------------------------------------
static MeasurementSystem prv_get_shell_prefs_metric_for_distance(void) {
  switch (sys_shell_prefs_get_units_distance()) {
    case UnitsDistance_Miles:
      return MeasurementSystemImperial;
    case UnitsDistance_KM:
      return MeasurementSystemMetric;
    default:
      return MeasurementSystemUnknown;
  }
}


// ----------------------------------------------------------------------------------------------
// Filter callbacks used by stats_calculate_basic()
bool health_service_private_non_zero_filter(int index, int32_t value, void *context) {
  return (index > 0 && value > 0);
}

bool health_service_private_weekday_filter(int index, int32_t value, void *tm_weekday_ref) {
  const int tm_weekday = (int)(uintptr_t)tm_weekday_ref;
  return (health_service_private_non_zero_filter(index, value, NULL) &&
    IS_WEEKDAY(positive_modulo(tm_weekday - index, DAYS_PER_WEEK)));
}

bool health_service_private_weekend_filter(int index, int32_t value, void *tm_weekday_ref) {
  const int tm_weekday = (int)(uintptr_t)tm_weekday_ref;
  return (health_service_private_non_zero_filter(index, value, NULL) &&
    IS_WEEKEND(positive_modulo(tm_weekday - index, DAYS_PER_WEEK)));
}

bool health_service_private_weekly_filter(int index, int32_t value, void *tm_weekday_ref) {
  const int tm_weekday = (int)(uintptr_t)tm_weekday_ref;
  return (health_service_private_non_zero_filter(index, value, NULL) &&
    (positive_modulo(tm_weekday - index, DAYS_PER_WEEK) == 0));
}



// ----------------------------------------------------------------------------------------------
bool health_service_private_get_metric_history(HealthMetric metric, uint32_t history_len,
                                               int32_t *history) {
  // Look up which activity metric maps to the given health metric. We separate the two because
  // in the future, the health APIs may need to go other services besides just the Activity service
  // to get information.
  ActivityMetric act_metric = prv_get_activity_metric(metric);
  return sys_activity_get_metric(act_metric, history_len, history);
}

// ----------------------------------------------------------------------------------------------
HealthServiceAccessibilityMask health_service_metric_accessible(
  HealthMetric metric, time_t time_start, time_t time_end) {
  if (!sys_activity_is_initialized()) {
    return HealthServiceAccessibilityMaskNotAvailable;
  }
  return health_service_metric_aggregate_averaged_accessible(metric, time_start, time_end,
                                                             prv_default_aggregation(metric),
                                                             HealthServiceTimeScopeOnce);
}

// ----------------------------------------------------------------------------------------------
HealthServiceAccessibilityMask health_service_metric_averaged_accessible(
  HealthMetric metric, time_t time_start, time_t time_end, HealthServiceTimeScope scope) {
  if (!sys_activity_is_initialized()) {
    return HealthServiceAccessibilityMaskNotAvailable;
  }
  return health_service_metric_aggregate_averaged_accessible(metric, time_start, time_end,
                                                             prv_default_aggregation(metric),
                                                             scope);
}

// ----------------------------------------------------------------------------------------------
HealthServiceAccessibilityMask health_service_metric_aggregate_averaged_accessible(
  HealthMetric metric, time_t time_start, time_t time_end, HealthAggregation aggregation,
  HealthServiceTimeScope scope) {
  if (!sys_activity_is_initialized()) {
    return HealthServiceAccessibilityMaskNotAvailable;
  }
  if (prv_is_heart_rate_metric(metric) && !sys_activity_prefs_heart_rate_is_enabled()) {
    return HealthServiceAccessibilityMaskNoPermission;
  }

  if (!prv_metric_aggregation_implemented(metric, time_start, time_end, aggregation, scope)) {
    return HealthServiceAccessibilityMaskNotSupported;
  }

  // Get our state
  HealthServiceState *state = prv_get_state(false);

  HealthServiceTimeRange time_range = {};
  HealthServiceDailyHistory daily_history = {};

  const HealthServiceAccessibilityMask accessible =
    prv_get_range_and_daily_history(state, metric, time_start, time_end, &time_range,
                                    &daily_history);
  if (accessible != HealthServiceAccessibilityMaskAvailable) {
    return accessible;
  }

  for (size_t i = 0; i < time_range.num_days; i++) {
    if (daily_history.totals[time_range.last_day_idx + i] >= 0) {
      return HealthServiceAccessibilityMaskAvailable;
    }
  }

  return HealthServiceAccessibilityMaskNotAvailable;
}


// ----------------------------------------------------------------------------------------------
HealthValue health_service_sum_today(HealthMetric metric) {
  if (!sys_activity_is_initialized()) {
    return 0;
  }
  const time_t today_midnight = sys_time_start_of_today();
  const time_t tomorrow_midnight = today_midnight + SECONDS_PER_DAY;
  return health_service_sum(metric, today_midnight, tomorrow_midnight);
}


// ----------------------------------------------------------------------------------------------
HealthValue health_service_sum(HealthMetric metric, time_t time_start, time_t time_end) {
  if (!sys_activity_is_initialized()) {
    return 0;
  }
  return health_service_aggregate_averaged(metric, time_start, time_end, HealthAggregationSum,
                                           HealthServiceTimeScopeOnce);
}

// ----------------------------------------------------------------------------------------------
// Compute the sum of a metric, but averaged over multiple days.
HealthValue health_service_sum_averaged(HealthMetric metric, time_t time_start, time_t time_end,
                                        HealthServiceTimeScope scope) {
  if (!sys_activity_is_initialized()) {
    return 0;
  }
  return health_service_aggregate_averaged(metric, time_start, time_end, HealthAggregationSum,
                                           scope);
}

// ----------------------------------------------------------------------------------------------
HealthValue health_service_peek_current_value(HealthMetric metric) {
  if (!sys_activity_is_initialized()) {
    return 0;
  }
  time_t now_utc = sys_get_time();
  return health_service_aggregate_averaged(metric, now_utc, now_utc, HealthAggregationAvg,
                                           HealthServiceTimeScopeOnce);
}


static HealthValue prv_hr_aggregate_averaged(HealthServiceState *state, HealthMetric metric,
                                             time_t time_start, time_t time_end,
                                             HealthAggregation aggregation) {
  PBL_ASSERTN(metric == HealthMetricHeartRateBPM || metric == HealthMetricHeartRateRawBPM);

  time_t now_utc = sys_get_time();
  const bool query_cur_minute = prv_interval_within_last_minute(now_utc, time_start, time_end);
  const bool valid_hr_sample_num = ((now_utc - time_start) <= HS_MAX_MINUTE_DATA_SEC);

  if (metric == HealthMetricHeartRateBPM) {
    if (query_cur_minute) {
      // If the client is querying the service for the most recent Stable/Median/Filtered value
      // and it is within the last X minutes, return it. If it's older than X minutes, return 0.
      // This is the behavior we shipped in FW 4.1, so we must keep it this way. We have added a new
      // metric HealthMetricHeartRateRawBPM if the user wants the most recent reading.
      HealthValue value;
      sys_activity_get_metric(ActivityMetricHeartRateFilteredUpdatedTimeUTC, 1, &value);
      const time_t hr_median_age = now_utc - value;
      if (hr_median_age >= HS_MAX_AGE_HR_SAMPLE) {
        return 0;
      }
      sys_activity_get_metric(ActivityMetricHeartRateFilteredBPM, 1, &value);
      return value;
    } else if (valid_hr_sample_num) {
      // If this is scope-once, the metric is BPM, and the time range is less than
      // HS_MAX_MINUTE_DATA_SEC, we can use minute history since the amount of data is manageable.
      return prv_compute_aggregate_using_minute_history(state, metric, time_start, time_end,
                                                        aggregation);
    }
  } else if (metric == HealthMetricHeartRateRawBPM) {
    // We don't allow the user to gather data from raw HR samples. Only return the current and
    // return. If time_start and time_end were not for the current time, they are filtered out
    // by the function above `prv_metric_aggregation_implemented`.
    int32_t raw_bpm;
    sys_activity_get_metric(ActivityMetricHeartRateRawBPM, 1, &raw_bpm);
    return raw_bpm;
  }

  // Invalid
  return 0;
}

// ----------------------------------------------------------------------------------------------
HealthValue health_service_aggregate_averaged(HealthMetric metric, time_t time_start,
                                              time_t time_end, HealthAggregation aggregation,
                                              HealthServiceTimeScope scope) {
  if (!sys_activity_is_initialized()) {
    return 0;
  }
  // Make sure this metric is supported by this type of aggregation
  if (!prv_metric_aggregation_implemented(metric, time_start, time_end, aggregation, scope)) {
    return 0;
  }

  // Get our state
  HealthServiceState *state = prv_get_state(true);

  if (scope == HealthServiceTimeScopeOnce && prv_is_heart_rate_metric(metric)) {
    return prv_hr_aggregate_averaged(state, metric, time_start, time_end, aggregation);
  }

  // --------
  // If asked for an averaged sum over less than a day, we can use the intraday averages
  if ((scope != HealthServiceTimeScopeOnce) && (aggregation == HealthAggregationSum)
    && ((time_end - time_start) < SECONDS_PER_DAY)) {
    // For now, we will use the day of the week that time_start falls on. In the future, we could
    // be better about blending weekday with weekend if the time range spans both
    struct tm *local_tm = pbl_override_localtime(&time_start);
    bool is_weekend = prv_is_weekend(local_tm->tm_wday);

    ActivityMetricAverages averages;
    unsigned num_sums = 0;
    HealthValue result = 0;
    if (scope == HealthServiceTimeScopeWeekly) {
      if (prv_get_intraday_averages(state, metric, &averages, local_tm->tm_wday)) {
        result += prv_sum_intraday_averages(&averages, time_start, time_end);
        num_sums++;
      }

    } else if ((scope == HealthServiceTimeScopeDaily)
      || (scope == HealthServiceTimeScopeDailyWeekdayOrWeekend)) {
      for (DayInWeek day = Sunday; day <= Saturday; day++) {
        if (scope == HealthServiceTimeScopeDailyWeekdayOrWeekend) {
          if (is_weekend != prv_is_weekend(day)) {
            continue;
          }
        }
        if (prv_get_intraday_averages(state, metric, &averages, day)) {
          result += prv_sum_intraday_averages(&averages, time_start, time_end);
          num_sums++;
        }
      }

    } else {
      APP_LOG(APP_LOG_LEVEL_ERROR, "Unsupported scope: %d", (int) scope);
      result = 0;
    }
    if (num_sums > 0) {
      result = ROUND(result, num_sums);
    }
    return result;
  }

  // --------
  // Default handling is to use daily totals
  if (scope == HealthServiceTimeScopeOnce) {
    return prv_compute_aggregate_using_daily_totals(state, metric, time_start, time_end,
                                                    aggregation);
  } else {
    return prv_compute_aggregate_averaged_using_daily_totals(state, metric, time_start, time_end,
                                                             aggregation, scope);
  }

}

// ----------------------------------------------------------------------------------------------
bool health_service_events_subscribe(HealthEventHandler handler, void *context) {
  HealthServiceState *state = prv_get_state(true);
  if (!state) {
    return false;
  }
  state->event_handler = handler;
  state->context = context;
  event_service_client_subscribe(&state->health_event_service_info);

  // Post a "significant update" event
  PebbleEvent event = {
    .type = PEBBLE_HEALTH_SERVICE_EVENT,
    .health_event = {
      .type = HealthEventSignificantUpdate,
      .data.significant_update = {
        .day_id = 0,
      },
    },
  };
  sys_send_pebble_event_to_kernel(&event);

  return true;
}

// ----------------------------------------------------------------------------------------------
bool health_service_events_unsubscribe(void) {
  HealthServiceState *state = prv_get_state(false);
  event_service_client_unsubscribe(&state->health_event_service_info);
  state->event_handler = NULL;
  prv_health_service_deinit_cache(state);
  return true;
}

// ----------------------------------------------------------------------------------------------
HealthMetricAlert *health_service_register_metric_alert(HealthMetric metric,
                                                        HealthValue threshold) {
  if (prv_is_heart_rate_metric(metric) && !sys_activity_prefs_heart_rate_is_enabled()) {
    return NULL;
  }

  HealthServiceState *state = prv_get_state(true);
  if (!state->cache) {
    return NULL;
  }

  switch (metric) {
    case HealthMetricHeartRateBPM:
      // If already registered, it's an error since we only have room for one registration per
      // metric right now
      if (state->cache->alert_threshold_heart_rate.threshold != 0) {
        APP_LOG(APP_LOG_LEVEL_INFO, "Only 1 alert allowed per metric");
        return NULL;
      }
      prv_init_metric_alert(state, HealthMetricHeartRateBPM, threshold,
                            &state->cache->alert_threshold_heart_rate);
      return (void *)HealthMetricHeartRateBPM;
    default:
      return NULL;
  }
}

// ----------------------------------------------------------------------------------------------
bool health_service_cancel_metric_alert(HealthMetricAlert *alert) {
  HealthServiceState *state = prv_get_state(true);
  if (!state->cache) {
    return NULL;
  }

  HealthMetric metric = (HealthMetric)alert;
  if (prv_is_heart_rate_metric(metric) && !sys_activity_prefs_heart_rate_is_enabled()) {
    return false;
  }

  switch (metric) {
    case HealthMetricHeartRateBPM:
      state->cache->alert_threshold_heart_rate = (HealthServiceMetricAlertInfo) {};
      return true;
    default:
      return false;
  }
}

// ----------------------------------------------------------------------------------------------
bool health_service_set_hrv_sample_period(uint16_t interval_sec) {
#if !defined(CONFIG_HRM) || !defined(CONFIG_HRM_HRV)
  return false;
#else
  if (!sys_activity_is_initialized()) {
    return false;
  }
  if (!sys_activity_prefs_heart_rate_is_enabled()) {
    return false;
  }

  AppInstallId app_id = app_get_app_id();
  if (app_id == INSTALL_ID_INVALID) {
    return false;
  }

  // If interval is 0, the caller wants to unsubscribe
  if (interval_sec == 0) {
    HRMSessionRef hrm_session = sys_hrm_manager_get_app_subscription(app_id);
    if (hrm_session != HRM_INVALID_SESSION_REF) {
      sys_hrm_manager_unsubscribe(hrm_session);
    }
    return true;
  }

  // Subscribe with the HRV feature; the sensor enables HRV while any such subscription is active.
  HRMSessionRef hrm_session = sys_hrm_manager_app_subscribe(app_id, interval_sec, 0 /*expire_sec*/,
                                                            HRMFeature_BPM | HRMFeature_HRV);
  return (hrm_session != HRM_INVALID_SESSION_REF);
#endif
}

bool health_service_set_heart_rate_sample_period(uint16_t interval_sec) {
#ifndef CONFIG_HRM
  return false;
#else
  if (!sys_activity_is_initialized()) {
    return false;
  }
  if (!sys_activity_prefs_heart_rate_is_enabled()) {
    return false;
  }

  // Get the app id
  AppInstallId  app_id = app_get_app_id();
  if (app_id == INSTALL_ID_INVALID) {
    return false;
  }

  // If interval is 0, the caller wants to unsubscribe
  if (interval_sec == 0) {
    HRMSessionRef hrm_session = sys_hrm_manager_get_app_subscription(app_id);
    if (hrm_session != HRM_INVALID_SESSION_REF) {
      sys_hrm_manager_unsubscribe(hrm_session);
    }
    return true;
  }

  // Subscribe now
  HRMSessionRef hrm_session = sys_hrm_manager_app_subscribe(app_id, interval_sec, 0 /*expire_sec*/,
                                                            HRMFeature_BPM);
  if (hrm_session == HRM_INVALID_SESSION_REF) {
    PBL_LOG_ERR("Error subscribing");
    return false;
  }

  return true;
#endif
}

// ----------------------------------------------------------------------------------------------
uint16_t health_service_get_heart_rate_sample_period_expiration_sec(void) {
#ifndef CONFIG_HRM
  return 0;
#else
  if (!sys_activity_prefs_heart_rate_is_enabled()) {
    return 0;
  }

  // Get the app id
  AppInstallId  app_id = app_get_app_id();
  if (app_id == INSTALL_ID_INVALID) {
    return 0;
  }

  // If not subscribed, return 0
  HRMSessionRef hrm_session = sys_hrm_manager_get_app_subscription(app_id);
  if (hrm_session == HRM_INVALID_SESSION_REF) {
    return 0;
  } else {
    return HRM_MANAGER_APP_EXIT_EXPIRATION_SEC;
  }
#endif
}

// ----------------------------------------------------------------------------------------------
uint32_t health_service_get_minute_history(HealthMinuteData *minute_data, uint32_t max_records,
                                           time_t *time_start, time_t *time_end) {
  if (!sys_activity_is_initialized()) {
    return 0;
  }
  if (!minute_data || max_records == 0 || !time_start) {
    return 0;
  }
  if (time_end && *time_end < *time_start) {
    return 0;
  }

  uint32_t num_records = max_records;

  // only query for as many records as necessary for the given time span
  if (time_end) {
    const time_t lower_bounded_start = (*time_start / SECONDS_PER_MINUTE) * SECONDS_PER_MINUTE;
    const time_t upper_bounded_end = *time_end + SECONDS_PER_MINUTE - 1;
    const uint32_t needed_partial_minutes =
      (upper_bounded_end - lower_bounded_start) / SECONDS_PER_MINUTE;
    num_records = MIN(num_records, needed_partial_minutes);
  }

  const bool success = sys_activity_get_minute_history(minute_data, &num_records, time_start);
  if (!success) {
    return 0;
  }

  if (time_end) {
    *time_end = *time_start + SECONDS_PER_MINUTE * num_records;
  }
  return num_records;
}


// ----------------------------------------------------------------------------------------------
HealthActivityMask health_service_peek_current_activities(void) {
  if (!sys_activity_is_initialized()) {
    return HealthActivityNone;
  }
  HealthValue sleep_state;
  if (!sys_activity_get_metric(ActivityMetricSleepState, 1, &sleep_state)) {
    return HealthActivityNone;
  }

  HealthActivityMask result = HealthActivityNone;
  if (sleep_state == ActivitySleepStateLightSleep) {
    result |= HealthActivitySleep;
  }
  // yes, when sleeping restful, there's also always an activity of HealthActivitySleep
  // when calling health_service_activities_iterate()
  if (sleep_state == ActivitySleepStateRestfulSleep) {
    result |= (HealthActivitySleep | HealthActivityRestfulSleep);
  }

  if (sys_activity_sessions_is_session_type_ongoing(ActivitySessionType_Walk)) {
    result |= HealthActivityWalk;
  }

  if (sys_activity_sessions_is_session_type_ongoing(ActivitySessionType_Run)) {
    result |= HealthActivityRun;
  }

  if (sys_activity_sessions_is_session_type_ongoing(ActivitySessionType_Open)) {
    result |= HealthActivityOpenWorkout;
  }

  return result;
}

// ----------------------------------------------------------------------------------------------
// The number of session we choose to store is arbitrary and taken from other examples
// today, we should store < 10 session, so the value is a trade-off between stack space and risk
// to miss sessions.
#define NUM_EVALUATED_SLEEP_SESSIONS 16

void health_service_activities_iterate(HealthActivityMask activity_mask,
                                       time_t time_start, time_t time_end,
                                       HealthIterationDirection direction,
                                       HealthActivityIteratorCB callback, void *context) {
  if (!sys_activity_is_initialized()) {
    return;
  }
  HealthServiceState *state = prv_get_state(true);
  if (!state->cache) {
    return;
  }

  if (callback == NULL || activity_mask == HealthActivityNone) {
    return;
  }

  uint32_t num_sessions = ARRAY_LENGTH(state->cache->sessions);
  if (!sys_activity_get_sessions(&num_sessions, state->cache->sessions)) {
    return;
  }

  const uint32_t actual_num_sessions = MIN(num_sessions, ARRAY_LENGTH(state->cache->sessions));
  prv_sessions_sort(state->cache->sessions, actual_num_sessions, direction);

  for (uint32_t idx = 0; idx < actual_num_sessions; idx++) {
    const ActivitySession *const session = &state->cache->sessions[idx];
    if (prv_activity_session_matches(session, activity_mask,
                                     time_start, time_end)) {
      HealthActivity session_activity = HealthActivityNone;
      switch (session->type) {
        case ActivitySessionType_Sleep:
        case ActivitySessionType_Nap:
          session_activity = HealthActivitySleep;
          break;
        case ActivitySessionType_RestfulSleep:
        case ActivitySessionType_RestfulNap:
          session_activity = HealthActivityRestfulSleep;
          break;
        case ActivitySessionType_Walk:
          session_activity = HealthActivityWalk;
          break;
        case ActivitySessionType_Run:
          session_activity = HealthActivityRun;
          break;
        case ActivitySessionType_Open:
          session_activity = HealthActivityOpenWorkout;
          break;
        case ActivitySessionType_None:
        case ActivitySessionTypeCount:
          WTF;
          break;
      }
      if (!callback(session_activity, session->start_utc,
                    session->start_utc + (session->length_min * SECONDS_PER_MINUTE),
                    context)) {
        // clients can interrupt the iteration at any time
        break;
      }
    }
  }
}

// ----------------------------------------------------------------------------------------------
bool health_service_private_get_yesterdays_sleep_activity(HealthValue *enter_sec,
                                                          HealthValue *exit_sec) {
  return
    sys_activity_get_metric(ActivityMetricSleepEnterAtSeconds, 1, enter_sec) &
      sys_activity_get_metric(ActivityMetricSleepExitAtSeconds, 1, exit_sec);
}



// ----------------------------------------------------------------------------------------------
HealthServiceAccessibilityMask health_service_any_activity_accessible(
  HealthActivityMask activity_mask,
  time_t start_time, time_t end_time) {
  // TODO: PBL-31628 permission system to reply with HealthServiceAccessibilityMaskNoPermission

  if (activity_mask == HealthActivityNone) {
    return HealthServiceAccessibilityMaskNotAvailable;
  }

  // TODO: PBL-31630 provide more accurate value for available time frame
  // for now, we say that there's only 1 day worth of data for sleep sessions
  HealthServiceTimeRange range;
  if (!prv_calculate_time_range(start_time, end_time, &range)) {
    return HealthServiceAccessibilityMaskNotAvailable;
  }

  if (range.last_day_idx > 2) {
    return HealthServiceAccessibilityMaskNotAvailable;
  }

  return HealthServiceAccessibilityMaskAvailable;
}

// ----------------------------------------------------------------------------------------------
MeasurementSystem health_service_get_measurement_system_for_display(HealthMetric metric) {
  switch (metric) {
    case HealthMetricWalkedDistanceMeters:
      return prv_get_shell_prefs_metric_for_distance();
    default:
      return MeasurementSystemUnknown;
  }
}

// ----------------------------------------------------------------------------------------------
void health_service_state_init(HealthServiceState *state) {
  *state = (HealthServiceState) {
    .health_event_service_info = {
      .type = PEBBLE_HEALTH_SERVICE_EVENT,
      .handler = &prv_health_event_handler,
    },
  };
}

// ----------------------------------------------------------------------------------------------
void health_service_state_deinit(HealthServiceState *state) {
  prv_health_service_deinit_cache(state);
}
