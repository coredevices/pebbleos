/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "data.h"
#include "data_private.h"

#include "applib/health_service_private.h"
#include <pbl/drivers/rtc.h>
#include "kernel/pbl_malloc.h"
#include "syscall/syscall.h"
#include <pbl/logging/logging.h>
#include "util/time/time.h"


T_STATIC void prv_merge_adjacent_sessions(ActivitySession *current,
                                          ActivitySession *previous) {
  if (previous == NULL || current == NULL) {
    return;
  }

  if (current->type != previous->type ||
      (current->type != ActivitySessionType_RestfulNap &&
       current->type != ActivitySessionType_RestfulSleep)) {
    // We only merge sessions if they are "deep" sleep/nap
    return;
  }

  // [FBO]: note that this only works because sleep sessions are
  // all we care about and they are sorted. Don't try to extend this to walk
  // or run sessions

  const uint16_t max_apart_merge_secs = 5 * SECONDS_PER_MINUTE;
  time_t end_time = previous->start_utc + previous->length_min * SECONDS_PER_MINUTE;
  if ((end_time + max_apart_merge_secs) > current->start_utc) {
    current->length_min += previous->length_min +
                           (current->start_utc - end_time) / SECONDS_PER_MINUTE;
    current->start_utc = previous->start_utc;
    previous->length_min = 0;
    previous->type = ActivitySessionType_None;
  }
}

// API Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

HealthData *health_data_create(void) {
  return (HealthData *)app_zalloc_check(sizeof(HealthData));
}

void health_data_destroy(HealthData *health_data) {
  app_free(health_data);
}

void health_data_update_quick(HealthData *health_data) {
  const time_t now = rtc_get_time();
  struct tm local_tm;
  localtime_r(&now, &local_tm);

  // Get the current steps
  health_service_private_get_metric_history(HealthMetricStepCount, 1, health_data->step_data);

  // Get the typical step averages for every 15 minutes
  activity_get_step_averages(local_tm.tm_wday, &health_data->step_averages);

  health_data->current_hr_bpm = health_service_peek_current_value(HealthMetricHeartRateBPM);
  // Get the most recent stable HR Reading timestamp.
  activity_get_metric(ActivityMetricHeartRateFilteredUpdatedTimeUTC, 1,
                      (int32_t *)&health_data->hr_last_updated);
}

void health_data_update(HealthData *health_data) {
  const time_t now = rtc_get_time();
  struct tm local_tm;
  localtime_r(&now, &local_tm);


  //! Step / activity related data
  // Get the step totals for today and the past 6 days
  health_service_private_get_metric_history(HealthMetricStepCount, DAYS_PER_WEEK,
                                            health_data->step_data);
  // Update distance / calories now that we have our steps
  health_data_update_step_derived_metrics(health_data);

  // Get the step averages for each 15 minute window. Used for typical steps
  activity_get_step_averages(local_tm.tm_wday, &health_data->step_averages);

  // Get the average steps for the past month
  activity_get_metric_monthly_avg(ActivityMetricStepCount, &health_data->monthly_step_average);


  //! Sleep related data
  health_service_private_get_metric_history(HealthMetricSleepSeconds, DAYS_PER_WEEK,
                                            health_data->sleep_data);
  // Check if we have sleep data for today. If not, we want to show the last sleep session
  // (yesterday's data)
  bool use_yesterday = (health_data->sleep_data[0] == 0 && health_data->sleep_data[1] > 0);
  int day_offset = use_yesterday ? 1 : 0;
  int wday = (local_tm.tm_wday - day_offset + 7) % 7;

  activity_get_metric_typical(ActivityMetricSleepTotalSeconds, wday,
                              &health_data->typical_sleep);

  int32_t deep_sleep_history[2];
  activity_get_metric(ActivityMetricSleepRestfulSeconds, 2, deep_sleep_history);
  health_data->deep_sleep = deep_sleep_history[day_offset];

  int32_t sleep_start_history[2];
  activity_get_metric(ActivityMetricSleepEnterAtSeconds, 2, sleep_start_history);
  health_data->sleep_start = sleep_start_history[day_offset];

  int32_t sleep_end_history[2];
  activity_get_metric(ActivityMetricSleepExitAtSeconds, 2, sleep_end_history);
  health_data->sleep_end = sleep_end_history[day_offset];

  activity_get_metric_typical(ActivityMetricSleepEnterAtSeconds, wday,
                              &health_data->typical_sleep_start);
  activity_get_metric_typical(ActivityMetricSleepExitAtSeconds, wday,
                              &health_data->typical_sleep_end);
  activity_get_metric_monthly_avg(ActivityMetricSleepTotalSeconds,
                                  &health_data->monthly_sleep_average);


  //! Activity sessions
  health_data->num_activity_sessions = ACTIVITY_MAX_ACTIVITY_SESSIONS_COUNT;
  if (!activity_get_sessions(&health_data->num_activity_sessions,
                             health_data->activity_sessions)) {
    PBL_LOG_ERR("Fetching activity sessions failed");
  } else {
    ActivitySession *previous_session = NULL;
    for (unsigned int i = 0; i < health_data->num_activity_sessions; i++) {
      ActivitySession *session = &health_data->activity_sessions[i];
      prv_merge_adjacent_sessions(session, previous_session);
      previous_session = session;
    }
  }

  //! HR related data
  health_data_update_current_bpm(health_data);
  health_data_update_hr_zone_minutes(health_data);
}

void health_data_update_step_derived_metrics(HealthData *health_data) {
  // get distance in meters
  health_data->current_distance_meters = health_service_sum_today(HealthMetricWalkedDistanceMeters);

  // get calories
  health_data->current_calories = health_service_sum_today(HealthMetricActiveKCalories)
                                + health_service_sum_today(HealthMetricRestingKCalories);
}

void health_data_update_steps(HealthData *health_data, uint32_t new_steps) {
  health_data->step_data[0] = new_steps;
  health_data_update_step_derived_metrics(health_data);
}

void health_data_update_sleep(HealthData *health_data, uint32_t new_sleep,
                              uint32_t new_deep_sleep) {
  health_data->sleep_data[0] = new_sleep;
  health_data->deep_sleep = new_deep_sleep;
}

void health_data_update_current_bpm(HealthData *health_data) {
  health_data->resting_hr_bpm = activity_prefs_heart_get_resting_hr();

  // Check the quality. If it doesn't meet our standards, bail
  int32_t quality;
  activity_get_metric(ActivityMetricHeartRateRawQuality, 1, &quality);
  if (quality < HRMQuality_Acceptable) {
    return;
  }

  uint32_t current_hr_timestamp;
  activity_get_metric(ActivityMetricHeartRateRawUpdatedTimeUTC, 1,
                      (int32_t *)&current_hr_timestamp);
  if (current_hr_timestamp > (uint32_t)health_data->hr_last_updated) {
    health_data->current_hr_bpm = health_service_peek_current_value(HealthMetricHeartRateRawBPM);
    health_data->hr_last_updated = current_hr_timestamp;
  }
}

void health_data_update_hr_zone_minutes(HealthData *health_data) {
  activity_get_metric(ActivityMetricHeartRateZone1Minutes, 1, &health_data->hr_zone1_minutes);
  activity_get_metric(ActivityMetricHeartRateZone2Minutes, 1, &health_data->hr_zone2_minutes);
  activity_get_metric(ActivityMetricHeartRateZone3Minutes, 1, &health_data->hr_zone3_minutes);
}

int32_t *health_data_steps_get(HealthData *health_data) {
  return health_data->step_data;
}

int32_t health_data_current_steps_get(HealthData *health_data) {
  return health_data->step_data[0];
}

int32_t health_data_current_distance_meters_get(HealthData *health_data) {
  return health_data->current_distance_meters;
}

int32_t health_data_current_calories_get(HealthData *health_data) {
  return health_data->current_calories;
}

static int32_t prv_health_data_get_n_average_chunks(HealthData *health_data, int number_of_chunks) {
  uint32_t total_steps_avg = 0;
  for (int i = 0; (i < ACTIVITY_NUM_METRIC_AVERAGES) && (i < number_of_chunks); i++) {
    if (health_data->step_averages.average[i] != ACTIVITY_METRIC_AVERAGES_UNKNOWN) {
      total_steps_avg += health_data->step_averages.average[i];
    }
  }
  return total_steps_avg;
}

int32_t health_data_steps_get_current_average(HealthData *health_data) {
  // get the current minutes into today
  time_t utc_sec = rtc_get_time();
  struct tm local_tm;
  localtime_r(&utc_sec, &local_tm);
  int32_t today_min = local_tm.tm_hour * MINUTES_PER_HOUR + local_tm.tm_min;
  const int k_minutes_per_step_avg = MINUTES_PER_DAY / ACTIVITY_NUM_METRIC_AVERAGES;

  // each average chunk is 15 mins long
  if (health_data->step_average_last_updated_time !=
         ((today_min / k_minutes_per_step_avg) * k_minutes_per_step_avg)) {
    // current_step_average is stale
    health_data->current_step_average =
      prv_health_data_get_n_average_chunks(health_data, today_min / k_minutes_per_step_avg);
    health_data->step_average_last_updated_time =
        (today_min / k_minutes_per_step_avg) * k_minutes_per_step_avg;
  }
  return health_data->current_step_average;
}

int32_t health_data_steps_get_current_average_minute(HealthData *health_data) {
  return health_data->step_average_last_updated_time;
}

int32_t health_data_steps_get_cur_wday_average(HealthData *health_data) {
  return prv_health_data_get_n_average_chunks(health_data, ACTIVITY_NUM_METRIC_AVERAGES);
}

int32_t health_data_steps_get_monthly_average(HealthData *health_data) {
  return health_data->monthly_step_average;
}


int32_t *health_data_sleep_get(HealthData *health_data) {
  return health_data->sleep_data;
}

int32_t health_data_current_sleep_get(HealthData *health_data) {
  if (health_data->sleep_data[0] == 0 && health_data->sleep_data[1] > 0) {
    return health_data->sleep_data[1];
  }
  return health_data->sleep_data[0];
}

int32_t health_data_sleep_get_cur_wday_average(HealthData *health_data) {
  return health_data->typical_sleep;
}

int32_t health_data_current_deep_sleep_get(HealthData *health_data) {
  return health_data->deep_sleep;
}

int32_t health_data_sleep_get_monthly_average(HealthData *health_data) {
  return health_data->monthly_sleep_average;
}

int32_t health_data_sleep_get_start_time(HealthData *health_data) {
  return health_data->sleep_start;
}

int32_t health_data_sleep_get_end_time(HealthData *health_data) {
  return health_data->sleep_end;
}

int32_t health_data_sleep_get_typical_start_time(HealthData *health_data) {
  return health_data->typical_sleep_start;
}

int32_t health_data_sleep_get_typical_end_time(HealthData *health_data) {
  return health_data->typical_sleep_end;
}

int32_t health_data_sleep_get_num_sessions(HealthData *health_data) {
  return health_data->num_activity_sessions;
}

ActivitySession *health_data_sleep_get_sessions(HealthData *health_data) {
  return health_data->activity_sessions;
}

uint32_t health_data_hr_get_current_bpm(HealthData *health_data) {
  return health_data->current_hr_bpm;
}

uint32_t health_data_hr_get_resting_bpm(HealthData *health_data) {
  return health_data->resting_hr_bpm;
}

time_t health_data_hr_get_last_updated_timestamp(HealthData *health_data) {
  return health_data->hr_last_updated;
}

int32_t health_data_hr_get_zone1_minutes(HealthData *health_data) {
  return health_data->hr_zone1_minutes;
}

int32_t health_data_hr_get_zone2_minutes(HealthData *health_data) {
  return health_data->hr_zone2_minutes;
}

int32_t health_data_hr_get_zone3_minutes(HealthData *health_data) {
  return health_data->hr_zone3_minutes;
}
