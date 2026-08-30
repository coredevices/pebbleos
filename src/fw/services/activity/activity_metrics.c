/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/data_logging.h"
#include "applib/health_service.h"
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "pbl/os/mutex.h"
#include "pbl/os/tick.h"
#include "pbl/services/protobuf_log/protobuf_log.h"
#include "syscall/syscall.h"
#include "syscall/syscall_internal.h"
#include "system/hexdump.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "util/base64.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"
#include "util/stats.h"
#include "util/units.h"

#include "pbl/services/activity/activity.h"
#include "pbl/services/activity/activity_algorithm.h"
#include "pbl/services/activity/activity_calculators.h"
#include "pbl/services/activity/activity_insights.h"
#include "pbl/services/activity/activity_private.h"

PBL_LOG_MODULE_DECLARE(service_activity, CONFIG_SERVICE_ACTIVITY_LOG_LEVEL);


// ---------------------------------------------------------------------------------------
// Storage converters. These convert metrics from their storage type (ActivityScalarStore,
// which is only 16-bits) into the uint32_t value returned by activity_get_metric. For example,
// we might convert minutes to seconds.
static uint32_t prv_convert_none(ActivityScalarStore in) {
  return in;
}

static uint32_t prv_convert_minutes_to_seconds(ActivityScalarStore in) {
  return (uint32_t)in * SECONDS_PER_MINUTE;
}


// ------------------------------------------------------------------------------------------------
// Returns info about each metric we capture
void activity_metrics_prv_get_metric_info(ActivityMetric metric, ActivityMetricInfo *info) {
  ActivityState *state = activity_private_state();
  *info = (ActivityMetricInfo) {
    .converter = prv_convert_none,
  };
  switch (metric) {
    case ActivityMetricStepCount:
      info->value_p = &state->step_data.steps;
      info->settings_key = ActivitySettingsKeyStepCountHistory;
      info->has_history = true;
      break;
    case ActivityMetricActiveSeconds:
      info->value_p = &state->step_data.step_minutes;
      info->settings_key = ActivitySettingsKeyStepMinutesHistory;
      info->has_history = true;
      info->converter = prv_convert_minutes_to_seconds;
      break;
    case ActivityMetricDistanceMeters:
      info->value_p = &state->step_data.distance_meters;
      info->settings_key = ActivitySettingsKeyDistanceMetersHistory;
      info->has_history = true;
      break;
    case ActivityMetricRestingKCalories:
      info->value_p = &state->step_data.resting_kcalories;
      info->settings_key = ActivitySettingsKeyRestingKCaloriesHistory;
      info->has_history = true;
      break;
    case ActivityMetricActiveKCalories:
      info->value_p = &state->step_data.active_kcalories;
      info->settings_key = ActivitySettingsKeyActiveKCaloriesHistory;
      info->has_history = true;
      break;
    case ActivityMetricSleepTotalSeconds:
      info->value_p = &state->sleep_data.total_minutes;
      info->settings_key = ActivitySettingsKeySleepTotalMinutesHistory;
      info->has_history = true;
      info->converter = prv_convert_minutes_to_seconds;
      break;
    case ActivityMetricSleepRestfulSeconds:
      info->value_p = &state->sleep_data.restful_minutes;
      info->settings_key = ActivitySettingsKeySleepDeepMinutesHistory;
      info->has_history = true;
      info->converter = prv_convert_minutes_to_seconds;
      break;
    case ActivityMetricSleepEnterAtSeconds:
      info->value_p = &state->sleep_data.enter_at_minute;
      info->settings_key = ActivitySettingsKeySleepEnterAtHistory;
      info->has_history = true;
      info->converter = prv_convert_minutes_to_seconds;
      break;
    case ActivityMetricSleepExitAtSeconds:
      info->value_p = &state->sleep_data.exit_at_minute;
      info->settings_key = ActivitySettingsKeySleepExitAtHistory;
      info->has_history = true;
      info->converter = prv_convert_minutes_to_seconds;
      break;
    case ActivityMetricSleepState:
      info->value_p = &state->sleep_data.cur_state;
      info->settings_key = ActivitySettingsKeySleepState;
      break;
    case ActivityMetricSleepStateSeconds:
      info->value_p = &state->sleep_data.cur_state_elapsed_minutes;
      info->settings_key = ActivitySettingsKeySleepStateMinutes;
      info->converter = prv_convert_minutes_to_seconds;
      break;
    case ActivityMetricLastVMC:
      info->value_p = &state->last_vmc;
      info->settings_key = ActivitySettingsKeyLastVMC;
      break;
    case ActivityMetricHeartRateRawBPM:
      info->value_p = &state->hr.metrics.current_bpm;
      break;
    case ActivityMetricHeartRateRawQuality:
      info->value_p = &state->hr.metrics.current_quality;
      break;
    case ActivityMetricHeartRateRawUpdatedTimeUTC:
      info->value_u32p = &state->hr.metrics.current_update_time_utc;
      break;
    case ActivityMetricHeartRateFilteredBPM:
      info->value_p = &state->hr.metrics.last_stable_bpm;
      break;
    case ActivityMetricHeartRateFilteredUpdatedTimeUTC:
      info->value_u32p = &state->hr.metrics.last_stable_bpm_update_time_utc;
      break;
    case ActivityMetricHeartRateZone1Minutes:
      info->value_p = &state->hr.metrics.minutes_in_zone[HRZone_Zone1];
      info->settings_key = ActivitySettingsKeyHeartRateZone1Minutes;
      info->has_history = false;
      break;
    case ActivityMetricHeartRateZone2Minutes:
      info->value_p = &state->hr.metrics.minutes_in_zone[HRZone_Zone2];
      info->settings_key = ActivitySettingsKeyHeartRateZone2Minutes;
      info->has_history = false;
      break;
    case ActivityMetricHeartRateZone3Minutes:
      info->value_p = &state->hr.metrics.minutes_in_zone[HRZone_Zone3];
      info->settings_key = ActivitySettingsKeyHeartRateZone3Minutes;
      info->has_history = false;
      break;
    case ActivityMetricNumMetrics:
      WTF;
      break;
  }
}


// ----------------------------------------------------------------------------------------------
// Set the value of a given metric.
// For the current day the cached value is only overridden when `force` is true or the new value is
// higher than the current one. Historical values can be overridden with any value.
static void prv_set_metric(ActivityMetric metric, DayInWeek wday, int32_t value, bool force) {
  if (!activity_tracking_on()) {
    return;
  }

  ActivityState *state = activity_private_state();
  mutex_lock_recursive(state->mutex);

  switch (metric) {
    case ActivityMetricActiveSeconds:
    case ActivityMetricSleepTotalSeconds:
    case ActivityMetricSleepRestfulSeconds:
    case ActivityMetricSleepEnterAtSeconds:
    case ActivityMetricSleepExitAtSeconds:
      // We only store minutes for these metrics. Convert before saving
      value /= SECONDS_PER_MINUTE;
      break;
    default:
      break;
  }

  ActivityMetricInfo m_info = {};
  activity_metrics_prv_get_metric_info(metric, &m_info);
  const DayInWeek cur_wday = time_util_get_day_in_week(rtc_get_time());

  bool current_value_updated = false;

  if (cur_wday == wday) {
    // Update our cached copy of the value when forced, or if it is larger than what we have
    if (m_info.value_p && (force || value > (int32_t)*m_info.value_p)) {
      *m_info.value_p = value;
      current_value_updated = true;
    } else if (m_info.value_u32p && (force || (uint32_t)value > *m_info.value_u32p)) {
      *m_info.value_u32p = value;
      current_value_updated = true;
    }
  } else if (m_info.has_history) {
    // This update is for a day in the past. Modify the copy stored in the settings file
    SettingsFile *file = activity_private_settings_open();
    if (!file) {
      goto unlock;
    }
    ActivitySettingsValueHistory history;
    settings_file_get(file, &m_info.settings_key, sizeof(m_info.settings_key),
                      &history, sizeof(history));

    int day = positive_modulo(cur_wday - wday, DAYS_PER_WEEK);
    if ((int32_t)history.values[day] != value) {
      history.values[day] = value;

      settings_file_set(file, &m_info.settings_key, sizeof(m_info.settings_key),
                        &history, sizeof(history));
    }
    activity_private_settings_close(file);
  }

  if (current_value_updated) {
    if (metric == ActivityMetricStepCount) {
      PebbleEvent e = {
        .type = PEBBLE_HEALTH_SERVICE_EVENT,
        .health_event = {
          .type = HealthEventMovementUpdate,
          .data.movement_update = {
            .steps = value,
          },
        },
      };
      event_put(&e);
    } else if (metric == ActivityMetricDistanceMeters) {
      state->distance_mm = state->step_data.distance_meters * MM_PER_METER;
    } else if (metric == ActivityMetricActiveKCalories) {
      state->active_calories = state->step_data.active_kcalories * ACTIVITY_CALORIES_PER_KCAL;
    } else if (metric == ActivityMetricRestingKCalories) {
      state->resting_calories = state->step_data.resting_kcalories * ACTIVITY_CALORIES_PER_KCAL;
    }
    activity_algorithm_metrics_changed_notification();
  }

unlock:
  mutex_unlock_recursive(state->mutex);
}


// ----------------------------------------------------------------------------------------------
// Set the value of a given metric. The current day's value is only overridden if the new value is
// higher; historical values can be overridden with any value.
void activity_metrics_prv_set_metric(ActivityMetric metric, DayInWeek wday, int32_t value) {
  prv_set_metric(metric, wday, value, false /* force */);
}


// ----------------------------------------------------------------------------------------------
// Force the current day's value of a metric to an exact value (may also decrease it). Intended for
// QEMU/test injection of health data.
void activity_metrics_set_metric_exact(ActivityMetric metric, int32_t value) {
  prv_set_metric(metric, time_util_get_day_in_week(rtc_get_time()), value, true /* force */);
}


// ----------------------------------------------------------------------------------------------
// Shift the history back one day and reset the current day's stats.
// We use NOINLINE to reduce the stack requirements during the minute handler (see PBL-38130)
static void NOINLINE prv_shift_history(time_t utc_now) {
  ActivityState *state = activity_private_state();
  PBL_LOG_INFO("resetting metrics for new day");
  mutex_lock_recursive(state->mutex);
  {
    SettingsFile *file = activity_private_settings_open();
    if (!file) {
      goto unlock;
    }
    ActivitySettingsValueHistory history;
    ActivityMetricInfo m_info;

    for (ActivityMetric metric = ActivityMetricFirst; metric < ActivityMetricNumMetrics;
         metric++) {
      activity_metrics_prv_get_metric_info(metric, &m_info);

      // Shift the history
      if (m_info.has_history) {
        PBL_ASSERTN(m_info.value_p);
        settings_file_get(file, &m_info.settings_key, sizeof(m_info.settings_key), &history,
                          sizeof(history));

        for (int i = ACTIVITY_HISTORY_DAYS - 1; i >= 1; i--) {
          history.values[i] = history.values[i - 1];
        }
        // We just wrapped up yesterday
        history.values[1] = *m_info.value_p;

        // Reset stats for today
        history.values[0] = 0;
        history.utc_sec = utc_now;

        settings_file_set(file, &m_info.settings_key, sizeof(m_info.settings_key), &history,
                          sizeof(history));
      }
    }
    activity_private_settings_close(file);
  }
unlock:
  mutex_unlock_recursive(state->mutex);
}


// --------------------------------------------------------------------------------------------
// Called from activity_get_metric() every time a client asks for a metric. Also called
// periodically from the minute handler before we save current metrics to setting.
static void prv_update_real_time_derived_metrics(void) {
  ActivityState *state = activity_private_state();
  mutex_lock_recursive(state->mutex);
  {
    state->step_data.distance_meters = ROUND(state->distance_mm,
                                                       MM_PER_METER);
    ACTIVITY_LOG_DEBUG("new distance: %"PRIu32"", state->step_data.distance_meters);

    state->step_data.active_kcalories = ROUND(state->active_calories,
                                                        ACTIVITY_CALORIES_PER_KCAL);
    ACTIVITY_LOG_DEBUG("new active kcal: %"PRIu32"", state->step_data.active_kcalories);
  }
  mutex_unlock_recursive(state->mutex);
}


// --------------------------------------------------------------------------------------------
// Called periodically from the minute handler to update step derived metrics that do not have to
// be updated in real time.
// We use NOINLINE to reduce the stack requirements during the minute handler (see PBL-38130)
static void NOINLINE prv_update_step_derived_metrics(time_t utc_sec) {
  ActivityState *state = activity_private_state();
  mutex_lock_recursive(state->mutex);
  {
    int minute_of_day = time_util_get_minute_of_day(utc_sec);
    // The "no-steps-during-sleep" logic can introduce negative steps, so make sure we clip
    // negative steps to 0 when computing the metrics below
    uint16_t steps_in_minute = 0;
    if (state->step_data.steps >= state->steps_per_minute_last_steps) {
      steps_in_minute = state->step_data.steps
                        - state->steps_per_minute_last_steps;
    }

    // Update the walking rate
    state->steps_per_minute = steps_in_minute;
    state->steps_per_minute_last_steps = state->step_data.steps;
    ACTIVITY_LOG_DEBUG("new steps/minute: %"PRIu32"", state->steps_per_minute);

    // Update the number of stepping minutes and the last active minute
    if (state->steps_per_minute >= ACTIVITY_ACTIVE_MINUTE_MIN_STEPS) {
      state->step_data.step_minutes++;
      ACTIVITY_LOG_DEBUG("new step minutes: %"PRIu32"", state->step_data.step_minutes);

      // The prior minute was the most recent active one
      state->last_active_minute = time_util_minute_of_day_adjust(minute_of_day, -1);
      ACTIVITY_LOG_DEBUG("last active minute: %"PRIu16"", state->last_active_minute);
    }

    // Update the resting calories
    state->resting_calories = activity_private_compute_resting_calories(minute_of_day);
    state->step_data.resting_kcalories = ROUND(state->resting_calories,
                                                         ACTIVITY_CALORIES_PER_KCAL);
    ACTIVITY_LOG_DEBUG("resting kcalories: %"PRIu32"",
                       state->step_data.resting_kcalories);
  }
  mutex_unlock_recursive(state->mutex);
}


// ------------------------------------------------------------------------------------------
// Pushes an HR Median/Filtered/LastStable event.
static void prv_push_median_hr_event(uint8_t median_hr) {
  if (median_hr > 0) {
    PebbleEvent event = {
      .type = PEBBLE_HEALTH_SERVICE_EVENT,
      .health_event = {
        .type = HealthEventHeartRateUpdate,
        .data.heart_rate_update = {
          .current_bpm = median_hr,
          .is_filtered = true,
        }
      }
    };
    event_put(&event);
  }
}


// ------------------------------------------------------------------------------------------
// Calculates and stores the most recent minutes median heart rate value.
// Used for the health_service and the minute level data.
static void prv_update_median_hr_bpm(ActivityState *state) {
  const ActivityHRSupport *hr = &state->hr;

  const uint16_t num_hr_samples = hr->num_samples;
  if (num_hr_samples > 0) {
    int32_t median, total_weight;

    // Stats requires an int32_t array and we need one for both the samples and the weights
    int32_t *sample_buf = task_zalloc_check(num_hr_samples * sizeof(int32_t));
    int32_t *weight_buf = task_zalloc_check(num_hr_samples * sizeof(int32_t));
    for (size_t i = 0; i < num_hr_samples; i++) {
      sample_buf[i] = hr->samples[i];
      weight_buf[i] = hr->weights[i];
    }

    // Calculate the total weight
    stats_calculate_basic(StatsBasicOp_Sum, weight_buf, hr->num_samples, NULL, NULL,
                          &total_weight);

    // Calculate the weighted median
    median = stats_calculate_weighted_median(sample_buf, weight_buf, num_hr_samples);
    task_free(sample_buf);
    task_free(weight_buf);

    state->hr.metrics.last_stable_bpm = (uint8_t)median;
    state->hr.metrics.last_stable_bpm_update_time_utc = rtc_get_time();
    state->hr.metrics.previous_median_bpm = (uint8_t)median;
    state->hr.metrics.previous_median_total_weight_x100 = total_weight;

    prv_push_median_hr_event(state->hr.metrics.previous_median_bpm);
  }
}

// ------------------------------------------------------------------------------------------
static void prv_write_hr_zone_info_to_flash(HRZone zone) {
  ActivityMetric metric;
  if (zone == HRZone_Zone1) {
    metric = ActivityMetricHeartRateZone1Minutes;
  } else if (zone == HRZone_Zone2) {
    metric = ActivityMetricHeartRateZone2Minutes;
  } else if (zone == HRZone_Zone3) {
    metric = ActivityMetricHeartRateZone3Minutes;
  } else {
    // Don't store data for Zone 0
    return;
  }

  SettingsFile *file = activity_private_settings_open();
  if (!file) {
    return;
  }

  ActivityMetricInfo m_info;
  activity_metrics_prv_get_metric_info(metric, &m_info);
  settings_file_set(file, &m_info.settings_key, sizeof(m_info.settings_key),
                    m_info.value_p, sizeof(*m_info.value_p));
  activity_private_settings_close(file);
}

// ------------------------------------------------------------------------------------------
// The median HR should get updated before calling this
static void prv_update_current_hr_zone(ActivityState *state) {
  int32_t hr_median;
  activity_metrics_prv_get_median_hr_bpm(&hr_median, NULL);
  HRZone new_hr_zone = hr_util_get_hr_zone(hr_median);

  if (new_hr_zone != HRZone_Zone0 && state->hr.num_samples < ACTIVITY_MIN_NUM_SAMPLES_FOR_HR_ZONE) {
    // There wasn't enough data in the past minute to give us confidence that
    // the new HR zone will represents that minute, default to Zone0
    new_hr_zone = HRZone_Zone0;
  }

  bool new_hr_elevated = hr_util_is_elevated(hr_median);
  // Before changing the zone make sure the user has an elevated heart rate.
  // This prevents erroneous HRM readings accumulating minutes in zone 1.
  // Then only go up/down 1 zone per minute.
  // This prevents erroneous HRM readings accumulating minutes in higher zones.
  if (!state->hr.metrics.is_hr_elevated && new_hr_elevated) {
    state->hr.metrics.is_hr_elevated = new_hr_elevated;
  } else if (new_hr_zone > state->hr.metrics.current_hr_zone) {
    state->hr.metrics.current_hr_zone++;
  } else if (new_hr_zone < state->hr.metrics.current_hr_zone) {
    state->hr.metrics.current_hr_zone--;
  } else if (!new_hr_elevated) {
    state->hr.metrics.is_hr_elevated = new_hr_elevated;
  }

  state->hr.metrics.minutes_in_zone[state->hr.metrics.current_hr_zone]++;

  prv_write_hr_zone_info_to_flash(state->hr.metrics.current_hr_zone);
}

// ------------------------------------------------------------------------------------------
// Called periodically from the minute handler to update the median HR and time spent in HR zones
static void prv_update_hr_derived_metrics(void) {
  ActivityState *state = activity_private_state();
  mutex_lock_recursive(state->mutex);
  {
    // Update the median HR / HR weight for the minute
    prv_update_median_hr_bpm(state);

    // Update our current HR zone (based on the median which is calculated above)
    prv_update_current_hr_zone(state);
  }
  mutex_unlock_recursive(state->mutex);
}

// ------------------------------------------------------------------------------------------
// The metrics minute handler
void activity_metrics_prv_minute_handler(time_t utc_sec) {
  ActivityState *state = activity_private_state();

  uint16_t cur_day_index = time_util_get_day(utc_sec);
  if (cur_day_index != state->cur_day_index) {
    // If we've just encountered a midnight rollover, shift history to the new day
    // before we compute metrics for the new day
    prv_shift_history(utc_sec);
  }

  // Update the derived metrics
  prv_update_real_time_derived_metrics();
  prv_update_step_derived_metrics(utc_sec);
  prv_update_hr_derived_metrics();
}


// --------------------------------------------------------------------------------------------
ActivityScalarStore activity_metrics_prv_steps_per_minute(void) {
  ActivityState *state = activity_private_state();
  return state->steps_per_minute;
}


// --------------------------------------------------------------------------------------------
uint32_t activity_metrics_prv_get_distance_mm(void) {
  ActivityState *state = activity_private_state();
  return state->distance_mm;
}


// --------------------------------------------------------------------------------------------
uint32_t activity_metrics_prv_get_resting_calories(void) {
  ActivityState *state = activity_private_state();
  return state->resting_calories;
}


// --------------------------------------------------------------------------------------------
uint32_t activity_metrics_prv_get_active_calories(void) {
  ActivityState *state = activity_private_state();
  return state->active_calories;
}


// --------------------------------------------------------------------------------------------
uint32_t activity_metrics_prv_get_steps(void) {
  ActivityState *state = activity_private_state();
  return state->step_data.steps;
}

static uint8_t prv_get_hr_quality_weight(HRMQuality quality) {
  static const struct {
    HRMQuality quality;
    uint8_t weight_x100;
  } s_hr_quality_weights_x100[] = {
    {HRMQuality_OffWrist, 0 },
    {HRMQuality_Worst, 1 },
    {HRMQuality_Poor, 1 },
    {HRMQuality_Acceptable, 60 },
    {HRMQuality_Good, 65 },
    {HRMQuality_Excellent, 85 },
  };

  for (size_t i = 0; i < ARRAY_LENGTH(s_hr_quality_weights_x100); i++) {
    if (quality == s_hr_quality_weights_x100[i].quality) {
      return s_hr_quality_weights_x100[i].weight_x100;
    }
  }
  return 0;
}

// --------------------------------------------------------------------------------------------
HRZone activity_metrics_prv_get_hr_zone(void) {
  ActivityState *state = activity_private_state();

  return state->hr.metrics.current_hr_zone;
}

// --------------------------------------------------------------------------------------------
void activity_metrics_prv_get_median_hr_bpm(int32_t *median_out,
                                            int32_t *heart_rate_total_weight_x100_out) {
  ActivityState *state = activity_private_state();

  if (median_out) {
    *median_out = state->hr.metrics.previous_median_bpm;
  }
  if (heart_rate_total_weight_x100_out) {
    *heart_rate_total_weight_x100_out = state->hr.metrics.previous_median_total_weight_x100;
  }
}

// --------------------------------------------------------------------------------------------
void activity_metrics_prv_get_spo2_sample(uint8_t *percent_out, uint8_t *quality_out) {
  ActivityState *state = activity_private_state();
  mutex_lock_recursive(state->mutex);
  {
    *percent_out = state->spo2.pending_percent;
    *quality_out = state->spo2.pending_quality;
    // Consume it so the same reading isn't repeated across subsequent minutes.
    state->spo2.pending_percent = 0;
    state->spo2.pending_quality = 0;
  }
  mutex_unlock_recursive(state->mutex);
}

// --------------------------------------------------------------------------------------------
void activity_metrics_prv_reset_hr_stats(void) {
  ActivityState *state = activity_private_state();
  mutex_lock_recursive(state->mutex);
  {
    state->hr.num_samples = 0;
    state->hr.num_good_quality_samples = 0;
    state->hr.num_excellent_samples = 0;
    memset(state->hr.samples, 0, sizeof(state->hr.samples));
    memset(state->hr.weights, 0, sizeof(state->hr.weights));

    state->hr.metrics.previous_median_bpm = 0;
    state->hr.metrics.previous_median_total_weight_x100 = 0;
  }
  mutex_unlock_recursive(state->mutex);
}

// --------------------------------------------------------------------------------------------
void activity_metrics_prv_set_hrm_worn_status(time_t now_utc, bool is_offwrist) {
  ActivityState *state = activity_private_state();
  mutex_lock_recursive(state->mutex);
  {
    state->hr.last_quality_event_utc = now_utc;
    state->hr.last_quality_was_offwrist = is_offwrist;
  }
  mutex_unlock_recursive(state->mutex);
}

// --------------------------------------------------------------------------------------------
bool activity_metrics_prv_is_hrm_offwrist(time_t now_utc) {
  ActivityState *state = activity_private_state();
  bool offwrist = false;
  mutex_lock_recursive(state->mutex);
  {
    if (state->hr.last_quality_event_utc != 0 &&
        state->hr.last_quality_was_offwrist &&
        (now_utc - state->hr.last_quality_event_utc) <= ACTIVITY_HRM_OFFWRIST_STALE_SEC) {
      offwrist = true;
    }
  }
  mutex_unlock_recursive(state->mutex);
  return offwrist;
}

// --------------------------------------------------------------------------------------------
void activity_metrics_prv_add_median_hr_sample(PebbleHRMEvent *hrm_event, time_t now_utc,
                                               time_t now_uptime) {
  ActivityState *state = activity_private_state();
  mutex_lock_recursive(state->mutex);
  {
    // Update stats used for computing the average
    if (hrm_event->bpm.bpm > 0) {
      // This should get reset about once a minute, so X minutes worth of samples means something
      // is terribly wrong.
      PBL_ASSERT(state->hr.num_samples <= ACTIVITY_MAX_HR_SAMPLES, "Too many samples");
      state->hr.samples[state->hr.num_samples] = hrm_event->bpm.bpm;
      state->hr.weights[state->hr.num_samples] =
          prv_get_hr_quality_weight(hrm_event->bpm.quality);
      if (hrm_event->bpm.quality >= HRMQuality_Good) {
        state->hr.num_good_quality_samples++;
      }
      if (hrm_event->bpm.quality >= HRMQuality_Excellent) {
        state->hr.num_excellent_samples++;
      }

      state->hr.num_samples++;
    }
    // Update the timestamp used for figuring out when we should change the sampling period.
    // This is based on uptime so that it doesn't get messed up if the mobile changes the
    // UTC time on us.
    state->hr.last_sample_ts = now_uptime;

    // Save the BPM, quality, and update time (UTC) of the last reading for activity_get_metric()
    state->hr.metrics.current_bpm = hrm_event->bpm.bpm;
    state->hr.metrics.current_quality = hrm_event->bpm.quality;
    state->hr.metrics.current_update_time_utc = now_utc;
  }
  mutex_unlock_recursive(state->mutex);
}

// ------------------------------------------------------------------------------------------------
void activity_metrics_prv_init(SettingsFile *file, time_t utc_now) {
  ActivityState *state = activity_private_state();
  // Roll back the history if needed and init each of the metrics for today
  for (ActivityMetric metric = ActivityMetricFirst; metric < ActivityMetricNumMetrics;
       metric++) {
    ActivityMetricInfo m_info;
    activity_metrics_prv_get_metric_info(metric, &m_info);
    if (m_info.has_history) {
      PBL_ASSERTN(m_info.value_p);
      ActivitySettingsValueHistory old_history = { 0 };
      ActivitySettingsValueHistory new_history = { 0 };

      // In case we change the length of the history, fetch the old size
      int fetch_size = sizeof(old_history);
      fetch_size = MIN(fetch_size, settings_file_get_len(file, &m_info.settings_key,
                       sizeof(m_info.settings_key)));
      settings_file_get(file, &m_info.settings_key, sizeof(m_info.settings_key), &old_history,
                        fetch_size);

      uint16_t day = time_util_get_day(old_history.utc_sec);
      int old_age = state->cur_day_index - day;

      // If this is resting kcalories, the default for each day is not 0
      if (metric == ActivityMetricRestingKCalories) {
        uint32_t full_day_resting_calories =
            activity_private_compute_resting_calories(MINUTES_PER_DAY);
        for (int i = 0; i < ACTIVITY_HISTORY_DAYS; i++) {
          if (i == 0) {
            uint32_t elapsed_minutes = time_util_get_minute_of_day(utc_now);
            uint32_t cur_day_resting_calories =
                activity_private_compute_resting_calories(elapsed_minutes);
            new_history.values[i] = ROUND(cur_day_resting_calories, ACTIVITY_CALORIES_PER_KCAL);
          } else {
            new_history.values[i] = ROUND(full_day_resting_calories, ACTIVITY_CALORIES_PER_KCAL);
          }
        }
      }

      // Copy values from old history into correct slot in new history
      for (int i = 0; i < ACTIVITY_HISTORY_DAYS; i++) {
        int new_index = i + old_age;
        if (new_index >= 0 && new_index < ACTIVITY_HISTORY_DAYS) {
          new_history.values[new_index] = old_history.values[i];
        }
      }
      // init the time stamp if not initialized yet
      if (new_history.utc_sec == 0) {
        new_history.utc_sec = utc_now;
      }

      // Init current value
      *m_info.value_p = new_history.values[0];

      // Only write to flash if the values change or this is a new day (to update the timestamp)
      if (memcmp(old_history.values, new_history.values, sizeof(old_history.values)) != 0
          || old_age != 0) {
        // Write out the updated history
        settings_file_set(file, &m_info.settings_key, sizeof(m_info.settings_key), &new_history,
                          sizeof(new_history));
      }

    } else if (m_info.settings_key != ActivitySettingsKeyInvalid) {
      // Metric with no history, just init current value
      PBL_ASSERTN(m_info.value_p);
      settings_file_get(file, &m_info.settings_key, sizeof(m_info.settings_key), m_info.value_p,
                        sizeof(*m_info.value_p));
    }
  }
}


// ------------------------------------------------------------------------------------------------
bool activity_get_metric(ActivityMetric metric, uint32_t history_len, int32_t *history) {
  ActivityState *state = activity_private_state();
  bool success = true;

  // Default results
  for (uint32_t i = 0; i < history_len; i++) {
    history[i] = -1;
  }

  mutex_lock_recursive(state->mutex);
  {
    // Update derived metrics
    prv_update_real_time_derived_metrics();

    ActivityMetricInfo m_info;
    activity_metrics_prv_get_metric_info(metric, &m_info);

    if (history_len == 0) {
      goto unlock;
    }

    // Clip history length
    history_len = MIN(history_len, ACTIVITY_HISTORY_DAYS);
    if (!m_info.has_history) {
      history_len = 1;
    }

    // Fill in current value
    if (m_info.value_p) {
      history[0] = m_info.converter(*m_info.value_p);
    } else {
      PBL_ASSERTN(m_info.value_u32p && (m_info.converter == prv_convert_none));
      history[0] = *m_info.value_u32p;
    }
    ACTIVITY_LOG_DEBUG("get current metric %"PRIi32" : %"PRIi32"", (int32_t)metric, history[0]);

    // Look up historical values
    if (history_len > 1) {
      // Read from the history stored in settings
      ActivitySettingsValueHistory setting_history = {};
      SettingsFile *file = activity_private_settings_open();
      if (!file) {
        PBL_LOG_ERR("Settings file DNE. No need to continue getting metric");
        success = false;
        goto unlock;
      }
      settings_file_get(file, &m_info.settings_key, sizeof(m_info.settings_key), &setting_history,
                        sizeof(setting_history));
      for (uint32_t i = 1; i < history_len; i++) {
        history[i] = m_info.converter(setting_history.values[i]);
        ACTIVITY_LOG_DEBUG("get metric %"PRIi32" %"PRIu32" days ago: %"PRIi32"", (int32_t)metric,
                           i, history[i]);
      }
      activity_private_settings_close(file);
    }
  }
unlock:
  mutex_unlock_recursive(state->mutex);
  return success;
}


// ------------------------------------------------------------------------------------------------
DEFINE_SYSCALL(bool, sys_activity_get_metric, ActivityMetric metric,
               uint32_t history_len, int32_t *history) {
  if (PRIVILEGE_WAS_ELEVATED) {
    // activity_get_metric() unconditionally writes history_len int32_t entries
    // to `history` before later clamping to ACTIVITY_HISTORY_DAYS. A huge
    // history_len makes `history_len * sizeof(*history)` wrap to a tiny size
    // that passes validation while the kernel keeps writing -1 past the end
    // (effectively an arbitrary-address kernel write).
    //
    // We never read more than ACTIVITY_HISTORY_DAYS entries anyway, so clamp
    // the count here and validate against the clamped value.
    if (history_len > ACTIVITY_HISTORY_DAYS) {
      history_len = ACTIVITY_HISTORY_DAYS;
    }
    if (history) {
      syscall_assert_userspace_buffer(history, history_len * sizeof(*history));
    }
  }

  return activity_get_metric(metric, history_len, history);
}
