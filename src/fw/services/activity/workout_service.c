/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/activity/workout_service.h"

#include "pbl/services/activity/activity_algorithm.h"
#include "pbl/services/activity/activity_calculators.h"
#include "pbl/services/activity/activity_insights.h"
#include "pbl/services/activity/activity_private.h"
#include "pbl/services/activity/hr_util.h"

#include "apps/system/workout/utils.h"
#include "applib/app.h"
#include "applib/health_service.h"
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "pbl/services/evented_timer.h"
#include "pbl/services/regular_timer.h"
#include "pbl/services/system_task.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "util/time/time.h"
#include "util/units.h"

#include <pbl/os/mutex.h>

PBL_LOG_MODULE_DECLARE(service_activity, CONFIG_SERVICE_ACTIVITY_LOG_LEVEL);

#define WORKOUT_HR_READING_TS_EXPIRE (SECONDS_PER_MINUTE)
#define WORKOUT_ENDED_HR_SUBSCRIPTION_TS_EXPIRE (10 * SECONDS_PER_MINUTE)
#define WORKOUT_ACTIVE_HR_SUBSCRIPTION_TS_EXPIRE (SECONDS_PER_HOUR)
#define WORKOUT_ABANDONED_NOTIFICATION_TIMEOUT_MS (55 * MS_PER_MINUTE)
#define WORKOUT_ABANDON_WORKOUT_TIMEOUT_MS (5 * MS_PER_MINUTE)

//! Allocated when a Workout is started
typedef struct CurrentWorkoutData {
  ActivitySessionType type;

  time_t start_utc;
  time_t last_paused_utc;
  time_t duration_completed_pauses_s;

  int32_t duration_s;
  int32_t steps;
  int32_t distance_m;
  // Pace
  int32_t active_calories;
  int32_t current_bpm;
  time_t current_bpm_timestamp_ts; // Time since boot
  HRZone current_hr_zone;
  int32_t hr_zone_time_s[HRZoneCount];
  int32_t hr_samples_sum;
  int32_t hr_samples_count;

  // The normal ActivitySession supplies the workout summary. This buffered DataLogging session
  // retains the one-second sensor readings while the phone is absent.
  DataLoggingSession *heart_rate_dls_session;
  uint32_t next_heart_rate_sequence;
  time_t last_logged_heart_rate_utc;

  // PPI/RR intervals can arrive more than once in a second. Keep their own stream and sequence
  // rather than downsampling or inferring them from the BPM stream.
  DataLoggingSession *ppi_dls_session;
  uint32_t next_ppi_sequence;

  // Step count total from the last HealthEventMovementUpdate
  int32_t last_event_step_count;
  time_t last_movement_event_time_ts;

  // Whether or not the current workout is paused
  bool paused;

  EventedTimerID workout_abandoned_timer;
} CurrentWorkoutData;

//! Persisted statically in RAM
typedef struct WorkoutServiceData {
  PebbleRecursiveMutex *s_workout_mutex;
  RegularTimerInfo second_timer;
  time_t last_workout_end_ts;
  time_t frontend_last_opened_ts;
  HRMSessionRef hrm_session;

  CurrentWorkoutData *current_workout;
} WorkoutServiceData;

static WorkoutServiceData s_workout_data;

typedef struct WorkoutHeartRateDlsFinishData {
  DataLoggingSession *session;
  uint32_t workout_id;
  uint32_t sequence;
  time_t end_utc;
} WorkoutHeartRateDlsFinishData;

typedef struct WorkoutPpiDlsFinishData {
  DataLoggingSession *session;
  uint32_t workout_id;
  uint32_t sequence;
  time_t end_utc;
} WorkoutPpiDlsFinishData;

static void prv_lock(void) {
  mutex_lock_recursive(s_workout_data.s_workout_mutex);
}

static void prv_unlock(void) {
  mutex_unlock_recursive(s_workout_data.s_workout_mutex);
}

static void prv_put_event(PebbleWorkoutEventType e_type) {
  PebbleEvent event = {
    .type = PEBBLE_WORKOUT_EVENT,
    .workout = {
      .type = e_type,
    }
  };
  event_put(&event);
}

static int32_t prv_get_avg_hr(void) {
  if (!s_workout_data.current_workout->hr_samples_count) {
    return 0;
  }

  return ROUND(s_workout_data.current_workout->hr_samples_sum,
               s_workout_data.current_workout->hr_samples_count);
}

static void prv_update_duration(void) {
  // We can't just increment the time on a second callback because of the inaccuracy of our timer
  // system. PBL-32523
  // Instead, we keep track of a start_utc, paused_time, and last_paused_utc. With these
  // we can accurately keep track of the total duration of the workout.
  if (!workout_service_is_workout_ongoing()) {
    return;
  }

  time_t now_utc = rtc_get_time();

  time_t total_paused_time_s = s_workout_data.current_workout->duration_completed_pauses_s;
  if (workout_service_is_paused()) {
    const time_t duration_current_pause = now_utc - s_workout_data.current_workout->last_paused_utc;
    total_paused_time_s += duration_current_pause;
  }

  s_workout_data.current_workout->duration_s =
      now_utc - s_workout_data.current_workout->start_utc - total_paused_time_s;
}

static void prv_reset_hr_data(void) {
  const time_t now_ts = time_get_uptime_seconds();
  s_workout_data.current_workout->current_bpm = 0;
  s_workout_data.current_workout->current_hr_zone = HRZone_Zone0;
  s_workout_data.current_workout->current_bpm_timestamp_ts = now_ts;
}

// ---------------------------------------------------------------------------------------
static bool prv_ensure_workout_heart_rate_dls_session(CurrentWorkoutData *wrkt_data) {
  if (wrkt_data->heart_rate_dls_session) {
    return true;
  }

  // Workout start is initiated in the app task, where a system DataLogging buffer cannot be
  // allocated. Health events are dispatched from the shell task, so create the buffered session
  // lazily here and let the service own its kernel buffer.
  const Uuid system_uuid = UUID_SYSTEM;
  wrkt_data->heart_rate_dls_session = dls_create(
      DlsSystemTagWorkoutHeartRate, DATA_LOGGING_BYTE_ARRAY,
      sizeof(WorkoutHeartRateDataLoggingRecord), true /* buffered */, false /* resume */,
      &system_uuid);
  if (!wrkt_data->heart_rate_dls_session) {
    PBL_LOG_WRN("Unable to create workout HR logging session");
    return false;
  }

  return true;
}

#ifdef CONFIG_HRM_HRV
// ---------------------------------------------------------------------------------------
static bool prv_ensure_workout_ppi_dls_session(CurrentWorkoutData *wrkt_data) {
  if (wrkt_data->ppi_dls_session) {
    return true;
  }

  // The HRV driver emits accepted PPI values on the shell task. Allocate the buffered system
  // session lazily here, after a real interval exists, so a Workout with no valid PPI consumes no
  // DataLogging storage and can still retain its normal ActivitySession summary.
  const Uuid system_uuid = UUID_SYSTEM;
  wrkt_data->ppi_dls_session = dls_create(
      DlsSystemTagWorkoutPpi, DATA_LOGGING_BYTE_ARRAY,
      sizeof(WorkoutPpiDataLoggingRecord), true /* buffered */, false /* resume */,
      &system_uuid);
  if (!wrkt_data->ppi_dls_session) {
    PBL_LOG_WRN("Unable to create workout PPI logging session");
    return false;
  }

  return true;
}

#endif  // CONFIG_HRM_HRV

// ---------------------------------------------------------------------------------------
static void prv_log_workout_heart_rate(CurrentWorkoutData *wrkt_data,
                                       HealthEventHeartRateUpdateData *event) {
  if (wrkt_data->paused || !prv_ensure_workout_heart_rate_dls_session(wrkt_data)) {
    return;
  }

  const time_t now_utc = rtc_get_time();
  // The workout subscription requests one update per second. If the lower-level source emits
  // more than once in a second, keep one deterministic point rather than wasting flash.
  if (now_utc == wrkt_data->last_logged_heart_rate_utc) {
    return;
  }

  WorkoutHeartRateDataLoggingRecord record = {
    .workout_id = (uint32_t)wrkt_data->start_utc,
    .sequence = wrkt_data->next_heart_rate_sequence++,
    .timestamp_utc = (uint32_t)now_utc,
    .bpm = event->current_bpm,
    .quality = event->quality,
    .flags = WORKOUT_HEART_RATE_FLAG_ACTIVE,
    .version = WORKOUT_HEART_RATE_LOGGING_VERSION,
  };

  DataLoggingResult result = dls_log(wrkt_data->heart_rate_dls_session, &record, 1);
  if (result != DATA_LOGGING_SUCCESS) {
    PBL_LOG_WRN("Workout HR log failed: %"PRIi32, (int32_t)result);
    return;
  }

  wrkt_data->last_logged_heart_rate_utc = now_utc;
}

#ifdef CONFIG_HRM_HRV
// ---------------------------------------------------------------------------------------
static void prv_log_workout_ppi(CurrentWorkoutData *wrkt_data,
                                 const HealthEventHRVUpdateData *event) {
  if (wrkt_data->paused || event->ppi_ms == 0 || !prv_ensure_workout_ppi_dls_session(wrkt_data)) {
    return;
  }

  // The firmware's HRV driver has already discarded invalid vendor results. Preserve every
  // accepted non-zero PPI plus its quality; a companion must not manufacture missing intervals.
  WorkoutPpiDataLoggingRecord record = {
    .workout_id = (uint32_t)wrkt_data->start_utc,
    .sequence = wrkt_data->next_ppi_sequence++,
    .timestamp_utc = (uint32_t)rtc_get_time(),
    .ppi_ms = event->ppi_ms,
    .quality = event->quality,
    .flags_and_version = (WORKOUT_PPI_LOGGING_VERSION << WORKOUT_PPI_VERSION_SHIFT) |
        WORKOUT_PPI_FLAG_ACTIVE,
  };

  const DataLoggingResult result = dls_log(wrkt_data->ppi_dls_session, &record, 1);
  if (result != DATA_LOGGING_SUCCESS) {
    // A buffered DLS session retains data locally and reports BUSY only when its bounded RAM
    // staging buffer is full. Do not reuse this sequence after a failed append: a future retry
    // must never overwrite an interval the phone might already have received.
    PBL_LOG_WRN("Workout PPI log failed: %"PRIi32, (int32_t)result);
  }
}
#endif  // CONFIG_HRM_HRV

// ---------------------------------------------------------------------------------------
static void prv_finish_workout_heart_rate_dls_session(void *data) {
  WorkoutHeartRateDlsFinishData *finish_data = data;
  WorkoutHeartRateDataLoggingRecord record = {
    .workout_id = finish_data->workout_id,
    .sequence = finish_data->sequence,
    .timestamp_utc = (uint32_t)finish_data->end_utc,
    .quality = -128, // terminal marker; not a sensor measurement
    .flags = WORKOUT_HEART_RATE_FLAG_COMPLETE,
    .version = WORKOUT_HEART_RATE_LOGGING_VERSION,
  };
  DataLoggingResult result = dls_log(finish_data->session, &record, 1);
  // Buffered sessions can temporarily be full while their preceding samples are being written
  // from the System Task queue.  The completion marker is the companion's commit record: losing
  // it would leave every otherwise valid detail point pending forever.  Requeue behind that
  // writer rather than finishing the session without a marker.
  if (result == DATA_LOGGING_BUSY &&
      system_task_add_callback(prv_finish_workout_heart_rate_dls_session, finish_data)) {
    PBL_LOG_WRN("Workout HR completion buffer busy; retrying");
    return;
  }
  if (result != DATA_LOGGING_SUCCESS) {
    PBL_LOG_WRN("Workout HR completion log failed: %"PRIi32, (int32_t)result);
  }
  dls_finish(finish_data->session);
  kernel_free(finish_data);
}

// ---------------------------------------------------------------------------------------
static void prv_finish_workout_ppi_dls_session(void *data) {
  WorkoutPpiDlsFinishData *finish_data = data;
  WorkoutPpiDataLoggingRecord record = {
    .workout_id = finish_data->workout_id,
    .sequence = finish_data->sequence,
    .timestamp_utc = (uint32_t)finish_data->end_utc,
    .quality = -128, // terminal marker; not a sensor measurement
    .flags_and_version = (WORKOUT_PPI_LOGGING_VERSION << WORKOUT_PPI_VERSION_SHIFT) |
        WORKOUT_PPI_FLAG_COMPLETE,
  };
  const DataLoggingResult result = dls_log(finish_data->session, &record, 1);
  if (result == DATA_LOGGING_BUSY &&
      system_task_add_callback(prv_finish_workout_ppi_dls_session, finish_data)) {
    PBL_LOG_WRN("Workout PPI completion buffer busy; retrying");
    return;
  }
  if (result != DATA_LOGGING_SUCCESS) {
    PBL_LOG_WRN("Workout PPI completion log failed: %"PRIi32, (int32_t)result);
  }
  dls_finish(finish_data->session);
  kernel_free(finish_data);
}

// ---------------------------------------------------------------------------------------
static void prv_handle_movement_update(HealthEventMovementUpdateData *event) {
  CurrentWorkoutData *wrkt_data = s_workout_data.current_workout;

  const int32_t new_event_steps = event->steps;
  const time_t now_ts = time_get_uptime_seconds();
  if (new_event_steps < wrkt_data->last_event_step_count) {
    PBL_LOG_WRN("Working out through midnight, resetting last_event_step_count");
    wrkt_data->last_event_step_count = 0;
  }

  if (!workout_service_is_paused()) {
    // Calculate the step delta
    const uint32_t delta_steps = new_event_steps - wrkt_data->last_event_step_count;
    wrkt_data->steps += delta_steps;

    // Calculate the distance delta
    const time_t delta_ms = (now_ts - wrkt_data->last_movement_event_time_ts) * MS_PER_SECOND;
    const int32_t delta_distance_mm = activity_private_compute_distance_mm(delta_steps, delta_ms);
    wrkt_data->distance_m += (delta_distance_mm / MM_PER_METER);

    // Calculate active calories
    const int32_t active_calories = activity_private_compute_active_calories(delta_distance_mm,
                                                                             delta_ms);
    wrkt_data->active_calories += active_calories;
  }

  // Reset the last event count regardless of whether we are paused
  wrkt_data->last_event_step_count = new_event_steps;
  wrkt_data->last_movement_event_time_ts = now_ts;
}

static void prv_handle_heart_rate_update(HealthEventHeartRateUpdateData *event) {
  CurrentWorkoutData *wrkt_data = s_workout_data.current_workout;

  if (event->is_filtered) {
    // We don't care about median heart rate updates
    return;
  }

  if (event->quality == HRMQuality_OffWrist) {
    // Reset to zero for OffWrist readings
    prv_reset_hr_data();
  } else if (event->quality >= HRMQuality_Worst) {
    const int prev_bpm_timestamp_ts = wrkt_data->current_bpm_timestamp_ts;

    wrkt_data->current_bpm = event->current_bpm;
    wrkt_data->current_hr_zone = hr_util_get_hr_zone(wrkt_data->current_bpm);
    wrkt_data->current_bpm_timestamp_ts = time_get_uptime_seconds();

    if (!workout_service_is_paused()) {
      // TODO: Maybe apply smoothing
      wrkt_data->hr_zone_time_s[wrkt_data->current_hr_zone] +=
          wrkt_data->current_bpm_timestamp_ts - prev_bpm_timestamp_ts;
      wrkt_data->hr_samples_count++;
      wrkt_data->hr_samples_sum += event->current_bpm;
      prv_log_workout_heart_rate(wrkt_data, event);
    }
  }
  return;
}

// ---------------------------------------------------------------------------------------
static void prv_handle_hrv_update(HealthEventHRVUpdateData *event) {
#ifdef CONFIG_HRM_HRV
  prv_log_workout_ppi(s_workout_data.current_workout, event);
#else
  (void)event;
#endif
}

// ---------------------------------------------------------------------------------------
bool workout_service_is_workout_type_supported(ActivitySessionType type) {
  return type == ActivitySessionType_Walk ||
         type == ActivitySessionType_Run ||
         type == ActivitySessionType_Open;
}

// ---------------------------------------------------------------------------------------
T_STATIC void prv_abandon_workout_timer_callback(void *unused) {
  workout_service_stop_workout();
}

// ---------------------------------------------------------------------------------------
T_STATIC void prv_abandoned_notification_timer_callback(void *unused) {
  workout_utils_send_abandoned_workout_notification();

  s_workout_data.current_workout->workout_abandoned_timer =
      evented_timer_register(WORKOUT_ABANDON_WORKOUT_TIMEOUT_MS, false,
                             prv_abandon_workout_timer_callback, NULL);
}

// ---------------------------------------------------------------------------------------
T_STATIC void prv_workout_timer_cb(void *unused) {
  // This runs on the NewTimer task, which has a watchdog. The workout mutex
  // can be held by paths that block on flash (e.g. stop_workout pushing the
  // session notification), so a blocking lock here can trip the watchdog and
  // reboot the watch. The cb is purely advisory (duration tick / HR aging),
  // so if the lock is contended just skip this tick — the next one catches up.
  if (!mutex_lock_recursive_with_timeout(s_workout_data.s_workout_mutex, 0)) {
    return;
  }

  if (s_workout_data.current_workout) {
    prv_update_duration();

    const time_t now_ts = time_get_uptime_seconds();
    const time_t age_hr_s = now_ts - s_workout_data.current_workout->current_bpm_timestamp_ts;
    if (s_workout_data.current_workout->current_bpm != 0 &&
        age_hr_s >= WORKOUT_HR_READING_TS_EXPIRE) {
      prv_reset_hr_data();
    }
  }
  prv_unlock();
}

// ---------------------------------------------------------------------------------------
void workout_service_health_event_handler(PebbleHealthEvent *event) {
  prv_lock();
  {
    if (!s_workout_data.current_workout) {
      goto unlock;
    }
    if (event->type == HealthEventMovementUpdate) {
      prv_handle_movement_update(&event->data.movement_update);
    } else if (event->type == HealthEventHeartRateUpdate) {
      prv_handle_heart_rate_update(&event->data.heart_rate_update);
    } else if (event->type == HealthEventHRVUpdate) {
      prv_handle_hrv_update(&event->data.hrv_update);
    }
  }
unlock:
  prv_unlock();
}

// ---------------------------------------------------------------------------------------
void workout_service_activity_event_handler(PebbleActivityEvent *event) {
  // workout_service_pause_workout is a no-op when no workout is ongoing,
  // so let it do the check under the lock rather than racing it here.
  if (event->type == PebbleActivityEvent_TrackingStopped) {
    workout_service_pause_workout(true);
  }
}

// ---------------------------------------------------------------------------------------
void workout_service_workout_event_handler(PebbleWorkoutEvent *event) {
  prv_lock();
  {
    if (!s_workout_data.current_workout) {
      goto unlock;
    }
    // Handling this with an event because the timer needs to be called from KernelMain
    if (event->type == PebbleWorkoutEvent_FrontendOpened) {
      evented_timer_cancel(s_workout_data.current_workout->workout_abandoned_timer);
    } else if (event->type == PebbleWorkoutEvent_FrontendClosed) {
      s_workout_data.current_workout->workout_abandoned_timer =
          evented_timer_register(WORKOUT_ABANDONED_NOTIFICATION_TIMEOUT_MS, false,
                                 prv_abandoned_notification_timer_callback, NULL);
    }
  }
unlock:
  prv_unlock();
}

// ---------------------------------------------------------------------------------------
void workout_service_init(void) {
  s_workout_data.s_workout_mutex = mutex_create_recursive();
}

// ---------------------------------------------------------------------------------------
// FIXME: We should probably handle this on KernelBG and not use the official app subscription
void workout_service_frontend_opened(void) {
  PBL_ASSERT_TASK(PebbleTask_App);
  prv_lock();
  {
#ifdef CONFIG_HRM
    s_workout_data.hrm_session =
        sys_hrm_manager_app_subscribe(app_get_app_id(), 1, 0, HRMFeature_BPM);
#endif // CONFIG_HRM
    s_workout_data.frontend_last_opened_ts = time_get_uptime_seconds();
    prv_put_event(PebbleWorkoutEvent_FrontendOpened);
  }
  prv_unlock();
}


// ---------------------------------------------------------------------------------------
// FIXME: We should probably handle this on KernelBG and not use the official app subscription
void workout_service_frontend_closed(void) {
  PBL_ASSERT_TASK(PebbleTask_App);
  prv_lock();
  {
#ifdef CONFIG_HRM
    int32_t hr_time_left;

    if (workout_service_is_workout_ongoing()) {
      // The workout app can be closed without stopping the workout. In this scenario keep
      // collecting HR data until so much time has passed that it is assumed the user has forgotten
      // about the workout
      hr_time_left = WORKOUT_ACTIVE_HR_SUBSCRIPTION_TS_EXPIRE;
    } else if (s_workout_data.frontend_last_opened_ts >= s_workout_data.last_workout_end_ts) {
      // If the app was opened and closed without starting a workout, turn the HR sensor off
      hr_time_left = 0;
    } else {
      // We have ended a workout while the app was open. Make sure to keep the HR sensor on for at
      // least a little bit after the workout is finished
      const time_t now_ts = time_get_uptime_seconds();
      const time_t time_since_workout = (now_ts - s_workout_data.last_workout_end_ts);

      // After a workout has finished, keep the HR sensor on for a bit to capture the user's HR
      // returning to a normal level.
      hr_time_left = WORKOUT_ENDED_HR_SUBSCRIPTION_TS_EXPIRE - time_since_workout;
    }

    if (hr_time_left > 0) {
      // Still some time left. Set a subscription with an expiration
      s_workout_data.hrm_session = sys_hrm_manager_app_subscribe(
          app_get_app_id(), 1, hr_time_left,
#ifdef CONFIG_HRM_HRV
          workout_service_is_workout_ongoing() ? (HRMFeature_BPM | HRMFeature_HRV) : HRMFeature_BPM
#else
          HRMFeature_BPM
#endif
      );
    } else {
      // No time left. Kill the subscription
      sys_hrm_manager_unsubscribe(s_workout_data.hrm_session);
    }
#endif // CONFIG_HRM

    prv_put_event(PebbleWorkoutEvent_FrontendClosed);
  }
  prv_unlock();
}


// ---------------------------------------------------------------------------------------
bool workout_service_start_workout(ActivitySessionType type) {
  bool rv = true;
  prv_lock();
  {
    if (!workout_service_is_workout_type_supported(type)) {
      rv = false;
      goto unlock;
    }

    if (workout_service_is_workout_ongoing()) {
      PBL_LOG_WRN("Only 1 workout at a time is supported");
      rv = false;
      goto unlock;
    }

    // Before starting this new session we need to deal with any in progress sessions
    uint32_t num_sessions = 0;
    ActivitySession *sessions = kernel_zalloc_check(sizeof(ActivitySession) *
                                                    ACTIVITY_MAX_ACTIVITY_SESSIONS_COUNT);
    activity_get_sessions(&num_sessions, sessions);
    for (unsigned i = 0; i < num_sessions; i++) {
      // End and save any automatically detected ongoing sessions
      if (sessions[i].ongoing) {
        sessions[i].ongoing = false;
        activity_sessions_prv_add_activity_session(&sessions[i]);
      }
    }
    kernel_free(sessions);

    s_workout_data.current_workout = kernel_zalloc_check(sizeof(CurrentWorkoutData));
    s_workout_data.current_workout->type = type;
    s_workout_data.current_workout->start_utc = rtc_get_time();
    s_workout_data.current_workout->current_bpm_timestamp_ts = time_get_uptime_seconds();
    // FIXME: This probably doesn't need to be on a timer. We can just flush out a new time on each
    // API function call
    s_workout_data.second_timer = (RegularTimerInfo) {
      .cb = prv_workout_timer_cb,
    };

    // Initialize all of our initial values for keeping track of metrics
    activity_get_metric(ActivityMetricStepCount, 1,
                        &s_workout_data.current_workout->last_event_step_count);
    s_workout_data.current_workout->last_movement_event_time_ts = time_get_uptime_seconds();

    regular_timer_add_seconds_callback(&s_workout_data.second_timer);

#ifdef CONFIG_HRM
    // A normal Workout begins with the existing one-second BPM subscription. Add HRV only while
    // the user is actively recording: this causes the optional Time 2 driver path to run and
    // stops it again on Workout end, without changing idle or merely-open Workout battery use.
    if (s_workout_data.hrm_session != HRM_INVALID_SESSION_REF) {
#ifdef CONFIG_HRM_HRV
      sys_hrm_manager_set_features(s_workout_data.hrm_session, HRMFeature_BPM | HRMFeature_HRV);
#endif
    }
#endif

    // Finally tell our algorithm it should stop automatically tracking activities
    activity_algorithm_enable_activity_tracking(false /* disable */);

    PBL_LOG_INFO("Starting a workout with type: %d", type);
    prv_put_event(PebbleWorkoutEvent_Started);
  }
unlock:
  prv_unlock();
  return rv;
}

// ---------------------------------------------------------------------------------------
bool workout_service_pause_workout(bool should_be_paused) {
  bool rv = false;
  prv_lock();
  {
    if (!s_workout_data.current_workout) {
      PBL_LOG_WRN("Workout (un)pause requested but no workout in progress");
      goto unlock;
    }

    CurrentWorkoutData *wrkt_data = s_workout_data.current_workout;

    if (wrkt_data->paused == should_be_paused) {
      // If no change in state, return early and successful
      rv = true;
      goto unlock;
    }

    if (wrkt_data->paused) {
      // We are paused and want to unpause. Add the in progress pause time to the total
      wrkt_data->duration_completed_pauses_s += (rtc_get_time() - wrkt_data->last_paused_utc);
    } else {
      // We are unpaused and want to pause. Set the last_paused_utc timestamp
      wrkt_data->last_paused_utc = rtc_get_time();
    }

    wrkt_data->paused = should_be_paused;

    // Update the global duration since we have changed the pause state
    prv_update_duration();
    PBL_LOG_INFO("Paused a workout with type: %d", wrkt_data->type);
    prv_put_event(PebbleWorkoutEvent_Paused);
    rv = true;
  }
unlock:
  prv_unlock();
  return rv;
}

// ---------------------------------------------------------------------------------------
bool workout_service_stop_workout(void) {
  bool save_session = false;
  ActivitySession session_to_save;
  int32_t avg_hr_to_save = 0;
  int32_t hr_zone_time_s_to_save[HRZoneCount];
  WorkoutHeartRateDlsFinishData *heart_rate_dls_finish_data = NULL;
  WorkoutPpiDlsFinishData *ppi_dls_finish_data = NULL;

  prv_lock();
  {
    if (!workout_service_is_workout_ongoing()) {
      PBL_LOG_WRN("No workout in progress");
      prv_unlock();
      return false;
    }

    CurrentWorkoutData *wrkt = s_workout_data.current_workout;

    // Snapshot the session data so we can persist it after dropping the
    // workout mutex. activity_insights_push_activity_session_notification
    // creates a notification (blob_db flash write) that can take long enough
    if (wrkt->duration_s >= SECONDS_PER_MINUTE) {
      const time_t len_min = MIN(ACTIVITY_SESSION_MAX_LENGTH_MIN,
                                 wrkt->duration_s / SECONDS_PER_MINUTE);
      session_to_save = (ActivitySession) {
        .type = wrkt->type,
        .start_utc = wrkt->start_utc,
        .length_min = len_min,
        .ongoing = false,
        .manual = true,
        .step_data.steps = wrkt->steps,
        .step_data.distance_meters = wrkt->distance_m,
        .step_data.active_kcalories = ROUND(wrkt->active_calories,
                                            ACTIVITY_CALORIES_PER_KCAL),
        .step_data.resting_kcalories = ROUND(activity_private_compute_resting_calories(len_min),
                                             ACTIVITY_CALORIES_PER_KCAL),
      };
      avg_hr_to_save = prv_get_avg_hr();
      memcpy(hr_zone_time_s_to_save, wrkt->hr_zone_time_s, sizeof(hr_zone_time_s_to_save));
      s_workout_data.last_workout_end_ts = time_get_uptime_seconds();
      save_session = true;
    }

    regular_timer_remove_callback(&s_workout_data.second_timer);

    // Re-enable automatic activity tracking
    activity_algorithm_enable_activity_tracking(true /* enable */);

#ifdef CONFIG_HRM
    // The workout app holds an unbounded 1-second HR subscription while open. Now that the workout
    // has ended, start the post-workout recovery countdown immediately so HR sampling drops back to
    // the user's preferred rate within a bounded window, regardless of when the app actually exits.
    sys_hrm_manager_set_update_interval(s_workout_data.hrm_session, 1,
                                        WORKOUT_ENDED_HR_SUBSCRIPTION_TS_EXPIRE);
#ifdef CONFIG_HRM_HRV
    // Return to the existing BPM-only recovery behavior before releasing the Workout state.
    sys_hrm_manager_set_features(s_workout_data.hrm_session, HRMFeature_BPM);
#endif
#endif // CONFIG_HRM

    PBL_LOG_INFO("Stopping a workout with type: %d", wrkt->type);
    prv_put_event(PebbleWorkoutEvent_Stopped);

    if (wrkt->heart_rate_dls_session) {
      heart_rate_dls_finish_data = kernel_zalloc_check(sizeof(*heart_rate_dls_finish_data));
      *heart_rate_dls_finish_data = (WorkoutHeartRateDlsFinishData) {
        .session = wrkt->heart_rate_dls_session,
        .workout_id = (uint32_t)wrkt->start_utc,
        .sequence = wrkt->next_heart_rate_sequence,
        .end_utc = rtc_get_time(),
      };
    }
    if (wrkt->ppi_dls_session) {
      ppi_dls_finish_data = kernel_zalloc_check(sizeof(*ppi_dls_finish_data));
      *ppi_dls_finish_data = (WorkoutPpiDlsFinishData) {
        .session = wrkt->ppi_dls_session,
        .workout_id = (uint32_t)wrkt->start_utc,
        .sequence = wrkt->next_ppi_sequence,
        .end_utc = rtc_get_time(),
      };
    }
    kernel_free(s_workout_data.current_workout);
    s_workout_data.current_workout = NULL;
  }
  prv_unlock();

  if (save_session) {
    activity_sessions_prv_add_activity_session(&session_to_save);
    activity_insights_push_activity_session_notification(rtc_get_time(), &session_to_save,
                                                         avg_hr_to_save, hr_zone_time_s_to_save);
  }

  if (heart_rate_dls_finish_data) {
    // Finish after releasing the workout mutex: buffered data is first flushed to flash and then
    // handed to the existing companion DataLogging path. No Bluetooth connection is required.
    if (!system_task_add_callback(prv_finish_workout_heart_rate_dls_session,
                                  heart_rate_dls_finish_data)) {
      PBL_LOG_WRN("Unable to finish workout HR logging session");
      kernel_free(heart_rate_dls_finish_data);
    }
  }
  if (ppi_dls_finish_data) {
    if (!system_task_add_callback(prv_finish_workout_ppi_dls_session, ppi_dls_finish_data)) {
      PBL_LOG_WRN("Unable to finish workout PPI logging session");
      kernel_free(ppi_dls_finish_data);
    }
  }

  return true;
}

// ---------------------------------------------------------------------------------------
bool workout_service_is_workout_ongoing(void) {
  prv_lock();
  bool rv = (s_workout_data.current_workout != NULL);
  prv_unlock();
  return rv;
}

// ---------------------------------------------------------------------------------------
bool workout_service_takeover_activity_session(ActivitySession *session) {
  bool rv = true;
  prv_lock();
  {
    if (!workout_service_is_workout_type_supported(session->type)) {
      rv = false;
      goto unlock;
    }

    ActivitySession session_copy = *session;

    // Remove the session from out list of sessions so it doesn't get counted twice
    activity_sessions_prv_delete_activity_session(session);

    // Start a new workout
    if (!workout_service_start_workout(session_copy.type)) {
      rv = false;
      goto unlock;
    }

    // Update the new workout to mirror the session we took over
    s_workout_data.current_workout->start_utc = session_copy.start_utc;
    s_workout_data.current_workout->duration_s = session_copy.length_min * SECONDS_PER_MINUTE;
    s_workout_data.current_workout->steps = session_copy.step_data.steps;
    s_workout_data.current_workout->distance_m = session_copy.step_data.distance_meters;
    s_workout_data.current_workout->active_calories =
        session_copy.step_data.active_kcalories * ACTIVITY_CALORIES_PER_KCAL;
  }
unlock:
  prv_unlock();
  return rv;
}

// ---------------------------------------------------------------------------------------
bool workout_service_is_paused(void) {
  prv_lock();
  bool rv = (workout_service_is_workout_ongoing() && s_workout_data.current_workout->paused);
  prv_unlock();
  return rv;
}


// ---------------------------------------------------------------------------------------
bool workout_service_get_current_workout_type(ActivitySessionType *type_out) {
  bool rv = true;
  prv_lock();
  if (!type_out || !workout_service_is_workout_ongoing()) {
    rv = false;
  } else {
    if (type_out) {
      *type_out = s_workout_data.current_workout->type;
    }
  }
  prv_unlock();
  return rv;
}

// ---------------------------------------------------------------------------------------
bool workout_service_get_current_workout_info(int32_t *steps_out, int32_t *duration_s_out,
                                              int32_t *distance_m_out, int32_t *current_bpm_out,
                                              HRZone *current_hr_zone_out) {
  bool rv = true;
  prv_lock();
  {
    if (!workout_service_is_workout_ongoing()) {
      rv = false;
    } else {
      if (steps_out) {
        *steps_out = s_workout_data.current_workout->steps;
      }
      if (duration_s_out) {
        *duration_s_out = s_workout_data.current_workout->duration_s;
      }
      if (distance_m_out) {
        *distance_m_out = s_workout_data.current_workout->distance_m;
      }
      if (current_bpm_out) {
        *current_bpm_out = s_workout_data.current_workout->current_bpm;
      }
      if (current_hr_zone_out) {
        *current_hr_zone_out = s_workout_data.current_workout->current_hr_zone;
      }
    }
  }
  prv_unlock();
  return rv;
}

#if UNITTEST
bool workout_service_get_avg_hr(int32_t *avg_hr_out) {
  if (!avg_hr_out || !workout_service_is_workout_ongoing()) {
    return false;
  }

  *avg_hr_out = prv_get_avg_hr();
  return true;
}

bool workout_service_get_current_workout_hr_zone_time(int32_t *hr_zone_time_s_out) {
  if (!hr_zone_time_s_out || !workout_service_is_workout_ongoing()) {
    return false;
  }

  prv_lock();
  {
    hr_zone_time_s_out[HRZone_Zone0] = s_workout_data.current_workout->hr_zone_time_s[HRZone_Zone0];
    hr_zone_time_s_out[HRZone_Zone1] = s_workout_data.current_workout->hr_zone_time_s[HRZone_Zone1];
    hr_zone_time_s_out[HRZone_Zone2] = s_workout_data.current_workout->hr_zone_time_s[HRZone_Zone2];
    hr_zone_time_s_out[HRZone_Zone3] = s_workout_data.current_workout->hr_zone_time_s[HRZone_Zone3];
  }
  prv_unlock();
  return true;
}

void workout_service_get_active_kcalories(int32_t *active) {
  if (workout_service_is_workout_ongoing()) {
    *active = ROUND(s_workout_data.current_workout->active_calories, ACTIVITY_CALORIES_PER_KCAL);
  }
}

void workout_service_reset(void) {
  if (s_workout_data.current_workout) {
    kernel_free(s_workout_data.current_workout);
  }
  s_workout_data = (WorkoutServiceData) {};
}
#endif
