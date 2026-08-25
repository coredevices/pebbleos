/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "pbl/services/activity/activity.h"
#include "pbl/services/activity/activity_calculators.h"
#include "pbl/services/activity/workout_service.h"
#include "pbl/services/hrm/hrm_manager.h"
#include "process_management/app_install_types.h"
#include "util/time/time.h"
#include "util/units.h"

// ---------------------------------------------------------------------------------------
#include "stubs_activity_insights.h"
#include "stubs_evented_timer.h"
#include "stubs_logging.h"
#include "stubs_pbl_malloc.h"
#include "stubs_regular_timer.h"

#include "fake_rtc.h"
#include "fake_mutex.h"

extern void prv_abandon_workout_timer_callback(void *data);
extern void prv_abandoned_notification_timer_callback(void *data);
extern void prv_workout_timer_cb(void *data);
extern bool workout_service_get_avg_hr(int32_t *avg_hr_out);
extern bool workout_service_get_current_workout_hr_zone_time(int32_t *hr_zone_time_s_out);
extern void workout_service_get_active_kcalories(int32_t *active);
extern void workout_service_reset(void);

// Stubs
///////////////////////////////////////////////////////////

static ActivitySession s_saved_session;
void activity_sessions_prv_add_activity_session(ActivitySession *session) {
  s_saved_session = *session;
}
void activity_sessions_prv_delete_activity_session(ActivitySession *session) {}
void activity_algorithm_enable_activity_tracking(bool enable) {}

bool activity_get_sessions(uint32_t *session_entries, ActivitySession *sessions) {
  return false;
}

uint8_t activity_prefs_get_age_years(void) {
  return 30; // This is our current default
}

ActivityGender activity_prefs_get_gender(void) {
  return ActivityGenderMale;
}

uint16_t activity_prefs_get_weight_dag(void) {
  return 7539;
}

uint16_t activity_prefs_get_height_mm(void) {
  return 1900;
}

uint8_t activity_prefs_heart_get_elevated_hr(void) {
  return 100;
}

uint8_t activity_prefs_heart_get_zone1_threshold(void) {
  return 130;
}

uint8_t activity_prefs_heart_get_zone2_threshold(void) {
  return 154;
}

uint8_t activity_prefs_heart_get_zone3_threshold(void) {
  return 172;
}

static HRMonitoringInterval s_hrm_measurement_interval;
HRMonitoringInterval activity_prefs_get_hrm_measurement_interval(void) {
  return s_hrm_measurement_interval;
}

AppInstallId app_get_app_id(void) {
  return 0;
}

// ---------------------------------------------------------------------------------------

static bool s_hrm_subscribed;
static uint32_t s_hrm_expiration;
HRMSessionRef sys_hrm_manager_app_subscribe(AppInstallId app_id, uint32_t update_interval_s,
                                            uint16_t expire_s, HRMFeature features) {
  s_hrm_subscribed = true;
  s_hrm_expiration = expire_s;
  return 1;
}

bool sys_hrm_manager_unsubscribe(HRMSessionRef ref) {
  s_hrm_subscribed = false;
  s_hrm_expiration = 0;
  return true;
}

bool sys_hrm_manager_set_update_interval(HRMSessionRef session, uint32_t update_interval_s,
                                         uint16_t expire_s) {
  s_hrm_expiration = expire_s;
  return true;
}

uint32_t time_get_uptime_seconds(void) {
  return SECONDS_PER_DAY + rtc_get_time();
}

static uint32_t s_total_step_count;
bool activity_get_metric(ActivityMetric metric, uint32_t history_len, int32_t *history) {
  if (metric != ActivityMetricStepCount) {
    return false;
  }
  *history = s_total_step_count;
  return true;
}

static bool s_abandoned_workout_notification_sent;
void workout_utils_send_abandoned_workout_notification() {
  s_abandoned_workout_notification_sent = true;
}

// ---------------------------------------------------------------------------------------
static void prv_inc_steps_and_put_event(int steps) {
  s_total_step_count += steps;

  PebbleEvent event = {
    .type = PEBBLE_HEALTH_SERVICE_EVENT,
    .health_event = {
      .type = HealthEventMovementUpdate,
      .data.movement_update.steps = s_total_step_count,
    },
  };
  event_put(&event);

  workout_service_health_event_handler(&event.health_event);
}

// ---------------------------------------------------------------------------------------
static void prv_put_bpm_event(int bpm, HRMQuality quality) {
  PebbleEvent event = {
    .type = PEBBLE_HEALTH_SERVICE_EVENT,
    .health_event = {
      .type = HealthEventHeartRateUpdate,
      .data.heart_rate_update.current_bpm = bpm,
      .data.heart_rate_update.quality = quality,
    },
  };
  event_put(&event);

  workout_service_health_event_handler(&event.health_event);
}

static void prv_inc_time(int seconds) {
  fake_rtc_increment_time(seconds);
  prv_workout_timer_cb(NULL);
}

// ---------------------------------------------------------------------------------------
void test_workout_service__initialize(void) {
  fake_rtc_init(0, 0);

  workout_service_reset();

  // Start the step value for the day at X
  s_total_step_count = 5000;
  s_hrm_expiration = 0;
  s_hrm_subscribed = false;
  s_hrm_measurement_interval = HRMonitoringInterval_10Min;
  s_abandoned_workout_notification_sent = false;

  const bool assert_all_unlocked = true;
  fake_mutex_reset(assert_all_unlocked);
  workout_service_init();
}

void test_workout_service__cleanup(void) {
  workout_service_stop_workout();

  const bool assert_all_unlocked = true;
  fake_mutex_reset(assert_all_unlocked);
}

// ---------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------
void test_workout_service__basic(void) {
  cl_assert(workout_service_start_workout(ActivitySessionType_Run));

  int32_t steps, duration_s, distance_m, bpm;
  HRZone hr_zone;

  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(steps, 0);
  cl_assert_equal_i(duration_s, 0);
  cl_assert_equal_i(distance_m, 0);
  cl_assert_equal_i(bpm, 0);
  cl_assert_equal_i(hr_zone, 0);

  // Get some step info
  prv_inc_time(5 * SECONDS_PER_MINUTE);
  prv_inc_steps_and_put_event(900 /* 180 steps per min * 5 mins */);
  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(steps, 900);
  cl_assert_equal_i(5900, s_total_step_count);
  cl_assert_equal_i(distance_m, 1201 /* 1.2km in 5 mins is reasonable */);
  cl_assert_equal_i(duration_s, 5 * SECONDS_PER_MINUTE);

  // Get some HR info
  prv_put_bpm_event(100, HRMQuality_Good);
  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(bpm, 100);
  cl_assert_equal_i(hr_zone, 0);

  // Get some more step info
  prv_inc_time(5 * SECONDS_PER_MINUTE);
  prv_inc_steps_and_put_event(900 /* 180 steps per min * 5 mins */);
  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(steps, 1800);
  cl_assert_equal_i(6800, s_total_step_count);
  cl_assert_equal_i(distance_m, 2402 /* 2.4km in 10 mins is reasonable */);
  cl_assert_equal_i(duration_s, 10 * SECONDS_PER_MINUTE);

  // Get some more HR info
  prv_inc_time(10);
  prv_put_bpm_event(180, HRMQuality_Good);
  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(bpm, 180);
  cl_assert_equal_i(hr_zone, 3);
  cl_assert_equal_i(duration_s, 10 * SECONDS_PER_MINUTE + 10);

  cl_assert(workout_service_stop_workout());
  cl_assert(!workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                      &bpm, &hr_zone));
}

// ---------------------------------------------------------------------------------------
void test_workout_service__takeover_activity_session(void) {
  int32_t steps, duration_s, distance_m, bpm, active_kcalories;
  HRZone hr_zone;

  // We have a session that started at utc=10 and length_min of 10. Increment the time
  // so that makes sense
  prv_inc_time(610);

  // Start a new session
  ActivitySession session = {
    .start_utc = 10,
    .length_min = 10,
    .type = ActivitySessionType_Run,
    .ongoing = true,
    .manual = false,
    .step_data = {
      .steps = 2000,
      .active_kcalories = 200,
      .resting_kcalories = 100,
      .distance_meters = 1600,
    },
  };

  cl_assert(workout_service_takeover_activity_session(&session));

  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(steps, 2000);
  cl_assert_equal_i(duration_s, 600);
  cl_assert_equal_i(distance_m, 1600);

  // Get the active calories, should be same as session
  workout_service_get_active_kcalories(&active_kcalories);
  cl_assert_equal_i(200, active_kcalories);

  // Add some time and steps
  prv_inc_time(600); /* 10 mins */
  prv_inc_steps_and_put_event(1800 /* 180 steps per min * 10 mins */);

  // Grab the new distance and active_kcalories
  int32_t new_active_kcalories, new_distance_m;
  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &new_distance_m,
                                                     &bpm, &hr_zone));

  // Compute our own version of active_kcalories
  const int32_t distance_delta_m = (new_distance_m - distance_m);
  const int32_t calculated_active_kcalories
      = ROUND(activity_private_compute_active_calories(distance_delta_m * MM_PER_METER, 600 * MS_PER_SECOND),
              ACTIVITY_CALORIES_PER_KCAL);

  // Make sure that the new_active_kcalories has increased
  workout_service_get_active_kcalories(&new_active_kcalories);
  cl_assert(active_kcalories < new_active_kcalories);

  // Double check that we calculated the same value that the workout service got
  cl_assert_equal_i(calculated_active_kcalories, new_active_kcalories - active_kcalories);

  workout_service_stop_workout();

  // Pull out the saved session and make sure it matches our numbers
  ActivitySession *stored_session = &s_saved_session;
  cl_assert_equal_i(stored_session->start_utc, session.start_utc);
  cl_assert_equal_i(stored_session->length_min, 20);
  cl_assert_equal_i(stored_session->step_data.steps, 3800);
  cl_assert_equal_i(stored_session->step_data.active_kcalories, 200 + calculated_active_kcalories);
  cl_assert_equal_i(stored_session->step_data.resting_kcalories,
                    ROUND(activity_private_compute_resting_calories(stored_session->length_min),
                          ACTIVITY_CALORIES_PER_KCAL));
  cl_assert_equal_b(stored_session->manual, true);
  cl_assert(stored_session->step_data.distance_meters > 2000);
}

// ---------------------------------------------------------------------------------------
void test_workout_service__pause_resume(void) {
  int32_t steps, duration_s, distance_m, bpm;
  HRZone hr_zone;

  cl_assert(workout_service_start_workout(ActivitySessionType_Run));
  prv_inc_time(10);
  prv_inc_steps_and_put_event(10);
  prv_put_bpm_event(100, HRMQuality_Good);

  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(steps, 10);
  cl_assert_equal_i(duration_s, 10);
  cl_assert_equal_i(bpm, 100);

  cl_assert(workout_service_pause_workout(true));

  // Get some new data but out stats shouldn't change (except HR)
  prv_inc_time(10);
  prv_inc_steps_and_put_event(10);
  prv_put_bpm_event(110, HRMQuality_Good);
  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(steps, 10);
  cl_assert_equal_i(duration_s, 10);
  cl_assert_equal_i(bpm, 110);

  // Get some more new data and out stats still shouldn't change (except HR)
  prv_inc_time(10);
  prv_inc_steps_and_put_event(10);
  prv_put_bpm_event(190, HRMQuality_Good);
  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(steps, 10);
  cl_assert_equal_i(duration_s, 10);
  cl_assert_equal_i(bpm, 190);

  // Resume the workout and get more data (it updates this time)
  cl_assert(workout_service_pause_workout(false));
  prv_inc_time(10);
  prv_inc_steps_and_put_event(10);
  prv_put_bpm_event(80, HRMQuality_Good);
  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(steps, 20);
  cl_assert_equal_i(duration_s, 20);
  cl_assert_equal_i(bpm, 80);

  cl_assert(workout_service_pause_workout(true));
  cl_assert(workout_service_pause_workout(true));
  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(steps, 20);
  cl_assert_equal_i(duration_s, 20);
  cl_assert_equal_i(bpm, 80);

  prv_inc_time(10);
  prv_inc_steps_and_put_event(10);
  prv_put_bpm_event(117, HRMQuality_Good);

  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(steps, 20);
  cl_assert_equal_i(duration_s, 20);
  cl_assert_equal_i(bpm, 117);

  cl_assert(workout_service_pause_workout(false));
  cl_assert(workout_service_pause_workout(false));

  prv_inc_time(10);
  prv_inc_steps_and_put_event(10);
  prv_put_bpm_event(113, HRMQuality_Good);

  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(steps, 30);
  cl_assert_equal_i(duration_s, 30);
  cl_assert_equal_i(bpm, 113);

  cl_assert(workout_service_stop_workout());
}

// ---------------------------------------------------------------------------------------
void test_workout_service__expire_hr_reading(void) {
  int32_t steps, duration_s, distance_m, bpm;
  HRZone hr_zone;

  cl_assert(workout_service_start_workout(ActivitySessionType_Run));
  prv_put_bpm_event(100, HRMQuality_Good);

  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(bpm, 100);

  // Time forward X seconds
  prv_inc_time(30);

  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(bpm, 100);

  // Time forward X seconds
  prv_inc_time(30);

  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  // HR Reading has expired. Should return 0
  cl_assert_equal_i(bpm, 0);

  cl_assert(workout_service_stop_workout());
}

// ---------------------------------------------------------------------------------------
void test_workout_service__receive_offwrist_reading(void) {
  int32_t steps, duration_s, distance_m, bpm;
  HRZone hr_zone;

  cl_assert(workout_service_start_workout(ActivitySessionType_Run));

  // Put a good quality reading. Verify it was accepted.
  prv_put_bpm_event(100, HRMQuality_Good);
  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(bpm, 100);

  // Put an OffWrist reading. Verify we received it.
  prv_put_bpm_event(50, HRMQuality_OffWrist);
  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(bpm, 0);
  cl_assert_equal_i(hr_zone, HRZone_Zone0);

  // Put a good quality reading. Verify it was accepted.
  prv_put_bpm_event(100, HRMQuality_Good);
  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(bpm, 100);

  cl_assert(workout_service_stop_workout());
}

// ---------------------------------------------------------------------------------------
void test_workout_service__working_out_past_midnight(void) {
  int32_t steps, duration_s, distance_m, bpm;
  HRZone hr_zone;

  cl_assert(workout_service_start_workout(ActivitySessionType_Run));

  // Start with only 10 steps and make sure it updates.
  prv_inc_steps_and_put_event(10);
  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(steps, 10);

  // Increment a lot so we can make sure that the wrap around of midnight works.
  prv_inc_steps_and_put_event(1000);
  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(steps, 1010);

  // Pretend we wrap around midnight. Midnight will reset the step count.
  s_total_step_count = 0;

  // Put some steps and make sure we don't delete or freak out, and append the new step count.
  prv_inc_steps_and_put_event(50);
  cl_assert(workout_service_get_current_workout_info(&steps, &duration_s, &distance_m,
                                                     &bpm, &hr_zone));
  cl_assert_equal_i(steps, 1060);
}

// ---------------------------------------------------------------------------------------
// Open the app and close the app. Make sure the HR monitor turns off instantly
void test_workout_service__app_open_close_no_workout(void) {
  // Put some time into the clock
  prv_inc_time(1 * SECONDS_PER_MINUTE);

  // Open the app, confirm that we are now subscribed with no end in sight
  workout_service_frontend_opened();
  cl_assert_equal_b(s_hrm_subscribed, true);
  cl_assert_equal_i(s_hrm_expiration, 0);

  workout_service_frontend_closed();
  cl_assert_equal_b(s_hrm_subscribed, false);
  cl_assert_equal_i(s_hrm_expiration, 0);
}

// ---------------------------------------------------------------------------------------
// Open the app, start a workout, close the app. Make sure the HR stays on for
// WORKOUT_ACTIVE_HR_SUBSCRIPTION_TS_EXPIRE
void test_workout_service__app_open_close_active_workout(void) {
  // Put some time into the clock
  prv_inc_time(1 * SECONDS_PER_MINUTE);

  // Open the app, confirm that we are now subscribed with no end in sight
  workout_service_frontend_opened();
  cl_assert_equal_b(s_hrm_subscribed, true);
  cl_assert_equal_i(s_hrm_expiration, 0);

  cl_assert(workout_service_start_workout(ActivitySessionType_Run));

  workout_service_frontend_closed();
  cl_assert_equal_b(s_hrm_subscribed, true);
  cl_assert_equal_i(s_hrm_expiration, SECONDS_PER_HOUR);
}

// ---------------------------------------------------------------------------------------
// Open the app, start a workout, 30s, stop the workout. Make sure the HR turns off instantly
// since workout wasn't valid (too short)
void test_workout_service__app_open_close_ended_invalid_workout(void) {
  // Put some time into the clock
  prv_inc_time(1 * SECONDS_PER_MINUTE);

  // Open the app, confirm that we are now subscribed with no end in sight
  workout_service_frontend_opened();
  cl_assert_equal_b(s_hrm_subscribed, true);
  cl_assert_equal_i(s_hrm_expiration, 0);

  prv_inc_time(1 * SECONDS_PER_MINUTE);
  cl_assert(workout_service_start_workout(ActivitySessionType_Run));
  // Workout of 30 seconds duration. Should be invalid, not keep HR on after leaving
  prv_inc_time(30);
  cl_assert(workout_service_stop_workout());

  workout_service_frontend_closed();
  cl_assert_equal_b(s_hrm_subscribed, false);
  cl_assert_equal_i(s_hrm_expiration, 0);
}

// ---------------------------------------------------------------------------------------
// Open the app, start a workout, 120s, stop the workout, 120s, close app.
// Make sure the HR stays on for WORKOUT_ENDED_HR_SUBSCRIPTION_TS_EXPIRE - 120s
void test_workout_service__app_open_close_valid_workout(void) {
  // Put some time into the clock
  prv_inc_time(1 * SECONDS_PER_MINUTE);

  // Open the app, confirm that we are now subscribed with no end in sight
  workout_service_frontend_opened();
  cl_assert_equal_b(s_hrm_subscribed, true);
  cl_assert_equal_i(s_hrm_expiration, 0);

  prv_inc_time(1 * SECONDS_PER_MINUTE);
  cl_assert(workout_service_start_workout(ActivitySessionType_Run));
  // Workout of 120 seconds duration. Should be valid
  prv_inc_time(2 * SECONDS_PER_MINUTE);
  cl_assert(workout_service_stop_workout());

  prv_inc_time(2 * SECONDS_PER_MINUTE);

  workout_service_frontend_closed();
  cl_assert_equal_b(s_hrm_subscribed, true);
  cl_assert_equal_i(s_hrm_expiration, 8 * SECONDS_PER_MINUTE);
}

// ---------------------------------------------------------------------------------------
// Same as above, but with background HR monitoring disabled. The user expects the sensor to run
// only during an activity, so closing the app after a workout must turn it off immediately rather
// than running the post-workout recovery window.
void test_workout_service__app_open_close_valid_workout_hrm_disabled(void) {
  s_hrm_measurement_interval = HRMonitoringInterval_Disabled;

  // Put some time into the clock
  prv_inc_time(1 * SECONDS_PER_MINUTE);

  // Open the app, confirm that we are now subscribed with no end in sight
  workout_service_frontend_opened();
  cl_assert_equal_b(s_hrm_subscribed, true);
  cl_assert_equal_i(s_hrm_expiration, 0);

  prv_inc_time(1 * SECONDS_PER_MINUTE);
  cl_assert(workout_service_start_workout(ActivitySessionType_Run));
  // Workout of 120 seconds duration. Should be valid
  prv_inc_time(2 * SECONDS_PER_MINUTE);
  cl_assert(workout_service_stop_workout());

  prv_inc_time(2 * SECONDS_PER_MINUTE);

  workout_service_frontend_closed();
  cl_assert_equal_b(s_hrm_subscribed, false);
  cl_assert_equal_i(s_hrm_expiration, 0);
}

// ---------------------------------------------------------------------------------------
// With background HR monitoring disabled, closing the app while a workout is still running must
// still keep the sensor on -- an ongoing workout is exactly the "during an activity" case.
void test_workout_service__app_open_close_active_workout_hrm_disabled(void) {
  s_hrm_measurement_interval = HRMonitoringInterval_Disabled;

  // Put some time into the clock
  prv_inc_time(1 * SECONDS_PER_MINUTE);

  workout_service_frontend_opened();
  cl_assert_equal_b(s_hrm_subscribed, true);
  cl_assert_equal_i(s_hrm_expiration, 0);

  cl_assert(workout_service_start_workout(ActivitySessionType_Run));

  workout_service_frontend_closed();
  cl_assert_equal_b(s_hrm_subscribed, true);
  cl_assert_equal_i(s_hrm_expiration, SECONDS_PER_HOUR);
}

// ---------------------------------------------------------------------------------------
// Open the app, start a workout, 120s, stop the workout, 20 min, close app.
// Make sure the HR turns off right after we leave the app.
void test_workout_service__app_open_wait_close_valid_workout(void) {
  // Put some time into the clock
  prv_inc_time(1 * SECONDS_PER_MINUTE);

  // Open the app, confirm that we are now subscribed with no end in sight
  workout_service_frontend_opened();
  cl_assert_equal_b(s_hrm_subscribed, true);
  cl_assert_equal_i(s_hrm_expiration, 0);

  prv_inc_time(1 * SECONDS_PER_MINUTE);
  cl_assert(workout_service_start_workout(ActivitySessionType_Run));
  // Workout of 120 seconds duration. Should be valid
  prv_inc_time(2 * SECONDS_PER_MINUTE);
  cl_assert(workout_service_stop_workout());

  // Wait 20 minutes. By this time, as soon as we leave the app, we should turn off the HR device.
  prv_inc_time(20 * SECONDS_PER_MINUTE);

  workout_service_frontend_closed();
  cl_assert_equal_b(s_hrm_subscribed, false);
  cl_assert_equal_i(s_hrm_expiration, 0);
}

// ---------------------------------------------------------------------------------------
// Stopping a workout should immediately apply a recovery-window expiration to the active HR
// subscription so sampling drops back to the user's preferred rate within a bounded time, even
// if the user lingers on the summary screen or the app exit path is otherwise delayed.
void test_workout_service__stop_workout_starts_recovery_timer(void) {
  prv_inc_time(1 * SECONDS_PER_MINUTE);

  workout_service_frontend_opened();
  cl_assert_equal_b(s_hrm_subscribed, true);
  cl_assert_equal_i(s_hrm_expiration, 0);

  cl_assert(workout_service_start_workout(ActivitySessionType_Run));
  prv_inc_time(2 * SECONDS_PER_MINUTE);
  cl_assert(workout_service_stop_workout());

  // The 1-second subscription should now have a 10-minute expiration so it doesn't run forever.
  cl_assert_equal_b(s_hrm_subscribed, true);
  cl_assert_equal_i(s_hrm_expiration, 10 * SECONDS_PER_MINUTE);
}

// ---------------------------------------------------------------------------------------
void test_workout_service__heart_rate_zone_time(void) {
  const int ZONE_0_HR = 100;
  const int ZONE_1_HR = 140;
  const int ZONE_2_HR = 160;
  const int ZONE_3_HR = 180;

  int32_t hr_zone_time_s[HRZoneCount];

  cl_assert(workout_service_start_workout(ActivitySessionType_Run));
  cl_assert(workout_service_get_current_workout_hr_zone_time(hr_zone_time_s));
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone0], 0);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone1], 0);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone2], 0);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone3], 0);

  prv_put_bpm_event(ZONE_1_HR, HRMQuality_Good);
  prv_inc_time(10);
  prv_put_bpm_event(ZONE_1_HR, HRMQuality_Good);
  cl_assert(workout_service_get_current_workout_hr_zone_time(hr_zone_time_s));
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone0], 0);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone1], 10);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone2], 0);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone3], 0);

  prv_inc_time(10);
  prv_put_bpm_event(ZONE_2_HR, HRMQuality_Good);
  cl_assert(workout_service_get_current_workout_hr_zone_time(hr_zone_time_s));
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone0], 0);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone1], 10);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone2], 10);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone3], 0);

  prv_inc_time(10);
  prv_put_bpm_event(ZONE_3_HR, HRMQuality_Good);
  cl_assert(workout_service_get_current_workout_hr_zone_time(hr_zone_time_s));
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone0], 0);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone1], 10);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone2], 10);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone3], 10);

  prv_inc_time(10);
  prv_put_bpm_event(ZONE_0_HR, HRMQuality_Good);
  cl_assert(workout_service_get_current_workout_hr_zone_time(hr_zone_time_s));
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone0], 10);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone1], 10);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone2], 10);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone3], 10);

  prv_inc_time(10);
  prv_put_bpm_event(ZONE_1_HR, HRMQuality_Good);
  workout_service_get_current_workout_hr_zone_time(hr_zone_time_s);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone0], 10);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone1], 20);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone2], 10);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone3], 10);

  // Pause the workout. Don't accumulate time in zones
  cl_assert(workout_service_pause_workout(true));

  prv_inc_time(10);
  prv_put_bpm_event(ZONE_3_HR, HRMQuality_Good);
  cl_assert(workout_service_get_current_workout_hr_zone_time(hr_zone_time_s));
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone0], 10);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone1], 20);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone2], 10);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone3], 10);

  prv_inc_time(10);
  prv_put_bpm_event(ZONE_1_HR, HRMQuality_Good);
  cl_assert(workout_service_get_current_workout_hr_zone_time(hr_zone_time_s));
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone0], 10);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone1], 20);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone2], 10);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone3], 10);

  // Unpause workout
  cl_assert(workout_service_pause_workout(false));
  prv_inc_time(10);
  prv_put_bpm_event(ZONE_3_HR, HRMQuality_Good);
  cl_assert(workout_service_get_current_workout_hr_zone_time(hr_zone_time_s));
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone0], 10);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone1], 20);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone2], 10);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone3], 20);

  prv_inc_time(10);
  prv_put_bpm_event(ZONE_2_HR, HRMQuality_Good);
  cl_assert(workout_service_get_current_workout_hr_zone_time(hr_zone_time_s));
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone0], 10);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone1], 20);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone2], 20);
  cl_assert_equal_i(hr_zone_time_s[HRZone_Zone3], 20);

  cl_assert(!workout_service_get_current_workout_hr_zone_time(NULL));
  cl_assert(workout_service_stop_workout());
  cl_assert(!workout_service_get_current_workout_hr_zone_time(hr_zone_time_s));
}

// ---------------------------------------------------------------------------------------
void test_workout_service__avg_hr(void) {
  int32_t avg_hr;
  cl_assert(!workout_service_get_avg_hr(&avg_hr));

  cl_assert(workout_service_start_workout(ActivitySessionType_Run));

  prv_put_bpm_event(140, HRMQuality_Good);
  prv_put_bpm_event(140, HRMQuality_Good);
  cl_assert(workout_service_get_avg_hr(&avg_hr));
  cl_assert_equal_i(avg_hr, 140);

  prv_put_bpm_event(160, HRMQuality_Good);
  prv_put_bpm_event(160, HRMQuality_Good);
  cl_assert(workout_service_get_avg_hr(&avg_hr));
  cl_assert_equal_i(avg_hr, 150);

  cl_assert(workout_service_pause_workout(true));

  prv_put_bpm_event(200, HRMQuality_Good);
  prv_put_bpm_event(200, HRMQuality_Good);
  cl_assert(workout_service_get_avg_hr(&avg_hr));
  cl_assert_equal_i(avg_hr, 150);

  cl_assert(workout_service_pause_workout(false));

  prv_put_bpm_event(180, HRMQuality_Good);
  prv_put_bpm_event(180, HRMQuality_Good);
  cl_assert(workout_service_get_avg_hr(&avg_hr));
  cl_assert_equal_i(avg_hr, 160);
}

// ---------------------------------------------------------------------------------------
// Open the app, start a workout, close app.
// 55 min - Make sure the notification was sent.
// 60 min - Make sure the workout was ended.
void test_workout_service__abandon_workout(void) {
  // Put some time into the clock
  prv_inc_time(1 * SECONDS_PER_MINUTE);

  // Open the app and wait a minute
  workout_service_frontend_opened();
  prv_inc_time(1 * SECONDS_PER_MINUTE);

  // Start workout and workout for 5 minutes
  cl_assert(workout_service_start_workout(ActivitySessionType_Run));
  prv_inc_time(5 * SECONDS_PER_MINUTE);

  // Close app and wait 30 minutes
  workout_service_frontend_closed();
  prv_inc_time(30 * SECONDS_PER_MINUTE);

  // Make sure notification is not sent yet and the workout is still ongoing
  cl_assert_equal_b(s_abandoned_workout_notification_sent, false);
  cl_assert_equal_b(workout_service_is_workout_ongoing(), true);

  // Wait 25 minutes, call evented timer callback and make sure the notification is sent
  prv_inc_time(25 * SECONDS_PER_MINUTE);
  prv_abandoned_notification_timer_callback(NULL);
  cl_assert_equal_b(s_abandoned_workout_notification_sent, true);

  // Wait 5 minutes, call evented timer callback and make sure the workout was ended
  prv_inc_time(5 * SECONDS_PER_MINUTE);
  prv_abandon_workout_timer_callback(NULL);
  cl_assert_equal_b(workout_service_is_workout_ongoing(), false);
}
