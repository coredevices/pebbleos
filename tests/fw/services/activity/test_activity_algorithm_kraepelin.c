/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/accel_service.h"
#include "applib/data_logging.h"
#include <pbl/drivers/ambient_light.h>
#include <pbl/drivers/rtc.h>
#include "pbl/services/regular_timer.h"
#include "pbl/services/battery/battery_state.h"
#include "pbl/services/activity/activity.h"
#include "pbl/services/activity/activity_private.h"
#include "pbl/services/activity/activity_algorithm.h"
#include "pbl/services/activity/kraepelin/activity_algorithm_kraepelin.h"
#include "pbl/services/activity/kraepelin/kraepelin_algorithm.h"
#include "pbl/services/data_logging/data_logging_service.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/settings/settings_file.h"
#include "pbl/services/system_task.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"

#include <stdint.h>
#include <string.h>
#include <applib/health_service.h>
#include <services/activity/kraepelin/activity_algorithm_kraepelin.h>

#include "clar.h"

// Stubs
#include "stubs_analytics.h"
#include "stubs_freertos.h"
#include "stubs_hexdump.h"
#include "stubs_hr_util.h"
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_prompt.h"
#include "stubs_sleep.h"
#include "stubs_task_watchdog.h"

// Fakes
#include "fake_accel_service.h"
#include "fake_new_timer.h"
#include "fake_rtc.h"
#include "fake_spi_flash.h"
#include "fake_system_task.h"


#define ASSERT_EQUAL_I(i1,i2,file,line) \
        clar__assert_equal_i((i1),(i2),file,line,#i1 " != " #i2, 1)

// Globals
AccelSamplingRate s_sample_rate;

static bool s_dls_created;
static DataLoggingSession *s_dls_session = (DataLoggingSession *)1;

// Logged items
static bool s_capture_dls_records = true;
static int s_num_dls_records;
static AlgMinuteDLSRecord s_dls_records[100];

// Which step count to return from kalg_analyze_samples()
static uint16_t s_alg_next_steps;

// Which vmc and orientation to return from kalg_minute_stats
static uint16_t s_alg_next_vmc;
static uint8_t s_alg_next_orientation;
static uint8_t s_alg_next_light;
static bool s_alg_next_plugged_in;

static struct tm s_start_time_tm = {
  .tm_hour = 17,
  .tm_mday = 1,
  .tm_mon = 0,
  .tm_year = 115
};


// ============================================================================================
// Misc stubs
uint32_t light_get_ambient_lux(void) {
  return s_alg_next_light << 4;
}

AmbientLightLevel ambient_light_level_to_enum(uint32_t light_level) {
  // Just return a predictable result to validate the unit tests
  return (light_level / ALG_RAW_LIGHT_SENSOR_DIVIDE_BY) % AMBIENT_LIGHT_LEVEL_ENUM_COUNT;
}

BatteryChargeState battery_get_charge_state(void) {
  BatteryChargeState state = {
    .charge_percent = 50,
    .is_charging = s_alg_next_plugged_in,
    .is_plugged = s_alg_next_plugged_in,
  };
  return state;
}

void kalg_enable_activity_tracking(KAlgState *kalg_state, bool enable) {}

bool activity_tracking_on(void) {
  return true;
}

// ------------------------------------------------------------------------------------
// Return true if the given activity type is a sleep activity
bool activity_sessions_prv_is_sleep_activity(ActivitySessionType activity_type) {
  switch (activity_type) {
    case ActivitySessionType_Sleep:
    case ActivitySessionType_RestfulSleep:
    case ActivitySessionType_Nap:
    case ActivitySessionType_RestfulNap:
      return true;
    case ActivitySessionType_Walk:
    case ActivitySessionType_Run:
    case ActivitySessionType_Open:
      return false;
    case ActivitySessionType_None:
    case ActivitySessionTypeCount:
      break;
  }
  WTF;
}

// ------------------------------------------------------------------------------------
uint16_t s_activity_sessions_count;
ActivitySession s_activity_sessions[ACTIVITY_MAX_ACTIVITY_SESSIONS_COUNT];
void activity_sessions_prv_add_activity_session(ActivitySession *session) {
  // If this is a duplicate activity, ignore it
  ActivitySession *stored_session = s_activity_sessions;
  for (uint16_t i = 0; i < s_activity_sessions_count; i++, stored_session++) {
    if ((session->type == stored_session->type)
        && (session->start_utc == stored_session->start_utc)) {
      return;
    }
  }

  // If no more room, fail
  if (s_activity_sessions_count >= ACTIVITY_MAX_ACTIVITY_SESSIONS_COUNT) {
    PBL_LOG_WRN("No more room for additional activities");
    return;
  }

  // Add this activity in
  s_activity_sessions[s_activity_sessions_count++] = *session;
}

// ------------------------------------------------------------------------------------
void activity_sessions_prv_delete_activity_session(ActivitySession *session) {
}


// =============================================================================================
// Data logging stubs
DataLoggingResult dls_log(DataLoggingSession *logging_session, const void *data,
                          uint32_t num_items) {
  if (!s_capture_dls_records) {
    return DATA_LOGGING_SUCCESS;
  }
  cl_assert(s_dls_created);
  cl_assert_equal_p(logging_session, s_dls_session);

  AlgMinuteDLSRecord *records = (AlgMinuteDLSRecord *)data;
  for (int i = 0; i < num_items; i++) {
    cl_assert(s_num_dls_records < ARRAY_LENGTH(s_dls_records));
    s_dls_records[s_num_dls_records++] = records[i];
  }

  return DATA_LOGGING_SUCCESS;
}

DataLoggingSession *dls_create(uint32_t tag, DataLoggingItemType item_type, uint16_t item_size,
                               bool buffered, bool resume, const Uuid *uuid) {
  s_dls_created = true;
  cl_assert_equal_i(item_size, sizeof(AlgMinuteDLSRecord));

  return s_dls_session;
}

void dls_send_all_sessions(void) {
}


// ============================================================================================
// Activity service stubs
// --------------------------------------------------------------------------------------------
// Values to return from activity_private_get_.*()
static uint32_t s_activity_next_distance_mm;
static uint32_t s_activity_next_active_calories;
static uint32_t s_activity_next_resting_calories;
static uint32_t s_activity_next_heart_rate_bpm;
static uint32_t s_activity_next_heart_rate_zone;
static uint32_t s_activity_next_heart_rate_heart_rate_total_weight_x100;

uint32_t activity_metrics_prv_get_steps(void) {
  return 0;
}


uint32_t activity_metrics_prv_get_distance_mm(void) {
  return s_activity_next_distance_mm;
}


// --------------------------------------------------------------------------------------------
uint32_t activity_metrics_prv_get_resting_calories(void) {
  return s_activity_next_resting_calories;
}


// --------------------------------------------------------------------------------------------
uint32_t activity_metrics_prv_get_active_calories(void) {
  return s_activity_next_active_calories;
}

HRZone activity_metrics_prv_get_hr_zone(void) {
  return s_activity_next_heart_rate_zone;
}

void activity_metrics_prv_get_median_hr_bpm(int32_t *median, int32_t *total_weight) {
  if (median) {
    *median = s_activity_next_heart_rate_bpm;
  }
  if (total_weight) {
    *total_weight = s_activity_next_heart_rate_heart_rate_total_weight_x100;
  }

}

void activity_metrics_prv_reset_hr_stats(void) {
  s_activity_next_heart_rate_bpm = 0;
  s_activity_next_heart_rate_zone = 0;
}

void activity_metrics_prv_set_hrm_worn_status(time_t now_utc, bool is_offwrist) {
}

bool activity_metrics_prv_is_hrm_offwrist(time_t now_utc) {
  return false;
}


// =============================================================================================
// Algorithm stubs
uint32_t kalg_state_size(void) {
  return 1;
}

bool kalg_init(KAlgState *state, KAlgStatsCallback stats_cb) {
  return true;
}

void kalg_deinit(KAlgState *state) {}

uint32_t kalg_analyze_samples(KAlgState *state, AccelRawData *data, uint32_t num_samples,
                              uint32_t *consumed_samples) {
  *consumed_samples = 0;
  return s_alg_next_steps;
}

void kalg_minute_stats(KAlgState *state, uint16_t *vmc, uint8_t *orientation, bool *still) {
  *vmc = s_alg_next_vmc;
  *orientation = s_alg_next_orientation;
  *still = false;
}

void kalg_set_weight(KAlgState *state, uint32_t grams) {
}

void kalg_activities_update(KAlgState *state, time_t utc_now, uint16_t steps, uint16_t vmc,
                            uint8_t orientation, bool definitely_not_worn,
                            uint32_t resting_calories,
                            uint32_t active_calories, uint32_t distance_mm, bool shutting_down,
                            KAlgActivitySessionCallback sessions_cb, void *context) {
}

time_t kalg_activity_last_processed_time(KAlgState *state, KAlgActivityType activity) {
  return rtc_get_time();
}

// Set these to simulate a sleep session (that should result in zeroing out any steps taken)
static time_t s_kalg_sleep_start_utc;
static uint16_t s_kalg_sleep_m;

void kalg_get_sleep_stats(KAlgState *alg_state, KAlgOngoingSleepStats *stats) {
  time_t now = rtc_get_time();
  if (s_kalg_sleep_start_utc == 0 || now < s_kalg_sleep_start_utc + SECONDS_PER_HOUR) {
    // We are before the requested sleep time
    *stats = (KAlgOngoingSleepStats) { };
  } else {
    // We are somewhere after the start of sleep
    time_t sleep_end = s_kalg_sleep_start_utc + s_kalg_sleep_m * SECONDS_PER_MINUTE;
    if (now < sleep_end + KALG_MAX_UNCERTAIN_SLEEP_M) {
      // Still haven't detected the end of sleep, the last KALG_MAX_UNCERTAIN_SLEEP_M minutes are
      // uncertain
      *stats = (KAlgOngoingSleepStats) {
        .sleep_start_utc = s_kalg_sleep_start_utc,
        .sleep_len_m = (now - s_kalg_sleep_start_utc) / SECONDS_PER_MINUTE
                       - KALG_MAX_UNCERTAIN_SLEEP_M,
        .uncertain_start_utc = now - KALG_MAX_UNCERTAIN_SLEEP_M * SECONDS_PER_MINUTE,
      };
    } else {
      // The sleep was in the past and has ended
      *stats = (KAlgOngoingSleepStats) {
        .sleep_start_utc = s_kalg_sleep_start_utc,
        .sleep_len_m = s_kalg_sleep_m,
        .uncertain_start_utc = 0,
      };
    }
  }
}


// --------------------------------------------------------------------------------------------
// Create sample data for testing sleep and data logging
static void prv_create_test_data(uint32_t num_minutes, AlgMinuteDLSSample *minute_data) {
  uint16_t next_vmc = 0;
  uint8_t next_orient = 1;
  uint8_t next_light = 2;
  uint16_t next_active_calories = 3;
  uint16_t next_resting_calories = 4;
  uint16_t next_distance_cm = 5;
  uint8_t next_heart_rate_bpm = 6;
  uint8_t next_heart_rate_heart_rate_total_weight_x100 = 7;
  uint8_t next_heart_rate_zone = 8;

  bool next_plugged_in = false;
  for (int i = 0; i < num_minutes; i++) {
    minute_data[i] = (AlgMinuteDLSSample) { };
    minute_data[i].base.steps = i;
    minute_data[i].base.vmc = next_vmc++;
    if (next_vmc == 65533) {
      // Make sure combinations of vmc/orient are mostly unique, so don't wrap at the same
      // module 256 boundary.
      next_vmc = 0;
    }
    minute_data[i].base.orientation = next_orient++;
    minute_data[i].base.light = next_light++;
    minute_data[i].base.plugged_in = next_plugged_in;
    next_plugged_in = !next_plugged_in;

    minute_data[i].active_calories = next_active_calories++;
    minute_data[i].resting_calories = next_resting_calories++;
    minute_data[i].distance_cm = next_distance_cm++;
    minute_data[i].base.active = (minute_data[i].base.steps >= ACTIVITY_ACTIVE_MINUTE_MIN_STEPS)
                                 ? 1 : 0;
    minute_data[i].heart_rate_bpm = next_heart_rate_bpm++;
    minute_data[i].heart_rate_total_weight_x100 = next_heart_rate_heart_rate_total_weight_x100++;
    minute_data[i].heart_rate_zone = next_heart_rate_zone++;
  }
}


// --------------------------------------------------------------------------------------------
// Feed in sleep data
static void prv_feed_minute_data(uint32_t num_minutes, AlgMinuteDLSSample *minute_data,
                                 bool simulate_bg_delays) {
  // Call the minute handler, which computes the minute stats and saves them to data logging
  // as well as the sleep PFS file.
  for (int i = 0; i < num_minutes; i++) {
    fake_rtc_increment_time(SECONDS_PER_MINUTE);
    s_alg_next_steps = minute_data[i].base.steps;
    AccelRawData samples[100] = { };
    uint64_t timestamp = 0;
    // Calling activity_algorithm_handle_accel() on our stub algorithm gives it the step
    // counts for this minute
    activity_algorithm_handle_accel(samples, s_sample_rate, timestamp);

    // Are we simulating delays in KernelBG processing?
    int delay = 0;
    if (simulate_bg_delays) {
      delay = (((i / ALG_MINUTES_PER_FILE_RECORD) + 1) % 30);
      rtc_set_time(rtc_get_time() + delay);
    }
    s_alg_next_vmc = minute_data[i].base.vmc;
    s_alg_next_orientation = minute_data[i].base.orientation;
    s_alg_next_light = minute_data[i].base.light;
    s_alg_next_plugged_in = minute_data[i].base.plugged_in;
    s_activity_next_distance_mm += minute_data[i].distance_cm * 10;
    s_activity_next_resting_calories += minute_data[i].resting_calories;
    s_activity_next_active_calories += minute_data[i].active_calories;
    s_activity_next_heart_rate_bpm = minute_data[i].heart_rate_bpm;
    s_activity_next_heart_rate_heart_rate_total_weight_x100 = minute_data[i].heart_rate_total_weight_x100;
    s_activity_next_heart_rate_zone = minute_data[i].heart_rate_zone;
    AlgMinuteRecord minute_record = {};
    activity_algorithm_minute_handler(rtc_get_time(), &minute_record);
    if (simulate_bg_delays) {
      rtc_set_time(rtc_get_time() - delay);
    }
  }
}


// =============================================================================================
// Start of unit tests
void test_activity_algorithm_kraepelin__initialize(void) {
  time_t utc_sec = mktime(&s_start_time_tm);
  fake_rtc_init(100 /*initial_ticks*/, utc_sec);

  fake_spi_flash_init(0, 0x1000000);
  pfs_init(false);
  pfs_format(false);

  // Init the algorithm
  s_activity_next_resting_calories = 0;
  s_activity_next_distance_mm = 0;
  s_activity_next_active_calories = 0;
  s_activity_next_heart_rate_bpm = 0;
  s_activity_next_heart_rate_zone = 0;
  s_kalg_sleep_start_utc = 0;
  s_kalg_sleep_m = 0;
  activity_algorithm_init(&s_sample_rate);
}


// ---------------------------------------------------------------------------------------
void test_activity_algorithm_kraepelin__cleanup(void) {
  fake_system_task_callbacks_invoke_pending();
  activity_algorithm_deinit();
}


// ---------------------------------------------------------------------------------------
// Test to make sure that the minute data gets sent to data logging correctly
void test_activity_algorithm_kraepelin__data_logging_test(void) {
  const int k_num_records = 2;
  const int num_minutes = k_num_records * ALG_MINUTES_PER_DLS_RECORD;

  // The test data
  AlgMinuteDLSSample minute_data[num_minutes];
  prv_create_test_data(num_minutes, minute_data);

  // Call the minute handler, which computes the minute stats and saves them to data logging
  // as well as the minute data settings file.
  prv_feed_minute_data(num_minutes, minute_data, false /*simulate_bg_delays*/);

  // Make sure the correct data got saved to data logging
  cl_assert_equal_i(s_num_dls_records, 2);
  for (int j = 0; j < k_num_records; j++) {
    cl_assert_equal_i(s_dls_records[j].hdr.version, ALG_DLS_MINUTES_RECORD_VERSION);
    for (int i = 0; i < ALG_MINUTES_PER_DLS_RECORD; i++) {
      cl_assert_equal_m(&s_dls_records[j].samples[i],
                        &minute_data[(j * ALG_MINUTES_PER_DLS_RECORD) + i],
                        sizeof(minute_data[i]));
    }
  }
}


// ------------------------------------------------------------------------------------
static void prv_assert_minute_data(HealthMinuteData *actual, AlgMinuteDLSSample *expected) {
  cl_assert_equal_i(actual->steps, expected->base.steps);
  cl_assert_equal_i(actual->orientation, expected->base.orientation);
  cl_assert_equal_i(actual->vmc, expected->base.vmc);
  cl_assert_equal_i(actual->light, ambient_light_level_to_enum(
    expected->base.light * ALG_RAW_LIGHT_SENSOR_DIVIDE_BY));
  cl_assert_equal_i(actual->heart_rate_bpm, expected->heart_rate_bpm);

}

// ---------------------------------------------------------------------------------------
// Test to make sure that when we re-boot we correctly get the saved minute data
void test_activity_algorithm_kraepelin__minute_data_after_boot(void) {
  const int num_minutes = 4 * MINUTES_PER_HOUR;
  time_t start_utc = rtc_get_time();

  // The test data
  AlgMinuteDLSSample minute_data[num_minutes];
  prv_create_test_data(num_minutes, minute_data);

  // Write first half of the data
  prv_feed_minute_data(num_minutes / 2, minute_data, false /*simulate_bg_delays*/);

  // Now, simulate a reboot, re-initialize of the algorithm. This will trigger a re-read
  // of the sleep data file
  activity_algorithm_deinit();
  activity_algorithm_init(&s_sample_rate);

  // Write the rest of the data
  int start_idx = num_minutes / 2;
  prv_feed_minute_data(num_minutes / 2, minute_data + start_idx, false /*simulate_bg_delays*/);

  // Retrieve all the minute data and verify the contents
  HealthMinuteData retrieve[num_minutes];
  uint32_t num_records = num_minutes;
  time_t start = start_utc;
  activity_algorithm_get_minute_history(retrieve, &num_records, &start);
  cl_assert_equal_i(num_records, num_minutes);
  cl_assert_equal_i(start, start_utc);
  for (int i = 0; i < num_minutes; i++) {
    prv_assert_minute_data(&retrieve[i], &minute_data[i]);
  }
}


// ---------------------------------------------------------------------------------------
// Test to make sure that the minute data file gets compacted correctly. If we write more than
// ALG_MINUTE_DATA_FILE_LEN worth of data to the sleep file, it's size should be capped at
// ALG_MINUTE_DATA_FILE_LEN and we should be able to successfully read back the most recent
// data we wrote.
void test_activity_algorithm_kraepelin__sleep_data_compaction_test(void) {
  const int num_minutes = ALG_SLEEP_HISTORY_HOURS_FOR_TODAY * MINUTES_PER_HOUR;

  // The test data
  AlgMinuteDLSSample minute_data[num_minutes];
  prv_create_test_data(num_minutes, minute_data);

  // Fill with garbage for more than ALG_MINUTE_DATA_FILE_LEN to force us to
  // chop off old data.
  s_capture_dls_records = false;
  uint32_t max_minutes = ALG_MINUTE_DATA_FILE_LEN * 3 / 2 / sizeof(AlgMinuteFileSample);
  // Make sure it's a multiple of ALG_MINUTES_PER_RECORD
  max_minutes = (max_minutes / ALG_MINUTES_PER_FILE_RECORD) * ALG_MINUTES_PER_FILE_RECORD;
  for (int i = 0; i < max_minutes; i++) {
    fake_rtc_increment_time(SECONDS_PER_MINUTE);
    s_alg_next_steps = 0x1234;
    AccelRawData samples[100] = { };
    uint64_t timestamp = 0;
    activity_algorithm_handle_accel(samples, s_sample_rate, timestamp);

    s_alg_next_vmc = 0x11;
    s_alg_next_orientation = 0x22;
    AlgMinuteRecord minute_record = {};
    activity_algorithm_minute_handler(rtc_get_time(), &minute_record);
  }

  // Get the size of the sleep data and make sure it is within the expected range
  uint32_t num_records;
  uint32_t data_bytes;
  uint32_t minutes;
  bool success = activity_algorithm_minute_file_info(false /*compact_first*/, &num_records,
                                                     &data_bytes, &minutes);
  cl_assert(success);
  cl_assert(data_bytes < ALG_MINUTE_DATA_FILE_LEN && data_bytes > ALG_MINUTE_DATA_FILE_LEN / 2);
  cl_assert(minutes < ALG_MINUTE_FILE_MAX_ENTRIES * ALG_MINUTES_PER_FILE_RECORD);
  cl_assert(minutes > ALG_MINUTE_FILE_MAX_ENTRIES * ALG_MINUTES_PER_FILE_RECORD / 2);


  // Now, put in our expected data
  // Call the minute handler, which computes the minute stats and saves them to data logging
  // as well as the sleep PFS file.
  time_t start_of_data_utc = rtc_get_time();
  prv_feed_minute_data(num_minutes, minute_data, false /*simulate_bg_delays*/);

  // Retrieve the minute data now
  HealthMinuteData retrieve[num_minutes];
  num_records = num_minutes;
  time_t start = start_of_data_utc;  // starting just past the minute to test that &start gets updated
  activity_algorithm_get_minute_history(retrieve, &num_records, &start);
  cl_assert_equal_i(num_records, num_minutes);
  cl_assert_equal_i(start, start_of_data_utc);
  for (int i = 0; i < num_minutes; i++) {
    prv_assert_minute_data(&retrieve[i], &minute_data[i]);
  }
}


// ---------------------------------------------------------------------------------------
// Test that the call to retrieve minute history from flash works correctly
void test_activity_algorithm_kraepelin__get_flash_minute_history(void) {
  const int num_minutes = 4 * MINUTES_PER_HOUR;
  time_t start_utc = rtc_get_time();

  // Let's start time not on a 15 minute boundary to aggravate the get_minute logic
  start_utc += 7 * SECONDS_PER_MINUTE;
  rtc_set_time(start_utc);

  // The test data
  AlgMinuteDLSSample minute_data[num_minutes];
  prv_create_test_data(num_minutes, minute_data);

  // Call the minute handler, which computes the minute stats and saves them to data logging
  // as well as to the sleep PFS file.
  prv_feed_minute_data(num_minutes, minute_data, false /*simulate_bg_delays*/);

  // Retrieve all of the minute data at once
  HealthMinuteData retrieve[num_minutes * 2];
  uint32_t num_records = num_minutes;
  time_t start = start_utc + 5;  // starting just past the minute to test that &start gets updated
  activity_algorithm_get_minute_history(retrieve, &num_records, &start);
  cl_assert_equal_i(num_records, num_minutes);
  cl_assert_equal_i(start, start_utc);
  for (int i = 0; i < num_minutes; i++) {
    prv_assert_minute_data(&retrieve[i], &minute_data[i]);
  }

  // Retrieve, trying to start from a lot farther back, it should return the UTC of the first
  // record available. Also ask for more than what is available
  num_records = num_minutes * 2;
  start = start_utc - SECONDS_PER_DAY;
  activity_algorithm_get_minute_history(retrieve, &num_records, &start);
  cl_assert_equal_i(num_records, num_minutes);
  cl_assert_equal_i(start, start_utc);
  for (int i = 0; i < num_minutes; i++) {
    prv_assert_minute_data(&retrieve[i], &minute_data[i]);
  }

  // Retrieve a little (10 minutes) at a time
  int num_records_left = num_minutes;
  int num_records_found = 0;
  start = start_utc;
  while (num_records_left) {
    uint32_t chunk;
    chunk = MIN(10, num_records_left);
    time_t first_ts = start;
    activity_algorithm_get_minute_history(&retrieve[num_records_found], &chunk, &first_ts);
    cl_assert_equal_i(start, first_ts);
    num_records_left -= chunk;
    num_records_found += chunk;
    start += chunk * SECONDS_PER_MINUTE;
  }
  cl_assert_equal_i(num_records_found, num_minutes);
  for (int i = 0; i < num_minutes; i++) {
    prv_assert_minute_data(&retrieve[i], &minute_data[i]);
  }
}

// ---------------------------------------------------------------------------------------
// Test that retrieving the most recent minute history works correctly. This test insures that
// we correctly include the minute history that has not yet been saved to flash
void test_activity_algorithm_kraepelin__get_ram_minute_history(void) {
  const int num_minutes = 1 * MINUTES_PER_HOUR;
  time_t start_utc = rtc_get_time();

  // Let's start time not on a 15 minute boundary to aggravate the get_minute logic
  start_utc += 7 * SECONDS_PER_MINUTE;
  rtc_set_time(start_utc);

  // The test data
  AlgMinuteDLSSample minute_data[num_minutes];
  prv_create_test_data(num_minutes, minute_data);

  // Call the minute handler to feed in enough data to write to flash. This computes the minute
  // stats and saves them to data logging as well as to the sleep PFS file.
  prv_feed_minute_data(ALG_MINUTES_PER_FILE_RECORD, minute_data, false /*simulate_bg_delays*/);
  uint32_t next_write_minute_idx = ALG_MINUTES_PER_FILE_RECORD;

  // Once a minute, retrieve the last ALG_MINUTES_PER_RECORD minutes of data. We should
  // get ALG_MINUTES_PER_RECORD records each time. We know that the activity algorithm code only
  // writes a new minute data record to flash once every ALG_MINUTES_PER_RECORD minutes, but
  // the records that are not yet saved to flash should be correctly retrieved from RAM.
  time_t oldest_to_fetch = rtc_get_time() - (ALG_MINUTES_PER_FILE_RECORD * SECONDS_PER_MINUTE);
  uint32_t next_read_minute_idx = 0;
  for (int i = 0; i < ALG_MINUTES_PER_FILE_RECORD; i++, oldest_to_fetch += SECONDS_PER_MINUTE,
    next_read_minute_idx++, next_write_minute_idx++) {

    // Ask for the last ALG_MINUTES_PER_RECORD minutes of data
    uint32_t num_records = ALG_MINUTES_PER_FILE_RECORD;
    time_t start = oldest_to_fetch;
    HealthMinuteData received_records[ALG_MINUTES_PER_FILE_RECORD];
    activity_algorithm_get_minute_history(received_records, &num_records, &start);

    cl_assert_equal_i(num_records, ALG_MINUTES_PER_FILE_RECORD);
    cl_assert_equal_i(start, oldest_to_fetch);

    printf("\nReceived %d minute records", (int)num_records);
    for (int j = 0; j < num_records; j++) {
      printf("\nRecord:%d, steps: %d", j, (int)received_records[j].steps);
    }

    // Verify the contents of the records
    for (int j = 0; j < num_records; j++) {
      prv_assert_minute_data(&received_records[j], &minute_data[next_read_minute_idx + j]);
    }

    // Advance another minute. It doesn't matter what data we feed in
    prv_feed_minute_data(1, minute_data + next_write_minute_idx, false /*simulate_bg_delays*/);
  }

  // Let's add data for a partial minute and make sure that gets returned
  const int exp_steps = 23;
  oldest_to_fetch = rtc_get_time() - SECONDS_PER_MINUTE;
  fake_rtc_increment_time(30); // 30 seconds
  s_alg_next_steps = exp_steps;
  AccelRawData samples[100] = { };
  uint64_t timestamp = 0;
  // Calling activity_algorithm_handle_accel() on our stub algorithm registers the new steps
  // counts for this minute
  activity_algorithm_handle_accel(samples, s_sample_rate, timestamp);

  // Fetch the last whole minute plus this partial minute
  time_t start = oldest_to_fetch;
  uint32_t num_records = 2;
  HealthMinuteData received_records[num_records];
  activity_algorithm_get_minute_history(received_records, &num_records, &start);
  cl_assert_equal_i(num_records, 2);
  cl_assert_equal_i(start, oldest_to_fetch);
  prv_assert_minute_data(&received_records[0],
                         &minute_data[next_read_minute_idx + ALG_MINUTES_PER_FILE_RECORD - 1]);
  cl_assert_equal_i(received_records[1].steps, exp_steps);
}


// ---------------------------------------------------------------------------------------
// Test the logic that detects naps. This logic is performed by the
// prv_sleep_sessions_post_process() method.
void test_activity_algorithm_kraepelin__sleep_post_process(void) {
  // NOTE: All tests by default start at 5pm. Let's advance time to 9pm to give us more
  // time to test the various nap scenarios
  time_t now_utc = rtc_get_time();
  now_utc += 4 * SECONDS_PER_HOUR;
  rtc_set_time(now_utc);
  time_t start_of_today = time_util_get_midnight_of(now_utc);

  { // Create a 2 hour session at 1pm ==> should be a nap
    ActivitySession sessions[] = {
      {
        .start_utc = start_of_today + (13 * SECONDS_PER_HOUR),  // 1pm
        .length_min = 2 * MINUTES_PER_HOUR,
        .type = ActivitySessionType_Sleep,
      },
      {
        .start_utc = start_of_today + (13 * SECONDS_PER_HOUR) + (15 * SECONDS_PER_MINUTE), // 1:15pm
        .length_min = 20,
        .type = ActivitySessionType_RestfulSleep,
      },
    };

    uint16_t session_entries = ARRAY_LENGTH(sessions);
    activity_algorithm_post_process_sleep_sessions(session_entries, sessions);
    cl_assert_equal_i(sessions[0].type, ActivitySessionType_Nap);
    cl_assert_equal_i(sessions[1].type, ActivitySessionType_RestfulNap);
  }

  { // Create a 4 hour session at 1pm ==> should be regular sleep
    ActivitySession sessions[] = {
      {
        .start_utc = start_of_today + (13 * (SECONDS_PER_HOUR)),  // 1pm
        .length_min = 4 * MINUTES_PER_HOUR,
        .type = ActivitySessionType_Sleep,
      },
    };

    uint16_t session_entries = ARRAY_LENGTH(sessions);
    activity_algorithm_post_process_sleep_sessions(session_entries, sessions);
    cl_assert_equal_i(sessions[0].type, ActivitySessionType_Sleep);
  }

  { // Create two 2 hour sessions, they should both be considered as separate naps
    ActivitySession sessions[] = {
      {
        .start_utc = start_of_today + (13 * SECONDS_PER_HOUR),  // 1pm
        .length_min = 2 * MINUTES_PER_HOUR,
        .type = ActivitySessionType_Sleep,
      },
      {
        .start_utc = start_of_today + (17 * SECONDS_PER_HOUR), // 5pm
        .length_min = 2 * MINUTES_PER_HOUR,
        .type = ActivitySessionType_Sleep,
      },
    };

    uint16_t session_entries = ARRAY_LENGTH(sessions);
    activity_algorithm_post_process_sleep_sessions(session_entries, sessions);
    cl_assert_equal_i(sessions[0].type, ActivitySessionType_Nap);
    cl_assert_equal_i(sessions[1].type, ActivitySessionType_Nap);
  }

  { // Create a 2 hour session that ends after 9pm ==> should be regular sleep
    ActivitySession sessions[] = {
      {
        .start_utc = start_of_today + (20 * SECONDS_PER_HOUR),  // 8pm
        .length_min = 2 * MINUTES_PER_HOUR,
        .type = ActivitySessionType_Sleep,
      },
    };

    uint16_t session_entries = ARRAY_LENGTH(sessions);
    activity_algorithm_post_process_sleep_sessions(session_entries, sessions);
    cl_assert_equal_i(sessions[0].type, ActivitySessionType_Sleep);
  }

  { // Create a 2 hour session that starts before 12pm ==> should be regular sleep
    ActivitySession sessions[] = {
      {
        .start_utc = start_of_today + (11 * SECONDS_PER_HOUR),  // 11am
        .length_min = 2 * MINUTES_PER_HOUR,
        .type = ActivitySessionType_Sleep,
      },
    };

    uint16_t session_entries = ARRAY_LENGTH(sessions);
    activity_algorithm_post_process_sleep_sessions(session_entries, sessions);
    cl_assert_equal_i(sessions[0].type, ActivitySessionType_Sleep);
  }

  { // Create a 2 hour session that is still on-going - should register as normal sleep
    time_t sleep_start_utc = now_utc - (2 * SECONDS_PER_HOUR);
    ActivitySession sessions[] = {
      {
        .start_utc = sleep_start_utc,
        .length_min = 2 * MINUTES_PER_HOUR,
        .type = ActivitySessionType_Sleep,
        .ongoing = true,
      },
      {
        .start_utc = sleep_start_utc + (15 * SECONDS_PER_MINUTE),
        .length_min = 20,
        .type = ActivitySessionType_RestfulSleep,
        .ongoing = true,
      },
    };
    uint16_t session_entries = ARRAY_LENGTH(sessions);
    activity_algorithm_post_process_sleep_sessions(session_entries, sessions);
    cl_assert_equal_i(sessions[0].type, ActivitySessionType_Sleep);
    cl_assert_equal_i(sessions[1].type, ActivitySessionType_RestfulSleep);
  }

  { // Create a 2h 39m  session that starts at 11:59pm ==> should be regular sleep
    ActivitySession sessions[] = {
      {
        .start_utc = start_of_today - (1 * SECONDS_PER_MINUTE),  // 11:59pm
        .length_min = (2 * MINUTES_PER_HOUR) + 39,
        .type = ActivitySessionType_Sleep,
      },
    };

    uint16_t session_entries = ARRAY_LENGTH(sessions);
    activity_algorithm_post_process_sleep_sessions(session_entries, sessions);
    cl_assert_equal_i(sessions[0].type, ActivitySessionType_Sleep);
  }

}

// ---------------------------------------------------------------------------------------
// Test to make sure we don't get steps counted while sleeping
void test_activity_algorithm_kraepelin__steps_during_sleep(void) {
  const int num_minutes = 120;

  // The test data
  AlgMinuteDLSSample minute_data[num_minutes];
  prv_create_test_data(num_minutes, minute_data);

  // Zero out the first hour of data. The sleep algorithm takes an hour to figure out that
  // you are sleeping, so it has no chance of zeroing out all the steps in that first hour.
  for (int i = 0; i < 60; i++) {
    minute_data[i].base.steps = 0;
  }

  // -------------------------------------------------------------------------------
  // Set to not sleeping
  s_kalg_sleep_start_utc = 0;
  s_kalg_sleep_m = 0;

  activity_algorithm_metrics_changed_notification();
  uint32_t steps_awake_60m;
  uint32_t steps_awake_100m;
  uint32_t steps_awake_120m;

  // Call the minute handler, which should zero out steps that occur while sleeping
  prv_feed_minute_data(60, &minute_data[0], false /*simulate_bg_delays*/);
  activity_algorithm_get_steps(&steps_awake_60m);

  prv_feed_minute_data(40, &minute_data[60], false /*simulate_bg_delays*/);
  activity_algorithm_get_steps(&steps_awake_100m);

  prv_feed_minute_data(20, &minute_data[100], false /*simulate_bg_delays*/);
  activity_algorithm_get_steps(&steps_awake_120m);

  // We should get steps counted while not sleeping
  printf("\nWhile awake: ");
  printf("\n  Counted %d steps first 60m", steps_awake_60m);
  printf("\n  Counted %d steps next 40m", steps_awake_100m - steps_awake_60m);
  printf("\n  Counted %d steps last 20m", steps_awake_120m - steps_awake_100m);
  printf("\n  Total: %d\n", steps_awake_120m);

  // Compute the expected number of steps
  int exp_steps = 0;
  for (int i = 0; i < num_minutes; i++) {
    exp_steps += minute_data[i].base.steps;
  }
  cl_assert_equal_i(steps_awake_120m, exp_steps);


  // -------------------------------------------------------------------------------
  // Try again while sleeping
  time_t start_utc = rtc_get_time();

  // Set to sleeping for the first 100 minutes
  s_kalg_sleep_start_utc = start_utc;
  s_kalg_sleep_m = 100;

  activity_algorithm_metrics_changed_notification();
  uint32_t steps_asleep_60m;
  uint32_t steps_asleep_100m;
  uint32_t steps_asleep_120m;

  // Call the minute handler, which should zero out steps that occur while sleeping
  prv_feed_minute_data(60, &minute_data[0], false /*simulate_bg_delays*/);
  activity_algorithm_get_steps(&steps_asleep_60m);

  prv_feed_minute_data(40, &minute_data[60], false /*simulate_bg_delays*/);
  activity_algorithm_get_steps(&steps_asleep_100m);

  prv_feed_minute_data(20, &minute_data[100], false /*simulate_bg_delays*/);
  activity_algorithm_get_steps(&steps_asleep_120m);

  // We should get steps counted while not sleeping
  printf("\nWhile asleep in the first 100m: ");
  printf("\n  Counted %d steps first 60m", steps_asleep_60m);
  printf("\n  Counted %d steps next 40m", steps_asleep_100m - steps_asleep_60m);
  printf("\n  Counted %d steps last 20m", steps_asleep_120m - steps_asleep_100m);
  printf("\n  Total: %d\n", steps_asleep_120m);

  // We should only get the steps counted from the last 20 minutes after waking
  cl_assert_equal_i(steps_asleep_120m, steps_awake_120m - steps_awake_100m);
}


// ---------------------------------------------------------------------------------------
// Test to make sure that the minute data we save has no steps during sleep
void test_activity_algorithm_kraepelin__minute_data_steps_during_sleep(void) {
  const int num_minutes = 120;

  // The test data
  AlgMinuteDLSSample minute_data[num_minutes];
  prv_create_test_data(num_minutes, minute_data);

  // Zero out the first hour of data. The sleep algorithm takes an hour to figure out that
  // you are sleeping, so it has no chance of zeroing out all the steps in that first hour.
  for (int i = 0; i < 60; i++) {
    minute_data[i].base.steps = 0;
  }

  time_t start_utc = rtc_get_time();

  // Set to sleeping for the first 100 minutes
  s_kalg_sleep_start_utc = start_utc;
  s_kalg_sleep_m = 100;

  // Write the data out
  prv_feed_minute_data(num_minutes, minute_data, false /*simulate_bg_delays*/);

  // Retrieve all the minute data and verify the contents
  HealthMinuteData retrieve[num_minutes];
  uint32_t num_records = num_minutes;
  time_t start = start_utc;
  activity_algorithm_get_minute_history(retrieve, &num_records, &start);
  cl_assert_equal_i(num_records, num_minutes);
  cl_assert_equal_i(start, start_utc);
  for (int i = 0; i < num_minutes; i++) {
    // If this is during the sleep period, steps should be 0
    if (i < 100) {
      cl_assert_equal_i(retrieve[i].steps, 0);
    } else {
      cl_assert_equal_i(retrieve[i].steps, minute_data[i].base.steps);
    }
    cl_assert_equal_i(retrieve[i].orientation, minute_data[i].base.orientation);
    cl_assert_equal_i(retrieve[i].vmc, minute_data[i].base.vmc);
    cl_assert_equal_i(retrieve[i].light, ambient_light_level_to_enum(
      minute_data[i].base.light * ALG_RAW_LIGHT_SENSOR_DIVIDE_BY));
    cl_assert_equal_i(retrieve[i].heart_rate_bpm, minute_data[i].heart_rate_bpm);
  }
}


