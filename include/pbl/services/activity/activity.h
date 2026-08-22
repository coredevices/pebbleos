/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "applib/accel_service_private.h"
#include "applib/health_service.h"
#include "pbl/util/attributes.h"
#include "util/time/time.h"

// Max # of days of history we store
#define ACTIVITY_HISTORY_DAYS                     30

// The max number of activity sessions we collect and cache at a time. Usually, there will only be
// about 4 or 5 sleep sessions (1 container and a handful of restful periods) in a night and
// a handful of walk and/or run sessions. Allocating space for 32 to should be more than enough.
#define ACTIVITY_MAX_ACTIVITY_SESSIONS_COUNT      32

// Number of calories in a kcalorie
#define ACTIVITY_CALORIES_PER_KCAL                1000

// Values for ActivitySettingGender
typedef enum {
  ActivityGenderFemale = 0,
  ActivityGenderMale = 1,
  ActivityGenderOther = 2
} ActivityGender;

// Activity Settings Struct, for storing to prefs
typedef struct PACKED ActivitySettings {
  int16_t height_mm;
  int16_t weight_dag;
  bool tracking_enabled;
  bool activity_insights_enabled;
  bool sleep_insights_enabled;
  int8_t age_years;
  int8_t gender;
} ActivitySettings;

// Heart Rate Preferences Struct, for storing to prefs
typedef struct PACKED HeartRatePreferences {
  uint8_t resting_hr;
  uint8_t elevated_hr;
  uint8_t max_hr;
  uint8_t zone1_threshold;
  uint8_t zone2_threshold;
  uint8_t zone3_threshold;
} HeartRatePreferences;

// HRM measurement interval options
typedef enum {
  HRMonitoringInterval_10Min = 0,
  HRMonitoringInterval_30Min,
  HRMonitoringInterval_1Hour,
  HRMonitoringInterval_Disabled,
  HRMonitoringIntervalCount,
} HRMonitoringInterval;

// Activity HRM Settings Struct, for storing to prefs
typedef struct PACKED ActivityHRMSettings {
  bool enabled;
  uint8_t measurement_interval; // HRMonitoringInterval value
  bool activity_tracking_enabled; // HR tracking during detected activities (walk/run)
} ActivityHRMSettings;

// Default values, taken from http://www.cdc.gov/nchs/fastats/body-measurements.htm
#define ACTIVITY_DEFAULT_HEIGHT_MM                1620    // 5'3.8"
// dag - decagram (10 g)
#define ACTIVITY_DEFAULT_WEIGHT_DAG               7539    // 166.2 lbs
#define ACTIVITY_DEFAULT_GENDER                   ActivityGenderFemale
#define ACTIVITY_DEFAULT_AGE_YEARS                30

#define ACTIVITY_DEFAULT_PREFERENCES { \
  .tracking_enabled = false, \
  .activity_insights_enabled = false, \
  .sleep_insights_enabled = false, \
  .age_years = ACTIVITY_DEFAULT_AGE_YEARS, \
  .gender = ACTIVITY_DEFAULT_GENDER, \
  .height_mm = ACTIVITY_DEFAULT_HEIGHT_MM, \
  .weight_dag = ACTIVITY_DEFAULT_WEIGHT_DAG, \
}

#define ACTIVITY_HEART_RATE_DEFAULT_PREFERENCES { \
  .resting_hr = 70, \
  .elevated_hr = 100, \
  .max_hr = 220 - ACTIVITY_DEFAULT_AGE_YEARS, \
  .zone1_threshold = 130 /* 50% of HRR */, \
  .zone2_threshold = 154 /* 70% of HRR */, \
  .zone3_threshold = 172 /* 85% of HRR */, \
}

#define ACTIVITY_HRM_DEFAULT_PREFERENCES { \
  .enabled = true, \
  .measurement_interval = HRMonitoringInterval_10Min, \
  .activity_tracking_enabled = false, \
}

// We consider values outside of this range to be invalid
// In the future we could pick these values based on user history
#define ACTIVITY_DEFAULT_MIN_HR  40
#define ACTIVITY_DEFAULT_MAX_HR  200

// Activity metric enums, accepted by activity_get_metric()
typedef enum {
  ActivityMetricFirst = 0,
  ActivityMetricStepCount = ActivityMetricFirst,
  ActivityMetricActiveSeconds,
  ActivityMetricRestingKCalories,
  ActivityMetricActiveKCalories,
  ActivityMetricDistanceMeters,
  ActivityMetricSleepTotalSeconds,
  ActivityMetricSleepRestfulSeconds,
  ActivityMetricSleepEnterAtSeconds,               // What time the user fell asleep. Measured in
                                                   // seconds after midnight.
  ActivityMetricSleepExitAtSeconds,                // What time the user woke up. Measured in
                                                   // seconds after midnight
  ActivityMetricSleepState,                        // returns an ActivitySleepState enum value
  ActivityMetricSleepStateSeconds,                 // how many seconds we've been in the
                                                   // ActivityMetricSleepState state
  ActivityMetricLastVMC,

  ActivityMetricHeartRateRawBPM,                   // Most recent heart rate reading
  ActivityMetricHeartRateRawQuality,               // Heart rate signal quality
  ActivityMetricHeartRateRawUpdatedTimeUTC,        // UTC of last heart rate update
  ActivityMetricHeartRateFilteredBPM,              // Most recent "Stable (median)" HR reading
  ActivityMetricHeartRateFilteredUpdatedTimeUTC,   // UTC of last stable HR reading

  ActivityMetricHeartRateZone1Minutes,
  ActivityMetricHeartRateZone2Minutes,
  ActivityMetricHeartRateZone3Minutes,

  // KEEP THIS AT THE END
  ActivityMetricNumMetrics,
  ActivityMetricInvalid = ActivityMetricNumMetrics,
} ActivityMetric;


// Activity session types, used in ActivitySession struct
typedef enum {
  ActivitySessionType_None = 0,

  // ActivityType_Sleep encapsulates an entire sleep session from sleep entry to wake, and
  // contains both light and deep sleep periods. An ActivityType_DeepSleep session identifies
  // a restful period and its start and end times will always be inside of a ActivityType_Sleep
  // session.
  ActivitySessionType_Sleep = 1,

  // A restful period, these will always be inside of a ActivityType_Sleep session
  ActivitySessionType_RestfulSleep = 2,

  // Like ActivityType_Sleep, but labeled as a nap because of its duration and time (as
  // compared to the assumed nightly sleep).
  ActivitySessionType_Nap = 3,

  // A restful period that was part of a nap, these will always be inside of a
  // ActivityType_Nap session
  ActivitySessionType_RestfulNap = 4,

  // A "significant" length walk
  ActivitySessionType_Walk = 5,

  // A run
  ActivitySessionType_Run = 6,

  // Open workout. Basically a catch all / generic activity type
  ActivitySessionType_Open = 7,

  // Leave at end
  ActivitySessionTypeCount,
  ActivitySessionType_Invalid = ActivitySessionTypeCount,
} ActivitySessionType;

// Sleep state, used in AlgorithmStateMinuteData and to express possible values of
// ActivityMetricSleepState when calling activity_get_metric().
typedef enum {
  ActivitySleepStateAwake = 0,
  ActivitySleepStateRestfulSleep,
  ActivitySleepStateLightSleep,
  ActivitySleepStateUnknown,
} ActivitySleepState;


// Data included for stepping related activities.
// NOTE: modifying this struct requires a bump to the ACTIVITY_SESSION_LOGGING_VERSION and
// an update to documentation on this wiki page:
//   https://pebbletechnology.atlassian.net/wiki/pages/viewpage.action?pageId=46301269
typedef struct PACKED {
  uint16_t steps;                                 // number of steps
  uint16_t active_kcalories;                      // number of active kcalories
  uint16_t resting_kcalories;                     // number of resting kcalories
  uint16_t distance_meters;                       // distance covered
} ActivitySessionDataStepping;

// Data included for sleep related activities
// NOTE: modifying this struct requires a bump to the ACTIVITY_SESSION_LOGGING_VERSION and
// an update to documentation on this wiki page:
//   https://pebbletechnology.atlassian.net/wiki/pages/viewpage.action?pageId=46301269
typedef struct {
} ActivitySessionDataSleeping;

#define ACTIVITY_SESSION_MAX_LENGTH_MIN MINUTES_PER_DAY

typedef struct PACKED {
  time_t start_utc;                               // session start time
  uint16_t length_min;                            // length of session in minutes
  ActivitySessionType type:8;                     // type of activity
  union {
    struct {
      uint8_t ongoing:1;                          // activity still ongoing
      uint8_t manual:1;                           // activity is a manual one
      uint8_t reserved:6;
    };
    uint8_t flags;
  };
  union {
    ActivitySessionDataStepping step_data;
    ActivitySessionDataSleeping sleep_data;
  };
} ActivitySession;

// Structure of data logging records generated by raw sample collection
// Each of the 32bit samples in the record is encoded as follows:
//    Each axis is encoded into 10 bits, by shifting the 16-bit raw value right by 3 bits and
//    masking with 0x3FF. This is done because the max dynamic range of an axis is +/- 4000 and
//    the least significant 3 bits are more or less noise.
//    0bxx 10bits_x 10bits_y 10bits_z  The accel sensor generated a run of 0bxx samples with
//                                     the given x, y, and z values
#define ACTIVITY_RAW_SAMPLES_VERSION 2
#define ACTIVITY_RAW_SAMPLES_MAX_ENTRIES 25

// Utilities for the encoded samples collected by raw sample collection.
#define ACTIVITY_RAW_SAMPLE_VALUE_BITS (10)
#define ACTIVITY_RAW_SAMPLE_VALUE_MASK (0x03FF)     // 10 bits per axis

// We throw away the least significant 3 bits and keep only 10 bits per axix. The + 4 is used
// so that we round to nearest instead of rounding down as a result of the shift right
#define ACTIVITY_RAW_SAMPLE_SHIFT 3
#define ACTIVITY_RAW_SAMPLE_VALUE_ENCODE(x) ((((x) + 4) >> ACTIVITY_RAW_SAMPLE_SHIFT) \
                                             & ACTIVITY_RAW_SAMPLE_VALUE_MASK)

#define ACTIVITY_RAW_SAMPLE_MAX_RUN_SIZE 3
#define ACTIVITY_RAW_SAMPLE_GET_RUN_SIZE(s) ((s) >> (3 * ACTIVITY_RAW_SAMPLE_VALUE_BITS))
#define ACTIVITY_RAW_SAMPLE_SET_RUN_SIZE(s, r) (s |= (r) << (3 * ACTIVITY_RAW_SAMPLE_VALUE_BITS))
#define ACTIVITY_RAW_SAMPLE_SIGN_EXTEND(x) ((x) & 0x1000 ? -1 * (0x2000 - (x)) : (x))

#define ACTIVITY_RAW_SAMPLE_GET_X(s) \
    ACTIVITY_RAW_SAMPLE_SIGN_EXTEND((((uint32_t)s >> (2 * ACTIVITY_RAW_SAMPLE_VALUE_BITS)) \
                                & ACTIVITY_RAW_SAMPLE_VALUE_MASK) << ACTIVITY_RAW_SAMPLE_SHIFT)
#define ACTIVITY_RAW_SAMPLE_GET_Y(s) \
    ACTIVITY_RAW_SAMPLE_SIGN_EXTEND(((s >> ACTIVITY_RAW_SAMPLE_VALUE_BITS) \
                                & ACTIVITY_RAW_SAMPLE_VALUE_MASK) << ACTIVITY_RAW_SAMPLE_SHIFT)
#define ACTIVITY_RAW_SAMPLE_GET_Z(s) \
    ACTIVITY_RAW_SAMPLE_SIGN_EXTEND((s \
                                & ACTIVITY_RAW_SAMPLE_VALUE_MASK) << ACTIVITY_RAW_SAMPLE_SHIFT)

#define ACTIVITY_RAW_SAMPLE_ENCODE(run_size, x, y, z)     \
        ((run_size) << (3 * ACTIVITY_RAW_SAMPLE_VALUE_BITS))                                \
    |   (ACTIVITY_RAW_SAMPLE_VALUE_ENCODE(x) << (2 * ACTIVITY_RAW_SAMPLE_VALUE_BITS))        \
    |   (ACTIVITY_RAW_SAMPLE_VALUE_ENCODE(y) << ACTIVITY_RAW_SAMPLE_VALUE_BITS)              \
    |   ACTIVITY_RAW_SAMPLE_VALUE_ENCODE(z)

#define ACTIVITY_RAW_SAMPLE_FLAG_FIRST_RECORD     0x01    // Set for first record of session
#define ACTIVITY_RAW_SAMPLE_FLAG_LAST_RECORD      0x02    // set for last record of session
typedef struct __attribute__((__packed__)) {
  uint16_t version;                  // Set to ACTIVITY_RAW_SAMPLE_VERSION
  uint16_t session_id;               // raw sample session id
  uint32_t time_local;               // local time
  uint8_t flags;                     // one or more of ACTIVITY_RAW_SAMPLE_FLAG_.*
  uint8_t len;                       // length of this blob, including this entire header
  uint8_t num_samples;               // number of uncompressed samples that this blob represents
  uint8_t num_entries;               // number of elements in the entries array below
  uint32_t entries[ACTIVITY_RAW_SAMPLES_MAX_ENTRIES];
                                     // array of entries, each entry can represent multiple samples
                                     // if we detect run lengths
} ActivityRawSamplesRecord;


//! Init the activity tracking service. This does not start it up - to start it up call
//! activity_start_tracking();
//! @return true if successfully initialized
bool activity_init(void);

//! Returns true if the activity service is initialized
bool activity_is_initialized(void);

//! Start the activity tracking service. This starts sampling of the accelerometer
//! @param test_mode if true, samples must be fed in using activity_feed_samples()
//! @return true if successfully started
bool activity_start_tracking(bool test_mode);

//! Stop the activity tracking service.
//! @return true if successfully stopped
bool activity_stop_tracking(void);

//! Return true if activity tracking is currently running
//! @return true if activity tracking is currently running
bool activity_tracking_on(void);

//! Enable/disable the activity service. This callback is ONLY for use by the service manager's
//! services_set_runlevel() method. If false gets passed to this method, then tracking is
//! turned off regardless of the state as set by activity_start_tracking/activity_stop_tracking.
void activity_set_enabled(bool enable);

// Functions for getting and setting the activity preferences (defined in shell/normal/prefs.c)

//! Enable/disable activity tracking and store new setting in prefs for the next reboot
//! @param enable if true, enable activity tracking
void activity_prefs_tracking_set_enabled(bool enable);

//! Returns true if activity tracking is enabled
bool activity_prefs_tracking_is_enabled(void);

//! Records the current time when called. Used to determine when activity was first used
// so that we can send insights X days after activation
void activity_prefs_set_activated(void);

//! @return The utc timestamp of the first call to activity_prefs_set_activated()
//! returns 0 if activity_prefs_set_activated() has never been called
time_t activity_prefs_get_activation_time(void);

typedef enum ActivationDelayInsightType ActivationDelayInsightType;

//! @return True if the activation delay insight has fired
bool activity_prefs_has_activation_delay_insight_fired(ActivationDelayInsightType type);

//! Mark an activation delay insight as having fired
void activity_prefs_set_activation_delay_insight_fired(ActivationDelayInsightType type);

//! @return Which version of the health app was last opened
//! @note 0 is "never opened"
uint8_t activity_prefs_get_health_app_opened_version(void);

//! Record that the health app has been opened at a given version
void activity_prefs_set_health_app_opened_version(uint8_t version);

//! @return Which version of the workout app was last opened
//! @note 0 is "never opened"
uint8_t activity_prefs_get_workout_app_opened_version(void);

//! Record that the workout app has been opened at a given version
void activity_prefs_set_workout_app_opened_version(uint8_t version);

//! Enable/disable activity insights
//! @param enable if true, enable activity insights
void activity_prefs_activity_insights_set_enabled(bool enable);

//! Returns true if activity insights are enabled
bool activity_prefs_activity_insights_are_enabled(void);

//! Enable/disable sleep insights
//! @param enable if true, enable sleep insights
void activity_prefs_sleep_insights_set_enabled(bool enable);

//! Returns true if sleep insights are enabled
bool activity_prefs_sleep_insights_are_enabled(void);

//! Set the user height
//! @param height_mm the height in mm
void activity_prefs_set_height_mm(uint16_t height_mm);

//! Get the user height
//! @return the user's height in mm
uint16_t activity_prefs_get_height_mm(void);

//! Set the user weight
//! @param weight_dag the weight in dag (decagrams)
void activity_prefs_set_weight_dag(uint16_t weight_dag);

//! Get the user weight
//! @return the user's weight in dag
uint16_t activity_prefs_get_weight_dag(void);

//! Set the user's gender
//! @param gender the new gender
void activity_prefs_set_gender(ActivityGender gender);

//! Get the user's gender
//! @return the user's set gender
ActivityGender activity_prefs_get_gender(void);

//! Set the user's age
//! @param age_years the user's age in years
void activity_prefs_set_age_years(uint8_t age_years);

//! Get the user's age in years
//! @return the user's age in years
uint8_t activity_prefs_get_age_years(void);

//! Get the user's resting heart rate
uint8_t activity_prefs_heart_get_resting_hr(void);

//! Get the user's elevated heart rate
uint8_t activity_prefs_heart_get_elevated_hr(void);

//! Get the user's max heart rate
uint8_t activity_prefs_heart_get_max_hr(void);

//! Get the user's hr zone1 threshold (lowest HR in zone 1)
uint8_t activity_prefs_heart_get_zone1_threshold(void);

//! Get the user's hr zone2 threshold (lowest HR in zone 2)
uint8_t activity_prefs_heart_get_zone2_threshold(void);

//! Get the user's hr zone3 threshold (lowest HR in zone 3)
uint8_t activity_prefs_heart_get_zone3_threshold(void);

//! Return true if the HRM is enabled, false if not
bool activity_prefs_heart_rate_is_enabled(void);

//! Return true when detailed heart-rate logging during detected overnight sleep is enabled.
bool activity_prefs_enhanced_overnight_hr_logging_is_enabled(void);

//! Enable or disable detailed heart-rate logging during detected overnight sleep.
void activity_prefs_set_enhanced_overnight_hr_logging_enabled(bool enabled);

#ifdef CONFIG_HRM
//! Get the HRM measurement interval setting
//! @return the current HRMonitoringInterval value
HRMonitoringInterval activity_prefs_get_hrm_measurement_interval(void);

//! Set the HRM measurement interval
//! @param interval the desired HRMonitoringInterval value
void activity_prefs_set_hrm_measurement_interval(HRMonitoringInterval interval);

//! Return true if HR tracking during detected activities (walk/run) is enabled
bool activity_prefs_hrm_activity_tracking_is_enabled(void);

//! Enable or disable HR tracking during detected activities (walk/run)
void activity_prefs_set_hrm_activity_tracking_enabled(bool enabled);
#endif

//! Get the current and (optionally) historical values for a given metric. The caller passes
//! in a pointer to an array that will be filled in with the results (current value for today at
//! index 0, yesterday's at index 1, etc.)
//! @param[in] metric which metric to fetch
//! @param[in] history_len This must contain the length of the history array being passed in (as
//!     number of entries). To determine a max size for this array, call
//!     health_service_max_days_history().
//! @param[out] history pointer to int32_t array that will contain the returned metric. The current
//!     value will be at index 0, yesterday's at index 1, etc. For days where no history is
//!     available, -1 will be written. For some metrics, like HealthMetricActiveDayID and
//!     HealthMetricSleepDayID, history is not applicable, so all entries past entry 0 will
//!     always be filled in with -1.
//! @return true on success, false on failure
bool activity_get_metric(ActivityMetric metric, uint32_t history_len, int32_t *history);

//! Get the typical value for a metric on a given day of the week
bool activity_get_metric_typical(ActivityMetric metric, DayInWeek day, int32_t *value_out);

//! Get the value for a metric over the last 4 weeks
bool activity_get_metric_monthly_avg(ActivityMetric metric, int32_t *value_out);


//! Get detailed info about activity sessions. This fills in an array with info on all of the
//! activity sessions that ended after 12am (midnight) of the current day. The caller must allocate
//! space for the array and tell this method how many entries the array can hold
//! ("session_entries"). This call returns the actual number of entries required, which may be
//! greater or less than the passed in size. If it is greater, only the first session_entries are
//! filled in.
//! @param[in,out] *session_entries size of sessions array (as number of elements) on entry.
//!                On exit, this is set to the number of entries required to hold all sessions.
//! @param[out] sessions this array is filled in with the list of sessions.
//! @return true on success, false on failure
bool activity_get_sessions(uint32_t *session_entries, ActivitySession *sessions);

//! Return historical minute data.
//! IMPORTANT: This call will block on KernelBG, so it can only be called from the app or
//! worker task.
//! @param[in] minute_data pointer to an array of HealthMinuteData records that will be filled
//!     in with the historical minute data
//! @param[in,out] num_records On entry, the number of records the minute_data array can hold.
//!      On exit, the number of records in the minute data array that were written, including
//!      any missing minutes between 0 and *num_records. To check to see if a specific minute is
//!      missing, compare the vmc value of that record to HEALTH_MINUTE_DATA_MISSING_VMC_VALUE.
//! @param[in,out] utc_start on entry, the UTC time of the first requested record. On exit,
//!      the UTC time of the first record returned.
//! @return true on success, false on failure
bool activity_get_minute_history(HealthMinuteData *minute_data, uint32_t *num_records,
                                 time_t *utc_start);

// Metric averages, returned by activity_get_step_averages()
#define ACTIVITY_NUM_METRIC_AVERAGES (4 * 24) //!< one average for each 15 minute interval of a day
#define ACTIVITY_METRIC_AVERAGES_UNKNOWN  0xFFFF //!< indicates the average is unknown
typedef struct {
  uint16_t average[ACTIVITY_NUM_METRIC_AVERAGES];
} ActivityMetricAverages;

//! Return step averages.
//! @param[in] day_of_week day of the week to get averages for. Sunday: 0, Monday: 1, etc.
//! @param[out] averages pointer to ActivityStepAverages structure that will be filled
//!     in with the step averages.
//! @return true on success, false on failure
bool activity_get_step_averages(DayInWeek day_of_week, ActivityMetricAverages *averages);

//! Control raw accel sample collection. This method can be used to start and stop raw
//! accel sample collection. The samples are sent to data logging with tag
//! ACTIVITY_DLS_TAG_RAW_SAMPLES and also PBL_LOG messages are generated by base64 encoding the
//! data (so that it can be sent in a support request). Every time raw sample collection is
//! enabled, a new raw sample session id is created. This session id is saved along with the
//! samples and can be displayed to the user in the watch UI to help later identify specific
//! sessions.
//! @param[in] enable if true, enable sample collection
//! @param[in] disable if true, disable sample collection
//! @param[out] *enabled true if sample collection is currently enabled
//! @param[out] *session_id the current raw sample session id. If sampling is currently disabled,
//!     this is the session id of the most recently ended session.
//! @param[out] *num_samples the number of samples collected for the current session. If sampling is
//!     currently disabled, this is the number of samples collected in the most recently
//!     ended session.
//! @param[out] *seconds the number of seconds of data collected for the current session. If
//!     sampling is currently disabled, this is the number of seconds of data in the most recently
//!     ended session.
//! @return true on success, false on error
bool activity_raw_sample_collection(bool enable, bool disable, bool *enabled,
                                    uint32_t *session_id, uint32_t *num_samples, uint32_t *seconds);

//! Dump the current sleep data using PBL_LOG. We write out base64 encoded data using PBL_LOG
//! so that it can be extracted using a support request.
//! @return true on success, false on error
//! IMPORTANT: This call will block on KernelBG, so it can only be called from the app or
//! worker task.
bool activity_dump_sleep_log(void);

//! Used by test apps (running on firmware): feed in samples, bypassing the accelerometer.
//! In order to use this, you must have called activity_start_tracking(test_mode = true);
//! @param[in] data array of samples to feed in
//! @param[in] num_samples number of samples in the data array
//! @return true on success, false on error
bool activity_test_feed_samples(AccelRawData *data, uint32_t num_samples);

//! Used by test apps (running on firmware): call the periodic minute callback. This can be used to
//! accelerate tests, to run in non-real time.
//! @return true on success, false on error
bool activity_test_run_minute_callback(void);

//! Used by test apps (running on firmware): Get info on the minute data file
//! @param[in] compact_first if true, compact the file first before getting info
//! @param[out] *num_records how many records it contains
//! @param[out] *data_bytes how many bytes of data it contains
//! @param[out] *minutes how many minutes of data it contains
//! @return true on success, false on error
bool activity_test_minute_file_info(bool compact_first, uint32_t *num_records, uint32_t *data_bytes,
                                    uint32_t *minutes);

//! Used by test apps (running on firmware): Fill up the minute data file with as much data as
//! possible. Used for testing performance of compaction and checking for watchdog timeouts when
//! the file gets very large.
//! @return true on success, false on error
bool activity_test_fill_minute_file(void);

//! Used by test apps (running on firmware): Send fake records to data logging. This sends the
//! following records: AlgMinuteDLSRecord, ActivityLegacySleepSessionDataLoggingRecord,
//! ActivitySessionDataLoggingRecord (one for each activity type).
//! Useful for mobile app testing
//! @return true if success
bool activity_test_send_fake_dls_records(void);

//! Used by test apps (running on firmware): Set the current step count
//! Useful for testing the health app
//! @param[in] new_steps the number of steps to set the current steps to
//! @param[in] current_avg the value to set the current step average to
//! @param[in] daily_avg the value to set the daily step average to
void activity_test_set_steps_and_avg(int32_t new_steps, int32_t current_avg, int32_t daily_avg);

//! Used by test apps (running on firmware): Set the past seven days of history
//! Useful for testing the health app
void activity_test_set_steps_history();

//! Used by test apps (running on firmware): Set the past seven days of history
//! Useful for testing the health app
void activity_test_set_sleep_history();
