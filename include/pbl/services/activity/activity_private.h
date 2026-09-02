/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "activity.h"

#include "pbl/kernel/sem.h"
#include "hr_util.h"

#include "applib/event_service_client.h"
#include "kernel/events.h"
#include "pbl/kernel/mutex.h"
#include "pbl/services/data_logging/data_logging_service.h"
#include "pbl/services/settings/settings_file.h"
#include "system/hexdump.h"
#include <pbl/logging/logging.h>
#include "pbl/util/attributes.h"

#include <stdbool.h>
#include <stdint.h>

#define ACTIVITY_LOG_DEBUG(fmt, args...) \
        PBL_LOG_D_DBG(LOG_DOMAIN_ACTIVITY, fmt, ## args)

#define ACTIVITY_HEXDUMP(data, length) \
        PBL_HEXDUMP_D(LOG_DOMAIN_DATA_ACTIVITY, LOG_LEVEL_DEBUG, data, length)

// How often we update settings with the current step/sleep stats for today.
#define ACTIVITY_SETTINGS_UPDATE_MIN              15

// How often we recompute the activity sessions (like sleep, walks, runs). This has significant
// enough CPU requirements to warrant only recomputing occasionally
#define ACTIVITY_SESSION_UPDATE_MIN               15

// Every scalar metric and setting is stored in globals and in the settings file using this
// typedef. Must be wide enough for daily step counts and distance in meters, which can
// legitimately exceed UINT16_MAX (FIRM-3071).
typedef uint32_t ActivityScalarStore;
#define ACTIVITY_SCALAR_MAX                       UINT32_MAX

// Each step average interval covers this many minutes
#define ACTIVITY_STEP_AVERAGES_MINUTES             (MINUTES_PER_DAY / ACTIVITY_NUM_METRIC_AVERAGES)

// flash vs. the most amount of data we could lose if we reset.
#define ACTIVITY_STEP_AVERAGES_PER_KEY 4
#define ACTIVITY_STEP_AVERAGES_KEYS_PER_DAY   \
                      (ACTIVITY_NUM_METRIC_AVERAGES / ACTIVITY_STEP_AVERAGES_PER_KEY)

// If we see at least this many steps in a minute, it was an "active minute"
#define ACTIVITY_ACTIVE_MINUTE_MIN_STEPS 40

// We consider any sleep session that ends after this minute of the day (representing 9pm) as
// part of the next day's sleep
#define ACTIVITY_LAST_SLEEP_MINUTE_OF_DAY  (21 * MINUTES_PER_HOUR)

// Default HeartRate sampling ON time
#define ACTIVITY_DEFAULT_HR_ON_TIME_SEC (60)

// Turn off the HR device after we've received X good quality samples
#define ACTIVITY_MIN_NUM_GOOD_SAMPLES_SHORT_CIRCUIT (10)

// Turn off the HR device after we've received X excellent quality samples
#define ACTIVITY_MIN_NUM_EXCELLENT_SAMPLES_SHORT_CIRCUIT (5)

// The minimum number of samples needed before we can approximate the user's HR zone
#define ACTIVITY_MIN_NUM_SAMPLES_FOR_HR_ZONE (5)

// HRM Subscription values during ON and OFF periods
#define ACTIVITY_HRM_SUBSCRIPTION_ON_PERIOD_SEC  (1)
#define ACTIVITY_HRM_SUBSCRIPTION_OFF_PERIOD_SEC (SECONDS_PER_DAY)

// After this many seconds without an HRM event, the cached worn-status is considered stale and
// the sleep algorithm falls back to its accel-based not-worn heuristics. Sized to comfortably
// cover the default HRMonitoringInterval_10Min cycle (~11 min) — at the longer 30/60-min
// intervals the cache will simply expire between bursts and sleep detection won't lean on a
// stale on-wrist reading.
#define ACTIVITY_HRM_OFFWRIST_STALE_SEC (15 * SECONDS_PER_MINUTE)

// Max number of stored HR samples to compute the median
#define ACTIVITY_MAX_HR_SAMPLES (3 * SECONDS_PER_MINUTE)

// Conversion factors
#define ACTIVITY_DAG_PER_KG  100


// -----------------------------------------------------------------------------------------
// Settings file info and keys
#define ACTIVITY_SETTINGS_FILE_NAME           "activity"
#define ACTIVITY_SETTINGS_FILE_LEN            0x4000

// The version of our settings file
// Version 1 - ActivitySettingsKeyVersion didn't exist
// Version 2 - Changed file size from 2k to 16k
// Version 3 - ActivityScalarStore widened from uint16_t to uint32_t, changing the layout of
//             ActivitySettingsValueHistory records and of scalar metric records
#define ACTIVITY_SETTINGS_CURRENT_VERSION     3

typedef struct {
  uint32_t utc_sec;                     // timestamp of first entry in list
  // One entry per day. The most recent day (today) is stored at index 0
  ActivityScalarStore values[ACTIVITY_HISTORY_DAYS];
} ActivitySettingsValueHistory;


// Keys of the settings we save in our settings file.
typedef enum {
  ActivitySettingsKeyInvalid = 0,                 // Used for error discovery
  ActivitySettingsKeyVersion,                     // uint16_t: ACTIVITY_SETTINGS_CURRENT_VERSION
  ActivitySettingsKeyUnused0,                     // Unused
  ActivitySettingsKeyUnused1,                     // Unused
  ActivitySettingsKeyUnused2,                     // Unused
  ActivitySettingsKeyUnused3,                     // Unused

  ActivitySettingsKeyStepCountHistory,            // ActivitySettingsValueHistory
  ActivitySettingsKeyStepMinutesHistory,          // ActivitySettingsValueHistory
  ActivitySettingsKeyUnused4,                     // Unused
  ActivitySettingsKeyDistanceMetersHistory,       // ActivitySettingsValueHistory
  ActivitySettingsKeySleepTotalMinutesHistory,    // ActivitySettingsValueHistory
  ActivitySettingsKeySleepDeepMinutesHistory,     // ActivitySettingsValueHistory
  ActivitySettingsKeySleepEntryMinutesHistory,    // ActivitySettingsValueHistory
                                                  // How long it took to fall asleep
  ActivitySettingsKeySleepEnterAtHistory,         // ActivitySettingsValueHistory
                                                  // What time the user fell asleep. Measured in
                                                  // minutes after midnight.
  ActivitySettingsKeySleepExitAtHistory,          // ActivitySettingsValueHistory
                                                  // What time the user woke up. Measured in
                                                  // minutes after midnight
  ActivitySettingsKeySleepState,                  // ActivityScalarStore
  ActivitySettingsKeySleepStateMinutes,           // ActivityScalarStore
  ActivitySettingsKeyStepAveragesWeekdayFirst,    // ACTIVITY_STEP_AVERAGES_PER_CHUNK * uint16_t
  ActivitySettingsKeyStepAveragesWeekdayLast =
        ActivitySettingsKeyStepAveragesWeekdayFirst + ACTIVITY_STEP_AVERAGES_KEYS_PER_DAY - 1,

  ActivitySettingsKeyStepAveragesWeekendFirst,    // ACTIVITY_STEP_AVERAGES_PER_CHUNK * uint16_t
  ActivitySettingsKeyStepAveragesWeekendLast =
        ActivitySettingsKeyStepAveragesWeekendFirst + ACTIVITY_STEP_AVERAGES_KEYS_PER_DAY - 1,
  ActivitySettingsKeyAgeYears,                    // uint16_t: age in years

  ActivitySettingsKeyUnused5,                     // Unused

  ActivitySettingsKeyInsightSleepRewardTime,      // time_t: time we last showed the sleep reward
                                                  // This will be 0 if we haven't triggered one yet
  ActivitySettingsKeyInsightActivityRewardTime,   // time_t: time we last showed the activity reward
                                                  // This will be 0 if we haven't triggered one yet
  ActivitySettingsKeyInsightActivitySummaryState, // SummaryPinLastState: the UUID and last time the
                                                  // pin was added
  ActivitySettingsKeyInsightSleepSummaryState,    // SummaryPinLastState: the UUID and last time the
                                                  // pin was added
  ActivitySettingsKeyRestingKCaloriesHistory,     // ActivitySettingsValueHistory
  ActivitySettingsKeyActiveKCaloriesHistory,      // ActivitySettingsValueHistory
  ActivitySettingsKeyLastSleepActivityUTC,        // time_t: UTC timestamp of the last sleep related
                                                  //  activity we logged to analytics
  ActivitySettingsKeyLastRestfulSleepActivityUTC, // time_t: UTC timestamp of the last restful sleep
                                                  // related activity we logged to analytics
  ActivitySettingsKeyLastStepActivityUTC,         // time_t: UTC timestamp of the last step related
                                                  //  activity we logged to analytics
  ActivitySettingsKeyStoredActivities,            // ActivitySession[
                                                  //         ACTIVITY_MAX_ACTIVITY_SESSIONS_COUNT]
  ActivitySettingsKeyInsightNapSessionTime,       // time_t: time we last showed the nap pin
  ActivitySettingsKeyInsightActivitySessionTime,  // time_t: time we last showed the activity pin
  ActivitySettingsKeyLastVMC,                     // ActivityScalarStore: the VMC at the last
                                                  // processed minute
  ActivitySettingsKeyRestingHeartRate,            // ActivitySettingsValueHistory
  ActivitySettingsKeyHeartRateZone1Minutes,
  ActivitySettingsKeyHeartRateZone2Minutes,
  ActivitySettingsKeyHeartRateZone3Minutes,
} ActivitySettingsKey;


// -----------------------------------------------------------------------------------------
// Internal structs
// IMPORTANT: activity_metrics_prv_get_metric_info() assumes that every element of ActivityStepData
// is an ActivityScalarStore
typedef struct {
  ActivityScalarStore steps;
  ActivityScalarStore step_minutes;
  ActivityScalarStore distance_meters;
  ActivityScalarStore resting_kcalories;
  ActivityScalarStore active_kcalories;
} ActivityStepData;

// IMPORTANT: activity_metrics_prv_get_metric_info() assumes that every element of ActivitySleepData
// is an ActivityScalarStore
typedef struct {
  ActivityScalarStore total_minutes;
  ActivityScalarStore restful_minutes;
  ActivityScalarStore enter_at_minute;            // minutes after midnight
  ActivityScalarStore exit_at_minute;             // minutes after midnight
  ActivityScalarStore cur_state;                  // HealthActivity
  ActivityScalarStore cur_state_elapsed_minutes;
} ActivitySleepData;

// IMPORTANT: activity_metrics_prv_get_metric_info() assumes that elements of
// ActivityHeartRateData are ActivityScalarStore by default. The update_time_utc is
// specially coded as a 32-bit metric and is allowed to be because we don't persist it in
// the settings file and it has no history
typedef struct {
  ActivityScalarStore current_bpm;           // Most current reading
  uint32_t current_update_time_utc;          // Timestamp of the current HR reading
  ActivityScalarStore current_hr_zone;
  ActivityScalarStore resting_bpm;
  ActivityScalarStore current_quality;       // HRMQuality
  ActivityScalarStore last_stable_bpm;
  uint32_t last_stable_bpm_update_time_utc;  // Timestamp of the last stable BPM
  ActivityScalarStore previous_median_bpm;   // Most recently calculated median HR in a minute
  int32_t previous_median_total_weight_x100;
  ActivityScalarStore minutes_in_zone[HRZoneCount];
  bool is_hr_elevated;
} ActivityHeartRateData;


// This callback used to convert a metric from the storage format (as a ActivityScalarStore) into
// the return format (uint32_t) returned by activity_get_metric. It might convert minutes to
// seconds, etc.
typedef uint32_t (*ActivityMetricConverter)(ActivityScalarStore storage_value);

// Filled in by activity_metrics_prv_get_metric_info()
typedef struct {
  ActivityScalarStore *value_p;       // pointer to storage in globals
  uint32_t *value_u32p;               // alternate value pointer for 32-bit metrics. These
                                      // can NOT have history and settings_key MUST be
                                      // ActivitySettingsKeyInvalid.
  bool has_history;                   // True if this metric has history. This determines the
                                      // size of the value as stored in settings
  ActivitySettingsKey settings_key;   // Settings key for this value
  ActivityMetricConverter converter;  // convert from storage value to return value.
} ActivityMetricInfo;

// Used by activity_feed_samples
typedef struct {
  uint16_t num_samples;
  AccelRawData data[];
} ActivityFeedSamples;

// Version of our legacy sleep session logging records (prior to FW 3.11). NOTE: The version
// field is treated as a bitfield. For version 1, only bit 0 is set. As long as we keep bit 0 set,
// we are free to add more fields to the end of ActivityLegacySleepSessionDataLoggingRecord and the
// mobile app will continue to assume it can parse the blob. If bit 0 is cleared, the mobile app
// will know that it has no chance of parsing the blob (until the mobile app is updated of course).
#define ACTIVITY_SLEEP_SESSION_LOGGING_VERSION 1

// Data logging record used to send sleep sessions to the phone
typedef struct PACKED {
  uint16_t version;              // set to ACTIVITY_SLEEP_SESSION_LOGGING_VERSION
  int32_t utc_to_local;          // Add this to UTC to get local time
  uint32_t start_utc;            // The start time in UTC
  uint32_t end_utc;              // The end time in UTC
  uint32_t restful_secs;
} ActivityLegacySleepSessionDataLoggingRecord;

// Version of our activity session logging records. NOTE: The version field is treated as a
// bitfield. For version 1, only bit 0 is set. As long as we keep bit 0 set, we are free to
// add more fields to the end of ActivitySessionDataLoggingRecord and the mobile app
// will continue to assume it can parse the blob. If bit 0 is cleared, the mobile app will know that
// it has no chance of parsing the blob (until the mobile app is updated of course).
#define ACTIVITY_SESSION_LOGGING_VERSION 3

// Data logging record used to send activity sessions to the phone
// NOTE: modifying this struct requires a bump to the ACTIVITY_SESSION_LOGGING_VERSION and
// an update to documentation on this wiki page:
//   https://pebbletechnology.atlassian.net/wiki/pages/viewpage.action?pageId=46301269
typedef struct PACKED {
  uint16_t version;                 // set to ACTIVITY_SESSION_LOGGING_VERSION
  uint16_t size;                    // size of this structure
  uint16_t activity;                // ActivitySessionType: the type of activity
  int32_t utc_to_local;             // Add this to UTC to get local time
  uint32_t start_utc;               // The start time in UTC
  uint32_t elapsed_sec;             // Elapsed time in seconds

  // New fields add in version 3
  union {
    ActivitySessionDataStepping step_data;
    ActivitySessionDataSleeping sleep_data;
  };
} ActivitySessionDataLoggingRecord;

// -----------------------------------------------------------------------------------------
// Globals

// Support for raw accel sample collection
typedef struct {
  // The data logging session for the current sample collection session
  DataLoggingSession *dls_session;

  // Most recently encoded accel sample value. Used for detecting and encoding runs of the same
  // value
  uint32_t prev_sample;           // See comments in ActivityRawSamplesRecord for encoding
  uint8_t run_size;               // run size of prev_sample

  // The currently forming record
  ActivityRawSamplesRecord record;

  // large enough to base64 encode half of the record at once.
  char base64_buf[sizeof(ActivityRawSamplesRecord)];

  // True if we are forming the first record
  bool first_record;
} ActivitySampleCollectionData;

// This type is defined in measurements_log.h but we can't include measurements_log.h in this header
// because of build issues with the auto-generated SDK files.
typedef void *ProtobufLogRef;

// Support for heart rate
typedef struct {
  ActivityHeartRateData metrics;      // ActivityMetrics for heart rate

  HRMSessionRef hrm_session;          // The HRM session we use
  ProtobufLogRef log_session;     // The measurements log we send data to

  bool currently_sampling;            // Are we activity sampling the HR
  uint32_t toggled_sampling_at_ts;    // When we last toggled our sampling rate
                                      // (from time_get_uptime_seconds)

  uint32_t last_sample_ts;            // When we last received a HR sample
                                      // (from time_get_uptime_seconds)

  uint16_t num_samples;               // number of samples in the past minute
  uint16_t num_good_quality_samples;  // number of samples in the past minute with good quality
  uint16_t num_excellent_samples;     // number of samples in the past minute with excellent quality
  uint8_t  samples[ACTIVITY_MAX_HR_SAMPLES]; // HR Samples stored
  uint8_t  weights[ACTIVITY_MAX_HR_SAMPLES]; // HR Sample Weights

  // Worn status from the most recent HRM BPM event. last_quality_event_utc is 0 if we've never
  // received one. Used by sleep tracking to suppress detection while the watch is off-wrist.
  time_t last_quality_event_utc;
  bool last_quality_was_offwrist;
} ActivityHRSupport;

typedef struct {
  // Mutex for serializing access to these globals
  struct pbl_mutex mutex;

  // Semaphore used for waiting for KernelBG to finish a callback
  struct pbl_sem bg_wait_semaphore;

  // Accel session ref
  AccelServiceState *accel_session;

  // Event Service to keep track of whether the charger is connected
  EventServiceInfo charger_subscription;

  // Cumulative stats for today
  ActivityStepData step_data;
  ActivitySleepData sleep_data;

  // We accumulate distance in mm to and active/resting calories in calories (not kcalories) to
  // minimize rounding errors since we increment them every time we get a new rate reading from the
  // algorithm (every 5 seconds).
  uint32_t distance_mm;
  uint32_t active_calories;
  uint32_t resting_calories;
  ActivityScalarStore last_vmc;
  uint8_t last_orientation;
  time_t rate_last_update_time;

  // Most recently calculated minute average walking rate
  ActivityScalarStore steps_per_minute;
  ActivityScalarStore steps_per_minute_last_steps;

  // The most recent minute that had any significant step activity. Used for computing
  // amount of time it takes to fall asleep
  uint16_t last_active_minute;

  // Heart rate support
  ActivityHRSupport hr;

  // Most recent values from prv_get_day()
  uint16_t cur_day_index;

  // Modulo counter used to periodically update settings file
  int8_t update_settings_counter;

  // Captured activity sessions
  uint16_t activity_sessions_count;        // how many sessions we have captured
  ActivitySession activity_sessions[ACTIVITY_MAX_ACTIVITY_SESSIONS_COUNT];
  bool need_activities_saved;              // true if activities need to be persisted

  // Set to true when a new sleep session is registered
  bool sleep_sessions_modified;

  // Exit time for the last sleep/step activities we logged. Used to prevent logging the same event
  // more than once.
  time_t logged_sleep_activity_exit_at_utc;
  time_t logged_restful_sleep_activity_exit_at_utc;
  time_t logged_step_activity_exit_at_utc;

  // Data logging session used for sending activity sessions (introduced in v3.11)
  DataLoggingSession *activity_dls_session;

  // Variables used for detecting "significant activity" events
  time_t activity_event_start_utc;            // UTC of first active minute, 0 if none detected

  // True if service has been enabled via services_set_runlevel.
  bool enabled_run_level;
  // True if the current state of charging allows the service to run.
  bool enabled_charging_state;

  // True if activity tracking should be started. If enabled is false, this can still be true
  // and will tell us that we should re-start tracking once enabled gets set again.
  bool should_be_started;

  // True if tracking has actually been started. This will only ever be set if enabled is also
  // true.
  bool started;

  // Support for raw accel sample collection
  bool sample_collection_enabled;
  uint16_t sample_collection_session_id;         // raw sample collection session id
  time_t sample_collection_seconds;              // if enabled is true, the UTC when sample
  // collection started, else the # of seconds of
  // of data in recently ended session
  uint16_t sample_collection_num_samples;        // number of samples collected so far
  ActivitySampleCollectionData *sample_collection_data;

  // True if activity_start_tracking was called with test_mode = true
  bool test_mode;
  bool pending_test_cb;
} ActivityState;

//! Get pointer to the activity state
ActivityState *activity_private_state(void);

//! Get whether HRM is present
bool activity_is_hrm_present(void);

//! Shared with activity_insights.c - opens the activity settings file
//! IMPORTANT: This function must only be called during activity init routines or while holding
//! the activity mutex
SettingsFile *activity_private_settings_open(void);

//! Shared with activity_insights.c - closes the activity settings file
//! IMPORTANT: This function must only be called during activity init routines or while holding
//! the activity mutex
void activity_private_settings_close(SettingsFile *file);

//! Used by test apps (running on firmware): Re-initialize activity service. If reset_settings is
//! true, all persistent data is cleared
//! @param[in] reset_settings if true, reset all stored settings
//! @param[in] tracking_on if true, turn on tracking if not already on. Otherwise, preserve
//!                         the current tracking status
//! @param[in] sleep_history if not NULL, rewrite sleep history to these values
//! @param[in] step_history if not NULL, rewrite step history to these values
bool activity_test_reset(bool reset_settings, bool tracking_on,
                         const ActivitySettingsValueHistory *sleep_history,
                         const ActivitySettingsValueHistory *step_history);

// --------------------------------------------------------------------------------
// Activity Sessions
// Load in the stored activities from our settings file
void activity_sessions_prv_init(SettingsFile *file, time_t utc_now);

// Get the start of the sleep-day window (the ACTIVITY_LAST_SLEEP_MINUTE_OF_DAY local-time
// cutoff) that now_utc belongs to
time_t activity_sessions_prv_get_sleep_window_start_utc(time_t now_utc);

// Get the UTC time bounds for the current day
void activity_sessions_prv_get_sleep_bounds_utc(time_t now_utc, time_t *enter_utc,
                                                time_t *exit_utc);

// Remove all activity sessions that are older than "today", those that are invalid because they
// are in the future, and optionally those that are still ongoing.
void activity_sessions_prv_remove_out_of_range_activity_sessions(time_t utc_sec,
                                                                 bool remove_ongoing);

//! Return true if the given activity type is sleep related
bool activity_sessions_prv_is_sleep_activity(ActivitySessionType activity_type);

//! Return true if the given activity type has session that is currently ongoing.
bool activity_sessions_is_session_type_ongoing(ActivitySessionType activity_type);

//! Register a new activity session. This is called by the algorithm logic when it detects a new
//! activity.
void activity_sessions_prv_add_activity_session(ActivitySession *session);

//! Delete an activity session. This is called by the algorithm logic when it decides to not
//! register a sleep session after all. Only sessions that are still 'ongoing' are allowed to be
//! deleted.
void activity_sessions_prv_delete_activity_session(ActivitySession *session);

//! Perform our once a minute activity session maintenance logic
void activity_sessions_prv_minute_handler(time_t utc_sec);

//! Send an activity session to data logging
void activity_sessions_prv_send_activity_session_to_data_logging(ActivitySession *session);


// ---------------------------------------------------------------------------
// Activity Metrics

//! Init all metrics
void activity_metrics_prv_init(SettingsFile *file, time_t utc_now);

//! Returns info about each metric we capture
void activity_metrics_prv_get_metric_info(ActivityMetric metric, ActivityMetricInfo *info);

//! Perform our once a minute metrics maintenance logic
void activity_metrics_prv_minute_handler(time_t utc_sec);

//! Returns the number of millimeters the user has walked so far today (since midnight)
uint32_t activity_metrics_prv_get_distance_mm(void);

//! Returns the number of resting calories the user has consumed so far today (since midnight)
uint32_t activity_metrics_prv_get_resting_calories(void);

//! Returns the number of active calories the user has consumed so far today (since midnight)
uint32_t activity_metrics_prv_get_active_calories(void);

//! Retrieve the median heart rate and the total weight x100 since it was last reset.
//! If no readings were recorded since it was reset, it will return 0.
//! This median can be reset using activity_metrics_prv_reset_hr_stats().
//! It is by default reset once a minute.
void activity_metrics_prv_get_median_hr_bpm(int32_t *median_out,
                                            int32_t *heart_rate_total_weight_x100_out);

//! Retrieve the current HR zone since it was last reset.
//! If no readings were recorded since it was reset, it will return 0.
//! This HR zone can be reset using activity_metrics_prv_reset_hr_stats().
//! It is by default reset once a minute.
HRZone activity_metrics_prv_get_hr_zone(void);

//! Reset the average / median heart rate and hr zone
void activity_metrics_prv_reset_hr_stats(void);

//! Feed in a new heart rate sample that will be used to update the median. This updates
//! the value returned by activity_metrics_prv_get_median_hr_bpm().
void activity_metrics_prv_add_median_hr_sample(PebbleHRMEvent *hrm_event, time_t now_utc,
                                               time_t now_uptime);

//! Record the worn status reported by the HRM. Called once per BPM event.
//! @param[in] now_utc current UTC time
//! @param[in] is_offwrist true if the event's HRMQuality was HRMQuality_OffWrist
void activity_metrics_prv_set_hrm_worn_status(time_t now_utc, bool is_offwrist);

//! Returns true if the HRM has recently reported the watch is off-wrist. The most recent BPM
//! event must have been HRMQuality_OffWrist and must have arrived within the last
//! ACTIVITY_HRM_OFFWRIST_STALE_SEC seconds, otherwise this returns false.
bool activity_metrics_prv_is_hrm_offwrist(time_t now_utc);

//! Returns the number of steps the user has taken so far today (since midnight)
uint32_t activity_metrics_prv_get_steps(void);

//! Returns the number of steps the user has walked in the past minute
ActivityScalarStore activity_metrics_prv_steps_per_minute(void);

//! Set a metric's value. Used from BlobDB to honor requests from the phone
void activity_metrics_prv_set_metric(ActivityMetric metric, DayInWeek day, int32_t value);

//! Force the current day's value of a metric to an exact value (may decrease it). Intended for
//! QEMU/test injection of health data.
void activity_metrics_set_metric_exact(ActivityMetric metric, int32_t value);
