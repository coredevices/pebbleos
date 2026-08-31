/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "pbl/services/activity/activity.h"

// Version of our minute file minute records
// Version history:
//   4: Initial version
//   5: Added the flags field and the plugged_in bit
//   5 (3/1/16): Added the active bit to flags
//   6: Added heart rate bpm
#define ALG_MINUTE_FILE_RECORD_VERSION  6

// Format of each minute in our minute file. In the minute file, which is stored as a settings file
// on the watch, we store a subset of what we send to data logging since we only need the
// information required by the sleep algorithm and the information that could be returned by
// the health_service_get_minute_history() API call.
typedef struct __attribute__((__packed__)) {
  // Base fields, present in versions 4 and 5
  uint8_t steps;                    // # of steps in this minute
  uint8_t orientation;              // average orientation of the watch
  uint16_t vmc;                     // VMC (Vector Magnitude Counts) for this minute
  uint8_t light;                    // light sensor reading divided by
  //  ALG_RAW_LIGHT_SENSOR_DIVIDE_BY
  // New fields added in version 5
  union {
    struct {
      uint8_t plugged_in:1;
      uint8_t active:1;              // This is an "active" minute
      uint8_t reserved:6;
    };
    uint8_t flags;
  };
} AlgMinuteFileSampleV5;

typedef struct __attribute__((__packed__)) {
  // Base fields, present in versions <= 5
  AlgMinuteFileSampleV5 v5_fields;
  // New fields added in version 6
  uint8_t heart_rate_bpm;
} AlgMinuteFileSample;


// Version of our minute data logging records.
// NOTE: AlgDlsMinuteData and the mobile app will continue to assume it can parse the blob,
// only appending more properties is allowed.

// Android 3.10-4.0 requires bit 2 to be set, while iOS requires the value to be <= 255.
// Available versions are: 4, 5, 6, 7, 12, 13, 14, 15, 20, ...

// Version history:
//    4: Initial version
//    5: Added the bases.flags field
//    6: Added based.flags.active, resting_calories, active_calories, and distance_cm
//    7: Added heart rate bpm
//   12: Added total heart rate weight
//   13: Added heart rate zone
//   14: ... (NYI, you decide!)
#define ALG_DLS_MINUTES_RECORD_VERSION  13

_Static_assert((ALG_DLS_MINUTES_RECORD_VERSION & (1 << 2)) > 0,
               "Android 3.10-4.0 requires bit 2 to be set");
_Static_assert(ALG_DLS_MINUTES_RECORD_VERSION <= 225,
               "iOS requires version less that 255");

// Format of each minute in our data logging minute records.
typedef struct __attribute__((__packed__)) {
  // Base fields, which are also stored in the minute file on the watch. These are
  // present in versions 4 and 5.
  AlgMinuteFileSampleV5 base;

  // New fields added in version 6
  uint16_t resting_calories;         // number of resting calories burned in this minute
  uint16_t active_calories;          // number of active calories burned in this minute
  uint16_t distance_cm;              // distance in centimeters traveled in this minute

  // New fields added in version 7
  uint8_t heart_rate_bpm;            // weighted median hr value in this minute

  // New fields added in version 12
  uint16_t heart_rate_total_weight_x100; // total weight of all HR values multiplied by 100

  // New fields added in version 13
  uint8_t heart_rate_zone;           // the hr zone for this minute
} AlgMinuteDLSSample;


// We store minute data in this struct into a circular buffer and then transfer from there to
// data logging and to the minute file in PFS as we get a batch big enough.
typedef struct {
  time_t utc_sec;
  AlgMinuteDLSSample data;
} AlgMinuteRecord;


// Record header. The same header is used for minute file records and minute data logging records
typedef struct __attribute__((__packed__)) {
  uint16_t version;                  // Set to ALG_DLS_MINUTES_RECORD_VERSION or
                                     //   ALG_MINUTE_FILE_RECORD_VERSION
  uint32_t time_utc;                 // UTC time
  int8_t time_local_offset_15_min;   // add this many 15 minute intervals to UTC to get local time.
  uint8_t sample_size;               // size in bytes of each sample
  uint8_t num_samples;               // # of samples included (ALG_MINUTES_PER_RECORD)
} AlgMinuteRecordHdr;


// Format of each data logging minute data record
#define ALG_MINUTES_PER_DLS_RECORD    15
typedef struct __attribute__((__packed__)) {
  AlgMinuteRecordHdr hdr;
  AlgMinuteDLSSample samples[ALG_MINUTES_PER_DLS_RECORD];
} AlgMinuteDLSRecord;

// Format of each minute file record
#define ALG_MINUTES_PER_FILE_RECORD    15
typedef struct __attribute__((__packed__)) {
  AlgMinuteRecordHdr hdr;
  AlgMinuteFileSample samples[ALG_MINUTES_PER_FILE_RECORD];
} AlgMinuteFileRecord;


// Size quota for the minute file
#define ALG_MINUTE_DATA_FILE_LEN   0x20000

// Max possible number of entries we can fit in our settings file if there was no overhead to
// the settings file at all. The actual number we can fit is less than this.
#define ALG_MINUTE_FILE_MAX_ENTRIES (ALG_MINUTE_DATA_FILE_LEN / sizeof(AlgMinuteFileRecord))

//! Init the algorithm
//! @param[out] sampling_rate the required sampling rate is returned in this variable
//! @return true if success
bool activity_algorithm_init(AccelSamplingRate *sampling_rate);

//! Called at the start of the activity teardown process
void activity_algorithm_early_deinit(void);

//! Deinit the algorithm
//! @return true if success
bool activity_algorithm_deinit(void);

//! Set the user metrics. These are used for the calorie calculation today, and possibly other
//! calculations in the future.
//! @return true if success
bool activity_algorithm_set_user(uint32_t height_mm, uint32_t weight_g, ActivityGender gender,
                                 uint32_t age_years);

//! Process accel samples
//! @param[in] data pointer to the accel samples
//! @param[in] num_samples number of samples to process
//! @param[in] timestamp_ms timestamp of the first sample in ms
void activity_algorithm_handle_accel(AccelRawData *data, uint32_t num_samples,
                                     uint64_t timestamp_ms);

//! Called once per minute so the algorithm can collect minute stats and log them. This is
//! usually the data that gets used to compute sleep.
//! @param[in] utc_sec the UTC timestamp when the minute handler was first triggered
//! @param[out] record_out an AlgMinuteRecord that will be filled in
void activity_algorithm_minute_handler(time_t utc_sec, AlgMinuteRecord *record_out);

//! Return the current number of steps computed
//! @param[out] steps the number of steps is returned in this variable
//! @return true if success
bool activity_algorithm_get_steps(uint32_t *steps);

//! Tells the activity algorithm whether or not it should automatically track activities
//! @param enable true to start tracking, false to stop tracking
void activity_algorithm_enable_activity_tracking(bool enable);

//! Return the most recent stepping rate computed. This rate is returned as a number of steps
//! and an elapsed time.
//! @param[out] steps the number of steps taken during the last 'elapsed_sec' is returned in this
//!   variable.
//! @param[out] elapsed_ms the number of elapsed milliseconds is returned in this variable
//! @param[out] end_sec the UTC timestamp of the last time rate was computed is returned in this
//!   variable.
//! @return true if success
bool activity_algorithm_get_step_rate(uint16_t *steps, uint32_t *elapsed_ms, time_t *end_sec);

//! Reset all metrics that the algorithm tracks. Used at midnight to reset all metrics for a new
//! day and whenever new values are written into healthDB
//! @return true if success
bool activity_algorithm_metrics_changed_notification(void);

//! Set the algorithm steps to the given value. Used when first starting up the algorithm after
//! a watch reboot.
//! @param[in] steps set the number of steps to this
//! @return true if success
bool activity_algorithm_set_steps(uint32_t steps);

//! Return the timestamp of the last minute that was processed by the sleep detector.
time_t activity_algorithm_get_last_sleep_utc(void);

//! Send current minute data right away
void activity_algorithm_send_minutes(void);

//! Scan the list of activity sessions for sleep sessions and relabel the ones that should be
//! labeled as naps.
//! @param[in] num_sessions number of activity sessions
//! @param[in] sessions pointer to array of activity sessions
void activity_algorithm_post_process_sleep_sessions(uint16_t num_sessions,
                                                    ActivitySession *sessions);

//! Retrieve minute history
//! @param[in] minute_data pointer to an array of HealthMinuteData records that will be filled
//!     in with the historical minute data
//! @param[in,out] num_records On entry, the number of records the minute_data array can hold.
//!      On exit, the number of records in the minute data array that were written, including
//!      any missing minutes between 0 and *num_records. To check to see if a specific minute is
//!      missing, compare the vmc value of that record to HEALTH_MINUTE_DATA_MISSING_VMC_VALUE.
//! @param[in,out] utc_start on entry, the UTC time of the first requested record. On exit,
//!      the UTC time of the first record returned.
//! @return true if success
bool activity_algorithm_get_minute_history(HealthMinuteData *minute_data, uint32_t *num_records,
                                           time_t *utc_start);

//! Dump the current sleep file to PBL_LOG. We write out base64 encoded data using PBL_LOG
//! so that it can be extracted using a support request.
//! @return true if success
bool activity_algorithm_dump_minute_data_to_log(void);

//! Get info on the sleep file
//! @param[in] compact_first if true, compact the file first
//! @param[out] *num_records number of records in file
//! @param[out] *data_bytes bytes of data it contains
//! @param[out] *minutes how many minutes of data it contains
//! @return true if success
bool activity_algorithm_minute_file_info(bool compact_first, uint32_t *num_records,
                                         uint32_t *data_bytes, uint32_t *minutes);

//! Fill the sleep file
//! @return true if success
bool activity_algorithm_test_fill_minute_file(void);

//! Send a fake minute logging record to data logging. Useful for mobile app testing
//! @return true if success
bool activity_algorithm_test_send_fake_minute_data_dls_record(void);

