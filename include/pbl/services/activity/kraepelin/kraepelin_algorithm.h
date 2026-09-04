#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "util/time/time.h"


// ---------------------------------------------------------------------------------------------
// Equates
// number of samples per second
#define KALG_SAMPLE_HZ 25

// Number of grams per kilogram
#define KALG_GRAMS_PER_KG 1000

typedef struct KAlgState KAlgState;

// This value in the encoded_vmc field of a KalgSleepMinute structure indicates that the
// watch was not worn
#define KALG_ENCODED_VMC_NOT_WORN 0

// The minimum value if the watch was worn
#define KALG_ENCODED_VMC_MIN_WORN_VALUE 1

// The maximum amount of time it takes for the sleep algorithm to figure out that the user
// woke up. This is used by activity_algorithm_kraepelin.c at compile time so it must be
// hard-coded. The correct value is checked in kalg_init() using a PBL_ASSERT since it can't
// be checked at compile time.
// should be: KALG_SLEEP_PARAMS.max_wake_minutes_early + KALG_SLEEP_HALF_WIDTH + 1
#define KALG_MAX_UNCERTAIN_SLEEP_M 19

// Activity types, used in KAlgSleepSessionCallback callback
typedef enum {
  // ActivityType_Sleep encapsulates an entire sleep session from sleep entry to wake, and
  // contains both light and deep sleep periods. An ActivityType_RestfulSleep session identifies
  // a restful period and its start and end times will always be inside of a ActivityType_Sleep
  // session.
  KAlgActivityType_Sleep,

  // A restful period, these will always be inside of a ActivityType_Sleep session
  KAlgActivityType_RestfulSleep,

  // A "sigificant" length walk
  KAlgActivityType_Walk,

  // A run
  KAlgActivityType_Run,

  // Leave at end
  KAlgActivityTypeCount,
} KAlgActivityType;

// Sleep stats, returned by kalg_get_sleep_stats
typedef struct {
  time_t sleep_start_utc;     // start time of a recent sleep session. 0 if no session recently
                              // detected, where "recent" means within the last
                              // minimum_sleep_session_length minutes (currently 60).
  uint16_t sleep_len_m;       // how many minutes of that sleep are *certain*, 0 if none.
  time_t uncertain_start_utc; // start time of the uncertain area of the sleep session, which
                              // always continues until the present time, 0 if none.
} KAlgOngoingSleepStats;

// Callback called by kalg_activities_update to register activity sessions.
// @param[in] context the context passed to kalg_activities_update()
// @param[in] activity_type the type of activity
// @param[in] start_utc start time of the activity
// @param[in] len_sec length of the activity
// @param[in] ongoing true if the activity is still ongoing, false if it ended
// @param[in] delete if true, delete this session that was previously added
// @param[in] steps the number of steps taken in this activity
// @param[in] resting_calories the number of resting calories burned
// @param[in] active_calories the number of active calories burned
// @param[in] distance_mm the distance covered, in millimeters
typedef void (*KAlgActivitySessionCallback)(void *context, KAlgActivityType activity_type,
                                            time_t start_utc, uint32_t len_sec, bool ongoing,
                                            bool delete, uint32_t steps, uint32_t resting_calories,
                                            uint32_t active_calories, uint32_t distance_mm);

// Callback called by kalg_analyze_samples and kalg_compute_activities to record
// statistics. This is used during algorithm development, not during normal runtime. The algorithm
// passes a list of statistic names and their values to this callback so that they can be
// collected and summarized
// @param[in] num_stats the number of elements in the names and stats arrays
// @param[in] names list of statistic names
// @param[in] stats the value for each statistic
typedef void (*KAlgStatsCallback)(uint32_t num_stats, const char **names, int32_t *stats);

// Return the amount of space needed for the state
uint32_t kalg_state_size(void);

// Init the state, return true on success
// @param[in] stats_cb if not NULL, this callback will be called while analyzing samples with
//  statistics that are computed.
bool kalg_init(KAlgState *state, KAlgStatsCallback stats_cb);

// Release resources held by the state. Must be called before freeing it.
// @param[in] state the state structure passed into kalg_init
void kalg_deinit(KAlgState *state);

// Analyze a set of accel samples
// @param[in] state the state structure passed into kalg_init
// @param[in] samples array of accel samples
// @param[in] num_samples number of samples in the samples array
// @param[out] consumed_samples The number of samples that were just consumed to compute steps
//             For many algorithms, this will often be 0 because the algorithm will wait until
//             it has stored up a larger batch before it runs the step algorithm on the samples.
// @return number of steps counted
uint32_t kalg_analyze_samples(KAlgState *state, AccelRawData *samples, uint32_t num_samples,
                              uint32_t *consumed_samples);

// Return the last minute's stats and reset them for the next minute. The minute stats are logged
// and also used for computing sleep
// @param[in] state the state structure passed into kalg_init
// @param[out] vmc the vmc (Vector Magnitude Counts) value is returned here
// @param[out] orientation the orientation value is returned here
// @param[out] still true if no movement above noise threshold was detected
void kalg_minute_stats(KAlgState *state, uint16_t *vmc, uint8_t *orientation, bool *still);

// Used by unit tests - run the partial epoch that hasn't been processed yet by
// kalg_analyze_samples()
// @param[in] state the state structure passed into kalg_init
// @return number of steps counted
uint32_t kalg_analyze_finish_epoch(KAlgState *state);

// Feed new minute data into the activity detection state machine. This logic looks for non-sleep
// activities, like walks, runs, etc.
// @param[in] state the state structure passed into kalg_init
// @param[in] utc_now current timestamp in UTC
// @param[in] steps number of steps taken in the last minute
// @param[in] vmc VMC for the last minute
// @param[in] orientation average orientation for the last minute
// @param[in] definitely_not_worn true if the watch is definitely not being worn this minute
//                       (caller passes "plugged into charger" OR'd with any other definite
//                       not-worn hints such as a recent HRM off-wrist reading). Treated as a
//                       hard "not worn" signal for sleep detection.
// @param[in] resting_calories number of resting calories burned in the last minute
// @param[in] active_calories number of active calories burned in the last minute
// @param[in] distance_mm distance covered in millimeters in the last minute
// @param[in] sessions_cb this callback will be called by kalg_compute_activities() for every
//            session that it finds.
// @param[in] context passed to the sessions_cb
void kalg_activities_update(KAlgState *state, time_t utc_now, uint16_t steps, uint16_t vmc,
                            uint8_t orientation, bool definitely_not_worn,
                            uint32_t resting_calories,
                            uint32_t active_calories, uint32_t distance_mm, bool shutting_down,
                            KAlgActivitySessionCallback sessions_cb, void *context);

// Return the timestamp of the last minute that was processed for the given activity type
// @param[in] state the state structure passed into kalg_init
// @param[in] activity which type of activity
time_t kalg_activity_last_processed_time(KAlgState *state, KAlgActivityType activity);

// Get sleep stats
// @param[out] stats this structure is filled in with the sleep stats
void kalg_get_sleep_stats(KAlgState *state, KAlgOngoingSleepStats *stats);

//! Tells the algorithm whether or not it should automatically track activities
//! @param kalg_state the state structure passed into kalg_init
//! @param enable true to start tracking, false to stop tracking
void kalg_enable_activity_tracking(KAlgState *kalg_state, bool enable);
