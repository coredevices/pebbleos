/* Project Kraepelin, Main file
The MIT License (MIT)

Copyright (c) 2015, Nathaniel T. Stockham

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

This license is taken to apply to any other files in the Project Kraepelin
Pebble App roject.
*/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "applib/accel_service.h"
#include "pbl/util/trig.h"
#include "pbl/services/hrm/hrm_manager_private.h"
#include "pbl/services/activity/activity.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/math.h"
#include "pbl/util/math_fixed.h"
#include "pbl/util/size.h"

#include "pbl/services/activity/kraepelin/kraepelin_algorithm.h"

PBL_LOG_MODULE_DECLARE(service_activity, CONFIG_SERVICE_ACTIVITY_LOG_LEVEL);

#define KALG_LOG_DEBUG(fmt, args...) \
        PBL_LOG_D_DBG(LOG_DOMAIN_ACTIVITY, fmt, ## args)

// Set this to 1 to get text graphs of the overall FFT magnitudes
#define KALG_LOG_OVERALL_MAGNITUDES 0

// Set this to 1 to get text graphs of magnitudes of each axis
#define KALG_LOG_AXIS_MAGNITUDES 0

// ---------------------------------------------------------------------------------------------
// Internal equates

// 5*25 = 125 samples recorded, 5 seconds for step count
#define KALG_N_SAMPLES_EPOCH  (5 * 25)

// We drop these LS bits from each accel sample
#define KALG_ACCEL_SAMPLE_DIV    8
#define KALG_ACCEL_SAMPLE_SHIFT  3

// Axes
#define KALG_N_AXES 3
#define KALG_AXIS_X 0
#define KALG_AXIS_Y 1
#define KALG_AXIS_Z 2

// For each minute, take a weighted integral of the N minutes before and after it.
#define KALG_SLEEP_HALF_WIDTH  4
#define KALG_SLEEP_FILTER_WIDTH  (2 * KALG_SLEEP_HALF_WIDTH + 1)

// 2^7 = 128 elements > 125 to allow fft
#define KALG_FFT_WIDTH  128

// 2^7 = 128 elements > 125 to allow fft
static const int16_t KALG_FFT_WIDTH_PWR_TWO = 7;

// scaled needed to prevent overflow in adding
static const uint32_t KALG_VECTOR_MAG_COUNTS_SCALE = 10;

// 125*500/2 = 25000 prevent overflow on the transforms, assuming +-250
static const int16_t KALG_FFT_SCALE = 2;

// convert the raw pim cpm to the actigraph vmcpm
// used for both VMCPM and CPM (cause a linear relation)
static const  uint32_t KALG_x100_RAW_1G_PIM_CPM_TO_REAL_CPM = 2408;

// How much we quantize the angle when returning orientation
static const uint32_t KALG_NUM_ANGLES = 16;

// Min and Max stepping frequency
static const int KALG_MIN_STEP_FREQ = 7;
static const int KALG_MAX_STEP_FREQ = 20;

// Size of butterworth filter used in prv_pim_filter
#define KALG_BUTTERWORTH_NUM_COEFICIENTS 5

// Used to indicate that we have not yet detected a potential starting point for a step activity
#define KALG_START_TIME_NONE 0

// State information for the walk activity detection
typedef struct {
  time_t start_time;           // potential start time of the activity
                               // default: KALG_START_TIME_NONE
  int inactive_minute_count;   // how many inactive minutes in a row we have detected
  uint16_t steps;              // summed steps
  uint32_t resting_calories;   // summed resting calories
  uint32_t active_calories;    // summed active calories
  uint32_t distance_mm;        // summed distance

  HRMSessionRef hrm_session;   // current hrm session
} KAlgStepActivityState;

// Returned by prv_get_step_activity_attributes() and used for classifying activities
typedef struct {
  uint16_t min_steps_per_min;
  uint16_t max_steps_per_min;
} KAlgActivityAttributes;


// ----------------------------------------------------------------------------------------
// Sleep detection structures
// The data for each minute that we use for computing sleep
typedef struct {
  uint16_t vmc;
  uint8_t orientation;
  bool definitely_not_worn;
} KAlgSleepMinute;

// State information for sleep detection
typedef struct {
  time_t start_time;                   // KALG_START_TIME_NONE if no start detected yet
  uint16_t num_non_zero_minutes;
  uint32_t vmc_sum;
  uint16_t consecutive_sleep_minutes;
  uint16_t consecutive_awake_minutes;
} KAlgSleepActivityStats;

typedef struct {
  // We do a convolution of encoded VMC values to get a score. This convolution requires
  // the KALG_SLEEP_HALF_WIDTH entries that come before and after the center point.
  uint8_t num_history_entries;              // how many entries are in vmc_history and orientation
  KAlgSleepMinute minute_history[KALG_SLEEP_FILTER_WIDTH];

  KAlgSleepActivityStats current_stats;
  KAlgOngoingSleepStats summary_stats;
  time_t last_sample_utc;
} KAlgSleepActivityState;

// Params used for sleep detection
typedef struct {
  // If the weighted integral of the VMCs around a minute (the score) is <= this value, it is
  // considered a "sleep minute"
  uint16_t max_sleep_minute_score;

  // If the weighted integral is greater than this for even 1 minute, we assume we are awake
  uint16_t force_wake_minute_score;

  // If the VMC is greater than this for even 1 minute, we assume we are awake
  uint16_t force_wake_minute_vmc;

  // If we see at least this many "sleep minutes", the sleep has started
  uint16_t min_sleep_minutes;

  // If we see at least this many "wake minutes" in a row, the sleep has ended
  uint16_t max_wake_minute_early_offset;  // before this duration, it is "early" in the sleep
  uint16_t max_wake_minutes_early;        // early in the session
  uint16_t max_wake_minutes_late;         // later in the session

  // Minimum sleep cyle length
  uint16_t min_sleep_cycle_len_minutes;

  // If we see a scores less than this value, we consider it a "zero" (no movement)
  uint16_t min_valid_vmc;

  // To count as a sleep cycle, the activity minutes must be fairly sparse
  uint16_t max_active_minutes_pct;

  // To count as a sleep cycle, the average VMC must be below this
  uint16_t max_avg_vmc;

  // We clip VMC's to this value when computing the average. This is so high activity
  // right after waking won't screw up the previous sleep session
  uint16_t vmc_clip;

  // We only start checking the percent of active minutes and average VMC when the sleep cycle
  // is at least this long
  uint16_t min_sleep_len_for_active_pct_check;
} KAlgSleepParams;


// Set the sleep parameters
#if defined(CONFIG_BOARD_ASTERIX)
static const KAlgSleepParams KALG_SLEEP_PARAMS = {
  .max_sleep_minute_score = 500,  // Increased significantly for asterix - much stricter
  .force_wake_minute_score = 8000,
  .force_wake_minute_vmc = 10000,
  .min_sleep_minutes = 5,

  .max_wake_minute_early_offset = 60,
  .max_wake_minutes_early = 14,
  .max_wake_minutes_late = 11,

  .min_sleep_cycle_len_minutes = 60,
  .min_valid_vmc = 30,  // Increased significantly for asterix
  .max_active_minutes_pct = 89,
  .max_avg_vmc = 250,  // Increased significantly for asterix
  .vmc_clip = 1000,
  .min_sleep_len_for_active_pct_check = 39,
};
#else
static const KAlgSleepParams KALG_SLEEP_PARAMS = {
  .max_sleep_minute_score = 330,
  .force_wake_minute_score = 8000,
  .force_wake_minute_vmc = 10000,
  .min_sleep_minutes = 5,

  .max_wake_minute_early_offset = 60,
  .max_wake_minutes_early = 14,
  .max_wake_minutes_late = 11,

  .min_sleep_cycle_len_minutes = 60,
  .min_valid_vmc = 20,
  .max_active_minutes_pct = 89,
  .max_avg_vmc = 180,
  .vmc_clip = 1000,
  .min_sleep_len_for_active_pct_check = 39,
};
#endif


// ----------------------------------------------------------------------------------------
// Deep Sleep detection structures
// State information for deep sleep detection
#define KALG_MAX_DEEP_SLEEP_SESSIONS 8    // Max number of deep sleep sessions per sleep session
typedef struct {
  time_t sleep_start_time;              // KALG_START_TIME_NONE if no KAlgDeepSleepAction_Start yet
  time_t deep_start_time;               // start of current deep sleep session
  uint16_t deep_score_count;            // how many deep sleep minutes in a row we've seen
  uint16_t non_deep_score_count;        // how many non-deep sleep minutes in a row we've seen
  bool ok_to_register;                  // if true, OK to register deep sleep session

  // List of deep sleep sessions we have detected. We don't actually register them until
  // we get notified by the sleep state machine that the current sleep session has ended
  // and is valid;
  uint8_t  num_sessions;                 // number of sessions we have detected
  uint16_t start_delta_sec[KALG_MAX_DEEP_SLEEP_SESSIONS];  // delta from sleep_start_time
  uint16_t len_m[KALG_MAX_DEEP_SLEEP_SESSIONS];
} KAlgDeepSleepActivityState;

// Actions that can be sent to the deep sleep state machine (prv_deep_sleep_update)
typedef enum {
  KAlgDeepSleepAction_Start,         // started a new sleep session
  KAlgDeepSleepAction_Continue,      // new sample for current sleep session
  KAlgDeepSleepAction_End,           // ended the current sleep session
  KAlgDeepSleepAction_Abort,         // aborted the current sleep session
} KAlgDeepSleepAction;

// Params used for deep sleep detection
typedef struct {
  // If we see a scores <= than this value, we consider it deep sleep
  uint16_t max_deep_score;

  // We define deep sleep has having runs of at least min_deep_score_count minutes with
  // low sleep scores (< max_deep_score) and no more than 1 high score between runs
  uint16_t min_deep_score_count;
  uint16_t min_minutes_after_sleep_entry;
} KAlgDeepSleepParams;


// Set the deep sleep parameters
static const KAlgDeepSleepParams KALG_DEEP_SLEEP_PARAMS = {
  .max_deep_score = 160,
  .min_deep_score_count = 20,
  .min_minutes_after_sleep_entry = 10,
};


// ----------------------------------------------------------------------------------------
// Not-worn detection structures
// State information for not-worn detection
#define KALG_NUM_NOT_WORN_SECTIONS  3
typedef struct {
  uint16_t maybe_not_worn_count;            // how many -"maybe" worn minutes in a row we've seen

  uint8_t prev_orientation;
  uint16_t prev_vmc;

  // Not worn sections. Index 0 has the current (most recent one)
  time_t potential_not_worn_start[KALG_NUM_NOT_WORN_SECTIONS];
  uint16_t potential_not_worn_len_m[KALG_NUM_NOT_WORN_SECTIONS];
} KAlgNotWornState;

// Params used for not worn detection
typedef struct {
  // If the VMC is higher than this, assume the watch is definitely being worn
  uint16_t max_non_worn_vmc;

  // If the VMC is less than this, assume not-worn
  uint16_t min_worn_vmc;

  // If the candidate section is longer than this, it is definitely a not-worn section
  uint16_t max_low_vmc_run_m;
} KAlgNotWornParams;


// Set the not-worn parameters
#if defined(CONFIG_BOARD_ASTERIX)
static const KAlgNotWornParams KALG_NOT_WORN_PARAMS = {
  .max_non_worn_vmc = 2500,
  .min_worn_vmc = 15,  // Increased significantly for asterix - much higher baseline noise
  .max_low_vmc_run_m = 120,  // Allow longer stillness during deep sleep while still catching desk time
};
#else
static const KAlgNotWornParams KALG_NOT_WORN_PARAMS = {
  .max_non_worn_vmc = 2500,
  .min_worn_vmc = 4,
  .max_low_vmc_run_m = 180,
};
#endif


// ---------------------------------------------------------------------------------------------
// State variables. Must be allocated by caller and initialized by kalg_init()
typedef struct KAlgState {
  // Character array used for formatting time for log messages
  char log_time_fmt[8];

  // Accel samples, separated into 3 separate axes. Note that we do an FFT in place on this
  // array, so it must be >= KALG_N_SAMPLES_EPOCH in size.
  int16_t accel_samples[KALG_N_AXES][KALG_FFT_WIDTH];
  uint16_t num_samples;

  // Work array, used for holding magnitude at each sample point
  int16_t work[KALG_FFT_WIDTH];

  // Summary period (1 minute) statistics
  int16_t summary_mean[KALG_N_AXES];
  uint32_t summary_pim[KALG_N_AXES];  // pim: "Proportional Integral Mode"

  // epoch index, mod 256. Used for subtracting an average of 0.5 from the step count
  uint8_t epoch_idx;

  // Used for adjusting steps when we first start/stop moving
  uint8_t prev_5s_steps;
  bool prev_partial_steps;

  // Stats callback
  KAlgStatsCallback stats_cb;

  // Butterworth filter state used in prv_pim_filter.
  Fixed_S64_32 yt[KALG_N_AXES][KALG_BUTTERWORTH_NUM_COEFICIENTS - 1];
  Fixed_S64_32 xt[KALG_N_AXES][KALG_BUTTERWORTH_NUM_COEFICIENTS];
  bool pim_filter_primed;   // Right after init, we need to "prime" the filter

  // State for the activity detectors
  KAlgStepActivityState walk_state;
  KAlgStepActivityState run_state;
  KAlgSleepActivityState sleep_state;
  KAlgDeepSleepActivityState deep_sleep_state;
  KAlgNotWornState not_worn_state;

  // Timestamp of the last minute of data passed to kalg_activities_update()
  time_t last_activity_update_utc;

  bool disable_activity_session_tracking; // If true don't automatically track activities
} KAlgState;


// ----------------------------------------------------------------------------------------
// Print a timestamp in a format useful for log messages (for debugging). This only prints
// the hour and minute: HH:MM
static const char* prv_log_time(KAlgState *alg_state, time_t utc) {
  int minutes = (utc / SECONDS_PER_MINUTE) % MINUTES_PER_HOUR;
  int hours = (utc / SECONDS_PER_HOUR) % HOURS_PER_DAY;

  snprintf(alg_state->log_time_fmt, sizeof(alg_state->log_time_fmt), "%02d:%02d", hours, minutes);
  return alg_state->log_time_fmt;
}

// ----------------------------------------------------------------------------------------
// Drop the HRM subscription held by a step activity, if it has one. Safe to call on a state that
// stays in progress: the next active minute re-subscribes.
static void prv_release_step_activity_hrm(KAlgStepActivityState *state) {
#ifdef CONFIG_HRM
  if (state->hrm_session != HRM_INVALID_SESSION_REF) {
    sys_hrm_manager_unsubscribe(state->hrm_session);
    state->hrm_session = HRM_INVALID_SESSION_REF;
  }
#endif
}

// ----------------------------------------------------------------------------------------
static void prv_reset_step_activity_state(KAlgStepActivityState *state) {
  prv_release_step_activity_hrm(state);
  *state = (KAlgStepActivityState) { };
}

// ----------------------------------------------------------------------------------------
static void prv_reset_state(KAlgState *state) {
  // Release the HRM unconditionally. The state reset below skips in-progress activities, but the
  // subscription must not outlive the reset: a state machine that stops running can never reach
  // prv_reset_step_activity_state, leaving the sensor pinned on until reboot.
  prv_release_step_activity_hrm(&state->walk_state);
  prv_release_step_activity_hrm(&state->run_state);

  // Only reset step activity states if they're not currently in progress to avoid
  // race conditions with the minute handler
  if (state->walk_state.start_time == KALG_START_TIME_NONE) {
    prv_reset_step_activity_state(&state->walk_state);
  }
  if (state->run_state.start_time == KALG_START_TIME_NONE) {
    prv_reset_step_activity_state(&state->run_state);
  }
  state->sleep_state = (KAlgSleepActivityState){};
  state->deep_sleep_state = (KAlgDeepSleepActivityState){};
  state->not_worn_state = (KAlgNotWornState){};
}

// -----------------------------------------------------------------------------------------
// Compute the mean of an array of int16's
static int32_t prv_mean(int16_t *d, int16_t dlen, int16_t scale) {
  int32_t mean = 0;

  for (int16_t i = 0; i < dlen; i++) {
    mean += d[i];
  }
  return mean * scale / dlen;
}


// -----------------------------------------------------------------------------------------
static uint32_t prv_isqrt(uint32_t x) {
  uint32_t op, res, one;

  op = x;
  res = 0;

  // "one" starts at the highest power of four <= than the argument.
  one = 1 << 30;  // second-to-top bit set
  while (one > op) one >>= 2;

  while (one != 0) {
    if (op >= res + one) {
      op -= res + one;
      res += one << 1;  // <-- faster than 2 * one
    }
    res >>= 1;
    one >>= 2;
  }
  return res;
}


// -------------------------------------------------------------------------------------------
// Integrate the abs(d) between given start and end index
static int32_t prv_integral_abs(int16_t *d, int16_t start, int16_t end) {
  int32_t int_abs = 0;

  for (int16_t i = start; i <= end; i++) {
    int_abs += abs(d[i]);
  }
  return int_abs;
}

// -----------------------------------------------------------------------------------------
// Return the sum(abs(x-mean)) for each x in the array
static uint32_t prv_pim_filter(KAlgState *state, int16_t *d, int16_t dlen, int16_t axis) {
  // We use a butterworth second order digital fitler with a bandpass
  // design of 0.25 to 1.75 hz
  static const Fixed_S64_32 cb[KALG_BUTTERWORTH_NUM_COEFICIENTS] = {
      {0x000000000721d150LL},   //  0.027859766117136
      {0x0000000000000000LL},   //  0.0
      {0xfffffffff1bc5d60LL},   // -0.055719532234272
      {0x0000000000000000LL},   //  0.0
      {0x000000000721d150LL}};  //  0.027859766117136
  static const Fixed_S64_32 ca[KALG_BUTTERWORTH_NUM_COEFICIENTS - 1] = {
      {0xfffffffc92b0910cLL},   // -3.426993307709624
      {0x0000000473f9a693LL},   //  4.453028117259779
      {0xfffffffd633c7d23LL},   // -2.612358264068663
      {0x0000000096405b5cLL}};  //  0.586919508061190

  int32_t pim = 0;
  for (int16_t i = 0; i < dlen; i++) {
    Fixed_S64_32 ytmp = math_fixed_recursive_filter(
        FIXED_S64_32_FROM_INT(d[i]), KALG_BUTTERWORTH_NUM_COEFICIENTS,
        KALG_BUTTERWORTH_NUM_COEFICIENTS - 1, cb, ca, state->xt[axis], state->yt[axis]);
    pim += abs(FIXED_S64_32_TO_INT(ytmp));
  }

  // REMEMBER, the scoring is done on the 1 SECOND level, so we
  // ONLY do thresholding at the 1 second level.
  const int32_t k_x1000_thres = 3750; // this is calibrated to pebble, 125 = 1G
  return (uint32_t) (((pim - (k_x1000_thres * dlen) / 1000) > 0)
                     ? (pim - (k_x1000_thres * dlen) / 1000)
                     : 0);
}


// -----------------------------------------------------------------------------------------
// Prime the butterworth filter used in the pim filter. This helps reduce the high VMC
// produced from the first set of samples fed in right after the algorithm has been initialized.
// It works by priming the butterworth filter with an odd-symmetric extension of the first few
// samples. These priming samples have roughly the same frequency characteristics as
// the first set of samples. Since the butterworth filter's memory is 5 samples, the priming
// sequence must be at least 10 long.
// If the first few real samples are:
//    10, 13, 9, 15, 6, ...
// Then the priming samples would be:
//    14, 5, 11, 7
// The value for priming sample i, based on N is:
//    p[i] = x[0] - (x[N-1-i] - x[0]) - x[0]
//    p[i] = 2 * x[0] - x[N-1-i]
// So, in the above example, when N is 5:
//    p[0] = 2 * 10 - 6 = 14
//    p[1] = 2 * 10 - 15 = 5
//    p[2] = 2 * 10 - 9 = 11
//    p[3] = 2 * 10 - 13 = 7
static void prv_pim_filter_prime(KAlgState *state, int16_t *d, int16_t dlen, int16_t axis) {
  const int n = 11;
  int16_t prime_data[n];

  for (int i = 0; i < n; i++) {
    prime_data[i] = 2 * d[0] - d[n - 1 - i];
  }
  prv_pim_filter(state, prime_data, n - 1, axis);
}


// -----------------------------------------------------------------------------------------
// Compute real counts from our internal raw counts
static uint32_t prv_real_counts_from_raw(uint32_t raw) {
  // The Pebble's raw accel readings have 1000 = 1G. We divide each reading by 8 though, so
  // 125 = 1G. We have empirically determined that scaling the VMC by
  // KALG_x100_RAW_1G_PIM_CPM_TO_REAL_CPM / 100 produces values equivalent to the Actigraph values.
  // So, to convert from raw VMC to real VMC, we need
  // to multiply by KALG_x100_RAW_1G_PIM_CPM_TO_REAL_CPM/100 and divide by 125 and we acccomplish
  // this in integer arithmetic by multiplying by KALG_x100_RAW_1G_PIM_CPM_TO_REAL_CPM and
  // dividing by 12500.
  uint32_t real_counts = raw * KALG_x100_RAW_1G_PIM_CPM_TO_REAL_CPM / 12500;
  return real_counts;
}


// -----------------------------------------------------------------------------------------
// Real-valued, in-place, 2-radix Fourier transform
//
//   This implementation of the fourier transform is taken directly from
//   Henrik V. Sorensen's 1987 paper "Real-valued Fast Fourier Tranform
//   Algorithms" with slight modifications to allow use of Pebble's cos and
//   sin lookup functions with input range of 0 to 2*pi angle scaled to
//   0 to 65536 and output range of -1 to 1 scaled to -65535 to 65536. This
//   descretization introduces some discrepancies between the results of this
//   function and the floating point equivalents that are not important for its
//   use here, but nonetheless documented in the accompaning Julia test code.
//
//   INPUT
//     d = input signal array pointer
//     width the width of d (must be a power of 2)
//     width_log_2 the log base 2 of width: 2^width_log_2 = width
//
//   OUTPUT
//     d = fourier tranformed array pointer, with array of real coefficents of form
//       [Re(0), Re(1),..., Re(N/2-1), Re(N/2), Im(N/2-1),..., Im(1)]
//
static void prv_fft_2radix_real(int16_t *d, int16_t width, int16_t width_log_2) {
  int16_t n = width;
  int16_t j = 1;
  int16_t n1 = n -1;
  int16_t k, dt;

  for (int16_t i = 1; i <= n1; i++) {
    if (i < j) {
      dt = d[j-1];
      d[j-1] = d[i-1];
      d[i-1] = dt;
    }
    k = n/2;
    while (k < j) {
      j = j - k;
      k = k / 2;
    }
    j = j + k;
  }

  for (int16_t i = 1; i <= n; i += 2) {
    dt = d[i-1];
    d[i-1] = dt + d[i];
    d[i] = dt - d[i];
  }

  int16_t n2 = 1;
  int16_t n4, i1, i2, i3, i4, t1, t2;
  int32_t E, A, ss, cc;

  for (int16_t k = 2; k <= width_log_2 ; k++) {
    n4 = n2;
    n2 = 2 * n4;
    n1 = 2 * n2;
    E = TRIG_MAX_ANGLE / n1;

    for (int16_t i = 1; i<= n; i+=n1) {
      dt = d[i-1];
      d[i-1] = dt + d[i+n2-1];
      d[i+n2-1] = dt - d[i+n2-1];
      d[i+n4+n2-1] = -1 * d[i+n4+n2-1];
      A = E;
      for (int16_t j = 1; j <= (n4-1); j++) {
        i1 = i + j;
        i2 = i - j + n2;
        i3 = i + j + n2;
        i4 = i - j + n1;

        ss = sin_lookup(A);
        cc = cos_lookup(A);

        A = A + E;

        t1 = (int16_t) ((d[i3-1] * cc + d[i4-1] * ss) / TRIG_MAX_ANGLE);
        t2 = (int16_t) ((d[i3-1] * ss - d[i4-1] * cc) / TRIG_MAX_ANGLE);

        d[i4-1] = d[i2-1] - t2;
        d[i3-1] = -d[i2-1] - t2;
        d[i2-1] = d[i1-1] - t1;
        d[i1-1] = d[i1-1] + t1;
      }
    }
  }
}


// -----------------------------------------------------------------------------------------
// Evaluate the magnitude of the FFT coefficents and write back to the first width/2 elements
// NOTE! this function modifies the input array in place
static void prv_fft_mag(int16_t *d, int16_t width) {
  // evaluate the fourier coefficent magnitude
  // NOTE: coeff @ index 0 and width/2 only have real components
  //    so their magnitude is exactly that
  for (int16_t i = 1; i < (width / 2); i++) {
    // NOTE: eval coeff mag for real and imag : R(i) & I(i)
    d[i] = prv_isqrt(d[i] * d[i] + d[width - i] * d[width - i]);
  }
}


#if LOG_DOMAIN_ACTIVITY && KALG_LOG_AXIS_MAGNITUDES
// -------------------------------------------------------------------------------------------
// Print a text graph of the values in the d array
static void prv_text_graph(const char *type_str, int16_t *d, int16_t start, int16_t end) {
#ifdef UNITTEST
  // Log the values to facilitate plotting
  printf("\nRaw values for plotting: [");
  for (int i = start; i < end; i++) {
    printf("%d, ", d[i]);
  }
  printf("%d]\n", d[end]);
#endif

  // Find the max value
  int16_t max_value = 0;
  for (int16_t i = start; i <= end; i++) {
    max_value = MAX(max_value, abs(d[i]));
  }

  // Create a string of stars for the bar graph
  const uint32_t k_max_stars = 40;
  char stars_str[k_max_stars + 2];
  for (uint32_t j = 0; j <= k_max_stars; j++) {
    stars_str[j] = '*';
  }
  stars_str[k_max_stars + 1] = 0;

  // Print each frequency with a bar
  for (int16_t i = start; i <= end; i++) {
    uint32_t num_stars = 0;
    if (max_value > 0) {
      num_stars = k_max_stars * abs(d[i]) / max_value;
    }
    num_stars = MIN(num_stars, k_max_stars);
    stars_str[num_stars] = 0;
    KALG_LOG_DEBUG("%s: %3"PRIi16": mag: %+3"PRIi16": %s", type_str, i, d[i], stars_str);
    stars_str[num_stars] = '*';
  }
}
#endif


// -------------------------------------------------------------------------------------------
// Log all magnitudes of the overall FFT
static void prv_log_overall_magnitudes(const char *type_str, int16_t *d, int16_t start,
                                       int16_t end) {
#if LOG_DOMAIN_ACTIVITY && KALG_LOG_OVERALL_MAGNITUDES
  prv_text_graph(type_str, d, start, end);
#endif
}


// -------------------------------------------------------------------------------------------
// Used to Log magnitudes of a specific axis
static void prv_log_axis_magnitudes(const char *type_str, int16_t *d, int16_t start, int16_t end) {
#if LOG_DOMAIN_ACTIVITY && KALG_LOG_AXIS_MAGNITUDES
  prv_text_graph(type_str, d, start, end);
#endif
}


// -----------------------------------------------------------------------------------------
static void prv_get_fftmag_0pad_mean0(int16_t *d, int16_t num_samples, int16_t fft_width,
                                      int16_t fft_width_log_2, int16_t input_scale) {
  // reduce input magnitudes before taking FFT
  for (int16_t i = 0; i < num_samples; i++) {
    d[i] = d[i] / input_scale;
  }

  // set the last few elements to the mean of the first elements
  int16_t mean = prv_mean(d, num_samples, 1);
  for (int16_t i = 0 ; i < num_samples; i++) {
    d[i] = d[i] - mean;
  }
  for (int16_t i = num_samples ; i < fft_width; i++) {
    d[i] = 0;
  }

  // Compute the FFT coefficients
  prv_fft_2radix_real(d, fft_width, fft_width_log_2);

  // Evaluate the magnitude of the coefficents and write back to the first fft_width/2
  // elements
  prv_fft_mag(d, fft_width);
}


// -----------------------------------------------------------------------------------------
// Apply a cosine filter to the given data array. This is often used before taking an FFT.
// Taking an FFT of a finite length sequence is mathematically like stacking the sequence end to
// end and then computing a regular FT. If the sequence end and beginning values are not the same
// value, this results in a discontinuity where it is stacked, resulting in the introduction of
// high frequencies in the FFT output. A cosine filter forces the start and end of the sequence to
// both taper off to 0.
static void prv_filt_cosine_win_mean0(int16_t *d, int16_t width, int32_t g_factor) {
  int32_t d_mean = prv_mean(d, width, 1);

  for (uint16_t i = 0; i < width; i++) {
    d[i] = (int16_t) (((d[i] - d_mean) * g_factor *
                      sin_lookup((TRIG_MAX_ANGLE * i) / (2 * width))) / (TRIG_MAX_RATIO));
  }
}


// -----------------------------------------------------------------------------------------
// Find the frequency with the maximum magnitude between lhz and hhz. If favor_low is set, then
// apply a dampening function to slightly favor lower frequencies over higher ones. Also, return
// the energy of that frequency in *energy.
// Since energy can be spread across 2 adjacent coefficients, setting inc_adjacent will sum
// adjacent ones and find the max that way.
static int16_t prv_max_mag(int16_t *d, int16_t lhz, int16_t hhz, bool favor_low, bool inc_adjacent,
                           uint32_t *energy) {
  // evaluate if the period is a step epoch, based on score
  int32_t max_hz_val = 0;
  int32_t max_hz_energy = 0;
  int16_t max_hz_i = lhz;

  // Find the hz index with largest mag.
  for (int16_t i = lhz; i <= hhz; i++) {
    int16_t val0 = abs(d[i]);
    int16_t val1 = 0;
    if (inc_adjacent) {
      val1 = abs(d[i + 1]);
    }
    int16_t test_hz = val1 > val0 ? i + 1 : i;
    if (test_hz > hhz) {
      // We evaluate as far as hhz + 1, skip if the peak ends up past the end
      continue;
    }

    int32_t test_energy = val0 + val1;
    int32_t test_val = test_energy;
    if (favor_low) {
      const int k_dampening = 100; // lower values dampen the high frequencies.
      // In this formula, the higher the frequency, the more it is dampened. A frequency of
      // 0 has no damping.
      test_val = test_val * (k_dampening - test_hz) / k_dampening;
    }

    if (test_val > max_hz_val) {
      max_hz_val = test_val;
      max_hz_i = test_hz;
      max_hz_energy = test_energy;
    }
  }

  // DC index is 0, so max_hz_i is HZ directly
  *energy = max_hz_energy;
  return max_hz_i;
}


// -----------------------------------------------------------------------------------------
// Compute scaled Vector Magnitude Counts (vmc)
// This function calculates the vector magnitude counts from the proportional integral mode array.
//
// @param pims  the array of counts, one for each axis
// @return the sqrt of the sum of the squares
static uint32_t prv_calc_raw_vmc(uint32_t *pims) {
  uint32_t d[KALG_N_AXES];

  // cap to prevent overflow when prop_integrals[:].^2 is summed
  const uint32_t max_value = 37500;

  // We divide by KALG_VECTOR_MAG_COUNTS_SCALE first to avoid an overflow while adding and
  // squaring then multiple it back in after the sqrt.
  for (int16_t axis = 0; axis < KALG_N_AXES; axis++) {
    d[axis] = pims[axis] / KALG_VECTOR_MAG_COUNTS_SCALE;
    d[axis] = (d[axis] < max_value) ? d[axis] : max_value;
  }

  // calculate VMCPM, then take sqrt to compress
  return KALG_VECTOR_MAG_COUNTS_SCALE * prv_isqrt(d[KALG_AXIS_X] * d[KALG_AXIS_X]
                                                  + d[KALG_AXIS_Y] * d[KALG_AXIS_Y]
                                                  + d[KALG_AXIS_Z] * d[KALG_AXIS_Z]);
}


// -----------------------------------------------------------------------------------------
// Compute the magnitude of the signal based on the given walking frequency. This sums
// the energy of the walking frequency, the arm frequency, and each of their harmonics.
// @param[in] d pointer to array of magnitudes
// @param[in] d_len length of d array
// @param[in] walk_hz which walking frequency to evalute
// @param[in] log log debugging information for this specific walking frequency
// @return the sum of the magntudes of the signal frequencies
static uint32_t prv_compute_signal_energy(int16_t *d, int16_t d_len, uint16_t walk_hz, bool log) {
  static const int k_min_arm_freq = 5;

  // Find the frequency with the highest magnitude within the walking range
  uint32_t walk_energy;
  prv_max_mag(d, walk_hz, walk_hz, false /*favor_low*/, false /*inc_adjacent*/, &walk_energy);

  // When walking at a decent rate, we can get an energy spike at half the walking rate due to
  // the arm swinging motion, so add that in as well
  uint32_t arm_energy = 0;
  uint16_t arm_hz = prv_max_mag(d, (walk_hz / 2) - 1, (walk_hz / 2) + 1, false /*favor_low*/,
                                false /*inc_adjacent*/, &arm_energy);

  // Include the 3rd harmonic of the arm
  uint32_t arm_3_energy = 0;
  uint16_t arm_3_hz = prv_max_mag(d, walk_hz + arm_hz - 1, walk_hz + arm_hz + 1,
                                  false /*favor_low*/, false /*inc_adjacent*/, &arm_3_energy);

  // Include the 2nd harmonic of the walking frequency
  uint32_t walk_2_energy;
  uint16_t walk_2_hz = prv_max_mag(d, (walk_hz * 2) - 1, (walk_hz * 2) + 1,
                                   false /*favor_low*/, false /*inc_adjacent*/, &walk_2_energy);

  // Include the 5th harmonic of the arm
  uint32_t arm_5_energy = 0;
  uint16_t arm_5_hz = prv_max_mag(d, (2 * walk_hz) + arm_hz - 1, (2 * walk_hz) + arm_hz + 1,
                                  false /*favor_low*/, false /*inc_adjacent*/, &arm_3_energy);

  // Include the 3rd harmonic of the walking frequency
  uint32_t walk_3_energy;
  uint16_t walk_3_hz = prv_max_mag(d, walk_hz + walk_2_hz - 1, walk_hz + walk_2_hz + 1,
                                   false /*favor_low*/, false /*inc_adjacent*/, &walk_3_energy);

  // Include the 4th harmonic of the walking frequency
  uint32_t walk_4_energy;
  uint16_t walk_4_hz = prv_max_mag(d, walk_3_hz + walk_hz - 1, walk_3_hz + walk_hz + 1,
                                   false /*favor_low*/, false /*inc_adjacent*/, &walk_4_energy);

  // Include the 5th harmonic of the walking frequency
  uint32_t walk_5_energy;
  uint16_t walk_5_hz = prv_max_mag(d, walk_4_hz + walk_hz - 1, walk_4_hz + walk_hz + 1,
                                   false /*favor_low*/, false /*inc_adjacent*/, &walk_5_energy);

  // If the arm frequency is very low, ignore it. Non-stepping activities can have significant
  // energy at these low frequencies.
  if (arm_hz < k_min_arm_freq) {
    arm_energy = 0;
  }

  // Compute the total energy of this signal
  uint32_t max_mag_energy = walk_energy + arm_energy + arm_3_energy + walk_2_energy + arm_5_energy
                          + walk_3_energy + walk_4_energy + walk_5_energy;
  if (log) {
    KALG_LOG_DEBUG(
        "walk:%"PRIu16",%"PRIu32"  arm: %"PRIu16",%"PRIu32"  ",
         walk_hz, walk_energy, arm_hz, arm_energy);
    KALG_LOG_DEBUG("arm3:%"PRIu16",%"PRIu32"   walk2:%"PRIu16",%"PRIu32"  arm5:%"PRIu16",%"PRIu32
                   "  ", arm_3_hz, arm_3_energy, walk_2_hz, walk_2_energy, arm_5_hz, arm_5_energy);
    KALG_LOG_DEBUG(
        "walk3:%"PRIu16",%"PRIu32"  walk4:%"PRIu16",%"PRIu32"  walk5:%"PRIu16",%"PRIu32"  ",
         walk_3_hz, walk_3_energy, walk_4_hz, walk_4_energy, walk_5_hz, walk_5_energy);
  }

  return max_mag_energy;
}


// -----------------------------------------------------------------------------------------
// Compute the most likely walking frequency and its score for this epoch. This searches for the
// max magnitude among the possible walking frequencies and computes the energy of the walking
// frequency (and its harmonics) relative to all other frequencies to generate the score.
// @param[in] d pointer to array of magnitudes
// @param[in] real_vmc_5s vmc for this epoch
// @param[in] d_len length of d array
// @param[out] *score_0_ret the score of the walking frequency
// @param[out] *score_hf the score of the high frequency components relative to the walking
//              frequency
// @param[out] *score_lf_ret the score of the low frequency components relative to the walking
//              frequency
// @param[out] *total_ret the total energy
// @return the walking frequency
static uint16_t prv_compute_scores(int16_t *d, uint32_t real_vmc_5s, int16_t d_len,
                                   uint16_t *score_0_ret, uint16_t *score_hf_ret,
                                   uint16_t *score_lf_ret, int32_t *total_ret) {
  static const int k_high_freq_min = 50;
  static const int k_low_freq_max = 4;

  // If VMC is below this slow walk threshold, we look for a max hz <= k_slow_walk_max_hz
  static const unsigned int k_slow_walk_max_vmc = 340;
  static const int k_slow_walk_max_hz = 10;

  // If VMC is below this med walk threshold, we look for a max hz <= k_med_walk_max_hz
  static const unsigned int k_med_walk_max_vmc = 2000;
  static const int k_med_walk_max_hz = 12;

  // For very high VMC's (only seen when running), we look for a max hz >= k_run_min_hz. Ignoring
  // the lower frequencies reduces the chance that we might confuse the arm-swing for the step
  // frequency.
  static const int k_run_min_hz = 10;

  // If the VMC is above this minimum running VMC, we do an extra search for a significantly
  // higher energy at the running frequency than we found at the normal walking frequency
  static const unsigned int k_min_run_vmc = 1000;

  // Find the frequency with the highest magnitude within the stepping range. The allowed
  // stepping range changes based on the VMC
  uint16_t min_allowed_hz;
  uint16_t max_allowed_hz;
  if (real_vmc_5s < k_slow_walk_max_vmc) {
    // Slow walk
    min_allowed_hz = KALG_MIN_STEP_FREQ;
    max_allowed_hz = k_slow_walk_max_hz;
  } else if (real_vmc_5s < k_med_walk_max_vmc) {
    // Medium speed walk
    min_allowed_hz = KALG_MIN_STEP_FREQ;
    max_allowed_hz = k_med_walk_max_hz;
  } else {
    // Run
    min_allowed_hz = k_run_min_hz;
    max_allowed_hz = KALG_MAX_STEP_FREQ;
  }
  uint32_t walk_energy;
  uint16_t center_hz = prv_max_mag(d, min_allowed_hz, max_allowed_hz,
                                   false /*favor_low*/, false /*inc_adjacent*/, &walk_energy);

  // Most runs will be in the high VMC range, but there is a chance that a run will show up in
  // the "medium" VMC range and we only latched onto the arm-swing signal. If we are in the
  // medium VMC range, let's see if there is a significantly stronger signal at a higher frequency
  // which would indicate a run.
  if ((real_vmc_5s >= k_min_run_vmc) && (max_allowed_hz < KALG_MAX_STEP_FREQ)) {
    uint32_t test_energy;
    uint16_t higher_hz = prv_max_mag(d, max_allowed_hz, KALG_MAX_STEP_FREQ,
                                     false /*favor_low*/, false /*inc_adjacent*/, &test_energy);
    if (test_energy > (walk_energy * 3 / 2)) {
      center_hz = higher_hz;
      max_allowed_hz = KALG_MAX_STEP_FREQ;
    }
  }

  // Let's scan around that frequency till we find the max energy
  uint16_t walk_hz = center_hz;
  uint32_t max_mag_energy = 0;
  for (uint16_t test_hz = center_hz - 2; test_hz <= center_hz + 2; test_hz++) {
    if ((test_hz < min_allowed_hz) || (test_hz > max_allowed_hz)) {
      continue;
    }
    uint32_t energy = prv_compute_signal_energy(d, d_len, test_hz, false /*log*/);
    if (energy > max_mag_energy) {
      max_mag_energy = energy;
      walk_hz = test_hz;
    }
  }

  // Log what we found
  if (LOG_DOMAIN_ACTIVITY) {
    prv_compute_signal_energy(d, d_len, walk_hz, true /*log*/);
  }

  uint16_t score_0 = 0;
  int32_t total_energy = prv_integral_abs(d, 0, d_len - 1);
  if (total_energy > 0) {
    score_0 = max_mag_energy * 100 / total_energy;
  }

  // Get the percent energy at high frequencies. A high amount here is a good indication of
  // driving in the car (which we want to ignore).
  int16_t score_high_freq = 0;
  if (max_mag_energy > 0) {
    score_high_freq = 100 * prv_integral_abs(d, k_high_freq_min, d_len - 1)  / max_mag_energy;
  }

  // Get the percent energy at low frequencies. A high amount here is a good indication of
  // non-walking activities like washing up, etc.
  int16_t score_low_freq = 0;
  if (max_mag_energy > 0) {
    score_low_freq = 100 * prv_integral_abs(d, 0, k_low_freq_max) / max_mag_energy;
  }

  KALG_LOG_DEBUG("max_mag_energy: %"PRIu32", total: %"PRIi32", score_0: %"PRIu16", "
                 "score_hf: %"PRIu16", score_lf: %"PRIu16"", max_mag_energy, total_energy,
                  score_0, score_high_freq, score_low_freq);

  *score_0_ret = score_0;
  *score_hf_ret = score_high_freq;
  *score_lf_ret = score_low_freq;
  *total_ret = total_energy;
  return walk_hz;
}



// -----------------------------------------------------------------------------------------
// Return true if the score and vmc combination indicate that the user is stepping
static bool prv_is_stepping(KAlgState *state, uint16_t max_mag_hz, uint16_t score_0,
                            uint16_t score_high_freq, uint16_t score_low_freq,
                            uint32_t real_vmc_5s, int32_t total_energy, bool *partial_steps) {
  *partial_steps = false;

  // -------------------------------------------------------------------
  // Our min score and vmc thresholds for full stepping epochs and partial epochs
  const uint16_t k_min_score = 15;
  const uint16_t k_min_vmc = 135;

  const uint16_t k_partial_min_score = 9;
  const uint16_t k_partial_min_vmc = 120;

  // If the frequency is high (close to running speed), insure that the VMC is also high.
  // This can filter out some false steps if we get a high freqency and low VMC.
  static const uint32_t k_high_step_freq_threshold = 12;
  static const uint32_t k_high_step_freq_vmc = 1000;

  // Ignore if we have too much high frequency component (probably driving)
  static const int16_t k_score_high_freq_max = 120;

  // Ignore if we have too much low frequency component (probably something like washing up)
  static const int16_t k_score_low_freq_max = 145;

  // Ignore if total energy is lower than this
  static const int32_t k_min_total_energy = 1000;

  // ---------------------------------------------------------------------
  // Use a simple linear regression to scale the fft_threshold with the
  // vmc. Actually, this is quite computationally sound, we just need to shift
  // it over by a few for safety, and we can auto adjust the parameters so that
  // they can be *very* tight. This way, we can reject steps very easily.
  bool is_stepping = false;
  if ((max_mag_hz >= KALG_MIN_STEP_FREQ) && (max_mag_hz <= KALG_MAX_STEP_FREQ)) {
    if ((score_0 >= k_min_score) && (real_vmc_5s >= k_min_vmc)) {
      is_stepping = true;
    }
  }

  // Ignore if we have too much high frequency component (probably driving)
  if (score_high_freq > k_score_high_freq_max) {
    is_stepping = false;
  }

  // Ignore if we have too much low frequency component (probably something like washing up)
  if (score_low_freq > k_score_low_freq_max) {
    is_stepping = false;
  }

  // Ignore if we have a high step rate, but low vmc
  if (max_mag_hz >= k_high_step_freq_threshold && real_vmc_5s < k_high_step_freq_vmc) {
    is_stepping = false;
  }

  // Ignore if total energy is too low
  if (total_energy < k_min_total_energy) {
    is_stepping = false;
  }

  // Treatment of epochs that include the start or stop of a walk
  // If step_count is 0, see if this epoch could counts as a start/stop of walking
  if (!is_stepping && (max_mag_hz >= KALG_MIN_STEP_FREQ - 1)
                   && (max_mag_hz <= KALG_MAX_STEP_FREQ)) {
    if ((score_0 >= k_partial_min_score) && (real_vmc_5s >= k_partial_min_vmc)) {
      *partial_steps = true;
    }
  }

  return is_stepping;
}


// -----------------------------------------------------------------------------------------
// On entry the first fft_width/2 elements of state->work contain the FFT magnitudes
static uint16_t prv_calc_steps_in_epoch(KAlgState *state, int16_t num_samples, int16_t fft_width,
                                int16_t fft_width_log_2, uint32_t *pim_epoch, int16_t fft_scale) {
  // The Pebble's raw accel readings have 1000 = 1G. We divide each reading by 8 though, so
  // 125 = 1G. We have empirically determined that scaling the VMC by
  // KALG_x100_RAW_1G_PIM_CPM_TO_REAL_CPM / 100 produces values equivalent to the Actigraph values.
  // So, to convert from raw VMC to real VMC, we need
  // to multiply by KALG_x100_RAW_1G_PIM_CPM_TO_REAL_CPM/100 and divide by 125 and we acccomplish
  // this in integer arithmetic by multiplying by KALG_x100_RAW_1G_PIM_CPM_TO_REAL_CPM and
  // dividing by 12500.
  uint32_t real_vmc_5s = prv_real_counts_from_raw(prv_calc_raw_vmc(pim_epoch));

  // Find the potential walking frequency and its score
  uint16_t score_0;
  uint16_t score_hf;
  uint16_t score_lf;
  int32_t total_energy;
  uint16_t max_mag_hz = prv_compute_scores(state->work, real_vmc_5s, fft_width / 2, &score_0,
                                           &score_hf, &score_lf, &total_energy);

  // ----------------------------------------
  // See if it passes for a step epoch
  bool partial_steps = false;
  bool stepping = prv_is_stepping(state, max_mag_hz, score_0, score_hf, score_lf, real_vmc_5s,
                                  total_energy, &partial_steps);

  // ----------------------------------------
  // Adjust for ending or starting a walk
  uint16_t step_count = stepping ? max_mag_hz : 0;
  uint16_t return_steps = step_count;
  if (state->prev_partial_steps && (step_count > 0)) {
    // non-walking to walking
    return_steps += step_count / 2;
  } else if ((state->prev_5s_steps > 0) && partial_steps) {
    // walk to non-walking
    return_steps += state->prev_5s_steps / 2;
  }

  // ----------------------------------------
  // Logging output for algorithm debugging
  const char *type_str = stepping ? "STEP" : (partial_steps ? "HALF" : "----");
  KALG_LOG_DEBUG("%s steps: %2"PRIu16", freq: %2"PRIu16", vmc: %4"PRIu32", score0: %"PRIu16", ",
                 type_str, return_steps, max_mag_hz, real_vmc_5s, score_0);
  KALG_LOG_DEBUG("score_hf: %"PRIi16", score_lf: %"PRIi16", total_energry: %"PRIi32" ",
                 score_hf, score_lf, total_energy);
  prv_log_overall_magnitudes("freq", state->work, 0, (fft_width / 2) - 1 /*index of last element*/);

  // Are we collecting statistics?
  if (state->stats_cb) {
    const char *names[] = {"steps", "freq", "vmc", "score_0", "score_hf", "score_lf", "total"};
    int32_t values[] = {return_steps, max_mag_hz, real_vmc_5s, score_0, score_hf, score_lf,
                        total_energy};
    state->stats_cb(ARRAY_LENGTH(names), names, values);
  }

  // Update state
  state->prev_partial_steps = partial_steps;
  state->prev_5s_steps = step_count;
  state->epoch_idx++;

  return return_steps;
}


// -----------------------------------------------------------------------------------------
// Return an encoding of an angle, quantized into num_angles possible values
static uint8_t prv_get_angle_encoding(int16_t x, int16_t y, uint8_t num_angles) {
  // get the angle resolution
  int32_t ang_res = TRIG_MAX_ANGLE / num_angles;

  // Get the angle from the pebble lookup
  // !! MAKE SURE RANGE IS APPROPRIATE, ie -TRIG_MAX_ANGLE/2 to TRIG_MAX_ANGLE/2
  int32_t atan = atan2_lookup(y, x);

  // IF the atan lookup has any consistency whatsoever, the -pi/2 to 0
  // for the atan2 will be mapped to the pi to 2*pi geometric angles.
  // This is the only thing that makes sense for consistency across
  // the various elements
  // BUT, in case it doesn't, here is the transformation to use
  // Shift the negative angles (-TRIG_MAX_ANGLE/2 to 0) so range is 0 to TRIG_MAX_ANGLE
  //   A = A > 0 ? A : (A + TRIG_MAX_ANGLE);

  // Divide out by ang_res to get the index of the angle.
  // We need to make sure that in all cases that the returned index is at MOST one less that
  // n_ang, because 0-15 shifted by (ang_res/2) so rounds int, not floor
  int32_t result = (atan + (ang_res / 2)) / ang_res;
  return (uint8_t) result < num_angles ? result : 0;
}


// -----------------------------------------------------------------------------------------
// Analyze and return the # of steps from this epoch.
static uint32_t prv_analyze_epoch(KAlgState *state) {
  if (state->num_samples == 0) {
    return 0;
  }

  // If this is the first epoch after an init, we need to prime the butterworth filter used
  // by prv_pim_filter to avoid getting jumps in VMC due to the discontinuity
  if (!state->pim_filter_primed) {
    for (int16_t axis = 0; axis < KALG_N_AXES; axis++) {
      prv_pim_filter_prime(state, &state->accel_samples[axis][0], KALG_SAMPLE_HZ, axis);
    }
    state->pim_filter_primed = true;
  }

  // 5 sec proportional integral mode (pim), used by the steps calculation
  uint32_t pim_epoch[KALG_N_AXES] = {0};

  // Calculate the axis metrics
  for (int16_t axis = 0; axis < KALG_N_AXES; axis++) {
    // add the local mean to the global mean array, additively
    state->summary_mean[axis] += prv_mean(state->accel_samples[axis], state->num_samples, 1);

    // calculate the proportional integral mode (pim) for each second:
    // KALG_N_SAMPLES_EPOCH / KALG_SAMPLE_HZ = num of seconds in epoch
    for (int16_t sec = 0; sec < state->num_samples / KALG_SAMPLE_HZ; sec++) {
      // The proportional integral mode is roughly the sum of the absolute value of all elements
      // after subtracing the mean, then run through a filter
      uint32_t pim;
      pim = prv_pim_filter(state, &state->accel_samples[axis][sec * KALG_SAMPLE_HZ],
                           KALG_SAMPLE_HZ, axis);

      // Thresholded integral for the VMCPM calculation, Actigraph equivalent
      state->summary_pim[axis] += pim;
      pim_epoch[axis] += pim;
    }
  }

  // Calculate the magnitude of the FFT. We will compute the FFT of each axis independently and
  // then compute the magnitude of that 3-axis FFT afterwards
  for (int16_t axis = 0; axis < KALG_N_AXES; axis++) {
    prv_log_axis_magnitudes("accel-before", &state->accel_samples[axis][0], 0,
                            state->num_samples - 1 /*index of last element*/);

    // Apply a cosine filter to the data before we FFT to reduce the chance of introduing
    // false high frequency components. See the function comment for prv_filt_cosine_win_mean0()
    // for more info.
    prv_filt_cosine_win_mean0(&state->accel_samples[axis][0], state->num_samples, 1);

    prv_log_axis_magnitudes("accel-after", &state->accel_samples[axis][0], 0,
                            state->num_samples - 1 /*index of last element*/);

    prv_get_fftmag_0pad_mean0(&state->accel_samples[axis][0], state->num_samples, KALG_FFT_WIDTH,
                              KALG_FFT_WIDTH_PWR_TWO, KALG_FFT_SCALE);

    prv_log_axis_magnitudes("fft-axis", &state->accel_samples[axis][0], 0,
                            KALG_FFT_WIDTH / 2 - 1 /*index of last element*/);
  }

  // Get the magnitude of each element now
  // The first KALG_FFT_WIDTH/2 elements of the FFT output are the magnitudes. The latter half are
  // the phase
  for (int16_t i = 0; i < KALG_FFT_WIDTH / 2; i++) {
    state->work[i] = (int16_t) prv_isqrt(state->accel_samples[0][i] * state->accel_samples[0][i]
                                         + state->accel_samples[1][i] * state->accel_samples[1][i]
                                         + state->accel_samples[2][i] * state->accel_samples[2][i]);
  }

  // Calculate the step count for this epoch
  uint16_t steps = prv_calc_steps_in_epoch(
      state, state->num_samples, KALG_FFT_WIDTH, KALG_FFT_WIDTH_PWR_TWO,
      pim_epoch, KALG_FFT_SCALE);

  return steps;
}


// -----------------------------------------------------------------------------------
// Compute the sleep score by convolving the VMCs around index i. The caller is responsible
// for insuring that i is at least half the filter width from either end.
static uint32_t prv_compute_sleep_score(KAlgSleepMinute *samples, int i) {
  // We take a weighted sum of the VMC scores around each minute according to these weights
  const int weights[KALG_SLEEP_FILTER_WIDTH] = {10, 15, 28, 31, 85, 15, 10, 0, 0};
  const int weight_divisor = 100;

  uint32_t score = 0;
  for (int j = 0; j < KALG_SLEEP_FILTER_WIDTH; j++) {
    uint32_t vmc = samples[i - KALG_SLEEP_HALF_WIDTH + j].vmc;
    score += weights[j] * vmc;
  }
  score = score / weight_divisor;
  return score;
}


// -----------------------------------------------------------------------------------------
// Return the size required for the state variables
uint32_t kalg_state_size(void) {
  return sizeof(KAlgState);
}


// -----------------------------------------------------------------------------------------
// Init the state, return true on success
bool kalg_init(KAlgState *state, KAlgStatsCallback stats_cb) {
  PBL_ASSERTN(state != NULL);
  *state = (KAlgState) {
    .stats_cb = stats_cb,
  };

  PBL_ASSERT((KALG_SLEEP_PARAMS.max_wake_minutes_early + KALG_SLEEP_HALF_WIDTH + 1)
                  == KALG_MAX_UNCERTAIN_SLEEP_M, "Invalid value for KALG_MAX_UNCERTAIN_SLEEP_M");
  return true;
}


// -----------------------------------------------------------------------------------------
// Release resources held by the state. Must be called before the caller frees it, otherwise the
// HRM subscriptions outlive the only references to them and pin the sensor on until reboot.
void kalg_deinit(KAlgState *state) {
  PBL_ASSERTN(state != NULL);
  prv_release_step_activity_hrm(&state->walk_state);
  prv_release_step_activity_hrm(&state->run_state);
}


// ------------------------------------------------------------------------------------
uint32_t kalg_analyze_samples(KAlgState *state, AccelRawData *data, uint32_t num_samples,
                              uint32_t *consumed_samples) {
  PBL_ASSERTN(state != NULL);
  uint32_t new_steps = 0;
  *consumed_samples = 0;

  // We do an FFT in place on the accel_samples array, so make sure our constraints are correct
  _Static_assert(KALG_N_SAMPLES_EPOCH < KALG_FFT_WIDTH, "Invalid array sizes");

  // Format the accel data for the algorithm - it wants the x, y and z values in separate arrays
  for (uint32_t i = 0; i < num_samples; i++) {
    state->accel_samples[KALG_AXIS_X][state->num_samples]
                      = (data[i].x + KALG_ACCEL_SAMPLE_DIV / 2) >> KALG_ACCEL_SAMPLE_SHIFT;
    state->accel_samples[KALG_AXIS_Y][state->num_samples]
                      = (data[i].y + KALG_ACCEL_SAMPLE_DIV / 2) >> KALG_ACCEL_SAMPLE_SHIFT;
    state->accel_samples[KALG_AXIS_Z][state->num_samples]
                      = (data[i].z + KALG_ACCEL_SAMPLE_DIV / 2) >> KALG_ACCEL_SAMPLE_SHIFT;
    state->num_samples++;

    if (state->num_samples >= KALG_N_SAMPLES_EPOCH) {
      new_steps += prv_analyze_epoch(state);
      state->num_samples = 0;
      *consumed_samples = KALG_N_SAMPLES_EPOCH;
    }
  }

  return new_steps;
}


// ------------------------------------------------------------------------------------
void kalg_minute_stats(KAlgState *state, uint16_t *vmc, uint8_t *orientation, bool *still) {
  PBL_ASSERTN(state != NULL);
  // -----------------------------------------
  // Compute the orientation
  // We want to fit the encoding into a byte, so
  // MAX num_angles is 16, as 16*15 + 15 = 255
  // The range of the theta_i and phi_i is 0 to (n_ang-1)
  // get theta, in the x-y plane. theta relative to +x-axis
  uint8_t theta = prv_get_angle_encoding(state->summary_mean[0], state->summary_mean[1],
                                         KALG_NUM_ANGLES);

  // get phi, in the xy_vm-z plane
  int16_t xy_vm = prv_isqrt(state->summary_mean[0] * state->summary_mean[0]
                            + state->summary_mean[1] * state->summary_mean[1]);

  // phi rel to  +z-axis, so z is on hoz-axis and xy_vm is vert-axis
  uint8_t phi_i = prv_get_angle_encoding(state->summary_mean[2], xy_vm, KALG_NUM_ANGLES);
  *orientation = KALG_NUM_ANGLES * phi_i + theta;


  uint32_t real_vmc = prv_real_counts_from_raw(prv_calc_raw_vmc(state->summary_pim));
  // Clip to a max of uint16_t
  *vmc = MIN(real_vmc, UINT16_MAX);

  // If we have a way of reliably distinguishing sleep from complete stillness (watch not being
  // worn), we will set this flag.
  *still = false;

  // KALG_LOG_DEBUG("minute_stats vmc: %"PRIu16", orientation: 0x%"PRIx8", still: %"PRIi8"",
  //               *vmc, *orientation, (int8_t)*still);

  // Clear status
  memset(&state->summary_mean, 0, sizeof(state->summary_mean));
  memset(&state->summary_pim, 0, sizeof(state->summary_pim));
}


// ------------------------------------------------------------------------------------
uint32_t kalg_analyze_finish_epoch(KAlgState *state) {
  PBL_ASSERTN(state != NULL);
  uint32_t new_steps = 0;

  if (state->num_samples) {
    new_steps += prv_analyze_epoch(state);
    state->num_samples = 0;
  }
  return new_steps;
}




// ------------------------------------------------------------------------------------------
// Update the not-worn detection state machine. This state machine gets called on every minute
// update. It returns true if it determines the watch was not worn
static bool prv_not_worn_update(KAlgState *alg_state, time_t utc_now, uint16_t vmc,
                                uint8_t orientation, bool definitely_not_worn) {
  // Handy access to some variables
  const KAlgNotWornParams *params = &KALG_NOT_WORN_PARAMS;
  KAlgNotWornState *state = &alg_state->not_worn_state;

  // Determine if this is a "maybe-not-worn" sample
  bool maybe_not_worn = ((orientation == state->prev_orientation)
      || ((vmc < params->min_worn_vmc) && (state->prev_vmc < params->min_worn_vmc)));

  // The upper 4 bits of orientation encode the angle to the Z axis. If this value is 0x0 or 0x8
  // the watch is sitting flat on a table, so it's more probable that it's not being worn
  const uint8_t z_axis = orientation >> 4;
  const bool watch_is_flat = (z_axis == 0x0) || (z_axis == 0x8);
  if (watch_is_flat) {
    maybe_not_worn = true;
  }

  // If the VMC is very high, must be worn
  if (vmc > params->max_non_worn_vmc) {
    maybe_not_worn = false;
  }

  // Look for specific VMC values here that indicate definite worn or not-worn status
  bool definite_not_worn = definitely_not_worn;

  // Update stats
  if (maybe_not_worn || definite_not_worn) {
    // We just encountered a "maybe-not-worn" minute
    if (state->maybe_not_worn_count == 0) {
      // Start a new run
      state->potential_not_worn_start[0] = utc_now;
    }
    state->maybe_not_worn_count++;
    state->potential_not_worn_len_m[0]
        = ((utc_now - state->potential_not_worn_start[0]) / SECONDS_PER_MINUTE) + 1;

  } else {
    // We just encountered a "definitely worn" minute
    if (state->potential_not_worn_start[0] != KALG_START_TIME_NONE) {
      // Save not-worn history and reset state
      for (int i = KALG_NUM_NOT_WORN_SECTIONS - 1; i >= 1; i--) {
        state->potential_not_worn_start[i] = state->potential_not_worn_start[i - 1];
        state->potential_not_worn_len_m[i] = state->potential_not_worn_len_m[i - 1];
      }
      state->potential_not_worn_start[0] = KALG_START_TIME_NONE;
      state->potential_not_worn_len_m[0] = 0;
    }
    state->maybe_not_worn_count = 0;
  }
  state->prev_orientation = orientation;
  state->prev_vmc = vmc;

  // Compute result
  bool result =  definite_not_worn || (state->maybe_not_worn_count >= params->max_low_vmc_run_m);
  KALG_LOG_DEBUG("       NW:          vmc: %"PRIu16", orient: 0x%"PRIx8", not_worn: %d, "
                 "mnw_min:%d, mnw_count:%"PRIu16"", vmc, orientation, result, maybe_not_worn,
                 state->maybe_not_worn_count);
  return result;
}


// ------------------------------------------------------------------------------------------
// Decide if a potential sleep session should be rejected based on the not-worn state.
// Even if the current "not-worn" status is false, as returned by prv_not_worn_update(), we
// might have a potential not-worn section that is nearly as long as the sleep section. If that is
// the case, we reject the sleep session.
// @return true if not-worn during session
static bool prv_not_worn_during_session(KAlgState *alg_state, time_t session_start_utc,
                                        uint16_t session_len_m, bool ongoing) {
  KAlgNotWornState *state = &alg_state->not_worn_state;

  // If a candidate not-worn section starts near the start of a sleep session AND ends
  // near the end of the sleep session, we say the watch was not-worn
  const int32_t k_max_start_margin_m = session_len_m / 10;
  const int32_t k_min_end_margin_m = session_len_m / 8;

  // Or, if the candidate not-worn section is longer than this it is not-worn, regardless of where
  // it occurs within the sleep section
  const uint16_t k_min_not_worn_len_m = 150;

  // Compute the boundary locations
  time_t not_worn_start_boundary = session_start_utc + (k_max_start_margin_m * SECONDS_PER_MINUTE);
  time_t not_worn_end_boundary = session_start_utc
                               + ((session_len_m - k_min_end_margin_m) * SECONDS_PER_MINUTE);
  time_t session_end = session_start_utc + (session_len_m * SECONDS_PER_MINUTE);

  for (int i = 0; i < KALG_NUM_NOT_WORN_SECTIONS; i++) {
    if (state->potential_not_worn_len_m[i] == 0) {
      continue;
    }
    time_t not_worn_end = state->potential_not_worn_start[i]
                          + (state->potential_not_worn_len_m[i] * SECONDS_PER_MINUTE);

    // If this sleep session overlaps a very long section of potential not worn, it is not-worn
    time_t overlap_start = MAX(state->potential_not_worn_start[i], session_start_utc);
    time_t overlap_end = MIN(not_worn_end, session_end);
    if ((overlap_end - overlap_start) >= (k_min_not_worn_len_m * SECONDS_PER_MINUTE)) {
      return true;
    }

    // We only check the boundary constraints for sessions that have ended
    if (ongoing) {
      continue;
    }

    if ((state->potential_not_worn_start[i] <= not_worn_start_boundary)
        && (not_worn_end >= not_worn_end_boundary)) {
      KALG_LOG_DEBUG("detected not worn from %s for %"PRIu16" minutes",
                     prv_log_time(alg_state, state->potential_not_worn_start[i]),
                     state->potential_not_worn_len_m[i]);
      return true;
    }
  }
  return false;
}


// ------------------------------------------------------------------------------------------
// Register the deep sleep sesions we've found
static void prv_deep_sleep_register_sessions(KAlgState *alg_state, time_t sample_time,
                                             bool abort, bool ongoing,
                                             KAlgActivitySessionCallback sessions_cb,
                                             void *context) {
  // Handy access to some variables
  KAlgDeepSleepActivityState *state = &alg_state->deep_sleep_state;

  KALG_LOG_DEBUG("DS: time: %s, rcv %s", prv_log_time(alg_state, sample_time),
                 ongoing ? "register" : (abort ? "abort" : "end"));
  PBL_ASSERT(state->sleep_start_time != KALG_START_TIME_NONE, "Unexpected call");

  // Register/delete prevous sessions we captured
  for (uint8_t i = 0; i < state->num_sessions; i++) {
    time_t start_utc = state->sleep_start_time + state->start_delta_sec[i];
    sessions_cb(context, KAlgActivityType_RestfulSleep, start_utc,
                state->len_m[i] * SECONDS_PER_MINUTE, ongoing, abort /*delete*/,
                0 /*steps*/, 0 /*resting calories*/, 0 /*active_calories*/, 0 /*distance_mm*/);
  }

  // update/delete the session that might still be in progress
  if (state->deep_start_time != KALG_START_TIME_NONE) {
    int len_sec = sample_time - state->deep_start_time;
    sessions_cb(context, KAlgActivityType_RestfulSleep, state->deep_start_time,
                len_sec, ongoing, abort /*delete*/, 0 /*steps*/,
                0 /*resting calories*/, 0 /*active_calories*/, 0 /*distance_mm*/);
  }
}

// ------------------------------------------------------------------------------------------
// Update the deep sleep detection state machine. This state machine waits for the caller to
// say a new sleep session has started (KAlgDeepSleepAction_Start). Once started, it keeps
// track of which deep periods it detects after each update via KAlgDeepSleepAction_Continue.
// It remembers the deep sleep periods but doesn't register them until it receives
// KAlgDeepSleepAction_End. If it receives KAlgDeepSleepAction_Abort, it forgets all deep
// sleep periods it detected and waits for another KAlgDeepSleepAction_Start.
//
// @param[in] alg_state pointer to the algorithm state
// @param[in] sample_time the time stamp of the minute sample we are evaluating
// @param[in] score the sleep score for this minute
// @param[in] action which action to take:
//    KAlgDeepSleepAction_Start:    start of a new sleep session, start capturing
//    KAlgDeepSleepAction_Continue: Another sample for the current sleep session
//    KAlgDeepSleepAction_End:      current sleep sesion has ended
//    KAlgDeepSleepAction_Abort:    Abort the current sleep session
// @param[in] ok_to_register if true, it is OK to register this as a deep sleep session. We don't
//    allow registration until we're sure the container sleep session it is in is valid.
// @param[in] sessions_cb callback used to register deep sleep activities
// @param[in] context context for the above callback
static void prv_deep_sleep_update(KAlgState *alg_state, time_t sample_time, uint32_t score,
                                  KAlgDeepSleepAction action, bool ok_to_register,
                                  KAlgActivitySessionCallback sessions_cb, void *context) {
  // Handy access to some variables
  const KAlgDeepSleepParams *params = &KALG_DEEP_SLEEP_PARAMS;
  KAlgDeepSleepActivityState *state = &alg_state->deep_sleep_state;

  // Update state based on the passed in action
  switch (action) {
    case KAlgDeepSleepAction_Start:
      KALG_LOG_DEBUG("DS: time: %s, rcv start of new sleep", prv_log_time(alg_state, sample_time));
      // Start of a new sleep session
      PBL_ASSERT(state->sleep_start_time == KALG_START_TIME_NONE, "Unexpected start");
      *state = (KAlgDeepSleepActivityState) {
        .sleep_start_time = sample_time,
       };
      return;

    case KAlgDeepSleepAction_Continue:
      // If this is the first time we are allowed to register, then register the sessions we
      // already found as ongoing.
      // ok_to_register is true the first time we are allowed to register a deep sleep session -
      //   which is only after we're sure the sleep container it is in is valid. Before that,
      //   we will have recorded zero or more deep sleep candidates but wouldn't have registered
      //   them yet. If this is the first time we see ok_to_register, we need to go back and
      //   register the ones we previously found.
      if (ok_to_register && !state->ok_to_register) {
        prv_deep_sleep_register_sessions(alg_state, sample_time, false /*abort*/, true /*ongoing*/,
                                         sessions_cb, context);
        state->ok_to_register = true;
      }
      break;

    case KAlgDeepSleepAction_Abort:
      prv_deep_sleep_register_sessions(alg_state, sample_time, true /*abort*/, false /*ongoing*/,
                                       sessions_cb, context);
      // No longer in sleep
      *state = (KAlgDeepSleepActivityState) { };
      return;

    case KAlgDeepSleepAction_End:
      prv_deep_sleep_register_sessions(alg_state, sample_time, false /*abort*/, false /*ongoing*/,
                                       sessions_cb, context);
      // No longer in sleep
      *state = (KAlgDeepSleepActivityState) { };
      return;
  }

  // Handle continuation of sleep
  bool is_deep_minute = (score <= params->max_deep_score);
  KALG_LOG_DEBUG("       DS:          is_deep_min:%"PRIu8", consecutive_deep_min:%"PRIu16", "
                 "consecutive_non_deep_min: %"PRIu16"", (uint8_t)is_deep_minute,
                 state->deep_score_count, state->non_deep_score_count);

  // Update counts
  uint16_t last_deep_run_size = state->deep_score_count;
  if (!is_deep_minute) {
    state->non_deep_score_count++;
    state->deep_score_count = 0;
  } else {
    state->non_deep_score_count = 0;
    state->deep_score_count++;
  }

  // Update state
  if (state->deep_start_time == KALG_START_TIME_NONE) {
    // We have not detected start yet, look for a start
    if (state->deep_score_count >= params->min_deep_score_count) {
      state->deep_start_time = sample_time - (state->deep_score_count * SECONDS_PER_MINUTE);
      KALG_LOG_DEBUG("Detected deep sleep start at %s",
                     prv_log_time(alg_state, state->deep_start_time));
    }

  } else {
    // We have a deep session in progress. Compute its end time and length
    time_t start_time = state->deep_start_time;
    time_t end_time = sample_time;

    if ((state->non_deep_score_count > 0) && (last_deep_run_size < params->min_deep_score_count)) {
      // We reached the end of it last_deep_run_size minutes ago
      end_time = sample_time - (last_deep_run_size * SECONDS_PER_MINUTE);
      uint16_t len_m = MAX((end_time - start_time) / SECONDS_PER_MINUTE, 0);
      PBL_LOG_DBG("Detected deep sleep of %"
        PRIu16
        " minutes starting at %s ",
              len_m, prv_log_time(alg_state, start_time));

      // Store the session we just found as a complete one now that we have the end
      if (state->num_sessions < ARRAY_LENGTH(state->start_delta_sec)) {
        uint16_t delta_sec = MAX(start_time - state->sleep_start_time, 0);
        state->start_delta_sec[state->num_sessions] = delta_sec;
        state->len_m[state->num_sessions] = len_m;
        state->num_sessions++;
      } else {
        PBL_LOG_WRN("No more room for another deep sleep session");
      }
      // Wait for another session
      state->deep_start_time = KALG_START_TIME_NONE;
    }
    // Register/update it as ongoing
    if (state->ok_to_register) {
      sessions_cb(context, KAlgActivityType_RestfulSleep, start_time,
                  end_time - start_time, true /*ongoing*/, false /*delete*/, 0 /*steps*/,
                  0 /*resting calories*/, 0 /*active_calories*/, 0 /*distance_mm*/);
    }
  }
}


// ------------------------------------------------------------------------------------------
// Collect minute data and update the statistics we need for a sleep update. This gets
// called at the beginning of prv_sleep_activity_update().
// @return true if we have enough data to compute the score for this minute
static bool prv_sleep_activity_update_stats(KAlgState *alg_state, time_t utc_now, uint16_t vmc,
                                            uint8_t orientation, bool definitely_not_worn,
                                            uint32_t *score_ret, time_t *sample_utc_ret,
                                            bool *is_sleep_minute_ret) {
  // Handy access to some variables
  const KAlgSleepParams *params = &KALG_SLEEP_PARAMS;
  KAlgSleepActivityState *state = &alg_state->sleep_state;

  // Add this data to our history
  const unsigned int history_capacity = ARRAY_LENGTH(state->minute_history);
  if (state->num_history_entries >= history_capacity) {
    memmove(state->minute_history, state->minute_history + 1,
            (history_capacity - 1) * sizeof(KAlgSleepMinute));
    state->num_history_entries--;
  }
  state->minute_history[state->num_history_entries++] = (KAlgSleepMinute) {
    .vmc = vmc,
    .orientation = orientation,
    .definitely_not_worn = definitely_not_worn,
  };

  // Get the not-worn status
  bool not_worn = prv_not_worn_update(alg_state, utc_now, vmc, orientation, definitely_not_worn);

  // We have to have at least a filter's worth of data
  if (state->num_history_entries < history_capacity) {
    return false;
  }

  // Compute the sleep score for the target minute and see if it's a sleep minute
  // The minute we are computing the score for *starts* at KALG_SLEEP_HALF_WIDTH + 1
  time_t sample_utc = utc_now - ((KALG_SLEEP_HALF_WIDTH + 1) * SECONDS_PER_MINUTE);
  uint32_t score = prv_compute_sleep_score(state->minute_history, KALG_SLEEP_HALF_WIDTH);
  bool is_sleep_minute = ((score <= params->max_sleep_minute_score) && !not_worn);

  // ----------------------------------------------------------------------------------
  // Update stats
  if (is_sleep_minute) {
    state->current_stats.consecutive_sleep_minutes++;
    state->current_stats.consecutive_awake_minutes = 0;
  } else {
    state->current_stats.consecutive_sleep_minutes = 0;
    state->current_stats.consecutive_awake_minutes++;
  }
  if (score > params->min_valid_vmc) {
    // If there is any movememnt at all, increment the "non-zero" minutes count.
    state->current_stats.num_non_zero_minutes++;
  }
  if (state->current_stats.start_time != KALG_START_TIME_NONE) {
    state->current_stats.vmc_sum += MIN(params->vmc_clip, vmc);
  }

  state->last_sample_utc = sample_utc;

  // Return results
  *score_ret = score;
  *sample_utc_ret = sample_utc;
  *is_sleep_minute_ret = is_sleep_minute;
  return true;
}


// ------------------------------------------------------------------------------------------
// See if we should start a new sleep session or end the current one
static void prv_sleep_activity_update_session_state(
    KAlgState *alg_state, time_t sample_utc, uint16_t vmc, uint32_t score, bool is_sleep_minute,
    unsigned minutes_since_sleep_started, bool shutting_down,
    KAlgActivitySessionCallback sessions_cb, void *context,
    time_t *sleep_end_time, bool *reject_session) {
  // Handy access to some variables
  const KAlgSleepParams *params = &KALG_SLEEP_PARAMS;
  KAlgSleepActivityState *state = &alg_state->sleep_state;

  // Compute running averages
  unsigned pct_non_zero = 0;
  uint16_t avg_vmc = 0;
  if (state->current_stats.start_time != KALG_START_TIME_NONE) {
    pct_non_zero = (state->current_stats.num_non_zero_minutes * 100) / minutes_since_sleep_started;
    avg_vmc = state->current_stats.vmc_sum / minutes_since_sleep_started;
  }

  // This gets set to true if we decided that the current sleep session we are in is
  // not a valid one.
  *reject_session = false;

  // This gets set to non-zero if we detected the end of the current sleep session
  *sleep_end_time = KALG_START_TIME_NONE;


  // ----------------------------------------------------------------------------------
  // See if we should start a new session or end the current one
  if (state->current_stats.start_time == KALG_START_TIME_NONE) {
    // We haven't detected bedtime yet, see if we should start sleep
    if (state->current_stats.consecutive_sleep_minutes >= params->min_sleep_minutes) {
      state->current_stats.start_time = sample_utc
        - (state->current_stats.consecutive_sleep_minutes * SECONDS_PER_MINUTE);
      state->current_stats.num_non_zero_minutes = 0;
      state->current_stats.vmc_sum = 0;

      KALG_LOG_DEBUG("Detected bedtime at %s", prv_log_time(alg_state,
                                                            state->current_stats.start_time));

      // Inform the deep sleep detection logic that a new sleep session just started
      prv_deep_sleep_update(alg_state, state->current_stats.start_time, score,
                            KAlgDeepSleepAction_Start, false /*ok_to_register*/, sessions_cb,
                            context);
    }

  } else {
    // We have detected a bedtime, see if we should wake yet.
    uint32_t wake_minutes_threshold =
      (minutes_since_sleep_started < params->max_wake_minute_early_offset)
      ? params->max_wake_minutes_early
      : params->max_wake_minutes_late;

    if (prv_not_worn_during_session(alg_state, state->current_stats.start_time,
                                    minutes_since_sleep_started, true /*ongoing*/)) {
      // Reject because of not-worn
      KALG_LOG_DEBUG("Cycle rejected because of not-worn");
      *sleep_end_time = sample_utc;
      *reject_session = true;

    } else if (state->current_stats.consecutive_awake_minutes >= wake_minutes_threshold) {
      // Too many awake minutes in a row
      *sleep_end_time = sample_utc
                         - (state->current_stats.consecutive_awake_minutes * SECONDS_PER_MINUTE);

    } else if (vmc > params->force_wake_minute_vmc) {
      // VMC for this minute is way too high
      *sleep_end_time = sample_utc;
      KALG_LOG_DEBUG("Cycle ended because VMC was too high for this minute");

    } else if (score > params->force_wake_minute_score) {
      // Score for this minute is way too high
      *sleep_end_time = sample_utc;
      KALG_LOG_DEBUG("Cycle ended because score was too high for this minute");

    } else if ((minutes_since_sleep_started > params->min_sleep_len_for_active_pct_check)
      && (pct_non_zero > params->max_active_minutes_pct)) {
      // Too high a percent of awake minutes
      // If the percentage of non-zero minutes is too high, reject this cycle.
      *sleep_end_time = sample_utc;
      *reject_session = true;
      KALG_LOG_DEBUG("Cycle rejected because too many non-zero minutes (%d pct)",
                     pct_non_zero);

    } else if ((minutes_since_sleep_started > params->min_sleep_len_for_active_pct_check)
      && (avg_vmc > params->max_avg_vmc)) {
      // Too high an average VMC, reject this cycle
      // If the percentage of non-zero minutes is too high, reject this cycle.
      *sleep_end_time = sample_utc;
      *reject_session = true;
      KALG_LOG_DEBUG("Cycle rejected because avg vmc is too high (%"PRIu16")", avg_vmc);
    } else if (shutting_down) {
      KALG_LOG_DEBUG("Cycle ended because we are shutting down");
      *sleep_end_time = sample_utc;
    }
  }

  // Print state
  KALG_LOG_DEBUG("%s: score:%5"PRIu32", is_sleep_min:%"PRIi8", cons_sleep_min:%"PRIi16", "
    "cons_awake_min: %"PRIi16", pct_non_zero: %u, avg_vmc: %"PRIu16" ",
                 prv_log_time(alg_state, sample_utc), score, (int8_t)is_sleep_minute,
                 state->current_stats.consecutive_sleep_minutes,
                 state->current_stats.consecutive_awake_minutes, pct_non_zero, avg_vmc);
}


// ------------------------------------------------------------------------------------------
// Process the minute data for sleep detection
static void prv_sleep_activity_update(KAlgState *alg_state, time_t utc_now, uint16_t vmc,
                                      uint8_t orientation, bool definitely_not_worn,
                                      bool shutting_down,
                                      KAlgActivitySessionCallback sessions_cb, void *context) {
  // Handy access to some variables
  const KAlgSleepParams *params = &KALG_SLEEP_PARAMS;
  KAlgSleepActivityState *state = &alg_state->sleep_state;

  // Update stats that we keep in our state variables and compute the score for this minute
  uint32_t score = 0;
  time_t sample_utc = 0;
  bool is_sleep_minute = false;
  if (shutting_down) {
    // Grab the most recent sample_utc we have and run the algorithm again with the added
    // constraint that we are shutting down right now. The reason we save it off and use it is
    // because the sleep algorithm can only be run when we accumulated enough minutes. We
    // essentially run it with old data, but with the added constraint that we are shutting down.
    sample_utc = alg_state->sleep_state.last_sample_utc;
  } else if (!prv_sleep_activity_update_stats(alg_state, utc_now, vmc, orientation,
                                              definitely_not_worn, &score, &sample_utc,
                                              &is_sleep_minute)) {
    return;
  }

  // How many minutes since sleep started?
  unsigned minutes_since_sleep_started = 0;
  if (state->current_stats.start_time != KALG_START_TIME_NONE) {
    minutes_since_sleep_started = (sample_utc - state->current_stats.start_time)
                                  / SECONDS_PER_MINUTE;
  }

  // Determine if the current session (if any) should end or if we should start a new one
  // ... Set true if we decided that the current sleep session we are in is not a valid one.
  bool reject_session;
  // ... Set non-zero if we detected the end of the current sleep session
  time_t sleep_end_time;
  prv_sleep_activity_update_session_state(alg_state, sample_utc, vmc, score, is_sleep_minute,
                                          minutes_since_sleep_started, shutting_down, sessions_cb,
                                          context, &sleep_end_time, &reject_session);


  // -------------------------------------------------------------------------------
  // If we've reached the end of a sleep cycle, validate the constraints of the session now
  // to see if we should accept it.
  if (sleep_end_time != KALG_START_TIME_NONE) {
    uint16_t session_len_m = (sleep_end_time - state->current_stats.start_time)
                             / SECONDS_PER_MINUTE;
    // Detected waking up. Validate the other constraints of a sleep cycle
    KALG_LOG_DEBUG("Detected wake at %s, cycle_len: %u",  prv_log_time(alg_state, sleep_end_time),
                   session_len_m);

    // Reject if the session is too short
    if (minutes_since_sleep_started < params->min_sleep_cycle_len_minutes) {
      reject_session = true;
      KALG_LOG_DEBUG("Cycle rejected because too short");
    }

    // Reject if we detect the watch was not worn at all during this session
    if (prv_not_worn_during_session(alg_state, state->current_stats.start_time, session_len_m,
                                    false /*ongoing*/)) {
      reject_session = true;
      KALG_LOG_DEBUG("Cycle rejected because not worn");
    }

    // If we got a valid sleep cycle, add it to the totals
    if (!reject_session) {
      PBL_LOG_DBG("Detected valid sleep cycle of len %d, starting at %s",
              session_len_m, prv_log_time(alg_state, state->current_stats.start_time));

      sessions_cb(context, KAlgActivityType_Sleep, state->current_stats.start_time,
                  session_len_m * SECONDS_PER_MINUTE, false /*ongoing*/, false /*delete*/,
                  0 /*steps*/, 0 /*resting_calories*/, 0 /*active_calories*/, 0 /*distane_mm*/);

      // Inform the deep sleep detection logic that the sleep session just ended
      prv_deep_sleep_update(alg_state, sample_utc, score, KAlgDeepSleepAction_End,
                            true /*ok_to_register*/, sessions_cb, context);
      // Update summary stats
      state->summary_stats = (KAlgOngoingSleepStats) {
        .sleep_start_utc = state->current_stats.start_time,
        .uncertain_start_utc = 0,
        .sleep_len_m = session_len_m,
      };

    } else {
      KALG_LOG_DEBUG("Cycle rejected");
      // Delete the previously registered ongoing session
      sessions_cb(context, KAlgActivityType_Sleep, state->current_stats.start_time,
                  session_len_m * SECONDS_PER_MINUTE, true /*ongoing*/, true /*delete*/,
                  0 /*steps*/, 0 /*resting_calories*/, 0 /*active_calories*/, 0 /*distane_mm*/);

      // Inform the deep sleep detection logic that this sleep session was aborted
      prv_deep_sleep_update(alg_state, sample_utc, score, KAlgDeepSleepAction_Abort,
                            false /*ok_to_register*/, sessions_cb, context);

      // Clear summary stats if they included this rejected session
      if (state->summary_stats.sleep_start_utc == state->current_stats.start_time) {
        state->summary_stats = (KAlgOngoingSleepStats) {};
      }
    }
    // No current session anymore
    state->current_stats = (KAlgSleepActivityStats) { };

  } else {
    // Sleep has not ended yet
    if (state->current_stats.start_time != KALG_START_TIME_NONE) {
      if (minutes_since_sleep_started >= params->min_sleep_cycle_len_minutes) {
        // Register ongoing sleep if we are in sleep
        sessions_cb(context, KAlgActivityType_Sleep, state->current_stats.start_time,
                    minutes_since_sleep_started * SECONDS_PER_MINUTE, true /*ongoing*/,
                    false /*delete*/, 0 /*steps*/, 0 /*resting_calories*/, 0 /*active_calories*/,
                    0 /*distane_mm*/);

        // Update summary stats
        state->summary_stats.sleep_start_utc = state->current_stats.start_time;
        state->summary_stats.uncertain_start_utc = utc_now
                                              - (KALG_MAX_UNCERTAIN_SLEEP_M * SECONDS_PER_MINUTE);
        state->summary_stats.sleep_len_m = (state->summary_stats.uncertain_start_utc
                                     - state->summary_stats.sleep_start_utc) / SECONDS_PER_MINUTE;
      }

      // Inform deep sleep state machine of the new sample
      bool ok_to_register = (minutes_since_sleep_started >= params->min_sleep_cycle_len_minutes);
      prv_deep_sleep_update(alg_state, sample_utc, score, KAlgDeepSleepAction_Continue,
                            ok_to_register, sessions_cb, context);
    }
  }
}


// ------------------------------------------------------------------------------------------
// Return activity attributes for the given activity
static const KAlgActivityAttributes *prv_get_step_activity_attributes(KAlgActivityType activity) {
  static const KAlgActivityAttributes k_attributes[KAlgActivityTypeCount] = {
    // min_steps_per_min, max_steps_per_min
    {0, 0},            // KAlgActivityType_Sleep
    {0, 0},            // KAlgActivityType_ResetfulSleep
    {40,  130},        // KAlgActivityType_Walk
    {130, 255},        // KAlgActivityType_Run
  };

  PBL_ASSERTN(activity < KAlgActivityTypeCount);
  return &k_attributes[activity];
}

#ifdef CONFIG_HRM
// ------------------------------------------------------------------------------------------
static void prv_hrm_subscription_cb(PebbleHRMEvent *hrm_event, void *context) {
  // The algorithm doesn't care about these events. It only subscribed so the activity service
  // gets events.
}
#endif

// ------------------------------------------------------------------------------------------
// Process the minute data for walk or run activity detection
static void prv_step_activity_update(KAlgState *alg_state, KAlgStepActivityState *state,
                                     time_t utc_now, uint16_t steps, uint32_t resting_calories,
                                     uint32_t active_calories, uint32_t distance_mm,
                                     bool shutting_down, KAlgActivitySessionCallback sessions_cb,
                                     void *context, KAlgActivityType activity_type) {
  // Get the attributes associated with this activity
  const KAlgActivityAttributes *attr = prv_get_step_activity_attributes(activity_type);

  // If we see more than this number of inactive minutes in a row, the activity has ended
  const uint16_t k_max_inactive_minutes = 6;

  // An activity must be at least this number of minutes long
  const uint32_t k_min_activity_secs = 10 * SECONDS_PER_MINUTE;

  // Is this an active minute?
  bool is_active_minute = (steps >= attr->min_steps_per_min) && (steps <= attr->max_steps_per_min);

  // If the variable `shutting_down` is true, we ar forcefully ending all activities. Don't allow
  // one to continue.
  const bool activity_in_progress = is_active_minute && !shutting_down;
  if (activity_in_progress) {
    // This is an active minute. Start a new activity, or extend the current one
    state->inactive_minute_count = 0;
    if (state->start_time == KALG_START_TIME_NONE) {
      state->start_time = utc_now - SECONDS_PER_MINUTE;
      KALG_LOG_DEBUG("Detected activity %d: start: %s ", (int)activity_type,
                     prv_log_time(alg_state, state->start_time));
    }
    state->steps += steps;
    state->resting_calories += resting_calories;
    state->active_calories += active_calories;
    state->distance_mm += distance_mm;

    PBL_ASSERTN(state->start_time < utc_now);
    uint32_t duration_secs = utc_now - state->start_time;

#ifdef CONFIG_HRM
    // Bound the subscription and re-arm it each active minute, so it lapses on its own if this
    // state machine ever stops running. An activity survives k_max_inactive_minutes without
    // reaching here, so the window has to comfortably exceed that.
    const uint16_t hrm_expire_s = 10 * SECONDS_PER_MINUTE;

    // Make sure we have a couple active minutes in a row before enabling the HRM to save battery
    const unsigned min_duration_for_hrm = 3 * SECONDS_PER_MINUTE;
    if (state->hrm_session != HRM_INVALID_SESSION_REF) {
      sys_hrm_manager_set_update_interval(state->hrm_session, 1 /* update interval */,
                                          hrm_expire_s);
    } else if (duration_secs >= min_duration_for_hrm &&
               activity_prefs_hrm_activity_tracking_is_enabled()) {
      state->hrm_session = hrm_manager_subscribe_with_callback(INSTALL_ID_INVALID,
          1 /* update interval */, hrm_expire_s, HRMFeature_BPM, prv_hrm_subscription_cb, NULL);
    }
#endif

    // If we've reached the minimum activity length, register/update it
    if (duration_secs >= k_min_activity_secs) {
      KALG_LOG_DEBUG("Updating activity %d: steps: %"PRIu16", rest_cal: %"PRIu32", "
        "active_cal: %"PRIu32", distance: %"PRIu32" ", (int)activity_type,
                     state->steps, state->resting_calories, state->active_calories,
                     state->distance_mm);

      sessions_cb(context, activity_type, state->start_time, duration_secs, true /*ongoing*/,
                  false /*delete*/, state->steps, state->resting_calories, state->active_calories,
                  state->distance_mm);
    }

  } else {
    // This is an inactive minute. See if we've reached the end of the activity
    if (state->start_time == KALG_START_TIME_NONE) {
      // No potential activity in progress, nothing left to do
      return;
    }

    // We can either end activity by reaching enough inactive minutes in a row or by forcefully
    // ending all activities by the `shutting_down` variable.
    const bool activity_ended = (shutting_down)
                                ? true
                                : (state->inactive_minute_count++ > k_max_inactive_minutes);

    if (activity_ended) {
      // This activity has ended
      int32_t duration_secs = utc_now - state->start_time
                              - (state->inactive_minute_count * SECONDS_PER_MINUTE);
      duration_secs = MAX(0, duration_secs);
      if ((uint32_t)duration_secs >= k_min_activity_secs) {
        KALG_LOG_DEBUG("Ending activity %d: steps: %"PRIu16", rest_cal: %"PRIu32", ""active_cal: "
                       "%"PRIu32", distance: %"PRIu32" ", (int) activity_type,
                       state->steps, state->resting_calories, state->active_calories,
                       state->distance_mm);
        sessions_cb(context, activity_type, state->start_time, duration_secs, false /*ongoing*/,
                    false /*delete*/, state->steps, state->resting_calories, state->active_calories,
                    state->distance_mm);
      }
      prv_reset_step_activity_state(state);
    } else {
      // This was an inactive minute, but the activity is still considered ongoing, so accumulate
      // whatever steps, calories we have in this minute
      state->steps += steps;
      state->resting_calories += resting_calories;
      state->active_calories += active_calories;
      state->distance_mm += distance_mm;
    }
  }
}


// ---------------------------------------------------------------------------------------
// Feed new minute data into the activity detection state machine. This logic looks for non-sleep
// activities, like walks, runs, etc.
void kalg_activities_update(KAlgState *state, time_t utc_now, uint16_t steps, uint16_t vmc,
                            uint8_t orientation, bool definitely_not_worn,
                            uint32_t resting_calories,
                            uint32_t active_calories, uint32_t distance_mm, bool shutting_down,
                            KAlgActivitySessionCallback sessions_cb, void *context) {
  // If we've encountered a significant change in UTC time (connecting to a new phone, factory
  // reset, etc.) it could wreak havoc with our activity state machines, so we need to reset
  // state
  if ((utc_now < state->last_activity_update_utc)
      || (utc_now > (state->last_activity_update_utc + (5 * SECONDS_PER_MINUTE)))) {
    PBL_LOG_WRN("Resetting state due to time travel");
    prv_reset_state(state);
  };
  state->last_activity_update_utc = utc_now;

  if (!state->disable_activity_session_tracking) {
    // Pass onto the walk activity detector
    prv_step_activity_update(state, &state->walk_state, utc_now, steps, resting_calories,
                             active_calories, distance_mm, shutting_down, sessions_cb, context,
                             KAlgActivityType_Walk);

    // Pass onto the run activity detector
    prv_step_activity_update(state, &state->run_state, utc_now, steps, resting_calories,
                             active_calories, distance_mm, shutting_down, sessions_cb, context,
                             KAlgActivityType_Run);

    // Pass onto the sleep detector
    prv_sleep_activity_update(state, utc_now, vmc, orientation, definitely_not_worn,
                              shutting_down, sessions_cb, context);
  }
}


// ---------------------------------------------------------------------------------------
time_t kalg_activity_last_processed_time(KAlgState *state, KAlgActivityType activity) {
  switch (activity) {
    case KAlgActivityType_Sleep:
    case KAlgActivityType_RestfulSleep:
      return state->last_activity_update_utc - (KALG_SLEEP_HALF_WIDTH * SECONDS_PER_MINUTE);
      break;
    case KAlgActivityType_Run:
    case KAlgActivityType_Walk:
      return state->last_activity_update_utc;
    case KAlgActivityTypeCount:
      break;
  }
  PBL_ASSERTN(false);
  return 0;
}


// ---------------------------------------------------------------------------------------
// Get sleep summary stats
void kalg_get_sleep_stats(KAlgState *alg_state, KAlgOngoingSleepStats *stats) {
  KAlgSleepActivityState *state = &alg_state->sleep_state;
  *stats = state->summary_stats;
}

// ---------------------------------------------------------------------------------------
void kalg_enable_activity_tracking(KAlgState *kalg_state, bool enable) {
  kalg_state->disable_activity_session_tracking = !enable;
  prv_reset_state(kalg_state);
}
