/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/app.h"
#include "applib/app_logging.h"
#include "applib/fonts/fonts.h"
#include "applib/health_service.h"
#include "applib/ui/simple_menu_layer.h"
#include "applib/ui/ui.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/activity/activity.h"
#include "pbl/services/activity/activity_algorithm.h"
#include "pbl/services/activity/activity_insights.h"
#include "pbl/services/activity/activity_private.h"
#include "pbl/services/activity/kraepelin/activity_algorithm_kraepelin.h"
#include "pbl/services/activity/insights_settings.h"
#include <pbl/logging/logging.h>
#include "pbl/util/math.h"
#include "pbl/util/size.h"

#include "activity_test.h"

#include <stdio.h>
#include <services/activity/activity.h>

// Test the activity API
typedef struct {
  Window *window;
  SimpleMenuLayer *menu_layer;
  SimpleMenuItem *menu_items;
  int test_index;                       // Which test is currently running
  uint32_t steps_updated_value;         // What we last received from the steps update handler
} ActivityTestAppData;

#define SAMPLES_PER_SECOND ACCEL_SAMPLING_25HZ

// Forward prototypes
static void prv_test_end(void *context, bool passed);


// ==========================================================================================
// Activities
// These samples were collected using the accel-logging-app:
//      git@github.com:pebble/accel-logging-app.git.
//
// That app saves the raw accel data to a data logging session. The pebble tool was used to
// extract the data out of the logging session and then the "parse_log.py" tool in the
// accel-logging-app repo was used to generate this static struct from the raw data stored
// by data logging.
//
// pebble data-logging list                           # list all sessions
// pebble data-logging download --session_id=<XXX>    # Download the session that has tag 4262
// python tools/parse_log.py <bin_file_downloaded>
//
// This is about 500 samples, 20 seconds worth of data
static AccelRawData s_walk_30_steps[] = {
    { -42, -52, -1027},
    { -43, -29, -1054},
    { -76, 12, -975},
    { -72, -17, -906},
    { -64, -40, -892},
    { -81, -37, -933},
    { -77, -15, -1008},
    { -83, 0, -1041},
    { -81, -27, -1029},
    { -80, -57, -993},
    { -97, -27, -973},
    { -119, -21, -991},
    { -120, -19, -1011},
    { -115, -27, -978},
    { -106, -44, -953},
    { -104, -62, -946},
    { -150, -90, -962},
    { -148, -66, -991},
    { -164, -101, -980},
    { -162, -102, -975},
    { -223, -67, -966},
    { -251, -84, -971},
    { -118, -134, -968},
    { -159, -54, -956},
    { -160, -125, -958},
    { -102, -152, -963},
    { -39, -154, -974},
    { 63, -317, -986},
    { -61, -190, -1000},
    { -131, -306, -1022},
    { -189, -295, -844},
    { -224, -317, -694},
    { -243, -177, -563},
    { -420, 58, -499},
    { -677, 416, -469},
    { -796, 908, -426},
    { -844, 1100, -288},
    { -762, 824, -233},
    { -813, 1019, -270},
    { -816, 1041, -119},
    { -865, 914, -6},
    { -848, 872, 24},
    { -817, 776, 42},
    { -765, 655, 14},
    { -804, 635, -20},
    { -839, 657, 13},
    { -874, 659, -1},
    { -926, 587, -19},
    { -976, 510, -31},
    { -937, 466, -78},
    { -1187, 483, 24},
    { -1046, 482, -87},
    { -1071, 566, -238},
    { -884, 460, -301},
    { -844, 144, -211},
    { -880, 213, -78},
    { -972, -2, 38},
    { -786, 53, 32},
    { -739, 232, -12},
    { -695, 344, -61},
    { -706, 394, -72},
    { -743, 351, -67},
    { -775, 334, -58},
    { -819, 333, -45},
    { -869, 365, -70},
    { -833, 405, -36},
    { -1466, 634, 18},
    { -1132, 698, -39},
    { -849, 548, -59},
    { -1073, 483, -47},
    { -970, 540, -84},
    { -883, 458, -35},
    { -781, 364, -41},
    { -732, 345, -40},
    { -751, 324, 2},
    { -752, 287, 42},
    { -727, 285, 29},
    { -718, 304, 44},
    { -813, 325, 79},
    { -903, 335, 75},
    { -880, 323, 4},
    { -1093, 404, 57},
    { -1177, 388, 76},
    { -1098, 513, -82},
    { -892, 549, -256},
    { -739, 484, -199},
    { -695, 337, -82},
    { -846, 302, 0},
    { -787, 298, 24},
    { -712, 392, 29},
    { -733, 439, -20},
    { -740, 464, -49},
    { -739, 450, -50},
    { -752, 411, -87},
    { -837, 438, -93},
    { -961, 453, -73},
    { -994, 442, -23},
    { -1059, 466, 2},
    { -1222, 594, 33},
    { -997, 552, 30},
    { -873, 477, -30},
    { -850, 443, 32},
    { -894, 421, 62},
    { -891, 367, 102},
    { -873, 322, 164},
    { -865, 316, 181},
    { -804, 364, 150},
    { -757, 337, 158},
    { -714, 338, 110},
    { -721, 405, 71},
    { -721, 419, 53},
    { -725, 400, 35},
    { -737, 373, 55},
    { -752, 386, 57},
    { -865, 451, 64},
    { -928, 486, 99},
    { -1113, 519, 257},
    { -1147, 531, 305},
    { -1069, 693, 49},
    { -815, 808, -126},
    { -679, 568, -43},
    { -748, 421, 29},
    { -886, 316, 59},
    { -906, 306, 26},
    { -846, 369, 0},
    { -843, 404, -15},
    { -828, 396, -2},
    { -826, 358, 23},
    { -755, 300, 41},
    { -700, 305, 46},
    { -747, 377, 28},
    { -794, 416, 47},
    { -1102, 470, 161},
    { -1303, 549, 281},
    { -1194, 656, 238},
    { -709, 508, 77},
    { -734, 453, 115},
    { -739, 456, 199},
    { -781, 422, 258},
    { -758, 330, 258},
    { -708, 344, 240},
    { -780, 317, 285},
    { -817, 324, 263},
    { -829, 355, 210},
    { -858, 364, 164},
    { -906, 377, 157},
    { -923, 359, 98},
    { -887, 352, 9},
    { -1061, 369, 100},
    { -1180, 384, 148},
    { -922, 410, 5},
    { -778, 464, -90},
    { -770, 444, -26},
    { -841, 375, 6},
    { -837, 321, 52},
    { -757, 354, 58},
    { -745, 372, 36},
    { -737, 432, -16},
    { -759, 420, -26},
    { -783, 429, -53},
    { -824, 437, -80},
    { -883, 473, -85},
    { -907, 472, -54},
    { -856, 403, 17},
    { -1254, 537, 65},
    { -1046, 516, 93},
    { -1008, 479, 88},
    { -771, 420, 33},
    { -884, 437, 88},
    { -907, 417, 109},
    { -837, 382, 101},
    { -802, 361, 88},
    { -789, 363, 102},
    { -788, 318, 123},
    { -761, 264, 123},
    { -762, 273, 114},
    { -790, 335, 109},
    { -870, 325, 138},
    { -913, 337, 109},
    { -871, 357, 18},
    { -1159, 395, 84},
    { -1036, 382, 52},
    { -907, 491, -125},
    { -798, 533, -195},
    { -784, 460, -111},
    { -835, 375, -31},
    { -840, 327, -9},
    { -773, 403, -1},
    { -739, 434, -30},
    { -773, 450, -57},
    { -786, 449, -64},
    { -862, 447, -89},
    { -934, 463, -112},
    { -975, 473, -92},
    { -897, 430, -62},
    { -851, 399, -43},
    { -1183, 530, 30},
    { -984, 538, 30},
    { -940, 525, 39},
    { -779, 470, 31},
    { -915, 462, 97},
    { -916, 397, 174},
    { -881, 386, 167},
    { -840, 391, 157},
    { -766, 343, 177},
    { -755, 324, 191},
    { -769, 317, 192},
    { -807, 352, 193},
    { -821, 380, 179},
    { -916, 396, 179},
    { -928, 360, 144},
    { -871, 359, 34},
    { -1104, 362, 83},
    { -1048, 368, -61},
    { -819, 467, -296},
    { -747, 536, -384},
    { -840, 488, -258},
    { -858, 404, -178},
    { -831, 316, -141},
    { -734, 466, -161},
    { -715, 431, -110},
    { -760, 436, -88},
    { -762, 422, -44},
    { -726, 430, -46},
    { -778, 447, -31},
    { -872, 477, -16},
    { -987, 484, 33},
    { -962, 420, 126},
    { -1254, 472, 174},
    { -1012, 451, 128},
    { -1023, 473, 152},
    { -863, 461, 115},
    { -836, 440, 169},
    { -887, 418, 182},
    { -946, 397, 235},
    { -878, 358, 206},
    { -779, 327, 142},
    { -748, 269, 156},
    { -745, 232, 145},
    { -737, 260, 111},
    { -772, 324, 96},
    { -874, 366, 115},
    { -954, 347, 148},
    { -919, 360, 83},
    { -994, 403, 83},
    { -1132, 383, 116},
    { -907, 417, -30},
    { -861, 496, -124},
    { -853, 482, -98},
    { -822, 430, -71},
    { -876, 385, -36},
    { -836, 391, -41},
    { -742, 396, -49},
    { -721, 389, -60},
    { -735, 392, -67},
    { -753, 370, -68},
    { -803, 402, -92},
    { -871, 446, -113},
    { -954, 472, -109},
    { -941, 440, -62},
    { -1021, 428, -15},
    { -1181, 546, -10},
    { -979, 498, 48},
    { -970, 454, 18},
    { -808, 412, 6},
    { -925, 465, -8},
    { -942, 404, 39},
    { -851, 352, 49},
    { -788, 336, 43},
    { -760, 304, 71},
    { -755, 270, 106},
    { -744, 257, 119},
    { -724, 280, 111},
    { -745, 307, 103},
    { -874, 345, 116},
    { -948, 331, 95},
    { -958, 361, 58},
    { -1237, 397, 140},
    { -1129, 438, 48},
    { -948, 546, -138},
    { -796, 531, -202},
    { -748, 418, -147},
    { -886, 384, -85},
    { -855, 300, -62},
    { -760, 381, -50},
    { -739, 410, -87},
    { -748, 417, -89},
    { -741, 405, -79},
    { -772, 384, -87},
    { -857, 422, -101},
    { -937, 444, -100},
    { -926, 409, -69},
    { -1087, 440, -28},
    { -1202, 533, -44},
    { -928, 472, -23},
    { -904, 425, -81},
    { -781, 414, -98},
    { -934, 455, -72},
    { -974, 342, -20},
    { -927, 341, 31},
    { -833, 357, 6},
    { -759, 306, 20},
    { -772, 292, 64},
    { -784, 281, 90},
    { -770, 289, 75},
    { -769, 301, 64},
    { -791, 281, 84},
    { -865, 311, 77},
    { -905, 356, 34},
    { -928, 376, 34},
    { -1242, 424, 119},
    { -1062, 453, 58},
    { -963, 584, -126},
    { -810, 527, -67},
    { -812, 403, -9},
    { -880, 330, 21},
    { -792, 329, 40},
    { -692, 363, 0},
    { -689, 397, -46},
    { -698, 405, -71},
    { -710, 379, -79},
    { -795, 419, -102},
    { -892, 468, -119},
    { -1012, 491, -96},
    { -1024, 453, -41},
    { -1262, 560, -19},
    { -1120, 525, 57},
    { -1004, 434, 31},
    { -819, 398, 63},
    { -761, 405, 134},
    { -779, 464, 82},
    { -827, 455, 99},
    { -837, 389, 148},
    { -752, 410, 137},
    { -744, 386, 145},
    { -753, 364, 124},
    { -760, 335, 143},
    { -818, 328, 129},
    { -916, 308, 157},
    { -985, 295, 165},
    { -905, 312, 83},
    { -1069, 368, 131},
    { -1014, 358, 120},
    { -917, 443, -7},
    { -762, 490, -147},
    { -820, 544, -160},
    { -753, 418, -100},
    { -873, 373, -25},
    { -828, 364, -24},
    { -806, 406, -60},
    { -757, 431, -105},
    { -779, 418, -84},
    { -790, 383, -105},
    { -818, 393, -119},
    { -869, 437, -133},
    { -958, 461, -104},
    { -874, 410, -26},
    { -1208, 503, 46},
    { -1075, 527, 44},
    { -1014, 502, 99},
    { -824, 458, 24},
    { -782, 446, 43},
    { -865, 422, 82},
    { -919, 364, 130},
    { -861, 358, 110},
    { -805, 357, 114},
    { -762, 325, 130},
    { -742, 301, 138},
    { -749, 298, 147},
    { -748, 305, 139},
    { -820, 341, 119},
    { -918, 331, 138},
    { -959, 353, 88},
    { -1027, 388, 79},
    { -1112, 380, 70},
    { -941, 440, -98},
    { -921, 522, -223},
    { -848, 539, -239},
    { -787, 394, -149},
    { -890, 329, -70},
    { -888, 341, -52},
    { -802, 381, -61},
    { -742, 399, -88},
    { -739, 403, -76},
    { -741, 364, -54},
    { -778, 357, -64},
    { -824, 402, -103},
    { -912, 451, -105},
    { -893, 426, -70},
    { -854, 394, -25},
    { -1267, 545, 34},
    { -982, 546, 27},
    { -927, 455, 9},
    { -748, 444, -42},
    { -903, 476, 7},
    { -953, 465, 38},
    { -921, 406, 49},
    { -860, 382, 61},
    { -795, 328, 60},
    { -754, 280, 78},
    { -753, 287, 83},
    { -765, 291, 78},
    { -797, 304, 77},
    { -847, 303, 95},
    { -918, 302, 89},
    { -887, 346, 49},
    { -1206, 375, 146},
    { -995, 384, 52},
    { -932, 490, -80},
    { -823, 524, -140},
    { -810, 473, -113},
    { -851, 386, -78},
    { -886, 362, -59},
    { -796, 392, -45},
    { -744, 396, -58},
    { -750, 414, -62},
    { -779, 421, -56},
    { -811, 410, -65},
    { -856, 427, -90},
    { -905, 457, -110},
    { -955, 463, -115},
    { -914, 431, -86},
    { -1064, 447, -36},
    { -1068, 509, -51},
    { -999, 499, 27},
    { -961, 451, 18},
    { -752, 419, -8},
    { -857, 444, 25},
    { -909, 409, 75},
    { -885, 376, 99},
    { -835, 344, 123},
    { -760, 307, 120},
    { -767, 296, 143},
    { -763, 289, 170},
    { -773, 310, 156},
    { -796, 343, 134},
    { -853, 329, 167},
    { -913, 337, 137},
    { -966, 368, 124},
    { -1257, 386, 184},
    { -891, 391, -52},
    { -823, 535, -266},
    { -770, 518, -245},
    { -864, 431, -138},
    { -955, 288, -95},
    { -936, 358, -128},
    { -833, 497, -102},
    { -782, 576, -113},
    { -774, 534, -104},
    { -798, 395, -78},
    { -801, 330, -25},
    { -849, 397, -49},
    { -879, 440, -35},
    { -875, 421, 0},
    { -841, 395, 46},
    { -983, 424, 95},
    { -1097, 478, 126},
    { -983, 499, 146},
    { -854, 450, 37},
    { -674, 450, 43},
    { -906, 448, 80},
    { -965, 379, 158},
    { -882, 328, 162},
    { -835, 354, 136},
    { -857, 347, 171},
    { -828, 307, 190},
    { -841, 307, 192},
    { -855, 339, 173},
    { -863, 323, 195},
    { -876, 319, 192},
    { -888, 320, 150},
    { -841, 343, 63},
    { -918, 377, 11},
    { -1145, 392, 66},
    { -866, 384, -129},
    { -768, 485, -290},
    { -796, 530, -255},
    { -900, 510, -141},
    { -970, 376, -113},
    { -891, 346, -100},
    { -797, 395, -38},
    { -774, 526, -58},
    { -837, 453, -35},
    { -840, 405, 9},
    { -844, 380, 31},
    { -854, 443, 22},
    { -892, 420, 43},
    { -889, 378, 65},
    { -872, 362, 99},
    { -841, 375, 101},
    { -1106, 546, 141},
    { -936, 614, 132},
    { -945, 643, 140},
    { -825, 713, 168},
    { -854, 725, 255},
    { -891, 784, 293},
    { -835, 912, 310},
    { -788, 924, 269},
    { -691, 953, 207},
    { -603, 999, 171},
};


// Clear out the app event queue
// This is necessary so that our app queue doesn't overflow while we are deep inside a test
// routine feeding in accel data and advancing the clock. We can get our queue full of health
// service updated steps/sleep events.
static void prv_clear_event_queue(void) {
  event_queue_cleanup_and_reset(app_manager_get_task_context()->to_process_event_queue);
}


// -------------------------------------------------------------------------------
// Run the minute callback enough times to cause everything that is normally periodically
// recomputed (walking rate, sleep, etc.) to be recomputed
static void prv_force_periodic_updates(void) {
  for (int i = 0; i < ACTIVITY_SESSION_UPDATE_MIN; i++) {
    activity_test_run_minute_callback();
  }
}


// -------------------------------------------------------------------------------
// Feed in N seconds of idle movement
static void prv_feed_idle_movement_sec(uint32_t seconds) {
  AccelRawData idle[SAMPLES_PER_SECOND];

  for (int i = 0; i < SAMPLES_PER_SECOND; i++) {
    idle[i] = (AccelRawData) {
      .x = i % 100,
      .y = i % 100,
      .z = i % 100,
    };
  }

  while (seconds--) {
    activity_test_feed_samples(idle, SAMPLES_PER_SECOND);
  }
}


// -------------------------------------------------------------------------------
// Feed in N minutes of walking movement
static void prv_feed_steps_min(uint32_t minutes) {
  uint32_t samples_remaining = minutes * 60 * SAMPLES_PER_SECOND;

  AccelRawData *step_samples = s_walk_30_steps;
  uint32_t num_step_samples = ARRAY_LENGTH(s_walk_30_steps);
  uint32_t step_sample_idx = 0;

  uint32_t minute_idx = samples_remaining / (60 * SAMPLES_PER_SECOND);
  while (samples_remaining) {
    uint32_t chunk_size = MIN(samples_remaining, SAMPLES_PER_SECOND);

    AccelRawData data[SAMPLES_PER_SECOND];
    for (uint32_t i = 0; i < chunk_size; i++) {
      data[i] = step_samples[step_sample_idx++];
      step_sample_idx = step_sample_idx % num_step_samples;
    }
    activity_test_feed_samples(data, chunk_size);
    rtc_set_time(rtc_get_time() + 1);

    samples_remaining -= chunk_size;
    if (minute_idx != samples_remaining / (60 * SAMPLES_PER_SECOND)) {
      activity_test_run_minute_callback();
      minute_idx = samples_remaining / (60 * SAMPLES_PER_SECOND);
      prv_clear_event_queue();
    }
  }
}

// -------------------------------------------------------------------------------
// Feed in N minutes of light sleep
static void prv_feed_light_sleep_min(uint32_t minutes) {
  // Light sleep produces minute statistics with step:0, variance:17-22, with
  // one minute of high variance (511) every 3-10 minutes.

  for (uint32_t minute = 0; minute < minutes; minute++) {
    if ((minute % 10) == 0) {
      activity_test_feed_samples(s_walk_30_steps, 10 * SAMPLES_PER_SECOND);
      prv_feed_idle_movement_sec(50);
    } else {
      prv_feed_idle_movement_sec(60);
    }
    activity_test_run_minute_callback();
    rtc_set_time(rtc_get_time() + SECONDS_PER_MINUTE);
    prv_clear_event_queue();
  }
}

// -------------------------------------------------------------------------------
// Feed in N minutes of deep sleep
static void prv_feed_deep_sleep_min(uint32_t minutes) {
  // Deep sleep produces minute statistics with step:0, variance:17-22, with
  // one minute of high variance (511) every 25-35 minutes.

  for (uint32_t minute = 0; minute < minutes; minute++) {
    if ((minute % 30) == 0) {
      activity_test_feed_samples(s_walk_30_steps, 2 * SAMPLES_PER_SECOND);
      prv_feed_idle_movement_sec(58);
    } else {
      prv_feed_idle_movement_sec(60);
    }
    activity_test_run_minute_callback();
    rtc_set_time(rtc_get_time() + SECONDS_PER_MINUTE);
    prv_clear_event_queue();
  }
}


// -------------------------------------------------------------------------------
static void prv_test_steps(void *context) {
  bool passed = false;

  // Reset all stored data
  activity_test_reset(true /*reset_settings*/, true /*tracking_on*/, NULL, NULL);

  // Fill the steps pipeline then capture step count before
  activity_test_feed_samples(s_walk_30_steps, ARRAY_LENGTH(s_walk_30_steps));
  int32_t before;
  activity_get_metric(ActivityMetricStepCount, 1, &before);

  // Walk 30 steps
  activity_test_feed_samples(s_walk_30_steps, ARRAY_LENGTH(s_walk_30_steps));

  // Check new step count
  int32_t steps;
  steps = health_service_sum_today(HealthMetricStepCount);
  steps -= before;

  PBL_LOG_DBG("steps: %"PRId32, steps);
  if (steps >= 27 && steps <= 33) {
    passed = true;
  }

  prv_test_end(context, passed);
}


// -------------------------------------------------------------------------------
static void prv_test_30_min_walk(void *context) {
  bool passed = false;

  activity_prefs_activity_insights_set_enabled(true);

  // Reset all stored data
  activity_test_reset(true /*reset_settings*/, true /*tracking_on*/, NULL, NULL);

  int32_t before;
  activity_get_metric(ActivityMetricStepCount, 1, &before);

  // Walk for about 30 minutes
  const int k_num_minutes = 30;

  // The sample we feed in feeds about 90 steps/min
  const int k_steps_per_minute = 90;

  // Feed in the data
  prv_feed_steps_min(k_num_minutes);

  // Check new step count
  int32_t steps;
  steps = health_service_sum_today(HealthMetricStepCount);
  steps -= before;
  PBL_LOG_DBG("steps: %"PRId32, steps);
  if (steps >= ((8 * k_steps_per_minute * k_num_minutes) / 10)) {
    passed = true;
  }

  // Trigger the activity session notification
  prv_feed_deep_sleep_min(30);

  prv_test_end(context, passed);
}


// -------------------------------------------------------------------------------
static void prv_test_sleep(void *context) {
  bool passed = true;
  int32_t value;

  // Reset all stored data
  activity_test_reset(true /*reset_settings*/, true /*tracking_on*/, NULL, NULL);

  // Change into awake state and capture the sleep before
  // Walk long enough to overlap with a periodic sleep recomputation
  prv_feed_steps_min(ACTIVITY_SESSION_UPDATE_MIN + 1);
  int32_t before_total, before_deep;
  activity_get_metric(ActivityMetricSleepTotalSeconds, 1, &before_total);
  activity_get_metric(ActivityMetricSleepRestfulSeconds, 1, &before_deep);
  PBL_LOG_DBG("start total: %d, start deep: %d", (int)before_total, (int)before_deep);

  // Capture steps before sleep
  int32_t steps_before;
  activity_get_metric(ActivityMetricStepCount, 1, &steps_before);

  // Do some light and deep sleep
  prv_feed_deep_sleep_min(60);
  prv_feed_light_sleep_min(180);
  prv_feed_deep_sleep_min(20);

  // Capture steps before sleep
  activity_get_metric(ActivityMetricSleepState, 1, &value);
  PBL_LOG_DBG("sleep state: %d", (int)value);

  // See how many steps we took during sleep. The light sleep simulator ends up providing about
  // 12 steps every 10 minutes, so without the "no steps during sleep" logic in the activity
  // service, we would end up with about 180/10 * 12 = 216 steps during sleep. With the
  // "no steps during sleep" logic in place, we should get close to 0 steps
  int32_t steps_after;
  activity_get_metric(ActivityMetricStepCount, 1, &steps_after);
  PBL_LOG_DBG("steps taken during sleep:: %d", (int)(steps_after - steps_before));
  if (steps_after - steps_before > 16) {
    PBL_LOG_ERR("too many steps during sleep: test FAILED");
    passed = false;
  }

  // Walk long enough to overlap with a periodic sleep recomputation
  prv_feed_steps_min(2 * ACTIVITY_SESSION_UPDATE_MIN);

  // Check sleep totals
  int32_t total_sleep, deep_sleep;
  activity_get_metric(ActivityMetricSleepTotalSeconds, 1, &total_sleep);
  total_sleep -= before_total;
  activity_get_metric(ActivityMetricSleepRestfulSeconds, 1, &deep_sleep);
  deep_sleep -= before_deep;

  PBL_LOG_DBG("total: %d, deep: %d", (int)total_sleep / SECONDS_PER_MINUTE,
          (int)deep_sleep / SECONDS_PER_MINUTE);
  const int k_min_total_sleep_min = 240;
  const int k_max_total_sleep_min = 280;
  const int k_min_deep_sleep_min = 40;
  const int k_max_deep_sleep_min = 80;
  if ((total_sleep < k_min_total_sleep_min * SECONDS_PER_MINUTE)
      || (total_sleep > k_max_total_sleep_min * SECONDS_PER_MINUTE)) {
    passed = false;
  }
  if ((deep_sleep < k_min_deep_sleep_min * SECONDS_PER_MINUTE)
      || (deep_sleep > k_max_deep_sleep_min * SECONDS_PER_MINUTE)) {
    passed = false;
  }

  // Check other sleep metrics
  activity_get_metric(ActivityMetricSleepEnterAtSeconds, 1, &value);
  PBL_LOG_DBG("entry minute: %d", (int)(value / SECONDS_PER_MINUTE));

  activity_get_metric(ActivityMetricSleepExitAtSeconds, 1, &value);
  PBL_LOG_DBG("exit minute: %d", (int)(value / SECONDS_PER_MINUTE));

  activity_get_metric(ActivityMetricSleepState, 1, &value);
  PBL_LOG_DBG("sleep state: %d", (int)value);

  activity_get_metric(ActivityMetricSleepStateSeconds, 1, &value);
  PBL_LOG_DBG("sleep state minutes: %d", (int)(value / SECONDS_PER_MINUTE));

  prv_test_end(context, passed);
}


// -------------------------------------------------------------------------------
// Test that we don't crash or get a weird sleep session if the UTC time changes while
// sleeping
static void prv_test_sleep_time_change(void *context) {
  bool passed = true;

  // Reset all stored data
  activity_test_reset(true /*reset_settings*/, true /*tracking_on*/, NULL, NULL);

  // Walk a little
  prv_feed_steps_min(15);

  // Get us into sleep mode
  time_t start_sleep_time = rtc_get_time();
  PBL_LOG_DBG("Start sleep: %d", (int)start_sleep_time);
  prv_feed_light_sleep_min(80);
  prv_feed_steps_min(2);
  prv_feed_light_sleep_min(1);

  // Shift UTC time back by 75 days
  rtc_set_time(rtc_get_time() - 75 * SECONDS_PER_DAY - 6 * SECONDS_PER_HOUR);

  // Sleep a little more
  prv_feed_light_sleep_min(10);

  // Restore time to just after we started sleeping before
  rtc_set_time(start_sleep_time + 5 * SECONDS_PER_MINUTE);

  // Sleep a little more, should not crash
  prv_feed_steps_min(90);
  prv_feed_steps_min(20);   // wake up

  // Make sure we registered sleep
  int32_t value;
  activity_get_metric(ActivityMetricSleepTotalSeconds, 1, &value);
  PBL_LOG_DBG("sleep total: %"PRIi32" ", value);
  if (value < (60 * SECONDS_PER_MINUTE) || value > (100 * SECONDS_PER_MINUTE)) {
    passed = false;
  }

  prv_test_end(context, passed);
}


// ---------------------------------------------------------------------------------
static void prv_count_sleep_sessions(time_t after_time, int *num_sleep, int *num_nap) {
  *num_sleep = 0;
  *num_nap = 0;

  ActivitySession sessions[ACTIVITY_MAX_ACTIVITY_SESSIONS_COUNT];
  uint32_t session_entries = ARRAY_LENGTH(sessions);
  bool success = activity_get_sessions(&session_entries, sessions);
  if (!success) {
    return;
  }

  PBL_LOG_DBG("Looking for sessions...");
  for (uint32_t i = 0; i < session_entries; i++) {
    ActivitySession *session = &sessions[i];
    PBL_LOG_DBG("  Found session type: %d, start_min: %d, length_min: %"PRIu16" ",
            (int)session->type, time_util_get_minute_of_day(session->start_utc),
            session->length_min);
    if (session->start_utc < after_time) {
      PBL_LOG_DBG("  Ignoring because too old");
      continue;
    }
    if ((session->type == ActivitySessionType_Sleep)
        || (session->type == ActivitySessionType_RestfulSleep)) {
      (*num_sleep)++;
    }
    if ((session->type == ActivitySessionType_Nap)
        || (session->type == ActivitySessionType_RestfulNap)) {
      (*num_nap)++;
    }
  }
  PBL_LOG_DBG("Done looking for sessions");
}


// -------------------------------------------------------------------------------
static void prv_test_nap(void *context) {
  bool passed = true;

  activity_prefs_sleep_insights_set_enabled(true);

  const time_t now_utc = rtc_get_time();
  PBL_LOG_DBG("test start time: %d", (int)now_utc);

  const time_t midnight_utc = time_util_get_midnight_of(now_utc);
  const time_t nap_time_start = midnight_utc + (ALG_PRIMARY_MORNING_MINUTE * SECONDS_PER_MINUTE);
  const time_t nap_time_end = midnight_utc + (ALG_PRIMARY_EVENING_MINUTE * SECONDS_PER_MINUTE);

  // Go to one hour after the time sleeps are considered naps if we aren't currently in it
  if (!WITHIN(now_utc, nap_time_start, nap_time_end)) {
    const time_t next_nap_time = (nap_time_start < now_utc ? nap_time_start + SECONDS_PER_DAY :
                                                             nap_time_start) + SECONDS_PER_HOUR;
    rtc_set_time(next_nap_time);
  }

  time_t test_start_utc = rtc_get_time();
  PBL_LOG_DBG("test start time changed to: %d", (int)test_start_utc);

  // Reset all stored data
  activity_test_reset(false /* reset_settings */, true /*tracking_on*/, NULL, NULL);

  // Walk a little first
  prv_feed_steps_min(15);

  // Sleep for 100 minutes: 20 light, 30 deep, 10 light, 30 deep, 10 light
  prv_feed_light_sleep_min(20);
  prv_feed_deep_sleep_min(30);
  prv_feed_light_sleep_min(10);
  prv_feed_deep_sleep_min(30);
  prv_feed_light_sleep_min(10);

  // We should have no nap sessions since the sleep hasn't ended yet
  int nap_count = 0;
  int sleep_count = 0;
  prv_count_sleep_sessions(test_start_utc, &sleep_count, &nap_count);
  PBL_LOG_DBG("Found %d sleep sessions and %d nap sessions", sleep_count, nap_count);
  if (nap_count > 0 || sleep_count == 0) {
    PBL_LOG_ERR("FAILED: expected only sleep but got %d naps and %d sleep", nap_count,
            sleep_count);
    prv_test_end(context, false);
    return;
  }

  // Walk long enough for a sleep computation to run
  prv_feed_steps_min(3 * ACTIVITY_SESSION_UPDATE_MIN);

  // We should have only nap sessions now
  prv_count_sleep_sessions(test_start_utc, &sleep_count, &nap_count);
  PBL_LOG_DBG("Found %d sleep sessions and %d nap sessions", sleep_count, nap_count);
  if (nap_count == 0 || sleep_count > 0) {
    PBL_LOG_ERR("FAILED: expected only naps but got %d nap and %d sleep", nap_count,
            sleep_count);
    prv_test_end(context, false);
    return;
  }

  prv_test_end(context, passed);
}

// -------------------------------------------------------------------------------
static void prv_test_sleep_reward(void *context) {
  bool passed = true;

  const ActivityScalarStore AVERAGE_SLEEP = 1 * MINUTES_PER_HOUR;
  const ActivityScalarStore GOOD_SLEEP = 2 * MINUTES_PER_HOUR;

  // Hack to get around midnight rollover bug (only affects tests)
  rtc_set_time(time_util_get_midnight_of(rtc_get_time()));

  bool prev_insights_enabled = activity_prefs_sleep_insights_are_enabled();
  activity_prefs_sleep_insights_set_enabled(true);

  // History with low median but good sleep over the past few days
  ActivitySettingsValueHistory sleep_history = {
    .utc_sec = rtc_get_time(),
    .values = {
      0, // This ends up overwritten anyway by the current sleep value
      GOOD_SLEEP,
      GOOD_SLEEP,
      GOOD_SLEEP,
      AVERAGE_SLEEP,
      AVERAGE_SLEEP,
      AVERAGE_SLEEP,
      AVERAGE_SLEEP,
      AVERAGE_SLEEP,
      AVERAGE_SLEEP,
      AVERAGE_SLEEP,
      AVERAGE_SLEEP,
      AVERAGE_SLEEP,
      AVERAGE_SLEEP
    }
  };

  // Reset all stored data
  activity_test_reset(true /*reset_settings*/, true /*tracking_on*/, &sleep_history, NULL);

  for (int i = 0; i < 3; ++i) {
    // Change into awake state and capture the sleep before
    // Walk long enough to overlap with a periodic sleep recomputation
    prv_feed_steps_min(ACTIVITY_SESSION_UPDATE_MIN + 1);

    // Do some deep sleep
    prv_feed_deep_sleep_min(GOOD_SLEEP);

    // Walk long enough to be registered as "awake" for over 2 hours
    prv_feed_steps_min((2.5 * MINUTES_PER_HOUR) + ACTIVITY_SESSION_UPDATE_MIN);

    // Fast forward time
    rtc_set_time(time_util_get_midnight_of(rtc_get_time()) + 4 * SECONDS_PER_DAY);
  }

  activity_prefs_sleep_insights_set_enabled(prev_insights_enabled);
  prv_test_end(context, passed);
}

// -------------------------------------------------------------------------------
static void prv_test_activity_reward(void *context) {
  bool passed = true;

  // TODO: is this needed here?
  // Hack to get around midnight rollover bug (only affects tests)
  rtc_set_time(time_util_get_midnight_of(rtc_get_time()));

  bool prev_insights_enabled = activity_prefs_activity_insights_are_enabled();
  activity_prefs_activity_insights_set_enabled(true);

  const ActivityScalarStore AVERAGE_STEPS = 1000;

  // History with low median
  ActivitySettingsValueHistory step_history = {
    .utc_sec = rtc_get_time(),
    .values = {
      0, // This ends up overwritten anyway by the current sleep value
      AVERAGE_STEPS,
      AVERAGE_STEPS,
      AVERAGE_STEPS,
      AVERAGE_STEPS,
      AVERAGE_STEPS,
      AVERAGE_STEPS,
      AVERAGE_STEPS,
      AVERAGE_STEPS,
      AVERAGE_STEPS,
      AVERAGE_STEPS
    }
  };

  // Reset all stored data
  activity_test_reset(true /*reset_settings*/, true /*tracking_on*/, NULL, &step_history);

  // Walk for about 30 minutes (this should give us over 2500 steps)
  const int k_num_minutes = 30;

  // Feed in the data (should see an insight)
  prv_feed_steps_min(k_num_minutes);

  // Fast forward a day and check that we update the settings cache when the file changes
  rtc_set_time(time_util_get_midnight_of(rtc_get_time()) + 1 * SECONDS_PER_DAY);

  ActivityInsightSettings original_settings;
  activity_insights_settings_read(ACTIVITY_INSIGHTS_SETTINGS_ACTIVITY_REWARD, &original_settings);

  ActivityInsightSettings disabled_settings = original_settings;
  disabled_settings.enabled = false;
  activity_insights_settings_write(ACTIVITY_INSIGHTS_SETTINGS_ACTIVITY_REWARD, &disabled_settings);

  prv_feed_steps_min(k_num_minutes);

  activity_insights_settings_write(ACTIVITY_INSIGHTS_SETTINGS_ACTIVITY_REWARD, &original_settings);
  activity_prefs_activity_insights_set_enabled(prev_insights_enabled);
  prv_test_end(context, passed);
}

// -------------------------------------------------------------------------------
static void prv_test_sleep_summary(void *context) {
  bool passed = true;

  // Start at 1am to make sure it doesn't get registered as a nap
  rtc_set_time(time_util_get_midnight_of(rtc_get_time()) + 1 * SECONDS_PER_HOUR);

  activity_prefs_sleep_insights_set_enabled(true);

  const ActivityScalarStore AVERAGE_SLEEP = 2 * MINUTES_PER_HOUR;

  // History with low median but good sleep over the past few days
  ActivitySettingsValueHistory sleep_history = {
    .utc_sec = rtc_get_time(),
    .values = {
      0, // This ends up overwritten anyway by the current sleep value
      AVERAGE_SLEEP,
      AVERAGE_SLEEP,
      AVERAGE_SLEEP,
      AVERAGE_SLEEP
    }
  };

  // Reset all stored data
  activity_test_reset(false /*reset_settings*/, true /*tracking_on*/, &sleep_history, NULL);

  // Change into awake state and capture the sleep before
  // Walk long enough to overlap with a periodic sleep recomputation
  prv_feed_steps_min(ACTIVITY_SESSION_UPDATE_MIN + 1);

  // Do some deep sleep
  prv_feed_deep_sleep_min(2 * MINUTES_PER_HOUR);

  // Walk long enough to be registered as "awake"
  prv_feed_steps_min(ACTIVITY_SESSION_UPDATE_MIN + 1);

  // Trigger the insight notif
  prv_feed_steps_min(2 * MINUTES_PER_HOUR);

  prv_test_end(context, passed);
}

// -------------------------------------------------------------------------------
static void prv_test_activity_summary(void *context) {
  bool passed = true;

  activity_prefs_activity_insights_set_enabled(true);

  // Set to the trigger time
  struct tm time_tm = {};
  const time_t now = rtc_get_time();
  localtime_r(&now, &time_tm);
  time_tm.tm_hour = 20;
  time_tm.tm_min = 25;
  rtc_set_time(mktime(&time_tm));

  // Set the step history
  activity_test_set_steps_history();
  prv_feed_steps_min(4);

  // Trigger insights
  activity_insights_recalculate_stats();
  prv_feed_steps_min(1);

  prv_test_end(context, passed);
}

// -------------------------------------------------------------------------------
static void prv_test_fill_sleep(void *context) {
  // Fill the sleep file
  bool success = activity_test_fill_minute_file();
  prv_test_end(context, success);
}


// -------------------------------------------------------------------------------
static void select_click_handler(ClickRecognizerRef recognizer, void *context) {
}

// -------------------------------------------------------------------------------
static void up_click_handler(ClickRecognizerRef recognizer, void *context) {
}

// -------------------------------------------------------------------------------
static void down_click_handler(ClickRecognizerRef recognizer, void *context) {
}


// -------------------------------------------------------------------------------
static void click_config_provider(void *context) {
  window_single_click_subscribe(BUTTON_ID_SELECT, select_click_handler);
  window_single_click_subscribe(BUTTON_ID_UP, up_click_handler);
  window_single_click_subscribe(BUTTON_ID_DOWN, down_click_handler);
}


// -------------------------------------------------------------------------------
static void prv_health_event_handler(HealthEventType event,
                                     void *context) {
  ActivityTestAppData *data = (ActivityTestAppData *)context;

  // Test the sum function
  if (event == HealthEventMovementUpdate) {
    int32_t steps_today = health_service_sum_today(HealthMetricStepCount);

     APP_LOG(APP_LOG_LEVEL_DEBUG, "Got steps update event.(today value: %d)",
             (int) steps_today);

    data->steps_updated_value = steps_today;
  }
}


// -------------------------------------------------------------------------------
// Tests
typedef struct {
  const char *title;
  AppTimerCallback callback;
} TestEntry;

static TestEntry s_test_entries[] = {
  {
    .title = "steps",
    .callback = prv_test_steps,
  }, {
    .title = "30m walk",
    .callback = prv_test_30_min_walk,
  }, {
    .title = "sleep",
    .callback = prv_test_sleep,
  }, {
    .title = "nap",
    .callback = prv_test_nap,
  }, {
    .title = "fill sleep",
    .callback = prv_test_fill_sleep
  }, {
    .title = "sleep reward",
    .callback = prv_test_sleep_reward
  }, {
    .title = "activity reward",
    .callback = prv_test_activity_reward
  }, {
    .title = "sleep summary",
    .callback = prv_test_sleep_summary
  }, {
    .title = "activity summary",
    .callback = prv_test_activity_summary
  }, {
    .title = "sleep, w/time chg",
    .callback = prv_test_sleep_time_change
  }
};


// -------------------------------------------------------------------------------
static void prv_test_begin(int index, void *context) {
  ActivityTestAppData *data = (ActivityTestAppData *)context;

  PBL_LOG_DBG("Running test: '%s'...", data->menu_items[index].title);
  data->menu_items[index].subtitle = "Running...";
  layer_mark_dirty(simple_menu_layer_get_layer(data->menu_layer));

  // Run the test from a timer callback so that the window can be updated
  data->test_index = index;
  app_timer_register(0, s_test_entries[index].callback, context);
}


// -------------------------------------------------------------------------------
static void prv_test_end(void *context, bool passed) {
  ActivityTestAppData *data = (ActivityTestAppData *)context;

  const char *result_str =  passed ? "PASS" : "FAIL";
  PBL_LOG_DBG("Test result: %s", result_str);

  data->menu_items[data->test_index].subtitle = result_str;
  layer_mark_dirty(simple_menu_layer_get_layer(data->menu_layer));
}


// -------------------------------------------------------------------------------
static void prv_window_load(Window *window) {
  ActivityTestAppData *data = window_get_user_data(window);
  Layer *window_layer = window_get_root_layer(window);
  GRect bounds = window_layer->bounds;

  uint32_t num_tests = ARRAY_LENGTH(s_test_entries);

  // Create the menu items
  SimpleMenuItem *menu_items = app_malloc_check(num_tests * sizeof(SimpleMenuItem));
  for (uint32_t i = 0; i < num_tests; i++) {
    menu_items[i] = (SimpleMenuItem) {
      .title = s_test_entries[i].title,
      .callback = prv_test_begin
    };
  }
  static SimpleMenuSection sections;
  sections.items = menu_items;
  sections.num_items = num_tests;

  data->menu_items = menu_items;
  data->menu_layer = simple_menu_layer_create(bounds, window, &sections, 1, data /*context*/);
  layer_add_child(window_layer, simple_menu_layer_get_layer(data->menu_layer));

  // Start activity service in test mode
  activity_stop_tracking();
  activity_start_tracking(true /*test_mode*/);

  // Subscribe to health update events
  health_service_events_subscribe(prv_health_event_handler, data);
}


// -------------------------------------------------------------------------------
static void prv_window_unload(Window *window) {
  ActivityTestAppData *data = window_get_user_data(window);

  simple_menu_layer_destroy(data->menu_layer);
  app_free(data->menu_items);

  // Restore normal mode
  activity_stop_tracking();
  activity_start_tracking(false /*test_mode*/);
}


// -------------------------------------------------------------------------------
static void deinit(void) {
  ActivityTestAppData *data = app_state_get_user_data();
  window_destroy(data->window);
  app_free(data);
}


// -------------------------------------------------------------------------------
static void init(void) {
  ActivityTestAppData *data = app_malloc_check(sizeof(ActivityTestAppData));
  memset(data, 0, sizeof(ActivityTestAppData));
  app_state_set_user_data(data);

  // Init window
  data->window = window_create();
  window_set_user_data(data->window, data);
  window_set_click_config_provider_with_context(data->window, click_config_provider, data);
  window_set_window_handlers(data->window, &(WindowHandlers) {
    .load = prv_window_load,
    .unload = prv_window_unload,
  });

  app_window_stack_push(data->window, true /* Animated */);
}


// -------------------------------------------------------------------------------
static void s_main(void) {
  init();
  app_event_loop();
  deinit();
}


// -------------------------------------------------------------------------------
const PebbleProcessMd* activity_test_get_app_info(void) {
  static const PebbleProcessMdSystem s_activity_test_app_info = {
    .common.main_func = &s_main,
    .name = "ActivityTest"
  };
  return (const PebbleProcessMd*) &s_activity_test_app_info;
}
