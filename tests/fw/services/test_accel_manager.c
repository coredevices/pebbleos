/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "fake_app_manager.h"
#include "fake_new_timer.h"
#include "fake_pbl_malloc.h"
#include "fake_pebble_tasks.h"
#include "fake_system_task.h"

#include "stubs_analytics.h"
#include "stubs_gettext.h"
#include "stubs_logging.h"
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_persist.h"
#include "stubs_prompt.h"
#include "stubs_msgq.h"
#include "stubs_resources.h"
#include "stubs_serial.h"
#include "stubs_syscall_internal.h"
#include "stubs_worker_manager.h"

#include <pbl/drivers/accel.h>
#include "pbl/services/event_service.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"

#include <stdio.h>

// helpers from accel manager
extern void test_accel_manager_get_subsample_info(
    AccelManagerState *state, uint16_t *num, uint16_t *den, uint16_t *samps_per_update);
extern void test_accel_manager_reset(void);

// stub
void event_service_init(PebbleEventType type,
                        EventServiceAddSubscriberCallback start_cb,
                        EventServiceRemoveSubscriberCallback stop_cb) {}
void sys_vibe_history_start_collecting(void) {}
void sys_vibe_history_stop_collecting(void) {}
int32_t sys_vibe_get_vibe_strength(void) {
  return 0;
}
int32_t vibes_get_vibe_strength(void) {
  return 0;
}
uint32_t vibes_get_time_since_last_vibe_ms(void) {
  return UINT32_MAX;
}
void accel_set_shake_sensitivity_high(bool sensitivity_high) {}
void accel_set_shake_sensitivity_percent(uint8_t percent) {}
bool shell_prefs_get_accel_shake_log_info_enabled(void) {
  return false;
}
struct pbl_msgq *pebble_task_get_to_queue(PebbleTask task) {
  return NULL;
}

// fake accel.h impl
static int s_sampling_interval_us = 1000000 / ACCEL_SAMPLING_25HZ;
static int s_num_samples = 0;

//! If true, ignore attempts to change the sampling interval
static bool s_force_sampling_interval;

uint32_t accel_set_sampling_interval(uint32_t interval_us) {
  if (!s_force_sampling_interval) {
    s_sampling_interval_us = interval_us;
  }
  return accel_get_sampling_interval();
}

uint32_t accel_get_sampling_interval(void) {
  return s_sampling_interval_us;
}

void accel_set_num_samples(uint32_t num_samples) {
  s_num_samples = num_samples;
}

uint32_t accel_get_max_num_samples(void) {
  return 32;
}
int accel_peek(AccelDriverSample *data) {
  return 0;
}
void accel_enable_shake_detection(bool on) {
}
bool accel_get_shake_detection_enabled(void) {
  return false;
}
void accel_enable_double_tap_detection(bool on) {
}
bool accel_get_double_tap_detection_enabled(void) {
  return false;
}

bool new_timer_add_work_callback_from_isr(NewTimerWorkCallback cb, void *data) {
  return false;
}
bool new_timer_add_work_callback(NewTimerWorkCallback cb, void *data) {
  return true;
}

// Unit Test Code

void test_accel_manager__initialize(void) {
  accel_manager_init();

  s_sampling_interval_us = 1000000 / ACCEL_SAMPLING_25HZ;
  s_num_samples = 0;
  s_force_sampling_interval = false;
}


void test_accel_manager__cleanup(void) {
  test_accel_manager_reset();
}

static void prv_noop_sample_handler(void *context) {

}

static void prv_validate_sample_rates(int *arr, int num_samples) {
  for (int i = 0; i < num_samples; i++) {
    // force a compiler error if user has not added all possible sample rates to array
    switch ((AccelSamplingRate)arr[i]) {
      case ACCEL_SAMPLING_10HZ:
      case ACCEL_SAMPLING_25HZ:
      case ACCEL_SAMPLING_50HZ:
      case ACCEL_SAMPLING_100HZ:
        break;
      default:
        cl_assert(0);
    }
  }
}

static void prv_run_accel_test(int *sample_arr, int num_items) {
  PebbleTask tasks[] = { PebbleTask_KernelMain, PebbleTask_Worker, PebbleTask_App };
  AccelManagerState* sessions[3];

  if (num_items > 3) {
    return; // we only support 3 simultaneous subscribers
  }

  int fastest_rate = 0;
  AccelRawData fake_buf[1];
  for (int i = 0; i < num_items; i++) {
    if (fastest_rate < sample_arr[i]) {
      fastest_rate = sample_arr[i];
    }

    sessions[i] = sys_accel_manager_data_subscribe(
        sample_arr[i], prv_noop_sample_handler, NULL, tasks[i]);

    // buffer size of 1
    sys_accel_manager_set_sample_buffer(sessions[i], fake_buf, 1);
  }

  //  make sure all sampling rates are what they should be
  for (int i = 0; i < num_items; i++) {
    stub_pebble_tasks_set_current(tasks[i]);

    uint16_t num, den, samps_per_update;
    test_accel_manager_get_subsample_info(sessions[i], &num, &den, &samps_per_update);

    if ((fastest_rate % sample_arr[i]) == 0) {
      // the current sample rate is a multiple of the rate we are running at
      cl_assert_equal_i(num, 1);
      cl_assert_equal_i(den, fastest_rate / sample_arr[i]);
      cl_assert_equal_i(samps_per_update, 1);
    } else {
      // the sample rate is not an even multiple of our fastest rate
      uint32_t gcd_of_rates = gcd(fastest_rate, sample_arr[i]);
      cl_assert_equal_i(num, sample_arr[i] / gcd_of_rates);
      cl_assert_equal_i(den, fastest_rate / gcd_of_rates);
      cl_assert_equal_i(samps_per_update, 1);
    }
  }

  cl_assert_equal_i(1000000 / s_sampling_interval_us, fastest_rate);
  cl_assert_equal_i(s_num_samples, 1);

  for (int i = 0; i < num_items; i++) {
    sys_accel_manager_data_unsubscribe(sessions[i]);
    stub_pebble_tasks_set_current(tasks[i]);
  }
}

// enumerate through all possible sampling rate combinations and confirm
// that the correct frequency is selected
void test_accel_manager__subscription_sampling_rates(void) {
  int sample_rates[] = { ACCEL_SAMPLING_10HZ, ACCEL_SAMPLING_25HZ,
                         ACCEL_SAMPLING_50HZ, ACCEL_SAMPLING_100HZ};
  prv_validate_sample_rates(sample_rates, ARRAY_LENGTH(sample_rates));

  int poss_rates = ARRAY_LENGTH(sample_rates);
  int max_permutations = 0x1 << poss_rates;

  for (int mask = 0; mask < max_permutations; mask++) {
    int count = __builtin_popcount(mask);
    if (count == 0) {
      continue; // we don't care about the empty set
    }

    int test_rates[count];
    int idx = 0;
    for (int j = 0; j < poss_rates; j++) {
      if ((mask & (0x1 << j)) != 0) {
        test_rates[idx] = sample_rates[j];
        idx++;
      }
    }

    printf("Testing: ");
    for (int i = 0; i < count; i++) {
      printf("%d ", sample_rates[i]);
    }
    printf("\n");

    prv_run_accel_test(test_rates, count);
  }
}

void test_accel_manager__jitterfree(void) {
  // Force the fake accel to only support the 125hz sample rate.
  s_force_sampling_interval = true;
  s_sampling_interval_us = (1000000000 / 125000);

  AccelRawData fake_buf[1];

  AccelManagerState *state = sys_accel_manager_data_subscribe(
      ACCEL_SAMPLING_25HZ, prv_noop_sample_handler, NULL, PebbleTask_KernelMain);
  uint32_t resulting_mhz = accel_manager_set_jitterfree_sampling_rate(state, 12500);
  sys_accel_manager_set_sample_buffer(state , fake_buf, ARRAY_LENGTH(fake_buf));

  cl_assert_equal_i(resulting_mhz, 12500);

  uint16_t num, den, samples_per_update;
  test_accel_manager_get_subsample_info(state, &num, &den, &samples_per_update);

  cl_assert_equal_i(num, 1);
  cl_assert_equal_i(den, 10);
  cl_assert_equal_i(samples_per_update, ARRAY_LENGTH(fake_buf));
}


void test_accel_manager__batched_samples(void) {
  AccelRawData fake_buf[30];

  stub_pebble_tasks_set_current(PebbleTask_KernelMain);
  AccelManagerState *main_session = sys_accel_manager_data_subscribe(
      ACCEL_SAMPLING_10HZ, prv_noop_sample_handler, NULL, PebbleTask_KernelMain);
  sys_accel_manager_set_sample_buffer(main_session, fake_buf, 11);

  stub_pebble_tasks_set_current(PebbleTask_Worker);
  AccelManagerState *worker_session = sys_accel_manager_data_subscribe(
      ACCEL_SAMPLING_25HZ, prv_noop_sample_handler, NULL, PebbleTask_KernelMain);
  sys_accel_manager_set_sample_buffer(worker_session, fake_buf, 22);

  cl_assert_equal_i(s_num_samples, 22);

  stub_pebble_tasks_set_current(PebbleTask_KernelMain);
  sys_accel_manager_set_sample_buffer(main_session, fake_buf, 3);
  cl_assert_equal_i(s_num_samples, 7); /* 300ms / (1000ms / 25 samps) */
}
