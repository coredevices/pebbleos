/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "comm/ble/gap_le_advert.h"
#include "comm/ble/gap_le_connection.h"
#include "pbl/services/regular_timer.h"
#include "pbl/util/size.h"

#include "clar.h"

// Fakes
///////////////////////////////////////////////////////////

#include "fake_bt_driver_advert.h"
#include "fake_new_timer.h"
#include "fake_rtc.h"
#include "fake_system_task.h"
#include "fake_pbl_malloc.h"

// Stubs
///////////////////////////////////////////////////////////

#include "stubs_analytics.h"
#include "stubs_bluetopia_interface.h"
#include "stubs_bt_lock.h"
#include "stubs_gatt_client_discovery.h"
#include "stubs_gatt_client_subscriptions.h"
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_prompt.h"

bool static s_is_connected_as_slave = false;

bool gap_le_connect_is_connected_as_slave(void) {
  return s_is_connected_as_slave;
}

void ble_legacy_discovery_enable(uint32_t timeout_secs) {
}

void gap_le_slave_reconnect_stop(void) {
}

void gap_le_slave_reconnect_start(void) {
}

void gatt_service_changed_server_cleanup_by_connection(GAPLEConnection *connection) {
}

void launcher_task_add_callback(void (*callback)(void *data), void *data) {
  callback(data);
}

static uint32_t s_unscheduled_cb_count;
static void * s_unscheduled_cb_data = "Callback Data";
static GAPLEAdvertisingJobRef s_unscheduled_job;
static bool s_unscheduled_completed;

static void unscheduled_callback(GAPLEAdvertisingJobRef job,
                                 bool completed,
                                 void *cb_data) {
  s_unscheduled_job = job;
  ++s_unscheduled_cb_count;
  s_unscheduled_completed = completed;
  cl_assert_equal_p(cb_data, s_unscheduled_cb_data);
}

void test_gap_le_advert__initialize(void) {
  fake_bt_driver_advert_init();

  s_unscheduled_cb_count = 0;
  s_unscheduled_job = NULL;
  s_unscheduled_completed = false;

  // This bypasses the work-around for the CC2564 advertising bug, that pauses the round-robinning
  // through scheduled advertisment jobs:
  s_is_connected_as_slave = true;

  regular_timer_init();
  gap_le_advert_init();
}

void test_gap_le_advert__cleanup(void) {
  gap_le_advert_deinit();

  // Make sure deinit did disable advertising and clean up timer:
  cl_assert(!gap_le_is_advertising_enabled());
  cl_assert_equal_i(regular_timer_seconds_count(), 0);

  regular_timer_deinit();
}

//! Helper to assert whether a piece of ad_data is set to the controller.
//! For convenience, C strings are used (while in reality it can be arbitrary
//! binary data).
#define assert_ad_data(ad_data_cstring) \
{ \
  Advertising_Data_t ad_data_out; \
  const size_t data_length = strlen(ad_data_cstring) + 1; \
  cl_assert_equal_i(gap_le_get_advertising_data(&ad_data_out), data_length); \
  cl_assert_equal_s((const char *) &ad_data_out, ad_data_cstring); \
}

//! Helper to create BLEAdData from C strings.
//! In reality, people will use ble_ad_create() and the ble_ad_set_* functions,
//! but that's part of another test.
static BLEAdData *create_ad(const char ad_data[],
                            const char scan_resp_data[]) {
  const size_t ad_data_length = ad_data ? strlen(ad_data) + 1 : 0;
  const size_t scan_resp_data_length =
      scan_resp_data ? strlen(scan_resp_data) + 1 : 0;

  BLEAdData *ad = (BLEAdData *) malloc(sizeof(BLEAdData) +
                                       ad_data_length +
                                       scan_resp_data_length);
  ad->ad_data_length = ad_data_length;
  ad->scan_resp_data_length = scan_resp_data_length;
  if (ad_data_length) {
    strncpy((char *) ad->data,
            ad_data, ad_data_length);
  }
  if (scan_resp_data_length) {
    strncpy((char *) ad->data + ad_data_length,
           scan_resp_data, scan_resp_data_length);
  }
  return ad;
}

void test_gap_le_advert__single_job(void) {
  const char ad_data[] = "ad data";
  const char scan_resp_data[] = "scan resp data";
  BLEAdData *ad = create_ad(ad_data, scan_resp_data);

  GAPLEAdvertisingJobTerm advert_term = {
    .interval = GAPLEAdvertisingInterval_Short,
    .duration_secs = 10,
  };
  GAPLEAdvertisingJobRef job;
  job = gap_le_advert_schedule(ad,
                               &advert_term,
                               sizeof(advert_term)/sizeof(GAPLEAdvertisingJobTerm),
                               unscheduled_callback, s_unscheduled_cb_data, 0);
  cl_assert(job);

  // Since there was nothing scheduled, expect that the ad data is set to the
  // controller immediately:
  cl_assert(gap_le_is_advertising_enabled());

  // Check that the ad data that is passed to the controller is the same that
  // was given when calling the API:
  assert_ad_data(ad_data);

  // Check that the scan resp data that is passed to the controller is the same
  // that was given when calling the API:
  Scan_Response_Data_t scan_resp_data_out;
  cl_assert_equal_i(gap_le_get_scan_response_data(&scan_resp_data_out),
                    sizeof(scan_resp_data));
  cl_assert(memcmp(&scan_resp_data_out, scan_resp_data,
                   sizeof(scan_resp_data)) == 0);

  // Expect one regular timer to be running for advertisements:
  cl_assert_equal_i(regular_timer_seconds_count(), 1);
  // Unschedule callback should not have been called:
  cl_assert_equal_i(s_unscheduled_cb_count, 0);

  // Unschedule and expect not to be advertising any more:
  gap_le_advert_unschedule(job);
  cl_assert(!gap_le_is_advertising_enabled());

  // Unschedule callback should have been called once:
  cl_assert_equal_i(s_unscheduled_cb_count, 1);
  cl_assert_equal_p(s_unscheduled_job, job);
  cl_assert_equal_b(s_unscheduled_completed, false);

  // Expect no advertisement timer
  cl_assert_equal_i(regular_timer_seconds_count(), 0);

  free(ad);
}

void test_gap_le_advert__single_job_multiple_terms_silence_and_loop_around(void) {
  BLEAdData *ad = create_ad("yo", NULL);

  GAPLEAdvertisingJobTerm advert_terms[] =
  {
    {
      .interval = GAPLEAdvertisingInterval_Short,
      .duration_secs = 1,
    },
    {
      .interval = GAPLEAdvertisingInterval_Long,
      .duration_secs = 1,
    },
    {
      .duration_secs = GAPLE_ADVERTISING_DURATION_LOOP_AROUND,
      .loop_around_index = 1,
    },
  };
  GAPLEAdvertisingJobRef job;
  job = gap_le_advert_schedule(ad,
                               advert_terms,
                               sizeof(advert_terms)/sizeof(GAPLEAdvertisingJobTerm),
                               unscheduled_callback, s_unscheduled_cb_data, 0);
  cl_assert(job);

  // First term:
  cl_assert(gap_le_is_advertising_enabled());
  assert_ad_data("yo");
  gap_le_assert_advertising_interval(GAPLEAdvertisingInterval_Short);

  regular_timer_fire_seconds(1);

  // Second term:
  cl_assert(gap_le_is_advertising_enabled());
  assert_ad_data("yo");
  gap_le_assert_advertising_interval(GAPLEAdvertisingInterval_Long);

  regular_timer_fire_seconds(1);

  // Looped around to second term (index==1):
  cl_assert(gap_le_is_advertising_enabled());
  assert_ad_data("yo");
  gap_le_assert_advertising_interval(GAPLEAdvertisingInterval_Long);

  cl_assert_equal_i(s_unscheduled_cb_count, 0);
}

void test_gap_le_advert__single_job_multiple_terms(void) {
  const char ad_data[] = "ad data";
  const char scan_resp_data[] = "scan resp data";
  BLEAdData *ad = create_ad(ad_data, scan_resp_data);

  GAPLEAdvertisingJobTerm advert_terms[2] =
  {
    {
      .interval = GAPLEAdvertisingInterval_Short,
      .duration_secs = 4,
    },
    {
      .interval = GAPLEAdvertisingInterval_Long,
      .duration_secs = 4,
    },
  };
  GAPLEAdvertisingJobRef job;
  job = gap_le_advert_schedule(ad,
                               advert_terms,
                               sizeof(advert_terms)/sizeof(GAPLEAdvertisingJobTerm),
                               unscheduled_callback, s_unscheduled_cb_data, 0);
  cl_assert(job);

  // Since there was nothing scheduled, expect that the ad data is set to the
  // controller immediately:
  cl_assert(gap_le_is_advertising_enabled());

  // Check that the ad data that is passed to the controller is the same that
  // was given when calling the API:
  assert_ad_data(ad_data);

  // Check that the scan resp data that is passed to the controller is the same
  // that was given when calling the API:
  Scan_Response_Data_t scan_resp_data_out;
  cl_assert_equal_i(gap_le_get_scan_response_data(&scan_resp_data_out),
                    sizeof(scan_resp_data));
  cl_assert(memcmp(&scan_resp_data_out, scan_resp_data,
                   sizeof(scan_resp_data)) == 0);

  // Expect one regular timer to be running for adverts:
  cl_assert_equal_i(regular_timer_seconds_count(), 1);
  // Unschedule callback should not have been called:
  cl_assert_equal_i(s_unscheduled_cb_count, 0);

  // Make sure all the terms in the job are run:
  GAPLEAdvertisingInterval expected_intervals[] = {
    GAPLEAdvertisingInterval_Short,
    GAPLEAdvertisingInterval_Long,
  };

  for (int term = 0; term < ARRAY_LENGTH(advert_terms); ++term) {
    for (int second_tick = 0; second_tick < 4; ++second_tick) {
      cl_assert_equal_i(s_unscheduled_cb_count, 0);
      assert_ad_data("ad data");
      gap_le_assert_advertising_interval(expected_intervals[term]);
      regular_timer_fire_seconds(1);
    }
  }

  cl_assert(!gap_le_is_advertising_enabled());
  cl_assert_equal_i(s_unscheduled_cb_count, 1);
  cl_assert_equal_p(s_unscheduled_job, job);

  // Expect no advertisement timer
  cl_assert_equal_i(regular_timer_seconds_count(), 0);

  free(ad);
}

void test_gap_le_advert__job_round_robin(void) {
  GAPLEAdvertisingJobTerm advert_term = {
    .interval = GAPLEAdvertisingInterval_Short,
    .duration_secs = GAPLE_ADVERTISING_DURATION_INFINITE,
  };

  // Schedule infinite job "A":
  BLEAdData *ad_a = create_ad("A", NULL);
  GAPLEAdvertisingJobRef infinite_job_a;
  infinite_job_a = gap_le_advert_schedule(ad_a,
                                          &advert_term, sizeof(advert_term)/sizeof(GAPLEAdvertisingJobTerm),
                                          unscheduled_callback, s_unscheduled_cb_data, 0);
  cl_assert(infinite_job_a);
  assert_ad_data("A");

  // Schedule infinite job "B":
  BLEAdData *ad_b = create_ad("B", NULL);
  GAPLEAdvertisingJobRef infinite_job_b;
  infinite_job_b = gap_le_advert_schedule(ad_b,
                                          &advert_term, sizeof(advert_term)/sizeof(GAPLEAdvertisingJobTerm),
                                          unscheduled_callback, s_unscheduled_cb_data, 0);
  cl_assert(infinite_job_b);
  assert_ad_data("B");

  // Round-robin 10 times:
  for (int i = 0; i < 10; ++i) {
    regular_timer_fire_seconds(1);
    assert_ad_data("A");
    regular_timer_fire_seconds(1);
    assert_ad_data("B");
  }

  // Introduce non-infinite job "C" for 10 seconds:
  advert_term.duration_secs = 10;
  BLEAdData *ad_c = create_ad("C", NULL);
  GAPLEAdvertisingJobRef infinite_job_c;
  infinite_job_c = gap_le_advert_schedule(ad_c,
                                          &advert_term, sizeof(advert_term)/sizeof(GAPLEAdvertisingJobTerm),
                                          unscheduled_callback, s_unscheduled_cb_data, 0);
  cl_assert(infinite_job_c);
  assert_ad_data("C");

  // Round-robin 5 times:
  for (int i = 0; i < 5; ++i) {
    regular_timer_fire_seconds(1);
    assert_ad_data("A");
    regular_timer_fire_seconds(1);
    assert_ad_data("B");
    regular_timer_fire_seconds(1);
    assert_ad_data("C");
  }

  // Introduce a second non-infinite job "D" for 10 seconds:
  BLEAdData *ad_d = create_ad("D", NULL);
  GAPLEAdvertisingJobRef infinite_job_d;
  infinite_job_d = gap_le_advert_schedule(ad_d,
                                          &advert_term, sizeof(advert_term)/sizeof(GAPLEAdvertisingJobTerm),
                                          unscheduled_callback, s_unscheduled_cb_data, 0);
  // It should get immediate air-time for one cycle:
  cl_assert(infinite_job_d);
  assert_ad_data("D");

  // Round-robin 4 times:
  for (int i = 0; i < 4; ++i) {
    regular_timer_fire_seconds(1);
    assert_ad_data("A");
    regular_timer_fire_seconds(1);
    assert_ad_data("B");
    regular_timer_fire_seconds(1);
    assert_ad_data("C");
    regular_timer_fire_seconds(1);
    assert_ad_data("D");
  }

  // Schedule yet another infinite job "E".
  BLEAdData *ad_e = create_ad("E", NULL);
  advert_term.duration_secs = GAPLE_ADVERTISING_DURATION_INFINITE;
  GAPLEAdvertisingJobRef infinite_job_e;
  infinite_job_e = gap_le_advert_schedule(ad_e,
                                          &advert_term, sizeof(advert_term)/sizeof(GAPLEAdvertisingJobTerm),
                                          unscheduled_callback, s_unscheduled_cb_data, 0);
  cl_assert(infinite_job_e);
  // Infinite jobs are equal in priority to finite jobs, so it should get immediate air-time for one cycle:
  assert_ad_data("E");

  // No jobs should been have been unscheduled:
  cl_assert_equal_i(s_unscheduled_cb_count, 0);

  // This is the last round for "C":
  regular_timer_fire_seconds(1);
  assert_ad_data("A");
  regular_timer_fire_seconds(1);
  assert_ad_data("B");
  regular_timer_fire_seconds(1);
  assert_ad_data("C");
  regular_timer_fire_seconds(1);
  assert_ad_data("D");

  // One job ("C") should been have been unscheduled:
  cl_assert_equal_i(s_unscheduled_cb_count, 1);


  // Round-robin 5 times:
  for (int i = 0; i < 5; ++i) {
    regular_timer_fire_seconds(1);
    assert_ad_data("E");
    regular_timer_fire_seconds(1);
    assert_ad_data("A");
    regular_timer_fire_seconds(1);
    assert_ad_data("B");
    regular_timer_fire_seconds(1);
    assert_ad_data("D");
  }

  // "D" should be done now, so expect only infinite jobs to get air-time again:
  for (int i = 0; i < 10; ++i) {
    regular_timer_fire_seconds(1);
    assert_ad_data("E");
    regular_timer_fire_seconds(1);
    assert_ad_data("A");
    regular_timer_fire_seconds(1);
    assert_ad_data("B");

    // Jobs "C" and "D" should have been unscheduled, infinite jobs should never
    // get unscheduled:
    cl_assert_equal_i(s_unscheduled_cb_count, 2);
    cl_assert_equal_b(s_unscheduled_completed, true);
  }

  free(ad_a);
  free(ad_b);
  free(ad_c);
  free(ad_d);
}


void test_gap_le_advert__job_round_robin_multiple_terms(void) {
  GAPLEAdvertisingJobTerm advert_terms[2] =
  {
    {
      .interval = GAPLEAdvertisingInterval_Short,
      .duration_secs = 5,
    },
    {
      .interval = GAPLEAdvertisingInterval_Long,
      .duration_secs = 5,
    },
  };

  // Schedule job "A":
  BLEAdData *ad_a = create_ad("A", NULL);
  GAPLEAdvertisingJobRef job_a;
  job_a = gap_le_advert_schedule(ad_a,
                                 advert_terms, sizeof(advert_terms)/sizeof(GAPLEAdvertisingJobTerm),
                                 unscheduled_callback, s_unscheduled_cb_data, 0);
  cl_assert(job_a);
  assert_ad_data("A");

  // Schedule job "B":
  BLEAdData *ad_b = create_ad("B", NULL);
  GAPLEAdvertisingJobRef job_b;
  job_b = gap_le_advert_schedule(ad_b,
                                 advert_terms, sizeof(advert_terms)/sizeof(GAPLEAdvertisingJobTerm),
                                 unscheduled_callback, s_unscheduled_cb_data, 0);
  cl_assert(job_b);
  assert_ad_data("B");

  // Round-robin. Each term is 5 seconds, each job is 10 seconds:
  for (int i = 0; i < 9; ++i) {
    regular_timer_fire_seconds(1);
    assert_ad_data("A");
    cl_assert_equal_i(s_unscheduled_cb_count, 0);
    regular_timer_fire_seconds(1);
    assert_ad_data("B");
    cl_assert_equal_i(s_unscheduled_cb_count, 0);
  }

  // Unschedule job "B", air job "A"
  regular_timer_fire_seconds(1);
  assert_ad_data("A");
  cl_assert_equal_i(s_unscheduled_cb_count, 1);
  cl_assert_equal_p(s_unscheduled_job, job_b);
  cl_assert_equal_b(s_unscheduled_completed, true);

  // Unschedule job "A"
  regular_timer_fire_seconds(1);
  cl_assert_equal_i(s_unscheduled_cb_count, 2);
  cl_assert_equal_p(s_unscheduled_job, job_a);
  cl_assert_equal_b(s_unscheduled_completed, true);

  cl_assert(!gap_le_is_advertising_enabled());

  free(ad_a);
  free(ad_b);
}

void test_gap_le_advert__expiring_job(void) {
  // Test that a job is expired correctly after the set duration:
  uint16_t seconds_left = 5;
  GAPLEAdvertisingJobTerm advert_term = {
    .interval = GAPLEAdvertisingInterval_Short,
    .duration_secs = seconds_left,
  };

  BLEAdData *ad = create_ad(NULL, NULL);
  GAPLEAdvertisingJobRef job;
  job = gap_le_advert_schedule(ad, &advert_term, sizeof(advert_term)/sizeof(GAPLEAdvertisingJobTerm),
                               unscheduled_callback, s_unscheduled_cb_data, 0);
  cl_assert(job);

  // No jobs should been have been unscheduled:
  cl_assert_equal_i(s_unscheduled_cb_count, 0);

  while (seconds_left) {
    cl_assert(gap_le_is_advertising_enabled());

    regular_timer_fire_seconds(1);
    --seconds_left;
  }

  cl_assert(!gap_le_is_advertising_enabled());
  cl_assert_equal_i(regular_timer_seconds_count(), 0);

  // The job should been have been unscheduled:
  cl_assert_equal_i(s_unscheduled_cb_count, 1);
  cl_assert_equal_p(s_unscheduled_job, job);
  cl_assert_equal_b(s_unscheduled_completed, true);

  free(ad);
}

void test_gap_le_advert__invalid_params(void) {
  GAPLEAdvertisingJobRef job;
  BLEAdData *ad = create_ad(NULL, NULL);

  GAPLEAdvertisingJobTerm advert_term = {
    .interval = GAPLEAdvertisingInterval_Short,
    .duration_secs = 1,
  };

  // Valid term should succeed:
  job = gap_le_advert_schedule(ad, &advert_term, sizeof(advert_term)/sizeof(GAPLEAdvertisingJobTerm),
                               unscheduled_callback, s_unscheduled_cb_data, 0);
  cl_assert(job);

  // Loop-around in the first term:
  advert_term.duration_secs = GAPLE_ADVERTISING_DURATION_LOOP_AROUND;
  job = gap_le_advert_schedule(ad, &advert_term, sizeof(advert_term)/sizeof(GAPLEAdvertisingJobTerm),
                               unscheduled_callback, s_unscheduled_cb_data, 0);
  cl_assert_equal_p(job, NULL);

  // No terms:
  job = gap_le_advert_schedule(ad, NULL, 0,
                               unscheduled_callback, s_unscheduled_cb_data, 0);
  cl_assert_equal_p(job, NULL);

  // No ad data:
  advert_term.duration_secs = 1;
  job = gap_le_advert_schedule(NULL, &advert_term,
                               sizeof(advert_term)/sizeof(GAPLEAdvertisingJobTerm),
                               unscheduled_callback, s_unscheduled_cb_data, 0);
  cl_assert_equal_p(job, NULL);

  free(ad);
}

void test_gap_le_advert__unschedule_non_existent(void) {
  // Unscheduling non-existent job should be fine, should not crash:
  gap_le_advert_unschedule((GAPLEAdvertisingJobRef)(uintptr_t) 0x1234);

  // Unschedule callback should not have been called:
  cl_assert_equal_i(s_unscheduled_cb_count, 0);
}

void test_gap_le_advert__deinit_unschedules(void) {
  BLEAdData *ad = create_ad(NULL, NULL);

  GAPLEAdvertisingJobTerm advert_term = {
    .interval = GAPLEAdvertisingInterval_Short,
    .duration_secs = 10,
  };
  GAPLEAdvertisingJobRef job;
  job = gap_le_advert_schedule(ad, &advert_term, sizeof(advert_term)/sizeof(GAPLEAdvertisingJobTerm),
                               unscheduled_callback, s_unscheduled_cb_data, 0);
  cl_assert(job);
  gap_le_advert_deinit();
  cl_assert_equal_i(s_unscheduled_cb_count, 1);
  cl_assert_equal_p(s_unscheduled_job, job);
  cl_assert_equal_b(s_unscheduled_completed, false);
  cl_assert_equal_i(regular_timer_seconds_count(), 0);
  free(ad);
}

void test_gap_le_advert__cant_schedule_after_deinit(void) {
  gap_le_advert_deinit();
  BLEAdData *ad = create_ad(NULL, NULL);
  GAPLEAdvertisingJobTerm advert_term = {
    .interval = GAPLEAdvertisingInterval_Short,
    .duration_secs = 10,
  };
  GAPLEAdvertisingJobRef job;
  job = gap_le_advert_schedule(ad, &advert_term, sizeof(advert_term)/sizeof(GAPLEAdvertisingJobTerm),
                               unscheduled_callback, s_unscheduled_cb_data, 0);
  cl_assert_equal_p(job, NULL);
  cl_assert_equal_i(regular_timer_seconds_count(), 0);
}

void test_gap_le_advert__continue_after_slave_connection(void) {
  BLEAdData *ad = create_ad(NULL, NULL);
  GAPLEAdvertisingJobTerm advert_term = {
    .interval = GAPLEAdvertisingInterval_Short,
    .duration_secs = 10,
  };
  GAPLEAdvertisingJobRef job;
  job = gap_le_advert_schedule(ad, &advert_term, sizeof(advert_term)/sizeof(GAPLEAdvertisingJobTerm),
                               unscheduled_callback, s_unscheduled_cb_data, 0);
  cl_assert_equal_b(gap_le_is_advertising_enabled(), true);

  // Simulate stopping advertising because of inbound connection:
  gap_le_set_advertising_disabled();
  s_is_connected_as_slave = true;

  // Call the connection handler:
  gap_le_advert_handle_connect_as_slave();
  // we should have stopped advertising for reconnection
  cl_assert_equal_b(gap_le_is_advertising_enabled(), false);

  // While connected as slave, the cycle timer must not re-enable advertising:
  // the bt_driver contract does not advertise during a connection.
  regular_timer_fire_seconds(1);
  cl_assert_equal_b(gap_le_is_advertising_enabled(), false);

  // Once disconnected, advertising resumes since the advertisement job is still
  // scheduled:
  gap_le_advert_handle_disconnect_as_slave();
  cl_assert_equal_b(gap_le_is_advertising_enabled(), true);

  free(ad);
}

// A stack restart while connected (airplane mode) never runs the disconnect
// handler, so init() has to clear the connected state itself.
void test_gap_le_advert__stack_restart_while_connected_resets_state(void) {
  gap_le_advert_handle_connect_as_slave();
  gap_le_advert_deinit();
  gap_le_advert_init();

  BLEAdData *ad = create_ad("A", NULL);
  GAPLEAdvertisingJobTerm advert_term = {
    .interval = GAPLEAdvertisingInterval_Short,
    .duration_secs = 10,
  };
  GAPLEAdvertisingJobRef job = gap_le_advert_schedule(
      ad, &advert_term, 1, unscheduled_callback, s_unscheduled_cb_data, 0);
  cl_assert(job != NULL);
  cl_assert_equal_b(gap_le_is_advertising_enabled(), true);

  // The job must age and complete on schedule:
  for (int i = 0; i < 10; ++i) {
    cl_assert_equal_b(gap_le_is_advertising_enabled(), true);
    regular_timer_fire_seconds(1);
  }
  cl_assert_equal_b(gap_le_is_advertising_enabled(), false);
  cl_assert_equal_i(s_unscheduled_cb_count, 1);
  cl_assert_equal_b(s_unscheduled_completed, true);

  free(ad);
}

void test_gap_le_advert__unschedule_job_types(void) {
  BLEAdData *ad = create_ad(NULL, NULL);
  GAPLEAdvertisingJobTerm advert_term = {
    .interval = GAPLEAdvertisingInterval_Short,
    .duration_secs = 10,
  };
  GAPLEAdvertisingJobRef job_a;
  job_a = gap_le_advert_schedule(ad, &advert_term, sizeof(advert_term)/sizeof(GAPLEAdvertisingJobTerm),
                               unscheduled_callback, s_unscheduled_cb_data, GAPLEAdvertisingJobTagDiscovery);

  GAPLEAdvertisingJobTag tag = GAPLEAdvertisingJobTagDiscovery;
  gap_le_advert_unschedule_job_types(&tag, 1);

  // make sure we can remove a tag when it is the only one scheduled
  cl_assert(s_unscheduled_job == job_a);
  cl_assert_equal_i(s_unscheduled_cb_count, 1);


  // add back the job we just unscheduled
  job_a = gap_le_advert_schedule(ad, &advert_term, sizeof(advert_term)/sizeof(GAPLEAdvertisingJobTerm),
                                 unscheduled_callback, s_unscheduled_cb_data, GAPLEAdvertisingJobTagDiscovery);

  GAPLEAdvertisingJobRef job_b;
  job_b = gap_le_advert_schedule(ad, &advert_term, sizeof(advert_term)/sizeof(GAPLEAdvertisingJobTerm),
                               unscheduled_callback, s_unscheduled_cb_data, GAPLEAdvertisingJobTagReconnection);


  GAPLEAdvertisingJobRef job_c;
  job_c = gap_le_advert_schedule(ad, &advert_term, sizeof(advert_term)/sizeof(GAPLEAdvertisingJobTerm),
                               unscheduled_callback, s_unscheduled_cb_data, GAPLEAdvertisingJobTagReconnection);

  // run some Ad cycling
  for (int i = 0; i < 3; i++) {
    regular_timer_fire_seconds(1);
  }

  cl_assert_equal_b(gap_le_is_advertising_enabled(), true);

  // make sure we can remove a few jobs with the same advertisement type
  tag = GAPLEAdvertisingJobTagReconnection;
  gap_le_advert_unschedule_job_types(&tag, 1);

  // should have 3 jobs unscheduled at this point and 1 still running
  cl_assert_equal_i(s_unscheduled_cb_count, 3);
  cl_assert_equal_b(gap_le_is_advertising_enabled(), true);

  // make sure after we unschedule all the jobs no more are running
  tag = GAPLEAdvertisingJobTagDiscovery;
  gap_le_advert_unschedule_job_types(&tag, 1);
  cl_assert_equal_i(s_unscheduled_cb_count, 4);
  cl_assert_equal_b(gap_le_is_advertising_enabled(), false);

  free(ad);
}
