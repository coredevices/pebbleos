/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kernel/util/interval_timer.h"

#include "fakes/fake_rtc.h"

#include "clar.h"

void passert_failed_no_message(const char* filename, int line_number) {
}

void pbl_irq_lock(void) {
}

void pbl_irq_unlock(void) {
}

void test_interval_timer__initialize(void) {
  fake_rtc_init(0, 0);
}

void test_interval_timer__simple(void) {
  IntervalTimer timer;
  interval_timer_init(&timer, 0, UINT32_MAX, 2);

  uint32_t num_intervals;
  uint32_t average_ms;

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 0);
  cl_assert_equal_i(average_ms, 0);

  interval_timer_take_sample(&timer);

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 0);
  cl_assert_equal_i(average_ms, 0);

  fake_rtc_increment_time_ms(1000);
  interval_timer_take_sample(&timer);

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 1);
  cl_assert_equal_i(average_ms, 1000);

  fake_rtc_increment_time_ms(1000);
  interval_timer_take_sample(&timer);

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 2);
  cl_assert_equal_i(average_ms, 1000);

  fake_rtc_increment_time_ms(1030);
  interval_timer_take_sample(&timer);

  // = 1000 + (0.5 * (1030 - 1000))
  // = 1000 + (0.5 * 30)
  // = 1015

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 3);
  cl_assert_equal_i(average_ms, 1015);
}

void test_interval_timer__invalid_samples(void) {
  IntervalTimer timer;
  interval_timer_init(&timer, 800, 1200, 2);

  uint32_t num_intervals;
  uint32_t average_ms;

  interval_timer_take_sample(&timer);

  fake_rtc_increment_time_ms(1000);
  interval_timer_take_sample(&timer);

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 1);
  cl_assert_equal_i(average_ms, 1000);

  // valid interval
  fake_rtc_increment_time_ms(1020);
  interval_timer_take_sample(&timer);

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 2);
  cl_assert_equal_i(average_ms, 1010);

  // invalid interval, too high
  fake_rtc_increment_time_ms(1220);
  interval_timer_take_sample(&timer);

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 2);
  cl_assert_equal_i(average_ms, 1010);

  // invalid interval, too low
  fake_rtc_increment_time_ms(780);
  interval_timer_take_sample(&timer);

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 2);
  cl_assert_equal_i(average_ms, 1010);

  // valid interval
  fake_rtc_increment_time_ms(1010);
  interval_timer_take_sample(&timer);

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 3);
  cl_assert_equal_i(average_ms, 1010);
}

// Make sure we don't run into any issues when our internals are close to UINT32_MAX
void test_interval_timer__big_interval(void) {
  IntervalTimer timer;
  interval_timer_init(&timer, 0, UINT32_MAX, 2);

  uint32_t num_intervals;
  uint32_t average_ms;

  interval_timer_take_sample(&timer);

  fake_rtc_increment_time_ms(3000000000);
  interval_timer_take_sample(&timer);

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 1);
  cl_assert_equal_i(average_ms, 3000000000);

  fake_rtc_increment_time_ms(3000000000);
  interval_timer_take_sample(&timer);

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 2);
  cl_assert_equal_i(average_ms, 3000000000);

  fake_rtc_increment_time_ms(3000000000);
  interval_timer_take_sample(&timer);

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 3);
  cl_assert_equal_i(average_ms, 3000000000);
}

void test_interval_timer__moving_average(void) {
  IntervalTimer timer;
  interval_timer_init(&timer, 0, UINT32_MAX, 4);

  uint32_t num_intervals;
  uint32_t average_ms;

  interval_timer_take_sample(&timer);

  fake_rtc_increment_time_ms(1000);
  interval_timer_take_sample(&timer);

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 1);
  cl_assert_equal_i(average_ms, 1000);

  fake_rtc_increment_time_ms(1020);
  interval_timer_take_sample(&timer);

  // = 1000 + (0.25 * (1020 - 1000))
  // = 1000 + (0.25 * 20)
  // = 1005

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 2);
  cl_assert_equal_i(average_ms, 1005);

  fake_rtc_increment_time_ms(1010);
  interval_timer_take_sample(&timer);

  // = 1005 + (0.25 * (1010 - 1005))
  // = 1005 + (0.25 * 5)
  // = 1006

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 3);
  cl_assert_equal_i(average_ms, 1006);

  fake_rtc_increment_time_ms(1010);
  interval_timer_take_sample(&timer);

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 4);
  cl_assert_equal_i(average_ms, 1007);

  fake_rtc_increment_time_ms(1030);
  interval_timer_take_sample(&timer);

  // = 1007 + (0.25 * (1030 - 1007))
  // = 1007 + (0.25 * 23)
  // = 1007 + 5
  // = 1012

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 5);
  cl_assert_equal_i(average_ms, 1012);

  fake_rtc_increment_time_ms(1030);
  interval_timer_take_sample(&timer);

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 6);
  cl_assert_equal_i(average_ms, 1016);

  // Take a bunch of samples to make sure the moving average moves up to the new interval
  for (int i = 0; i < 10; ++i) {
    fake_rtc_increment_time_ms(1030);
    interval_timer_take_sample(&timer);
  }

  num_intervals = interval_timer_get(&timer, &average_ms);
  cl_assert_equal_i(num_intervals, 16);
  // Close enough, rounding issues will prevent us from every actually hitting 1030
  // = 1027 + (0.25 * (1030 - 1027))
  // = 1027 + (0.25 * 3)
  // = 1027
  cl_assert_equal_i(average_ms, 1027);
}

