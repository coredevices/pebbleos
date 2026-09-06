/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <math.h>
#include <time.h>

#include <clar.h>

// Tests
///////////////////////////////////////////////////////////
void test_mktime__birthdays(void) {
  struct tm francois_birthday = {
    .tm_sec = 0,
    .tm_min = 44,
    .tm_hour = 10,
    .tm_mday = 30,
    .tm_mon = 4,
    .tm_year = 89,
  }; 
  cl_assert_equal_i(mktime(&francois_birthday), 612528240);

  struct tm rons_birthday = {
    .tm_sec = 17,
    .tm_min = 1,
    .tm_hour = 9,
    .tm_mday = 10,
    .tm_mon = 4,
    .tm_year = 63,
  };
  cl_assert_equal_i(mktime(&rons_birthday), -1);

  struct tm alex_marianetti_birthday = {
    .tm_sec = 29,
    .tm_min = 4,
    .tm_hour = 17,
    .tm_mday = 2,
    .tm_mon = 9,
    .tm_year = 107,
  };
  cl_assert_equal_i(mktime(&alex_marianetti_birthday), 1191344669);

  struct tm chris_birthday = {
    .tm_sec = 59,
    .tm_min = 3,
    .tm_hour = 10,
    .tm_mday = 15,
    .tm_mon = 5,
    .tm_year = 89,
  };
  cl_assert_equal_i(mktime(&chris_birthday), 613908239);
}

void test_mktime__epoch(void) {
  struct tm epoch = {
    .tm_sec = 0,
    .tm_min = 0,
    .tm_hour = 0,
    .tm_mday = 1,
    .tm_mon = 0,
    .tm_year = 70,
  };
  cl_assert_equal_i(mktime(&epoch), 0);
}

void test_mktime__leap(void) {
  struct tm real_leap = {
    .tm_sec = 0,
    .tm_min = 0,
    .tm_hour = 10,
    .tm_mday = 29,
    .tm_mon = 1,
    .tm_year = 112,
  };
  cl_assert_equal_i(mktime(&real_leap), 1330509600);
}

