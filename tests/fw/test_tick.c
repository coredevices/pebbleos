/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/types.h"


#include <stdlib.h>
#include <string.h>

#include "clar.h"

// Stubs
///////////////////////////////////////////////////////////
#include "stubs_passert.h"
#include "stubs_logging.h"
#include "stubs_tick.h"

// Tests
///////////////////////////////////////////////////////////

static const double s_tick_rate_hz = PBL_TICK_HZ;

static double pbl_ms_to_ticks_double(double milliseconds) {
  return (milliseconds * s_tick_rate_hz) / 1000.0;
}

void test_tick__should_convert_48h_to_ticks(void) {
  uint32_t time_ms = 24 * 7 * 60 * 60 * 1000;
  cl_assert_equal_i(pbl_ms_to_ticks(time_ms), (uint32_t)pbl_ms_to_ticks_double(time_ms));
}

void test_tick__should_convert_max_to_ticks(void) {
  const uint32_t uint32_max = ~0;

  // The maximum input time possible until overflow
  uint32_t max_time_ms = ((double)uint32_max * 1000.0) / s_tick_rate_hz;

  cl_assert(pbl_ms_to_ticks(max_time_ms) == (uint32_max - 1));
  cl_assert(pbl_ms_to_ticks(max_time_ms) == (uint32_t)pbl_ms_to_ticks_double(max_time_ms));
}
