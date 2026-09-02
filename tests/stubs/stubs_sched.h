/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/kernel/sched.h>

bool pbl_kernel_is_started(void) {
  return true;
}

bool pbl_kernel_is_running(void) {
  return true;
}

bool pbl_sched_is_locked(void) {
  return false;
}

pbl_tick_t pbl_uptime_ticks(void) {
  return 0;
}
