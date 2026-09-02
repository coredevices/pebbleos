/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/ambient_light.h>

#include "board/board.h"
#include "pbl/kernel/mutex.h"
#include "system/passert.h"

#include <stdbool.h>
#include <stdint.h>

// Refcount framework for prime/release/suspend/resume. Drivers receive only
// the resulting (active, sampling) booleans via ambient_light_driver_set_state.

static PBL_MUTEX_DEFINE(s_mutex);
static bool s_initialized;
static uint16_t s_prime_refcount;
static uint16_t s_suspend_refcount;

static void prv_apply_locked(void) {
  const bool active = (s_prime_refcount > 0U);
  const bool sampling = active && (s_suspend_refcount == 0U);
  ambient_light_driver_set_state(active, sampling);
}

void ambient_light_common_init(void) {
  s_initialized = true;
}

void ambient_light_prime(void) {
  if (!s_initialized) {
    return;
  }
  pbl_mutex_lock(&s_mutex, PBL_FOREVER);
  s_prime_refcount++;
  prv_apply_locked();
  pbl_mutex_unlock(&s_mutex);
}

void ambient_light_release(void) {
  if (!s_initialized) {
    return;
  }
  pbl_mutex_lock(&s_mutex, PBL_FOREVER);
  PBL_ASSERTN(s_prime_refcount > 0U);
  s_prime_refcount--;
  prv_apply_locked();
  pbl_mutex_unlock(&s_mutex);
}

void ambient_light_suspend(void) {
  if (!s_initialized) {
    return;
  }
  pbl_mutex_lock(&s_mutex, PBL_FOREVER);
  s_suspend_refcount++;
  prv_apply_locked();
  pbl_mutex_unlock(&s_mutex);
}

void ambient_light_resume(void) {
  if (!s_initialized) {
    return;
  }
  pbl_mutex_lock(&s_mutex, PBL_FOREVER);
  PBL_ASSERTN(s_suspend_refcount > 0U);
  s_suspend_refcount--;
  prv_apply_locked();
  pbl_mutex_unlock(&s_mutex);
}

bool ambient_light_lux_available(void) {
  return BOARD_CONFIG.ambient_light_lux_den != 0U;
}

uint32_t ambient_light_level_to_lux(uint32_t light_level) {
  const uint32_t den = BOARD_CONFIG.ambient_light_lux_den;
  const uint32_t offset = BOARD_CONFIG.ambient_light_lux_dark_offset;
  if (den == 0U) {
    // Uncalibrated board: keep the raw-count domain so thresholds tuned in
    // counts continue to behave identically.
    return light_level;
  }
  if (light_level <= offset) {
    return 0;
  }
  return (uint32_t)(((uint64_t)(light_level - offset) *
                     BOARD_CONFIG.ambient_light_lux_num) / den);
}
