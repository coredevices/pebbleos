/* SPDX-FileCopyrightText: 2026 Adrian Pascu */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/battery/battery_charge_limit.h"
#include "pbl/services/battery/battery_state.h"

#include "clar.h"

// Stubs
////////////////////////////////////
#include "stubs_logging.h"
#include "stubs_passert.h"

#include <pbl/drivers/battery.h>
#include "shell/prefs.h"

static uint8_t s_pref_limit_pct;
static bool s_force_disabled;
static bool s_charge_enabled;
static int s_charge_enable_calls;
static int s_force_updates;

uint8_t shell_prefs_get_charge_limit_pct(void) {
  return s_pref_limit_pct;
}

bool battery_is_charge_force_disabled(void) {
  return s_force_disabled;
}

void battery_set_charge_enable(bool charging_enabled) {
  s_charge_enabled = charging_enabled;
  ++s_charge_enable_calls;
}

void battery_state_force_update(void) {
  ++s_force_updates;
}

extern bool s_limit_active;

static PreciseBatteryChargeState prv_state(uint8_t pct, bool is_plugged) {
  return (PreciseBatteryChargeState) {
    .pct = pct,
    .is_plugged = is_plugged,
    .is_charging = is_plugged,
  };
}

// Setup
////////////////////////////////////
void test_battery_charge_limit__initialize(void) {
  s_limit_active = false;
  s_pref_limit_pct = CHARGE_LIMIT_PCT_DISABLED;
  s_force_disabled = false;
  s_charge_enabled = true;
  s_charge_enable_calls = 0;
  s_force_updates = 0;
}

// Tests
////////////////////////////////////
void test_battery_charge_limit__inactive_when_disabled(void) {
  battery_charge_limit_evaluate(prv_state(85, true));

  cl_assert(!s_limit_active);
  cl_assert_equal_i(s_charge_enable_calls, 0);
}

void test_battery_charge_limit__pauses_charging_at_limit(void) {
  s_pref_limit_pct = 80;

  battery_charge_limit_evaluate(prv_state(79, true));
  cl_assert(!s_limit_active);
  cl_assert_equal_i(s_charge_enable_calls, 0);

  battery_charge_limit_evaluate(prv_state(80, true));
  cl_assert(s_limit_active);
  cl_assert(!s_charge_enabled);
  cl_assert_equal_i(s_charge_enable_calls, 1);

  battery_charge_limit_evaluate(prv_state(80, true));
  cl_assert_equal_i(s_charge_enable_calls, 1);
}

void test_battery_charge_limit__respects_configured_limit(void) {
  s_pref_limit_pct = 90;

  battery_charge_limit_evaluate(prv_state(85, true));
  cl_assert(!s_limit_active);
  cl_assert_equal_i(s_charge_enable_calls, 0);

  battery_charge_limit_evaluate(prv_state(90, true));
  cl_assert(s_limit_active);
  cl_assert(!s_charge_enabled);
}

void test_battery_charge_limit__resumes_below_limit(void) {
  s_pref_limit_pct = 80;
  battery_charge_limit_evaluate(prv_state(80, true));
  cl_assert(!s_charge_enabled);

  battery_charge_limit_evaluate(prv_state(79, true));
  cl_assert(!s_limit_active);
  cl_assert(s_charge_enabled);
}

void test_battery_charge_limit__limit_raise_resumes_charging(void) {
  s_pref_limit_pct = 80;
  battery_charge_limit_evaluate(prv_state(80, true));
  cl_assert(!s_charge_enabled);

  s_pref_limit_pct = 90;
  battery_charge_limit_evaluate(prv_state(80, true));
  cl_assert(!s_limit_active);
  cl_assert(s_charge_enabled);
}

void test_battery_charge_limit__unplug_reenables_charging(void) {
  s_pref_limit_pct = 80;
  battery_charge_limit_evaluate(prv_state(80, true));
  cl_assert(!s_charge_enabled);

  battery_charge_limit_evaluate(prv_state(80, false));
  cl_assert(!s_limit_active);
  cl_assert(s_charge_enabled);

  battery_charge_limit_evaluate(prv_state(50, true));
  cl_assert(!s_limit_active);
  cl_assert(s_charge_enabled);
}

void test_battery_charge_limit__disable_reenables_charging(void) {
  s_pref_limit_pct = 80;
  battery_charge_limit_evaluate(prv_state(80, true));
  cl_assert(!s_charge_enabled);

  s_pref_limit_pct = CHARGE_LIMIT_PCT_DISABLED;
  battery_charge_limit_evaluate(prv_state(80, true));
  cl_assert(!s_limit_active);
  cl_assert(s_charge_enabled);
}

void test_battery_charge_limit__leaves_force_disabled_charger_alone(void) {
  s_pref_limit_pct = 80;
  battery_charge_limit_evaluate(prv_state(80, true));
  cl_assert(s_limit_active);
  cl_assert_equal_i(s_charge_enable_calls, 1);

  s_force_disabled = true;
  battery_charge_limit_evaluate(prv_state(50, true));
  cl_assert(!s_limit_active);
  cl_assert(!s_charge_enabled);
  cl_assert_equal_i(s_charge_enable_calls, 1);

  s_force_disabled = false;
  s_charge_enabled = true;
  battery_charge_limit_evaluate(prv_state(80, true));
  cl_assert(s_limit_active);
  cl_assert(!s_charge_enabled);
  cl_assert_equal_i(s_charge_enable_calls, 2);
}

void test_battery_charge_limit__pref_change_forces_update(void) {
  battery_charge_limit_handle_pref_change();
  cl_assert_equal_i(s_force_updates, 1);
}
