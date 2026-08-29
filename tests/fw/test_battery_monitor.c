/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/battery/battery_charge_limit.h"
#include "pbl/services/battery/battery_monitor.h"
#include "pbl/services/battery/battery_state.h"
#include "pbl/services/battery/battery_curve.h"

#include "clar.h"

// Stubs
////////////////////////////////////
#include "stubs_analytics.h"
#include "stubs_logging.h"
#include "stubs_passert.h"
#include "stubs_prompt.h"
#include "stubs_serial.h"
#include "fake_new_timer.h"
#include "fake_battery.h"
#include "fake_system_task.h"
#include "fake_rtc.h"

#include "kernel/events.h"
#include <pbl/logging/logging.h>
#include "system/reboot_reason.h"

static bool s_entered_standby;
static bool s_in_low_power;
static bool s_error_window_shown;
static bool s_warning_window_shown;
static PebbleEvent s_last_event_put;

bool battery_is_usb_connected_raw(void) {
  return false;
}

void low_power_standby(void) {
  s_entered_standby = true;
}

void low_power_exit(void) {
  s_in_low_power = false;
}

void low_power_enter(void) {
  s_in_low_power = true;
}

bool low_power_is_active(void) {
  return s_in_low_power;
}

bool firmware_update_is_in_progress(void) {
  return false;
}

void battery_force_charge_enable(bool is_charging) { }

void battery_charge_limit_evaluate(PreciseBatteryChargeState state) { }

static void periodic_timer_trigger(int count) {
  TimerID timer_id = battery_state_get_periodic_timer_id();
  cl_assert(timer_id != TIMER_INVALID_ID);
  cl_assert(stub_new_timer_is_scheduled(timer_id));
  for (int i=0; i<count; ++i) {
    stub_new_timer_fire(timer_id);
    fake_system_task_callbacks_invoke_pending();
  }
}

static void standby_timer_trigger(int count) {
  TimerID timer_id = battery_monitor_get_standby_timer_id();

  cl_assert(timer_id != TIMER_INVALID_ID);
  cl_assert(stub_new_timer_is_scheduled(timer_id));
  for (int i = 0; i<count; ++i) {
    stub_new_timer_fire(timer_id);
    fake_system_task_callbacks_invoke_pending();
  }
}

static bool standby_timer_is_scheduled() {
  TimerID timer_id = battery_monitor_get_standby_timer_id();
  return timer_id != TIMER_INVALID_ID && stub_new_timer_is_scheduled(timer_id);
}

static uint32_t standby_timer_get_timeout() {
  TimerID timer_id = battery_monitor_get_standby_timer_id();

  cl_assert(timer_id != TIMER_INVALID_ID);
  cl_assert(stub_new_timer_is_scheduled(timer_id));
  return stub_new_timer_timeout(timer_id);
}

void enter_standby(RebootReasonCode reason) {
  s_entered_standby = true;
}

void event_put(PebbleEvent* event) {
  s_last_event_put = *event;

  if (event->type == PEBBLE_BATTERY_STATE_CHANGE_EVENT) {
    battery_monitor_handle_state_change_event(event->battery_state.new_state);
  } else if (event->type == PEBBLE_BATTERY_CONNECTION_EVENT) {
    battery_state_handle_connection_event(event->battery_connection.is_connected);
    periodic_timer_trigger(1);
  }
}

// Setup
////////////////////////////////////
void test_battery_monitor__initialize(void) {
  //g_pbl_log_enabled = true;
  //g_pbl_log_level = 255;

  s_entered_standby = false;
  s_error_window_shown = false;
  s_warning_window_shown = false;
  s_in_low_power = false;
  fake_rtc_init(0, 0);
  fake_rtc_auto_increment_ticks(0);

  // The discharge curve's 100% reference is mutated by
  // battery_curve_set_full_voltage() during charge-complete handling and
  // persists across tests; restore it so each test starts from a clean curve.
  battery_curve_reset_for_tests();
}

void test_battery_monitor__cleanup(void) {
}

// Tests
////////////////////////////////////

int32_t battery_curve_lookup_percent_with_scaling_factor(
    int battery_mv, bool is_charging, uint32_t scaling_factor);

void test_battery_monitor__scaled_reading(void) {
  int32_t scaling_factor = INT32_MAX / 100;
  int32_t prev_reading = 0;

  // run through a wide range of battery readings. Confirm as the mv increases,
  // the percentage reported increases. Use the largest scaling factor to check
  // for integer overflows
  for (int mv = 3000; mv < 5000; mv++) {

    int32_t res = battery_curve_lookup_percent_with_scaling_factor(
        mv, false, scaling_factor);

    cl_assert(prev_reading <= res);
    prev_reading = res;
  }

  // make sure that when we compute the largest possible (100% - 0%) and lowest possible
  // (0% - 100%) battery delta that we don't overflow the computation
  int32_t start_percent = battery_curve_lookup_percent_with_scaling_factor(
       2000, false, scaling_factor);

  int32_t end_percent = battery_curve_lookup_percent_with_scaling_factor(
       5000, false, scaling_factor);

  int32_t delta_percent = end_percent - start_percent;
  cl_assert(delta_percent > (INT32_MAX - 100));

  delta_percent = start_percent - end_percent;
  cl_assert(delta_percent < (INT32_MIN + 100));
}

// Check that the percentage reported is somewhat protected from transient voltage changes
void test_battery_monitor__charge_fluctuate_voltage(void) {
  int high_percent = 70;
  int low_percent = 20;
  int high_mv = battery_curve_lookup_voltage_by_percent(high_percent, true);
  int low_mv = battery_curve_lookup_voltage_by_percent(low_percent, true);

  fake_battery_init(high_mv, true, true);

  battery_monitor_init();
  periodic_timer_trigger(1);
  // For the first sample, it will be identical
  cl_assert_equal_i(battery_get_charge_state().charge_percent, high_percent);

  // ...and should stay that way
  periodic_timer_trigger(10);
  cl_assert_equal_i(battery_get_charge_state().charge_percent, high_percent);

  // Then, when the voltage drops, the percentage should begin to decline - but should not reach the low value yet
  fake_battery_set_millivolts(low_mv);
  periodic_timer_trigger(1);
  int delta = high_percent - battery_get_charge_state().charge_percent;
  cl_assert(delta >= 0);
  cl_assert(delta < high_percent - low_percent);

  // But, it should approach that value over time
  int last_delta = delta;
  while(battery_get_charge_state().charge_percent > low_percent) {
    periodic_timer_trigger(1);
    delta = high_percent - battery_get_charge_state().charge_percent;
    cl_assert(delta >= last_delta);
    last_delta = delta;
  }

  cl_assert_equal_i(battery_get_charge_state().charge_percent, low_percent);
}

void test_battery_monitor__connection_reset(void) {
  // Test for PBL-19951: Reset charge percent on reconnection events
  int percent = 10;
  int charge_mv = battery_curve_lookup_voltage_by_percent(percent, true);
  int discharge_mv = battery_curve_lookup_voltage_by_percent(percent, false);

  fake_battery_init(discharge_mv, false, false);

  battery_monitor_init();
  periodic_timer_trigger(1);
  cl_assert_equal_i(battery_get_charge_state().charge_percent, percent);

  fake_battery_set_charging(true);
  fake_battery_set_millivolts(charge_mv);
  fake_battery_set_connected(true);
  cl_assert_equal_i(battery_get_charge_state().charge_percent, percent);

  fake_battery_set_charging(false);
  fake_battery_set_millivolts(discharge_mv);
  fake_battery_set_connected(false);
  cl_assert_equal_i(battery_get_charge_state().charge_percent, percent);
}

void test_battery_monitor__curve_adjustment_when_charge_complete(void) {
  int charge_mv = battery_curve_lookup_voltage_by_percent(0, true);
  int full_mv = battery_curve_lookup_voltage_by_percent(100, false);
  int charge_terminate_mv = battery_curve_lookup_voltage_by_percent(95, false);

  fake_battery_init(charge_mv, true, true);

  battery_monitor_init();
  periodic_timer_trigger(1);

  fake_battery_set_millivolts(charge_terminate_mv);
  fake_battery_set_charging(false);
  periodic_timer_trigger(1);

  cl_assert_equal_i(battery_get_charge_state().charge_percent, 100);

  fake_battery_set_millivolts(full_mv);
  periodic_timer_trigger(1);

  cl_assert_equal_i(battery_get_charge_state().charge_percent, 100);
}

void test_battery_monitor__curve_doesnt_shift_too_far(void) {
  int charge_mv = battery_curve_lookup_voltage_by_percent(0, true);
  int charge_terminate_mv = battery_curve_lookup_voltage_by_percent(80, false);

  fake_battery_init(charge_mv, true, true);

  battery_monitor_init();
  periodic_timer_trigger(1);

  fake_battery_set_millivolts(charge_terminate_mv);
  fake_battery_set_charging(false);
  periodic_timer_trigger(1);

  cl_assert_equal_i(battery_get_charge_state().charge_percent, 80);
}

/*
good -> lpm
lpm -> good

good -> critical
critical -> lpm

lpm -> critical
critical -> good
*/
// Must mirror the PowerStateID enum in src/fw/services/battery/battery_monitor.c.
// PowerStatePluggedIn was added in 9d3a9548f ("add plugged in status"): whenever
// the watch is plugged in, it ignores battery level/charge status and operates
// normally, so the state machine reports PluggedIn rather than Good/LowPower.
typedef enum {
  PowerStateGood,
  PowerStateLowPower,
  PowerStateCritical,
  PowerStatePluggedIn,
  PowerStateStandby
} PowerStateID;
extern PowerStateID s_power_state;

void test_battery_monitor__transitions(void) {
  int good_mv = battery_curve_lookup_voltage_by_percent(100, false);
  int low_mv = battery_curve_lookup_voltage_by_percent(3, false);
  int critical_mv = battery_curve_lookup_voltage_by_percent(0, false);

  fake_battery_init(good_mv, false, false);

  battery_monitor_init();
  periodic_timer_trigger(1);
  cl_assert(!s_in_low_power && !battery_monitor_critical_lockout());
  cl_assert_equal_i(s_power_state, PowerStateGood);

  // good -> lpm
  fake_battery_set_millivolts(low_mv);
  periodic_timer_trigger(10);
  cl_assert(s_in_low_power);
  cl_assert_equal_i(s_power_state, PowerStateLowPower);

  // lpm -> good (plugged in => PluggedIn, which resumes normal operation)
  fake_battery_set_charging(true);
  fake_battery_set_connected(true);
  periodic_timer_trigger(1);
  cl_assert(!s_in_low_power);
  cl_assert_equal_i(s_power_state, PowerStatePluggedIn);

  // good -> critical
  fake_battery_set_millivolts(critical_mv);
  fake_battery_set_charging(false);
  fake_battery_set_connected(false);
  periodic_timer_trigger(20);
  cl_assert(battery_monitor_critical_lockout());
  cl_assert_equal_i(s_power_state, PowerStateCritical);

  // critical -> lpm (only possible if unstable)
  fake_battery_set_millivolts(low_mv);
  battery_state_force_update();
  periodic_timer_trigger(1);
  cl_assert(s_in_low_power);
  cl_assert_equal_i(s_power_state, PowerStateLowPower);

  // lpm -> critical
  fake_battery_set_millivolts(critical_mv);
  periodic_timer_trigger(20);
  cl_assert(battery_monitor_critical_lockout());
  cl_assert(s_in_low_power);
  cl_assert_equal_i(s_power_state, PowerStateCritical);

  // critical -> good (plugged in => PluggedIn, which resumes normal operation)
  fake_battery_set_charging(true);
  fake_battery_set_connected(true);
  fake_battery_set_millivolts(good_mv);
  periodic_timer_trigger(20);
  cl_assert(!battery_monitor_critical_lockout());
  cl_assert(!s_in_low_power);
  cl_assert_equal_i(s_power_state, PowerStatePluggedIn);
}

void test_battery_monitor__low_first_run(void) {
  int low_mv = battery_curve_lookup_voltage_by_percent(3, false);

  fake_battery_init(low_mv, false, false);

  battery_monitor_init();
  periodic_timer_trigger(1);
  cl_assert(battery_monitor_critical_lockout());

  cl_assert_equal_i(standby_timer_get_timeout(), 2000);
  standby_timer_trigger(1);
  cl_assert(s_entered_standby);
}

void test_battery_monitor__critical(void) {
  int good_mv = battery_curve_lookup_voltage_by_percent(10, false);
  int critical_mv = battery_curve_lookup_voltage_by_percent(0, false);

  fake_battery_init(good_mv, false, false);

  battery_monitor_init();
  periodic_timer_trigger(1);
  cl_assert(!battery_monitor_critical_lockout());

  fake_battery_set_millivolts(critical_mv);
  periodic_timer_trigger(25);
  cl_assert(battery_monitor_critical_lockout());

  cl_assert_equal_i(standby_timer_get_timeout(), 30000);
  standby_timer_trigger(1);
  cl_assert(s_entered_standby);
}

void test_battery_monitor__critical_plugged_in(void) {
  int good_mv = battery_curve_lookup_voltage_by_percent(10, false);
  int critical_mv = battery_curve_lookup_voltage_by_percent(0, false);

  fake_battery_init(good_mv, false, false);

  battery_monitor_init();
  periodic_timer_trigger(1);
  cl_assert(!battery_monitor_critical_lockout());

  fake_battery_set_millivolts(critical_mv);
  periodic_timer_trigger(25);
  cl_assert(battery_monitor_critical_lockout());

  cl_assert_equal_i(standby_timer_get_timeout(), 30000);

  // Plugging in transitions to PowerStatePluggedIn, whose exit-from-critical
  // action (prv_exit_critical) stops the standby timer, so standby is averted.
  fake_battery_set_charging(true);
  fake_battery_set_connected(true);
  periodic_timer_trigger(1);
  cl_assert(!standby_timer_is_scheduled());
  cl_assert(!s_entered_standby);
}

void test_battery_monitor__increase_discharging(void) {
  int low_mv = battery_curve_lookup_voltage_by_percent(50, false);
  int high_mv = battery_curve_lookup_voltage_by_percent(100, false);
  int lower_mv = battery_curve_lookup_voltage_by_percent(20, false);

  fake_battery_init(low_mv, false, false);
  fake_rtc_auto_increment_ticks(50000);

  battery_monitor_init();
  periodic_timer_trigger(5);
  cl_assert_equal_i(battery_get_charge_state().charge_percent, 50);

  // Should be stable by now
  // Shouldn't update percent (actually, shouldn't even send events.)
  PBL_LOG_DBG("Shouldn't be any updates");
  PBL_LOG_DBG("▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼");
  fake_battery_set_millivolts(high_mv);
  periodic_timer_trigger(20);
  cl_assert_equal_i(battery_get_charge_state().charge_percent, 50);
  PBL_LOG_DBG("▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲");

  // Should still update if it goes lower
  fake_battery_set_millivolts(lower_mv);
  periodic_timer_trigger(20);
  cl_assert_equal_i(battery_get_charge_state().charge_percent, 20);
}

void test_battery_monitor__connection_states(void) {
  int charge_mv = battery_curve_lookup_voltage_by_percent(60, true);
  int okay_mv = battery_curve_lookup_voltage_by_percent(5, false);
  int discharge_mv = battery_curve_lookup_voltage_by_percent(3, false);

  // Begin in LPM, unplugged and discharging.
  fake_battery_init(okay_mv, false, false);
  battery_monitor_init();
  periodic_timer_trigger(1);
  fake_battery_set_millivolts(discharge_mv);
  periodic_timer_trigger(1);
  cl_assert(s_in_low_power);

  // If we somehow begin charging, ignore it.
  fake_battery_set_charging(true);
  periodic_timer_trigger(1);
  cl_assert(s_in_low_power);

  // If we're charging and connected, reset the filter
  fake_battery_set_millivolts(charge_mv);
  fake_battery_set_connected(true);
  periodic_timer_trigger(1);
  cl_assert(!s_in_low_power && battery_get_charge_state().charge_percent == 60);

  // Discharging but connected: prv_update_done_charging() shifts the discharge
  // curve's 100% reference down to (current voltage - fudge). With the current
  // gabbro/silk curve data the 60% charge voltage (3970mV) is far below the
  // discharge curve's 90% point (4120mV), and battery_curve_set_full_voltage()
  // clamps the full-voltage so it cannot drop below that 90% point + 1. So the
  // 3970mV reading interpolates to ~73% raw, which battery_get_charge_state()
  // bins to 70% - it no longer clamps to 100%.
  fake_battery_set_charging(false);
  periodic_timer_trigger(1);
  cl_assert_equal_i(battery_get_charge_state().charge_percent, 70);
}

void test_battery_monitor__battery_get_charge_state(void) {
  // range through all discrete percentages and verify that battery_get_charge_state()
  // returns sane values
  BatteryChargeState result;
  uint8_t last_charge_percent = 0;
  uint8_t last_discharge_percent = 100;
  for (uint32_t charge_percent = 0; charge_percent <= 100; ++charge_percent) {
    int charge_mv = battery_curve_lookup_voltage_by_percent(charge_percent, true);
    int discharge_mv = battery_curve_lookup_voltage_by_percent(100 - charge_percent, false);
    //
    // test as if the battery is plugged and charging
    //
    bool charging = charge_percent < 100;
    fake_battery_init(charge_mv, true, charging);
    battery_monitor_init();
    periodic_timer_trigger(1);
    result = battery_get_charge_state();
    // due to fudge factors we merely check that the percentage is in range and
    // that it is monotonically increasing
    cl_assert((result.charge_percent >= 0) && (result.charge_percent <= 100));
    cl_assert(result.charge_percent >= last_charge_percent);
    cl_assert(result.is_charging == charging);
    cl_assert(result.is_plugged);
    last_charge_percent = result.charge_percent;
    //
    // test as if the battery is unplugged and discharging
    //
    fake_battery_init(discharge_mv, false, false);
    battery_monitor_init();
    periodic_timer_trigger(1);
    result = battery_get_charge_state();
    // due to fudge factors we merely check that the percentage is in range and
    // that it is monotonically decreasing
    cl_assert((result.charge_percent >= 0) && (result.charge_percent <= 100));
    cl_assert(result.charge_percent <= last_discharge_percent);
    cl_assert(!result.is_charging);
    cl_assert(!result.is_plugged);
    last_discharge_percent = result.charge_percent;
  }
}
