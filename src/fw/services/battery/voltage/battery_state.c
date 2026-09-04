/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/battery/battery_state.h"

#ifdef CONFIG_QEMU
#include <pbl/drivers/qemu/qemu_battery.h>
#endif

#include "board/board.h"
#include "debug/power_tracking.h"
#include <pbl/drivers/battery.h>
#include "kernel/events.h"
#include "pbl/services/analytics/analytics.h"
#include "pbl/services/battery/battery_curve.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/system_task.h"
#include "syscall/syscall_internal.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/math.h"
#include "util/ratio.h"

PBL_LOG_MODULE_DECLARE(service_battery, CONFIG_SERVICE_BATTERY_LOG_LEVEL);

#ifdef DEBUG_BATTERY_STATE
#define BATTERY_SAMPLE_RATE_MS 1000
#else
#define BATTERY_SAMPLE_RATE_MS (60 * 1000)
#endif

typedef void (*EntryFunc)(void);

typedef enum {
  ConnectionStateInvalid,
  ConnectionStateChargingPlugged,
  ConnectionStateDischargingPlugged,
  ConnectionStateDischargingUnplugged
} ConnectionStateID;

typedef struct ConnectionState {
  EntryFunc enter;
} ConnectionState;

static void prv_update_plugged_change(void);
static void prv_update_done_charging(void);

static const ConnectionState s_transitions[] = {
  [ConnectionStateChargingPlugged] = { .enter = prv_update_plugged_change },
  [ConnectionStateDischargingPlugged] = { .enter = prv_update_done_charging },
  [ConnectionStateDischargingUnplugged] = { .enter = prv_update_plugged_change }
};

typedef struct BatteryState {
  uint64_t init_time;
  uint32_t percent;
  uint16_t voltage;
  ConnectionStateID connection;
} BatteryState;

static BatteryState s_last_battery_state;
static TimerID s_periodic_timer_id = TIMER_INVALID_ID;
static int s_analytics_previous_mv = 0;
static uint32_t s_analytics_last_cpct = 0;

static void prv_schedule_update(uint32_t delay, bool force_update);
PreciseBatteryChargeState prv_get_precise_charge_state(const BatteryState *state);

static void prv_transition(BatteryState *state, ConnectionStateID next_state) {
  state->connection = next_state;
  s_transitions[state->connection].enter();
}

static void prv_update_plugged_change(void) {
  // If the connection state changed or we finished charging, reset the filter since we're
  // probably switching to a new curve.
  battery_state_reset_filter();

  bool is_charging = battery_charge_controller_thinks_we_are_charging();

  if (is_charging) {
    PBL_ANALYTICS_TIMER_START(battery_charge_time_ms);
    PBL_ANALYTICS_TIMER_STOP(battery_discharge_duration_ms);
  } else {
    bool is_plugged = battery_is_usb_connected();

    PBL_ANALYTICS_TIMER_STOP(battery_charge_time_ms);

    if (!is_plugged) {
      PBL_ANALYTICS_TIMER_START(battery_discharge_duration_ms);
    } else {
      PBL_ANALYTICS_TIMER_STOP(battery_discharge_duration_ms);
    }
  }
}

static void prv_update_done_charging(void) {
  prv_update_plugged_change();

  // Amount in mV to drop the "Full" voltage by to briefly stay at 100% once unplugged
  const uint16_t BATTERY_FULL_FUDGE_AMOUNT = 10;
  PBL_LOG_DBG("Done charging - Updating curve");
  battery_curve_set_full_voltage(s_last_battery_state.voltage - BATTERY_FULL_FUDGE_AMOUNT);
}

static void battery_state_put_change_event(PreciseBatteryChargeState state) {
  PebbleEvent e = {
    .type = PEBBLE_BATTERY_STATE_CHANGE_EVENT,
    .battery_state = {
      .new_state = state,
    },
  };
  event_put(&e);
}

void battery_state_reset_filter(void) {
  s_last_battery_state.voltage = battery_get_millivolts();
  // Reset the stablization timer in case we encountered a current spike during the reset
  s_last_battery_state.init_time = rtc_get_ticks();
}

static uint32_t prv_filter_voltage(uint32_t avg_mv, uint32_t battery_mv) {
  // Basic low-pass filter - See PBL-23637
  const uint8_t VOLTAGE_FILTER_BETA = 2;
  uint32_t avg = (avg_mv << VOLTAGE_FILTER_BETA);
  avg -= avg_mv;
  avg += battery_mv;
  return avg >> VOLTAGE_FILTER_BETA;
}

static bool prv_is_stable(const BatteryState *state) {
  // After a reboot, we typically source a lot of current which can drastically impact
  // our mV readings due to the internal resistance of the battery. We use the
  // system_likely_stabilized flag as an indicator of how trustworthy our readings are
  const uint64_t STABLE_TICKS = 3 * 60 * RTC_TICKS_HZ;
  uint64_t elapsed_ticks = rtc_get_ticks() - state->init_time;
  return elapsed_ticks > STABLE_TICKS;
}

static ConnectionStateID prv_get_connection_state(void) {
  const bool charging = battery_charge_controller_thinks_we_are_charging();
  const bool plugged_in = battery_is_usb_connected();

  if (plugged_in) {
    if (charging) {
      return ConnectionStateChargingPlugged;
    } else {
      return ConnectionStateDischargingPlugged;
    }
  } else {
    if (charging) {
      // Since we can't be charging and disconnected,
      // just log a warning and pretend we aren't charging.
      PBL_LOG_WRN("PMIC reported charging while unplugged - ignoring");
    }
    return ConnectionStateDischargingUnplugged;
  }
}

static void prv_update_state(void *force_update) {
  bool forced = (bool)force_update;

  // Driver communication

  bool state_changed = false;
  ConnectionStateID next_state = prv_get_connection_state();
  if (s_last_battery_state.connection != next_state) {
    // Do not allow DischargingPlugged -> ChargingPlugged state
    if ((s_last_battery_state.connection != ConnectionStateDischargingPlugged) ||
        (next_state != ConnectionStateChargingPlugged)) {
      prv_transition(&s_last_battery_state, next_state);
      state_changed = true;
    }
  }

  s_last_battery_state.voltage = prv_filter_voltage(s_last_battery_state.voltage,
                                                    battery_get_millivolts());
  bool charging = (s_last_battery_state.connection == ConnectionStateChargingPlugged);

  // Update Percent & Filtering

  const uint32_t ALWAYS_UPDATE_THRESHOLD = ratio32_from_percent(10);
  bool likely_stable = prv_is_stable(&s_last_battery_state);

  uint32_t new_charge_percent =
      battery_curve_sample_ratio32_charge_percent(s_last_battery_state.voltage, charging);
  // Allow updates iff:
  // - We are charging
  // - We are discharging and:
  //    - The readings have stabilized and the battery percent did not go up
  //    - The readings have not yet stablized
  // TL;DR: Allow updates unless we're stable and discharging but the % went up.
  if (!charging && likely_stable &&
      new_charge_percent > s_last_battery_state.percent) {
    // It's okay to return early since any connection/plugged changes will reset the filter,
    // so we won't catch those.
    return;
  }

  s_last_battery_state.percent = new_charge_percent;

  PBL_LOG_DBG("mV Raw: %"PRIu16" Ratio: %"PRIu32" Percent: %"PRIu32,
          s_last_battery_state.voltage, s_last_battery_state.percent,
          ratio32_to_percent(s_last_battery_state.percent));

  PWR_TRACK_BATT(charging ? "CHARGING" : "DISCHARGING", s_last_battery_state.voltage);

  if (forced || likely_stable || s_last_battery_state.percent <= ALWAYS_UPDATE_THRESHOLD ||
      charging || state_changed) {
    battery_state_put_change_event(prv_get_precise_charge_state(&s_last_battery_state));
  }
}

static void prv_update_callback(void *data) {
  // Running the battery monitor on the timer task is not a good idea because
  // we could be sampling right in the middle of a flash erase, etc. Therefore,
  // dispatch to a lower priority task
  system_task_add_callback(prv_update_state, data);

  // Reschedule ourselves again so we create a loop
  prv_schedule_update(BATTERY_SAMPLE_RATE_MS, false);
}

static void prv_schedule_update(uint32_t delay, bool force_update) {
  bool success = new_timer_start(s_periodic_timer_id, delay, prv_update_callback,
      (void *)force_update, 0 /*flags*/);
  PBL_ASSERTN(success);
}

void battery_state_force_update(void) {
  // Fire off our periodic timer. Note that we rely on the callback to reschedule the timer
  // for 1 minute intervals rather than create it as a repeating timer. This is because
  // we occasionally want the callback to get triggered immediately
  // (in response to the charging cable being plugged in). In these instances, we reschedule it
  // from the main task.
  prv_schedule_update(0, true);
}

void battery_state_init(void) {
  s_periodic_timer_id = new_timer_create();

  s_last_battery_state = (BatteryState) { .connection = ConnectionStateDischargingUnplugged };
  battery_state_reset_filter();
  battery_state_force_update();

  s_analytics_previous_mv = s_last_battery_state.voltage;
  s_analytics_last_cpct = (s_last_battery_state.percent * 10000U) / RATIO32_MAX;
}

void battery_state_handle_connection_event(bool is_connected) {
  static const uint32_t RECONNECTION_DELAY_MS = 1000;

  PBL_LOG_VERBOSE("USB Connected:%d", is_connected);

  // Trigger a reset update to the state machine. Delay the update to allow the battery voltage to
  // settle and to debounce reconnection events
  prv_schedule_update(RECONNECTION_DELAY_MS, true);
}

PreciseBatteryChargeState prv_get_precise_charge_state(const BatteryState *state) {
  PreciseBatteryChargeState event_state = {
    .charge_percent = state->percent,
    .pct = ratio32_to_percent(state->percent),
    .is_charging = (s_last_battery_state.connection == ConnectionStateChargingPlugged),
    .is_plugged = (s_last_battery_state.connection != ConnectionStateDischargingUnplugged)
  };
  return event_state;
}

DEFINE_SYSCALL(BatteryChargeState, sys_battery_get_charge_state, void) {
  return battery_get_charge_state();
}

BatteryChargeState battery_get_charge_state(void) {
  bool is_plugged = (s_last_battery_state.connection != ConnectionStateDischargingUnplugged);

#ifdef CONFIG_QEMU
  // Read the exact percent the host set via `pebble emu-battery --percent N`,
  // skipping the lossy voltage-curve roundtrip and the low-power-reserve remap
  // so the watch displays exactly what was requested.
  int32_t percent = MIN((int32_t)qemu_battery_get_percent(), 100);
  int32_t percent_normalized = percent;
  uint8_t charge_percent = (uint8_t)percent;
#else
  int32_t percent = ratio32_to_percent(s_last_battery_state.percent);

  // subtract low power reserve, so developer will see 0% when we're approaching low power mode
  int32_t percent_normalized = MAX((percent - BOARD_CONFIG_POWER.low_power_threshold
                  + percent / (100 / BOARD_CONFIG_POWER.low_power_threshold)), 0);

  // massage rounding factor so that between 100% to 50% charge the SOC reported is biased to a
  // higher charge percent bin.
  int32_t rounding_factor = 5 + MAX(((percent - 50) / 10), 0);
  uint8_t charge_percent = MIN(10 * ((percent_normalized + rounding_factor) / 10), 100);
#endif
  BatteryChargeState state = {
    .charge_percent = charge_percent,
    .is_charging = is_plugged && percent_normalized < 100,
    .is_plugged = is_plugged,
  };
  return state;
}

// For unit tests
TimerID battery_state_get_periodic_timer_id(void) {
  return s_periodic_timer_id;
}

uint16_t battery_state_get_voltage(void) {
  return s_last_battery_state.voltage;
}

int32_t battery_state_get_temp(void) {
  return 0;
}

#include "console/prompt.h"
void command_print_battery_status(void) {
  char buffer[32];
  PreciseBatteryChargeState state = prv_get_precise_charge_state(&s_last_battery_state);
  prompt_send_response_fmt(buffer, 32, "%"PRIu16" mV", s_last_battery_state.voltage);
  prompt_send_response_fmt(buffer, 32,
      "batt_percent: %"PRIu32"%%", ratio32_to_percent(state.charge_percent));
  prompt_send_response_fmt(buffer, 32, "plugged: %s", state.is_plugged ? "YES" : "NO");
  prompt_send_response_fmt(buffer, 32, "charging: %s", state.is_charging ? "YES" : "NO");
}

/////////////////
// Analytics

// Note that this is run on a different thread than battery_state!
void pbl_analytics_external_collect_battery(void) {
  int32_t battery_mv = s_last_battery_state.voltage;
  uint32_t battery_soc_cpct = (s_last_battery_state.percent * 10000U) / RATIO32_MAX;
  int32_t d_mv;
  uint32_t d_soc_cpct;

  d_mv = battery_mv - s_analytics_previous_mv;
  PBL_ANALYTICS_SET_UNSIGNED(battery_voltage, battery_mv);
  PBL_ANALYTICS_SET_SIGNED(battery_voltage_delta, d_mv);
  s_analytics_previous_mv = battery_mv;

  d_soc_cpct = MAX((int32_t)s_analytics_last_cpct - (int32_t)battery_soc_cpct, 0);
  PBL_ANALYTICS_SET_UNSIGNED(battery_soc_pct, battery_soc_cpct);
  PBL_ANALYTICS_SET_UNSIGNED(battery_soc_pct_drop, d_soc_cpct);
  s_analytics_last_cpct = battery_soc_cpct;
}

static void prv_set_forced_charge_state(bool is_charging) {
  battery_force_charge_enable(is_charging);

  // Trigger an immediate update to the state machine: may trigger an event
  battery_state_force_update();
}

void command_battery_charge_option(const char* option) {
  if (!strcmp("disable", option)) {
    prv_set_forced_charge_state(false);
  } else if (!strcmp("enable", option)) {
    prv_set_forced_charge_state(true);
  }
}
