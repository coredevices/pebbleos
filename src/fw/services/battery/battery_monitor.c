/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/battery/battery_monitor.h"

#include "board/board.h"
#include "kernel/low_power.h"
#include "kernel/util/standby.h"
#include "pbl/services/battery/battery_charge_limit.h"
#include "pbl/services/firmware_update.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/system_task.h"
#include <pbl/logging/logging.h>
#include "util/ratio.h"

#include <stdint.h>

PBL_LOG_MODULE_DEFINE(service_battery, CONFIG_SERVICE_BATTERY_LOG_LEVEL);

// State machine stuff

typedef void (*Action)(void);

typedef struct PowerState {
  Action enter;
  Action exit;
} PowerState;

typedef enum {
  PowerStateGood,
  PowerStateLowPower,
  PowerStateCritical,
  PowerStatePluggedIn,
  PowerStateStandby
} PowerStateID;

static void prv_enter_lpm(void);
static void prv_exit_lpm(void);
static void prv_begin_standby_timer(void);
static void prv_enter_standby(void);
static void prv_exit_critical(void);

static const PowerState power_states[] = {
  [PowerStateGood] = { 0 },
  [PowerStateLowPower] = { .enter = prv_enter_lpm, .exit = prv_exit_lpm },
  [PowerStateCritical] = { .enter = prv_begin_standby_timer, .exit = prv_exit_critical },
  [PowerStatePluggedIn] = { 0 },
  [PowerStateStandby] = { .enter = prv_enter_standby }
};

////////////////////////
// Business logic
static TimerID s_standby_timer_id = TIMER_INVALID_ID;
T_STATIC PowerStateID s_power_state;
static bool s_low_on_first_run;
static bool s_first_run;

static void prv_transition(PowerStateID next_state) {
  if (next_state == s_power_state) {
    return;
  }
  PowerStateID old_state = s_power_state;
  s_power_state = next_state;
  if (power_states[old_state].exit) {
    power_states[old_state].exit();
  }
  if (power_states[next_state].enter) {
    power_states[next_state].enter();
  }
}

static void prv_enter_lpm(void) {
  if (!firmware_update_is_in_progress()) {
    low_power_enter();
  }
  PBL_LOG_INFO("Battery low: enter low power mode");
}

static void prv_resume_normal_operation(void) {
  low_power_exit();
  PBL_LOG_INFO("Battery good: resume normal operation");
}

static void prv_exit_critical(void) {
  // Cancel the standby timer so we don't enter standby if we're no longer critical
  // (e.g. charger was plugged in before the timer expired).
  new_timer_stop(s_standby_timer_id);

  // Checking the state here is a bit of a hack because the state machine does not have proper
  // transition actions, only entry/exit actions.
  // We check that the state is PowerStateGood because the state machine does not transition through
  // all states in between the new and old states in a transition.
  if (s_power_state == PowerStateGood || s_power_state == PowerStatePluggedIn) {
    prv_resume_normal_operation();
  }
}

static void prv_exit_lpm(void) {
  // Checking the state here is a bit of a hack because the state machine does not have proper
  // transition actions, only entry/exit actions
  if (s_power_state == PowerStateGood || s_power_state == PowerStatePluggedIn) {
    prv_resume_normal_operation();
  }
}

static void prv_standby_timer_callback(void* data) {
  // FIXME This is so broken: battery_state_force_update schedules a new timer callback to execute
  // immediately, which then pends a background task callback to perform the update, so this will
  // never update before we check the power_state.
  battery_state_force_update();
  if (s_power_state == PowerStateCritical) {
    // Still critical after timeout, transition to standby
    prv_transition(PowerStateStandby);
  }
}

static void prv_begin_standby_timer(void) {
  PBL_LOG_INFO("Battery critical: begin standby timer");
  // If the watch was already running, give them 30s, otherwise just 2s.
  uint32_t standby_timeout = (s_first_run) ? 2000: 30000;
  new_timer_start(s_standby_timer_id, standby_timeout,
      prv_standby_timer_callback, NULL, 0 /*flags*/);
}

static void system_task_handle_battery_critical(void* data) {
  PBL_LOG_INFO("Battery critical: go to standby mode");
  if (low_power_is_active()) {
    low_power_standby();
  } else {
    enter_standby(RebootReasonCode_LowBattery);
  }
}

static void prv_enter_standby(void) {
  system_task_add_callback(system_task_handle_battery_critical, NULL);
}

static void prv_log_battery_state(PreciseBatteryChargeState state) {
  const uint16_t k_min_percent_diff = 5;
  const uint16_t percent = ratio32_to_percent(state.charge_percent);

  union LoggingBattState{
    struct {
      uint16_t is_charging:1;
      uint16_t is_plugged:1;
      uint16_t percent:14;
    };
    uint16_t all;
  };
  static union LoggingBattState s_prev_batt_state;

  union LoggingBattState new_batt_state = {
    .percent = percent / k_min_percent_diff,
    .is_charging = state.is_charging,
    .is_plugged = state.is_plugged,
  };

  if ((percent < BOARD_CONFIG_POWER.low_power_threshold) ||
      (s_prev_batt_state.all != new_batt_state.all) ||
      s_first_run) {
        s_prev_batt_state.all = new_batt_state.all;
      }
}

void battery_monitor_handle_state_change_event(PreciseBatteryChargeState state) {
  // Update Critical/Low Power Mode

  // Standby behaviour, as gleaned from the previous implementation:
  //  Once the battery voltage falls below exactly 0%, the standby lockout is displayed.
  //  If the USB cable is disconnected, the standby timer starts. This standby delay is 2s
  //    (if at first start), otherwise it is 30s (if the watch was already running).
  //  The shutdown can be averted if the watch is plugged in before the timer expires.
  //  Similarly, if the battery voltage has rebounded when the timer expires, the shutdown
  //    will not occur.

#ifdef CONFIG_QEMU
  // QEMU has no real battery, so never enter LPM or standby — otherwise
  // `pebble emu-battery --percent 1` (and any low value) shuts the emulator
  // down or locks it into the low-power UI.
  bool critical = false;
  bool low_power = false;
  s_low_on_first_run = false;
#else
  bool critical = (state.charge_percent == 0) && !state.is_charging;

#ifndef CONFIG_RECOVERY_FW
  const uint32_t LOW_POWER_PERCENT = ratio32_from_percent(BOARD_CONFIG_POWER.low_power_threshold);

  bool low_power = !state.is_charging && (state.charge_percent <= LOW_POWER_PERCENT);
  if (low_power && s_first_run && !state.is_plugged) {
    s_low_on_first_run = true;
  } else if (!low_power) {
    s_low_on_first_run = false;
  }
#else
  const uint32_t PRF_LOW_POWER_THRESHOLD_PERCENT = ratio32_from_percent(5);

  // We want to keep the LPM UI up until we've hit 10% regardless of charging
  bool low_power = state.charge_percent < PRF_LOW_POWER_THRESHOLD_PERCENT;
  s_low_on_first_run = false;
#endif
#endif

  PowerStateID new_state;

  if (state.is_plugged) {
    new_state = PowerStatePluggedIn;
  } else if (critical || s_low_on_first_run) {
    new_state = PowerStateCritical;
  } else if (low_power) {
    new_state = PowerStateLowPower;
  } else {
    new_state = PowerStateGood;
  }

  // All state transitions are valid in this state machine.
  prv_transition(new_state);

  prv_log_battery_state(state);

  battery_charge_limit_evaluate(state);

  s_first_run = false;
}

void battery_monitor_init(void) {
  s_standby_timer_id = new_timer_create();
  s_power_state = PowerStateGood;
  s_low_on_first_run = false;
  s_first_run = true;

  // Initialize driver interface
  battery_state_init();
}

bool battery_monitor_critical_lockout(void) {
  // critical or low on first run
  return s_power_state == PowerStateCritical;
}

TimerID battery_monitor_get_standby_timer_id(void) {
  return s_standby_timer_id;
}
