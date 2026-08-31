/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kernel/low_power.h"

#include "apps/prf/low_power.h"
#include "kernel/event_loop.h"
#include "kernel/ui/modals/modal_manager.h"
#include "kernel/util/standby.h"
#include "mfg/mfg_mode/mfg_factory_mode.h"
#include "process_management/app_manager.h"
#include "process_management/worker_manager.h"
#include "pbl/services/analytics/analytics.h"
#include "pbl/services/runlevel.h"

#include <stdbool.h>

static bool s_low_power_active, s_prev_low_power_active;
static TimerID s_toggle_timer = TIMER_INVALID_ID;

static void prv_low_power_launcher_task_callback(void *unused) {
  if (s_low_power_active == s_prev_low_power_active) {
    // We settled into the same state as before toggling.
    return;
  }

  if (s_low_power_active) {
    PBL_ANALYTICS_TIMER_START(low_power_time_ms);
    worker_manager_disable();
    services_set_runlevel(RunLevel_LowPower);
  } else {
    PBL_ANALYTICS_TIMER_STOP(low_power_time_ms);
    worker_manager_enable();
    services_set_runlevel(RunLevel_Normal);
  }

  s_prev_low_power_active = s_low_power_active;
}

static void prv_low_power_toggle_timer_callback(void* data) {
  launcher_task_add_callback(prv_low_power_launcher_task_callback, data);
}

static void prv_low_power_transition(bool active) {
  s_low_power_active = active;
  if (s_toggle_timer == TIMER_INVALID_ID) {
    s_toggle_timer = new_timer_create();
  }
  // Rapid charger connection changes (aligning magnetic connector for example)
  // will cause repeated low power on/off requests. Require that a few seconds
  // elapse without further transitions before acting upon it to give us some
  // time to settle on one state or the other.
  new_timer_start(s_toggle_timer, 3000, prv_low_power_toggle_timer_callback,
                  NULL, 0 /*flags*/);

  // FIXME PBL-XXXXX: This should be in a shell/prf/battery_ui_fsm.c
#ifdef CONFIG_RECOVERY_FW
  if (active) {
    app_manager_launch_new_app(&(AppLaunchConfig) {
      .md = prf_low_power_app_get_info(),
    });
  } else {
    // In MFG mode, disable low power mode above but don't close the app.
    if (mfg_is_mfg_mode()) {
      return;
    }
    app_manager_close_current_app(true);
  }
#endif
}

void low_power_standby(void) {
  PBL_ANALYTICS_TIMER_STOP(low_power_time_ms);
  enter_standby(RebootReasonCode_LowBattery);
}

void low_power_enter(void) {
#ifdef CONFIG_RECOVERY_FW
  if (mfg_is_mfg_mode()) {
    return;
  }
#endif
  prv_low_power_transition(true);
}

void low_power_exit(void) {
  prv_low_power_transition(false);
}

bool low_power_is_active(void) {
  return s_low_power_active;
}
