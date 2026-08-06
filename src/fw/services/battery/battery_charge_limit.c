/* SPDX-FileCopyrightText: 2026 Shashvat Prabhu */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/battery/battery_charge_limit.h"

#include <pbl/drivers/battery.h>
#include "pbl/util/attributes.h"
#include "shell/prefs.h"
#include <pbl/logging/logging.h>

PBL_LOG_MODULE_DECLARE(service_battery, CONFIG_SERVICE_BATTERY_LOG_LEVEL);

T_STATIC bool s_limit_active;

static void prv_set_limit_active(bool active, int pct) {
  if (active == s_limit_active) {
    return;
  }
  battery_set_charge_enable(!active);
  s_limit_active = active;
  if (active) {
    PBL_LOG_DBG("Charge limit: pausing charging at %d%%", pct);
  } else {
    PBL_LOG_DBG("Charge limit: resuming charging at %d%%", pct);
  }
}

void battery_charge_limit_evaluate(PreciseBatteryChargeState state) {
  // The debug prompt owns the charger while it is force-disabled.
  if (battery_is_charge_force_disabled()) {
    s_limit_active = false;
    return;
  }

  uint8_t limit_pct = shell_prefs_get_charge_limit_pct();
  bool limit_reached = (limit_pct != CHARGE_LIMIT_PCT_DISABLED) && state.is_plugged &&
                       (state.pct >= limit_pct);
  prv_set_limit_active(limit_reached, state.pct);
}

void battery_charge_limit_handle_pref_change(void) {
  battery_state_force_update();
}
