/* SPDX-FileCopyrightText: 2026 Shashvat Prabhu */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/battery/battery_state.h"

//! Optionally pauses charging once the battery reaches the configured percentage limit
//! to reduce degradation from sustained high charge levels.

#define CHARGE_LIMIT_PCT_DISABLED 0
#define CHARGE_LIMIT_PCT_MIN 50
#define CHARGE_LIMIT_PCT_MAX 95

void battery_charge_limit_evaluate(PreciseBatteryChargeState state);

void battery_charge_limit_handle_pref_change(void);
