/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/graphics/gtypes.h"
#include "pbl/services/battery/battery_monitor.h"

typedef enum BatteryUIWarningLevel {
  BatteryUIWarningLevel_None = -1,
  BatteryUIWarningLevel_Low,
  BatteryUIWarningLevel_VeryLow
} BatteryUIWarningLevel;

//! Process the incoming battery state change notification
void battery_ui_handle_state_change_event(PreciseBatteryChargeState new_state);

//! Handle shutting down the watch.
//!
//! If the watch is plugged in at the time, a "shut down while charging" UI is
//! displayed to give the user feedback on the charge state. Standby will be
//! entered once the watch is unplugged.
void battery_ui_handle_shut_down(void);

//! Shut down the watch using MCU hibernate instead of PMIC ship mode.
//!
//! This preserves the RTC and any backup state; the device wakes on button
//! press and cold-boots normally.
#ifdef CONFIG_HIBERNATE
void battery_ui_handle_mcu_shutdown(void);
#endif

//! Show the 'battery charging' modal dialog
void battery_ui_display_plugged(void);

//! Show the 'battery charged' modal dialog
void battery_ui_display_fully_charged(void);

//! Show the 'battery critical' modal dialog
void battery_ui_display_warning(uint32_t percent, BatteryUIWarningLevel warning_level);

//! Dismiss the battery UI modal window.
void battery_ui_dismiss_modal(void);
