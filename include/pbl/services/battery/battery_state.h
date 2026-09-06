/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once
#include "pbl/services/new_timer/new_timer.h"
#include <stdbool.h>
#include <stdint.h>

// Handles all battery-related driver communication, filters out events

//! @addtogroup Foundation
//! @{
//!   @addtogroup Battery Battery
//!   \brief Functions related to getting the battery status
//!
//! This module contains the functions necessary to find the current charge status.
//! @note Battery charge state is a complex topic; our modelling of the charge
//! that is exposed by these functions represents a very simplified model based
//! mostly on empirically derived charge and discharge voltage curves.  As
//! such, you should expect that the output will not have a high degree of
//! accuracy.
//!   @{

//! Structure for retrieval of the battery charge state
typedef struct {
  //! A percentage (0-100) of how full the battery is
  uint8_t charge_percent;
  //! True if the battery is currently being charged. False if not.
  bool is_charging;
  //! True if the charger cable is connected. False if not.
  bool is_plugged;
} BatteryChargeState;

//! @internal
//! Structure for retrieval of the exact battery charge state
typedef struct {
  //! The battery's percentage as a ratio32
  uint32_t charge_percent;
  //! The battery percentage 0-100
  uint8_t pct;
  //! WARNING: This maps to @see battery_charge_controller_thinks_we_are_charging as opposed to
  //! the user-facing definition of whether we're charging (100% battery).
  bool is_charging;
  bool is_plugged;
} PreciseBatteryChargeState;

//! Function to get the current battery charge state
//! @returns a \ref BatteryChargeState struct with the current charge state
BatteryChargeState battery_get_charge_state(void);

//!   @}
//! @}

void battery_state_force_update(void);

void battery_state_init(void);

void battery_state_handle_connection_event(bool is_connected);

void battery_state_reset_filter(void);

// Get the last recorded voltage
uint16_t battery_state_get_voltage(void);

// Get the last recorded temperature (mC)
int32_t battery_state_get_temperature(void);

// For unit tests
TimerID battery_state_get_periodic_timer_id(void);
