/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once
#include <inttypes.h>
#include <stdbool.h>

/* TODO ***************************************************************
 * Some of the comments in this file may not be accurate for snowy.
 * Some thought should go into making sure that the function names and
 * comments are relevant to both snowy and previous generations
 * ********************************************************************/

//! Battery charge status.
typedef enum {
  //! Unknown charge status.
  BatteryChargeStatusUnknown,
  //! Charging is complete, battery full.
  BatteryChargeStatusComplete,
  //! Battery is in trickle charge mode.
  BatteryChargeStatusTrickle,
  //! Battery is in constant current charge mode.
  BatteryChargeStatusCC,
  //! Battery is in constant voltage charge mode.
  BatteryChargeStatusCV,
} BatteryChargeStatus;

//! Battery constants.
typedef struct BatteryConstants {
  //!< Battery voltage in millivolts.
  int32_t v_mv;
  //!< Battery current in microamperes.
  int32_t i_ua;
  //!< Battery temperature in millidegrees Celsius.
  int32_t t_mc;
} BatteryConstants;

void battery_init(void);

/** @returns the battery voltage after smoothing and averaging
 */
int battery_get_millivolts(void);

/**
 * Obtain battery constants.
 *
 * @param[out] constants
 *
 * @retval 0 On success.
 * @retval error Error code on failure.
 */
int battery_get_constants(BatteryConstants *constants);

/** @returns true if the battery charge controller thinks we are charging.
 * This is often INCORRECT on Pebble Steel due to the additional current
 * draw from the LED when charging, and as a result, this is not
 * the definition of "charging" we use for most places in the
 * code (i.e. battery_get_charge_state().is_charging), which depends on
 * SoC percentage. If you are not the battery_monitor state machine,
 * you probably don't want to use this. See PBL-2538 for context.
 */
bool battery_charge_controller_thinks_we_are_charging(void);

/** @returns true if both:
 * - the USB voltage is higher than 3.15V
 * - the USB voltage is higher than the battery voltage
 */
bool battery_is_usb_connected(void);
void battery_set_charge_enable(bool charging_enabled);
void battery_set_fast_charge(bool fast_charge_enabled);

// These are used by battery_common to allow forcing of charge states
bool battery_is_usb_connected_impl(void);
bool battery_charge_controller_thinks_we_are_charging_impl(void);
void battery_force_charge_enable(bool is_charging);
bool battery_is_charge_force_disabled(void);

//! The current voltage numbers from the battery. These structs are created by
//! the battery_read_voltage_monitor struct. Each _total value is a sum of 40 samples where
//! each sample is a number between 0 and 4095 representing a value between 0 and 1.8V. See
//! the comments inside battery_convert_reading_to_millivolts to see how to convert this to a
//! useful value.
typedef struct ADCVoltageMonitorReading {
  uint32_t vref_total;
  uint32_t vmon_total;
} ADCVoltageMonitorReading;

//! Read voltage numbers through an ADC on the voltage monitor pin. This is usually hooked up
//! to the battery voltage, but can be also used to read voltages on other rails by configuring
//! the PMIC to different values.
ADCVoltageMonitorReading battery_read_voltage_monitor(void);

//! Convert a ADCVoltageMonitorReading into a single mV reading using a given dividing ratio.
//! @param reading The voltage monitor reading to convert.
//! @param numerator The numerator to multiply the result by.
//! @param denominator The denominator to divide the result by.
uint32_t battery_convert_reading_to_millivolts(ADCVoltageMonitorReading reading,
                                               uint32_t numerator, uint32_t denominator);

//! Get the current battery charge status.
//!
//! @param[out] status The current charge status.
//!
//! @retval 0 On success.
//! @retval error Error code on failure.
int battery_charge_status_get(BatteryChargeStatus *status);
