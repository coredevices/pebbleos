/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "services/common/battery/battery_curve.h"

#include "board/board.h"
#include "system/logging.h"
#include "util/math.h"
#include "util/ratio.h"
#include "util/size.h"

typedef struct VoltagePoint {
  uint8_t percent;
  uint16_t voltage;
} VoltagePoint;

// TODO: Move these curves somewhere else. Related: PBL-21049

#ifdef PLATFORM_TINTIN
// When the voltage drops below these (mV), the watch will start heading for standby (after delay)
#define BATTERY_CRITICAL_VOLTAGE_CHARGING 3200
#define BATTERY_CRITICAL_VOLTAGE_DISCHARGING 3100
// Battery Tables for non-Snowy
static VoltagePoint discharge_curve[] = {
  {0,   BATTERY_CRITICAL_VOLTAGE_DISCHARGING},
  {2,   3410},
  {5,   3600},
  {10,  3670},
  {20,  3710},
  {30,  3745},
  {40,  3775},
  {50,  3810},
  {60,  3860},
  {70,  3925},
  {80,  4000},
  {90,  4080},
  {100, 4120},
};

static const VoltagePoint charge_curve[] = {
  {0,   BATTERY_CRITICAL_VOLTAGE_CHARGING},
  {5,   3725},
  {10,  3750},
  {20,  3790},
  {30,  3830},
  {40,  3845},
  {50,  3870},
  {60,  3905},
  {70,  3970},
  {80,  4025},
  {90,  4090},
  {100, 4130},
};
#elif BOARD_SNOWY_S3 || PLATFORM_ROBERT
// When the voltage drops below these (mV), the watch will start heading for standby (after delay)
#define BATTERY_CRITICAL_VOLTAGE_CHARGING 3700
#define BATTERY_CRITICAL_VOLTAGE_DISCHARGING 3300
// Battery Tables for Bobby Smiles
static VoltagePoint discharge_curve[] = {
  {0,   BATTERY_CRITICAL_VOLTAGE_DISCHARGING},
  {2,   3465},
  {5,   3615},
  {10,  3685},
  {20,  3725},
  {30,  3760},
  {40,  3795},
  {50,  3830},
  {60,  3885},
  {70,  3955},
  {80,  4065},
  {90,  4160},
  {100, 4250},
};

static const VoltagePoint charge_curve[] = {
  {0,   BATTERY_CRITICAL_VOLTAGE_CHARGING},
  {2,   3850},
  {5,   3935},
  {10,  4000},
  {20,  4040},
  {30,  4090},
  {40,  4145},
  {50,  4175},
  {60,  4225},
  {70,  4250},
};
#elif PLATFORM_SNOWY || PLATFORM_CALCULUS
// When the voltage drops below these (mV), the watch will start heading for standby (after delay)
#define BATTERY_CRITICAL_VOLTAGE_CHARGING 3500
#define BATTERY_CRITICAL_VOLTAGE_DISCHARGING 3300
// Battery Tables for Snowy
static VoltagePoint discharge_curve[] = {
  {0,   BATTERY_CRITICAL_VOLTAGE_DISCHARGING},
  {2,   3500},
  {5,   3600},
  {10,  3640},
  {20,  3690},
  {30,  3730},
  {40,  3750},
  {50,  3790},
  {60,  3840},
  {70,  3910},
  {80,  4000},
  {90,  4120},
  {100, 4250},
};

static const VoltagePoint charge_curve[] = {
  {0,   BATTERY_CRITICAL_VOLTAGE_CHARGING},
  {10,  3970},
  {20,  4020},
  {30,  4060},
  {40,  4090},
  {50,  4130},
  {60,  4190},
  {70,  4250},
};
#elif PLATFORM_SPALDING
// When the voltage drops below these (mV), the watch will start heading for standby (after delay)
#define BATTERY_CRITICAL_VOLTAGE_CHARGING 3700
#define BATTERY_CRITICAL_VOLTAGE_DISCHARGING 3300
// Battery Tables for Spalding
static VoltagePoint discharge_curve[] = {
  {0,   BATTERY_CRITICAL_VOLTAGE_DISCHARGING},
  {2,   3470},
  {5,   3600},
  {10,  3680},
  {20,  3720},
  {30,  3760},
  {40,  3790},
  {50,  3830},
  {60,  3875},
  {70,  3950},
  {80,  4050},
  {90,  4130},
  {100, 4250},
};

static const VoltagePoint charge_curve[] = {
  {0,   BATTERY_CRITICAL_VOLTAGE_CHARGING},
  {10,  3950},
  {20,  3990},
  {30,  4030},
  {40,  4090},
  {50,  4180},
  {60,  4230},
  {70,  4250},
};

// TODO: Needs customization for Asterix/Obelix
#elif PLATFORM_SILK || PLATFORM_ASTERIX || PLATFORM_OBELIX
// When the voltage drops below these (mV), the watch will start heading for standby (after delay)
#define BATTERY_CRITICAL_VOLTAGE_CHARGING 3550
#define BATTERY_CRITICAL_VOLTAGE_DISCHARGING 3300
// Battery Tables for Silk
static VoltagePoint discharge_curve[] = {
  {0,   BATTERY_CRITICAL_VOLTAGE_DISCHARGING},
  {2,   3490},
  {5,   3615},
  {10,  3655},
  {20,  3700},
  {30,  3735},
  {40,  3760},
  {50,  3800},
  {60,  3855},
  {70,  3935},
  {80,  4025},
  {90,  4120},
  {100, 4230}
};

static const VoltagePoint charge_curve[] = {
  {0,   BATTERY_CRITICAL_VOLTAGE_CHARGING},
  {2,   3570},
  {5,   3600},
  {10,  3645},
  {20,  3730},
  {30,  3800},
  {40,  3860},
  {50,  3915},
  {60,  3970},
  {70,  4030},
  {80,  4095},
  {90,  4175},
  {100, 4260}
};

#else
#error "No battery curve for platform!"
#endif

static const uint8_t NUM_DISCHARGE_POINTS = ARRAY_LENGTH(discharge_curve);
static const uint8_t NUM_CHARGE_POINTS = ARRAY_LENGTH(charge_curve);

static int s_battery_compensation_values[BATTERY_CURVE_COMPENSATE_COUNT];

// Shifts the 100% reference on the discharge curve, as long as it
// doesn't drop below the next highest point.
void battery_curve_set_full_voltage(uint16_t voltage) {
  voltage = MAX(voltage, discharge_curve[NUM_DISCHARGE_POINTS-2].voltage + 1);
  discharge_curve[NUM_DISCHARGE_POINTS-1].voltage = voltage;
}

static uint32_t prv_lookup_percent_by_voltage(
    int battery_mv, bool is_charging, uint32_t scaling_factor) {
  VoltagePoint const * const battery_curve = (is_charging) ? charge_curve : discharge_curve;
  uint8_t const num_curve_points = (is_charging) ? NUM_CHARGE_POINTS : NUM_DISCHARGE_POINTS;

  // Constrain the voltage between the min and max points of the curve
  if (battery_mv <= battery_curve[0].voltage) {
    return battery_curve[0].percent * scaling_factor;
  } else if (battery_mv >= battery_curve[num_curve_points-1].voltage) {
    return battery_curve[num_curve_points-1].percent * scaling_factor;
  }

  // search through the curves for the next charge level...
  uint32_t charge_index;
  for (charge_index = num_curve_points-2;
    (charge_index > 0) && (battery_mv < battery_curve[charge_index].voltage);
    --charge_index) {
  }

  // linearly interpolate between battery_curve[charge_index] and battery_curve[charge_index+1]
  uint32_t delta_mv = (battery_mv - battery_curve[charge_index].voltage);
  uint32_t delta_next_mv = (battery_curve[charge_index + 1].voltage -
      battery_curve[charge_index].voltage);
  uint32_t delta_next_percent = (battery_curve[charge_index + 1].percent -
      battery_curve[charge_index].percent);

  uint32_t start_percent = battery_curve[charge_index].percent * scaling_factor;
  delta_next_percent *= scaling_factor;

  uint32_t charge_percent =
      (((uint64_t)delta_next_percent * delta_mv) / delta_next_mv) + start_percent;
  return charge_percent;
}

uint32_t battery_curve_lookup_percent_by_voltage(uint32_t battery_mv, bool is_charging) {
  return prv_lookup_percent_by_voltage(battery_mv, is_charging, 1);
}

static uint32_t prv_sample_scaled_charge_percent(
    uint32_t battery_mv, bool is_charging, uint32_t scaling_factor) {
  int compensate = 0;  // We compensate 5% during rounding, so don't do here

  for (int i = 0; i < BATTERY_CURVE_COMPENSATE_COUNT; ++i) {
    compensate += s_battery_compensation_values[i];
  }

  return prv_lookup_percent_by_voltage(battery_mv + compensate, is_charging, scaling_factor);
}

uint32_t battery_curve_sample_ratio32_charge_percent(uint32_t battery_mv, bool is_charging) {
  const uint32_t scaling_factor = ratio32_from_percent(100) / 100 + 1;
  return prv_sample_scaled_charge_percent(battery_mv, is_charging, scaling_factor);
}

void battery_curve_set_compensation(BatteryCurveVoltageCompensationKey key, int mv) {
  s_battery_compensation_values[key] = mv;
}

// This is used by unit tests and QEMU
uint32_t battery_curve_lookup_voltage_by_percent(uint32_t percent, bool is_charging) {
  VoltagePoint const * const battery_curve = (is_charging) ? charge_curve : discharge_curve;
  uint32_t const num_curve_points = (is_charging) ? NUM_CHARGE_POINTS : NUM_DISCHARGE_POINTS;

  // Clip if above curve upper bound
  if (percent > battery_curve[num_curve_points-1].percent) {
    return battery_curve[num_curve_points-1].voltage;
  }

  // search through the curves for the next charge level...
  uint32_t charge_index;
  for (charge_index = num_curve_points-2;
    (charge_index > 0) && (percent < battery_curve[charge_index].percent);
    --charge_index) {
  }

  // linearly interpolate between battery_curve[charge_index] and battery_curve[charge_index+1]
  uint32_t delta_next_mv = (battery_curve[charge_index+1].voltage -
                            battery_curve[charge_index].voltage);
  uint32_t delta_next_percent = (battery_curve[charge_index+1].percent -
                                 battery_curve[charge_index].percent);
  uint32_t battery_mv = battery_curve[charge_index].voltage +
                        ((percent - battery_curve[charge_index].percent) * delta_next_mv) /
                        delta_next_percent;
  return battery_mv;
}

//! This call is used internally with PreciseBatteryChargeState
//! So must remove low_power_threshold to get correct remaining hours
//! before low_power_mode is triggered
uint32_t battery_curve_get_hours_remaining(uint32_t percent_remaining) {
  if (percent_remaining <= BOARD_CONFIG_POWER.low_power_threshold) {
    return 0;
  }
  percent_remaining -= BOARD_CONFIG_POWER.low_power_threshold;
  return ((BOARD_CONFIG_POWER.battery_capacity_hours * percent_remaining) / 100);
}

//! This call is used internally with PreciseBatteryChargeState
//! So must add low_power_threshold to get percentage in terms of
//! PreciseBatteryChargeState (which includes low_power_mode)
uint32_t battery_curve_get_percent_remaining(uint32_t hours) {
  return ((hours * 100) / BOARD_CONFIG_POWER.battery_capacity_hours) +
    BOARD_CONFIG_POWER.low_power_threshold;
}

// for unit tests and analytics
int32_t battery_curve_lookup_percent_with_scaling_factor(
    int battery_mv, bool is_charging, uint32_t scaling_factor) {
  return (prv_lookup_percent_by_voltage(battery_mv, is_charging, scaling_factor));
}
