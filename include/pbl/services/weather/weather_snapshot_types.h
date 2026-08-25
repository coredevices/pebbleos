/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

//! App-facing weather snapshot types.
//!
//! Kept deliberately separate from weather_service.h / weather_types.h:
//! weather_types.h transitively includes a per-platform auto-generated
//! resource header (for TimelineResourceId) that isn't resolvable by the SDK
//! export tool's limited parse environment, so nothing in this header's
//! include chain may depend on it.

#include "util/time/time.h"

#include <stdbool.h>
#include <stdint.h>

#define WEATHER_SNAPSHOT_MAX_PHRASE_LEN (32)

//! Weather icon category. Numerically mirrors WeatherType (see
//! src/fw/services/weather/weather_type_tuples.def, the source of truth for
//! these IDs) without depending on its header - if weather_type_tuples.def
//! ever changes, update this to match.
typedef enum {
  WeatherSnapshotIcon_PartlyCloudy = 0,
  WeatherSnapshotIcon_CloudyDay = 1,
  WeatherSnapshotIcon_LightSnow = 2,
  WeatherSnapshotIcon_LightRain = 3,
  WeatherSnapshotIcon_HeavyRain = 4,
  WeatherSnapshotIcon_HeavySnow = 5,
  WeatherSnapshotIcon_Generic = 6,
  WeatherSnapshotIcon_Sun = 7,
  WeatherSnapshotIcon_RainAndSnow = 8,
  WeatherSnapshotIcon_Unknown = 255,
} WeatherSnapshotIcon;

//! A pointer-free snapshot of the default location's forecast, safe to hand
//! directly to apps: unlike WeatherLocationForecast, none of its fields are
//! heap pointers only valid in kernel context.
typedef struct WeatherServiceSnapshot {
  bool has_data;
  bool is_current_location;
  int current_temp;
  int today_high;
  int today_low;
  WeatherSnapshotIcon current_weather_type;
  char current_weather_phrase[WEATHER_SNAPSHOT_MAX_PHRASE_LEN];
  int tomorrow_high;
  int tomorrow_low;
  WeatherSnapshotIcon tomorrow_weather_type;
  time_t time_updated_utc;
} WeatherServiceSnapshot;
