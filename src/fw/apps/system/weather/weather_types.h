/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pebble_compat.h"
#include "weather_platform.h"

#define WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP (32767)

// Message keys — must match messageKeys in package.json
#define MESSAGE_KEY_LOCATION_NAME      0
#define MESSAGE_KEY_CURRENT_TEMP       1
#define MESSAGE_KEY_DAY_INDEX          2
#define MESSAGE_KEY_DAY_HIGH           3
#define MESSAGE_KEY_DAY_LOW            4
#define MESSAGE_KEY_DAY_WEATHER_TYPE   5
#define MESSAGE_KEY_DAY_WEATHER_PHRASE 6
#define MESSAGE_KEY_DAY_LABEL          7
// Hourly weather types: byte array of 24 values, one per hour 0-23.
#define MESSAGE_KEY_HOURLY_WEATHER     8
// Watch → JS: request an immediate data refresh.
#define MESSAGE_KEY_REFRESH_REQUEST    9
// Today's UV index (0-11), rain chance (%), and max wind speed (mph).
#define MESSAGE_KEY_DAY_UV             10
#define MESSAGE_KEY_DAY_PRECIP         11
// Hourly temperatures: byte array of 24 signed values (int8), one per hour 0-23.
#define MESSAGE_KEY_HOURLY_TEMP        12
// Watch â†’ JS: select a preset city by index. JS â†’ Watch: request id echoed
// in every response message so stale weather responses can be ignored.
#define MESSAGE_KEY_CITY_SELECT_INDEX  13
#define MESSAGE_KEY_WEATHER_REQUEST_ID 14
#define MESSAGE_KEY_CURRENT_LATITUDE   15
#define MESSAGE_KEY_CURRENT_LONGITUDE  16
// (key 17 retired with the watch-side dictation add flow — locations are phone-owned)
#define MESSAGE_KEY_CURRENT_LOCATION_REQUEST 18
#define MESSAGE_KEY_DAY_WIND           19

typedef enum {
  WeatherType_PartlyCloudy = 0,
  WeatherType_CloudyDay    = 1,
  WeatherType_LightSnow    = 2,
  WeatherType_LightRain    = 3,
  WeatherType_HeavyRain    = 4,
  WeatherType_HeavySnow    = 5,
  WeatherType_Generic      = 6,
  WeatherType_Sun          = 7,
  WeatherType_RainAndSnow  = 8,
  WeatherType_Unknown      = 255,
} WeatherType;

typedef struct WeatherLocationForecast {
  char *location_name;
  bool is_current_location;
  int current_temp;
  int today_high;
  int today_low;
  int today_uv;        // UV index 0-11, or -1 if unknown (the DAY's figure — Open-Meteo's
                       // daily uv_index_max, i.e. the peak; NOT a live reading)
  int today_uv_now;    // UV index for the CURRENT hour, -1 unknown. Needs the v4 minor-4
                       // hourly UV block; falls back to today_uv until the phone sends it.
  int today_precip_mm; // rain chance percent, or -1 if unknown
  int today_wind_mph;
  int today_wind_dir_deg;   // dominant direction 0..359, -1 unknown (v4.3)  // max wind speed in mph, or -1 if unknown
  int today_feels;     // feels-like temp, UNKNOWN_TEMP if unknown (today only)
  int today_wmo;       // WMO weather code, -1 if unknown (v4.2, today only)
  int today_humidity;  // relative humidity %, -1 if unknown (v4.2, today only)
  int today_visibility_m;   // minimum visibility in meters, -1 if unknown (v4.2, today only)
  int today_precip_sum_mm;  // total precipitation in mm, -1 if unknown (v4.2, today only)
  WeatherType current_weather_type;
  char *current_weather_phrase;
  // CURRENT-HOUR conditions (the mainscreen header): derived from the
  // hourly arrays the clock screen already syncs, indexed at the LOCATION's local hour, so
  // the header tracks the day instead of freezing at sync time. Fall back to the synced
  // current_* fields when the record carries no hourly block. current_phrase_now points at
  // either a static type-name ("Sunny") or the synced phrase buffer — never freed.
  WeatherType current_type_now;
  int current_temp_now;
  char *current_phrase_now;
  char *label;
  time_t time_updated_utc;
} WeatherLocationForecast;

GColor weather_type_bg_color(WeatherType weather_type);
//! The colour to fill this condition's disc with, or GColorClear where no
//! disc should draw. On the 1-bit boards this is ALWAYS GColorClear: the tone
//! cannot carry the condition there, so the icon carries it alone.
GColor weather_type_disc_color(WeatherType weather_type);
//! Flip the ink of a palettized icon in place (non-transparent palette
//! entries complement their RGB). Self-inverse.
void weather_icon_invert_ink(GBitmap *icon);
//! Draw a (palettized) icon bitmap; with invert_ink the ink flips for the
//! draw and is restored after — the cached bitmaps are shared between
//! rows/positions.
void weather_icon_draw(GContext *ctx, GBitmap *icon, GRect rect, bool invert_ink);
uint32_t weather_type_icon_tiny_resource(WeatherType weather_type);
uint32_t weather_type_icon_small_resource(WeatherType weather_type);
// 80x80 PDC (GDrawCommandImage) — the Timeline weather pin's big icon.
uint32_t weather_type_icon_large_resource(WeatherType weather_type);
#if defined(PBL_PLATFORM_GABBRO)
uint32_t weather_type_icon_clock_resource(WeatherType weather_type);
#endif
