/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/weather/weather_service.h"
#include "pbl/services/weather/weather_types.h"
#include "system/status_codes.h"
#include "pbl/util/attributes.h"
#include "util/pstring.h"
#include "util/time/time.h"
#include "pbl/util/uuid.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// Weather BlobDB schema version
//
// v3 = legacy schema (current/today/tomorrow only). See WeatherDBEntryV3.
// v4 = rich schema for the full Weather app. Appends today's extended metrics
//      (feels-like, UV, precip probability, wind), the location's coordinates
//      (for the globe), a multi-day daily forecast array, and today's hourly
//      type+temp series. All v4 fixed fields are appended AFTER the v3 prefix
//      (whose offsets are preserved byte-for-byte) and BEFORE the trailing
//      variable-length pstring16s, so a v3-era reader still finds the v3 fields
//      and the version byte is the single source of compatibility truth.
//
// The phone only writes v4 records when the firmware advertises
// `weather_db_v4_support` (see session_remote_version.h / system_versions.c);
// otherwise it keeps writing v3. The firmware parses BOTH during rollout.
// ---------------------------------------------------------------------------
// Minor history: 0 = base v4. 1 = appends location_utc_offset_min + daily_metrics[]
// after the hourly arrays (before the trailing pstrings). 2 = appends today's raw
// warning readings (WMO code, humidity, min visibility, precipitation sum) after
// daily_metrics — they feed the weather report's warning line. 3 = appends dominant
// wind direction (today + per-day). 4 = appends today's hourly UV. 5 = appends
// TOMORROW's hourly type/temp series (the clock dial's post-midnight positions).
// Older-minor records remain fully parseable — readers gate the appended fields on
// minor_version + record length, and the trailing-strings offset is resolved per
// minor (see weather_db_entry_get_strings). Unknown FUTURE minors are rejected at
// insert (their strings offset is unknowable), so the phone must gate each new
// minor on firmware support.
#define WEATHER_DB_CURRENT_VERSION (4)
#define WEATHER_DB_CURRENT_MINOR_VERSION (5)
#define WEATHER_DB_LEGACY_VERSION (3)

// Days of daily forecast a v4 record carries (today + 6).
#define WEATHER_DB_MAX_FORECAST_DAYS (7)
// Hours of hourly data a v4 record carries (today only — keeps the record small).
#define WEATHER_DB_HOURLY_COUNT (24)

typedef Uuid WeatherDBKey;

// ---------------------------------------------------------------------------
// Legacy v3 record. Kept verbatim so the firmware can still read records
// written by an older mobile app during the rollout window. Do not change.
// ---------------------------------------------------------------------------
typedef struct PACKED {
  uint8_t version;
  int16_t current_temp;
  WeatherType current_weather_type;
  int16_t today_high_temp;
  int16_t today_low_temp;
  WeatherType tomorrow_weather_type;
  int16_t tomorrow_high_temp;
  int16_t tomorrow_low_temp;
  time_t last_update_time_utc;
  bool is_current_location;
  SerializedArray pstring16s;
} WeatherDBEntryV3;

// ---------------------------------------------------------------------------
// One day of daily forecast (v4+). Weather type stored as uint8_t (values map
// 1:1 to WeatherType; WeatherType_Unknown == 255). Cast on read.
// ---------------------------------------------------------------------------
typedef struct PACKED {
  int16_t high_temp;       // WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP if unknown
  int16_t low_temp;        // WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP if unknown
  uint8_t weather_type;    // WeatherType; 255 if unknown
} WeatherDBDailyForecast;

// ---------------------------------------------------------------------------
// Per-day extended metrics (v4 minor 1+), parallel to daily[] (index 0 = today).
// The watch shows per-day precipitation on the scrolled forecast and per-day
// precip/wind/UV as the user pages through days — without these, future days
// render "--". 255 = unknown for every field.
// ---------------------------------------------------------------------------
typedef struct PACKED {
  uint8_t precip_probability;  // 0..100 (%), 255 if unknown
  uint8_t wind_speed;          // whole units, same unit as today_wind_speed; 255 if unknown
  uint8_t uv_index_x10;        // UV index * 10 (0..110), 255 if unknown
} WeatherDBDailyMetrics;

// ---------------------------------------------------------------------------
// v4 record. Layout: [ v3 fixed prefix, unchanged offsets ] + [ v4 fixed
// fields ] + [ trailing pstring16s ]. The trailing pstring array MUST be last.
// ---------------------------------------------------------------------------
typedef struct PACKED {
  // --- v3-compatible fixed prefix (identical offsets to WeatherDBEntryV3) ---
  uint8_t version;                 // == WEATHER_DB_CURRENT_VERSION (4)
  int16_t current_temp;
  WeatherType current_weather_type;
  int16_t today_high_temp;
  int16_t today_low_temp;
  WeatherType tomorrow_weather_type;
  int16_t tomorrow_high_temp;
  int16_t tomorrow_low_temp;
  time_t last_update_time_utc;
  bool is_current_location;

  // --- v4 additions (fixed-size, appended) ---
  uint8_t minor_version;            // == WEATHER_DB_CURRENT_MINOR_VERSION
  int16_t today_feels_like_temp;    // WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP if unknown
  int16_t today_uv_index_x10;       // UV index * 10 (0..110), or -1 if unknown
  int16_t today_precip_probability; // 0..100 (%), or -1 if unknown
  uint16_t today_wind_speed;        // whole units (km/h or mph per phone), 0 if unknown
  uint16_t today_wind_direction;    // degrees 0..359, 0xFFFF if unknown
  int16_t latitude_e2;              // latitude * 100 (for the globe), INT16_MIN if unknown
  int16_t longitude_e2;             // longitude * 100 (for the globe), INT16_MIN if unknown
  uint8_t num_daily;                // valid entries in daily[] (0..WEATHER_DB_MAX_FORECAST_DAYS)
  WeatherDBDailyForecast daily[WEATHER_DB_MAX_FORECAST_DAYS];
  uint8_t today_hourly_count;       // 0 or WEATHER_DB_HOURLY_COUNT
  uint8_t today_hourly_weather_type[WEATHER_DB_HOURLY_COUNT]; // WeatherType per hour 0-23
  int8_t today_hourly_temp[WEATHER_DB_HOURLY_COUNT];          // temp per hour 0-23

  // --- v4 minor 1 additions (appended; present only when minor_version >= 1) ---
  int16_t location_utc_offset_min;  // location's timezone, minutes EAST of UTC (e.g. Tokyo
                                    // +540, New York DST -240); INT16_MIN if unknown. Lets the
                                    // watch show the LOCATION's local sunset/hourly times for
                                    // saved cities instead of watch-local ones.
  WeatherDBDailyMetrics daily_metrics[WEATHER_DB_MAX_FORECAST_DAYS]; // parallel to daily[]

  // --- v4 minor 2 additions (appended; present only when minor_version >= 2) ---
  // Today's raw warning readings for the weather report's alert line. Open-Meteo
  // sources: daily weather_code; hourly
  // relative_humidity_2m (daily mean); hourly visibility (daily MINIMUM, meters);
  // daily precipitation_sum (whole mm).
  uint8_t today_wmo_code;         // WMO weather code; 0xFF if unknown
  uint8_t today_humidity_pct;     // relative humidity 0..100 %; 0xFF if unknown
  uint16_t today_visibility_m;    // minimum visibility, meters (clamp 65534); 0xFFFF if unknown
  uint16_t today_precip_sum_mm;   // total precipitation, whole mm (clamp 65534); 0xFFFF if unknown
  // Per-day feels-like, parallel to daily[] (Open-Meteo daily apparent_temperature_max);
  // WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP if unknown.
  int16_t daily_feels_like[WEATHER_DB_MAX_FORECAST_DAYS];

  // --- v4.3 appended fields (dominant wind direction) ---
  // Open-Meteo: hourly winddirection_10m (today, dominant) and daily
  // winddirection_10m_dominant. Degrees 0..359; -1 if unknown.
  int16_t today_wind_dir_deg;
  int16_t daily_wind_dir_deg[WEATHER_DB_MAX_FORECAST_DAYS];

  // --- v4.4 appended fields (hourly UV) ---
  // Open-Meteo: hourly uv_index. UV * 10 per hour 0-23 (UV 6.5 -> 65), 255 if unknown.
  // Lets the watch show the CURRENT hour's UV; today_uv_index_x10 stays the day's figure.
  uint8_t today_hourly_uv_x10[WEATHER_DB_HOURLY_COUNT];

  // --- v4.5 appended fields (TOMORROW's hourly series) ---
  // Mirrors the today_hourly_* block for the next calendar day (location-local).
  // The clock dial shows the next 12 hours, so from early afternoon it crosses
  // midnight — these give its post-midnight positions real data. Open-Meteo:
  // hourly weather_code + temperature_2m for tomorrow's 24 slots.
  uint8_t tomorrow_hourly_count;    // 0 or WEATHER_DB_HOURLY_COUNT
  uint8_t tomorrow_hourly_weather_type[WEATHER_DB_HOURLY_COUNT]; // WeatherType, 255 unknown
  int8_t tomorrow_hourly_temp[WEATHER_DB_HOURLY_COUNT];          // temp per hour 0-23

  // --- variable-length trailing strings (MUST stay last) ---
  SerializedArray pstring16s;
} WeatherDBEntry;

typedef enum WeatherDbStringIndex {
  WeatherDbStringIndex_LocationName,
  WeatherDbStringIndex_ShortPhrase,
  WeatherDbStringIndexCount,
} WeatherDbStringIndex;

// Fixed portion of a v4.0 record (through the hourly arrays) — the minimum any
// v4 record must carry, and where a minor-0 record's trailing strings start.
#define WEATHER_DB_V4_0_FIXED_SIZE (offsetof(WeatherDBEntry, location_utc_offset_min))
// Fixed portion of a v4.1 record — where a minor-1 record's trailing strings start.
#define WEATHER_DB_V4_1_FIXED_SIZE (offsetof(WeatherDBEntry, today_wmo_code))
// Fixed portion of a v4.2 record — where a minor-2 record's trailing strings start.
#define WEATHER_DB_V4_2_FIXED_SIZE (offsetof(WeatherDBEntry, today_wind_dir_deg))
// Fixed portion of a v4.3 record — where a minor-3 record's trailing strings start.
#define WEATHER_DB_V4_3_FIXED_SIZE (offsetof(WeatherDBEntry, today_hourly_uv_x10))
// Fixed portion of a v4.4 record — where a minor-4 record's trailing strings start.
#define WEATHER_DB_V4_4_FIXED_SIZE (offsetof(WeatherDBEntry, tomorrow_hourly_count))
// Fixed portion of a current (v4.5) record, i.e. everything except the trailing
// pstring16s SerializedArray header/payload.
#define WEATHER_DB_V4_FIXED_SIZE (offsetof(WeatherDBEntry, pstring16s))

// Smallest acceptable record is a legacy v3 record (smaller fixed prefix).
#define MIN_ENTRY_SIZE (sizeof(WeatherDBEntryV3))
#define MAX_ENTRY_SIZE (sizeof(WeatherDBEntry) + \
                        WEATHER_SERVICE_MAX_WEATHER_LOCATION_BUFFER_SIZE + \
                        WEATHER_SERVICE_MAX_SHORT_PHRASE_BUFFER_SIZE)

//! @return true if the firmware can parse a record stamped with this major version.
static inline bool weather_db_version_is_supported(uint8_t version) {
  return (version == WEATHER_DB_CURRENT_VERSION) || (version == WEATHER_DB_LEGACY_VERSION);
}

//! @return true if the firmware can parse this record. A v4 record with a minor
//! NEWER than this firmware understands must be rejected: a newer minor appends
//! fixed fields, which moves the trailing strings, so parsing it with our offsets
//! would read the appended fields as string data.
static inline bool weather_db_entry_is_supported(const WeatherDBEntry *entry) {
  if (!weather_db_version_is_supported(entry->version)) {
    return false;
  }
  return (entry->version < WEATHER_DB_CURRENT_VERSION) ||
         (entry->minor_version <= WEATHER_DB_CURRENT_MINOR_VERSION);
}

//! @return the byte offset of the trailing pstring16s array for a record of the
//! given version + minor. v3, v4.0, v4.1, v4.2, v4.3 and v4.4 place it differently.
//! Each rung must name the offset of the FIRST field the NEXT minor appends — using
//! offsetof(pstring16s) for anything but the current minor silently demands the newest
//! record length from older records and rejects every one of them.
static inline size_t weather_db_entry_strings_offset(uint8_t version, uint8_t minor_version) {
  if (version < WEATHER_DB_CURRENT_VERSION) {
    return offsetof(WeatherDBEntryV3, pstring16s);
  }
  if (minor_version >= 5) return offsetof(WeatherDBEntry, pstring16s);
  if (minor_version >= 4) return WEATHER_DB_V4_4_FIXED_SIZE;
  if (minor_version >= 3) return WEATHER_DB_V4_3_FIXED_SIZE;
  if (minor_version >= 2) return WEATHER_DB_V4_2_FIXED_SIZE;
  if (minor_version >= 1) return WEATHER_DB_V4_1_FIXED_SIZE;
  return WEATHER_DB_V4_0_FIXED_SIZE;
}

//! @return a pointer to the trailing pstring16s array, located correctly for the
//! record's version + minor. Use this instead of &entry->pstring16s so v3 and
//! minor-0 records still resolve their strings after fields were appended.
static inline SerializedArray *weather_db_entry_get_strings(WeatherDBEntry *entry) {
  const uint8_t minor =
      (entry->version >= WEATHER_DB_CURRENT_VERSION) ? entry->minor_version : 0;
  return (SerializedArray *)((uint8_t *)entry +
                             weather_db_entry_strings_offset(entry->version, minor));
}

// Memory ownership: pointer to key and entry must not be saved, as they become invalid after
// the callback finishes
typedef void (*WeatherDBIteratorCallback)(WeatherDBKey *key, WeatherDBEntry *entry, void *context);

// ------------------------------------------------------------------------------------
// WeatherDB functions
status_t weather_db_for_each(WeatherDBIteratorCallback cb, void *context);

// ------------------------------------------------------------------------------------
// BlobDB Implementation

void weather_db_init(void);

status_t weather_db_flush(void);

status_t weather_db_compact(void);

status_t weather_db_insert(const uint8_t *key, int key_len, const uint8_t *val, int val_len);

int weather_db_get_len(const uint8_t *key, int key_len);

status_t weather_db_read(const uint8_t *key, int key_len, uint8_t *val_out, int val_out_len);

status_t weather_db_delete(const uint8_t *key, int key_len);
