/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */
//! Implementation of the weather data-source adapter. This translation unit is
//! the only one that includes the firmware weather headers.
#include "weather_data_source.h"

#include "pbl/services/weather/weather_service.h"
#include "pbl/services/weather/weather_service_private.h"  // SerializedWeatherAppPrefs
#include "pbl/services/weather/weather_types.h"
#include "pbl/services/blob_db/weather_db.h"
#include "pbl/services/blob_db/watch_app_prefs_db.h"
#include "kernel/pbl_malloc.h"  // task_zalloc_check / task_free
#include "pbl/drivers/rtc.h"        // rtc_get_time (QEMU synth "last updated")

#include <limits.h>
#include <string.h>

static void prv_copy_str(char *dst, size_t dst_size, const char *src) {
  if (!src) {
    dst[0] = '\0';
    return;
  }
  strncpy(dst, src, dst_size - 1);
  dst[dst_size - 1] = '\0';
}

static void prv_fill_from_fw(WxDsForecast *out, const WeatherLocationForecast *f, int index) {
  memset(out, 0, sizeof(*out));
  prv_copy_str(out->location_name, sizeof(out->location_name), f->location_name);
  prv_copy_str(out->short_phrase, sizeof(out->short_phrase), f->current_weather_phrase);
  out->is_current_location = (index == 0);
  out->current_temp = f->current_temp;
  out->today_high = f->today_high;
  out->today_low = f->today_low;
  out->current_weather_type = (uint8_t)f->current_weather_type;
  out->tomorrow_high = f->tomorrow_high;
  out->tomorrow_low = f->tomorrow_low;
  out->tomorrow_weather_type = (uint8_t)f->tomorrow_weather_type;
  // v4 metrics are not in weather_service's v3 forecast; prv_overlay_v4 reads
  // them from the v4 WeatherDBEntry directly (no-op for v3 records).
  out->today_uv = -1;
  out->has_hourly_uv = false;
  for (int i = 0; i < WX_DS_HOURLY; i++) out->hourly_uv[i] = -1;
  out->today_precip = -1;
  out->today_wind = -1;
  out->today_feels = WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP;
  out->today_wmo = -1;
  out->today_wind_dir = -1;
  out->today_humidity = -1;
  out->today_visibility_m = -1;
  out->today_precip_sum_mm = -1;
  out->latitude_e2 = INT16_MIN;
  out->longitude_e2 = INT16_MIN;
  out->utc_offset_min = INT16_MIN;
  out->time_updated_utc = (int32_t)f->time_updated_utc;
}

//! Overlay the v4-only fields (UV/precip/wind, lat-lon, 7-day daily, today's
//! hourly) onto *out by reading the active location's full WeatherDBEntry.
//! weather_service exposes only the v3 prefix + ordering; it stores the prefs
//! ordering index in node->id, so we map id -> prefs->locations[id] -> the
//! record key and read it directly. No-op (graceful degradation) when the
//! record is still a v3 entry written by a pre-v4 phone.
static void prv_overlay_v4(WxDsForecast *out, int location_id) {
  if (location_id < 0) {
    return;
  }
  SerializedWeatherAppPrefs *prefs = watch_app_prefs_get_weather();
  if (!prefs) {
    return;
  }
  if ((size_t)location_id >= prefs->num_locations) {
    watch_app_prefs_destroy_weather(prefs);
    return;
  }
  WeatherDBKey key = prefs->locations[location_id];
  watch_app_prefs_destroy_weather(prefs);

  const int len = weather_db_get_len((uint8_t *)&key, sizeof(key));
  if (len < (int)sizeof(WeatherDBEntryV3)) {
    return;
  }
  WeatherDBEntry *entry = task_zalloc_check(len);
  const status_t rv =
      weather_db_read((uint8_t *)&key, sizeof(key), (uint8_t *)entry, len);
  // Version alone isn't enough: a truncated/corrupt record can carry version==4 with a
  // v3-sized payload (weather_db_insert validates length, insert_stale does not) — reading
  // the v4 fields below would then run past the allocation. Gate on the minor-0 base size;
  // minor-1 appended fields get their own length gate below.
  if ((rv == S_SUCCESS) && (entry->version == WEATHER_DB_CURRENT_VERSION) &&
      (len >= (int)WEATHER_DB_V4_0_FIXED_SIZE)) {
    out->is_v4 = true;
    if (entry->today_uv_index_x10 >= 0) {
      out->today_uv = entry->today_uv_index_x10 / 10;  // schema stores UV*10
    }
    if (entry->today_precip_probability >= 0) {
      out->today_precip = entry->today_precip_probability;
    }
    if (entry->today_wind_speed > 0) {
      out->today_wind = entry->today_wind_speed;
    }
    if (entry->today_feels_like_temp !=
        WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP) {
      out->today_feels = entry->today_feels_like_temp;
    }
    out->latitude_e2 = entry->latitude_e2;
    out->longitude_e2 = entry->longitude_e2;

    uint8_t nd = entry->num_daily;
    if (nd > WX_DS_DAYS) {
      nd = WX_DS_DAYS;
    }
    out->num_daily = nd;
    for (uint8_t i = 0; i < nd; i++) {
      out->daily[i].high = entry->daily[i].high_temp;
      out->daily[i].low = entry->daily[i].low_temp;
      out->daily[i].type = entry->daily[i].weather_type;
      out->daily[i].precip = -1;  // minor-0 records carry no per-day metrics
      out->daily[i].wind = -1;
      out->daily[i].uv = -1;
      out->daily[i].wind_dir = -1;
      out->daily[i].feels = WX_DS_UNKNOWN_TEMP;   // zeroed struct would read as a KNOWN 0°
    }
    if (entry->today_hourly_count == WEATHER_DB_HOURLY_COUNT) {
      out->hourly_count = WX_DS_HOURLY;
      memcpy(out->hourly_type, entry->today_hourly_weather_type, WX_DS_HOURLY);
      memcpy(out->hourly_temp, entry->today_hourly_temp, WX_DS_HOURLY);
    }
    // v4.1 appended block (utc offset + per-day metrics): only present when the
    // phone stamped minor >= 1 AND the record is long enough to carry it all
    // (defensive against insert_stale).
    if (entry->minor_version >= 1 && len >= (int)WEATHER_DB_V4_1_FIXED_SIZE) {
      out->utc_offset_min = entry->location_utc_offset_min;
      for (uint8_t i = 0; i < nd; i++) {
        const WeatherDBDailyMetrics *m = &entry->daily_metrics[i];
        if (m->precip_probability != 255) out->daily[i].precip = m->precip_probability;
        if (m->wind_speed != 255)         out->daily[i].wind = m->wind_speed;
        if (m->uv_index_x10 != 255)       out->daily[i].uv = m->uv_index_x10 / 10;
      }
    }
    // v4.4 appended block (hourly UV) — the only source of a CURRENT UV reading;
    // today_uv_index_x10 is the day's figure, not a live one.
    if (entry->minor_version >= 4 && len >= (int)WEATHER_DB_V4_FIXED_SIZE) {
      for (int i = 0; i < WX_DS_HOURLY; i++) {
        const uint8_t v = entry->today_hourly_uv_x10[i];
        out->hourly_uv[i] = (v == 255) ? -1 : (int8_t)(v / 10);
      }
      out->has_hourly_uv = true;
    }
    // v4.2 appended block (today's raw warning readings + per-day feels-like).
    if (entry->minor_version >= 2 && len >= (int)WEATHER_DB_V4_2_FIXED_SIZE) {
      if (entry->today_wmo_code != 0xFF)         out->today_wmo = entry->today_wmo_code;
      if (entry->today_humidity_pct != 0xFF)     out->today_humidity = entry->today_humidity_pct;
      if (entry->today_visibility_m != 0xFFFF)   out->today_visibility_m = entry->today_visibility_m;
      if (entry->today_precip_sum_mm != 0xFFFF)  out->today_precip_sum_mm = entry->today_precip_sum_mm;
      for (uint8_t i = 0; i < nd; i++) {
        if (entry->daily_feels_like[i] != WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP) {
          out->daily[i].feels = entry->daily_feels_like[i];
        }
      }
    }
    // v4.3 appended block (dominant wind direction, today + per-day).
    if (entry->minor_version >= 3 && len >= (int)WEATHER_DB_V4_3_FIXED_SIZE) {
      if (entry->today_wind_dir_deg >= 0) {
        out->today_wind_dir = entry->today_wind_dir_deg;
      }
      for (uint8_t i = 0; i < nd; i++) {
        if (entry->daily_wind_dir_deg[i] >= 0) {
          out->daily[i].wind_dir = entry->daily_wind_dir_deg[i];
        }
      }
    }
  }
  task_free(entry);
}

// ===========================================================================
// Placeholder forecast data, served until the phone syncs real v4 weather records.
// Synthesizes a v4-shaped dataset (7-day daily[] + 24h hourly) from the real
// current conditions so the full 7-day touch-scrub + clock dial can be verified
// on-device before v4 data exists. Applied only when the active record is NOT
// already v4, so it vanishes automatically the moment real v4 records arrive.
// Set to 0 (or delete this block + its call site) to disable.
//
// QEMU-ONLY. It fabricates placeholder COORDINATES (San Francisco) for records
// that carry none, which on real hardware would pin a coordinate-less phone
// location to the wrong place on the globe — and hide the fact that the phone
// never sent coordinates for it. Locations are phone-owned now, so the watch
// must show exactly what the phone sent and nothing it invented.
// ===========================================================================
#if defined(CONFIG_SOC_QEMU)
#define WEATHER_V4_TEST_SEED 1
#else
#define WEATHER_V4_TEST_SEED 0
#endif
#if WEATHER_V4_TEST_SEED
static void prv_seed_v4_test(WxDsForecast *out) {
  if (out->is_v4) {
    return;  // real v4 data present — never overlay placeholder data on top of it
  }
  int hi = (out->today_high != WX_DS_UNKNOWN_TEMP) ? out->today_high : 21;
  int lo = (out->today_low != WX_DS_UNKNOWN_TEMP) ? out->today_low : 13;
  if (hi <= lo) {
    hi = lo + 6;
  }
  const uint8_t t0 = (out->current_weather_type <= 8) ? out->current_weather_type
                                                      : (uint8_t)WeatherType_Sun;

  // Today's extended metrics, so the detail view shows UV/precip/wind.
  if (out->today_uv < 0)     out->today_uv = 5;
  if (out->today_precip < 0) out->today_precip = 20;
  if (out->today_wind < 0)     out->today_wind = 12;
  if (out->today_wind_dir < 0) out->today_wind_dir = 225;   // SW, per the design mock
  // v4.2 warning readings — quiet values (no warning fires; the report's alert
  // stays on the precip path, "Chance of snow" for the seed's LightSnow today).
  if (out->today_wmo < 0)            out->today_wmo = 71;      // WMO light snow
  if (out->today_humidity < 0)       out->today_humidity = 62;
  if (out->today_visibility_m < 0)   out->today_visibility_m = 18000;
  if (out->today_precip_sum_mm < 0)  out->today_precip_sum_mm = 3;
  if (out->today_feels == WX_DS_UNKNOWN_TEMP && out->current_temp != WX_DS_UNKNOWN_TEMP) {
    out->today_feels = out->current_temp - 2;   // plausible wind-chill placeholder
  }

  // Coordinates so the globe has a location to reveal to (San Francisco).
  if (out->latitude_e2 == INT16_MIN)  out->latitude_e2 = 3777;    // 37.77 N
  if (out->longitude_e2 == INT16_MIN) out->longitude_e2 = -12242; // 122.42 W

  static const uint8_t kTypes[WX_DS_DAYS] = {
    WeatherType_Sun, WeatherType_PartlyCloudy, WeatherType_CloudyDay,
    WeatherType_LightRain, WeatherType_HeavyRain, WeatherType_PartlyCloudy,
    WeatherType_Sun,
  };
  // High-contrast spread (test only): fan days = 25/13, 30/16, 20/10, 28/15, 23/12.
  static const int8_t kHiDelta[WX_DS_DAYS] = { 0, -3,  2, -8,  0, -5, -1 };
  static const int8_t kLoDelta[WX_DS_DAYS] = { 0, -5, -2, -8, -3, -6, -1 };
  // Per-day precip % + wind mph + UV index — test seed only (real v4 is today-only → future = -1).
  static const int8_t kPrecip[WX_DS_DAYS] = { 20, 10, 15, 55, 80, 25, 5 };
  static const int8_t kWind[WX_DS_DAYS]   = { 12,  8, 14, 20, 25, 10, 6 };
  static const int8_t kUv[WX_DS_DAYS]     = {  5,  6,  3,  2,  1,  5,  7 };
  static const int16_t kDir[WX_DS_DAYS]   = { 225, 270, 90, 200, 180, 315, 135 };  // varied compass spread

  out->num_daily = WX_DS_DAYS;
  out->daily[0].high = hi;
  out->daily[0].low = lo;
  out->daily[0].type = t0;
  out->daily[0].precip = out->today_precip;  // today's value (set above)
  out->daily[0].wind   = out->today_wind;
  out->daily[0].wind_dir = out->today_wind_dir;
  out->daily[0].uv     = out->today_uv;
  out->daily[0].feels  = out->today_feels;
  for (int i = 1; i < WX_DS_DAYS; i++) {
    out->daily[i].high = hi + kHiDelta[i];
    out->daily[i].low = lo + kLoDelta[i];
    out->daily[i].type = kTypes[i];
    out->daily[i].precip = kPrecip[i];
    out->daily[i].wind   = kWind[i];
    out->daily[i].wind_dir = kDir[i];
    out->daily[i].uv     = kUv[i];
    out->daily[i].feels  = out->daily[i].high - 2;   // plausible per-day apparent temp
  }
  // Don't synthesize today/tomorrow: daily[0] already used the real current conditions above, and
  // daily[1] uses the real next-day forecast from the blobDB when present. Only days 2+ stay
  // synthesized until the phone ships real multi-day (v4) records.
  if (out->tomorrow_high != WX_DS_UNKNOWN_TEMP && out->tomorrow_low != WX_DS_UNKNOWN_TEMP) {
    out->daily[1].high   = out->tomorrow_high;
    out->daily[1].low    = out->tomorrow_low;
    out->daily[1].type   = (out->tomorrow_weather_type <= WeatherType_RainAndSnow)
                           ? out->tomorrow_weather_type : t0;
    out->daily[1].precip = kPrecip[1];  // keep the seeded chance for the demo (v3 has no real one)
  }

  // Shared diurnal hourly curve. Declared here rather than via weather_math.h:
  // this TU must not include the app headers (firmware weather-type collisions).
  extern const uint8_t weather_diurnal_curve[24];
  out->hourly_count = WX_DS_HOURLY;
  const int span = hi - lo;
  for (int h = 0; h < WX_DS_HOURLY; h++) {
    out->hourly_type[h] = t0;
    out->hourly_temp[h] = (int8_t)(lo + (span * weather_diurnal_curve[h]) / 100);
  }
}
#endif  // WEATHER_V4_TEST_SEED

#if defined(CONFIG_SOC_QEMU)
// QEMU has no phone → no weather_db records, so the weather app would be empty.
// Synthesize locations (current conditions + the v4 test seed) so the whole UI
// incl. the round 5-day screen is reachable for visual testing in the emulator.
// Index 0 is the "current location"; the next five match the default saved-city
// presets so the at-a-glance saved-locations list lights up with weather. The
// last ("Kingston, Jamaica") is deliberately NOT a preset — it stands in for a
// record the phone synced for a voice-added custom location (pairs with the
// QEMU-seeded "Jamaica" custom in saved_locations.c to exercise coordinate
// backfill + globe placement). Auto-disabled on real hardware.
typedef struct {
  const char *name;
  const char *phrase;
  uint8_t type;
  int8_t temp;
  int8_t high;
  int8_t low;
  int16_t lat_e2;
  int16_t lon_e2;
  int16_t utc_off_min;   // minutes east of UTC (summer offsets — synth only)
} QemuSynthCity;

static const QemuSynthCity s_qemu_cities[] = {
  { "Dursley, Gloucestershire, UK", "Light Snow", WeatherType_LightSnow,
    23, 28, 18, 5168, -235, 60 },
  { "New York, United States", "Sunny", WeatherType_Sun,
    31, 33, 24, 4071, -7401, -240 },
  { "London, United Kingdom", "Heavy Rain", WeatherType_HeavyRain,
    14, 16, 9, 5151, -13, 60 },
  { "Paris, France", "Partly Cloudy", WeatherType_PartlyCloudy,
    19, 21, 12, 4886, 235, 120 },
  { "Tokyo, Japan", "Cloudy", WeatherType_CloudyDay,
    26, 27, 20, 3568, 13969, 540 },
  { "Sydney, Australia", "Light Rain", WeatherType_LightRain,
    11, 13, 7, -3387, 15121, 600 },
  { "Kingston, Jamaica", "Sunny", WeatherType_Sun,
    31, 33, 25, 1797, -7679, -300 },
};
#define QEMU_SYNTH_CITY_COUNT ((int)(sizeof(s_qemu_cities) / sizeof(s_qemu_cities[0])))

static void prv_qemu_synth(int index, WxDsForecast *out) {
  const QemuSynthCity *city = &s_qemu_cities[index];
  *out = (WxDsForecast){0};
  for (size_t li = 0; city->name[li] && li < sizeof(out->location_name) - 1; li++) {
    out->location_name[li] = city->name[li];  // struct is zeroed → null-terminated
  }
  // day-0 condition uses the record's short_phrase
  for (size_t pi = 0; city->phrase[pi] && pi < sizeof(out->short_phrase) - 1; pi++) {
    out->short_phrase[pi] = city->phrase[pi];
  }
  out->is_current_location = (index == 0);
  out->current_temp = city->temp;
  out->current_weather_type = city->type;
  out->today_high = city->high;
  out->today_low = city->low;
  out->tomorrow_high = city->high - 4;   // QEMU has no phone/blobDB → stand-in tomorrow
  out->tomorrow_low = city->low - 4;
  out->tomorrow_weather_type = WeatherType_CloudyDay;
  out->today_uv = -1;
  out->today_precip = -1;
  out->today_wind = -1;
  out->today_feels = WX_DS_UNKNOWN_TEMP;   // zeroed struct would read as a KNOWN 0deg
  out->today_wmo = -1;                     // same trap: zeroed = WMO 0 "clear" / 0m visibility
  out->today_wind_dir = -1;                // zeroed = a KNOWN due-north wind
  out->today_humidity = -1;
  out->today_visibility_m = -1;
  out->today_precip_sum_mm = -1;
  out->latitude_e2 = city->lat_e2;
  out->longitude_e2 = city->lon_e2;
  out->utc_offset_min = city->utc_off_min;
  out->time_updated_utc = (int32_t)(rtc_get_time() - 240);   // "last updated" ~4 min ago
  out->is_v4 = false;
#if WEATHER_V4_TEST_SEED
  prv_seed_v4_test(out);
#endif
  // Mark the fully-populated synth record as v4: its coordinates are REAL (per-city
  // above, not the seed's placeholder), so gates that trust only v4 coords (the
  // custom-location coordinate backfill) behave in QEMU exactly as on hardware.
  out->is_v4 = true;
}
#endif

bool weather_ds_name_prefix(const char *name, const char *needle) {
  if (!name || !needle || !needle[0]) return false;
  for (size_t i = 0; needle[i]; i++) {
    char a = name[i], b = needle[i];
    if (a >= 'A' && a <= 'Z') a += 'a' - 'A';
    if (b >= 'A' && b <= 'Z') b += 'a' - 'A';
    if (a != b) return false;
  }
  return true;
}

bool weather_ds_name_matches(const char *name, const char *needle) {
  if (!name) return false;
  for (const char *p = name; *p; p++) {
    if (weather_ds_name_prefix(p, needle)) return true;
  }
  return false;
}

bool weather_ds_supported(void) {
#if defined(CONFIG_SOC_QEMU)
  return true;
#else
  return weather_service_supported_by_phone();
#endif
}

int weather_ds_location_count(void) {
#if defined(CONFIG_SOC_QEMU)
  return QEMU_SYNTH_CITY_COUNT;  // synthesized QEMU locations
#else
  if (!weather_service_supported_by_phone()) {
    return 0;
  }
  size_t count = 0;
  WeatherDataListNode *head = weather_service_locations_list_create(&count);
  weather_service_locations_list_destroy(head);
  return (int)count;
#endif
}

bool weather_ds_read_index(int index, WxDsForecast *out) {
  if (!out || index < 0) {
    return false;
  }
  size_t count = 0;
  WeatherDataListNode *head = weather_service_locations_list_create(&count);
  if (head && (size_t)index < count) {
    WeatherDataListNode *node =
        weather_service_locations_list_get_location_at_index(head, (unsigned int)index);
    if (node) {
      prv_fill_from_fw(out, &node->forecast, index);
      prv_overlay_v4(out, (int)node->id);  // v4 extras; no-op for v3 records
#if WEATHER_V4_TEST_SEED
      prv_seed_v4_test(out);  // Placeholder 7-day + hourly data until real v4 records arrive.
#endif
      weather_service_locations_list_destroy(head);
      return true;
    }
  }
  weather_service_locations_list_destroy(head);
#if defined(CONFIG_SOC_QEMU)
  if (index < QEMU_SYNTH_CITY_COUNT) {
    prv_qemu_synth(index, out);  // no phone in QEMU → synthesize so the UI is reachable
    return true;
  }
#endif
  return false;
}
