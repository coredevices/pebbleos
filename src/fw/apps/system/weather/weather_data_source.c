/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
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
  (void)index;
  memset(out, 0, sizeof(*out));
  prv_copy_str(out->location_name, sizeof(out->location_name), f->location_name);
  prv_copy_str(out->short_phrase, sizeof(out->short_phrase), f->current_weather_phrase);
  // The record carries the real flag — list position is NOT it (the current
  // location need not be first, and may be absent entirely).
  out->is_current_location = f->is_current_location;
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
    if (entry->minor_version >= 4 && len >= (int)WEATHER_DB_V4_4_FIXED_SIZE) {
      for (int i = 0; i < WX_DS_HOURLY; i++) {
        const uint8_t v = entry->today_hourly_uv_x10[i];
        out->hourly_uv[i] = (v == 255) ? -1 : (int8_t)(v / 10);
      }
      out->has_hourly_uv = true;
    }
    // v4.5 appended block (TOMORROW's hourly series) — fills the clock dial's
    // post-midnight positions with real data instead of blanks.
    if (entry->minor_version >= 5 && len >= (int)WEATHER_DB_V4_FIXED_SIZE &&
        entry->tomorrow_hourly_count == WEATHER_DB_HOURLY_COUNT) {
      out->tomorrow_hourly_count = WX_DS_HOURLY;
      memcpy(out->tomorrow_hourly_type, entry->tomorrow_hourly_weather_type, WX_DS_HOURLY);
      memcpy(out->tomorrow_hourly_temp, entry->tomorrow_hourly_temp, WX_DS_HOURLY);
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
  return weather_service_supported_by_phone();
}

int weather_ds_location_count(void) {
  if (!weather_service_supported_by_phone()) {
    return 0;
  }
  size_t count = 0;
  WeatherDataListNode *head = weather_service_locations_list_create(&count);
  weather_service_locations_list_destroy(head);
  return (int)count;
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
      weather_service_locations_list_destroy(head);
      return true;
    }
  }
  weather_service_locations_list_destroy(head);
  return false;
}
