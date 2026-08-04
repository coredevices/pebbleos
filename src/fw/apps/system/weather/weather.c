/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

//! Weather system app — the rich ported UI, fed by the weather BlobDB.
//!
//! The animated forecast (PAGE_LIST) is the home/base window of the carousel
//! ring: forecast -> clock -> globe -> forecast. The forecast renders the active
//! location's days from the BlobDB; the clock face and globe are pushed on top of
//! it and dismissed back to it. City switching is wired from the forecast list
//! and the globe.
//! Multi-day data is today + tomorrow from a v3 record (the full 7-day + hourly
//! arrive once the phone writes v4 records and we switch to the direct v4 read).
//! No AppMessage — data comes from weather_db + a PEBBLE_WEATHER_EVENT
//! subscription.

#include "weather.h"
#include "clock_face.h"
#include "warning_dialog.h"
#include "weather_report.h"
#include "expanded_view.h"
#include "forecast_list.h"
#include "globe_view.h"
#include "saved_locations.h"
#include "weather_types.h"
#include "weather_math.h"
#include "weather_data_source.h"

#include "pebble_compat.h"
#include "pbl/services/timeline/timeline.h"  // UUID_WEATHER_DATA_SOURCE

#define WX_MAX_DAYS 7

typedef struct WeatherAppData {
  GlobeView *globe_view;
  EventServiceInfo weather_event_info;

  int location_count;
  int active_index;          // which configured location is shown
  unsigned int current_day_index;
  size_t days_received;

  WeatherLocationForecast days[WX_MAX_DAYS];
  char location_buf[64];
  char phrase_buf0[32];               // day 0: the record's real short_phrase
  char phrase_buf[WX_MAX_DAYS][15];   // days 1+: derived ("Partly Cloudy" max); row 0 unused (day 0 = phrase_buf0), kept for index parity with days[]
  char label_buf[WX_MAX_DAYS][10];    // "Wednesday" is the widest

  // Today's hourly series (v4; clock dial glyphs + Select temperature reveal).
  // The v4 record carries hourly for TODAY only, so this applies to day 0.
  uint8_t hourly_type[24];
  int8_t  hourly_temp[24];
  bool    hourly_valid;
  // Active location coordinates (v4; for the 3D globe).
  int16_t latitude_e2;
  int16_t longitude_e2;
  // Active location's timezone, minutes east of UTC (v4.1; for city-local
  // sunset/hourly display). INT16_MIN = unknown → fall back to the watch tz.
  int16_t utc_offset_min;
  // The user's GPS location — ALWAYS the record flagged is_current_location,
  // NEVER the active selection. Feeds the globe's "My Location" entry and the
  // saved-locations Current row, so switching city on the globe can't overwrite
  // them with a duplicate of the selected city.
  char    current_loc_buf[64];
  int16_t current_lat_e2;
  int16_t current_lon_e2;
  WeatherAppWarningDialog *warning_dialog;
} WeatherAppData;

static WeatherAppData *s_data;

static void prv_refresh(WeatherAppData *data);
static void prv_on_list_transition_done(void *ctx);
static void prv_on_city_select_requested(void *ctx);
static void prv_push_globe(WeatherAppData *data, bool animated);
static void prv_push_expanded(WeatherAppData *data, ExpandedViewEntrance entrance);
static void prv_expanded_select_to_globe(void *ctx);
static void prv_expanded_down_to_main(void *ctx);
static void prv_globe_back_to_expanded(void *ctx);
static void prv_sync_glance_strings(WeatherAppData *data);

// ---- Data: fill days[] from the active location's BlobDB record ----

// Derive a short condition phrase from the weather type. v4 records don't store
// per-day phrases (the phone used to send them via AppMessage), so future days
// get a derived phrase; day 0 keeps the record's real short_phrase.
static const char *prv_phrase_for_type(uint8_t type) {
  switch ((WeatherType)type) {
    case WeatherType_Sun:          return "Sunny";
    case WeatherType_PartlyCloudy: return "Partly Cloudy";
    case WeatherType_CloudyDay:    return "Cloudy";
    case WeatherType_LightRain:    return "Light Rain";
    case WeatherType_HeavyRain:    return "Rain";
    case WeatherType_LightSnow:    return "Light Snow";
    case WeatherType_HeavySnow:    return "Snow";
    case WeatherType_RainAndSnow:  return "Rain & Snow";
    case WeatherType_Generic:
    case WeatherType_Unknown:
    default:                       return "";
  }
}

// Derive the day label from the date: day 0 = "Today", day 1 = "Tomorrow",
// day 2+ = weekday abbreviation (v4 records don't carry labels).
static void prv_day_label(int day_offset, char *buf, size_t bufsize) {
  if (bufsize == 0) {
    return;
  }
  if (day_offset == 0) {
    strncpy(buf, "Today", bufsize - 1);
  } else if (day_offset == 1) {
    strncpy(buf, "Tomorrow", bufsize - 1);
  } else {
    static const char *const kWday[7] = {"Sunday", "Monday", "Tuesday", "Wednesday",
                                         "Thursday", "Friday", "Saturday"};
    time_t t = rtc_get_time() + (time_t)day_offset * SECONDS_PER_DAY;
    struct tm *lt = localtime(&t);  // compat maps to pbl_override_localtime
    int w = (lt && lt->tm_wday >= 0 && lt->tm_wday < 7) ? lt->tm_wday : 0;
    strncpy(buf, kWday[w], bufsize - 1);
  }
  buf[bufsize - 1] = '\0';
}

// The LOCATION's local hour (a saved city's "now" must follow that city's clock, via
// utc_offset_min), else the watch's own. -1 when the clock is unreadable. gmtime() is not
// linked into the firmware, so UTC is derived from localtime + the watch's own offset (the
// same pair the sunset computation uses).
static int prv_location_local_hour(const WxDsForecast *ds) {
  time_t now = rtc_get_time();
  struct tm *lt = localtime(&now);
  if (!lt) return -1;
  int hour = lt->tm_hour;
  if (ds && ds->utc_offset_min != INT16_MIN) {
    int mins = lt->tm_hour * 60 + lt->tm_min
             - (int)(time_get_gmtoffset() / 60) + ds->utc_offset_min;
    mins = ((mins % 1440) + 1440) % 1440;
    hour = mins / 60;
  }
  return (hour >= 0 && hour < WX_DS_HOURLY) ? hour : -1;
}

static int prv_uv_for_current_hour(const WxDsForecast *ds) {
  if (!ds || !ds->has_hourly_uv) return ds ? ds->today_uv : -1;
  const int hour = prv_location_local_hour(ds);
  if (hour < 0) return ds->today_uv;
  const int uv = ds->hourly_uv[hour];
  return (uv < 0) ? ds->today_uv : uv;
}

static void prv_fill_days_from_ds(WeatherAppData *data, const WxDsForecast *ds) {
  strncpy(data->location_buf, ds->location_name, sizeof(data->location_buf) - 1);
  data->location_buf[sizeof(data->location_buf) - 1] = '\0';

  // Today's hourly series + coordinates (v4 only; otherwise left invalid).
  data->hourly_valid = (ds->hourly_count == 24);
  if (data->hourly_valid) {
    memcpy(data->hourly_type, ds->hourly_type, sizeof(data->hourly_type));
    memcpy(data->hourly_temp, ds->hourly_temp, sizeof(data->hourly_temp));
  }
  data->latitude_e2 = ds->latitude_e2;
  data->longitude_e2 = ds->longitude_e2;
  data->utc_offset_min = ds->utc_offset_min;

  // Day 0 = today (full current metrics; v4 fills UV/precip/wind, else -1).
  snprintf(data->phrase_buf0, sizeof(data->phrase_buf0), "%s", ds->short_phrase);
  prv_day_label(0, data->label_buf[0], sizeof(data->label_buf[0]));
  // CURRENT-hour UV. The record's today_uv is the day's figure (peak), so a live value can
  // only come from the minor-4 hourly block; without it we fall back to the day's figure so
  // the screen shows something sane rather than "--".
  data->days[0] = (WeatherLocationForecast) {
    .location_name = data->location_buf,
    .is_current_location = ds->is_current_location,
    .current_temp = ds->current_temp,
    .today_high = ds->today_high,
    .today_low = ds->today_low,
    .today_uv = ds->today_uv,
    .today_uv_now = prv_uv_for_current_hour(ds),
    .today_precip_mm = ds->today_precip,
    .today_wind_mph = ds->today_wind,
    .today_wind_dir_deg = ds->today_wind_dir,
    .today_feels = ds->today_feels,
    .today_wmo = ds->today_wmo,
    .today_humidity = ds->today_humidity,
    .today_visibility_m = (int)ds->today_visibility_m,
    .today_precip_sum_mm = ds->today_precip_sum_mm,
    .current_weather_type = (WeatherType)ds->current_weather_type,
    .current_weather_phrase = data->phrase_buf0,
    // Current-HOUR conditions for the round header. hourly_type entries are phone-provided:
    // validate the enum range before trusting one; any gap falls back to the synced current.
    .current_type_now = (WeatherType)ds->current_weather_type,
    .current_temp_now = ds->current_temp,
    .current_phrase_now = data->phrase_buf0,
    .label = data->label_buf[0],
    .time_updated_utc = ds->time_updated_utc,
  };
  {
    // WATCH-local hour, deliberately NOT prv_location_local_hour: the header's spec is "what
    // the clock screen shows", and the clock indexes the hourly arrays by the watch's own
    // hour — using the location-shifted hour here made header and clock disagree by the tz
    // delta on the same city. Consistency with the clock outranks the saved-city refinement
    // (which the clock doesn't have either).
    time_t now_t = rtc_get_time();
    struct tm *lt_h = localtime(&now_t);
    const int hour = lt_h ? lt_h->tm_hour : -1;
    if (ds->hourly_count == WX_DS_HOURLY && hour >= 0 && hour < WX_DS_HOURLY &&
        ds->hourly_type[hour] <= WeatherType_RainAndSnow) {
      data->days[0].current_type_now   = (WeatherType)ds->hourly_type[hour];
      data->days[0].current_temp_now   = ds->hourly_temp[hour];
      // The hourly block has no phrase; derive one from the type. Static string — safe.
      data->days[0].current_phrase_now = (char *)prv_phrase_for_type(ds->hourly_type[hour]);
    }
  }
  data->days_received = 1;

  if (ds->num_daily >= 2) {
    // v4: days 1..N-1 from the daily[] array (high/low/type; current_temp and
    // the today-only metrics don't apply to future days).
    size_t nd = ds->num_daily;
    if (nd > WX_MAX_DAYS) {
      nd = WX_MAX_DAYS;
    }
    for (size_t i = 1; i < nd; i++) {
      prv_day_label((int)i, data->label_buf[i], sizeof(data->label_buf[i]));
      snprintf(data->phrase_buf[i], sizeof(data->phrase_buf[i]), "%s",
               prv_phrase_for_type(ds->daily[i].type));
      data->days[i] = (WeatherLocationForecast) {
        .location_name = data->location_buf,
        .is_current_location = ds->is_current_location,
        .current_temp = WX_DS_UNKNOWN_TEMP,
        .today_high = ds->daily[i].high,
        .today_low = ds->daily[i].low,
        .today_uv = ds->daily[i].uv,             // per-day UV (v4.1 daily_metrics; -1 when absent)
        .today_uv_now = -1,                      // hourly UV is today-only
        .today_precip_mm = ds->daily[i].precip,  // per-day precip + wind (v4.1; -1 when absent)
        .today_wind_mph = ds->daily[i].wind,
        .today_wind_dir_deg = ds->daily[i].wind_dir,
        .today_feels = ds->daily[i].feels,       // per-day feels-like (v4.2 daily_feels_like)
        .today_wmo = -1,             // warning readings are today-only (v4.2)
        .today_humidity = -1,
        .today_visibility_m = -1,
        .today_precip_sum_mm = -1,
        .current_weather_type = (WeatherType)ds->daily[i].type,
        .current_weather_phrase = data->phrase_buf[i],
        .label = data->label_buf[i],
        .time_updated_utc = ds->time_updated_utc,
      };
    }
    data->days_received = nd;
  } else {
    // v3 fallback: day 1 = tomorrow from the v3 prefix fields (high/low/type).
    const bool have_tomorrow = (ds->tomorrow_high != WX_DS_UNKNOWN_TEMP) ||
                               (ds->tomorrow_low != WX_DS_UNKNOWN_TEMP);
    if (have_tomorrow) {
      prv_day_label(1, data->label_buf[1], sizeof(data->label_buf[1]));
      snprintf(data->phrase_buf[1], sizeof(data->phrase_buf[1]), "%s",
               prv_phrase_for_type(ds->tomorrow_weather_type));
      data->days[1] = (WeatherLocationForecast) {
        .location_name = data->location_buf,
        .is_current_location = ds->is_current_location,
        .current_temp = WX_DS_UNKNOWN_TEMP,
        .today_high = ds->tomorrow_high,
        .today_low = ds->tomorrow_low,
        .today_uv = -1,
        .today_uv_now = -1,
        .today_precip_mm = -1,
        .today_wind_mph = -1,
        .today_wind_dir_deg = -1,
        .today_feels = WEATHER_SERVICE_LOCATION_FORECAST_UNKNOWN_TEMP,  // zeroed = a KNOWN 0°
        .today_wmo = -1,
        .today_humidity = -1,
        .today_visibility_m = -1,
        .today_precip_sum_mm = -1,
        .current_weather_type = (WeatherType)ds->tomorrow_weather_type,
        .current_weather_phrase = data->phrase_buf[1],
        .label = data->label_buf[1],
        .time_updated_utc = ds->time_updated_utc,
      };
      data->days_received = 2;
    }
  }
}

// Push the card's glance strings (sunset title, high/low°, location) to the forecast so its
// hero icon-fly can animate the identical text in, synced to the icon landing.
static void prv_sync_glance_strings(WeatherAppData *data) {
  const WeatherLocationForecast *today = data->days_received >= 1 ? &data->days[0] : NULL;
  char sunset[20], temp[16], loc[64];
  expanded_view_format_glance(today, data->latitude_e2, data->longitude_e2,
                              data->utc_offset_min,
                              sunset, sizeof(sunset), temp, sizeof(temp), loc, sizeof(loc));
  forecast_list_set_glance(sunset, temp,
                           today ? today->today_uv : -1,
                           today ? today->today_precip_mm : -1,
                           today ? today->today_wind_mph : -1);
}

// Refresh the GPS ("My Location") identity from the record flagged is_current_location
// (fallback: index 0 by convention). Kept SEPARATE from the active selection on purpose —
// see the WeatherAppData fields. Reuses the caller's scratch record.
static void prv_read_current_location(WeatherAppData *data, WxDsForecast *scratch) {
  int idx = 0;
  for (int i = 0; i < data->location_count; i++) {
    if (weather_ds_read_index(i, scratch) && scratch->is_current_location) {
      idx = i;
      break;
    }
  }
  if (!weather_ds_read_index(idx, scratch)) return;
  strncpy(data->current_loc_buf, scratch->location_name, sizeof(data->current_loc_buf) - 1);
  data->current_loc_buf[sizeof(data->current_loc_buf) - 1] = '\0';
  data->current_lat_e2 = scratch->latitude_e2;
  data->current_lon_e2 = scratch->longitude_e2;
}

static void prv_warning_dialog_dismiss_cb(void) {
  if (s_data) {
    s_data->warning_dialog = NULL;
  }
}

// No synced locations: warn and leave the dialog as the ONLY window, exactly like
// the original app — dismissing it empties the stack and app_event_loop returns,
// so the app closes. Covers launch (nothing pushed yet, the pop is a no-op) and a
// mid-run flush alike.
static void prv_show_no_data_warning(WeatherAppData *data) {
  if (data->warning_dialog) {
    return;  // only show one dialog at a time
  }
  app_window_stack_pop_all(false /* animated */);
  /// Shown when there are no forecasts available to show the user
  const char *warning_text = i18n_get("No location information available. To see weather, add "
                                      "locations in your Pebble mobile app.", data);
  data->warning_dialog =
      weather_app_warning_dialog_push(warning_text, prv_warning_dialog_dismiss_cb);
}

static void prv_refresh(WeatherAppData *data) {
  data->location_count = weather_ds_location_count();
  if (data->location_count <= 0) {
    data->days_received = 0;
    forecast_list_update_data(NULL, 0);
    prv_show_no_data_warning(data);
    return;
  }
  if (data->active_index >= data->location_count) {
    data->active_index = 0;
  }
  WxDsForecast ds;
  prv_read_current_location(data, &ds);
  if (weather_ds_read_index(data->active_index, &ds)) {
    prv_fill_days_from_ds(data, &ds);
    if (data->current_day_index >= data->days_received) {
      data->current_day_index = 0;
    }
    // Push the freshly-read days to the forecast base window. This safely
    // no-ops if the forecast window isn't created yet (e.g. the initial
    // prv_refresh in prv_init runs before the base window is pushed).
    forecast_list_update_data(data->days, data->days_received);
    // Keep the expanded card current too if it happens to be showing (no-op otherwise).
    expanded_view_update_data(data->days_received >= 1 ? &data->days[0] : NULL,
                              data->latitude_e2, data->longitude_e2,
                              data->utc_offset_min);
    // And the weather report — it borrows days[] and must re-render today's edition.
    weather_report_update_data(data->days, data->days_received);
    prv_sync_glance_strings(data);   // for the hero-fly text
  }
}

// ---- Navigation ----

// Page identities. The animated forecast (PAGE_LIST) is the persistent base window; at most one of
// {clock, globe, expanded card} sits on top of it, reached via buttons. s_page is a simple tag the
// button handlers keep updated (the swipe carousel that once read it has been removed). PAGE_MAIN,
// the old scrub screen, is gone; its enum value is retained only to keep the numbering stable.
typedef enum { PAGE_MAIN = 0, PAGE_LIST, PAGE_CLOCK, PAGE_GLOBE, PAGE_EXPANDED, PAGE_COUNT }
    WeatherCarouselPage;
static WeatherCarouselPage s_page = PAGE_LIST;

// ---- Sub-view transitions ----

// Push the clock face for the current day. static_push=true skips the clock's
// spiral intro (a plain appearance — used arriving from the globe); false plays
// the spiral intro (used after the main->clock icon shrink).
static void prv_push_clock(WeatherAppData *data, bool static_push) {
  unsigned int di = data->current_day_index;
  if (di >= WX_MAX_DAYS) {
    di = 0;
  }
  // Build the day's 24h hourly series so the clock dial + tap-to-reveal work for
  // EVERY day. Day 0 uses the real record's hourly when present; every other day
  // is synthesized from that day's high/low + type (a placeholder until the v4 struct is
  // extended to carry per-day hourly for the phone to populate — today is the only
  // day the record carries hourly for today).
  uint8_t s_hourly_type[24];   // stack scratch — every consumer copies synchronously
  int8_t  s_hourly_temp[24];
  const uint8_t *hourly_types = NULL;
  bool have_hourly = false;

  if (di == 0 && data->hourly_valid) {
    memcpy(s_hourly_type, data->hourly_type, sizeof(s_hourly_type));
    memcpy(s_hourly_temp, data->hourly_temp, sizeof(s_hourly_temp));
    hourly_types = s_hourly_type;
    have_hourly = true;
  } else if (di < data->days_received) {
    const WeatherLocationForecast *d = &data->days[di];
    int hi = (d->today_high != WX_DS_UNKNOWN_TEMP) ? d->today_high : 20;
    int lo = (d->today_low  != WX_DS_UNKNOWN_TEMP) ? d->today_low  : 12;
    if (hi <= lo) {
      hi = lo + 6;
    }
    // Shared diurnal curve — one shape for every day so each day's dial
    // reads naturally.
    const int span = hi - lo;
    for (int h = 0; h < 24; h++) {
      s_hourly_type[h] = d->current_weather_type;
      s_hourly_temp[h] = (int8_t)(lo + (span * weather_diurnal_curve[h]) / 100);
    }
    hourly_types = s_hourly_type;
    have_hourly = true;
  }

  if (static_push) {
    clock_face_push_static(data->days, data->days_received, (int)di,
                           hourly_types, have_hourly ? 24 : 0, false);
  } else {
    clock_face_push(data->days, data->days_received, (int)di,
                    hourly_types, have_hourly ? 24 : 0, false);
  }
  if (have_hourly) {
    clock_face_update_hourly_temps_for_day((int)di, s_hourly_temp, 24);
  }
}

// The forecast list's DOWN-again clock burst (dot flies to centre + shakes) ends here: push the
// clock face with its spiral-bloom intro ON TOP of the base list, so the centred dot blooms into
// today's hourly weather. The list resets itself to the resting forecast beneath.
static void prv_on_clock_burst_requested(void *ctx) {
  WeatherAppData *data = (WeatherAppData *)ctx;
  s_page = PAGE_CLOCK;
  prv_push_clock(data, false);  // static_push=false -> clock_face_push (spiral intro), no slide
}

// The expanded weather card is the middle of the vertical stack: forecast -> card -> globe.
// It borrows today's forecast (day 0) and takes the UP/DOWN callbacks that step the stack.
static void prv_push_expanded(WeatherAppData *data, ExpandedViewEntrance entrance) {
  const WeatherLocationForecast *today = (data->days_received >= 1) ? &data->days[0] : NULL;
  expanded_view_push(today, data->latitude_e2, data->longitude_e2,
                     data->utc_offset_min, entrance,
                     prv_expanded_down_to_main, data,
                     prv_expanded_select_to_globe, data);
}

static void prv_on_list_up_to_expanded(void *ctx) {
  WeatherAppData *data = (WeatherAppData *)ctx;
  // UP at the top of the forecast: the mainscreen has squashed down off the bottom and the hero
  // icon-fly + glance text have already animated in (drawn by the forecast, synced to the icon
  // landing). Reveal the static card; don't replay the text slide.
  if (expanded_view_is_showing()) return;
  s_page = PAGE_EXPANDED;
  prv_push_expanded(data, ExpandedViewEntranceStatic);
}

static void prv_expanded_select_to_globe(void *ctx) {
  WeatherAppData *data = (WeatherAppData *)ctx;
  // SELECT on the card: the card has already slid out to the LEFT (expanded_view fired us on the
  // slide-out completion). Push the globe sliding in from the RIGHT on top, then dismiss the
  // now-hidden card underneath (its white slide-out already covered the base — no flash).
  if (!data->globe_view) { expanded_view_dismiss(false); s_page = PAGE_LIST; return; }
  if (data->current_lat_e2 != INT16_MIN && data->current_lon_e2 != INT16_MIN) {
    globe_view_set_current_location(data->globe_view, data->current_loc_buf,
                                    data->current_lat_e2, data->current_lon_e2);
  }
  // Open hovering the ACTIVE location's pin — the city last selected to look at.
  globe_view_focus_coords(data->globe_view, data->latitude_e2, data->longitude_e2);
  s_page = PAGE_GLOBE;
  globe_view_push_slide_in_right(data->globe_view);
  expanded_view_dismiss(false);   // remove the card, now hidden beneath the globe
}

static void prv_expanded_down_to_main(void *ctx) {
  (void)ctx;
  // DOWN (or BACK) on the card: the reverse hero — the forecast rises back in
  // while the card's icon zooms down into the today header. Dismiss the card
  // un-animated so the fly IS the transition (no competing compositor slide).
  forecast_list_arm_return_fly();
  expanded_view_dismiss(false);
  s_page = PAGE_LIST;
}

static void prv_report_after_exit(void *ctx) {
  WeatherAppData *data = (WeatherAppData *)ctx;
  weather_report_arm_static_in();   // the unfold scene already delivered the entrance — hard cut
  weather_report_push(data->days, data->days_received, (int)data->current_day_index);
}

static void prv_on_report_requested(void *ctx) {
  WeatherAppData *data = (WeatherAppData *)ctx;
  // SELECT on the forecast main view -> slide the main screen off to the left, then hard-cut to
  // the weather report. Falls back to a plain push where the today-icon rect isn't available.
  if (!data || weather_report_is_showing()) return;
#if PBL_ROUND
  // Round has no hero icon-fly, but it DOES play the ball -> unfold -> paper scene, so it
  // must not be gated on the icon rect (which is rect-only and always false here — that
  // is why round used to hard-cut with no transition at all).
  forecast_list_start_select_exit(prv_report_after_exit, data);
#else
  GRect from;
  if (forecast_list_get_today_icon_rect(&from)) {
    forecast_list_start_select_exit(prv_report_after_exit, data);
  } else {
    weather_report_push(data->days, data->days_received, (int)data->current_day_index);
  }
#endif
}

static void prv_on_list_transition_done(void *ctx) {
  WeatherAppData *data = (WeatherAppData *)ctx;
  // The animated forecast is the carousel base window — pushed once and never dismissed.
  forecast_list_push(data->days, data->days_received,
                     (int)data->current_day_index, false,
                     NULL, NULL,
                     prv_on_city_select_requested, data);
  forecast_list_set_on_clock_request(prv_on_clock_burst_requested, data);
  forecast_list_set_on_up_request(prv_on_list_up_to_expanded, data);
  forecast_list_set_on_select_request(prv_on_report_requested, data);
  // The base window now exists, so the initial glance strings will actually land (the prv_refresh
  // in prv_init ran before it, when forecast_list_set_glance_strings was a no-op).
  prv_sync_glance_strings(data);
}

// ---- Globe (3D earth) ----
// The globe renders + touch-spins and is reachable from the forecast list's
// city-select and from the clock's wrap gesture. Actually changing the active
// location from the globe (location_select) is not yet wired up — for now
// selecting a location just returns to the main screen.

static void prv_push_globe(WeatherAppData *data, bool animated) {
  if (!data->globe_view) {
    return;
  }
  if (data->current_lat_e2 != INT16_MIN && data->current_lon_e2 != INT16_MIN) {
    globe_view_set_current_location(data->globe_view, data->current_loc_buf,
                                    data->current_lat_e2, data->current_lon_e2);
  }
  globe_view_focus_coords(data->globe_view, data->latitude_e2, data->longitude_e2);
  globe_view_push_animated(data->globe_view, animated);
}

static void prv_on_city_select_requested(void *ctx) {
  prv_push_globe((WeatherAppData *)ctx, true);
}

// (globe DOWN-to-expanded path deleted with the dead main_callback chain)

static void prv_globe_back_to_expanded(void *ctx) {
  WeatherAppData *data = (WeatherAppData *)ctx;
  // BACK on the globe cradle: the globe has already slid out to the RIGHT (globe_view fired us on the
  // slide-out completion). Dismiss it (revealing the forecast base) and bring the whole card back in
  // from the LEFT — the exact mirror of the SELECT slide-out-left that got us here.
  if (data->globe_view) {
    globe_view_dismiss(data->globe_view, false);
  }
  if (clock_face_is_showing()) {
    clock_face_dismiss(false);
  }
  s_page = PAGE_EXPANDED;
  prv_push_expanded(data, ExpandedViewEntranceCardFromLeft);
}

// Switch the ACTIVE location to a phone-synced record. `ds_index` comes straight
// off the pin/row, so the selection can't fail to resolve.
static void prv_select_ds_location(WeatherAppData *data, int ds_index) {
  if (!data || ds_index < 0 || ds_index >= weather_ds_location_count()) return;
  // prv_refresh re-reads the record and pushes the chosen city into every view
  // (forecast base, card, report, glance).
  data->active_index = ds_index;
  data->current_day_index = 0;
  prv_refresh(data);
  // The reveal that follows shows the mainscreen: replay the clock-slot intro so
  // the NEW city's name holds the top slot, then swaps to the time.
  forecast_list_replay_location_intro();
}

static void prv_globe_location_selected(int ds_index, bool force, void *ctx) {
  (void)force;
  // IMPORTANT: do NOT dismiss the globe here. The globe pops itself right after
  // this callback returns (commit_hovered_location() → globe_view_pop() pops the
  // top window); its self-pop lands on the forecast base — the mainscreen.
  prv_select_ds_location((WeatherAppData *)ctx, ds_index);
  s_page = PAGE_LIST;
}

// ---- Saved-locations list ----
// Reached by tapping the globe cradle's saved-locations bar. Locations are owned
// by the PHONE app; this screen only picks between them. It dismisses itself.
static void prv_on_saved_location_selected(int ds_index, void *ctx) {
  WeatherAppData *data = (WeatherAppData *)ctx;
  prv_select_ds_location(data, ds_index);
  // Land on the MAINSCREEN, like a globe pin commit: clear every window between
  // the saved-locations list (it dismisses itself right after this returns) and
  // the forecast base. All unanimated — they sit hidden beneath the list.
  if (data && data->globe_view) {
    globe_view_dismiss(data->globe_view, false);
  }
  expanded_view_dismiss(false);
  clock_face_dismiss(false);
  s_page = PAGE_LIST;
}

static void prv_open_saved_locations(void *ctx) {
  WeatherAppData *data = (WeatherAppData *)ctx;
  if (!data) {
    return;
  }
  SavedLocationsConfig config = {
    // Both are weather-ds record indices now, so the active city pre-highlights.
    .active_ds_index = data->active_index,
    .select_callback = prv_on_saved_location_selected,
    .select_context = data,
  };
  saved_locations_push(&config);
}

static void prv_on_clock_wrap_to_main(void *ctx) {
  (void)ctx;
  // UP on the clock returns to the forecast base with a Timeline squash-stretch: the clock has
  // already jelly-stretched down off the bottom (clock_face.c); arm the forecast's matching
  // drop-in-from-above BEFORE dismissing the clock so it plays on the first revealed frame. The
  // forecast is the persistent base window — popping the clock simply reveals it. (The globe
  // stays reachable via the carousel / swipe-left.)
  forecast_list_arm_squash_in();
  if (clock_face_is_showing()) {
    clock_face_dismiss(false);
  }
  s_page = PAGE_LIST;
}

// The swipe-driven carousel ring (weather_carousel_navigate + prv_carousel_show + s_ring) was
// removed: screens are now reached only via buttons. s_page remains as a simple page-identity tag
// the button handlers keep updated (ready for gesture nav to be re-added against the new layout).

static void prv_handle_weather(PebbleEvent *event, void *context) {
  prv_refresh(s_data);
}

static NOINLINE void prv_init(void) {
  // System-app statics survive across launches and a crashed run never reaches
  // unload — clear every module's launch-persistent state first (like s_page).
  clock_face_reset();
  expanded_view_reset();
  weather_report_reset();

  WeatherAppData *data = app_zalloc_check(sizeof(WeatherAppData));
  app_state_set_user_data(data);
  s_data = data;
#ifdef CONFIG_TOUCH
  // v4.32 runs a SYSTEM touch-navigation twin inside every system app by default: a system
  // subscription slot sees each touch first and holds gestures while it classifies swipes
  // into synthetic button events. This app carries its own complete touch navigation (every
  // screen maps swipes itself, the globe pans on the raw stream), so the twin both delays
  // and swallows our gestures — on-watch this read as "swipes are laggy or dead everywhere
  // but the globe". Opt out so our raw touch_service subscriptions see the stream exactly
  // as they did on the v4.18 base.
  app_touch_navigation_enable(false);
#endif
  // The animated forecast is the carousel base window; start the ring there.
  s_page = PAGE_LIST;
  // zalloc leaves coords at 0 — a VALID lat/lon (0°,0°). Unknown must be INT16_MIN so the
  // "have GPS coords?" gates don't pass before prv_refresh reads the current-location record.
  data->current_lat_e2 = INT16_MIN;
  data->current_lon_e2 = INT16_MIN;

  prv_refresh(data);
  if (data->warning_dialog) {
    // No data — the warning dialog is the only window; dismissing it exits the
    // app. Skip the rest of the UI (globe, event subscription, base window).
    return;
  }

  // Create the globe once (held for the app's lifetime; ~31KB of cubemap +
  // starfield + sequence resources). prv_refresh ran first, so lat/lon is set.
  data->globe_view = globe_view_create();
  if (data->globe_view) {
    globe_view_set_location_select_callback(data->globe_view,
                                            prv_globe_location_selected, data);
    globe_view_set_back_callback(data->globe_view, prv_globe_back_to_expanded, data);
    globe_view_set_saved_locations_callback(data->globe_view, prv_open_saved_locations, data);
    if (data->current_lat_e2 != INT16_MIN && data->current_lon_e2 != INT16_MIN) {
      globe_view_set_current_location(data->globe_view, data->current_loc_buf,
                                      data->current_lat_e2, data->current_lon_e2);
    }
  }
  clock_face_set_wrap_callback(prv_on_clock_wrap_to_main, data);

  // App-global weather event subscription: re-read the BlobDB whenever the phone
  // writes a new weather record (there is no persistent main window to host it).
  data->weather_event_info = (EventServiceInfo) {
    .type = PEBBLE_WEATHER_EVENT,
    .handler = prv_handle_weather,
  };
  event_service_client_subscribe(&data->weather_event_info);

  // Push the animated forecast as the carousel base window.
  prv_on_list_transition_done(data);

}

static void prv_deinit(void) {
  if (s_data) {
    if (s_data->weather_event_info.handler) {
      event_service_client_unsubscribe(&s_data->weather_event_info);
    }
    i18n_free_all(s_data);
    clock_face_set_wrap_callback(NULL, NULL);
    if (s_data->globe_view) {
      globe_view_destroy(s_data->globe_view);
      s_data->globe_view = NULL;
    }
  }
  s_data = NULL;
}

static void prv_main(void) {
  prv_init();
  app_event_loop();
  prv_deinit();
}

const PebbleProcessMd* weather_app_get_info(void) {
  static const PebbleProcessMdSystem s_weather_app_info = {
    .common = {
      .main_func = prv_main,
      .uuid = UUID_WEATHER_DATA_SOURCE,
    },
    .name = i18n_noop("Weather"),
    .icon_resource_id = RESOURCE_ID_GENERIC_WEATHER_TINY,
  };
  return weather_ds_supported() ? (const PebbleProcessMd *)&s_weather_app_info : NULL;
}
