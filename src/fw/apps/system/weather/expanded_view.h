/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "weather_types.h"

// The card's weather-icon box. This is ALSO the hero-fly landing target in forecast_list.c
// (prv_start_up_to_card's fly_dest) — both draw sites use these so the flown icon and the static
// card icon land pixel-identically. EV_ICON_Y centres the icon glyph vertically between the status
// time and the "Sunset H:MM" title.
// ROUND: the icon carries its share of the group shift (EV_ROUND_GROUP_DY) that centres
// icon + "Sunset H:MM" + "high/low°" on the screen's vertical midline. Both draw sites read
// this constant, so the hero fly's landing target moves with the static icon automatically —
// do NOT hardcode either one.
// Small rect (flint/asterix 144x168): the card column compresses — smaller
// icon, tighter rows, and the two dials become a single meters text line.
#define EV_SMALL_RECT (!PBL_ROUND && PBL_DISPLAY_HEIGHT < 200)
#define EV_ICON_SIZE (EV_SMALL_RECT ? 56 : 74)
#define EV_ICON_Y    PBL_IF_ROUND_ELSE(38, (EV_SMALL_RECT ? 24 : 26))

// How the card animates in when pushed.
typedef enum {
  ExpandedViewEntranceStatic = 0,     // no entrance — the forecast's hero icon-fly already placed it
  ExpandedViewEntranceTextFromLeft,   // the glance text slides in from the left (arriving from globe DOWN)
  ExpandedViewEntranceCardFromLeft,   // the whole card slides in from the left (arriving from globe BACK)
} ExpandedViewEntrance;

// Full-screen "expanded weather" card for TODAY — a standalone recreation of the
// Pebble Timeline weather pin card (src/fw/services/timeline/weather_layout.c):
// a condition title, the big LECO temperature, the weather icon, the location, a
// friendly date header, and a short stats body.
//
// It sits between the forecast list (below) and the globe (above) in the vertical
// button navigation:
//   DOWN   -> on_down   callback (weather.c dismisses the card, revealing the forecast)
//   BACK   -> on_down   callback (same as DOWN)
//   SELECT -> on_select callback: the card slides out to the LEFT, then on_select fires and
//             weather.c pushes the globe sliding in from the RIGHT. Fired AFTER the slide-out.
//   UP     -> unbound (no navigation)
//
// `today` is borrowed only for the duration of this call; the fields are copied, so
// the caller may free/reuse its forecast afterward. lat_e2/lon_e2 are the active
// location's coordinates (degrees * 100, INT16_MIN if unknown) — used to compute the
// local sunset time (there is no sunset field in the synced weather data).
// utc_off_min: the LOCATION's timezone (minutes east of UTC, v4.1) so the sunset reads in
// city-local time for saved cities; INT16_MIN falls back to the watch's timezone.
// entrance: how the card animates in (see ExpandedViewEntrance). Use ...Static when arriving via
// the forecast's hero icon-fly (which already animated the text in, synced to the icon landing),
// ...TextFromLeft from globe DOWN, ...CardFromLeft from globe BACK (mirrors the SELECT slide-out).
//! Clear the launch-persistent statics; call once from the app's init (a crashed
//! run never reaches unload, so the next launch would read stale pointers).
void expanded_view_reset(void);

void expanded_view_push(const WeatherLocationForecast *today,
                        int16_t lat_e2, int16_t lon_e2, int16_t utc_off_min,
                        ExpandedViewEntrance entrance,
                        void (*on_down)(void *ctx), void *on_down_ctx,
                        void (*on_select)(void *ctx), void *on_select_ctx);

// Format the glance strings (sunset title, "high/low°", location) exactly as the card renders them.
// Used by the forecast to animate the identical text in during the hero icon-fly.
void expanded_view_format_glance(const WeatherLocationForecast *today, int16_t lat_e2, int16_t lon_e2,
                                 int16_t utc_off_min,
                                 char *sunset, size_t sunset_sz, char *temp, size_t temp_sz,
                                 char *loc, size_t loc_sz);

// Draw the whole card body EXCEPT the icon (status bar, sunset title, high/low°, UV + RAIN meters),
// every element shifted right by `tdx`. `status` is the status-bar text (the time, or "Last updated
// ..."); NULL skips the status bar (the card draws its own during the last-updated -> time swap).
// The card and the forecast's hero icon-fly both call this so the two are pixel-identical.
void expanded_view_draw_glance_content(GContext *ctx, int W, int tdx, const char *status,
                                       const char *sunset, const char *temp, int uv, int precip,
                                       int wind);

// Format "Last updated H:MM" from the forecast's time_updated_utc, exactly as the card shows it.
void expanded_view_format_updated(const WeatherLocationForecast *today, char *out, size_t out_sz);

// Remove the card if it is currently on the stack.
void expanded_view_dismiss(bool animated);

bool expanded_view_is_showing(void);

// Refresh the displayed data when a new record arrives while the card is up.
// No-op if the card is not currently showing.
void expanded_view_update_data(const WeatherLocationForecast *today,
                               int16_t lat_e2, int16_t lon_e2, int16_t utc_off_min);
