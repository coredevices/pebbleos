/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pebble_compat.h"
#include "weather_types.h"

// "The Weather Report" — the screen SELECT opens from the forecast list.
// On rect (emery/obelix): a static newspaper front page for today — masthead, dateline,
// weekday headline, condition deck, framed press-photo icon, temperature column and a
// boxed UV-index tally on cream newsprint. (7-day paging comes later.)
// On round (gabbro/getafix): the classic condensed layout host (weather_app_layout),
// hold UP/DOWN to scroll days — unchanged.
// `days` is BORROWED (not copied) — the caller (weather.c) owns it for the app's lifetime.
void weather_report_push(const WeatherLocationForecast *days, size_t num_days, int start_day_index);


// Arm a hard-cut entrance for the NEXT push (no system slide, no slide-in) — used when a
// transition scene (the unfold) has already delivered the entrance. Rect only.
void weather_report_arm_static_in(void);

bool weather_report_is_showing(void);

// Re-point the borrowed days array after a data refresh. No-op when the view isn't showing.
void weather_report_update_data(const WeatherLocationForecast *days, size_t num_days);
