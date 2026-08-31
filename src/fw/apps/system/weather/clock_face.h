/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "weather_types.h"

typedef void (*ClockFaceWrapCallback)(void *context);

//! Clear the launch-persistent statics; call once from the app's init (a crashed
//! run never reaches unload, so the next launch would read stale pointers).
void clock_face_reset(void);

void clock_face_set_wrap_callback(ClockFaceWrapCallback callback, void *context);

bool clock_face_is_showing(void);

// Push the clock face window onto the stack.
// days/num_days: forecast data (same arrays as weather.c).
// day_index: which day the clock represents (0 = today). The centre shows the
//            current temperature for today, or the weekday abbreviation otherwise.
// hourly_types/hourly_count: the selected day's hourly weather types.
// animated: pass false when a pre-push transition has already played so the
//           built-in clock spiral intro fires without a system slide animation.
void clock_face_push(const WeatherLocationForecast *days, size_t num_days,
                     int day_index, const uint8_t *hourly_types, size_t hourly_count,
                     bool animated);

void clock_face_push_static(const WeatherLocationForecast *days, size_t num_days,
                            int day_index, const uint8_t *hourly_types,
                            size_t hourly_count, bool animated);

void clock_face_dismiss(bool animated);

// Update hourly temperatures for a given day (24 signed values).
// The update is applied only if the clock is currently showing that day.
void clock_face_update_hourly_temps_for_day(int day_index, const int8_t *temps, size_t count);

// TOMORROW's hourly series (v4 minor 5): feeds day 0's post-midnight dial
// positions. Both arrays must carry 24 entries.
void clock_face_set_tomorrow_hourly(const uint8_t *types, const int8_t *temps, size_t count);
