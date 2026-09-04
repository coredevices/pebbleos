/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "weather_types.h"

// Push the condensed weekly forecast list onto the window stack.
// days/num_days are the same arrays maintained by weather.c.
// start_day_index: the day that was focused in the main view; list opens scrolled to it.
// animated: true for normal push, false for instant cut (used after transition anim).
// on_pop/on_pop_ctx: optional callback fired when the list window is dismissed.
// The Weather Channel intro card: solid hold, then a cross-dissolve into the
// live forecast (call once, right after the launch push).

// Re-arm the clock-slot location intro (2s city name -> sunset-swap to the time):
// the next appear replays it — call on a globe city commit before the reveal.
void forecast_list_replay_location_intro(void);

void forecast_list_push(const WeatherLocationForecast *days, size_t num_days,
                        int start_day_index, bool animated,
                        void (*on_pop)(void *ctx), void *on_pop_ctx,
                        void (*on_city_select)(void *ctx), void *on_city_select_ctx);

// Register the callback the DOWN-again clock-burst fires on completion (to push the clock face).
void forecast_list_set_on_clock_request(void (*cb)(void *ctx), void *ctx);

// Register the callback the UP-at-top squash-out fires on completion (to push the expanded card).
void forecast_list_set_on_up_request(void (*cb)(void *ctx), void *ctx);

// Arm the Timeline squash-stretch "drop-in from above" the next time the forecast window
// reappears. Called by weather.c right before dismissing the clock on UP (clock → forecast).
void forecast_list_arm_squash_in(void);

// Round report-BACK: mainscreen slides in from the left (the card<->globe hslide pair's
// grammar). Armed by weather_report.c just before its slide-out pops the window.
void forecast_list_arm_hslide_in(void);

// Arm the card->forecast return: the rise-in squash PLUS the card's big icon
// squash-stretch zooming back down into the today-header spot (reverse hero).
void forecast_list_arm_return_fly(void);

// Register the callback SELECT fires on the resting main view (to open the condensed view screen).
void forecast_list_set_on_select_request(void (*cb)(void *ctx), void *ctx);

// SELECT expand (Timeline pin->card): the resting today-icon rect (screen coords) the condensed
// view grows from, and the main-slides-left phase that hard-cuts to the condensed view on stop.
bool forecast_list_get_today_icon_rect(GRect *out);
void forecast_list_start_select_exit(void (*done_cb)(void *ctx), void *ctx);

// Call whenever new forecast data arrives so the list stays current.
// No-op if the list window is not currently showing.
void forecast_list_update_data(const WeatherLocationForecast *days, size_t num_days);

// The expanded card's glance data (sunset title + high/low° strings, UV + rain %). The UP-to-card
// hero transition animates this identical content (time, text, meters) in from the left, synced to
// the icon-fly landing.
void forecast_list_set_glance(const char *sunset, const char *temp, int uv, int precip,
                              int wind);
