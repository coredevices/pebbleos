/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/graphics/gdraw_command_image.h"
#include "applib/ui/content_indicator_private.h"
#include "applib/ui/kino/kino_layer.h"
#include "pbl/services/weather/weather_service.h"

typedef struct WeatherAppLayout {
  Layer root_layer;
  Layer content_layer;
  KinoLayer current_weather_icon_layer;
  KinoLayer tomorrow_weather_icon_layer;
  const WeatherLocationForecast *forecast;
  GFont location_font;
  GFont temperature_font;
  GFont high_low_phrase_font;
  GFont tomorrow_font;
  Layer down_arrow_layer;
  ContentIndicator content_indicator;
  struct { // used during animations
    const WeatherLocationForecast *next_forecast;
    bool hide_bottom_half_text;
  } animation_state;
} WeatherAppLayout;

void weather_app_layout_init(WeatherAppLayout *layout, const GRect *frame);

void weather_app_layout_set_data(WeatherAppLayout *layout,
                                 const WeatherLocationForecast *forecast);

void weather_app_layout_set_down_arrow_visible(WeatherAppLayout *layout, bool is_down_visible);

void weather_app_layout_deinit(WeatherAppLayout *layout);

void weather_app_layout_animate(WeatherAppLayout *layout, WeatherLocationForecast *new_forecast,
                                bool animate_down);
