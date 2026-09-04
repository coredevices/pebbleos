/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "layout_layer.h"
#include "timeline_layout.h"

typedef enum {
  WeatherTimeType_None = 0,
  WeatherTimeType_Pin,
} WeatherTimeType;

typedef struct {
  TimelineLayout timeline_layout;
} WeatherLayout;

LayoutLayer *weather_layout_create(const LayoutLayerConfig *config);

bool weather_layout_verify(bool existing_attributes[]);
