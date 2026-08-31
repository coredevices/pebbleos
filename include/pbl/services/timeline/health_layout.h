/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "item.h"
#include "layout_layer.h"
#include "timeline_layout.h"

#include "pbl/util/attributes.h"

#define HEALTH_METRIC_BUFFER_LENGTH 128

//! The different types of health cards the app shows
typedef enum HealthCardType {
  HealthCardType_Activity = 0,
  HealthCardType_Sleep,
  HealthCardTypeCount
} HealthCardType;

//! Shared with insights to allow the app to launch into the appropriate card
typedef struct PACKED HealthLaunchArgs {
  union {
    struct {
      HealthCardType card_type:8; //!< Tells us if we need to launch into an activity or sleep card
    };
    uint32_t args;
  };
} HealthLaunchArgs;

typedef enum {
  ActivitySessionMetric_Duration = 0,
  ActivitySessionMetric_Calories,
  ActivitySessionMetric_Distance,
  ActivitySessionMetric_Pace,
  ActivitySessionMetricCount,
} ActivitySessionMetric;

typedef struct {
  KinoLayer *icon_layer;
  const char *value_fmt;
} MetricData;

typedef struct {
  TimelineLayout timeline_layout;
  MetricData metric_data[ActivitySessionMetricCount];
} HealthLayout;

LayoutLayer *health_layout_create(const LayoutLayerConfig *config);

bool health_layout_verify(bool existing_attributes[]);
