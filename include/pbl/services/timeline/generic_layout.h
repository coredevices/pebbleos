/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "layout_layer.h"
#include "timeline_layout.h"

typedef struct {
  TimelineLayout timeline_layout;
} GenericLayout;

LayoutLayer *generic_layout_create(const LayoutLayerConfig *config);

bool generic_layout_verify(bool existing_attributes[]);
