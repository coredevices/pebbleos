/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/ui/animation.h"
#include "applib/ui/progress_layer.h"

#include <stdint.h>

#define LOADING_LAYER_DEFAULT_SIZE { 79, PROGRESS_SUGGESTED_HEIGHT }

typedef void (*LoadingLayerAnimCompleteCb)(void *context);

typedef struct {
  ProgressLayer progress_layer;
  Animation *animation;
  GRect full_frame;
} LoadingLayer;

void loading_layer_init(LoadingLayer *loading_layer, const GRect *frame);

void loading_layer_deinit(LoadingLayer *loading_layer);

void loading_layer_shrink(LoadingLayer *loading_layer, uint32_t delay, uint32_t duration,
                          AnimationStoppedHandler stopped_handler, void *context);

void loading_layer_grow(LoadingLayer *loading_layer, uint32_t duration, uint32_t delay);

void loading_layer_pause(LoadingLayer *loading_layer);
