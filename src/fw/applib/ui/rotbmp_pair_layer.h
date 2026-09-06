/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/ui/layer.h"
#include "applib/ui/rotate_bitmap_layer.h"
#include "applib/graphics/gtypes.h"

/**
   A pair of images, one drawn white-transparent, the other black-transparent
   used to draw a single image which has white, black, and transparent regions
 **/
typedef struct {
  Layer layer;

  RotBitmapLayer white_layer;
  RotBitmapLayer black_layer;

} RotBmpPairLayer;

//! white and black *must* have the same dimensions, and *shouldn't* have any overlap of each other
void rotbmp_pair_layer_init(RotBmpPairLayer *pair, GBitmap *white, GBitmap *black);

void rotbmp_pair_layer_deinit(RotBmpPairLayer *pair);

void rotbmp_pair_layer_set_angle(RotBmpPairLayer *pair, int32_t angle);
void rotbmp_pair_layer_increment_angle(RotBmpPairLayer *pair, int32_t angle_change);

void rotbmp_pair_layer_set_src_ic(RotBmpPairLayer *pair, GPoint ic);

//! exchanges black with white
void rotbmp_pair_layer_inver_colors(RotBmpPairLayer *pair);
