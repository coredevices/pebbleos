/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "animations.h"

#include "applib/ui/property_animation.h"
#include "applib/ui/ui.h"

#include <stdint.h>

int64_t timeline_animation_interpolate_moook_soft(int32_t normalized,
                                                  int64_t from, int64_t to) {
  return interpolate_moook_soft(normalized, from, to, TIMELINE_NUM_MOOOK_FRAMES_MID);
}

int64_t timeline_animation_interpolate_moook_second_half(int32_t normalized,
                                                         int64_t from, int64_t to) {
  const int32_t cut = (normalized + ANIMATION_NORMALIZED_MAX) / 2;
  return timeline_animation_interpolate_moook_soft(cut, from, to);
}

void timeline_animation_layer_stopped_cut_to_end(Animation *animation, bool finished,
                                                 void *context) {
  PropertyAnimation *property_animation = context;
  if (finished) {
    return;
  }

  GRect to;
  Layer *layer;
  if (property_animation_get_to_grect(property_animation, &to) &&
      property_animation_get_subject(property_animation, (void **)&layer)) {
    layer_set_frame(layer, &to);
  }
}
