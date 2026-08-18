/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "applib/ui/animation.h"
#include "applib/ui/animation_private.h"
#include "applib/ui/property_animation.h"
#include "applib/ui/property_animation_private.h"
#include "pbl/util/attributes.h"

Animation *WEAK animation_create(void) {
  Animation *animation = malloc(sizeof(AnimationPrivate));
  memset(animation, 0, sizeof(AnimationPrivate));
  return animation;
}

bool WEAK animation_destroy(Animation *animation) {
  if (!animation) {
    return false;
  }
  free(animation);
  return true;
}

bool WEAK animation_set_elapsed(Animation *animation, uint32_t elapsed_ms) {
  if (!animation) {
    return false;
  }
  ((AnimationPrivate*)animation)->abs_start_time_ms += elapsed_ms;
  return true;
}

bool WEAK animation_get_progress(Animation *animation, AnimationProgress *progress) {
  if (!animation) {
    return false;
  }
  // For use in this stub, start time is 0, so anything past 0 is elapsed
  *progress = ((AnimationPrivate*)animation)->duration_ms * ANIMATION_NORMALIZED_MAX /
      ((AnimationPrivate*)animation)->abs_start_time_ms;
  return true;
}

bool WEAK animation_get_elapsed(Animation *animation, int32_t *elapsed_ms) {
  if (!animation) {
    return false;
  }
  // For use in this stub, start time is 0, so anything past 0 is elapsed
  *elapsed_ms = ((AnimationPrivate*)animation)->abs_start_time_ms;
  return true;
}

bool WEAK animation_set_delay(Animation *animation, uint32_t delay_ms) {
  if (!animation) {
    return false;
  }
  ((AnimationPrivate*)animation)->delay_ms = delay_ms;
  return true;
}

bool WEAK animation_set_immutable(Animation *animation) {
  if (!animation) {
    return false;
  }
  ((AnimationPrivate*)animation)->immutable = true;
  return true;
}

bool WEAK animation_is_immutable(Animation *animation) {
  if (!animation) {
    return false;
  }
  return ((AnimationPrivate*)animation)->immutable;
}

bool WEAK animation_set_reverse(Animation *animation, bool reverse) {
  if (!animation) {
    return false;
  }
  ((AnimationPrivate*)animation)->reverse = reverse;
  return true;
}

bool WEAK animation_get_reverse(Animation *animation) {
  if (!animation) {
    return false;
  }

  return ((AnimationPrivate*)animation)->reverse;
}

bool WEAK animation_set_play_count(Animation *animation, uint32_t play_count) {
  if (!animation) {
    return false;
  }
  ((AnimationPrivate*)animation)->play_count = play_count;
  return true;
}

bool WEAK animation_set_duration(Animation *animation, uint32_t duration_ms) {
  if (!animation) {
    return false;
  }
  ((AnimationPrivate*)animation)->duration_ms = duration_ms;
  return true;
}

uint32_t WEAK animation_get_duration(Animation *animation, bool include_delay, bool include_play_count) {
  if (!animation) {
    return 0;
  }
  return ((AnimationPrivate*)animation)->duration_ms;
}

bool WEAK animation_set_curve(Animation *animation, AnimationCurve curve) { return true; }

bool WEAK animation_set_custom_curve(Animation *animation,
                                     AnimationCurveFunction curve_function) {
  return true;
}

bool WEAK animation_set_custom_interpolation(Animation *animation_h,
                                             InterpolateInt64Function interpolate_function) {
  return true;
}

bool WEAK animation_set_handlers(Animation *animation, AnimationHandlers callbacks,
                                 void *context) {
  if (!animation) {
    return false;
  }
  ((AnimationPrivate*)animation)->context = context;
  return true;
}

void *WEAK animation_get_context(Animation *animation) {
  if (!animation) {
    return false;
  }
  return ((AnimationPrivate*)animation)->context;
}

bool WEAK animation_schedule(Animation *animation) {
  if (!animation) {
    return false;
  }
  ((AnimationPrivate*)animation)->abs_start_time_ms = 0;
  ((AnimationPrivate*)animation)->scheduled = true;
  return true;
}

bool WEAK animation_unschedule(Animation *animation) {
  if (!animation) {
    return false;
  }
  ((AnimationPrivate*)animation)->scheduled = false;
  return true;
}

void WEAK animation_unschedule_all(void) {}

bool WEAK animation_set_implementation(Animation *animation,
                                       const AnimationImplementation *implementation) {
  return true;
}

bool WEAK animation_is_scheduled(Animation *animation) {

  return animation ? ((AnimationPrivate*)animation)->scheduled : false;
}

Animation *WEAK animation_sequence_create(Animation *animation_a, Animation *animation_b,
                                          Animation *animation_c, ...) {
  return animation_create();
}

Animation *WEAK animation_sequence_create_from_array(Animation **animation_array,
                                                     uint32_t array_len) {
  return animation_create();
}

Animation *WEAK animation_spawn_create(Animation *animation_a, Animation *animation_b,
                                       Animation *animation_c, ...) {
  return animation_create();
}

Animation *WEAK animation_spawn_create_from_array(Animation **animation_array,
                                                  uint32_t array_len) {
  return animation_create();
}

bool WEAK animation_set_auto_destroy(Animation *animation, bool auto_destroy) {
  return false;
}

PropertyAnimation *WEAK property_animation_create_layer_frame(
    struct Layer *layer, GRect *from_frame, GRect *to_frame) {
  return property_animation_create(NULL, layer, from_frame, to_frame);
}

PropertyAnimation *WEAK property_animation_create(
    const PropertyAnimationImplementation *implementation, void *subject, void *from_value,
    void *to_value) {
  PropertyAnimationPrivate *animation = malloc(sizeof(PropertyAnimationPrivate));
  property_animation_init((PropertyAnimation *)animation, implementation, subject, from_value,
                          to_value);
  return (PropertyAnimation *)animation;
}

void WEAK property_animation_destroy(PropertyAnimation* property_animation_h) {
  animation_destroy((Animation *)property_animation_h);
}

bool WEAK property_animation_init(PropertyAnimation *animation,
                                  const PropertyAnimationImplementation *implementation,
                                  void *subject, void *from_value, void *to_value) {
  if (!animation) {
    return false;
  }
  *(PropertyAnimationPrivate *)animation = (PropertyAnimationPrivate){
      .animation.implementation = (const AnimationImplementation *)implementation,
      .subject = subject,
  };
  return true;
}

bool WEAK property_animation_subject(PropertyAnimation *property_animation, void **subject,
                                     bool set) {
  if (!property_animation) {
    return false;
  }
  if (set) {
    ((PropertyAnimationPrivate *)property_animation)->subject = *subject;
  } else {
    *subject = ((PropertyAnimationPrivate *)property_animation)->subject;
  }
  return true;
}

bool WEAK property_animation_to(PropertyAnimation *property_animation, void *to, size_t size,
                                bool set) {
  if (!property_animation) {
    return false;
  }
  void *field = &((PropertyAnimationPrivate *)property_animation)->values.to;
  memcpy((set ? field : to), (set ? to : field), size);
  return true;
}

void WEAK property_animation_update_gpoint(PropertyAnimation *property_animation,
                                           const uint32_t distance_normalized) {}

Animation *WEAK property_animation_get_animation(PropertyAnimation *property_animation) {
  return (Animation *)property_animation;
}

InterpolateInt64Function WEAK animation_private_current_interpolate_override(void) { return NULL; }

void WEAK property_animation_update_int16(PropertyAnimation *property_animation,
                                          const uint32_t distance_normalized) {}
