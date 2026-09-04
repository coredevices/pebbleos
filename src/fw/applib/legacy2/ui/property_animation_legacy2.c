/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "property_animation_legacy2.h"
#include "applib/ui/property_animation.h"

#include "applib/ui/layer.h"

#include "kernel/pbl_malloc.h"
#include <pbl/logging/logging.h>

/////////////////////
// Property Animation
//

static inline int16_t distance_interpolate(const int32_t normalized,
                                           int16_t from, int16_t to) {
  return from + ((normalized * (to - from)) / ANIMATION_NORMALIZED_MAX);
}

void property_animation_legacy2_update_int16(PropertyAnimationLegacy2 *property_animation,
                                     const uint32_t distance_normalized) {
  int16_t result = distance_interpolate(distance_normalized,
                                        property_animation->values.from.int16,
                                        property_animation->values.to.int16);
  ((PropertyAnimationLegacy2Implementation*)property_animation->animation.implementation)
              ->accessors.setter.int16(property_animation->subject, result);
}

void property_animation_legacy2_update_gpoint(PropertyAnimationLegacy2 *property_animation,
                                      const uint32_t distance_normalized) {
  GPoint result;
  result.x = distance_interpolate(distance_normalized,
                                  property_animation->values.from.gpoint.x,
                                  property_animation->values.to.gpoint.x);
  result.y = distance_interpolate(distance_normalized,
                                  property_animation->values.from.gpoint.y,
                                  property_animation->values.to.gpoint.y);
  ((PropertyAnimationLegacy2Implementation*)
   property_animation->animation.implementation)
    ->accessors.setter.gpoint(property_animation->subject, result);
}

void property_animation_legacy2_update_grect(PropertyAnimationLegacy2 *property_animation,
                                     const uint32_t distance_normalized) {
  GRect result;
  result.origin.x = distance_interpolate(
      distance_normalized,
      property_animation->values.from.grect.origin.x,
      property_animation->values.to.grect.origin.x);
  result.origin.y = distance_interpolate(
      distance_normalized,
      property_animation->values.from.grect.origin.y,
      property_animation->values.to.grect.origin.y);
  result.size.w = distance_interpolate(
      distance_normalized,
      property_animation->values.from.grect.size.w,
      property_animation->values.to.grect.size.w);
  result.size.h = distance_interpolate(
      distance_normalized,
      property_animation->values.from.grect.size.h,
      property_animation->values.to.grect.size.h);
  ((PropertyAnimationLegacy2Implementation*)
   property_animation->animation.implementation)
    ->accessors.setter.grect(property_animation->subject, result);
}

void property_animation_legacy2_init(
    PropertyAnimationLegacy2 *property_animation,
    const PropertyAnimationLegacy2Implementation *implementation,
    void *subject, void *from_value, void *to_value) {
  animation_legacy2_init(&property_animation->animation);
  memset(&property_animation->values, 0xff, sizeof(property_animation->values));
  property_animation->animation.implementation =
                                  (AnimationLegacy2Implementation*) implementation;
  property_animation->subject = subject;

  // NOTE: we are also comparing against the 3.0 animation update handlers so that modules
  // like scroll_layer and menu_layer can use the legacy 2.0 animation when interfacing with a
  // 2.x app.
  if (property_animation->animation.implementation->update
        == (AnimationLegacy2UpdateImplementation) property_animation_legacy2_update_int16
      || property_animation->animation.implementation->update
        == (AnimationLegacy2UpdateImplementation) property_animation_update_int16) {
    property_animation->values.to.int16 = to_value ? *((int16_t*)to_value)
                                        : implementation->accessors.getter.int16(subject);
    property_animation->values.from.int16 = from_value ? *((int16_t*)from_value)
                                        : implementation->accessors.getter.int16(subject);
  } else if (property_animation->animation.implementation->update
               == (AnimationLegacy2UpdateImplementation) property_animation_legacy2_update_gpoint
          || property_animation->animation.implementation->update
               == (AnimationLegacy2UpdateImplementation) property_animation_update_gpoint) {
    property_animation->values.to.gpoint = to_value ? *((GPoint*)to_value)
                                         : implementation->accessors.getter.gpoint(subject);
    property_animation->values.from.gpoint =
        from_value ? *((GPoint*)from_value) : implementation->accessors.getter.gpoint(subject);
  } else if (property_animation->animation.implementation->update
               == (AnimationLegacy2UpdateImplementation) property_animation_legacy2_update_grect
          || property_animation->animation.implementation->update
             == (AnimationLegacy2UpdateImplementation) property_animation_update_grect) {
    property_animation->values.to.grect = to_value ? *((GRect*)to_value)
                                        : implementation->accessors.getter.grect(subject);
    property_animation->values.from.grect = from_value ? *((GRect*)from_value)
                                          : implementation->accessors.getter.grect(subject);
  }
}

struct PropertyAnimationLegacy2* property_animation_legacy2_create(
    const struct PropertyAnimationLegacy2Implementation *implementation,
    void *subject, void *from_value, void *to_value) {
  struct PropertyAnimationLegacy2* property_animation =
      task_malloc(sizeof(struct PropertyAnimationLegacy2));
  if (property_animation) {
    property_animation_legacy2_init(property_animation, implementation, subject,
                            from_value, to_value);
  }
  return property_animation;
}

void property_animation_legacy2_destroy(struct PropertyAnimationLegacy2* property_animation) {
  if (property_animation == NULL) {
    return;
  }
  animation_legacy2_unschedule(&property_animation->animation);
  task_free(property_animation);
}

void property_animation_legacy2_init_layer_frame(
    PropertyAnimationLegacy2 *property_animation, struct Layer *layer,
    GRect *from_frame, GRect *to_frame) {
  static const PropertyAnimationLegacy2Implementation implementation = {
    .base = {
      .update = (AnimationLegacy2UpdateImplementation) property_animation_legacy2_update_grect,
    },
    .accessors = {
      .setter = { .grect = (const GRectSetter) layer_set_frame_by_value, },
      .getter = { .grect = (const GRectGetter) layer_get_frame_by_value, },
    },
  };
  property_animation_legacy2_init(property_animation, &implementation, layer,
                          from_frame, to_frame);
}

struct PropertyAnimationLegacy2* property_animation_legacy2_create_layer_frame(
    struct Layer *layer, GRect *from_frame, GRect *to_frame) {
  struct PropertyAnimationLegacy2* property_animation =
      task_malloc(sizeof(struct PropertyAnimationLegacy2));
  if (property_animation) {
    property_animation_legacy2_init_layer_frame(property_animation, layer, from_frame,
                                        to_frame);
  }
  return property_animation;
}

