/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "transform.h"
#include "scale_segmented.h"
#include "unfold.h"

#include "applib/applib_malloc.auto.h"
#include "pbl/util/trig.h"
#include "applib/graphics/gdraw_command_private.h"
#include "syscall/syscall.h"

static AnimationProgress prv_ease_in_out_first_quarter(AnimationProgress progress) {
  return animation_timing_curve(animation_timing_clip(4 * progress), AnimationCurveEaseInOut);
}

typedef struct {
  int32_t angle;
  int num_delay_groups;
  Fixed_S32_16 group_delay;
} AngleLookupContext;

static GPointIndexLookup *prv_create_lookup_by_angle(GDelayCreatorContext *ctx, void *userdata) {
  AngleLookupContext *data = userdata;

  GPoint origin = { ctx->size.w / 2, ctx->size.h / 2 };
  GPointIndexLookup *lookup = gdraw_command_list_create_index_lookup_by_angle(ctx->list, origin,
                                                                              data->angle);
  gpoint_index_lookup_set_groups(lookup, data->num_delay_groups, data->group_delay);

  ctx->owns_lookup = true;
  return lookup;
}

KinoReel *kino_reel_unfold_create(KinoReel *from_reel, bool take_ownership, GRect screen_frame,
                                  int32_t angle, int num_delay_groups, Fixed_S32_16 group_delay) {
  GDrawCommandList *list = kino_reel_get_gdraw_command_list(from_reel);
  if (!list) {
    return from_reel;
  }

  AngleLookupContext *ctx = applib_malloc(sizeof(AngleLookupContext));
  if (!ctx) {
    return NULL;
  }
  *ctx = (AngleLookupContext) {
    .angle = angle,
    .num_delay_groups = num_delay_groups,
    .group_delay = group_delay,
  };
  if (!ctx->angle) {
    ctx->angle = rand() % TRIG_MAX_ANGLE;
  }

  KinoReel *reel = kino_reel_scale_segmented_create(from_reel, take_ownership, screen_frame);
  if (reel) {
    const bool take_ownership = true;
    kino_reel_scale_segmented_set_delay_lookup_creator(reel, prv_create_lookup_by_angle, ctx,
                                                       take_ownership);
    kino_reel_scale_segmented_set_point_duration(reel, UNFOLD_DEFAULT_POINT_DURATION);
    kino_reel_scale_segmented_set_effect_duration(reel, UNFOLD_DEFAULT_EFFECT_DURATION);
  }
  return reel;
}

void kino_reel_unfold_set_start_as_dot(KinoReel *reel, int16_t radius) {
  GRect frame = kino_reel_transform_get_from_frame(reel);
  kino_reel_transform_set_from_frame(
      reel, (GRect) { grect_center_point(&frame), UNFOLD_DOT_SIZE });

  const Fixed_S16_3 from = Fixed_S16_3((2 * radius) << FIXED_S16_3_PRECISION);
  kino_reel_scale_segmented_set_from_stroke_width(reel, from, GStrokeWidthOpSet);
  kino_reel_scale_segmented_set_stroke_width_curve(reel, prv_ease_in_out_first_quarter);
}
