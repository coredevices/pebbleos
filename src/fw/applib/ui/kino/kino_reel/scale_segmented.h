/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/graphics/gdraw_command_transforms.h"
#include "applib/ui/kino/kino_reel.h"

// These are KinoReels that use the per-point segmented delayed scaling animation.
// @see gdraw_command_list_scale_segmented_to.

#define SCALE_SEGMENTED_DEFAULT_POINT_DURATION \
  Fixed_S32_16(2 * FIXED_S32_16_ONE.raw_value / 3)

#define SCALE_SEGMENTED_DEFAULT_EFFECT_DURATION \
  Fixed_S32_16(2 * FIXED_S32_16_ONE.raw_value / 3)

#define SCALE_SEGMENTED_DOT_SIZE_PX 0
#define SCALE_SEGMENTED_DOT_SIZE GSize(SCALE_SEGMENTED_DOT_SIZE_PX, SCALE_SEGMENTED_DOT_SIZE_PX)

//! A GDelayCreatorContext gives the information needed to build a delay index lookup for a given
//! GDrawCommandList.
typedef struct {
  GDrawCommandList * const list;
  const GSize size;
  //! Whether the transform should free the lookup after use.
  //! Specifying false allows the creator to reuse buffers or references existing lookups.
  bool owns_lookup;
} GDelayCreatorContext;

//! GPointIndexLookup creator which is passed a GDelayCreatorContext.
//! @param ctx GDelayCreatorContext with the information needed to build the delay index lookup
//! @param userdata User supplied data for delay index lookup specific data.
typedef GPointIndexLookup *(*GPointIndexLookupCreator)(GDelayCreatorContext *ctx, void *userdata);

//! A KinoReel that can perform a one-stage or two-stage scale and translate with or
//! without a deflation and bounce back effect defined by a custom \ref GPointIndexLookup.
//! The effects can be simultaneous or independent and are achieved by overlapping two
//! invocations of \ref gdraw_command_list_scale_segmented_to.
//! If looking for a stretching effect, consider using the other factory functions available.
//! @param from_reel KinoReel to display and animate.
//! @param take_ownership true if this KinoReel will free `image` when destroyed.
//! @param screen_frame Position and size of the parent KinoLayer in global coordinates.
//! @return a scale segmented KinoReel
//! @see gdraw_command_list_scale_segmented_to, kino_reel_dci_transform_create, GPointIndexLookup
KinoReel *kino_reel_scale_segmented_create(KinoReel *from_reel, bool take_ownership,
                                           GRect screen_frame);

//! Sets a GPointIndexLookup
//! @param reel KinoReel to modify
//! @param creator Creator of a GPointIndexLookup with the assigned delay multiplier for each point
//! @param context Context to pass to the creator
//! @param take_ownership true if this KinoReel will free `context` when destroyed
void kino_reel_scale_segmented_set_delay_lookup_creator(
    KinoReel *reel, GPointIndexLookupCreator creator, void *context, bool take_ownership);

//! Sets a GPointIndexLookup based on the distance to a target
//! @param reel KinoReel to modify
//! @param target Position to pull and stretch the image from in image coordinates.
bool kino_reel_scale_segmented_set_delay_by_distance(KinoReel *reel, GPoint target);

//! Set the point duration
//! @param reel KinoReel to modify
//! @param point_duration Fraction of the total animation time a point should animate.
void kino_reel_scale_segmented_set_point_duration(KinoReel *reel, Fixed_S32_16 point_duration);

//! Set the effect duration. Ignored if expand and bounce are disabled
//! @param reel KinoReel to modify
//! @param point_duration Fraction of the total animation time an effect should animate.
void kino_reel_scale_segmented_set_effect_duration(KinoReel *reel, Fixed_S32_16 point_duration);

//! Set the magnitude of the deflate effect
//! @param reel KinoReel to modify
//! @param expand Expansion length of `to` in pixels that would result in a deflation animation.
//! Set as 0 to disable.
void kino_reel_scale_segmented_set_deflate_effect(KinoReel *reel, int16_t expand);

//! Set the magnitude of the bounce back effect. Requires all frames to be set before use.
//! @param reel KinoReel to modify
//! @param bounce Overshoot length of `to` in pixels that would result in a bounce back animation.
//! Set as 0 to disable.
void kino_reel_scale_segmented_set_bounce_effect(KinoReel *reel, int16_t bounce);

//! Set the animation interpolation
void kino_reel_scale_segmented_set_interpolate(KinoReel *reel,
                                               InterpolateInt64Function interpolate);

//! Set from stroke width
//! @param reel KinoReel to modify
//! @param from Fixed_S16_3 From stroke width operator value
//! @param from_op GStrokeWidthOp operation to start with
//! @see GStrokeWidthOp
void kino_reel_scale_segmented_set_from_stroke_width(KinoReel *reel, Fixed_S16_3 from,
                                                     GStrokeWidthOp from_op);

//! Set to stroke width
//! @param reel KinoReel to modify
//! @param to Fixed_S16_3 To stroke width operator value
//! @param to_op GStrokeWidthOp operation to end with
//! @see GStrokeWidthOp
void kino_reel_scale_segmented_set_to_stroke_width(KinoReel *reel, Fixed_S16_3 to,
                                                   GStrokeWidthOp to_op);

//! Set the stroke width curve
void kino_reel_scale_segmented_set_stroke_width_curve(KinoReel *reel,
                                                      AnimationCurveFunction curve);

//! Set the animation to end as a dot. Requires the to frame to be set before use.
void kino_reel_scale_segmented_set_end_as_dot(KinoReel *reel, int16_t radius);
