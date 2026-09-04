/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "gdraw_command_transforms.h"

#include "applib/applib_malloc.auto.h"
#include "applib/graphics/gdraw_command_private.h"
#include "pbl/util/trig.h"
#include "applib/ui/animation.h"
#include "applib/ui/animation_interpolate.h"
#include "applib/ui/animation_timing.h"
#include "system/passert.h"
#include "pbl/util/math_fixed.h"

////////////////////
// scale

typedef struct {
  GSize from;
  GSize to;
} ScaleCBContext;

T_STATIC bool prv_gdraw_command_scale(GDrawCommand *command, uint32_t index, void *context) {
  ScaleCBContext *scale = context;
  const uint16_t num_points = gdraw_command_get_num_points(command);
  for (uint16_t i = 0; i < num_points; i++) {
    command->points[i] = gpoint_scale_by_gsize(command->points[i], scale->from, scale->to);
  }
  return true;
}

void gdraw_command_list_scale(GDrawCommandList *list, GSize from, GSize to) {
  ScaleCBContext ctx = {
    .from = from,
    .to = to,
  };
  gdraw_command_list_iterate(list, prv_gdraw_command_scale, &ctx);
}

void gdraw_command_image_scale(GDrawCommandImage *image, GSize to) {
  gdraw_command_list_scale(&image->command_list, image->size, to);
  image->size = to;
}


////////////////////
// attract to square

static int16_t prv_int_attract_to(int16_t value, int16_t bounds, int32_t normalized) {
  const int16_t delta_0 = (int16_t) ((0 + 1) - value);
  const int16_t delta_b = (int16_t) ((bounds - 1) - value);
  const int16_t delta = ABS(delta_0) < ABS(delta_b) ? delta_0 : delta_b;

  return (int16_t) (value + delta * normalized / ANIMATION_NORMALIZED_MAX);
}

GPoint gpoint_attract_to_square(GPoint point, GSize size, int32_t normalized) {
  // hacky to square - TODO: implement for real
  point.y += 1;
  point = GPoint(
      prv_int_attract_to(point.x, size.w, normalized),
      prv_int_attract_to(point.y, size.h, normalized));
  return point;
}

typedef struct {
  GSize integer_size;
  GSize precise_size;
  int32_t normalized;
} ToSquareCBContext;

T_STATIC bool prv_gdraw_command_attract_to_square(GDrawCommand *command, uint32_t index,
                                                  void *context) {
  ToSquareCBContext *to_square = context;
  const uint16_t num_points = gdraw_command_get_num_points(command);
  for (uint16_t i = 0; i < num_points; i++) {
    const GSize size = (command->type == GDrawCommandTypePrecisePath)
        ? to_square->precise_size : to_square->integer_size;
    command->points[i] = gpoint_attract_to_square(command->points[i],
                                                  size, to_square->normalized);
  }
  return true;
}

void gdraw_command_list_attract_to_square(GDrawCommandList *list, GSize size, int32_t normalized) {
  ToSquareCBContext ctx = {
    .integer_size = size,
    .precise_size = gsize_scalar_lshift(size, GPOINT_PRECISE_PRECISION),
    .normalized = normalized,
  };
  gdraw_command_list_iterate(list, prv_gdraw_command_attract_to_square, &ctx);
}

void gdraw_command_image_attract_to_square(GDrawCommandImage *image, int32_t normalized) {
  gdraw_command_list_attract_to_square(&image->command_list, image->size, normalized);
}


////////////////////
// gpoint index lookup creator

typedef struct {
  const struct {
    const GPoint *points;
    uint16_t num_points;
  } values;
  struct {
    GPointIndexLookup *lookup;
    uint32_t current_index;
  } iter;
} GPointCreateIndexCBContext;

T_STATIC bool prv_gdraw_command_create_point_index_lookup(GDrawCommand *command, uint32_t index,
                                                          void *context) {
  GPointCreateIndexCBContext *lookup = context;
  const uint16_t num_points = gdraw_command_get_num_points(command);
  for (uint16_t i = 0; i < num_points; i++) {
    GPoint point = command->points[i];

    if (command->type == GDrawCommandTypePrecisePath) {
      point = gpoint_scalar_rshift(point, GPOINT_PRECISE_PRECISION);
    }

    const uint32_t lookup_length = lookup->values.num_points;
    for (uint16_t j = 0; j < lookup_length; j++) {
      if (gpoint_equal(&point, &lookup->values.points[j])) {
        lookup->iter.lookup->index_lookup[lookup->iter.current_index] = j;
        break;
      }
    }
    lookup->iter.current_index++;
  }
  return true;
}

GPointIndexLookup *gdraw_command_list_create_index_lookup(GDrawCommandList *list,
    GPointComparator comparator, void *context, bool reverse) {
  uint16_t num_points = 0;
  const bool is_precise = false;
  GPoint * const points = gdraw_command_list_collect_points(list, is_precise, &num_points);
  if (!points) {
    return NULL;
  }

  gpoint_sort(points, num_points, comparator, context, reverse);

  GPointIndexLookup *lookup = applib_malloc(sizeof(GPointIndexLookup)
      + num_points * sizeof(uint16_t));
  if (!lookup) {
    applib_free(points);
    return lookup;
  }

  lookup->num_points = num_points;
  lookup->max_index = num_points - 1;

  GPointCreateIndexCBContext ctx = {
    .values = {
      .points = points,
      .num_points = num_points,
    },
    .iter = {
      .lookup = lookup,
    },
  };
  gdraw_command_list_iterate(list, prv_gdraw_command_create_point_index_lookup, &ctx);

  applib_free(points);
  return lookup;
}

typedef struct {
  const GPoint origin;
  const int32_t angle;
} AngleComparatorContext;

static int prv_angle_comparator(const GPoint * const a, const GPoint * const b,
    void *context) {
  AngleComparatorContext *ctx = context;
  const int16_t angle_a = ABS(positive_modulo(
      (atan2_lookup(a->y - ctx->origin.y, a->x - ctx->origin.x) + ctx->angle), TRIG_MAX_ANGLE) -
      TRIG_MAX_ANGLE / 2);
  const int16_t angle_b = ABS(positive_modulo(
      (atan2_lookup(b->y - ctx->origin.y, b->x - ctx->origin.x) + ctx->angle), TRIG_MAX_ANGLE) -
      TRIG_MAX_ANGLE / 2);
  return (angle_a > angle_b ? 1 : -1);
}

GPointIndexLookup *gdraw_command_list_create_index_lookup_by_angle(GDrawCommandList *list,
                                                                   GPoint origin, int32_t angle) {
  AngleComparatorContext ctx = {
    .origin = origin,
    .angle = angle,
  };
  return gdraw_command_list_create_index_lookup(list, prv_angle_comparator, &ctx, false);
}

static int prv_distance_comparator(const GPoint * const a, const GPoint * const b,
    void *context) {
  const GPoint * const target = context;
  uint32_t distance_a = gpoint_distance_squared(*a, *target);
  uint32_t distance_b = gpoint_distance_squared(*b, *target);
  return (distance_a > distance_b ? 1 : -1);
}

GPointIndexLookup *gdraw_command_list_create_index_lookup_by_distance(GDrawCommandList *list,
                                                                      GPoint target) {
  return gdraw_command_list_create_index_lookup(list, prv_distance_comparator, &target, false);
}

void gpoint_index_lookup_add_at(GPointIndexLookup *lookup, int delay_index, int delay_amount) {
  if (delay_index < 0 || delay_index >= lookup->max_index) {
    return;
  }
  // We are adding additional delay, the max delay index increases
  lookup->max_index += delay_amount;
  for (int i = 0; i < lookup->num_points; i++) {
    // The lookup maps definition index => delay index
    // We want to add delay to points at or above a certain delay index
    if (lookup->index_lookup[i] >= delay_index) {
      lookup->index_lookup[i] += delay_amount;
    }
  }
}

void gpoint_index_lookup_set_groups(GPointIndexLookup *lookup, int num_groups,
                                    Fixed_S32_16 group_delay) {
  const int num_points_per_group = lookup->num_points / num_groups;
  const int delay_per_group = (num_points_per_group / group_delay.raw_value) /
                              FIXED_S32_16_ONE.raw_value;
  const int group_delay_amount = num_points_per_group + delay_per_group;
  for (uint16_t i = num_groups - 1; i >= 1; i--) {
    gpoint_index_lookup_add_at(lookup, (i * num_points_per_group), group_delay_amount);
  }
}

////////////////////
// segmented scale: index based segmentation of scale + transform

static int16_t prv_int_scale_to(
    int16_t value, int16_t size, int16_t from_range, int16_t to_range, int32_t normalized,
    InterpolateInt64Function interpolate) {
  return value + ((int32_t) value * interpolate(
          normalized, from_range - size, to_range - size)) / size;
}

T_STATIC int16_t prv_int_scale_and_translate_to(
    int16_t value, int16_t size, int16_t from_range, int16_t to_range,
    int16_t from_min, int16_t to_min, int32_t normalized, InterpolateInt64Function interpolate) {
  const int32_t scale = prv_int_scale_to(value, size, from_range, to_range, normalized,
                                         interpolate);
  const int32_t translate = interpolate(normalized, from_min, to_min);
  return scale + translate;
}

GPoint gpoint_scale_to(GPoint point, GSize size, GRect from, GRect to, int32_t normalized,
                       InterpolateInt64Function interpolate) {
  return GPoint(
      prv_int_scale_and_translate_to(point.x, size.w, from.size.w, to.size.w,
                       from.origin.x, to.origin.x, normalized, interpolate),
      prv_int_scale_and_translate_to(point.y, size.h, from.size.h, to.size.h,
                       from.origin.y, to.origin.y, normalized, interpolate));
}

typedef struct {
  GRect from;
  GRect to;
  GSize size;
  GPoint offset;
} ScaleToGValues;

typedef struct {
  const struct {
    ScaleToGValues integer;
    ScaleToGValues precise;

    Fixed_S32_16 duration_fraction;
    GPointIndexLookup *lookup;
    AnimationProgress normalized;
    InterpolateInt64Function interpolate;

    bool is_offset;
  } values;
  struct {
    uint32_t current_index;
  } iter;
} ScaleToCBContext;

T_STATIC int64_t prv_default_interpolate(int32_t normalized, int64_t from, int64_t to) {
  const int32_t curved = animation_timing_curve(normalized, AnimationCurveEaseInOut);
  return interpolate_int64_linear(curved, from, to);
}

T_STATIC bool prv_gdraw_command_scale_segmented(GDrawCommand *command, uint32_t index,
                                                void *context) {
  ScaleToCBContext *scale = context;
  const ScaleToGValues * const gvalues = (command->type == GDrawCommandTypePrecisePath)
      ? &scale->values.precise : &scale->values.integer;

  const uint16_t num_points = gdraw_command_get_num_points(command);
  for (uint16_t i = 0; i < num_points; i++) {
    const int32_t point_index = scale->values.lookup->index_lookup[scale->iter.current_index];
    GPoint point = command->points[i];

    if (scale->values.is_offset) {
      gpoint_sub_eq(&point, gvalues->offset);
    }

    const AnimationProgress normalized = animation_timing_segmented(
        scale->values.normalized, point_index, scale->values.lookup->max_index + 1,
        scale->values.duration_fraction);

    const InterpolateInt64Function interpolate = scale->values.interpolate ?
        scale->values.interpolate : prv_default_interpolate;

    point = gpoint_scale_to(point, gvalues->size, gvalues->from, gvalues->to, normalized,
                            interpolate);

    if (scale->values.is_offset) {
      gpoint_add_eq(&point, gvalues->offset);
    }

    command->points[i] = point;

    scale->iter.current_index++;
  }
  return true;
}

void gdraw_command_list_scale_segmented_to(
    GDrawCommandList *list, GSize size, GRect from, GRect to, AnimationProgress normalized,
    InterpolateInt64Function interpolate, GPointIndexLookup *lookup, Fixed_S32_16 duration_fraction,
    bool is_offset) {
  GPoint offset = GPointZero;
  if (is_offset) {
    offset = from.origin;
    to.origin = gpoint_sub(to.origin, from.origin);
    from.origin = GPointZero;
  }

  ScaleToCBContext ctx = {
    .values = {
      .integer = {
        .from = from,
        .to = to,
        .size = size,
        .offset = offset,
      },
      .precise = {
        .from = grect_scalar_lshift(from, GPOINT_PRECISE_PRECISION),
        .to = grect_scalar_lshift(to, GPOINT_PRECISE_PRECISION),
        .size = gsize_scalar_lshift(size, GPOINT_PRECISE_PRECISION),
        .offset = gpoint_scalar_lshift(offset, GPOINT_PRECISE_PRECISION),
      },
      .duration_fraction = duration_fraction,
      .lookup = lookup,
      .normalized = normalized,
      .interpolate = interpolate,
      .is_offset = is_offset,
    },
  };
  gdraw_command_list_iterate(list, prv_gdraw_command_scale_segmented, &ctx);
}

void gdraw_command_image_scale_segmented_to(
    GDrawCommandImage *image, GRect from, GRect to, AnimationProgress normalized,
    InterpolateInt64Function interpolate, GPointIndexLookup *lookup, Fixed_S32_16 duration_fraction,
    bool is_offset) {
  gdraw_command_list_scale_segmented_to(
      &image->command_list, image->size, from, to, normalized, interpolate, lookup,
      duration_fraction, is_offset);
  image->size = to.size;
}


////////////////////
// scale stroke width

typedef struct {
  Fixed_S16_3 from;
  Fixed_S16_3 to;

  AnimationProgress progress;

  GStrokeWidthOp from_op;
  GStrokeWidthOp to_op;
} ScaleStrokeWidthCBContext;

Fixed_S16_3 prv_stroke_width_transform(Fixed_S16_3 native, Fixed_S16_3 op_value,
                                       GStrokeWidthOp op) {
  switch (op) {
    case GStrokeWidthOpSet:
      return op_value;
    case GStrokeWidthOpMultiply:
      return Fixed_S16_3_mul(native, op_value);
    case GStrokeWidthOpAdd:
      return Fixed_S16_3_add(native, op_value);
    default:
      WTF;
  }
}

static bool prv_gdraw_command_scale_stroke_width(GDrawCommand *command, uint32_t index,
                                                 void *context) {
  ScaleStrokeWidthCBContext *scale = context;
  const Fixed_S16_3 stroke_width =
      Fixed_S16_3(gdraw_command_get_stroke_width(command) << FIXED_S16_3_PRECISION);

  Fixed_S16_3 from_stroke_width = prv_stroke_width_transform(stroke_width, scale->from,
                                                             scale->from_op);
  Fixed_S16_3 to_stroke_width = prv_stroke_width_transform(stroke_width, scale->to,
                                                           scale->to_op);

  const uint16_t new_stroke_width = interpolate_int64_linear(
      scale->progress, from_stroke_width.raw_value, to_stroke_width.raw_value);
  gdraw_command_set_stroke_width(
      command, ((new_stroke_width + FIXED_S16_3_HALF.raw_value) >> FIXED_S16_3_PRECISION));

  return true;
}

void gdraw_command_list_scale_stroke_width(GDrawCommandList *list, Fixed_S16_3 from, Fixed_S16_3 to,
                                           GStrokeWidthOp from_op, GStrokeWidthOp to_op,
                                           AnimationProgress progress) {
  ScaleStrokeWidthCBContext ctx = {
    .from = from,
    .to = to,
    .from_op = from_op,
    .to_op = to_op,
    .progress = progress,
  };
  gdraw_command_list_iterate(list, prv_gdraw_command_scale_stroke_width, &ctx);
}

void gdraw_command_image_scale_stroke_width(GDrawCommandImage *image, Fixed_S16_3 from,
                                            Fixed_S16_3 to, GStrokeWidthOp from_op,
                                            GStrokeWidthOp to_op, AnimationProgress progress) {
  gdraw_command_list_scale_stroke_width(&image->command_list, from, to, from_op, to_op, progress);
}


////////////////////
// replace color

typedef struct {
  GColor from;
  GColor to;
} ReplaceColorCBContext;

void gdraw_command_replace_color(GDrawCommand *command, GColor from, GColor to) {
  if (gcolor_equal(from, command->fill_color)) {
    command->fill_color = to;
  }
  if (gcolor_equal(from, command->stroke_color)) {
    command->stroke_color = to;
  }
}

bool prv_replace_color(GDrawCommand *command, uint32_t index, void *context) {
  ReplaceColorCBContext *cb_context = context;
  gdraw_command_replace_color(command, cb_context->from, cb_context->to);
  return true;
}

void gdraw_command_list_replace_color(GDrawCommandList *list, GColor from, GColor to) {
  ReplaceColorCBContext context = {
      .from = from,
      .to = to,
  };
  gdraw_command_list_iterate(list, prv_replace_color, &context);
}

void gdraw_command_frame_replace_color(GDrawCommandFrame *frame, GColor from, GColor to) {
  gdraw_command_list_replace_color(&frame->command_list, from, to);
}
