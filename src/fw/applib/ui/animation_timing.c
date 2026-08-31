/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "animation_timing.h"

#include "system/passert.h"
#include "pbl/util/math_fixed.h"
#include "pbl/util/size.h"

//! @file animation_timing.c

static const uint16_t s_ease_in_table[] = {
  0, 64, 256, 576,
  1024, 1600, 2304, 3136,
  4096, 5184, 6400, 7744,
  9216, 10816, 12544, 14400,
  16384, 18496, 20736, 23104,
  25600, 28224, 30976, 33856,
  36864, 40000, 43264, 46656,
  50176, 53824, 57600, 61504,
  65535
};

static const uint16_t s_ease_out_table[] = {
  0, 4031, 7935, 11711,
  15359, 18879, 22271, 25535,
  28671, 31679, 34559, 37311,
  39935, 42431, 44799, 47039,
  49151, 51135, 52991, 54719,
  56319, 57791, 59135, 60351,
  61439, 62399, 63231, 63935,
  64511, 64959, 65279, 65471,
  65535
};

static const uint16_t s_ease_in_out_table[] = {
  0, 128, 512, 1152,
  2048, 3200, 4608, 6272,
  8192, 10368, 12800, 15488,
  18432, 21632, 25088, 28800,
  32770, 36737, 40449, 43905,
  47105, 50049, 52737, 55169,
  57345, 59265, 60929, 62337,
  63488, 64384, 65024, 65408,
  65535
};

typedef struct {
  const size_t num_entries;
  const uint16_t *table;
} EasingTable;

static const EasingTable s_easing_tables[] = {
  [AnimationCurveEaseIn] = { ARRAY_LENGTH(s_ease_in_table), s_ease_in_table },
  [AnimationCurveEaseOut] = { ARRAY_LENGTH(s_ease_out_table), s_ease_out_table },
  [AnimationCurveEaseInOut] = { ARRAY_LENGTH(s_ease_in_out_table), s_ease_in_out_table },
};

int32_t animation_timing_segmented(int32_t time_normalized, int32_t index,
    uint32_t num_segments, Fixed_S32_16 duration_fraction) {
  PBL_ASSERTN(num_segments > 0 && duration_fraction.raw_value > 0);
  if (index < 0) {
    return ANIMATION_NORMALIZED_MAX;
  }
  if ((uint32_t)index >= num_segments) {
    return 0;
  }

  const int32_t duration_per_item = ((int64_t) ANIMATION_NORMALIZED_MAX
      * duration_fraction.raw_value) / FIXED_S32_16_ONE.raw_value;
  const int32_t delay_per_item = (ANIMATION_NORMALIZED_MAX - duration_per_item) / num_segments;
  const int32_t normalized_offset = time_normalized - index * delay_per_item;
  if (normalized_offset < 0) {
    return 0;
  }
  const int32_t relative_progress = ((int64_t) normalized_offset
      * FIXED_S32_16_ONE.raw_value) / duration_fraction.raw_value;
  if (relative_progress > ANIMATION_NORMALIZED_MAX) {
    return ANIMATION_NORMALIZED_MAX;
  }
  return relative_progress;
}

typedef int64_t (*ArrayAccessorInt64)(const void *array, size_t index);

static int64_t prv_uint16_getter(const void *array, size_t idx) {
  return ((uint16_t*)array)[idx];
}

static int64_t prv_int32_getter(const void *array, size_t idx) {
  return ((int32_t*)array)[idx];
}

AnimationProgress prv_animation_timing_interpolate(AnimationProgress progress, const void *array,
                                                   ArrayAccessorInt64 getter, size_t num_entries) {
  PBL_ASSERTN(num_entries > 0);

  const size_t max_entry = num_entries - 1;
  if (progress <= ANIMATION_NORMALIZED_MIN) {
    return (AnimationProgress) getter(array, 0);
  }
  if (progress >= ANIMATION_NORMALIZED_MAX) {
    return (AnimationProgress) getter(array, max_entry);
  }

  // Linear interpolate from the easing table.
  const size_t stride = ANIMATION_NORMALIZED_MAX / max_entry;
  const size_t index = (progress * max_entry) / ANIMATION_NORMALIZED_MAX;
  const int64_t from = getter(array, index);
  const int64_t delta = getter(array, index + 1) - from;
  return (AnimationProgress) (from + (delta * (progress - index * stride)) / stride);
}

AnimationProgress animation_timing_interpolate(
    AnimationProgress time_normalized, const uint16_t *table, size_t num_entries) {
  return prv_animation_timing_interpolate(time_normalized, table, prv_uint16_getter, num_entries);
}

AnimationProgress animation_timing_interpolate32(
    AnimationProgress time_normalized, const int32_t *table, size_t num_entries) {
  return prv_animation_timing_interpolate(time_normalized, table, prv_int32_getter, num_entries);
}

AnimationProgress animation_timing_curve(AnimationProgress time_normalized,
                                         AnimationCurve curve) {
  switch (curve) {
    case AnimationCurveEaseIn:
    case AnimationCurveEaseOut:
    case AnimationCurveEaseInOut: {
      const EasingTable *easing = &s_easing_tables[curve];
      return animation_timing_interpolate(time_normalized, easing->table, easing->num_entries);
    }
    case AnimationCurveLinear:
    default:
      return time_normalized;
  }
}

AnimationProgress animation_timing_scaled(AnimationProgress time_normalized,
                                          AnimationProgress interval_start,
                                          AnimationProgress interval_end) {
  int64_t result = time_normalized - interval_start;
  result = (result * ANIMATION_NORMALIZED_MAX) / (interval_end - interval_start);
  // no overflow worries here, result and ANIMATION_NORMALIZED_MAX are both <= 2^16.
  return (AnimationProgress)result;
}
