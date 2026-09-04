/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "transform.h"

#include "applib/graphics/gdraw_command_private.h"
#include "applib/ui/animation.h"
#include "applib/ui/animation_interpolate.h"
#include "applib/ui/animation_timing.h"
#include "applib/ui/kino/kino_reel.h"
#include "applib/ui/kino/kino_reel_custom.h"
#include "applib/applib_malloc.auto.h"
#include "syscall/syscall.h"
#include "pbl/util/math.h"
#include "pbl/util/struct.h"

typedef struct {
  GRect layer_frame;
  GRect from;
  GRect to;

  const TransformImpl *impl;
  void *context;

  int32_t normalized;
  uint32_t elapsed;
  uint32_t duration;

  KinoReel *from_reel;
  KinoReel *to_reel;

  GDrawCommandList *list_copy;
  GSize list_copy_size;

  bool owns_from_reel;
  bool owns_to_reel;
  bool global;
} KinoReelTransformData;

static bool prv_is_currently_from(KinoReelTransformData *data) {
  return (!data->to_reel || (data->normalized < ANIMATION_NORMALIZED_MAX / 2));
}

static KinoReel *prv_get_current_reel(KinoReelTransformData *data) {
  return prv_is_currently_from(data) ? data->from_reel : data->to_reel;
}

static GSize prv_get_current_size(KinoReelTransformData *data) {
  return prv_is_currently_from(data) ? data->from.size : data->to.size;
}

static GPoint prv_get_interpolated_origin(KinoReelTransformData *data) {
  const GSize size = prv_get_current_size(data);
  const GPoint offset = interpolate_gpoint(data->normalized, grect_center_point(&data->from),
                                           grect_center_point(&data->to));
  return gpoint_sub(offset, GPoint(size.w / 2, size.h / 2));
}

static bool prv_image_size_eq_rect_size(KinoReelTransformData *data, GRect rect) {
  GSize size = kino_reel_get_size(prv_get_current_reel(data));
  return gsize_equal(&size, &rect.size);
}

static void prv_free_list_copy(KinoReelTransformData *data) {
  applib_free(data->list_copy);
  data->list_copy = NULL;
}

static GDrawCommandList *prv_get_or_create_list_copy(KinoReelTransformData *data,
                                                     GDrawCommandList *list) {
  if (data->list_copy && (gdraw_command_list_get_data_size(data->list_copy) >=
                          gdraw_command_list_get_data_size(list))) {
    return data->list_copy;
  }
  prv_free_list_copy(data);
  data->list_copy = gdraw_command_list_clone(list);
  return data->list_copy;
}

static void prv_destructor(KinoReel *reel) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);
  if (data->impl->destructor) {
    data->impl->destructor(data->context);
  }
  if (data->owns_from_reel) {
    kino_reel_destroy(data->from_reel);
  }
  if (data->owns_to_reel) {
    kino_reel_destroy(data->to_reel);
  }
  gdraw_command_list_destroy(data->list_copy);
  applib_free(data);
}

static uint32_t prv_elapsed_getter(KinoReel *reel) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);
  return scale_int32(data->normalized, ANIMATION_NORMALIZED_MAX, data->duration);
}

static uint32_t prv_get_duration(KinoReelTransformData *data) {
  uint32_t duration = data->duration;
  if (data->from_reel) {
    const uint32_t from_duration = kino_reel_get_duration(data->from_reel);
    // If we don't have a 'to_reel' then we are looping back to the 'from_reel' so it's okay for
    // the 'from_reel' to have an infinite duration
    //
    // If we have a 'to_reel' then ignore infinite duration requests because we will never get to
    // it and burn a lot of power along the way!
    if ((data->to_reel == NULL) || (from_duration != PLAY_DURATION_INFINITE)) {
      duration = MAX(duration, from_duration);
    }
  }

  if (data->to_reel) {
    // We want to make sure the transform duration is at least as long as the to_reel duration
    // so that the resource transition runs to completion if it's animated
    const uint32_t to_duration = kino_reel_get_duration(data->to_reel);
    duration = MAX(duration, to_duration);
  }
  return duration;
}

static bool prv_elapsed_setter(KinoReel *reel, uint32_t elapsed) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);
  if (data->elapsed == elapsed) {
    return false;
  }

  bool changed = false;

  data->elapsed = elapsed;
  if (data->from_reel && kino_reel_set_elapsed(data->from_reel, elapsed)) {
    changed = true;
  }
  if (data->to_reel && kino_reel_set_elapsed(data->to_reel, elapsed)) {
    changed = true;
  }

  const int32_t normalized = animation_timing_clip(
      scale_int32(elapsed, data->duration, ANIMATION_NORMALIZED_MAX));
  if (data->normalized == normalized) {
    return changed;
  }

  data->normalized = normalized;

  // No position setter is shorthand for always triggering a transform on any position setting
  bool transform_changed = true;
  if (data->impl->position_setter &&
      !data->impl->position_setter(normalized, data->context)) {
    transform_changed = false;
  }

  return (changed || transform_changed);
}

static uint32_t prv_duration_getter(KinoReel *reel) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);
  return prv_get_duration(data);
}

static GSize prv_size_getter(KinoReel *reel) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);
  return interpolate_gsize(data->normalized, data->from.size, data->to.size);
}

static void prv_transform_list(KinoReelTransformData *data) {
  KinoReel *reel = prv_get_current_reel(data);
  GDrawCommandList *source_list = kino_reel_get_gdraw_command_list(reel);
  GDrawCommandList *list = prv_get_or_create_list_copy(data, source_list);
  if (!list) {
    return;
  }
  if (!gdraw_command_list_copy(list, gdraw_command_list_get_data_size(source_list),
                                source_list)) {
    return;
  }
  if (data->impl->apply) {
    const GSize size = kino_reel_get_size(reel);
    data->impl->apply(list, size, &data->from, &data->to, data->normalized, data->context);
  }
}

static void prv_draw_command_list_processed(GContext *ctx, GDrawCommandList *list, GPoint offset,
                                            KinoReelProcessor *processor) {
  GPoint draw_box_origin = ctx->draw_state.drawing_box.origin;
  graphics_context_move_draw_box(ctx, offset);
  gdraw_command_list_draw_processed(ctx, list, NULL_SAFE_FIELD_ACCESS(processor,
                                                                      draw_command_processor,
                                                                      NULL));
  ctx->draw_state.drawing_box.origin = draw_box_origin;
}

static void prv_draw_reel_or_command_list_processed(
    GContext *ctx, KinoReel *reel, GDrawCommandList *list, GPoint offset,
    KinoReelProcessor *processor) {
  if (list) {
    prv_draw_command_list_processed(ctx, list, offset, processor);
  } else {
    kino_reel_draw_processed(reel, ctx, offset, processor);
  }
}

static void prv_draw_processed_in_local(KinoReelTransformData *data, GContext *ctx, GPoint offset,
                                        KinoReelProcessor *processor) {
  KinoReel *reel = prv_get_current_reel(data);
  GDrawCommandList *source_list = kino_reel_get_gdraw_command_list(reel);
  GDrawCommandList *list = prv_get_or_create_list_copy(data, source_list);
  if (data->global) {
    offset = gpoint_sub(GPointZero, data->layer_frame.origin);
  }
  prv_draw_reel_or_command_list_processed(ctx, reel, list, offset, processor);
}

static void prv_draw_processed_in_global(KinoReelTransformData *data, GContext *ctx, GPoint offset,
                                         KinoReelProcessor *processor) {
  KinoReel *reel = prv_get_current_reel(data);
  GDrawCommandList *source_list = kino_reel_get_gdraw_command_list(reel);
  GDrawCommandList *list = prv_get_or_create_list_copy(data, source_list);
  offset = gpoint_to_local_coordinates(GPointZero, ctx);
  if (!list) {
    // There is no list with global coordinates embedded. Instead, interpolate the offset.
    gpoint_add_eq(&offset, prv_get_interpolated_origin(data));
  }
  prv_draw_reel_or_command_list_processed(ctx, reel, list, offset, processor);
}

static void prv_draw_processed_at_rect(KinoReelTransformData *data, GContext *ctx, GPoint offset,
                                       GRect rect, KinoReelProcessor *processor) {
  if (!prv_image_size_eq_rect_size(data, rect)) {
    prv_transform_list(data);
    prv_draw_processed_in_local(data, ctx, offset, processor);
    return;
  }
  prv_free_list_copy(data);
  if (data->global) {
    offset = gpoint_sub(GPointZero, data->layer_frame.origin);
  }
  offset = gpoint_add(offset, rect.origin);
  KinoReel *reel = prv_get_current_reel(data);
  GDrawCommandList *source_list = kino_reel_get_gdraw_command_list(reel);
  prv_draw_reel_or_command_list_processed(ctx, reel, source_list, offset, processor);
}

static void prv_draw_processed_func(KinoReel *reel, GContext *ctx, GPoint offset,
                                    KinoReelProcessor *processor) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);

  if (data->normalized == 0) {
    prv_draw_processed_at_rect(data, ctx, offset, data->from, processor);
    return;
  }

  if (data->normalized == ANIMATION_NORMALIZED_MAX) {
    prv_draw_processed_at_rect(data, ctx, offset, data->to, processor);
    return;
  }

  prv_transform_list(data);

  if (data->global) {
    prv_draw_processed_in_global(data, ctx, offset, processor);
  } else {
    prv_draw_processed_in_local(data, ctx, offset, processor);
  }
}

static GDrawCommandList *prv_get_gdraw_command_list(KinoReel *reel) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);
  if (data) {
    KinoReel *reel = prv_get_current_reel(data);
    return kino_reel_get_gdraw_command_list(reel);
  }
  return NULL;
}

static const KinoReelImpl s_kino_reel_impl_transform = {
  .destructor = prv_destructor,
  .get_elapsed = prv_elapsed_getter,
  .set_elapsed = prv_elapsed_setter,
  .get_duration = prv_duration_getter,
  .get_size = prv_size_getter,
  .draw_processed = prv_draw_processed_func,
  .get_gdraw_command_list = prv_get_gdraw_command_list,
};

KinoReel *kino_reel_transform_create(const TransformImpl *impl, void *context) {
  KinoReelTransformData *data = applib_malloc(sizeof(KinoReelTransformData));
  if (!data) {
    return NULL;
  }

  *data = (KinoReelTransformData) {
    .impl = impl,
    .context = context,
    .duration = ANIMATION_DEFAULT_DURATION_MS,
  };

  KinoReel *reel = kino_reel_custom_create(&s_kino_reel_impl_transform, data);
  if (!reel) {
    applib_free(data);
  }
  return reel;
}

void *kino_reel_transform_get_context(KinoReel *reel) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);
  if (data) {
    return data->context;
  }
  return NULL;
}

void kino_reel_transform_set_from_reel(KinoReel *reel, KinoReel *from_reel,
                                            bool take_ownership) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);
  if (!data) {
    return;
  }
  if (data->owns_from_reel) {
    kino_reel_destroy(data->from_reel);
  }
  data->from_reel = from_reel;
  data->owns_from_reel = take_ownership;
  prv_free_list_copy(data);
}

KinoReel *kino_reel_transform_get_from_reel(KinoReel *reel) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);
  if (data) {
    return data->from_reel;
  }
  return NULL;
}

void kino_reel_transform_set_to_reel(KinoReel *reel, KinoReel *to_reel,
                                          bool take_ownership) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);
  if (!data) {
    return;
  }
  if (data->owns_to_reel) {
    kino_reel_destroy(data->to_reel);
  }
  data->to_reel = to_reel;
  data->owns_to_reel = take_ownership;
  prv_free_list_copy(data);
}

KinoReel *kino_reel_transform_get_to_reel(KinoReel *reel) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);
  if (data) {
    return data->to_reel;
  }
  return NULL;
}

void kino_reel_transform_set_layer_frame(KinoReel *reel, GRect layer_frame) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);
  if (data) {
    data->layer_frame = layer_frame;
  }
}

void kino_reel_transform_set_from_frame(KinoReel *reel, GRect from) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);
  if (data) {
    data->from = from;
  }
}

void kino_reel_transform_set_to_frame(KinoReel *reel, GRect to) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);
  if (data) {
    data->to = to;
  }
}

GRect kino_reel_transform_get_from_frame(KinoReel *reel) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);
  if (data) {
    return data->from;
  }
  return GRectZero;
}

GRect kino_reel_transform_get_to_frame(KinoReel *reel) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);
  if (data) {
    return data->to;
  }
  return GRectZero;
}

void kino_reel_transform_set_global(KinoReel *reel, bool global) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);
  if (data) {
    data->global = global;
  }
}

void kino_reel_transform_set_transform_duration(KinoReel *reel, uint32_t duration) {
  KinoReelTransformData *data = kino_reel_custom_get_data(reel);
  if (data) {
    data->duration = duration;
  }
}
