/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kino_reel_gbitmap_sequence.h"

#include "applib/applib_malloc.auto.h"
#include "applib/graphics/gbitmap_sequence.h"
#include "syscall/syscall.h"
#include "pbl/util/struct.h"

typedef struct {
  KinoReel base;
  GBitmapSequence *sequence;
  bool owns_sequence;
  GBitmap *render_bitmap;
  uint32_t elapsed_ms;
} KinoReelImplGBitmapSequence;

static void prv_destructor(KinoReel *reel) {
  KinoReelImplGBitmapSequence *sequence_reel = (KinoReelImplGBitmapSequence *)reel;
  if (sequence_reel->owns_sequence) {
    gbitmap_sequence_destroy(sequence_reel->sequence);
  }

  gbitmap_destroy(sequence_reel->render_bitmap);
  applib_free(sequence_reel);
}

static uint32_t prv_elapsed_getter(KinoReel *reel) {
  KinoReelImplGBitmapSequence *sequence_reel = (KinoReelImplGBitmapSequence *)reel;
  return sequence_reel->elapsed_ms;
}

static bool prv_elapsed_setter(KinoReel *reel, uint32_t elapsed_ms) {
  KinoReelImplGBitmapSequence *sequence_reel = (KinoReelImplGBitmapSequence *)reel;
  sequence_reel->elapsed_ms = elapsed_ms;

  if (elapsed_ms == 0) {
    gbitmap_sequence_restart(sequence_reel->sequence);
  }

  return gbitmap_sequence_update_bitmap_by_elapsed(sequence_reel->sequence,
                                                   sequence_reel->render_bitmap,
                                                   sequence_reel->elapsed_ms);
}

static uint32_t prv_duration_getter(KinoReel *reel) {
  KinoReelImplGBitmapSequence *sequence_reel = (KinoReelImplGBitmapSequence *)reel;
  uint32_t duration = gbitmap_sequence_get_total_duration(sequence_reel->sequence);
  if (duration == 0) {
    duration = PLAY_DURATION_INFINITE;
  }
  return duration;
}

static GSize prv_size_getter(KinoReel *reel) {
  KinoReelImplGBitmapSequence *sequence_reel = (KinoReelImplGBitmapSequence *)reel;
  return gbitmap_sequence_get_bitmap_size(sequence_reel->sequence);
}

static void prv_draw_processed_func(KinoReel *reel, GContext *ctx, GPoint offset,
                                    KinoReelProcessor *processor) {
  KinoReelImplGBitmapSequence *sequence_reel = (KinoReelImplGBitmapSequence *)reel;
  GRect bounds = gbitmap_get_bounds(sequence_reel->render_bitmap);
  bounds.origin = gpoint_add(bounds.origin, offset);
  // Save compositing mode
  GCompOp prev_compositing_mode = ctx->draw_state.compositing_mode;

  graphics_context_set_compositing_mode(ctx, GCompOpSet);  // Enable compositing

  graphics_draw_bitmap_in_rect_processed(ctx, sequence_reel->render_bitmap, &bounds,
                                         NULL_SAFE_FIELD_ACCESS(processor, bitmap_processor, NULL));

  // Restore previous compositing mode
  graphics_context_set_compositing_mode(ctx, prev_compositing_mode);
}

static GBitmap *prv_get_gbitmap(KinoReel *reel) {
  if (reel) {
    return ((KinoReelImplGBitmapSequence*)reel)->render_bitmap;
  }
  return NULL;
}

static GBitmapSequence *prv_get_gbitmap_sequence(KinoReel *reel) {
  if (reel) {
    return ((KinoReelImplGBitmapSequence*)reel)->sequence;
  }
  return NULL;
}

static const KinoReelImpl KINO_REEL_IMPL_GBITMAPSEQUENCE = {
  .reel_type = KinoReelTypeGBitmapSequence,
  .destructor = prv_destructor,
  .get_elapsed = prv_elapsed_getter,
  .set_elapsed = prv_elapsed_setter,
  .get_duration = prv_duration_getter,
  .get_size = prv_size_getter,
  .draw_processed = prv_draw_processed_func,
  .get_gbitmap = prv_get_gbitmap,
  .get_gbitmap_sequence = prv_get_gbitmap_sequence,
};

KinoReel *kino_reel_gbitmap_sequence_create(GBitmapSequence *sequence, bool take_ownership) {
  KinoReelImplGBitmapSequence *reel = applib_zalloc(sizeof(KinoReelImplGBitmapSequence));
  if (reel) {
    reel->sequence = sequence;
    reel->owns_sequence = take_ownership;
    reel->elapsed_ms = 0;
    reel->base.impl = &KINO_REEL_IMPL_GBITMAPSEQUENCE;
    // init render bitmap
    reel->render_bitmap = gbitmap_create_blank(gbitmap_sequence_get_bitmap_size(sequence),
                                               GBitmapFormat8Bit);
    // Render initial frame upon load
    prv_elapsed_setter((KinoReel *)reel, 0);
  }

  return (KinoReel *)reel;
}

KinoReel *kino_reel_gbitmap_sequence_create_with_resource(uint32_t resource_id) {
  ResAppNum app_num = sys_get_current_resource_num();
  return kino_reel_gbitmap_sequence_create_with_resource_system(app_num, resource_id);
}

KinoReel *kino_reel_gbitmap_sequence_create_with_resource_system(ResAppNum app_num,
                                                                 uint32_t resource_id) {
  GBitmapSequence *sequence = gbitmap_sequence_create_with_resource_system(app_num, resource_id);
  if (sequence == NULL) {
    return NULL;
  }
  return kino_reel_gbitmap_sequence_create(sequence, true);
}
