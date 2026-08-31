/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kino_reel_pdcs.h"

#include "applib/applib_malloc.auto.h"
#include "applib/graphics/gdraw_command_frame.h"
#include "applib/graphics/gdraw_command_private.h"
#include "applib/graphics/gdraw_command_sequence.h"
#include "resource/resource_ids.auto.h"
#include "syscall/syscall.h"
#include "pbl/util/struct.h"

typedef struct {
  KinoReel base;
  GDrawCommandSequence *sequence;
  bool owns_sequence;
  GDrawCommandFrame *current_frame;
  uint32_t elapsed_ms;
} KinoReelImplPDCS;

static void prv_destructor(KinoReel *reel) {
  KinoReelImplPDCS *dcs_reel = (KinoReelImplPDCS *)reel;
  if (dcs_reel->owns_sequence) {
    gdraw_command_sequence_destroy(dcs_reel->sequence);
  }

  applib_free(dcs_reel);
}

static uint32_t prv_elapsed_getter(KinoReel *reel) {
  KinoReelImplPDCS *dcs_reel = (KinoReelImplPDCS *)reel;
  return dcs_reel->elapsed_ms;
}

static bool prv_elapsed_setter(KinoReel *reel, uint32_t elapsed_ms) {
  KinoReelImplPDCS *dcs_reel = (KinoReelImplPDCS *)reel;
  dcs_reel->elapsed_ms = elapsed_ms;
  GDrawCommandFrame *frame = gdraw_command_sequence_get_frame_by_elapsed(dcs_reel->sequence,
                                                                         dcs_reel->elapsed_ms);
  bool frame_changed = false;
  if (frame != dcs_reel->current_frame) {
    dcs_reel->current_frame = frame;
    frame_changed = true;
  }

  return frame_changed;
}

static uint32_t prv_duration_getter(KinoReel *reel) {
  KinoReelImplPDCS *dcs_reel = (KinoReelImplPDCS *)reel;
  return gdraw_command_sequence_get_total_duration(dcs_reel->sequence);
}

static GSize prv_size_getter(KinoReel *reel) {
  KinoReelImplPDCS *dcs_reel = (KinoReelImplPDCS *)reel;
  return gdraw_command_sequence_get_bounds_size(dcs_reel->sequence);
}

static size_t prv_data_size_getter(const KinoReel *reel) {
  KinoReelImplPDCS *dcs_reel = (KinoReelImplPDCS *)reel;
  return gdraw_command_sequence_get_data_size(dcs_reel->sequence);
}

static void prv_draw_processed_func(KinoReel *reel, GContext *ctx, GPoint offset,
                                    KinoReelProcessor *processor) {
  KinoReelImplPDCS *dcs_reel = (KinoReelImplPDCS *)reel;
  if (!dcs_reel->current_frame) {
    return;
  }

  gdraw_command_frame_draw_processed(ctx, dcs_reel->sequence, dcs_reel->current_frame, offset,
                                     NULL_SAFE_FIELD_ACCESS(processor, draw_command_processor,
                                                            NULL));
}

static GDrawCommandSequence *prv_get_gdraw_command_sequence(KinoReel *reel) {
  if (reel) {
    return ((KinoReelImplPDCS*)reel)->sequence;
  }
  return NULL;
}

static GDrawCommandList *prv_get_gdraw_command_list(KinoReel *reel) {
  KinoReelImplPDCS *dcs_reel = (KinoReelImplPDCS *)reel;
  if (dcs_reel) {
    return gdraw_command_frame_get_command_list(
        gdraw_command_sequence_get_frame_by_elapsed(dcs_reel->sequence, dcs_reel->elapsed_ms));
  }
  return NULL;
}

static const KinoReelImpl KINO_REEL_IMPL_PDCS = {
  .reel_type = KinoReelTypePDCS,
  .destructor = prv_destructor,
  .get_elapsed = prv_elapsed_getter,
  .set_elapsed = prv_elapsed_setter,
  .get_duration = prv_duration_getter,
  .get_size = prv_size_getter,
  .get_data_size = prv_data_size_getter,
  .draw_processed = prv_draw_processed_func,
  .get_gdraw_command_sequence = prv_get_gdraw_command_sequence,
  .get_gdraw_command_list = prv_get_gdraw_command_list,
};

KinoReel *kino_reel_pdcs_create(GDrawCommandSequence *sequence, bool take_ownership) {
  KinoReelImplPDCS *reel = applib_zalloc(sizeof(KinoReelImplPDCS));
  if (reel) {
    reel->sequence = sequence;
    reel->owns_sequence = take_ownership;
    reel->elapsed_ms = 0;
    reel->base.impl = &KINO_REEL_IMPL_PDCS;
    reel->current_frame = gdraw_command_sequence_get_frame_by_index(sequence, 0);
  }

  return (KinoReel *)reel;
}

KinoReel *kino_reel_pdcs_create_with_resource(uint32_t resource_id) {
  ResAppNum app_num = sys_get_current_resource_num();
  return kino_reel_pdcs_create_with_resource_system(app_num, resource_id);
}

KinoReel *kino_reel_pdcs_create_with_resource_system(ResAppNum app_num, uint32_t resource_id) {
  GDrawCommandSequence *sequence = gdraw_command_sequence_create_with_resource_system(app_num,
                                                                                      resource_id);
  if (sequence == NULL) {
    return NULL;
  }
  return kino_reel_pdcs_create(sequence, true);
}
