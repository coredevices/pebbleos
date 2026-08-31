/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kino_reel.h"
#include "kino_reel_pdci.h"
#include "kino_reel_pdcs.h"
#include "kino_reel_gbitmap.h"
#include "kino_reel_gbitmap_sequence.h"

#include "applib/graphics/gdraw_command_private.h"
#include "applib/graphics/gbitmap_png.h"

#include "resource/resource.h"
#include "resource/resource_ids.auto.h"
#include "syscall/syscall.h"
#include "util/net.h"

KinoReel *kino_reel_create_with_resource(uint32_t resource_id) {
  ResAppNum app_num = sys_get_current_resource_num();
  return kino_reel_create_with_resource_system(app_num, resource_id);
}

KinoReel *kino_reel_create_with_resource_system(ResAppNum app_num, uint32_t resource_id) {
  if (resource_id == RESOURCE_ID_INVALID) {
    return NULL;
  }

  // The first 4 bytes for media data files contains the type signature (except legacy PBI)
  uint32_t data_signature;
  if (sys_resource_load_range(app_num, resource_id, 0, (uint8_t*)&data_signature,
        sizeof(data_signature)) != sizeof(data_signature)) {
    return NULL;
  }

  switch (ntohl(data_signature)) {
    case PDCS_SIGNATURE:
      return kino_reel_pdcs_create_with_resource_system(app_num, resource_id);
    case PDCI_SIGNATURE:
      return kino_reel_pdci_create_with_resource_system(app_num, resource_id);
    case PNG_SIGNATURE:
      {
        bool is_apng = false;
        // Check if the PNG is an APNG by seeking for the actl chunk
        png_seek_chunk_in_resource_system(app_num, resource_id, PNG_HEADER_SIZE, true, &is_apng);
        if (is_apng) {
          return kino_reel_gbitmap_sequence_create_with_resource_system(app_num, resource_id);
        } else {
          return kino_reel_gbitmap_create_with_resource_system(app_num, resource_id);
        }
      }
    default:
      // We don't have any good way to validate that something
      // is indeed a gbitmap. We use it as our fallback.
      return kino_reel_gbitmap_create_with_resource_system(app_num, resource_id);
  }

  return NULL;
}

void kino_reel_destroy(KinoReel *reel) {
  if (reel) {
    reel->impl->destructor(reel);
  }
}

void kino_reel_draw_processed(KinoReel *reel, GContext *ctx, GPoint offset,
                              KinoReelProcessor *processor) {
  if (reel) {
    reel->impl->draw_processed(reel, ctx, offset, processor);
  }
}

void kino_reel_draw(KinoReel *reel, GContext *ctx, GPoint offset) {
  kino_reel_draw_processed(reel, ctx, offset, NULL);
}

GSize kino_reel_get_size(KinoReel *reel) {
  if (reel && reel->impl->get_size) {
    return reel->impl->get_size(reel);
  }

  return GSize(0, 0);
}

size_t kino_reel_get_data_size(const KinoReel *reel) {
  if (reel && reel->impl->get_data_size) {
    return reel->impl->get_data_size(reel);
  }

  return 0;
}

bool kino_reel_set_elapsed(KinoReel *reel, uint32_t elapsed) {
  if (reel && reel->impl->set_elapsed) {
    return reel->impl->set_elapsed(reel, elapsed);
  }

  return false;
}

uint32_t kino_reel_get_elapsed(KinoReel *reel) {
  if (reel && reel->impl->get_elapsed) {
    return reel->impl->get_elapsed(reel);
  }

  return 0;
}

uint32_t kino_reel_get_duration(KinoReel *reel) {
  if (reel && reel->impl->get_duration) {
    return reel->impl->get_duration(reel);
  }

  return 0;
}

GDrawCommandImage *kino_reel_get_gdraw_command_image(KinoReel *reel) {
  if (reel && reel->impl->get_gdraw_command_image) {
    return reel->impl->get_gdraw_command_image(reel);
  }

  return NULL;
}

GDrawCommandList *kino_reel_get_gdraw_command_list(KinoReel *reel) {
  if (reel && reel->impl->get_gdraw_command_list) {
    return reel->impl->get_gdraw_command_list(reel);
  }

  return NULL;
}

GDrawCommandSequence *kino_reel_get_gdraw_command_sequence(KinoReel *reel) {
  if (reel && reel->impl->get_gdraw_command_sequence) {
    return reel->impl->get_gdraw_command_sequence(reel);
  }

  return NULL;
}

GBitmap *kino_reel_get_gbitmap(KinoReel *reel) {
  if (reel && reel->impl->get_gbitmap) {
    return reel->impl->get_gbitmap(reel);
  }

  return NULL;
}

GBitmapSequence* kino_reel_get_gbitmap_sequence(KinoReel *reel) {
  if (reel && reel->impl->get_gbitmap_sequence) {
    return reel->impl->get_gbitmap_sequence(reel);
  }

  return NULL;
}

KinoReelType kino_reel_get_type(KinoReel *reel) {
  if (reel) {
    return reel->impl->reel_type;
  }

  return KinoReelTypeInvalid;
}
