/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdlib.h>

#include "applib/graphics/gbitmap_sequence.h"
#include "applib/graphics/gdraw_command_image.h"
#include "applib/graphics/gdraw_command_sequence.h"
#include "applib/graphics/gtypes.h"

struct KinoReel;
typedef struct KinoReel KinoReel;

struct KinoReelImpl;
typedef struct KinoReelImpl KinoReelImpl;

struct KinoReelProcessor;
typedef struct KinoReelProcessor KinoReelProcessor;

typedef void (*KinoReelDestructor)(KinoReel *reel);
typedef uint32_t (*KinoReelElapsedGetter)(KinoReel *reel);
typedef bool (*KinoReelElapsedSetter)(KinoReel *reel, uint32_t elapsed_ms);
typedef uint32_t (*KinoReelDurationGetter)(KinoReel *reel);
#pragma push_macro("GSize")
#undef GSize
typedef GSize (*KinoReelSizeGetter)(KinoReel *reel);
#pragma pop_macro("GSize")
typedef size_t (*KinoReelDataSizeGetter)(const KinoReel *reel);
typedef void (*KinoReelDrawProcessedFunc)(KinoReel *reel, GContext *ctx, GPoint offset,
                                          KinoReelProcessor *processor);
typedef GDrawCommandImage* (*KinoReelGDrawCommandImageGetter)(KinoReel *reel);
typedef GDrawCommandList* (*KinoReelGDrawCommandListGetter)(KinoReel *reel);
typedef GDrawCommandSequence* (*KinoReelGDrawCommandSequenceGetter)(KinoReel *reel);
typedef GBitmap* (*KinoReelGBitmapGetter)(KinoReel *reel);
typedef GBitmapSequence* (*KinoReelGBitmapSequenceGetter)(KinoReel *reel);

struct KinoReelProcessor {
  GBitmapProcessor * const bitmap_processor;
  GDrawCommandProcessor * const draw_command_processor;
};

typedef enum {
  KinoReelTypeInvalid = 0,
  KinoReelTypeGBitmap,
  KinoReelTypeGBitmapSequence,
  KinoReelTypePDCI,
  KinoReelTypePDCS,
  KinoReelTypeCustom,
} KinoReelType;

struct KinoReelImpl {
  KinoReelType reel_type;
  KinoReelDestructor destructor;
  KinoReelElapsedSetter set_elapsed;
  KinoReelElapsedGetter get_elapsed;
  KinoReelDurationGetter get_duration;
  KinoReelSizeGetter get_size;
  KinoReelDataSizeGetter get_data_size;
  KinoReelDrawProcessedFunc draw_processed;

  // Kino Reel data retrieval, allows access to underlying assets
  KinoReelGDrawCommandImageGetter get_gdraw_command_image;
  KinoReelGDrawCommandListGetter get_gdraw_command_list;
  KinoReelGDrawCommandSequenceGetter get_gdraw_command_sequence;
  KinoReelGBitmapGetter get_gbitmap;
  KinoReelGBitmapSequenceGetter get_gbitmap_sequence;
};

struct KinoReel {
  const KinoReelImpl *impl;
};

KinoReel *kino_reel_create_with_resource(uint32_t resource_id);

KinoReel *kino_reel_create_with_resource_system(ResAppNum app_num, uint32_t resource_id);

void kino_reel_destroy(KinoReel *reel);

void kino_reel_draw_processed(KinoReel *reel, GContext *ctx, GPoint offset,
                              KinoReelProcessor *processor);

void kino_reel_draw(KinoReel *reel, GContext *ctx, GPoint offset);

bool kino_reel_set_elapsed(KinoReel *reel, uint32_t elapsed_ms);

uint32_t kino_reel_get_elapsed(KinoReel *reel);

uint32_t kino_reel_get_duration(KinoReel *reel);

GSize kino_reel_get_size(KinoReel *reel);

size_t kino_reel_get_data_size(const KinoReel *reel);

GDrawCommandImage *kino_reel_get_gdraw_command_image(KinoReel *reel);

GDrawCommandList *kino_reel_get_gdraw_command_list(KinoReel *reel);

GDrawCommandSequence *kino_reel_get_gdraw_command_sequence(KinoReel *reel);

GBitmap *kino_reel_get_gbitmap(KinoReel *reel);

GBitmapSequence *kino_reel_get_gbitmap_sequence(KinoReel *reel);

KinoReelType kino_reel_get_type(KinoReel *reel);
