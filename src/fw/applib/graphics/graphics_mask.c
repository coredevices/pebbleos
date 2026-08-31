/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "gcontext.h"

GDrawMask *graphics_context_mask_create(const GContext *ctx, bool transparent) {
  return NULL;
}

bool graphics_context_mask_record(GContext *ctx, GDrawMask *mask) {
  return false;
}

bool graphics_context_mask_use(GContext *ctx, GDrawMask *mask) {
  return false;
}

void graphics_context_mask_destroy(GContext *ctx, GDrawMask *mask) {
}
