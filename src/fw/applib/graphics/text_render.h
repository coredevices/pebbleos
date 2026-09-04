/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/fonts/codepoint.h"
#include "applib/fonts/fonts_private.h"
#include "applib/graphics/gtypes.h"

#include "gtypes.h"

typedef struct GContext GContext;

typedef void (*SpecialCodepointHandlerCb)(GContext *ctx, Codepoint codepoint, GRect cursor,
              void *context);

void render_glyph(GContext* const ctx, const uint32_t codepoint, FontInfo* const font,
                  const GRect cursor);

// This function sets a handler callback for handling special codepoints encountered during text
// rendering. This allows special draw operations at the cursor position that the codepoint occurs.
// This must be set to NULL when the window using it goes out of focus.
void text_render_set_special_codepoint_cb(SpecialCodepointHandlerCb handler, void *context);
