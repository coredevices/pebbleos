/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "text_layer.h"
#include "text_layer_flow.h"

#include "applib/app_logging.h"
#include "applib/applib_malloc.auto.h"
#include "applib/fonts/fonts.h"
#include "applib/graphics/graphics.h"
#include "applib/graphics/gtypes.h"
#include "applib/preferred_content_size.h"
#include "process_state/app_state/app_state.h"
#include "shell/system_theme.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

#include <string.h>
#include <stddef.h>

static GTextLayoutCacheRef prv_text_layer_get_cache_handle(TextLayer *text_layer) {
  PBL_ASSERTN(text_layer);
  return text_layer->should_cache_layout ? text_layer->layout_cache : NULL;
}

void text_layer_update_proc(TextLayer *text_layer, GContext* ctx) {
  PBL_ASSERTN(text_layer);
  const GColor bg_color = text_layer->background_color;
  if (!(gcolor_equal(bg_color, GColorClear))) {
    graphics_context_set_fill_color(ctx, bg_color);
    graphics_fill_rect(ctx, &text_layer->layer.bounds);
  }
  if (text_layer->text && strlen(text_layer->text) > 0) {
    graphics_context_set_text_color(ctx, text_layer->text_color);
    graphics_draw_text(ctx, text_layer->text, text_layer->font,
        text_layer->layer.bounds, text_layer->overflow_mode,
        text_layer->text_alignment, prv_text_layer_get_cache_handle(text_layer));
  }
}

static const char * const s_text_layer_default_fonts[NumPreferredContentSizes] = {
  //! @note this is the same as Medium until Small is designed
  [PreferredContentSizeSmall] = FONT_KEY_GOTHIC_14_BOLD,
  [PreferredContentSizeMedium] = FONT_KEY_GOTHIC_14_BOLD,
  [PreferredContentSizeLarge] = FONT_KEY_GOTHIC_18_BOLD,
  //! @note this is the same as Large until ExtraLarge is designed
  [PreferredContentSizeExtraLarge] = FONT_KEY_GOTHIC_18_BOLD,
};

void text_layer_init_with_parameters(TextLayer *text_layer, const GRect *frame, const char *text,
                                     GFont font, GColor text_color, GColor back_color,
                                     GTextAlignment text_align, GTextOverflowMode overflow_mode) {
  PBL_ASSERTN(text_layer);
  *text_layer = (TextLayer){};
  text_layer->layer.frame = *frame;
  text_layer->layer.bounds = (GRect){{0, 0}, frame->size};
  text_layer->layer.update_proc = (LayerUpdateProc)text_layer_update_proc;
  text_layer->text_color = text_color;
  text_layer->background_color = back_color;
  text_layer->overflow_mode = overflow_mode;
  layer_set_clips(&text_layer->layer, true);

  text_layer->text_alignment = text_align;
  // Default font
  if (font == NULL) {
    const PreferredContentSize runtime_platform_default_size =
        system_theme_get_default_content_size_for_runtime_platform();
    font = fonts_get_system_font(s_text_layer_default_fonts[runtime_platform_default_size]);
  }
  text_layer->font = font;
  text_layer->text = text;

  layer_mark_dirty(&(text_layer->layer));
}

void text_layer_init(TextLayer *text_layer, const GRect *frame) {
  text_layer_init_with_parameters(text_layer, frame, NULL, NULL, GColorBlack, GColorWhite,
                                  GTextAlignmentLeft, GTextOverflowModeTrailingEllipsis);
}

TextLayer* text_layer_create(GRect frame) {
  TextLayer* layer = applib_type_malloc(TextLayer);
  if (layer) {
    text_layer_init(layer, &frame);
  }
  return layer;
}

void text_layer_destroy(TextLayer* text_layer) {
  if (!text_layer) {
    return;
  }
  text_layer_deinit(text_layer);
  applib_free(text_layer);
}

void text_layer_deinit(TextLayer *text_layer) {
  PBL_ASSERTN(text_layer);
  layer_deinit(&text_layer->layer);
  graphics_text_layout_cache_deinit(&text_layer->layout_cache);
  text_layer->layout_cache = NULL;
}

Layer* text_layer_get_layer(TextLayer *text_layer) {
  PBL_ASSERTN(text_layer);
  return &text_layer->layer;
}

void text_layer_set_size(TextLayer *text_layer, const GSize max_size) {
  PBL_ASSERTN(text_layer);
  layer_set_frame(&text_layer->layer, &(GRect) { text_layer->layer.frame.origin, max_size });
  layer_mark_dirty(&text_layer->layer);
}

GSize text_layer_get_size(TextLayer* text_layer) {
  PBL_ASSERTN(text_layer);
  return text_layer->layer.frame.size;
}

void text_layer_set_text(TextLayer *text_layer, const char *text) {
  PBL_ASSERTN(text_layer);
  text_layer->text = text;
  layer_mark_dirty(&(text_layer->layer));
}

const char* text_layer_get_text(TextLayer *text_layer) {
  PBL_ASSERTN(text_layer);
  return text_layer->text;
}

void text_layer_set_background_color(TextLayer *text_layer, GColor color) {
  PBL_ASSERTN(text_layer);
  const GColor bg_color = text_layer->background_color;
  if (gcolor_equal(color, bg_color)) {
    return;
  }
  text_layer->background_color = color;
  layer_mark_dirty(&(text_layer->layer));
}

void text_layer_set_text_color(TextLayer *text_layer, GColor color) {
  PBL_ASSERTN(text_layer);
  const GColor text_color = text_layer->text_color;
  if (gcolor_equal(color, text_color)) {
    return;
  }
  text_layer->text_color = color;
  layer_mark_dirty(&(text_layer->layer));
}

void text_layer_set_text_alignment(TextLayer *text_layer, GTextAlignment text_alignment) {
  PBL_ASSERTN(text_layer);
  if (text_alignment == text_layer->text_alignment) {
    return;
  }
  text_layer->text_alignment = text_alignment;
  layer_mark_dirty(&(text_layer->layer));
}

void text_layer_set_overflow_mode(TextLayer *text_layer, GTextOverflowMode overflow_mode) {
  PBL_ASSERTN(text_layer);
  if (overflow_mode == text_layer->overflow_mode) {
    return;
  }
  text_layer->overflow_mode = overflow_mode;
  layer_mark_dirty(&(text_layer->layer));
}

void text_layer_set_font(TextLayer *text_layer, GFont font) {
  PBL_ASSERTN(text_layer);
  text_layer->font = font;
  layer_mark_dirty(&(text_layer->layer));
}

void text_layer_set_should_cache_layout(TextLayer *text_layer, bool should_cache_layout) {
  PBL_ASSERTN(text_layer);
  if (should_cache_layout == text_layer->should_cache_layout) {
    return;
  }

  text_layer->should_cache_layout = should_cache_layout;

  if (text_layer->should_cache_layout) {
    PBL_LOG_DBG("Init layout");
    graphics_text_layout_cache_init(&text_layer->layout_cache);
  } else {
    graphics_text_layout_cache_deinit(&text_layer->layout_cache);
    text_layer->layout_cache = NULL;
  }
}

GSize text_layer_get_content_size(GContext* ctx, TextLayer *text_layer) {
  PBL_ASSERTN(text_layer);
  if (!text_layer->should_cache_layout) {
    text_layer_set_should_cache_layout(text_layer, true);
  }
  GTextLayoutCacheRef layout = prv_text_layer_get_cache_handle(text_layer);
  PBL_ASSERTN(layout);
  // content size now depends on position on screen due perimeter text flow
  GRect box;
  layer_get_global_frame(&text_layer->layer, &box);
  box.size = text_layer->layer.bounds.size;
  return graphics_text_layout_get_max_used_size(ctx, text_layer->text, text_layer->font,
      box, text_layer->overflow_mode, text_layer->text_alignment, layout);
}

GSize app_text_layer_get_content_size(TextLayer *text_layer) {
  PBL_ASSERTN(text_layer);
  GContext* ctx = app_state_get_graphics_context();
  return text_layer_get_content_size(ctx, text_layer);
}

void text_layer_set_line_spacing_delta(TextLayer *text_layer, int16_t delta) {
  PBL_ASSERTN(text_layer);
  // Initialize cached layout if not already initialized
  text_layer_set_should_cache_layout(text_layer, true);
  graphics_text_layout_set_line_spacing_delta(text_layer->layout_cache, delta);
  layer_mark_dirty(&(text_layer->layer));
}

int16_t text_layer_get_line_spacing_delta(TextLayer *text_layer) {
  PBL_ASSERTN(text_layer);
  return graphics_text_layout_get_line_spacing_delta(text_layer->layout_cache);
}

void text_layer_enable_screen_text_flow_and_paging(TextLayer *text_layer, uint8_t inset) {
  if (!text_layer) {
    return;
  }
  if (text_layer->layer.window == NULL) {
    APP_LOG(APP_LOG_LEVEL_ERROR, "Before calling %s, layer must be attached to view hierarchy.",
            __func__);
    return;
  }

  text_layer_set_should_cache_layout(text_layer, true);
  graphics_text_attributes_enable_screen_text_flow(text_layer->layout_cache, inset);
  GPoint origin;
  GRect page;

  if (text_layer_calc_text_flow_paging_values(text_layer, &origin, &page)) {
    graphics_text_attributes_enable_paging(text_layer->layout_cache, origin, page);
    layer_mark_dirty(&text_layer->layer);
  };
}

void text_layer_restore_default_text_flow_and_paging(TextLayer *text_layer) {
  if (!text_layer) {
    return;
  }
  if (text_layer->layout_cache) {
    graphics_text_attributes_restore_default_text_flow(text_layer->layout_cache);
    graphics_text_attributes_restore_default_paging(text_layer->layout_cache);
    layer_mark_dirty(&text_layer->layer);
  }
}
