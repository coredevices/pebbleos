/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "action_button.h"

#include "applib/graphics/graphics.h"
#include "applib/preferred_content_size.h"
#include "applib/ui/action_bar_layer.h"

void action_button_draw(GContext *ctx, Layer *layer, GColor fill_color) {
  // This should match the window bounds
  const GRect bounds = layer->bounds;
  const bool on_right = action_bar_layer_is_on_right();

  const int radius = PBL_IF_ROUND_ELSE(12, 13);
  GRect rect = { .size = { radius * 2, radius * 2 } };
  grect_align(&rect, &bounds, on_right ? GAlignRight : GAlignLeft, false);

  rect.origin.x += on_right ? radius : -radius;

  const int extra = PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
    //! @note this is the same as Medium until Small is designed
    /* small */ PBL_IF_ROUND_ELSE(1, 8),
    /* medium */ PBL_IF_ROUND_ELSE(1, 8),
    /* large */ 4,
    //! @note this is the same as Large until ExtraLarge is designed
    /* extralarge */ 4);
  rect.origin.x += on_right ? extra : -extra;

  graphics_context_set_fill_color(ctx, fill_color);
  graphics_fill_oval(ctx, rect, GOvalScaleModeFitCircle);
}

void action_button_update_proc(Layer *action_button_layer, GContext *ctx) {
  action_button_draw(ctx, action_button_layer, GColorBlack);
}
