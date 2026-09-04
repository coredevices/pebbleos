/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "rotate_bitmap_layer.h"

#include "applib/graphics/graphics.h"
#include "applib/graphics/gtypes.h"
#include "pbl/util/trig.h"
#include "applib/applib_malloc.auto.h"
#include "system/passert.h"
#include "pbl/util/math.h"

void rot_bitmap_layer_update_proc(RotBitmapLayer *image, GContext* ctx) {
  const GColor corner_clip_color = image->corner_clip_color;
  if (!(gcolor_is_transparent(corner_clip_color))) {
    graphics_context_set_fill_color(ctx, corner_clip_color);
    graphics_fill_rect(ctx, &image->layer.bounds);
  }
  graphics_context_set_compositing_mode(ctx, image->compositing_mode);
  graphics_draw_rotated_bitmap(ctx, image->bitmap, image->src_ic, image->rotation, image->dest_ic);
}

void rot_bitmap_layer_init(RotBitmapLayer *image, GBitmap *bitmap) {
  *image = (RotBitmapLayer){};

  image->bitmap = bitmap;
  const int32_t bmp_width = bitmap->bounds.size.w;
  const int32_t bmp_height = bitmap->bounds.size.h;
  const int32_t layer_size = integer_sqrt(bmp_width * bmp_width + bmp_height * bmp_height);

  layer_init(&image->layer, &GRect(0, 0, layer_size, layer_size));
  image->layer.update_proc = (LayerUpdateProc)rot_bitmap_layer_update_proc;

  image->src_ic = GPoint(bmp_width / 2, bmp_height / 2);
  image->dest_ic = GPoint(layer_size / 2, layer_size / 2);
  image->rotation = 0;

  image->corner_clip_color = GColorClear;
  image->compositing_mode = GCompOpAssign;

  layer_mark_dirty(&(image->layer));
}

RotBitmapLayer* rot_bitmap_layer_create(GBitmap *bitmap) {
  RotBitmapLayer* layer = applib_type_malloc(RotBitmapLayer);
  if (layer) {
    rot_bitmap_layer_init(layer, bitmap);
  }
  return layer;
}

void rot_bitmap_layer_deinit(RotBitmapLayer *rot_bitmap_layer) {
  layer_deinit(&rot_bitmap_layer->layer);
}

void rot_bitmap_layer_destroy(RotBitmapLayer *rot_bitmap_layer) {
  if (rot_bitmap_layer == NULL) {
    return;
  }
  rot_bitmap_layer_deinit(rot_bitmap_layer);
  applib_free(rot_bitmap_layer);
}

Layer* rot_bitmap_layer_get_layer(const RotBitmapLayer *rot_bitmap_layer) {
  return &((RotBitmapLayer *)rot_bitmap_layer)->layer;
}

void rot_bitmap_layer_set_corner_clip_color(RotBitmapLayer *image, GColor color) {
  if (gcolor_equal(color, image->corner_clip_color)) {
    return;
  }
  image->corner_clip_color = color;
  layer_mark_dirty(&(image->layer));
}

void rot_bitmap_layer_set_corner_clip_color_2bit(RotBitmapLayer *bitmap, GColor2 color) {
  rot_bitmap_layer_set_corner_clip_color(bitmap, get_native_color(color));
}

void rot_bitmap_layer_set_angle(RotBitmapLayer *image, int32_t angle) {
  if (angle % TRIG_MAX_ANGLE == image->rotation) {
    return;
  }
  image->rotation = angle % TRIG_MAX_ANGLE;
  layer_mark_dirty(&(image->layer));
}

void rot_bitmap_layer_increment_angle(RotBitmapLayer *image, int32_t angle_change) {
  if (angle_change % TRIG_MAX_ANGLE == 0) {
    return;
  }
  image->rotation = (image->rotation + angle_change) % TRIG_MAX_ANGLE;
  layer_mark_dirty(&(image->layer));
}

void rot_bitmap_set_src_ic(RotBitmapLayer *image, GPoint ic) {
  image->src_ic = ic;

  // adjust the frame so the whole image will still be visible
  const int32_t right = abs(image->bitmap->bounds.size.w - ic.x);
  const int32_t horiz = MAX(ic.x, right);
  const int32_t bottom = abs(image->bitmap->bounds.size.h - ic.y);
  const int32_t vert = MAX(ic.y, bottom);

  GRect r = image->layer.frame;
  const int32_t new_dist = integer_sqrt(horiz*horiz + vert*vert) * 2;

  r.size.w = new_dist;
  r.size.h = new_dist;
  layer_set_frame(&image->layer, &r);

  r.origin = GPoint(0, 0);
  layer_set_bounds(&image->layer, &r);

  image->dest_ic = GPoint(new_dist / 2, new_dist / 2);

  layer_mark_dirty(&image->layer);
}

void rot_bitmap_set_compositing_mode(RotBitmapLayer *image, GCompOp mode) {
  if (mode == image->compositing_mode) {
    return;
  }
  image->compositing_mode = mode;
  layer_mark_dirty(&image->layer);
}
