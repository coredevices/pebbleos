/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/compositor/default/compositor_shutter_transitions.h"

#include "pbl/services/compositor/compositor_transitions.h"

#include "applib/graphics/framebuffer.h"
#include "applib/graphics/gpath.h"
#include "applib/graphics/graphics.h"
#include "applib/graphics/graphics_private.h"
#include "system/passert.h"

typedef struct {
  CompositorTransitionDirection direction;
  GColor color;
  GColor sampled_color;
  GPath path;
  bool is_first_half;
  int16_t animation_offset_px;
  Animation *first_anim;
  Animation *second_anim;
} CompositorShutterTransitionData;

static CompositorShutterTransitionData s_data;

#define PATH_WEDGE_POINTS 3
#define PATH_QUAD_POINTS 4

static GPathInfo s_path_wedge = {
  .num_points = PATH_WEDGE_POINTS,
  .points = (GPoint[PATH_WEDGE_POINTS]) {
    // These are just placeholders to allocate the needed space.
    // They will be set to the proper values during the animation.
    {0, 0}, {0, 0}, {0, 0},
  },
};

static GPathInfo s_path_quad = {
  .num_points = PATH_QUAD_POINTS,
  .points = (GPoint[PATH_QUAD_POINTS]) {
    // These are just placeholders to allocate the needed space.
    // They will be set to the proper values during the animation.
    {0, 0}, {0, 0}, {0, 0}, {0, 0},
  },
};

typedef struct PathInterpDefinition {
  GPoint start, end;
} PathInterpDefinition;

typedef struct PathDefinition {
  PathInterpDefinition wedge_verts[PATH_WEDGE_POINTS];
  PathInterpDefinition quad_verts[PATH_QUAD_POINTS];
} PathDefinition;

#define PATH_INTERP_DEF_TL_CORNER \
  { { 0, 0 }, \
    { 0, 0 }, }
#define PATH_INTERP_DEF_TR_CORNER \
  { { DISP_COLS, 0 }, \
    { DISP_COLS, 0 }, }
#define PATH_INTERP_DEF_BL_CORNER \
  { { 0, DISP_ROWS }, \
    { 0, DISP_ROWS }, }
#define PATH_INTERP_DEF_BR_CORNER \
  { { DISP_COLS, DISP_ROWS }, \
    { DISP_COLS, DISP_ROWS }, }


// These factors are based on getting pixel coordinates from the videos, then dividing them by
// the designed screen size (144x168). By being ratios instead of pixel counts, these can work
// out of the box on Robert.
static const PathDefinition s_path_defs[4] = {
  [CompositorTransitionDirectionUp] = {
    .wedge_verts = {
      // BL: 0,M -> 0,109 (0.65)
      { {                0, DISP_ROWS },
        {                0, DISP_ROWS * 0.65 }, },
      // BM: 72,M (0.5) -> 115,M (0.8)
      { { DISP_COLS * 0.5, DISP_ROWS },
        { DISP_COLS * 0.8, DISP_ROWS }, },
      PATH_INTERP_DEF_BL_CORNER,
    },
    .quad_verts = {
      // TR: M,0 -> M,52 (0.31)
      { {        DISP_COLS,         0 },
        {        DISP_COLS, DISP_ROWS * 0.31 }, },
      // TL: 0,0 -> 0,30 (0.18)
      { {                0,         0 },
        {                0, DISP_ROWS * 0.18 }, },
      PATH_INTERP_DEF_TL_CORNER,
      PATH_INTERP_DEF_TR_CORNER,
    },
  },
  [CompositorTransitionDirectionDown] = {
    // Vertical mirror of Up
    .wedge_verts = {
      // TL: 0,0 -> 0,59 (0.35)
      { {                0, 0 },
        {                0, DISP_ROWS * 0.35 }, },
      // TM: 72,0 (0.5) -> 115,0 (0.8)
      { { DISP_COLS * 0.5, 0 },
        { DISP_COLS * 0.8, 0 }, },
      PATH_INTERP_DEF_TL_CORNER,
    },
    .quad_verts = {
      // BR: M,M -> M,116 (0.69)
      { {        DISP_COLS, DISP_ROWS },
        {        DISP_COLS, DISP_ROWS * 0.69 }, },
      // BL: 0,M -> 0,138 (0.82)
      { {                0, DISP_ROWS },
        {                0, DISP_ROWS * 0.82 }, },
      PATH_INTERP_DEF_BL_CORNER,
      PATH_INTERP_DEF_BR_CORNER,
    },
  },
  // We don't have one for Left because the shutter will not be drawn on it.
  [CompositorTransitionDirectionRight] = {
    .wedge_verts = {
      // TL: 0,0 -> 50,0 (0.35)
      { {                0, 0 },
        { DISP_COLS * 0.35, 0 }, },
      // ML: 0,50 (0.3) -> 0,117 (0.7)
      { {                0, DISP_ROWS * 0.3 },
        {                0, DISP_ROWS * 0.7 }, },
      PATH_INTERP_DEF_TL_CORNER,
    },
    .quad_verts = {
      // BR: M,M -> 93,M (0.65)
      { {        DISP_COLS, DISP_ROWS },
        { DISP_COLS * 0.65, DISP_ROWS }, },
      // TR: M,0 -> 119,0 (0.83)
      { {        DISP_COLS,         0 },
        { DISP_COLS * 0.83,         0 }, },
      PATH_INTERP_DEF_TR_CORNER,
      PATH_INTERP_DEF_BR_CORNER,
    },
  },
};

// Creates a gpoint from a PathInterpDefinition and animation progress.
static void prv_gpoint_interpolate(GPoint *result, int32_t normalized,
                                   const PathInterpDefinition *def) {
  result->x = interpolate_int16(normalized, def->start.x, def->end.x);
  result->y = interpolate_int16(normalized, def->start.y, def->end.y);
}

// piecewise interpolator between 0 and to for the first half of ANIMATION_NORMALIZED_MAX
// and between -to and 0 for the second half
static int16_t prv_interpolate_two_ways(AnimationProgress normalized_progress, int16_t end) {
  const int16_t from = s_data.is_first_half ? 0 : -end;
  const int16_t to = s_data.is_first_half ? end : 0;
  return interpolate_int16(normalized_progress, from, to);
}

// Draws the shutters based on the path definitions
static void prv_draw_shutter(GContext *ctx, uint32_t distance, bool vertical) {
  graphics_context_set_antialiased(ctx, true);
  graphics_context_set_stroke_width(ctx, 1);
  graphics_context_set_stroke_color(ctx, s_data.color);
  graphics_context_set_fill_color(ctx, s_data.color);

  for (int i = 0; i < PATH_WEDGE_POINTS; i++) {
    prv_gpoint_interpolate(&s_path_wedge.points[i], distance,
      &s_path_defs[s_data.direction].wedge_verts[i]);
  }
  gpath_init(&s_data.path, &s_path_wedge);
  gpath_draw_outline(ctx, &s_data.path);
  gpath_draw_filled(ctx, &s_data.path);

  for (int i = 0; i < PATH_QUAD_POINTS; i++) {
    prv_gpoint_interpolate(&s_path_quad.points[i], distance,
      &s_path_defs[s_data.direction].quad_verts[i]);
  }
  gpath_init(&s_data.path, &s_path_quad);
  gpath_draw_outline(ctx, &s_data.path);
  gpath_draw_filled(ctx, &s_data.path);
}

// Moves the current framebuffer around
static int16_t prv_move_in(GContext *ctx, int move_size, uint32_t distance, bool vertical) {
  const int16_t current_offset_px = prv_interpolate_two_ways(distance, move_size);
  const int16_t diff = s_data.animation_offset_px - current_offset_px;
  if (vertical) {
    graphics_private_move_pixels_vertically(&ctx->dest_bitmap, -diff);
  } else {
    graphics_private_move_pixels_horizontally(&ctx->dest_bitmap, diff, true /* patch_garbage */);
  }
  s_data.animation_offset_px = current_offset_px;
  framebuffer_dirty_all(compositor_get_framebuffer());
  return current_offset_px;
}

// Draws in the new application's framebuffer and any transparent modal
static void prv_draw_in(GContext *ctx, int move_size, uint32_t distance, bool vertical,
                        bool invert) {
  const int16_t current_offset_px = prv_interpolate_two_ways(distance, invert ? -move_size :
                                                                                move_size);
  const GPoint point = vertical ? GPoint(0, -current_offset_px) :
                                  GPoint(-current_offset_px, 0);

  // Make sure the undrawn areas are the shutter color
  graphics_context_set_fill_color(ctx, s_data.sampled_color);
  graphics_fill_rect(ctx, &DISP_FRAME);

  compositor_scaled_app_fb_copy(GRect(point.x, point.y, DISP_COLS, DISP_ROWS), false /* copy_relative_to_origin */);

  const GPoint drawing_box_origin = ctx->draw_state.drawing_box.origin;
  gpoint_add_eq(&ctx->draw_state.drawing_box.origin, point);
  compositor_render_modal();
  ctx->draw_state.drawing_box.origin = drawing_box_origin;
}

const int32_t s_small_movement_size = DISP_COLS * 0.042; // 6 on snowy
const int32_t s_large_movement_size = DISP_COLS * 0.14; // 20 on snowy
const int32_t s_vertical_movement_size = DISP_ROWS * 0.18; // 30 on snowy

static void prv_transition_animation_update(GContext *ctx, Animation *animation,
                                            uint32_t progress) {
  const bool direction_negative = ((s_data.direction == CompositorTransitionDirectionRight) ||
                                   (s_data.direction == CompositorTransitionDirectionUp));
  const bool direction_vertical = ((s_data.direction == CompositorTransitionDirectionDown) ||
                                   (s_data.direction == CompositorTransitionDirectionUp));
  // Left has no wedge path definition, so no shutter is drawn on it
  const bool direction_has_shutter =
      direction_negative || (s_data.direction == CompositorTransitionDirectionDown);

  static GDrawState prev_state;
  prev_state = ctx->draw_state;

  if (s_data.is_first_half) {
    const int32_t movement_size = direction_vertical ? s_vertical_movement_size
                                                     : s_large_movement_size;
    prv_move_in(ctx, direction_negative ? movement_size : -movement_size, progress,
                direction_vertical);
    if (direction_has_shutter) {
      prv_draw_shutter(ctx, progress, direction_vertical);
    }
  } else {
    // Settle the incoming framebuffer along the transition direction: negative directions
    // (Up/Right) settle downwards/rightwards, Down and Left settle the other way.
    prv_draw_in(ctx, s_small_movement_size, progress, direction_vertical,
                /*invert*/ direction_negative);
  }

  ctx->draw_state = prev_state;
}

static void prv_transition_animation_first_update(Animation *animation,
                                            const AnimationProgress progress) {
  s_data.is_first_half = true;
  compositor_transition_render(prv_transition_animation_update, animation, progress);
}

static void prv_transition_animation_second_update(Animation *animation,
                                            const AnimationProgress progress) {
  if (s_data.is_first_half) {
    // This needs to be sampled here instead of in init because apparently the app framebuffer
    // hasn't been drawn at all during init.
    const GBitmap app_bitmap = compositor_get_app_framebuffer_as_bitmap();
    s_data.sampled_color = graphics_private_sample_line_color(&app_bitmap,
                                                              (GColorSampleEdge)s_data.direction,
                                                              GColorBlack);
    // Force the sampled color to be completely opaque, because we're using this to fill the
    // framebuffer background when moving the new app into focus.
    s_data.sampled_color.a = 3;
  }
  s_data.is_first_half = false;
  compositor_transition_render(prv_transition_animation_update, animation, progress);
}

static void prv_transition_animation_init(Animation *animation) {
  Animation *anim_array[2] = {NULL};
  size_t anim_count = 0;
  uint32_t duration = 0;

  s_data.first_anim = animation_create();
  if (s_data.first_anim) {
    anim_array[anim_count++] = s_data.first_anim;
    static const AnimationImplementation s_first_animation_impl = {
      .update = prv_transition_animation_first_update,
    };
    animation_set_implementation(s_data.first_anim, &s_first_animation_impl);
    animation_set_duration(s_data.first_anim, SHUTTER_TRANSITION_FIRST_DURATION_MS);
    duration += SHUTTER_TRANSITION_FIRST_DURATION_MS;
    animation_set_curve(s_data.first_anim, AnimationCurveEaseIn);
  }

  s_data.second_anim = animation_create();
  if (s_data.second_anim) {
    anim_array[anim_count++] = s_data.second_anim;
    static const AnimationImplementation s_second_animation_impl = {
      .update = prv_transition_animation_second_update,
    };
    animation_set_implementation(s_data.second_anim, &s_second_animation_impl);
    animation_set_duration(s_data.second_anim, SHUTTER_TRANSITION_SECOND_DURATION_MS);
    duration += SHUTTER_TRANSITION_SECOND_DURATION_MS;
    animation_set_curve(s_data.second_anim, AnimationCurveEaseOut);
  }

  PBL_ASSERTN(animation_sequence_init_from_array(animation, anim_array, anim_count));
  animation_set_duration(animation, duration);
  animation_set_curve(animation, AnimationCurveLinear);

  s_data.animation_offset_px = 0;
}

static void prv_transition_animation_update_stub(GContext *ctx,
                                                 Animation *animation,
                                                 uint32_t progress) {
}

const CompositorTransition *compositor_shutter_transition_get(
    CompositorTransitionDirection direction, GColor color) {
  if (compositor_transition_app_to_app_should_be_skipped()) {
    return NULL;
  }

  s_data = (CompositorShutterTransitionData){};
  s_data.color = color;
  s_data.direction = direction;

  static const CompositorTransition s_impl = {
    .init = prv_transition_animation_init,
    .update = prv_transition_animation_update_stub,
    .skip_modal_render_after_update = true,
  };
  return &s_impl;
}
