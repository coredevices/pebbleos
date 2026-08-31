/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/graphics/graphics.h"
#include "applib/ui/layer.h"
#include "applib/ui/kino/kino_layer.h"
#include "applib/ui/text_layer.h"
#include "pbl/services/evented_timer.h"
#include "pbl/services/timeline/timeline_resources.h"

#define PEEK_LAYER_UNFOLD_DURATION 500
#define PEEK_LAYER_SCALE_DURATION 300

#define MAX_PEEK_LAYER_TEXT_LEN 40
#define MAX_PEEK_LAYER_NUMBER_LEN 10

//! Icon position adjustment applied to user given frames
//! Usually user frames are the window bounds, so this moves icons slightly above center
#define PEEK_LAYER_ICON_OFFSET_Y (-10)

//! The spacing between the subtitle and title text fields
#define PEEK_LAYER_SUBTITLE_MARGIN PBL_IF_RECT_ELSE(1, -2)

typedef struct {
  TextLayer text_layer;
  char text_buffer[MAX_PEEK_LAYER_TEXT_LEN];
} PeekTextLayer;

//! A Peek Layer unfolds an icon in full view to give the user context or a peek of the content
//! that will eventually enter the screen.
typedef struct {
  Layer layer;
  GColor bg_color;
  KinoLayer kino_layer;
  PeekTextLayer number;
  PeekTextLayer title;
  PeekTextLayer subtitle;
  AppResourceInfo res_info;
  EventedTimerID hidden_fields_timer;
  int16_t icon_offset_y;
  int16_t subtitle_margin;
  uint8_t dot_diameter;
  bool show_dot;
} PeekLayer;


//! Create a peek layer with a frame.
PeekLayer *peek_layer_create(GRect frame);

//! Destroy a peek layer.
void peek_layer_destroy(PeekLayer *peek_layer);

//! Initialize a peek layer with a frame.
void peek_layer_init(PeekLayer *peek_layer, const GRect *frame);

//! Deinit a peek layer.
void peek_layer_deinit(PeekLayer *peek_layer);

//! Set the frame of the peek layer.
void peek_layer_set_frame(PeekLayer *peek_layer, const GRect *frame);

//! Set the peek layer with a PDCI resource.
//! The peek layer will be primed with an unfold animation.
//! The resource will begin as a dot until the peek layer is played.
void peek_layer_set_icon(PeekLayer *peek_layer, const TimelineResourceInfo *timeline_res);
void peek_layer_set_icon_with_invert(PeekLayer *peek_layer,
                                     const TimelineResourceInfo *timeline_res, bool invert);
void peek_layer_set_icon_with_size(PeekLayer *peek_layer, const TimelineResourceInfo *timeline_res,
                                   TimelineResourceSize res_size, GRect icon_from);

//! Set the peek layer to have a stretching animation to a frame.
void peek_layer_set_scale_to(PeekLayer *peek_layer, GRect icon_to);
//! Same as peek_layer_set_scale_to, but with an image. If align_in_frame is true, scale the
//! image to the resource size and align within icon_to instead of scaling to the icon_to size.
void peek_layer_set_scale_to_image(PeekLayer *peek_layer, const TimelineResourceInfo *timeline_res,
                                   TimelineResourceSize res_size, GRect icon_to,
                                   bool align_in_frame);

//! Set the duration of the primed animation in milliseconds.
void peek_layer_set_duration(PeekLayer *peek_layer, uint32_t duration);

//! Play the primed animation of the peek layer.
void peek_layer_play(PeekLayer *peek_layer);

//! Get the size of the primed animation reel.
GSize peek_layer_get_size(PeekLayer *peek_layer);

//! Create the primed animation of the peek layer.
ImmutableAnimation *peek_layer_create_play_animation(PeekLayer *peek_layer);

//! Create a section of the primed animation of the peek layer.
ImmutableAnimation *peek_layer_create_play_section_animation(PeekLayer *peek_layer,
                                                             uint32_t from_elapsed_ms,
                                                             uint32_t to_elapsed_ms);

//! Set the background color of the peek layer.
void peek_layer_set_background_color(PeekLayer *peek_layer, GColor color);

//! Sets the text of the peek layer text fields. The text is copied over.
//! See the individual text field setters for more information about each field.
void peek_layer_set_fields(PeekLayer *peek_layer, const char *number, const char *title,
                           const char *subtitle);

//! Clears all text of the peek layer.
//! Equivalent to calling set fields with an empty string for each field.
void peek_layer_clear_fields(PeekLayer *peek_layer);

//! Hides visibility of the fields while retaining the text.
void peek_layer_set_fields_hidden(PeekLayer *peek_layer, bool hidden);

//! Set the peek layer number text. The text is copied over.
//! If the title starts with a number, such as in "5 MIN.", number should be used in conjunction
//! with the title rather than including the number in the title text. It is positioned to the left
//! of the title in the same line, and together they are horizontally centered. Its font size is
//! comparable to the title and is larger than the subtitle.
void peek_layer_set_number(PeekLayer *peek_layer, const char *number);

//! Set the peek layer title text. The text is copied over.
//! The title is suitable for most use cases, appearing as dialog text in a dialog.
//! Its default font size is larger than the subtitle, but can also be configured to any other font
//! with \ref peek_layer_set_title_font.
void peek_layer_set_title(PeekLayer *peek_layer, const char *title);

//! Set the peek layer subtitle text. The text is copied over.
//! The subtitle is for providing additional context that the user may wish to have.
//! It is positioned above the title, and has a font size smaller than the title.
void peek_layer_set_subtitle(PeekLayer *peek_layer, const char *subtitle);

//! Set the title font of the peek layer.
void peek_layer_set_title_font(PeekLayer *peek_layer, GFont font);

//! Set the subtitle font of the peek layer.
//! @param peek_layer The peek layer to set the subtitle of.
//! @param font The new subtitle font to use.
//! @param margin The new margin to use for spacing after the subtitle.
void peek_layer_set_subtitle_font(PeekLayer *peek_layer, GFont font, int16_t margin);

//! Set the dot diameter of the peek layer.
void peek_layer_set_dot_diameter(PeekLayer *peek_layer, uint8_t dot_diameter);

//! Set the icon offset y of the peek layer.
void peek_layer_set_icon_offset_y(PeekLayer *peek_layer, int16_t icon_offset_y);
