/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

//! Private layout interface (ie for unit testing)

#include "pbl/util/iterator.h"
#include "applib/fonts/codepoint.h"
#include "text.h"
#include "gtypes.h"
#include "utf8.h"

#include <stdint.h>

typedef struct {
  const Utf8Bounds* utf8_bounds; //<! start and end of utf-8 codepoints
  GRect box;
  GFont font;
  GTextOverflowMode overflow_mode;
  GTextAlignment alignment;
  int16_t line_spacing_delta;
} TextBoxParams;

//! Parameters required to render a line 
typedef struct {
  utf8_t* start;
  GPoint origin; //<! Relative to text_box_params origin
  int16_t height_px;
  int16_t width_px;
  int16_t max_width_px; //<! Maximum length of the line
  Codepoint suffix_codepoint;
} Line;

//! Definition of a word:
//!  "A brown   dog\njumps" becomes:
//!   - "A"
//!   - " brown" // whitespace is trimmed if word wraps
//!   - "   dog" // whitespace is trimmed if word wraps
//!   - "\n"
//!   - "jumps"
//!
//!   - Word start points to first printable codepoint in word, inclusive,
//!     including whitespace
//!   - Word end points to codepoint after the last printable codepoint in a word,
//!     excluding whitespace (eg, end of word, exclusive); note this codepoint may
//!     not be valid since it may be the end of the string
//!   - The preceding whitespace of a word is trimmed if the word wraps
//!   - Reserved codepoints are skipped
//!   - Newlines are treated as stand-alone words so as to not mess up the height
//!     and width word metrics
typedef struct {
  utf8_t* start;
  utf8_t* end;
  int16_t width_px;
} Word;

#define WORD_EMPTY ((Word){ 0, 0, 0 })

typedef struct {
  const TextBoxParams* text_box_params;
  Iterator utf8_iter;
  Utf8IterState utf8_iter_state;
} CharIterState;

//! Uses character iterator to iterate over characters
typedef struct {
  GContext* ctx;
  const TextBoxParams* text_box_params;
  Word current;
} WordIterState;

#define WORD_ITER_STATE_EMPTY ((WordIterState){ 0, 0, WORD_EMPTY })

typedef struct {
  GContext *ctx;
  Line *current;
  Iterator word_iter;
  WordIterState word_iter_state;
} LineIterState;

typedef struct {
  TextBoxParams text_box;
  Line line;
  LineIterState line_iter_state;
} TextDrawState;

void char_iter_init(Iterator* char_iter, CharIterState* char_iter_state, const TextBoxParams* const text_box_params, utf8_t* start);
void word_iter_init(Iterator* word_iter, WordIterState* word_iter_state, GContext* ctx, const TextBoxParams* const text_box_params, utf8_t* start);
void line_iter_init(Iterator* line_iter, LineIterState* line_iter_state, GContext* ctx);

bool word_init(GContext* ctx, Word* word, const TextBoxParams* const text_box_params, utf8_t* start);

bool char_iter_next(IteratorState state);
bool char_iter_prev(IteratorState state);
bool word_iter_next(IteratorState state);
bool line_iter_next(IteratorState state);

typedef void (*LastLineCallback)(GContext* ctx, Line* line,
                                 const TextBoxParams* const text_box_params,
                                 const bool is_text_remaining);
typedef void (*RenderLineCallback)(GContext* ctx, Line* line,
                                   const TextBoxParams* const text_box_params);
typedef void (*LayoutUpdateCallback)(TextLayout* layout, Line* line,
                                     const TextBoxParams* const text_box_params);
typedef bool (*StopConditionCallback)(GContext* ctx, Line* line,
                                      const TextBoxParams* const text_box_params);

bool line_add_word(GContext* ctx, Line* line, Word* word, const TextBoxParams* const text_box_params);
bool line_add_words(Line* line, Iterator* word_iter, LastLineCallback last_line_cb);

typedef struct {
  LastLineCallback last_line_cb;
  RenderLineCallback render_line_cb;
  LayoutUpdateCallback layout_update_cb;
  StopConditionCallback stop_condition_cb;
} WalkLinesCallbacks;

#define WALK_LINE_CALLBACKS_EMPTY ((WalkLinesCallbacks){ 0, 0, 0, 0 })

