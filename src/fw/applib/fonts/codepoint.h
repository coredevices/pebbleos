/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef uint32_t Codepoint;

#define EM_DASH "—"
#define EN_DASH "–"

#define ELLIPSIS_CODEPOINT 0x2026
#define HYPHEN_CODEPOINT 0x002D
#define MINUS_SIGN_CODEPOINT 0x2212
#define SPACE_CODEPOINT ' '
#define NEWLINE_CODEPOINT '\n'
#define NULL_CODEPOINT '\0'
#define NO_BREAK_SPACE_CODEPOINT 0x00A0
#define EN_QUAD_CODEPOINT 0x2000
#define EM_QUAD_CODEPOINT 0x2001
#define EN_SPACE_CODEPOINT 0x2002
#define EM_SPACE_CODEPOINT 0x2003
#define THREE_PER_EM_SPACE_CODEPOINT 0x2004
#define FOUR_PER_EM_SPACE_CODEPOINT 0x2005
#define SIX_PER_EM_SPACE_CODEPOINT 0x2006
#define FIGURE_SPACE_CODEPOINT 0x2007
#define PUNCTUATION_SPACE_CODEPOINT 0x2008
#define THIN_SPACE_CODEPOINT 0x2009
#define HAIR_SPACE_CODEPOINT 0x200A
#define ZERO_WIDTH_SPACE_CODEPOINT 0x200B
#define NARROW_NO_BREAK_SPACE_CODEPOINT 0x202F
#define MEDIUM_MATHEMATICAL_SPACE_CODEPOINT 0x205F
#define WORD_JOINER_CODEPOINT 0x2060
#define IDEOGRAPHIC_SPACE_CODEPOINT 0x3000
// Generic substitute for regional-indicator flag pairs (white flag)
#define FLAG_CODEPOINT 0x1F3F3

bool codepoint_is_unicode_space(const Codepoint codepoint);

bool codepoint_is_formatting_indicator(const Codepoint codepoint);

bool codepoint_is_skin_tone_modifier(const Codepoint codepoint);

bool codepoint_is_end_of_word(const Codepoint codepoint);

bool codepoint_is_ideograph(const Codepoint codepoint);

bool codepoint_should_skip(const Codepoint codepoint);

bool codepoint_is_zero_width(const Codepoint codepoint);

bool codepoint_is_zero_advance(const Codepoint codepoint);

bool codepoint_is_latin(const Codepoint codepoint);

bool codepoint_is_emoji(const Codepoint codepoint);

// Regional indicator symbol (U+1F1E6..U+1F1FF); a pair encodes a flag
bool codepoint_is_regional_indicator(const Codepoint codepoint);

// Fold a multi-codepoint emoji sequence into one glyph codepoint. A
// regional-indicator pair (a flag) maps to the generic flag codepoint, and a
// stray single indicator maps to the same. Returns the replacement codepoint
// for curr_cp and sets *consumed_next when next_cp is folded into it; returns
// curr_cp unchanged otherwise. Callers must give a consumed codepoint zero
// width and must not draw it.
Codepoint emoji_shape_pair(const Codepoint curr_cp, const Codepoint next_cp,
                           bool *consumed_next);

// This is a least dirty hack to enable special rendering when a special codepoint is hit in the
// text being rendered
bool codepoint_is_special(const Codepoint codepoint);

// Check if a codepoint is from a right-to-left script (Arabic, Hebrew)
bool codepoint_is_rtl(const Codepoint codepoint);
