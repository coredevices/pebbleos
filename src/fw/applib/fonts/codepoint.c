/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "codepoint.h"

#include "pbl/util/size.h"

#include <stddef.h>

#define MAX_LATIN_CODEPOINT 0x02AF
#define MIN_ARABIC_CODEPOINT 0x0600
#define MAX_ARABIC_CODEPOINT 0x06FF
#define MIN_HEBREW_CODEPOINT 0x0590
#define MAX_HEBREW_CODEPOINT 0x05FF
#define MIN_UNIFIED_EMOJI_CODEPOINT 0x1F300
#define MAX_UNIFIED_EMOJI_CODEPOINT 0x1FAFF
#define MIN_MISC_TECHNICAL_CODEPOINT 0x2300
#define MAX_MISC_TECHNICAL_CODEPOINT 0x23FF
#define MIN_MISC_SYMBOLS_CODEPOINT 0x2600
#define MAX_MISC_SYMBOLS_CODEPOINT 0x27BF
#define MIN_MISC_ARROWS_CODEPOINT 0x2B00
#define MAX_MISC_ARROWS_CODEPOINT 0x2BFF
#define MIN_ENCLOSED_EMOJI_CODEPOINT 0x1F100
#define MAX_ENCLOSED_EMOJI_CODEPOINT 0x1F2FF
#define MIN_REGIONAL_INDICATOR_CODEPOINT 0x1F1E6
#define MAX_REGIONAL_INDICATOR_CODEPOINT 0x1F1FF
#define MIN_SYMBOLS_CODEPOINT 0x2000
#define MAX_SYMBOLS_CODEPOINT 0x2BFF
#define MIN_IDEOGRAPH_CODEPOINT 0x2e80
#define MIN_SPECIAL_CODEPOINT 0xE0A0
#define MAX_SPECIAL_CODEPOINT 0xE0A2
#define MIN_SKIN_TONE_CODEPOINT 0x1F3FB
#define MAX_SKIN_TONE_CODEPOINT 0x1F3FF

// Note: Please keep these sorted. These are emoji-presentation codepoints outside the
// ranges checked in codepoint_is_emoji(). Geometric Shapes (25xx) are listed one by one
// because that block also holds glyphs the base fonts carry (the lozenge 25CA and the
// wildcard box 25AF), which must keep resolving to the base font.
static const Codepoint NONSTANDARD_EMOJI_CODEPOINTS[] = {
    0x2192, // rightwards_arrow
    0x25AA, // black_small_square
    0x25AB, // white_small_square
    0x25B6, // black_right_pointing_triangle
    0x25BA, // black_right_pointing_pointer
    0x25C0, // black_left_pointing_triangle
    0x25FB, // white_medium_square
    0x25FC, // black_medium_square
    0x25FD, // white_medium_small_square
    0x25FE, // black_medium_small_square
};

// Note: Please keep these sorted
// Unicode TR14 line break classes: spaces with class BA (Break After) allow line breaks.
// Non-breaking spaces (class GL): U+00A0, U+2007, U+202F are intentionally excluded.
static const Codepoint END_OF_WORD_CODEPOINTS[] = {
  NULL_CODEPOINT, // 0x0
  NEWLINE_CODEPOINT, // 0xa
  SPACE_CODEPOINT, // 0x20
  HYPHEN_CODEPOINT, // 0x2d
  EN_QUAD_CODEPOINT, // 0x2000 (BA)
  EM_QUAD_CODEPOINT, // 0x2001 (BA)
  EN_SPACE_CODEPOINT, // 0x2002 (BA)
  EM_SPACE_CODEPOINT, // 0x2003 (BA)
  THREE_PER_EM_SPACE_CODEPOINT, // 0x2004 (BA)
  FOUR_PER_EM_SPACE_CODEPOINT, // 0x2005 (BA)
  SIX_PER_EM_SPACE_CODEPOINT, // 0x2006 (BA)
  PUNCTUATION_SPACE_CODEPOINT, // 0x2008 (BA)
  THIN_SPACE_CODEPOINT, // 0x2009 (BA)
  HAIR_SPACE_CODEPOINT, // 0x200a (BA)
  ZERO_WIDTH_SPACE_CODEPOINT, // 0x200b
  MEDIUM_MATHEMATICAL_SPACE_CODEPOINT, // 0x205f (BA)
  IDEOGRAPHIC_SPACE_CODEPOINT, // 0x3000 (BA)
};

//  Note: Please keep these sorted
static const Codepoint FORMATTING_CODEPOINTS[] = {
  0x7F,   // delete
  0x200C, // zero-width non-joiner
  0x200D, // zero-width joiner
  0x200E, // left to right
  0x200F, // right to left
  0x202A, // bidirectional - right to left
  0x202B, // bidirectional - right to left embedding
  0x202C, // bidirectional - pop direction
  0x202D, // left to right override
  0x202E, // right to left override
  0x20E3, // combining enclosing keycap; no font has the ring, keep the base digit
  0xFE0E, // variation selector 1
  0xFE0F, // variation selector 2
  0xFEFF, // zero-width-no-break
  0xFFFC, // object replacement character
};

// Note: Please keep these sorted
static const Codepoint ZERO_WIDTH_CODEPOINTS[] = {
  ZERO_WIDTH_SPACE_CODEPOINT,
  WORD_JOINER_CODEPOINT,
};

static const Codepoint THAI_COMBINING_MARKS[] = {
  0x0E31,
  0x0E34, 0x0E35, 0x0E36, 0x0E37,
  0x0E3A,
  0x0E47, 0x0E48, 0x0E49, 0x0E4A, 0x0E4B, 0x0E4C, 0x0E4D, 0x0E4E,
};

static bool codepoint_in_list(const Codepoint codepoint, const Codepoint *codepoints, size_t size) {
  for (size_t i = 0; i < size; ++i) {
    if (codepoints[i] >= codepoint) {
      return (codepoints[i] == codepoint);
    }
  }

  return false;
}

bool codepoint_is_unicode_space(const Codepoint codepoint) {
  switch (codepoint) {
    case NO_BREAK_SPACE_CODEPOINT:
    case EN_QUAD_CODEPOINT:
    case EM_QUAD_CODEPOINT:
    case EN_SPACE_CODEPOINT:
    case EM_SPACE_CODEPOINT:
    case THREE_PER_EM_SPACE_CODEPOINT:
    case FOUR_PER_EM_SPACE_CODEPOINT:
    case SIX_PER_EM_SPACE_CODEPOINT:
    case FIGURE_SPACE_CODEPOINT:
    case PUNCTUATION_SPACE_CODEPOINT:
    case THIN_SPACE_CODEPOINT:
    case HAIR_SPACE_CODEPOINT:
    case NARROW_NO_BREAK_SPACE_CODEPOINT:
    case MEDIUM_MATHEMATICAL_SPACE_CODEPOINT:
    case IDEOGRAPHIC_SPACE_CODEPOINT:
      return true;
    default:
      return false;
  }
}

bool codepoint_is_formatting_indicator(const Codepoint codepoint) {
  return codepoint_in_list(codepoint, FORMATTING_CODEPOINTS, ARRAY_LENGTH(FORMATTING_CODEPOINTS));
}

bool codepoint_is_ideograph(const Codepoint codepoint) {
  if (codepoint > MIN_IDEOGRAPH_CODEPOINT) {
    // non ideographic characters. This is an approximation that is good enough until
    // we start supporting some exotic scripts (e.g. tibetan)
    return true;
  } else {
    return false;
  }
}

// see http://www.unicode.org/reports/tr14/ for the whole enchilada
bool codepoint_is_end_of_word(const Codepoint codepoint) {
  return codepoint_in_list(codepoint, END_OF_WORD_CODEPOINTS,
                           ARRAY_LENGTH(END_OF_WORD_CODEPOINTS));
}

// see http://unicode.org/reports/tr51/ section 2.2 "Diversity"
bool codepoint_is_skin_tone_modifier(const Codepoint codepoint) {
  return (codepoint >= MIN_SKIN_TONE_CODEPOINT && codepoint <= MAX_SKIN_TONE_CODEPOINT);
}

bool codepoint_should_skip(const Codepoint codepoint) {
  return ((codepoint < 0x20 && codepoint != NEWLINE_CODEPOINT) ||
          (codepoint >= 0x80 && codepoint <= 0x9F) ||
          (codepoint >= 0x2061 && codepoint <= 0x206F) ||
          (codepoint >= 0xFFF9 && codepoint <= 0xFFFB) ||
          (codepoint >= 0xE0000 && codepoint <= 0xE007F) ||
          (codepoint >= 0xF0000 && codepoint <= 0x10FFFD) ||
          (codepoint_is_skin_tone_modifier(codepoint)));
}

bool codepoint_is_zero_width(const Codepoint codepoint) {
  return codepoint_in_list(codepoint, ZERO_WIDTH_CODEPOINTS, ARRAY_LENGTH(ZERO_WIDTH_CODEPOINTS));
}

bool codepoint_is_zero_advance(const Codepoint codepoint) {
  return codepoint_in_list(codepoint, THAI_COMBINING_MARKS, ARRAY_LENGTH(THAI_COMBINING_MARKS));
}

bool codepoint_is_latin(const Codepoint codepoint) {
  return (codepoint <= MAX_LATIN_CODEPOINT ||
         (codepoint >= MIN_SYMBOLS_CODEPOINT &&
          codepoint <= MAX_SYMBOLS_CODEPOINT));
}

bool codepoint_is_emoji(const Codepoint codepoint) {
  // search for the codepoint in the list of nonstandard emoji codepoints first.
  const bool found = codepoint_in_list(codepoint,
                                       NONSTANDARD_EMOJI_CODEPOINTS,
                                       ARRAY_LENGTH(NONSTANDARD_EMOJI_CODEPOINTS));
  if (found) {
    return true;
  }
  return (codepoint >= MIN_UNIFIED_EMOJI_CODEPOINT &&
          codepoint <= MAX_UNIFIED_EMOJI_CODEPOINT) ||
         (codepoint >= MIN_MISC_TECHNICAL_CODEPOINT &&
          codepoint <= MAX_MISC_TECHNICAL_CODEPOINT) ||
         (codepoint >= MIN_MISC_SYMBOLS_CODEPOINT &&
          codepoint <= MAX_MISC_SYMBOLS_CODEPOINT) ||
         (codepoint >= MIN_MISC_ARROWS_CODEPOINT &&
          codepoint <= MAX_MISC_ARROWS_CODEPOINT) ||
         (codepoint >= MIN_ENCLOSED_EMOJI_CODEPOINT &&
          codepoint <= MAX_ENCLOSED_EMOJI_CODEPOINT);
}

bool codepoint_is_regional_indicator(const Codepoint codepoint) {
  return (codepoint >= MIN_REGIONAL_INDICATOR_CODEPOINT &&
          codepoint <= MAX_REGIONAL_INDICATOR_CODEPOINT);
}

Codepoint emoji_shape_pair(const Codepoint curr_cp, const Codepoint next_cp,
                           bool *consumed_next) {
  *consumed_next = false;

  // A flag is a pair of regional indicators; the emoji font carries no
  // per-country glyphs, so both draw as one generic flag.
  if (codepoint_is_regional_indicator(curr_cp)) {
    *consumed_next = codepoint_is_regional_indicator(next_cp);
    return FLAG_CODEPOINT;
  }

  return curr_cp;
}

bool codepoint_is_special(const Codepoint codepoint) {
  return (codepoint >= MIN_SPECIAL_CODEPOINT &&
          codepoint <= MAX_SPECIAL_CODEPOINT);
}

bool codepoint_is_rtl(const Codepoint codepoint) {
  return (codepoint >= MIN_ARABIC_CODEPOINT && codepoint <= MAX_ARABIC_CODEPOINT) ||
         (codepoint >= MIN_HEBREW_CODEPOINT && codepoint <= MAX_HEBREW_CODEPOINT);
}
