/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/fonts/codepoint.h"

#include "clar.h"

///////////////////////////////////////////////////////////
// Stubs

#include "stubs_logging.h"
#include "stubs_passert.h"

///////////////////////////////////////////////////////////
// Tests

void test_codepoint__initialize(void) {
}

void test_codepoint__cleanup(void) {
}

void test_codepoint__is_unicode_space(void) {
  // All Unicode space variants should be recognized
  cl_assert(codepoint_is_unicode_space(NO_BREAK_SPACE_CODEPOINT));
  cl_assert(codepoint_is_unicode_space(EN_QUAD_CODEPOINT));
  cl_assert(codepoint_is_unicode_space(EM_QUAD_CODEPOINT));
  cl_assert(codepoint_is_unicode_space(EN_SPACE_CODEPOINT));
  cl_assert(codepoint_is_unicode_space(EM_SPACE_CODEPOINT));
  cl_assert(codepoint_is_unicode_space(THREE_PER_EM_SPACE_CODEPOINT));
  cl_assert(codepoint_is_unicode_space(FOUR_PER_EM_SPACE_CODEPOINT));
  cl_assert(codepoint_is_unicode_space(SIX_PER_EM_SPACE_CODEPOINT));
  cl_assert(codepoint_is_unicode_space(FIGURE_SPACE_CODEPOINT));
  cl_assert(codepoint_is_unicode_space(PUNCTUATION_SPACE_CODEPOINT));
  cl_assert(codepoint_is_unicode_space(THIN_SPACE_CODEPOINT));
  cl_assert(codepoint_is_unicode_space(HAIR_SPACE_CODEPOINT));
  cl_assert(codepoint_is_unicode_space(NARROW_NO_BREAK_SPACE_CODEPOINT));
  cl_assert(codepoint_is_unicode_space(MEDIUM_MATHEMATICAL_SPACE_CODEPOINT));
  cl_assert(codepoint_is_unicode_space(IDEOGRAPHIC_SPACE_CODEPOINT));

  // Regular space and other characters should not match
  cl_assert(!codepoint_is_unicode_space(SPACE_CODEPOINT));
  cl_assert(!codepoint_is_unicode_space('A'));
  cl_assert(!codepoint_is_unicode_space(ZERO_WIDTH_SPACE_CODEPOINT));
  cl_assert(!codepoint_is_unicode_space(NEWLINE_CODEPOINT));
  cl_assert(!codepoint_is_unicode_space(ELLIPSIS_CODEPOINT));
}

void test_codepoint__thai_combining_marks_are_zero_width(void) {
  cl_assert(codepoint_is_zero_advance(0x0E31));
  cl_assert(codepoint_is_zero_advance(0x0E35));
  cl_assert(codepoint_is_zero_advance(0x0E48));
  cl_assert(codepoint_is_zero_advance(0x0E4E));
  cl_assert(!codepoint_is_zero_width(0x0E35));
  cl_assert(!codepoint_is_zero_width(0x0E30));
  cl_assert(!codepoint_is_zero_width(0x0E33));
  cl_assert(!codepoint_is_zero_width(0x0E40));
  cl_assert(!codepoint_is_zero_width(0x0E4F));
  cl_assert(!codepoint_is_zero_advance(0x0E30));
  cl_assert(!codepoint_is_zero_advance(0x0E33));
  cl_assert(!codepoint_is_zero_advance(0x0E40));
  cl_assert(!codepoint_is_zero_advance(0x0E4F));
}

void test_codepoint__end_of_word_breakable_spaces(void) {
  // Unicode spaces with TR14 class BA (Break After) should be word breaks
  cl_assert(codepoint_is_end_of_word(EN_QUAD_CODEPOINT));
  cl_assert(codepoint_is_end_of_word(EM_QUAD_CODEPOINT));
  cl_assert(codepoint_is_end_of_word(EN_SPACE_CODEPOINT));
  cl_assert(codepoint_is_end_of_word(EM_SPACE_CODEPOINT));
  cl_assert(codepoint_is_end_of_word(THREE_PER_EM_SPACE_CODEPOINT));
  cl_assert(codepoint_is_end_of_word(FOUR_PER_EM_SPACE_CODEPOINT));
  cl_assert(codepoint_is_end_of_word(SIX_PER_EM_SPACE_CODEPOINT));
  cl_assert(codepoint_is_end_of_word(PUNCTUATION_SPACE_CODEPOINT));
  cl_assert(codepoint_is_end_of_word(THIN_SPACE_CODEPOINT));
  cl_assert(codepoint_is_end_of_word(HAIR_SPACE_CODEPOINT));
  cl_assert(codepoint_is_end_of_word(MEDIUM_MATHEMATICAL_SPACE_CODEPOINT));
  cl_assert(codepoint_is_end_of_word(IDEOGRAPHIC_SPACE_CODEPOINT));
}

void test_codepoint__end_of_word_non_breaking_spaces(void) {
  // Non-breaking spaces (TR14 class GL) must NOT be word breaks
  cl_assert(!codepoint_is_end_of_word(NO_BREAK_SPACE_CODEPOINT));
  cl_assert(!codepoint_is_end_of_word(FIGURE_SPACE_CODEPOINT));
  cl_assert(!codepoint_is_end_of_word(NARROW_NO_BREAK_SPACE_CODEPOINT));
}

void test_codepoint__end_of_word_existing(void) {
  // Existing word break characters should still work
  cl_assert(codepoint_is_end_of_word(NULL_CODEPOINT));
  cl_assert(codepoint_is_end_of_word(NEWLINE_CODEPOINT));
  cl_assert(codepoint_is_end_of_word(SPACE_CODEPOINT));
  cl_assert(codepoint_is_end_of_word(HYPHEN_CODEPOINT));
  cl_assert(codepoint_is_end_of_word(ZERO_WIDTH_SPACE_CODEPOINT));

  // Regular characters should not be word breaks
  cl_assert(!codepoint_is_end_of_word('A'));
  cl_assert(!codepoint_is_end_of_word('0'));
}

void test_codepoint__should_skip_controls(void) {
  // C0 controls (except newline) should be skipped
  cl_assert(codepoint_should_skip(NULL_CODEPOINT));
  cl_assert(codepoint_should_skip(0x01));
  cl_assert(codepoint_should_skip(0x07));  // bell
  cl_assert(codepoint_should_skip(0x09));  // tab
  cl_assert(codepoint_should_skip(0x0D));  // carriage return
  cl_assert(codepoint_should_skip(0x1F));

  // Newline must NOT be skipped (it's a line break)
  cl_assert(!codepoint_should_skip(NEWLINE_CODEPOINT));

  // C1 controls (U+0080-U+009F) should be skipped — they have no visible glyphs
  // and otherwise render as the font's wildcard box.
  cl_assert(codepoint_should_skip(0x80));
  cl_assert(codepoint_should_skip(0x85));  // NEL
  cl_assert(codepoint_should_skip(0x9F));

  // Word-Joiner block (U+2061-U+206F): invisible math operators, BiDi isolates,
  // deprecated formatting — none have visible glyphs in our fonts.
  // U+2060 WORD JOINER is excluded — it is needed for CJK word segmentation.
  cl_assert(!codepoint_should_skip(0x2060));  // word joiner — handled separately
  cl_assert(codepoint_should_skip(0x2062));  // invisible times
  cl_assert(codepoint_should_skip(0x2066));  // LRI
  cl_assert(codepoint_should_skip(0x2069));  // PDI
  cl_assert(codepoint_should_skip(0x206F));  // nominal digit shapes (deprecated)

  // Interlinear annotation anchors and tag characters
  cl_assert(codepoint_should_skip(0xFFF9));  // interlinear anchor
  cl_assert(codepoint_should_skip(0xFFFB));  // interlinear terminator
  cl_assert(codepoint_should_skip(0xE0001));  // language tag
  cl_assert(codepoint_should_skip(0xE007F));  // cancel tag

  // Supplementary Private Use Area-B (Apple SF Symbols) should be skipped
  cl_assert(codepoint_should_skip(0xF0000));
  cl_assert(codepoint_should_skip(0x100000));  // start of typical SF Symbols range
  cl_assert(codepoint_should_skip(0x10FFFD));

  // Boundary characters around C1 must NOT be skipped
  cl_assert(!codepoint_should_skip(0x7F));  // DEL — handled by formatting indicator
  cl_assert(!codepoint_should_skip(0xA0));  // no-break space — printable
  cl_assert(!codepoint_should_skip(' '));
  cl_assert(!codepoint_should_skip('A'));

  // Codepoints just outside the new ranges must NOT be skipped
  cl_assert(!codepoint_should_skip(0x205F));  // medium math space — printable
  cl_assert(!codepoint_should_skip(0x2070));  // superscript zero — printable
  cl_assert(!codepoint_should_skip(0xFFF8));  // unassigned, but not in our skip range
  cl_assert(!codepoint_should_skip(0xFFFC));  // object replacement — handled by formatting indicator
  cl_assert(!codepoint_should_skip(0xEFFFF));
}

void test_codepoint__formatting_indicator_invisibles(void) {
  // Object replacement character (used by iOS for inline attachments) must be
  // treated as a formatting indicator so it doesn't render as a tofu box.
  cl_assert(codepoint_is_formatting_indicator(0xFFFC));
  // BiDi controls — full U+202A-U+202E block including the previously-missed
  // RLE (U+202B) and RLO (U+202E)
  cl_assert(codepoint_is_formatting_indicator(0x202A));
  cl_assert(codepoint_is_formatting_indicator(0x202B));
  cl_assert(codepoint_is_formatting_indicator(0x202C));
  cl_assert(codepoint_is_formatting_indicator(0x202D));
  cl_assert(codepoint_is_formatting_indicator(0x202E));
  // Combining enclosing keycap (1️⃣ = digit + FE0F + 20E3): no font has the
  // ring, so it must vanish and leave the plain digit.
  cl_assert(codepoint_is_formatting_indicator(0x20E3));
  // Existing entries still recognised
  cl_assert(codepoint_is_formatting_indicator(0x7F));
  cl_assert(codepoint_is_formatting_indicator(0xFEFF));
  // Unrelated codepoints are not
  cl_assert(!codepoint_is_formatting_indicator('A'));
  cl_assert(!codepoint_is_formatting_indicator(0xFFFD));  // replacement char
}

// A regional-indicator pair (a flag) folds into the generic flag codepoint and
// consumes the second indicator.
void test_codepoint__flag_pair_folds(void) {
  bool consumed = false;
  // Regional indicators U (0x1F1FA) and S (0x1F1F8), the 🇺🇸 pair
  Codepoint cp = emoji_shape_pair(0x1F1FA, 0x1F1F8, &consumed);
  cl_assert_equal_i(cp, FLAG_CODEPOINT);
  cl_assert(consumed);
}

// A stray single indicator maps to the flag codepoint without consuming the
// codepoint after it.
void test_codepoint__stray_indicator_maps_to_flag(void) {
  bool consumed = true;
  Codepoint cp = emoji_shape_pair(0x1F1E6, ' ', &consumed);
  cl_assert_equal_i(cp, FLAG_CODEPOINT);
  cl_assert(!consumed);
}

// Anything else passes through untouched.
void test_codepoint__emoji_shape_pair_passthrough(void) {
  bool consumed = true;
  Codepoint cp = emoji_shape_pair('A', 0x1F1E6, &consumed);
  cl_assert_equal_i(cp, 'A');
  cl_assert(!consumed);

  cp = emoji_shape_pair(0x1F600, 0x1F1E6, &consumed);  // non-indicator emoji
  cl_assert_equal_i(cp, 0x1F600);
  cl_assert(!consumed);
}
