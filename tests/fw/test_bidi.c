/* SPDX-FileCopyrightText: 2026 Khalid Nuaim (kaluaim) */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/graphics/bidi.h"

#include "clar.h"

#include <string.h>

// Stubs
///////////////////////////////////////////////////////////
#include "stubs_logging.h"
#include "stubs_passert.h"

// Helpers
///////////////////////////////////////////////////////////

typedef struct {
  size_t offset;
  size_t length;
  bool is_rtl;
} ExpectedRun;

//! Split `text` into runs the way the render path does and compare the result
//! against the expected offsets, lengths and directions.
static void prv_assert_runs(const char *text, const ExpectedRun *expected, int num_expected) {
  utf8_t *start = (utf8_t *)text;
  const utf8_t *end = start + strlen(text);
  const bool para_is_rtl = bidi_paragraph_is_rtl(start, end);

  utf8_t *pos = start;
  int index = 0;
  while (pos < end) {
    bool run_is_rtl = false;
    utf8_t *run_end = bidi_next_run(start, pos, end, para_is_rtl, &run_is_rtl);
    cl_assert(run_end > pos);
    cl_assert_lt(index, num_expected);
    cl_assert_equal_i((int)(pos - start), (int)expected[index].offset);
    cl_assert_equal_i((int)(run_end - pos), (int)expected[index].length);
    cl_assert_equal_b(run_is_rtl, expected[index].is_rtl);
    pos = run_end;
    index++;
  }
  cl_assert_equal_i(index, num_expected);
}

static void prv_assert_reversed(const char *text, const char *expected) {
  utf8_t buffer[64];
  const size_t length = bidi_reverse_run((const utf8_t *)text, strlen(text),
                                         buffer, sizeof(buffer));
  cl_assert_equal_i((int)length, (int)strlen(expected));
  cl_assert_equal_s((const char *)buffer, expected);
}

// Tests
///////////////////////////////////////////////////////////

void test_bidi__not_needed_for_latin_text(void) {
  const char *text = "Hello (world) 123";
  cl_assert(!bidi_is_needed((const utf8_t *)text, (const utf8_t *)text + strlen(text)));

  const char *accented = "Café (naïve)";
  cl_assert(!bidi_is_needed((const utf8_t *)accented, (const utf8_t *)accented + strlen(accented)));
}

void test_bidi__needed_for_arabic_and_hebrew(void) {
  const char *arabic = "Hello مرحبا";
  cl_assert(bidi_is_needed((const utf8_t *)arabic, (const utf8_t *)arabic + strlen(arabic)));

  const char *hebrew = "Hello שלום";
  cl_assert(bidi_is_needed((const utf8_t *)hebrew, (const utf8_t *)hebrew + strlen(hebrew)));
}

void test_bidi__paragraph_direction(void) {
  const char *rtl = "الرسالة (Hello) وصلت";
  cl_assert(bidi_paragraph_is_rtl((const utf8_t *)rtl, (const utf8_t *)rtl + strlen(rtl)));

  // Digits and punctuation are not strong, so the Arabic word decides.
  const char *leading_digits = "123 مرحبا";
  cl_assert(bidi_paragraph_is_rtl((const utf8_t *)leading_digits,
                                  (const utf8_t *)leading_digits + strlen(leading_digits)));

  const char *ltr = "Say (שלום) now";
  cl_assert(!bidi_paragraph_is_rtl((const utf8_t *)ltr, (const utf8_t *)ltr + strlen(ltr)));

  const char *neutral = " (123) ";
  cl_assert(!bidi_paragraph_is_rtl((const utf8_t *)neutral,
                                   (const utf8_t *)neutral + strlen(neutral)));
}

void test_bidi__mirror_codepoint(void) {
  cl_assert_equal_i(bidi_mirror_codepoint('('), ')');
  cl_assert_equal_i(bidi_mirror_codepoint(')'), '(');
  cl_assert_equal_i(bidi_mirror_codepoint('['), ']');
  cl_assert_equal_i(bidi_mirror_codepoint(']'), '[');
  cl_assert_equal_i(bidi_mirror_codepoint('{'), '}');
  cl_assert_equal_i(bidi_mirror_codepoint('<'), '>');
  cl_assert_equal_i(bidi_mirror_codepoint(0x00AB), 0x00BB);
  cl_assert_equal_i(bidi_mirror_codepoint(0x00BB), 0x00AB);

  // Codepoints without a mirrored form are returned unchanged.
  cl_assert_equal_i(bidi_mirror_codepoint('a'), 'a');
  cl_assert_equal_i(bidi_mirror_codepoint('-'), '-');
  cl_assert_equal_i(bidi_mirror_codepoint(0x0645), 0x0645);
}

void test_bidi__reverse_run(void) {
  prv_assert_reversed("abc", "cba");
  prv_assert_reversed("مرحبا", "ابحرم");
  prv_assert_reversed("", "");
}

void test_bidi__reverse_run_keeps_marks_on_their_base(void) {
  // "ab́c" reverses to "cb́a": the acute stays behind the b.
  prv_assert_reversed("ab\xCC\x81" "c", "c" "b\xCC\x81" "a");

  // Arabic fatha and shadda stay behind the letter they belong to.
  prv_assert_reversed("\xD9\x85\xD9\x8E" "\xD8\xA8", "\xD8\xA8" "\xD9\x85\xD9\x8E");
}

void test_bidi__latin_in_arabic_parentheses(void) {
  // "الرسالة (Hello) وصلت" - the brackets take the paragraph direction, so both
  // land in RTL runs and are mirrored when drawn.
  static const ExpectedRun expected[] = {
    { 0, 16, true },   // "الرسالة ("
    { 16, 5, false },  // "Hello"
    { 21, 10, true },  // ") وصلت"
  };
  prv_assert_runs("الرسالة (Hello) وصلت", expected, 3);
}

void test_bidi__arabic_in_arabic_parentheses(void) {
  // Nothing breaks the direction, so the whole line is one RTL run.
  static const ExpectedRun expected[] = {
    { 0, 36, true },
  };
  prv_assert_runs("الرسالة (مرحبا) وصلت", expected, 1);
}

void test_bidi__arabic_then_latin_in_parentheses(void) {
  static const ExpectedRun expected[] = {
    { 0, 27, true },   // "الرسالة (مرحبا "
    { 27, 2, false },  // "Hi"
    { 29, 10, true },  // ") وصلت"
  };
  prv_assert_runs("الرسالة (مرحبا Hi) وصلت", expected, 3);
}

void test_bidi__latin_then_arabic_in_parentheses(void) {
  static const ExpectedRun expected[] = {
    { 0, 16, true },   // "الرسالة ("
    { 16, 2, false },  // "Hi"
    { 18, 21, true },  // " مرحبا) وصلت"
  };
  prv_assert_runs("الرسالة (Hi مرحبا) وصلت", expected, 3);
}

void test_bidi__arabic_indic_digits_read_left_to_right(void) {
  // Arabic-Indic digits sit in the Arabic block but are laid out left to right,
  // so they form their own run and keep their order.
  static const ExpectedRun expected[] = {
    { 0, 15, true },
    { 15, 8, false },
    { 23, 9, true },
  };
  prv_assert_runs("الرسالة ٢٠٢٦ وصلت", expected, 3);
}

void test_bidi__digits_keep_their_order_inside_arabic(void) {
  // Arabic-Indic and Western digits both read left to right, so each forms its
  // own run rather than being reversed with the letters around it.
  static const ExpectedRun arabic_indic[] = {
    { 0, 2, true },   // Alef
    { 2, 4, false },  // "\u0662\u0663"
  };
  prv_assert_runs("\u0627\u0662\u0663", arabic_indic, 2);

  static const ExpectedRun western[] = {
    { 0, 2, true },   // Alef
    { 2, 3, false },  // "123"
  };
  prv_assert_runs("\u0627" "123", western, 2);
}

void test_bidi__date_keeps_its_groups_in_order(void) {
  // The slashes bind to the digits either side, so the whole date stays one
  // left-to-right run and the groups are not reordered.
  static const ExpectedRun expected[] = {
    { 0, 18, false },
  };
  prv_assert_runs("\u0662\u0660\u0662\u0666/\u0660\u0666/\u0662\u0662", expected, 1);
}

void test_bidi__arabic_indic_time_keeps_its_groups(void) {
  static const ExpectedRun expected[] = {
    { 0, 9, false },
  };
  prv_assert_runs("\u0661\u0662:\u0663\u0664", expected, 1);
}

void test_bidi__separator_needs_a_number_on_both_sides(void) {
  // A separator with a letter on one side is not part of the number, so it
  // resolves as a neutral and stays with the Arabic run.
  static const ExpectedRun expected[] = {
    { 0, 3, true },   // Alef + '/'
    { 3, 2, false },  // "\u0662"
  };
  prv_assert_runs("\u0627/\u0662", expected, 2);
}

void test_bidi__number_keeps_its_separator(void) {
  // The colon binds the two halves of the time together instead of splitting
  // the number into three runs.
  static const ExpectedRun expected[] = {
    { 0, 3, true },
    { 3, 5, false },  // "12:30"
    { 8, 3, true },
  };
  prv_assert_runs("م 12:30 م", expected, 3);
}

void test_bidi__number_keeps_its_terminator(void) {
  static const ExpectedRun expected[] = {
    { 0, 3, true },
    { 3, 3, false },  // "50%"
    { 6, 3, true },
  };
  prv_assert_runs("م 50% م", expected, 3);
}

void test_bidi__trailing_emoji_joins_the_arabic_run(void) {
  // A trailing neutral has no strong character after it, so it takes the
  // paragraph direction and ends up at the visual start of the line.
  static const ExpectedRun expected[] = {
    { 0, 15, true },
  };
  prv_assert_runs("مرحبا 👋", expected, 1);
}

void test_bidi__mark_stays_with_the_run_of_its_base(void) {
  // The acute on the final "e" must join the Latin run rather than resolving on
  // its own, or reordering would strand it at the visual start of the line.
  static const ExpectedRun expected[] = {
    { 0, 11, true },   // "\u0645\u0631\u062d\u0628\u0627 "
    { 11, 6, false },  // "caf" + "e" + combining acute
  };
  prv_assert_runs("\u0645\u0631\u062d\u0628\u0627 caf" "e\u0301", expected, 2);
}

void test_bidi__latin_phrase_with_a_number_stays_together(void) {
  // W7: a number after a Latin word takes that word's direction, so the
  // neutrals around it stay left-to-right and the phrase is not split into
  // runs that the paragraph reorder would then reverse.
  static const ExpectedRun expected[] = {
    { 0, 11, true },   // "\u0645\u0631\u062d\u0628\u0627 "
    { 11, 11, false},  // "abc 123 def"
  };
  prv_assert_runs("\u0645\u0631\u062d\u0628\u0627 abc 123 def", expected, 2);
}

void test_bidi__honorific_mark_stays_with_its_base(void) {
  // U+0610-U+061A are transparent to the shaper and must be treated as marks
  // here too, or the reversal detaches them from the letter they sit on.
  prv_assert_reversed("\u0628\u0610\u062c", "\u062c\u0628\u0610");

  static const ExpectedRun expected[] = {
    { 0, 6, true },
  };
  prv_assert_runs("\u0628\u0610\u062c", expected, 1);
}

void test_bidi__direction_does_not_cross_a_newline(void) {
  // A newline is a paragraph separator, so a neutral at the end of one line
  // must not take its direction from the text after the break. The bracket here
  // resolves the same way whether or not an Arabic paragraph follows.
  static const ExpectedRun alone[] = {
    { 0, 4, false },  // "abc "
    { 4, 8, true },   // "\u05e9\u05dc\u05d5\u05dd"
    { 12, 1, false }, // ")"
  };
  prv_assert_runs("abc \u05e9\u05dc\u05d5\u05dd)", alone, 3);

  static const ExpectedRun with_next_paragraph[] = {
    { 0, 4, false },
    { 4, 8, true },
    { 12, 1, false }, // ")" - unchanged by the Arabic after the break
    { 13, 1, false }, // the separator itself
    { 14, 10, true }, // "\u0645\u0631\u062d\u0628\u0627"
  };
  prv_assert_runs("abc \u05e9\u05dc\u05d5\u05dd)\n\u0645\u0631\u062d\u0628\u0627",
                  with_next_paragraph, 5);
}

void test_bidi__extended_arabic_mark_stays_with_its_base(void) {
  // The Arabic Extended-A/B blocks carry combining marks among their letters,
  // so the range cannot be classified as strong right-to-left wholesale.
  prv_assert_reversed("\u0628\u08f0\u062c", "\u062c\u0628\u08f0");
}

void test_bidi__marks_outside_the_main_block_stay_with_their_base(void) {
  // Combining marks live in several blocks; every one of them has to be
  // recognised or the reversal drops the mark onto the neighbouring character.
  prv_assert_reversed("a\u1ab0b", "b" "a\u1ab0");  // marks extended
  prv_assert_reversed("a\u1dc0b", "b" "a\u1dc0");  // marks supplement
  prv_assert_reversed("a\u20d0b", "b" "a\u20d0");  // marks for symbols
  prv_assert_reversed("a\ufe20b", "b" "a\ufe20");  // combining half marks
}

void test_bidi__supplementary_script_is_not_neutral(void) {
  // Only emoji are neutral above the BMP; a supplementary ideograph keeps the
  // left-to-right default and forms its own run instead of being reversed with
  // the Arabic around it.
  static const ExpectedRun expected[] = {
    { 0, 11, true },   // "\u0645\u0631\u062d\u0628\u0627 "
    { 11, 4, false },  // U+20000
  };
  prv_assert_runs("\u0645\u0631\u062d\u0628\u0627 \U00020000", expected, 2);
}

void test_bidi__separator_is_a_run_of_its_own(void) {
  // Resolving the separator already steps past it, so a run starting on one
  // must end there rather than continuing into the next paragraph.
  static const ExpectedRun expected[] = {
    { 0, 3, false },  // "abc"
    { 3, 1, false },  // the separator
    { 4, 3, false },  // "xyz"
  };
  prv_assert_runs("abc\nxyz", expected, 3);
  prv_assert_runs("abc\rxyz", expected, 3);
}

void test_bidi__flag_keeps_its_indicator_order(void) {
  // A flag is an ordered pair of regional indicators. Reversing them would name
  // a different country, so the pair travels together like a cluster.
  prv_assert_reversed("\U0001f1fa\U0001f1f8" "a", "a" "\U0001f1fa\U0001f1f8");

  // Pairing runs from the start of the sequence, as the renderer does, so an
  // odd trailing indicator stands alone rather than re-pairing the ones before
  // it into a different flag: A B C reverses to C, then A B.
  prv_assert_reversed("\U0001f1e6\U0001f1e7\U0001f1e8",
                      "\U0001f1e8" "\U0001f1e6\U0001f1e7");
  prv_assert_reversed("\U0001f1e6\U0001f1e7\U0001f1e8\U0001f1e9",
                      "\U0001f1e8\U0001f1e9" "\U0001f1e6\U0001f1e7");
}

void test_bidi__hebrew_inside_a_latin_paragraph(void) {
  // The paragraph reads left to right, so the brackets stay left to right and
  // are drawn unmirrored.
  static const ExpectedRun expected[] = {
    { 0, 5, false },  // "Say ("
    { 5, 8, true },   // "שלום"
    { 13, 5, false }, // ") now"
  };
  prv_assert_runs("Say (שלום) now", expected, 3);
}

void test_bidi__contains_arabic(void) {
  const char *arabic = "hi مرحبا";
  cl_assert(bidi_contains_arabic((const utf8_t *)arabic,
                                 (const utf8_t *)arabic + strlen(arabic)));

  const char *hebrew = "hi שלום";
  cl_assert(!bidi_contains_arabic((const utf8_t *)hebrew,
                                  (const utf8_t *)hebrew + strlen(hebrew)));

  const char *latin = "hi there";
  cl_assert(!bidi_contains_arabic((const utf8_t *)latin,
                                  (const utf8_t *)latin + strlen(latin)));
}
