/* SPDX-FileCopyrightText: 2026 Ahmed Hussein */
/* SPDX-FileCopyrightText: 2026 Khalid Nuaim (kaluaim) */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "utf8.h"

#include <stdbool.h>
#include <stddef.h>

//! Minimal bidirectional text engine, a subset of Unicode UAX 9.
//!
//! Implements the rules that matter for a single line of watch text:
//! P2/P3 (paragraph direction), W4-W6 (numbers absorb their separators and
//! terminators), N1/N2 (neutrals take the surrounding direction, otherwise the
//! paragraph direction), L2 (run reordering, done by the caller) and L4
//! (mirrored glyphs). Explicit directional controls, isolates and levels above
//! two are not implemented.

//! Check whether a UTF-8 range needs bidirectional processing at all.
//! Scans raw bytes, so pure-ASCII text costs one comparison per byte and the
//! caller can stay on its left-to-right path.
//! @param start Pointer to start of UTF-8 string
//! @param end Pointer to end of UTF-8 string (exclusive)
//! @return true if the range contains at least one right-to-left character
bool bidi_is_needed(const utf8_t *start, const utf8_t *end);

//! Resolve the paragraph direction of a UTF-8 range (UAX 9 P2/P3).
//! The first strong character wins; ranges without one are left-to-right.
//! @param start Pointer to start of UTF-8 string
//! @param end Pointer to end of UTF-8 string (exclusive)
//! @return true if the paragraph reads right-to-left
bool bidi_paragraph_is_rtl(const utf8_t *start, const utf8_t *end);

//! Find the extent of the directional run starting at @p pos.
//!
//! Weak types are folded into the neighbouring number and neutrals are
//! resolved against the surrounding runs, so the returned range is a maximal
//! stretch of one direction. Numbers always form a left-to-right run, matching
//! the level they are assigned in either paragraph direction.
//!
//! @param line_start Start of the line, used to look behind @p pos
//! @param pos Position to start the run at, must be within the line
//! @param end End of the line (exclusive)
//! @param para_is_rtl Paragraph direction, from \ref bidi_paragraph_is_rtl
//! @param[out] run_is_rtl Direction of the returned run
//! @return Pointer to the first codepoint after the run, or @p pos on failure
utf8_t *bidi_next_run(const utf8_t *line_start, utf8_t *pos, const utf8_t *end,
                      bool para_is_rtl, bool *run_is_rtl);

//! Return the mirrored form of a codepoint (UAX 9 L4).
//! Glyphs such as brackets are drawn mirrored inside a right-to-left run.
//! Codepoints without a mirrored form are returned unchanged.
Codepoint bidi_mirror_codepoint(Codepoint cp);

//! Reverse the codepoints of a run for right-to-left display.
//! Combining marks stay behind the base character they attach to.
//! @param src Source UTF-8 string
//! @param src_len Length of source string in bytes
//! @param dest Destination buffer for the reversed string
//! @param dest_size Size of destination buffer in bytes
//! @return Number of bytes written to dest (excluding null terminator), or 0 on failure
size_t bidi_reverse_run(const utf8_t *src, size_t src_len, utf8_t *dest, size_t dest_size);

//! Check if a UTF-8 string range contains any shapeable Arabic letters.
//! @param start Pointer to start of UTF-8 string
//! @param end Pointer to end of UTF-8 string (exclusive)
//! @return true if the range contains at least one shapeable Arabic letter
bool bidi_contains_arabic(const utf8_t *start, const utf8_t *end);
