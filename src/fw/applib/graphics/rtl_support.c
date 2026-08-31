/* SPDX-FileCopyrightText: 2026 Ahmed Hussein */
/* SPDX-License-Identifier: Apache-2.0 */

#include "rtl_support.h"

#include "applib/fonts/codepoint.h"
#include "utf8.h"

// Maximum number of codepoints we can handle in a single reversal.
// 32 is sufficient for real Hebrew words including long morphological forms.
#define MAX_RTL_CODEPOINTS 32

bool utf8_contains_rtl(const utf8_t *start, const utf8_t *end) {
  if (start == NULL || end == NULL || start >= end) {
    return false;
  }

  utf8_t *ptr = (utf8_t *)start;
  while (ptr < end && *ptr != '\0') {
    utf8_t *next = NULL;
    Codepoint cp = utf8_peek_codepoint(ptr, &next);
    if (cp == 0 || next == NULL) {
      break;
    }
    if (codepoint_is_rtl(cp)) {
      return true;
    }
    ptr = next;
  }

  return false;
}

//! Check if a codepoint is a shapeable Arabic letter (U+0621-U+064A)
static bool prv_codepoint_is_arabic_letter(Codepoint cp) {
  // Arabic letters that require contextual shaping
  // Excludes diacritics (U+064B-U+065F) and numerals (U+0660-U+0669)
  return (cp >= 0x0621 && cp <= 0x064A);
}

bool utf8_contains_arabic(const utf8_t *start, const utf8_t *end) {
  if (start == NULL || end == NULL || start >= end) {
    return false;
  }

  utf8_t *ptr = (utf8_t *)start;
  while (ptr < end && *ptr != '\0') {
    utf8_t *next = NULL;
    Codepoint cp = utf8_peek_codepoint(ptr, &next);
    if (cp == 0 || next == NULL) {
      break;
    }
    if (prv_codepoint_is_arabic_letter(cp)) {
      return true;
    }
    ptr = next;
  }

  return false;
}

// Weak-LTR digits: Western (0x30-0x39, as used in Arabic, Hebrew and other RTL
// text) and Arabic-Indic (0x0660-0x0669, 0x06F0-0x06F9). Inside an RTL run
// these keep their left-to-right order; reversing them with the run would turn
// a number such as 2026 into 6202.
static bool prv_codepoint_is_digit(Codepoint cp) {
  return (cp >= 0x30 && cp <= 0x39) ||
         (cp >= 0x0660 && cp <= 0x0669) ||
         (cp >= 0x06F0 && cp <= 0x06F9);
}

// Separators that stay inside a numeric run when flanked by digits, so a time
// or date such as 12:34 or 2026/06/22 keeps its left-to-right group order.
static bool prv_codepoint_is_numeric_separator(Codepoint cp) {
  return cp == ':' || cp == '/' || cp == '.' || cp == ',';
}

utf8_t *rtl_segment_content_end(utf8_t *start, utf8_t *end) {
  utf8_t *content_end = start;
  utf8_t *scan = start;
  while (scan < end) {
    utf8_t *scan_next = NULL;
    Codepoint scan_cp = utf8_peek_codepoint(scan, &scan_next);
    if (scan_cp == 0 || scan_next == NULL) {
      return end;
    }
    if (scan_cp != SPACE_CODEPOINT) {
      content_end = scan_next;
    }
    scan = scan_next;
  }
  return content_end;
}

size_t utf8_reverse_for_rtl(const utf8_t *src, size_t src_len,
                            utf8_t *dest, size_t dest_size) {
  if (src == NULL || dest == NULL || src_len == 0 || dest_size == 0) {
    return 0;
  }

  // First pass: find the end of the bounded input we will reverse.
  size_t num_codepoints = 0;
  utf8_t *ptr = (utf8_t *)src;
  const utf8_t *end = src + src_len;

  while (ptr < end && *ptr != '\0' && num_codepoints < MAX_RTL_CODEPOINTS) {
    utf8_t *next = NULL;
    Codepoint cp = utf8_peek_codepoint(ptr, &next);
    if (cp == 0 || next == NULL) {
      break;
    }
    ptr = next;
    num_codepoints++;
  }

  if (num_codepoints == 0) {
    return 0;
  }

  const utf8_t *reverse_ptr = ptr;
  size_t dest_offset = 0;

  // Second pass: walk backward over UTF-8 sequence starts and write each
  // codepoint to the destination. This avoids a stack array in the render path.
  while (reverse_ptr > src) {
    const utf8_t *cp_start = reverse_ptr - 1;
    while (cp_start > src && ((*cp_start & 0xC0) == 0x80)) {
      cp_start--;
    }

    utf8_t *next = NULL;
    Codepoint cp = utf8_peek_codepoint((utf8_t *)cp_start, &next);
    if (cp == 0 || next == NULL || next > reverse_ptr) {
      break;
    }

    if (prv_codepoint_is_digit(cp)) {
      // Weak-LTR: emit a contiguous digit run in logical (left-to-right)
      // order instead of reversing it.
      const utf8_t *run_start = cp_start;
      while (run_start > src) {
        const utf8_t *prev_start = run_start - 1;
        while (prev_start > src && ((*prev_start & 0xC0) == 0x80)) {
          prev_start--;
        }
        utf8_t *prev_next = NULL;
        Codepoint prev_cp = utf8_peek_codepoint((utf8_t *)prev_start, &prev_next);
        if (prev_cp == 0 || prev_next == NULL) {
          break;
        }
        if (prv_codepoint_is_digit(prev_cp)) {
          run_start = prev_start;
          continue;
        }
        // A separator joins the run only between two digits. run_start already
        // points at a digit (so the separator is followed by one); require a
        // digit before it too, then pull both into the run in one step.
        if (prv_codepoint_is_numeric_separator(prev_cp) && prev_start > src) {
          const utf8_t *before = prev_start - 1;
          while (before > src && ((*before & 0xC0) == 0x80)) {
            before--;
          }
          utf8_t *before_next = NULL;
          Codepoint before_cp = utf8_peek_codepoint((utf8_t *)before, &before_next);
          if (before_cp != 0 && before_next != NULL && prv_codepoint_is_digit(before_cp)) {
            run_start = before;
            continue;
          }
        }
        break;
      }

      const utf8_t *fwd = run_start;
      while (fwd < reverse_ptr) {
        utf8_t *fwd_next = NULL;
        Codepoint dcp = utf8_peek_codepoint((utf8_t *)fwd, &fwd_next);
        if (dcp == 0 || fwd_next == NULL || dest_offset + 4 >= dest_size) {
          break;
        }
        size_t n = utf8_encode_codepoint(dcp, dest + dest_offset);
        if (n != 0) {
          dest_offset += n;
        }
        fwd = fwd_next;
      }
      reverse_ptr = run_start;
      continue;
    }

    // Make sure we have room for at least 4 bytes + null terminator
    if (dest_offset + 4 >= dest_size) {
      break;
    }

    size_t bytes_written = utf8_encode_codepoint(cp, dest + dest_offset);
    if (bytes_written == 0) {
      reverse_ptr = cp_start;
      continue;
    }
    dest_offset += bytes_written;
    reverse_ptr = cp_start;
  }

  // Null-terminate if we have space
  if (dest_offset < dest_size) {
    dest[dest_offset] = '\0';
  }

  return dest_offset;
}
