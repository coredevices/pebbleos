/* SPDX-FileCopyrightText: 2026 Ahmed Hussein */
/* SPDX-FileCopyrightText: 2026 Khalid Nuaim (kaluaim) */
/* SPDX-License-Identifier: Apache-2.0 */

#include "bidi.h"

#include "arabic_shaping.h"
#include "utf8.h"

#include "applib/fonts/codepoint.h"
#include "pbl/util/size.h"

#include <string.h>

// Bidirectional character classes, coarsened from UAX 9 to the ones that
// change the outcome for a single line of text.
typedef enum {
  BidiClassL,    // Strong left-to-right
  BidiClassR,    // Strong right-to-left
  BidiClassEN,   // European number
  BidiClassAN,   // Arabic-Indic number
  BidiClassES,   // European separator, binds two European numbers
  BidiClassCS,   // Common separator, binds two numbers of the same class
  BidiClassET,   // European terminator, binds to an adjacent European number
  BidiClassNSM,  // Non-spacing mark, inherits the class of its base
  BidiClassB,    // Paragraph separator, ends the range direction is resolved over
  BidiClassON,   // Other neutral
} BidiClass;

// Mirrored pairs from the Unicode BidiMirroring table, limited to the ones
// that turn up in watch text.
typedef struct {
  uint16_t first;
  uint16_t second;
} BidiMirrorPair;

static const BidiMirrorPair s_mirror_pairs[] = {
  { 0x0028, 0x0029 },  // Parentheses
  { 0x003C, 0x003E },  // Less-than, greater-than
  { 0x005B, 0x005D },  // Square brackets
  { 0x007B, 0x007D },  // Curly brackets
  { 0x00AB, 0x00BB },  // Double angle quotation marks
  { 0x2039, 0x203A },  // Single angle quotation marks
  { 0x2045, 0x2046 },  // Square brackets with quill
  { 0x207D, 0x207E },  // Superscript parentheses
  { 0x208D, 0x208E },  // Subscript parentheses
  { 0x2264, 0x2265 },  // Less-than or equal, greater-than or equal
};

#define MAX_MIRRORED_CODEPOINT 0x2265

static BidiClass prv_ascii_class(Codepoint cp) {
  if (cp >= '0' && cp <= '9') {
    return BidiClassEN;
  }
  if ((cp >= 'A' && cp <= 'Z') || (cp >= 'a' && cp <= 'z')) {
    return BidiClassL;
  }
  switch (cp) {
    case '\n':
    case '\r':
      return BidiClassB;
    case '#':
    case '$':
    case '%':
      return BidiClassET;
    case '+':
    case '-':
      return BidiClassES;
    case ',':
    case '.':
    case '/':
    case ':':
      return BidiClassCS;
    default:
      return BidiClassON;
  }
}

//! Non-spacing marks across every block this engine can meet. A mark must never
//! resolve on its own: W1 gives it the class of its base, and the reversal keeps
//! it behind that base. Kept in one place so a new block cannot be half-covered.
static bool prv_is_combining_mark(Codepoint cp) {
  return (cp >= 0x0300 && cp <= 0x036F) ||   // Combining diacritical marks
         // Hebrew points and cantillation, minus the punctuation sharing the range
         (cp >= 0x0591 && cp <= 0x05C7 && cp != 0x05BE && cp != 0x05C0 &&
          cp != 0x05C3 && cp != 0x05C6) ||
         (cp >= 0x0610 && cp <= 0x061A) ||   // Arabic honorifics
         (cp >= 0x064B && cp <= 0x065F) ||   // Arabic harakat
         (cp == 0x0670) ||
         (cp >= 0x06D6 && cp <= 0x06DC) || (cp >= 0x06DF && cp <= 0x06E4) ||
         (cp >= 0x06E7 && cp <= 0x06E8) || (cp >= 0x06EA && cp <= 0x06ED) ||
         (cp >= 0x0730 && cp <= 0x074A) ||   // Syriac points
         (cp >= 0x07A6 && cp <= 0x07B0) ||   // Thaana vowel signs
         (cp >= 0x07EB && cp <= 0x07F3) || (cp == 0x07FD) ||  // NKo marks
         (cp >= 0x0898 && cp <= 0x089F) ||   // Arabic Extended-B marks
         (cp >= 0x08CA && cp <= 0x08E1) || (cp >= 0x08E3 && cp <= 0x08FF) ||
         (cp >= 0x1AB0 && cp <= 0x1AFF) ||   // Combining marks extended
         (cp >= 0x1DC0 && cp <= 0x1DFF) ||   // Combining marks supplement
         (cp >= 0x20D0 && cp <= 0x20FF) ||   // Combining marks for symbols
         (cp >= 0xFE00 && cp <= 0xFE0F) ||   // Variation selectors
         (cp >= 0xFE20 && cp <= 0xFE2F);     // Combining half marks
}

static BidiClass prv_class(Codepoint cp) {
  if (cp < 0x0080) {
    return prv_ascii_class(cp);
  }
  if (prv_is_combining_mark(cp)) {
    return BidiClassNSM;
  }
  if (cp >= 0x0590 && cp <= 0x05FF) {  // Hebrew
    return BidiClassR;
  }
  if (cp >= 0x0600 && cp <= 0x06FF) {  // Arabic
    if ((cp >= 0x0600 && cp <= 0x0605) || (cp >= 0x0660 && cp <= 0x0669) ||
        cp == 0x066B || cp == 0x066C || cp == 0x06DD) {
      return BidiClassAN;
    }
    if (cp >= 0x06F0 && cp <= 0x06F9) {  // Extended Arabic-Indic digits
      return BidiClassEN;
    }
    if (cp == 0x060C) {  // Arabic comma
      return BidiClassCS;
    }
    if (cp == 0x0609 || cp == 0x060A || cp == 0x066A) {  // Per mille, per ten thousand, percent
      return BidiClassET;
    }
    return BidiClassR;
  }
  if (cp >= 0x0700 && cp <= 0x08FF) {  // Syriac, Thaana, NKo, Arabic Extended-A/B
    // These blocks carry Arabic number signs among their letters.
    if (cp == 0x0890 || cp == 0x0891 || cp == 0x08E2) {
      return BidiClassAN;
    }
    return BidiClassR;
  }
  if ((cp >= 0xFB1D && cp <= 0xFDFF) || (cp >= 0xFE70 && cp <= 0xFEFC)) {
    return BidiClassR;  // Hebrew and Arabic presentation forms
  }
  switch (cp) {
    case 0x00A0:  // No-break space
      return BidiClassCS;
    case 0x00B0:  // Degree sign
    case 0x00B1:  // Plus-minus sign
    case 0x20AC:  // Euro sign
      return BidiClassET;
    case 0x2212:  // Minus sign
      return BidiClassES;
    case 0x200E:  // Left-to-right mark
      return BidiClassL;
    case 0x200F:  // Right-to-left mark
      return BidiClassR;
    default:
      break;
  }
  if (cp >= 0x00A1 && cp <= 0x00BF) {  // Latin-1 punctuation and symbols
    return BidiClassON;
  }
  if (cp >= 0x2000 && cp <= 0x2BFF) {  // Punctuation, symbols, arrows, math
    return BidiClassON;
  }
  if (codepoint_is_emoji(cp) || codepoint_is_regional_indicator(cp)) {
    return BidiClassON;
  }
  return BidiClassL;
}

//! Decode the codepoint at @p pos and return its class, setting @p next to the
//! following codepoint. @p next is NULL when nothing could be decoded.
static BidiClass prv_class_at(const utf8_t *pos, const utf8_t *end, utf8_t **next) {
  *next = NULL;
  if (pos == NULL || pos >= end || *pos == '\0') {
    return BidiClassON;
  }

  Codepoint cp = utf8_peek_codepoint((utf8_t *)pos, next);
  if (cp == 0 || *next == NULL || *next > end) {
    *next = NULL;
    return BidiClassON;
  }
  return prv_class(cp);
}

//! Direction a class contributes when a neutral looks at it (N1). Numbers act
//! as right-to-left for this purpose even though they are laid out the other way.
static bool prv_side_is_rtl(BidiClass cls) {
  return (cls == BidiClassR) || (cls == BidiClassEN) || (cls == BidiClassAN);
}

static bool prv_class_has_side(BidiClass cls) {
  return (cls == BidiClassL) || prv_side_is_rtl(cls);
}

//! W7: a European number takes the direction of the strong character before it.
//! Returns true when that character is left-to-right, so the number stops acting
//! as right-to-left towards the neutrals around it.
static bool prv_number_follows_ltr(const utf8_t *line_start, const utf8_t *pos,
                                   const utf8_t *end, bool para_is_rtl) {
  utf8_t *cur = (utf8_t *)pos;
  while (cur > line_start) {
    cur = utf8_get_previous((utf8_t *)line_start, cur);
    if (cur == NULL) {
      break;
    }
    utf8_t *next = NULL;
    BidiClass cls = prv_class_at(cur, end, &next);
    if (next == NULL) {
      break;
    }
    if (cls == BidiClassL) {
      return true;
    }
    if ((cls == BidiClassR) || (cls == BidiClassB)) {
      return false;
    }
  }
  // Nothing strong precedes it, so the paragraph direction decides.
  return !para_is_rtl;
}

//! Direction the class at @p pos contributes to a neighbouring neutral, with W7
//! applied to European numbers.
static bool prv_resolved_side_is_rtl(const utf8_t *line_start, const utf8_t *pos,
                                     const utf8_t *end, bool para_is_rtl, BidiClass cls) {
  if (cls == BidiClassEN) {
    return !prv_number_follows_ltr(line_start, pos, end, para_is_rtl);
  }
  return prv_side_is_rtl(cls);
}

//! Direction of the last strong class or number before @p pos.
static bool prv_prev_side(const utf8_t *line_start, const utf8_t *pos,
                          const utf8_t *end, bool para_is_rtl, bool *is_rtl) {
  utf8_t *cur = (utf8_t *)pos;
  while (cur > line_start) {
    cur = utf8_get_previous((utf8_t *)line_start, cur);
    if (cur == NULL) {
      break;
    }
    utf8_t *next = NULL;
    BidiClass cls = prv_class_at(cur, end, &next);
    if ((next == NULL) || (cls == BidiClassB)) {
      break;
    }
    if (prv_class_has_side(cls)) {
      *is_rtl = prv_resolved_side_is_rtl(line_start, cur, end, para_is_rtl, cls);
      return true;
    }
  }
  return false;
}

//! Direction of the first strong class or number at or after @p pos.
static bool prv_next_side(const utf8_t *line_start, const utf8_t *pos,
                          const utf8_t *end, bool para_is_rtl, bool *is_rtl) {
  utf8_t *cur = (utf8_t *)pos;
  while (cur < end && *cur != '\0') {
    utf8_t *next = NULL;
    BidiClass cls = prv_class_at(cur, end, &next);
    if ((next == NULL) || (cls == BidiClassB)) {
      break;
    }
    if (prv_class_has_side(cls)) {
      *is_rtl = prv_resolved_side_is_rtl(line_start, cur, end, para_is_rtl, cls);
      return true;
    }
    cur = next;
  }
  return false;
}

static utf8_t *prv_skip_terminators(utf8_t *pos, const utf8_t *end) {
  utf8_t *cur = pos;
  while (cur < end && *cur != '\0') {
    utf8_t *next = NULL;
    if (prv_class_at(cur, end, &next) != BidiClassET || next == NULL) {
      break;
    }
    cur = next;
  }
  return cur;
}

//! Consume a number along with the separators and terminators that bind to it
//! (UAX 9 W4-W6). @p pos must point at a number of class @p num_cls.
static utf8_t *prv_scan_number(utf8_t *pos, const utf8_t *end, BidiClass num_cls) {
  utf8_t *cur = pos;
  while (cur < end && *cur != '\0') {
    utf8_t *next = NULL;
    BidiClass cls = prv_class_at(cur, end, &next);
    if (next == NULL) {
      break;
    }
    if (cls == num_cls || cls == BidiClassNSM) {
      cur = next;
      continue;
    }
    // W4: a separator surrounded by numbers of the same class joins them.
    if ((cls == BidiClassCS) || (cls == BidiClassES && num_cls == BidiClassEN)) {
      utf8_t *after = NULL;
      if (prv_class_at(next, end, &after) == num_cls && after != NULL) {
        cur = after;
        continue;
      }
      break;
    }
    // W5: terminators next to a European number join it.
    if (cls == BidiClassET && num_cls == BidiClassEN) {
      cur = next;
      continue;
    }
    break;
  }
  return cur;
}

//! Class a non-spacing mark inherits (UAX 9 W1). A mark takes the class of the
//! character it follows, or Other Neutral when nothing precedes it.
static BidiClass prv_inherited_class(const utf8_t *line_start, const utf8_t *pos,
                                     const utf8_t *end) {
  utf8_t *cur = (utf8_t *)pos;
  while (cur > line_start) {
    cur = utf8_get_previous((utf8_t *)line_start, cur);
    if (cur == NULL) {
      break;
    }
    utf8_t *next = NULL;
    BidiClass cls = prv_class_at(cur, end, &next);
    if (next == NULL) {
      break;
    }
    if (cls != BidiClassNSM) {
      return cls;
    }
  }
  return BidiClassON;
}

//! Resolve the direction of the span starting at @p pos and report where it
//! ends. A span is one strong character, a number with its weak neighbours, or
//! a stretch of neutrals resolved together.
static bool prv_resolve_span(const utf8_t *line_start, utf8_t *pos, const utf8_t *end,
                             bool para_is_rtl, bool *span_is_rtl, utf8_t **span_end) {
  utf8_t *next = NULL;
  BidiClass cls = prv_class_at(pos, end, &next);
  if (next == NULL) {
    return false;
  }

  // W1: a mark joins whatever it follows, so it never splits off on its own.
  if (cls == BidiClassNSM) {
    cls = prv_inherited_class(line_start, pos, end);
  }

  switch (cls) {
    case BidiClassL:
      *span_is_rtl = false;
      *span_end = next;
      return true;
    case BidiClassR:
      *span_is_rtl = true;
      *span_end = next;
      return true;
    case BidiClassEN:
    case BidiClassAN:
      // Numbers read left-to-right in either paragraph direction.
      *span_is_rtl = false;
      *span_end = prv_scan_number(pos, end, cls);
      return true;
    case BidiClassB:
      // A paragraph separator stands on its own at the paragraph direction.
      *span_is_rtl = para_is_rtl;
      *span_end = next;
      return true;
    default:
      break;
  }

  // A terminator run directly ahead of a European number belongs to it (W5).
  if (cls == BidiClassET) {
    utf8_t *number = prv_skip_terminators(pos, end);
    utf8_t *after = NULL;
    if (prv_class_at(number, end, &after) == BidiClassEN && after != NULL) {
      *span_is_rtl = false;
      *span_end = prv_scan_number(number, end, BidiClassEN);
      return true;
    }
  }

  // Neutral stretch, up to the next strong character or number.
  utf8_t *stretch_end = pos;
  while (stretch_end < end && *stretch_end != '\0') {
    utf8_t *stretch_next = NULL;
    BidiClass stretch_cls = prv_class_at(stretch_end, end, &stretch_next);
    if (stretch_next == NULL || prv_class_has_side(stretch_cls) ||
        (stretch_cls == BidiClassB)) {
      break;
    }
    if (stretch_cls == BidiClassET) {
      utf8_t *number = prv_skip_terminators(stretch_end, end);
      utf8_t *after = NULL;
      if (prv_class_at(number, end, &after) == BidiClassEN && after != NULL) {
        break;
      }
    }
    stretch_end = stretch_next;
  }
  if (stretch_end == pos) {
    stretch_end = next;
  }

  // N1: neutrals between two runs of the same direction take that direction.
  // N2: otherwise they take the paragraph direction.
  bool before_is_rtl = false;
  bool after_is_rtl = false;
  const bool has_before = prv_prev_side(line_start, pos, end, para_is_rtl, &before_is_rtl);
  const bool has_after = prv_next_side(line_start, stretch_end, end, para_is_rtl, &after_is_rtl);

  *span_is_rtl = (has_before && has_after && (before_is_rtl == after_is_rtl)) ?
      before_is_rtl : para_is_rtl;
  *span_end = stretch_end;
  return true;
}

bool bidi_is_needed(const utf8_t *start, const utf8_t *end) {
  if (start == NULL || end == NULL || start >= end) {
    return false;
  }

  // Hebrew (U+0590) through Arabic (U+06FF) encode with lead bytes 0xD6-0xDB.
  // Continuation bytes never land in that range, so a raw byte scan is enough
  // and pure-ASCII text costs one comparison per byte.
  for (const utf8_t *ptr = start; ptr < end && *ptr != '\0'; ptr++) {
    if (*ptr < 0xD6 || *ptr > 0xDB) {
      continue;
    }
    if (*ptr > 0xD6) {
      return true;
    }
    // Armenian (U+0580-U+058F) shares the 0xD6 lead byte with Hebrew.
    if ((ptr + 1) < end && ptr[1] >= 0x90) {
      return true;
    }
  }

  return false;
}

bool bidi_paragraph_is_rtl(const utf8_t *start, const utf8_t *end) {
  if (start == NULL || end == NULL || start >= end) {
    return false;
  }

  utf8_t *ptr = (utf8_t *)start;
  while (ptr < end && *ptr != '\0') {
    utf8_t *next = NULL;
    BidiClass cls = prv_class_at(ptr, end, &next);
    if (next == NULL) {
      break;
    }
    if (cls == BidiClassB) {
      break;
    }
    if (cls == BidiClassL) {
      return false;
    }
    if (cls == BidiClassR) {
      return true;
    }
    ptr = next;
  }

  return false;
}

utf8_t *bidi_next_run(const utf8_t *line_start, utf8_t *pos, const utf8_t *end,
                      bool para_is_rtl, bool *run_is_rtl) {
  if (line_start == NULL || pos == NULL || end == NULL || run_is_rtl == NULL ||
      pos >= end) {
    return pos;
  }

  bool dir = para_is_rtl;
  utf8_t *cur = NULL;
  if (!prv_resolve_span(line_start, pos, end, para_is_rtl, &dir, &cur)) {
    return pos;
  }
  *run_is_rtl = dir;

  // The separator itself is the whole run: resolving already stepped past it,
  // so the check below would otherwise look at the next paragraph's first
  // character and let the run continue across the break.
  utf8_t *first = NULL;
  if (prv_class_at(pos, end, &first) == BidiClassB) {
    return cur;
  }

  while (cur < end && *cur != '\0') {
    utf8_t *peek = NULL;
    if (prv_class_at(cur, end, &peek) == BidiClassB) {
      break;
    }
    bool span_is_rtl = false;
    utf8_t *span_end = NULL;
    if (!prv_resolve_span(line_start, cur, end, para_is_rtl, &span_is_rtl, &span_end)) {
      break;
    }
    if (span_is_rtl != dir || span_end <= cur) {
      break;
    }
    cur = span_end;
  }

  return cur;
}

Codepoint bidi_mirror_codepoint(Codepoint cp) {
  if (cp > MAX_MIRRORED_CODEPOINT) {
    return cp;
  }

  for (size_t i = 0; i < ARRAY_LENGTH(s_mirror_pairs); i++) {
    if (s_mirror_pairs[i].first == cp) {
      return s_mirror_pairs[i].second;
    }
    if (s_mirror_pairs[i].second == cp) {
      return s_mirror_pairs[i].first;
    }
  }

  return cp;
}

size_t bidi_reverse_run(const utf8_t *src, size_t src_len, utf8_t *dest, size_t dest_size) {
  if (dest == NULL || dest_size == 0) {
    return 0;
  }
  dest[0] = '\0';
  if (src == NULL || src_len == 0) {
    return 0;
  }

  // Bound the input to the first null byte or undecodable sequence.
  const utf8_t *limit = src + src_len;
  const utf8_t *end = src;
  while (end < limit && *end != '\0') {
    utf8_t *next = NULL;
    Codepoint cp = utf8_peek_codepoint((utf8_t *)end, &next);
    if (cp == 0 || next == NULL || next > limit) {
      break;
    }
    end = next;
  }

  size_t dest_offset = 0;
  const utf8_t *tail = end;
  while (tail > src) {
    const utf8_t *base = utf8_get_previous((utf8_t *)src, (utf8_t *)tail);
    if (base == NULL) {
      break;
    }

    // Combining marks are emitted after the base they attach to, so a cluster
    // keeps its logical order inside the reversed run.
    while (base > src) {
      utf8_t *next = NULL;
      Codepoint cp = utf8_peek_codepoint((utf8_t *)base, &next);
      if (cp == 0 || next == NULL || prv_class(cp) != BidiClassNSM) {
        break;
      }
      const utf8_t *prev = utf8_get_previous((utf8_t *)src, (utf8_t *)base);
      if (prev == NULL) {
        break;
      }
      base = prev;
    }

    // A flag is a pair of regional indicators, paired from the start of the
    // sequence the way the renderer pairs them. Step back onto the first member
    // when this one completes a pair, so the pair still names the same country
    // once the run has been reversed. An odd trailing indicator stands alone.
    utf8_t *base_next = NULL;
    if (codepoint_is_regional_indicator(utf8_peek_codepoint((utf8_t *)base, &base_next)) &&
        (base_next != NULL)) {
      size_t preceding = 0;
      const utf8_t *scan = base;
      while (scan > src) {
        const utf8_t *prev = utf8_get_previous((utf8_t *)src, (utf8_t *)scan);
        utf8_t *prev_next = NULL;
        if ((prev == NULL) ||
            !codepoint_is_regional_indicator(utf8_peek_codepoint((utf8_t *)prev, &prev_next)) ||
            (prev_next == NULL)) {
          break;
        }
        preceding++;
        scan = prev;
      }
      if ((preceding % 2) == 1) {
        const utf8_t *pair_start = utf8_get_previous((utf8_t *)src, (utf8_t *)base);
        if (pair_start != NULL) {
          base = pair_start;
        }
      }
    }

    const size_t cluster_len = (size_t)(tail - base);
    if ((dest_offset + cluster_len) >= dest_size) {
      break;
    }
    memcpy(dest + dest_offset, base, cluster_len);
    dest_offset += cluster_len;
    tail = base;
  }

  dest[dest_offset] = '\0';
  return dest_offset;
}

bool bidi_contains_arabic(const utf8_t *start, const utf8_t *end) {
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
    if (arabic_is_shapeable(cp)) {
      return true;
    }
    ptr = next;
  }

  return false;
}
