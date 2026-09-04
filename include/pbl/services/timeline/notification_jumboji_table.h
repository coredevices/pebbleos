/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/fonts/codepoint.h"
#include "resource/resource_ids.auto.h"

typedef struct {
  Codepoint codepoint;
  ResourceId resource_id;
#if UNITTEST
  const char *string;
  const char *resource_name;
#endif
} EmojiEntry;

#if UNITTEST
#define EMOJI_ENTRY(string, codepoint, resource_id) \
    { codepoint, resource_id, string, #resource_id }
#else
#define EMOJI_ENTRY(string, codepoint, resource_id) \
    { codepoint, resource_id }
#endif

// Codepoint sorted table of supported Jumboji
#define JUMBOJI_TABLE(ENTRY) { \
  ENTRY("♥️", 0x02665, RESOURCE_ID_EMOJI_HEART_LARGE), \
  ENTRY("❤️", 0x02764, RESOURCE_ID_EMOJI_HEART_LARGE), \
  ENTRY("👍", 0x1f44d, RESOURCE_ID_EMOJI_THUMBS_UP_LARGE), \
  ENTRY("💙", 0x1f499, RESOURCE_ID_EMOJI_HEART_LARGE), \
  ENTRY("💚", 0x1f49a, RESOURCE_ID_EMOJI_HEART_LARGE), \
  ENTRY("💛", 0x1f49b, RESOURCE_ID_EMOJI_HEART_LARGE), \
  ENTRY("💜", 0x1f49c, RESOURCE_ID_EMOJI_HEART_LARGE), \
  ENTRY("😀", 0x1f600, RESOURCE_ID_EMOJI_BIG_OPEN_SMILE_LARGE), \
  ENTRY("😁", 0x1f601, RESOURCE_ID_EMOJI_SMILING_WITH_TEETH_LARGE), \
  ENTRY("😂", 0x1f602, RESOURCE_ID_EMOJI_LAUGHING_WITH_TEARS_LARGE), \
  ENTRY("😃", 0x1f603, RESOURCE_ID_EMOJI_BIG_OPEN_SMILE_LARGE), \
  ENTRY("😄", 0x1f604, RESOURCE_ID_EMOJI_BIG_SMILE_LARGE), \
  ENTRY("😉", 0x1f609, RESOURCE_ID_EMOJI_WINK_LARGE), \
  ENTRY("😊", 0x1f60a, RESOURCE_ID_EMOJI_SMILING_BLUSH_LARGE), \
  ENTRY("😍", 0x1f60d, RESOURCE_ID_EMOJI_SMILING_HEARTS_LARGE), \
  ENTRY("😘", 0x1f618, RESOURCE_ID_EMOJI_KISSING_WITH_HEART_LARGE), \
  ENTRY("😜", 0x1f61c, RESOURCE_ID_EMOJI_WINK_TONGUE_LARGE), \
  ENTRY("😞", 0x1f61e, RESOURCE_ID_EMOJI_SAD_LARGE), \
  ENTRY("😟", 0x1f61f, RESOURCE_ID_EMOJI_SAD_LARGE), \
}
