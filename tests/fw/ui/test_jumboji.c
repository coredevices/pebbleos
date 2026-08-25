/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/timeline/notification_jumboji_table.h"
#include "pbl/services/timeline/notification_layout.h"
#include "pbl/util/size.h"

#include "clar.h"

#include <stdio.h>

// Stubs
/////////////////////

#include "stubs_analytics.h"
#include "stubs_attribute.h"
#include "stubs_clock.h"
#include "stubs_graphics.h"
#include "stubs_graphics_context.h"
#include "stubs_kino_layer.h"
#include "stubs_layer.h"
#include "stubs_layout_node.h"
#include "stubs_logging.h"
#include "stubs_notification_image.h"
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"
#include "stubs_pin_db.h"
#include "stubs_resources.h"
#include "stubs_shell_prefs.h"
#include "stubs_text_node.h"
#include "stubs_timeline_item.h"
#include "stubs_timeline_resources.h"

// Not in stubs_graphics.h: test_bitmap_layer.c defines its own.
void graphics_draw_bitmap_in_rect(GContext *ctx, const GBitmap *src_bitmap, const GRect *rect) {}

// Statics
////////////////////////////////////
static const EmojiEntry s_emoji_table[] = JUMBOJI_TABLE(EMOJI_ENTRY);

// Tests
//////////////////////

void test_jumboji__jumboji_table(void) {
  for (unsigned int i = 0; i < ARRAY_LENGTH(s_emoji_table); i++) {
    const EmojiEntry *emoji = &s_emoji_table[i];
    const Codepoint codepoint = utf8_peek_codepoint((utf8_t *)emoji->string, NULL);
    if (emoji->codepoint != codepoint) {
      printf("  ENTRY(\"%s\", 0x%05x, %s)\n", emoji->string, (unsigned int)codepoint,
             emoji->resource_name);
    }
    cl_check(emoji->codepoint == codepoint);
  }
}

ResourceId prv_get_emoji_icon_by_string(const EmojiEntry *table, const char *str);

void test_jumboji__jumboji_detection(void) {
  // NULL is not an emoji
  cl_check(prv_get_emoji_icon_by_string(s_emoji_table, NULL) == INVALID_RESOURCE);

  // Empty string is not an emoji
  cl_check(prv_get_emoji_icon_by_string(s_emoji_table, "") == INVALID_RESOURCE);

  // Single emoji is detected
  cl_check(prv_get_emoji_icon_by_string(s_emoji_table, "😀") ==
           RESOURCE_ID_EMOJI_BIG_OPEN_SMILE_LARGE);

  // Leading whitespace is ignored
  cl_check(prv_get_emoji_icon_by_string(s_emoji_table, " 😀") ==
           RESOURCE_ID_EMOJI_BIG_OPEN_SMILE_LARGE);

  // Trailing whitespace is ignored
  cl_check(prv_get_emoji_icon_by_string(s_emoji_table, "😀 ") ==
           RESOURCE_ID_EMOJI_BIG_OPEN_SMILE_LARGE);

  // Leading and trailing whitespace is ignored
  cl_check(prv_get_emoji_icon_by_string(s_emoji_table, " 😀 ") ==
           RESOURCE_ID_EMOJI_BIG_OPEN_SMILE_LARGE);

  // Double emoji is ignored
  cl_check(prv_get_emoji_icon_by_string(s_emoji_table, "😀😂") == INVALID_RESOURCE);

  // LTR indicator is ignored
  cl_check(prv_get_emoji_icon_by_string(s_emoji_table, "\u200E😂") ==
           RESOURCE_ID_EMOJI_LAUGHING_WITH_TEARS_LARGE);

  // Zero-width-no-break at the end is ignored
  cl_check(prv_get_emoji_icon_by_string(s_emoji_table, "😂\uFEFF") ==
           RESOURCE_ID_EMOJI_LAUGHING_WITH_TEARS_LARGE);

  // Skin tone modifier is ignored
  cl_check(prv_get_emoji_icon_by_string(s_emoji_table, "👍\xf0\x9f\x8f\xbe") ==
           RESOURCE_ID_EMOJI_THUMBS_UP_LARGE);
}
