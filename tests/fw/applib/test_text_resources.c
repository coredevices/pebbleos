/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"
#include "fixtures/load_test_resources.h"

// FW headers
#include "applib/graphics/text_resources.h"
#include "resource/resource.h"
#include "resource/resource_ids.auto.h"
#include "resource/system_resource.h"
#include "pbl/util/size.h"

// Fakes
#include "fake_app_manager.h"

// Stubs
#include "stubs_analytics.h"
#include "stubs_bootbits.h"
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_pbl_malloc.h"
#include "stubs_pebble_tasks.h"
#include "stubs_passert.h"
#include "stubs_print.h"
#include "stubs_prompt.h"
#include "stubs_serial.h"
#include "stubs_sleep.h"
#include "stubs_syscalls.h"
#include "stubs_prompt.h"
#include "stubs_task_watchdog.h"
#include "stubs_memory_layout.h"

#define WILDCARD_CODEPOINT 0x25AF

static FontCache s_font_cache;
static FontInfo s_font_info;

// The renderer obtains the per-glyph fallback font from fonts_get_fallback_font(), which calls
// sys_font_get_system_font(NULL). A test installs a fallback by pointing s_test_fallback_font at
// its own FontInfo (NULL = no fallback, the default). The strong override of the weak syscall stub
// lives in test_text_resources_font_stub.c so it resolves at link time without colliding with the
// in-TU stub.
extern FontInfo *s_test_fallback_font;
// Installed emoji font, returned for any named font key (NULL = none, the default).
extern FontInfo *s_test_emoji_font;

#define FONT_COMPRESSION_FIXTURE_PATH "font_compression"

// Helpers
////////////////////////////////////

static uint8_t glyph_get_size_bytes(const GlyphData *glyph) {
  return ((glyph->header.width_px * glyph->header.height_px) + (8 - 1)) / 8;
}

void test_text_resources__initialize(void) {
  fake_spi_flash_init(0, 0x1000000);
  pfs_init(false);
  pfs_format(true /* write erase headers */);
  load_resource_fixture_in_flash(RESOURCES_FIXTURE_PATH, SYSTEM_RESOURCES_FIXTURE_NAME, false /* is_next */);
  load_resource_fixture_on_pfs(RESOURCES_FIXTURE_PATH, CHINESE_FIXTURE_NAME, "lang");
  //cl_assert(resource_has_valid_system_resources());

  memset(&s_font_info, 0, sizeof(s_font_info));
  memset(&s_font_cache, 0, sizeof(s_font_cache));
  s_test_fallback_font = NULL;
  s_test_emoji_font = NULL;

  FontCache *font_cache = &s_font_cache;
  memset(font_cache->cache_keys, 0, sizeof(font_cache->cache_keys));
  memset(font_cache->cache_data, 0, sizeof(font_cache->cache_data));
  keyed_circular_cache_init(&font_cache->line_cache, font_cache->cache_keys,
                            font_cache->cache_data, sizeof(LineCacheData), LINE_CACHE_SIZE);

  resource_init();
}

void test_text_resources__cleanup(void) {

}

void test_text_resources__init_font(void) {
  uint32_t gothic_18_handle = RESOURCE_ID_GOTHIC_18;
  cl_assert(text_resources_init_font(0, gothic_18_handle, 0, &s_font_info));
  cl_assert_equal_i(FONT_VERSION(s_font_info.base.md.version), 3);
  cl_assert_equal_i(s_font_info.base.md.wildcard_codepoint, WILDCARD_CODEPOINT);
  cl_assert_equal_i(s_font_info.base.md.codepoint_bytes, 2);
}

void test_text_resources__horiz_advance(void) {
  uint32_t gothic_18_handle = RESOURCE_ID_GOTHIC_18;
  cl_assert(text_resources_init_font(0, gothic_18_handle, 0, &s_font_info));

  int8_t horiz_advance = text_resources_get_glyph_horiz_advance(&s_font_cache, 'a', &s_font_info);
  cl_assert(horiz_advance != 0);
  cl_assert_equal_i(horiz_advance,
      text_resources_get_glyph_horiz_advance(&s_font_cache, 'a', &s_font_info));
  cl_assert_equal_i(horiz_advance,
      text_resources_get_glyph_horiz_advance(&s_font_cache, 'a', &s_font_info));
}

void test_text_resources__horiz_advance_multiple(void) {
  uint32_t gothic_18_handle = RESOURCE_ID_GOTHIC_18;
  cl_assert(text_resources_init_font(0, gothic_18_handle, 0, &s_font_info));

  int8_t horiz_advance = text_resources_get_glyph_horiz_advance(&s_font_cache, 'a', &s_font_info);
  cl_assert(horiz_advance != 0);
  cl_assert_equal_i(horiz_advance,
      text_resources_get_glyph_horiz_advance(&s_font_cache, 'a', &s_font_info));

  horiz_advance = text_resources_get_glyph_horiz_advance(&s_font_cache, 'b', &s_font_info);
  cl_assert(horiz_advance != 0);
  cl_assert_equal_i(horiz_advance,
     text_resources_get_glyph_horiz_advance(&s_font_cache, 'b', &s_font_info));

  horiz_advance = text_resources_get_glyph_horiz_advance(&s_font_cache, 'c', &s_font_info);
  cl_assert(horiz_advance != 0);
  cl_assert_equal_i(horiz_advance,
     text_resources_get_glyph_horiz_advance(&s_font_cache, 'c', &s_font_info));
}

void test_text_resources__get_glyph_multiple(void) {
  const uint8_t a_glyph_data_bytes[] = {0x2e, 0x42, 0x2e, 0x63, 0xb6};
  const uint8_t b_glyph_data_bytes[] = {0x21, 0x84, 0x36, 0x63, 0x8c, 0x71, 0x36};
  const uint8_t c_glyph_data_bytes[] = {0x2e, 0x86, 0x10, 0x42, 0x74};

  uint32_t gothic_18_handle = RESOURCE_ID_GOTHIC_18;
  cl_assert(text_resources_init_font(0, gothic_18_handle, 0, &s_font_info));

  uint8_t glyph_size_bytes;
  const GlyphData *glyph;

  glyph = text_resources_get_glyph(&s_font_cache, 'a', &s_font_info, NULL);
  glyph_size_bytes = glyph_get_size_bytes(glyph);
  cl_assert_equal_m(a_glyph_data_bytes, glyph->data, glyph_size_bytes);

  glyph = text_resources_get_glyph(&s_font_cache, 'b', &s_font_info, NULL);
  glyph_size_bytes = glyph_get_size_bytes(glyph);
  cl_assert_equal_m(b_glyph_data_bytes, glyph->data, glyph_size_bytes);

  glyph = text_resources_get_glyph(&s_font_cache, 'c', &s_font_info, NULL);
  glyph_size_bytes = glyph_get_size_bytes(glyph);
  cl_assert_equal_m(c_glyph_data_bytes, glyph->data, glyph_size_bytes);
}

void test_text_resources__init_backup_font(void) {
  // load the built in fallback font
  uint32_t font_fallback = RESOURCE_ID_FONT_FALLBACK_INTERNAL;
  cl_assert(text_resources_init_font(0, font_fallback, 0, &s_font_info));
  cl_assert_equal_i(FONT_VERSION(s_font_info.base.md.version), 3);
  cl_assert_equal_i(s_font_info.base.md.wildcard_codepoint, WILDCARD_CODEPOINT);
}

void test_text_resources__test_backup_wildcard(void) {
  const uint8_t wildcard_bytes[] = {0x3f, 0xc6, 0x18, 0x63, 0x8c, 0x31, 0xc6, 0x0f};

  uint32_t font_fallback = RESOURCE_ID_FONT_FALLBACK_INTERNAL;
  cl_assert(text_resources_init_font(0, font_fallback, 0, &s_font_info));

  int8_t horiz_advance = text_resources_get_glyph_horiz_advance(&s_font_cache,
                                                                WILDCARD_CODEPOINT, &s_font_info);
  cl_assert(horiz_advance != 0);
  cl_assert_equal_i(horiz_advance,
      text_resources_get_glyph_horiz_advance(&s_font_cache, WILDCARD_CODEPOINT, &s_font_info));

  const GlyphData *glyph = text_resources_get_glyph(&s_font_cache,
                                                    WILDCARD_CODEPOINT, &s_font_info, NULL);
  cl_assert_equal_i(glyph->header.width_px, 5);
  cl_assert_equal_i(glyph->header.height_px, 12);
  uint8_t glyph_size_bytes = glyph_get_size_bytes(glyph);
  cl_assert_equal_m(wildcard_bytes, glyph->data, glyph_size_bytes);
}

void test_text_resources__test_gothic_wildcard(void) {
  uint8_t wildcard_bytes[] = {0xff, 0x60, 0x30, 0x18, 0x0c, 0x06, 0x83, 0xc1, 0x60, 0x30, 0x18, 0x0c, 0xfe, 0x01};

  uint32_t gothic_18_handle = RESOURCE_ID_GOTHIC_18;
  cl_assert(text_resources_init_font(0, gothic_18_handle, 0, &s_font_info));

  int8_t horiz_advance = text_resources_get_glyph_horiz_advance(&s_font_cache,
                                                                WILDCARD_CODEPOINT,
                                                                &s_font_info);
  cl_assert(horiz_advance != 0);
  cl_assert_equal_i(horiz_advance,
      text_resources_get_glyph_horiz_advance(&s_font_cache, WILDCARD_CODEPOINT, &s_font_info));

  const GlyphData *glyph = text_resources_get_glyph(&s_font_cache,
                                                    WILDCARD_CODEPOINT,
                                                    &s_font_info, NULL);
  cl_assert_equal_i(glyph->header.width_px, 7);
  cl_assert_equal_i(glyph->header.height_px, 15);
  uint8_t glyph_size_bytes = glyph_get_size_bytes(glyph);
  cl_assert_equal_m(wildcard_bytes, glyph->data, glyph_size_bytes);
}

void test_text_resources__extended_font(void) {
  const uint8_t chinese_wildcard_bytes[] = {0xff, 0x60, 0x30, 0x18, 0x0c, 0x06, 0x83, 0xc1, 0x60, 0x30, 0x18, 0x0c, 0xfe, 0x01};
  const uint8_t a_glyph_data_bytes[] = {0x2e, 0x42, 0x2e, 0x63, 0xb6};
  const uint8_t chinese_glyph_data_bytes[] = {0x00, 0x0C, 0xE2, 0x01, 0x0F, 0x80, 0x30, 0x40,
                                              0x08, 0x10, 0x04, 0x08, 0x82, 0xFC, 0xFF, 0x80,
                                              0x00, 0x44, 0x00, 0x26, 0x01, 0x11, 0x41, 0x08,
                                              0x11, 0x84, 0x04, 0x82, 0xC0, 0x01, 0x40, 0x00};

  uint32_t gothic_18_bold_handle = RESOURCE_ID_GOTHIC_18;
  uint32_t gothic_18_bold_extended_handle = RESOURCE_ID_GOTHIC_18_EXTENDED;
  cl_assert(text_resources_init_font(0, gothic_18_bold_handle, gothic_18_bold_extended_handle, &s_font_info));
  cl_assert(s_font_info.loaded);
  cl_assert(s_font_info.extended);

  uint8_t glyph_size_bytes;
  const GlyphData *glyph;

  glyph = text_resources_get_glyph(&s_font_cache, 'a', &s_font_info, NULL);
  cl_assert(glyph != NULL);
  glyph_size_bytes = glyph_get_size_bytes(glyph);
  cl_assert_equal_m(a_glyph_data_bytes, glyph->data, glyph_size_bytes);

  glyph = text_resources_get_glyph(&s_font_cache, 0x4E50 /* 乐 */, &s_font_info, NULL);
  // the chinese pbpack contains the letter 你, it should succeed
  cl_assert(glyph != NULL);
  glyph_size_bytes = glyph_get_size_bytes(glyph);
  cl_assert_equal_m(chinese_glyph_data_bytes, glyph->data, glyph_size_bytes);

  glyph = text_resources_get_glyph(&s_font_cache, 0x8888 /* 袈 */, &s_font_info, NULL);
  // the chinese pbpack does not contain the letter 袈, it should return the wildcard
  cl_assert(glyph != NULL);
  glyph_size_bytes = glyph_get_size_bytes(glyph);
  cl_assert_equal_m(chinese_wildcard_bytes, glyph->data, glyph_size_bytes);
}

void test_text_resources__test_emoji_font(void) {
  const uint8_t phone_bytes[] = {0xfe, 0x81, 0x81, 0x3c, 0x66, 0x42, 0xc3, 0xe7, 0xff, 0x00, 0x00, 0x00};

  uint32_t gothic_18_emoji_handle = RESOURCE_ID_GOTHIC_18_EMOJI;
  cl_assert(text_resources_init_font(0, gothic_18_emoji_handle, 0, &s_font_info));

  uint8_t glyph_size_bytes;
  const GlyphData *glyph;

  const Codepoint PHONE_CODEPOINT = 0x260E;
  glyph = text_resources_get_glyph(&s_font_cache, PHONE_CODEPOINT, &s_font_info, NULL);
  cl_assert(glyph != NULL);
  glyph_size_bytes = glyph_get_size_bytes(glyph);
  cl_assert_equal_m(phone_bytes, glyph->data, glyph_size_bytes);
}

void DISABLED_test_text_resources__test_emoji_fallback(void) {
  const uint8_t phone_bytes[] = {0xfe, 0x81, 0x81, 0x3c, 0x66, 0x42, 0xc3, 0xe7, 0xff, 0x00, 0x00, 0x00};

  uint32_t gothic_18_handle = RESOURCE_ID_GOTHIC_18;
  cl_assert(text_resources_init_font(0, gothic_18_handle, 0, &s_font_info));

  uint8_t glyph_size_bytes;
  const GlyphData *glyph;

  const Codepoint PHONE_CODEPOINT = 0x260E;
  glyph = text_resources_get_glyph(&s_font_cache, PHONE_CODEPOINT, &s_font_info, NULL);
  cl_assert(glyph != NULL);
  glyph_size_bytes = glyph_get_size_bytes(glyph);
  cl_assert_equal_m(phone_bytes, glyph->data, glyph_size_bytes);
}


// Per-glyph fallback chain
////////////////////////////////////

// Core regression: a codepoint absent from the primary font but present in the configured
// fallback font now resolves to the real fallback glyph, instead of silently yielding only the
// primary wildcard box.
void test_text_resources__per_glyph_fallback(void) {
  // primary: plain gothic 18, no extension -> lacks CJK 0x4E50
  cl_assert(text_resources_init_font(0, RESOURCE_ID_GOTHIC_18, 0, &s_font_info));

  // Baseline with no fallback: 0x4E50 misses -> gothic wildcard, NOT the CJK glyph.
  const uint8_t gothic_wildcard[] = {0xff, 0x60, 0x30, 0x18, 0x0c, 0x06, 0x83, 0xc1,
                                     0x60, 0x30, 0x18, 0x0c, 0xfe, 0x01};
  const GlyphData *g0 = text_resources_get_glyph(&s_font_cache, 0x4E50, &s_font_info, NULL);
  cl_assert(g0 != NULL);
  cl_assert_equal_m(gothic_wildcard, g0->data, glyph_get_size_bytes(g0));

  // Install fallback = gothic 18 + extended (which contains 0x4E50) as the system fallback font.
  static FontInfo s_fallback;
  memset(&s_fallback, 0, sizeof(s_fallback));
  cl_assert(text_resources_init_font(0, RESOURCE_ID_GOTHIC_18,
                                     RESOURCE_ID_GOTHIC_18_EXTENDED, &s_fallback));
  s_test_fallback_font = &s_fallback;

  // Fresh cache so the primary negative-cache entry from g0 does not short-circuit.
  memset(&s_font_cache, 0, sizeof(s_font_cache));
  keyed_circular_cache_init(&s_font_cache.line_cache, s_font_cache.cache_keys,
                            s_font_cache.cache_data, sizeof(LineCacheData), LINE_CACHE_SIZE);

  const uint8_t cjk_bytes[] = {0x00, 0x0C, 0xE2, 0x01, 0x0F, 0x80, 0x30, 0x40, 0x08, 0x10, 0x04,
                               0x08, 0x82, 0xFC, 0xFF, 0x80, 0x00, 0x44, 0x00, 0x26, 0x01, 0x11,
                               0x41, 0x08, 0x11, 0x84, 0x04, 0x82, 0xC0, 0x01, 0x40, 0x00};
  const GlyphData *g1 = text_resources_get_glyph(&s_font_cache, 0x4E50, &s_font_info, NULL);
  cl_assert(g1 != NULL);
  cl_assert_equal_m(cjk_bytes, g1->data, glyph_get_size_bytes(g1));  // real fallback glyph
}

// A glyph served by the system fallback font reports a positive baseline adjust, so the renderer
// can drop it onto the primary font's baseline instead of the fallback font's.
void test_text_resources__baseline_adjust_for_fallback_font(void) {
  // primary: gothic 24, no extension -> lacks CJK 0x4E50
  cl_assert(text_resources_init_font(0, RESOURCE_ID_GOTHIC_24, 0, &s_font_info));

  // A glyph from the primary font itself needs no adjust.
  int16_t adjust = -1;
  const GlyphData *g0 = text_resources_get_glyph(&s_font_cache, 'a', &s_font_info, &adjust);
  cl_assert(g0 != NULL);
  cl_assert_equal_i(adjust, 0);

  // Install a shorter fallback (gothic 18 + extended) that carries 0x4E50.
  static FontInfo s_fallback;
  memset(&s_fallback, 0, sizeof(s_fallback));
  cl_assert(text_resources_init_font(0, RESOURCE_ID_GOTHIC_18,
                                     RESOURCE_ID_GOTHIC_18_EXTENDED, &s_fallback));
  s_test_fallback_font = &s_fallback;

  // Fresh cache so the primary negative-cache entry does not short-circuit.
  memset(&s_font_cache, 0, sizeof(s_font_cache));
  keyed_circular_cache_init(&s_font_cache.line_cache, s_font_cache.cache_keys,
                            s_font_cache.cache_data, sizeof(LineCacheData), LINE_CACHE_SIZE);

  // The reference is the fallback font's own baseline, i.e. its BASE resource height.
  const int16_t expected = (int16_t)s_font_info.base.md.max_height -
                           (int16_t)s_fallback.base.md.max_height;
  cl_assert(expected > 0);  // premise: the fallback really is shorter

  adjust = -1;
  const GlyphData *g1 = text_resources_get_glyph(&s_font_cache, 0x4E50, &s_font_info, &adjust);
  cl_assert(g1 != NULL);
  cl_assert_equal_i(adjust, expected);
}

// Regression: an extension is part of the SAME font and its glyph offsets are already baked
// against that font's base baseline, so a short extension (as shipped for ar_SA, where a 36px
// font carries a 20px-tall extension) must never be pushed down.
void test_text_resources__baseline_adjust_zero_for_own_extension(void) {
  // gothic 36 base with a much shorter extension, mimicking the ar_SA font pack
  cl_assert(text_resources_init_font(0, RESOURCE_ID_GOTHIC_36,
                                     RESOURCE_ID_GOTHIC_18_EXTENDED, &s_font_info));
  cl_assert(s_font_info.extended);
  // premise: the extension really is shorter than the base
  cl_assert(s_font_info.extension.md.max_height < s_font_info.base.md.max_height);

  // Compare against the extension's own glyph bytes, so a wildcard from the base cannot pass.
  const uint8_t cjk_bytes[] = {0x00, 0x0C, 0xE2, 0x01, 0x0F, 0x80, 0x30, 0x40, 0x08, 0x10, 0x04,
                               0x08, 0x82, 0xFC, 0xFF, 0x80, 0x00, 0x44, 0x00, 0x26, 0x01, 0x11,
                               0x41, 0x08, 0x11, 0x84, 0x04, 0x82, 0xC0, 0x01, 0x40, 0x00};
  int16_t adjust = -1;
  const GlyphData *g = text_resources_get_glyph(&s_font_cache, 0x4E50, &s_font_info, &adjust);
  cl_assert(g != NULL);
  cl_assert_equal_m(cjk_bytes, g->data, glyph_get_size_bytes(g));
  cl_assert_equal_i(adjust, 0);
}

// The case this all exists for: a 36px font has no emoji font of its own size, so the 28px one
// takes over and its glyphs must be dropped 8px to reach the 36px baseline.
void test_text_resources__baseline_adjust_for_emoji_font(void) {
  cl_assert(text_resources_init_font(0, RESOURCE_ID_GOTHIC_36, 0, &s_font_info));
  cl_assert_equal_i(s_font_info.base.md.max_height, 36);

  // fonts_get_system_emoji_font_for_size(36) picks the 28px emoji font, which this stub serves.
  static FontInfo s_emoji;
  memset(&s_emoji, 0, sizeof(s_emoji));
  cl_assert(text_resources_init_font(0, RESOURCE_ID_GOTHIC_28_EMOJI, 0, &s_emoji));
  cl_assert_equal_i(s_emoji.base.md.max_height, 28);
  s_test_emoji_font = &s_emoji;

  const Codepoint PHONE_CODEPOINT = 0x260E;
  int16_t adjust = -1;
  const GlyphData *g = text_resources_get_glyph(&s_font_cache, PHONE_CODEPOINT, &s_font_info,
                                                &adjust);
  cl_assert(g != NULL);
  cl_assert_equal_i(adjust, 8);  // 36px primary baseline - 28px emoji font baseline
}

// A codepoint present in the primary font is served by the primary font; the fallback is not
// consulted. Wire a different fallback whose 'a' glyph provably differs from the primary's (a
// smaller font has different glyph dimensions and therefore different bitmap bytes), capture both
// dynamically, assert the premise, then verify the result equals the PRIMARY bytes.
void test_text_resources__fallback_not_used_when_present(void) {
  // Step 1: capture the primary (GOTHIC_18) 'a' bytes.
  cl_assert(text_resources_init_font(0, RESOURCE_ID_GOTHIC_18, 0, &s_font_info));
  memset(&s_font_cache, 0, sizeof(s_font_cache));
  keyed_circular_cache_init(&s_font_cache.line_cache, s_font_cache.cache_keys,
                            s_font_cache.cache_data, sizeof(LineCacheData), LINE_CACHE_SIZE);

  const GlyphData *primary_g = text_resources_get_glyph(&s_font_cache, 'a', &s_font_info, NULL);
  cl_assert(primary_g != NULL);
  uint8_t primary_size = glyph_get_size_bytes(primary_g);

  // Copy the primary glyph bitmap so we have a stable snapshot even after a cache reset.
  uint8_t primary_bytes[CACHE_GLYPH_SIZE];
  cl_assert(primary_size <= sizeof(primary_bytes));
  memcpy(primary_bytes, primary_g->data, primary_size);

  // Step 2: init GOTHIC_14 as the fallback and fetch its 'a' directly.
  // GOTHIC_14 is shorter (14px cap-height vs 18px), so its 'a' bitmap has different dimensions
  // and therefore different bytes — the premise check in step 3 will catch any regression where
  // they happen to collide.
  static FontInfo s_fallback;
  memset(&s_fallback, 0, sizeof(s_fallback));
  cl_assert(text_resources_init_font(0, RESOURCE_ID_GOTHIC_14, 0, &s_fallback));

  // Fresh cache so there is no stale entry from the primary fetch above.
  memset(&s_font_cache, 0, sizeof(s_font_cache));
  keyed_circular_cache_init(&s_font_cache.line_cache, s_font_cache.cache_keys,
                            s_font_cache.cache_data, sizeof(LineCacheData), LINE_CACHE_SIZE);

  const GlyphData *fallback_g = text_resources_get_glyph(&s_font_cache, 'a', &s_fallback, NULL);
  cl_assert(fallback_g != NULL);
  uint8_t fallback_size = glyph_get_size_bytes(fallback_g);

  uint8_t fallback_bytes[CACHE_GLYPH_SIZE];
  cl_assert(fallback_size <= sizeof(fallback_bytes));
  memcpy(fallback_bytes, fallback_g->data, fallback_size);

  // Step 3: assert the premise — the fallback 'a' must differ from the primary 'a'.
  // If it doesn't, this test cannot distinguish "primary used" from "fallback used".
  cl_assert(primary_size != fallback_size ||
            memcmp(primary_bytes, fallback_bytes, primary_size) != 0);

  // Step 4: install the fallback, reset the cache, and fetch 'a' through the primary font.
  // The result must equal the CAPTURED PRIMARY bytes, proving the fallback was not consulted.
  s_test_fallback_font = &s_fallback;
  memset(&s_font_cache, 0, sizeof(s_font_cache));
  keyed_circular_cache_init(&s_font_cache.line_cache, s_font_cache.cache_keys,
                            s_font_cache.cache_data, sizeof(LineCacheData), LINE_CACHE_SIZE);

  const GlyphData *result_g = text_resources_get_glyph(&s_font_cache, 'a', &s_font_info, NULL);
  cl_assert(result_g != NULL);
  cl_assert_equal_i(glyph_get_size_bytes(result_g), primary_size);
  cl_assert_equal_m(primary_bytes, result_g->data, primary_size);
}

// When the primary font IS the system fallback font, the self-reference guard skips the redundant
// second lookup: no loop or crash, and a missing codepoint still yields the primary wildcard.
void test_text_resources__fallback_self_reference(void) {
  cl_assert(text_resources_init_font(0, RESOURCE_ID_GOTHIC_18, 0, &s_font_info));
  s_test_fallback_font = &s_font_info;  // primary is its own fallback

  const uint8_t gothic_wildcard[] = {0xff, 0x60, 0x30, 0x18, 0x0c, 0x06, 0x83, 0xc1,
                                     0x60, 0x30, 0x18, 0x0c, 0xfe, 0x01};
  const GlyphData *g = text_resources_get_glyph(&s_font_cache, 0x4E50, &s_font_info, NULL);
  cl_assert(g != NULL);                 // returns primary wildcard, no hang
  cl_assert_equal_m(gothic_wildcard, g->data, glyph_get_size_bytes(g));
}

// A codepoint missing from BOTH primary and fallback still yields the primary wildcard,
// proving the chain-exhausted tail survives.
void test_text_resources__fallback_miss_yields_primary_wildcard(void) {
  cl_assert(text_resources_init_font(0, RESOURCE_ID_GOTHIC_18, 0, &s_font_info));
  static FontInfo s_fallback;
  memset(&s_fallback, 0, sizeof(s_fallback));
  cl_assert(text_resources_init_font(0, RESOURCE_ID_GOTHIC_18, 0, &s_fallback));
  s_test_fallback_font = &s_fallback;

  const uint8_t gothic_wildcard[] = {0xff, 0x60, 0x30, 0x18, 0x0c, 0x06, 0x83, 0xc1,
                                     0x60, 0x30, 0x18, 0x0c, 0xfe, 0x01};
  const GlyphData *g = text_resources_get_glyph(&s_font_cache, 0x8888 /* absent */, &s_font_info,
                                                NULL);
  cl_assert(g != NULL);
  cl_assert_equal_m(gothic_wildcard, g->data, glyph_get_size_bytes(g));
}

// Glyph-presence-driven routing (in-font base<->extension rescue)
////////////////////////////////////

// Mirror of #1709: a codepoint whose Unicode class routes it to the EXTENSION but which lives only
// in the BASE (e.g. pi U+03C0) must still resolve to the base glyph once an extension is loaded,
// instead of degrading to the fallback/wildcard. Works on the shipped fixtures: GOTHIC_18 base
// carries U+03C0, the CJK extension does not.
void test_text_resources__extension_routed_miss_rescued_from_base(void) {
  const Codepoint PI = 0x03C0;

  // Reference: base-only GOTHIC_18 resolves U+03C0 to its own glyph (no extension, so it routes to
  // base directly).
  cl_assert(text_resources_init_font(0, RESOURCE_ID_GOTHIC_18, 0, &s_font_info));
  const GlyphData *base_g = text_resources_get_glyph(&s_font_cache, PI, &s_font_info, NULL);
  cl_assert(base_g != NULL);
  uint8_t base_size = glyph_get_size_bytes(base_g);
  uint8_t base_bytes[CACHE_GLYPH_SIZE];
  cl_assert(base_size <= sizeof(base_bytes));
  memcpy(base_bytes, base_g->data, base_size);

  // Now load GOTHIC_18 + CJK extension. U+03C0 is not latin-classified, so it routes to the
  // extension, which lacks it. Without the in-font rescue this misses and yields the wildcard.
  memset(&s_font_info, 0, sizeof(s_font_info));
  cl_assert(text_resources_init_font(0, RESOURCE_ID_GOTHIC_18,
                                     RESOURCE_ID_GOTHIC_18_EXTENDED, &s_font_info));
  cl_assert(s_font_info.extended);

  memset(&s_font_cache, 0, sizeof(s_font_cache));
  keyed_circular_cache_init(&s_font_cache.line_cache, s_font_cache.cache_keys,
                            s_font_cache.cache_data, sizeof(LineCacheData), LINE_CACHE_SIZE);

  int16_t adjust = -1;
  const GlyphData *g = text_resources_get_glyph(&s_font_cache, PI, &s_font_info, &adjust);
  cl_assert(g != NULL);
  cl_assert_equal_i(glyph_get_size_bytes(g), base_size);
  cl_assert_equal_m(base_bytes, g->data, base_size);  // rescued from base, not the wildcard
  cl_assert_equal_i(adjust, 0);                        // owner is the primary font
}

// #1709 itself: a latin-classified codepoint absent from the base but present in the extension must
// resolve from the extension. The shipped GOTHIC_18 base already carries the extension's only latin
// codepoint (U+2026), so we pair the extension with a base font that lacks it — GOTHIC_18_EMOJI —
// which faithfully exercises the base-routed -> extension rescue path.
void test_text_resources__base_routed_miss_rescued_from_extension(void) {
  const Codepoint ELLIPSIS = 0x2026;

  // Reference: the extension's own U+2026 glyph, captured by loading the extension as a base font.
  static FontInfo s_probe;
  memset(&s_probe, 0, sizeof(s_probe));
  cl_assert(text_resources_init_font(0, RESOURCE_ID_GOTHIC_18_EXTENDED, 0, &s_probe));
  const GlyphData *ext_g = text_resources_get_glyph(&s_font_cache, ELLIPSIS, &s_probe, NULL);
  cl_assert(ext_g != NULL);
  uint8_t ext_size = glyph_get_size_bytes(ext_g);
  uint8_t ext_bytes[CACHE_GLYPH_SIZE];
  cl_assert(ext_size <= sizeof(ext_bytes));
  memcpy(ext_bytes, ext_g->data, ext_size);

  // Base = emoji font (lacks U+2026) + CJK extension (has U+2026). U+2026 is latin-classified, so
  // it routes to base, misses, and must be rescued from the extension.
  memset(&s_font_info, 0, sizeof(s_font_info));
  cl_assert(text_resources_init_font(0, RESOURCE_ID_GOTHIC_18_EMOJI,
                                     RESOURCE_ID_GOTHIC_18_EXTENDED, &s_font_info));
  cl_assert(s_font_info.extended);

  memset(&s_font_cache, 0, sizeof(s_font_cache));
  keyed_circular_cache_init(&s_font_cache.line_cache, s_font_cache.cache_keys,
                            s_font_cache.cache_data, sizeof(LineCacheData), LINE_CACHE_SIZE);

  int16_t adjust = -1;
  const GlyphData *g = text_resources_get_glyph(&s_font_cache, ELLIPSIS, &s_font_info, &adjust);
  cl_assert(g != NULL);
  cl_assert_equal_i(glyph_get_size_bytes(g), ext_size);
  cl_assert_equal_m(ext_bytes, g->data, ext_size);  // rescued from extension, not the wildcard
  cl_assert_equal_i(adjust, 0);                       // owner is the primary font
}

// Regression guards for the in-font rescue: (a) a codepoint present in the routed resource still
// wins over the other resource, and (b) a codepoint absent from both still yields the wildcard.
void test_text_resources__in_font_rescue_regressions(void) {
  const Codepoint ELLIPSIS = 0x2026;
  const Codepoint ABSENT = 0x8888;  // absent from base, extension and fallback

  // Base-only references for U+2026 (base carries its own ellipsis) and the wildcard for U+8888.
  cl_assert(text_resources_init_font(0, RESOURCE_ID_GOTHIC_18, 0, &s_font_info));

  const GlyphData *base_ellipsis = text_resources_get_glyph(&s_font_cache, ELLIPSIS, &s_font_info,
                                                            NULL);
  cl_assert(base_ellipsis != NULL);
  uint8_t base_ellipsis_size = glyph_get_size_bytes(base_ellipsis);
  uint8_t base_ellipsis_bytes[CACHE_GLYPH_SIZE];
  cl_assert(base_ellipsis_size <= sizeof(base_ellipsis_bytes));
  memcpy(base_ellipsis_bytes, base_ellipsis->data, base_ellipsis_size);

  const GlyphData *base_wildcard = text_resources_get_glyph(&s_font_cache, ABSENT, &s_font_info,
                                                            NULL);
  cl_assert(base_wildcard != NULL);
  uint8_t wildcard_size = glyph_get_size_bytes(base_wildcard);
  uint8_t wildcard_bytes[CACHE_GLYPH_SIZE];
  cl_assert(wildcard_size <= sizeof(wildcard_bytes));
  memcpy(wildcard_bytes, base_wildcard->data, wildcard_size);

  // Load GOTHIC_18 + CJK extension. Both base and extension carry U+2026 (with different bitmaps),
  // so the routed resource (base) must win.
  memset(&s_font_info, 0, sizeof(s_font_info));
  cl_assert(text_resources_init_font(0, RESOURCE_ID_GOTHIC_18,
                                     RESOURCE_ID_GOTHIC_18_EXTENDED, &s_font_info));
  cl_assert(s_font_info.extended);

  memset(&s_font_cache, 0, sizeof(s_font_cache));
  keyed_circular_cache_init(&s_font_cache.line_cache, s_font_cache.cache_keys,
                            s_font_cache.cache_data, sizeof(LineCacheData), LINE_CACHE_SIZE);

  // (a) routed resource wins: U+2026 resolves to the BASE bytes, not the extension's.
  const GlyphData *g_ellipsis = text_resources_get_glyph(&s_font_cache, ELLIPSIS, &s_font_info,
                                                         NULL);
  cl_assert(g_ellipsis != NULL);
  cl_assert_equal_i(glyph_get_size_bytes(g_ellipsis), base_ellipsis_size);
  cl_assert_equal_m(base_ellipsis_bytes, g_ellipsis->data, base_ellipsis_size);

  // (b) absent everywhere still yields the primary wildcard.
  const GlyphData *g_absent = text_resources_get_glyph(&s_font_cache, ABSENT, &s_font_info, NULL);
  cl_assert(g_absent != NULL);
  cl_assert_equal_i(glyph_get_size_bytes(g_absent), wildcard_size);
  cl_assert_equal_m(wildcard_bytes, g_absent->data, wildcard_size);
}

void test_text_resources__test_glyph_decompression(void) {
  // There is no way to get the list of glyphs present in a font with the existing API. This list
  // of ranges lists the 371 glyphs currently in fontname.ttf
  typedef struct Codepoint_Range {
    uint16_t start;
    uint16_t end;
  } CodePoint_Range;
  const CodePoint_Range codepoint_range[] = {
    { 0x0020, 0x007E }, { 0x00A0, 0x00AC }, { 0x00AE, 0x00D6 }, { 0x00D9, 0x017F },
    { 0x0192, 0x0192 }, { 0x01FC, 0x01FF }, { 0x0218, 0x021B }, { 0x02C6, 0x02DD },
    { 0x03C0, 0x03C0 }, { 0x2013, 0x2014 }, { 0x2018, 0x201A }, { 0x201C, 0x201E },
    { 0x2020, 0x2022 }, { 0x2026, 0x2026 }, { 0x2030, 0x2030 }, { 0x2039, 0x203A },
    { 0x2044, 0x2044 }, { 0x20AC, 0x20AC }, { 0x2122, 0x2122 }, { 0x2126, 0x2126 },
    { 0x2202, 0x2202 }, { 0x2206, 0x2206 }, { 0x220F, 0x220F }, { 0x2211, 0x2212 },
    { 0x221A, 0x221A }, { 0x221E, 0x221E }, { 0x222B, 0x222B }, { 0x2248, 0x2248 },
    { 0x2260, 0x2260 }, { 0x2264, 0x2265 }, { 0x25AF, 0x25AF }, { 0x25CA, 0x25CA },
    { 0xF6C3, 0xF6C3 }, { 0xFB01, 0xFB02 }
  };

  // Create a second FontInfo for the compressed font.
  // The uncompressed font will use the global.
  FontInfo font_info_compressed;
  memset(&font_info_compressed, 0, sizeof(font_info_compressed));

  // Load GOTHIC_18
  uint32_t gothic_18_handle = RESOURCE_ID_GOTHIC_18;
  cl_assert(text_resources_init_font(0, gothic_18_handle, 0, &s_font_info));
  cl_assert_equal_i(FONT_VERSION(s_font_info.base.md.version), 3);
  cl_assert(!HAS_FEATURE(s_font_info.base.md.version, VERSION_FIELD_FEATURE_RLE4));

  // Load GOTHIC_18_COMPRESSED. This is the same font, added by hand to the system resource pack.
  // To do this, simply copy the GOTHIC_18 stanza in resource/normal/base/resource_map.json, change
  // the name to include _COMPRESSED, and add the field: "compress": "RLE4". Rebuild, and run
  // ./tools/update_system_pbpack.sh
  // GOTHIC_18_COMPRESSED isn't in the regular pbpack; it has to be added by hand (see comment
  // above) and the resource_ids.auto.h override updated to give it a real ID. When it's absent
  // the symbol isn't defined, so skip the decompression coverage entirely.
#ifdef RESOURCE_ID_GOTHIC_18_COMPRESSED
  uint32_t gothic_18_compressed_handle = RESOURCE_ID_GOTHIC_18_COMPRESSED;
  if (gothic_18_compressed_handle == INVALID_RESOURCE) {
    return;
  }
  cl_assert(text_resources_init_font(0, gothic_18_compressed_handle, 0, &font_info_compressed));
  cl_assert_equal_i(FONT_VERSION(font_info_compressed.base.md.version), 3);
  cl_assert(HAS_FEATURE(font_info_compressed.base.md.version, VERSION_FIELD_FEATURE_RLE4));

  // For each glyph in the font, get both the compressed and uncompressed bit field & header, and
  // assert that they are identical (ignoring any possible garbage after the bitmap).
  // Load the uncompressed glyph into this local glyph_buffer, and use the font cache for the
  // compressed glyph.
  uint8_t glyph_buffer[sizeof(GlyphHeaderData) + CACHE_GLYPH_SIZE];
  for (unsigned index = 0; index < ARRAY_LENGTH(codepoint_range); ++index) {
    for (unsigned codepoint = codepoint_range[index].start;
         codepoint <= codepoint_range[index].end; ++codepoint) {

      const GlyphData *glyph = text_resources_get_glyph(&s_font_cache, codepoint, &s_font_info,
                                                        NULL);
      cl_assert(glyph);

      unsigned glyph_size = sizeof(GlyphHeaderData) + glyph_get_size_bytes(glyph);
      memcpy(glyph_buffer, glyph->data, glyph_size);

      glyph = text_resources_get_glyph(&s_font_cache, codepoint, &font_info_compressed, NULL);
      cl_assert(glyph);

      cl_assert_equal_m(glyph->data, glyph_buffer, glyph_size);
    }
  }
#endif
}
