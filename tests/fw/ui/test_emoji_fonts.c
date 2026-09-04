/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/graphics/framebuffer.h"
#include "applib/graphics/graphics.h"
#include "resource/resource.h"
#include "resource/resource_ids.auto.h"
#include "util/graphics.h"
#include "pbl/util/size.h"

#include "clar.h"

#include <stdio.h>

// Fakes
/////////////////////

#include "fixtures/load_test_resources.h"

// Stubs
/////////////////////

#include "stubs_analytics.h"
#include "stubs_app_install_manager.h"
#include "stubs_app_state.h"
#include "stubs_bootbits.h"
#include "stubs_heap.h"
#include "stubs_logging.h"
#include "stubs_memory_layout.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"
#include "stubs_pebble_tasks.h"
#include "stubs_print.h"
#include "stubs_prompt.h"
#include "stubs_serial.h"
#include "stubs_shell_prefs.h"
#include "stubs_sleep.h"
#include "stubs_syscalls.h"
#include "stubs_task_watchdog.h"
#include "stubs_ui_window.h"
#include "stubs_unobstructed_area.h"

// Helper Functions
/////////////////////
#include "../graphics/test_graphics.h"
#include "../graphics/util.h"

// Setup and Teardown
////////////////////////////////////

static FrameBuffer *fb;
static GContext s_ctx;
static FontInfo s_font_info;

static GBitmap *s_dest_bitmap;

void test_emoji_fonts__initialize(void) {
  fake_spi_flash_init(0, 0x1000000);
  pfs_init(false);
  pfs_format(true /* write erase headers */);
  load_resource_fixture_in_flash(RESOURCES_FIXTURE_PATH, SYSTEM_RESOURCES_FIXTURE_NAME,
                                 false /* is_next */);

  memset(&s_font_info, 0, sizeof(s_font_info));

  resource_init();

  fb = malloc(sizeof(FrameBuffer));
  framebuffer_init(fb, &(GSize) {DISP_COLS, DISP_ROWS});

  test_graphics_context_init(&s_ctx, fb);
  framebuffer_clear(fb);
}

void test_emoji_fonts__cleanup(void) {
  free(fb);
  fb = NULL;

  gbitmap_destroy(s_dest_bitmap);
  s_dest_bitmap = NULL;
}

// Helpers
//////////////////////

static char s_emoji_string[] =
  "😄😃😀😊☺😉😍😘😚😗😙😜😝😛😳😁😔😌😒😞😣😢😂😭😥😪😰😅😓😩😫😨😱"
  "😠😡😤😖😆😋😷😎😴😵😲😟😧😈👿😮😬😐😕😯😶😇😏😑😺😸😻😽😼🙀😿😹😾💩"
  "👍👎👌👊✊✌👋✋👐👆👇👉👈🙌🙏☝👏💛💙💜💚❤💔💗💓💕💖💞💘💋🐥🎉💩🍻🍺"
  "💪🔥🐵🙈→►★🎤🎥📷🎵🎁";

static void prv_render_text(GContext *ctx, const char *text, GFont const font, const GRect *box,
                            const GTextOverflowMode overflow_mode, const GTextAlignment alignment,
                            GTextLayoutCacheRef layout, bool render) {
  if (render) {
    graphics_draw_text(ctx, text, font, *box, overflow_mode, alignment, layout);
  } else {
    graphics_text_layout_get_max_used_size(ctx, text, font, *box, overflow_mode, alignment,
                                           layout);
  }
}

static GSize prv_draw_emoji(GContext *ctx, const GRect *bounds, GFont font, bool render) {
  TextLayoutExtended layout = {
    .line_spacing_delta = 2, // Give some space for the larger emojis
  };
  graphics_context_set_text_color(ctx, GColorBlack);
  prv_render_text(ctx, s_emoji_string, font, bounds, GTextOverflowModeWordWrap,
                  GTextAlignmentLeft, (GTextLayoutCacheRef)&layout, true /* render */);
  return layout.max_used_size;
}

void prv_prepare_canvas(GSize bitmap_size, GColor background_color) {
  gbitmap_destroy(s_dest_bitmap);

  s_dest_bitmap = gbitmap_create_blank(bitmap_size, GBitmapFormat8Bit);

  s_ctx.dest_bitmap = *s_dest_bitmap;
  s_ctx.draw_state.clip_box.size = bitmap_size;
  s_ctx.draw_state.drawing_box.size = bitmap_size;

  memset(s_dest_bitmap->addr, background_color.argb,
         s_dest_bitmap->row_size_bytes * s_dest_bitmap->bounds.size.h);
}

void prv_prepare_canvas_and_render_emoji(ResourceId font_handle) {
  // Calculate canvas size necessary
  cl_assert(text_resources_init_font(0, font_handle, 0, &s_font_info));

  const GPoint margin = { 10, 10 }; // Canvas margins
  const int max_width = 300; // Canvas width, choose any visually pleasing width
  const int max_height = 2000; // Large size protected from overflow and automatically truncated
  const GSize max_size = { max_width, max_height };

  // FIXME: PBL-34261 graphics_text_layout_get_max_used_size reports a max of 192px in a unit test
  // with the default aplite framebuffer size. Work around this issue by creating a larger canvas.
  prv_prepare_canvas(max_size, GColorWhite);

  const GRect max_draw_frame = grect_inset_internal((GRect){ .size = max_size },
                                                    margin.x, margin.y);
  const GSize used_size = prv_draw_emoji(&s_ctx, &max_draw_frame, &s_font_info,
                                         false /* render */);

  // Resize the canvas to the used size.
  const GSize canvas_size = { max_width, used_size.h + 2 * margin.y };
  prv_prepare_canvas(canvas_size, GColorWhite);

  // Create the new canvas
  const GRect draw_frame = grect_inset_internal((GRect){ .size = canvas_size },
                                                margin.x, margin.y);
  prv_draw_emoji(&s_ctx, &draw_frame, &s_font_info, true /* render */);
}

// Tests
//////////////////////

void test_emoji_fonts__gothic_14_emoji(void) {
  prv_prepare_canvas_and_render_emoji(RESOURCE_ID_GOTHIC_14_EMOJI);
  cl_check(gbitmap_pbi_eq(s_dest_bitmap, TEST_PBI_FILE));
}

void test_emoji_fonts__gothic_18_emoji(void) {
  prv_prepare_canvas_and_render_emoji(RESOURCE_ID_GOTHIC_18_EMOJI);
  cl_check(gbitmap_pbi_eq(s_dest_bitmap, TEST_PBI_FILE));
}

void test_emoji_fonts__gothic_24_emoji(void) {
  prv_prepare_canvas_and_render_emoji(RESOURCE_ID_GOTHIC_24_EMOJI);
  cl_check(gbitmap_pbi_eq(s_dest_bitmap, TEST_PBI_FILE));
}

void test_emoji_fonts__gothic_28_emoji(void) {
  prv_prepare_canvas_and_render_emoji(RESOURCE_ID_GOTHIC_28_EMOJI);
  cl_check(gbitmap_pbi_eq(s_dest_bitmap, TEST_PBI_FILE));
}
