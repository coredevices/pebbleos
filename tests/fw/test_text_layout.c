/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/util/iterator.h"
#include "applib/graphics/utf8.h"
#include "applib/graphics/text.h"
#include "applib/graphics/text_layout_private.h"
#include "applib/graphics/graphics.h"
#include "applib/graphics/framebuffer.h"

#include "clar.h"


///////////////////////////////////////////////////////////
// Stubs

#include "stubs_logging.h"
#include "stubs_passert.h"
#include "stubs_hexdump.h"
#include "stubs_heap.h"
#include "stubs_pebble_tasks.h"
#include "stubs_pbl_malloc.h"

#include "stubs_applib_resource.h"
#include "stubs_app_state.h"
#include "stubs_fonts.h"
#include "stubs_text_resources.h"
#include "stubs_text_render.h"
#include "stubs_reboot_reason.h"
#include "stubs_resources.h"
#include "stubs_syscalls.h"
#include "stubs_compiled_with_legacy2_sdk.h"

#if CONFIG_SCREEN_COLOR_DEPTH_BITS == 8
#define FONT_LINE_DELTA 2
#else
#define FONT_LINE_DELTA 0
#endif

///////////////////////////////////////////////////////////
// Tests

// NOTE: Font height is set to be 10 in stubs_fonts.h

void test_text_layout__ellipsis_overflow(void) {
  GContext gcontext;
  FrameBuffer *fb = malloc(sizeof(FrameBuffer));
  framebuffer_init(fb, &(GSize) { DISP_COLS, DISP_ROWS });

  graphics_context_init(&gcontext, fb, GContextInitializationMode_App);
  framebuffer_clear(fb);

  GFont font = (GFont) { 0 };
  GRect box = (GRect) { (GPoint) { 0, 0 }, (GSize) { 20 * HORIZ_ADVANCE_PX + 1, 13 } };
  TextLayoutExtended layout = (TextLayoutExtended) {
    .hash = 0,
    .box = (GRect) { (GPoint) { 0, 0 }, (GSize) { 20 * HORIZ_ADVANCE_PX + 1, 13 } },
    .font = (GFont) { 0 },
    .overflow_mode = GTextOverflowModeWordWrap,
    .alignment = GTextAlignmentLeft,
    .max_used_size = (GSize) { 0, 0 }
  };
  layout.box = box;

  graphics_draw_text(&gcontext,
               "Twitter\n@pebble is talking about a lot of really really cool important stuff.\n",
                     font, box,
                     GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, (void*)&layout);
  cl_assert_equal_i(layout.box.size.w, box.size.w);
  cl_assert_equal_i(layout.max_used_size.w, 8 * HORIZ_ADVANCE_PX);

  graphics_draw_text(&gcontext, "Twitter\n\n\n\n\n\n\n\n", font, box,
                     GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, (void*)&layout);
  cl_assert_equal_i(layout.box.size.w, box.size.w);
  cl_assert_equal_i(layout.max_used_size.w, 8 * HORIZ_ADVANCE_PX);

  graphics_draw_text(&gcontext, "Twitter    \n   \n \n\n   \n \n \n\n     ", font, box,
                     GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, (void*)&layout);
  cl_assert_equal_i(layout.box.size.w, box.size.w);
  cl_assert_equal_i(layout.max_used_size.w, 8 * HORIZ_ADVANCE_PX);
}

void test_text_layout__cache_vert_overflow(void) {
  GContext gcontext = (GContext) { };
  GFont font = (GFont) { 0 };
  GRect box = (GRect) { (GPoint) { 0, 0 }, (GSize) { 4 * HORIZ_ADVANCE_PX + 1, 2 * FONT_HEIGHT + 1 } };
  TextLayoutExtended layout = (TextLayoutExtended) {
    .hash = 0,
    .box = (GRect) { (GPoint) { 0, 0 }, (GSize) { 7 * HORIZ_ADVANCE_PX + 1, FONT_HEIGHT - 1 } },
    .font = (GFont) { 0 },
    .overflow_mode = GTextOverflowModeWordWrap,
    .alignment = GTextAlignmentLeft,
    .max_used_size = (GSize) { 0, 0 }
  };

  graphics_text_layout_get_max_used_size(&gcontext, "JR Whopper", font, box, GTextOverflowModeFill, GTextAlignmentLeft, (void*)&layout);

  cl_assert(layout.box.size.w == box.size.w);
  cl_assert_equal_i(layout.max_used_size.w, 4 * HORIZ_ADVANCE_PX);

  cl_assert_equal_i(layout.max_used_size.h, 2 * FONT_HEIGHT); // 2 lines - all that will completely fit in the box ("Jr\nWho-")

  graphics_text_layout_get_max_used_size(&gcontext, "JR Whopper", font, box, GTextOverflowModeWordWrap, GTextAlignmentLeft, (void*)&layout);

  cl_assert(layout.box.size.w == box.size.w);
  cl_assert_equal_i(layout.max_used_size.w, 4 * HORIZ_ADVANCE_PX);

  cl_assert_equal_i(layout.max_used_size.h, 3 * FONT_HEIGHT); // 3 lines - one line extra being laid out so that it will clip ("Jr\nWho-\npper")

  graphics_text_layout_get_max_used_size(&gcontext, "JR Whopper 123", font, box, GTextOverflowModeWordWrap, GTextAlignmentLeft, (void*)&layout);

  cl_assert_equal_i(layout.max_used_size.w, 4 * HORIZ_ADVANCE_PX);

  cl_assert_equal_i(layout.max_used_size.h, 3 * FONT_HEIGHT); // 3 lines - but not 4, since the fourth has no chance of appearing ("Jr\nWho-\npper")
}

void test_text_layout__cache_vert_overflow_first_line(void) {
  GContext gcontext = (GContext) { };
  GFont font = (GFont) { 0 };
  GRect box = (GRect) { (GPoint) { 0, 0 }, (GSize) { 5 * HORIZ_ADVANCE_PX + 1, 7 } };
  TextLayoutExtended layout = (TextLayoutExtended) {
    .hash = 0,
    .box = (GRect) { (GPoint) { 0, 0 }, (GSize) { 7 * HORIZ_ADVANCE_PX + 1, FONT_HEIGHT - 1 } },
    .font = (GFont) { 0 },
    .overflow_mode = GTextOverflowModeWordWrap,
    .alignment = GTextAlignmentLeft,
    .max_used_size = (GSize) { 0, 0 }
  };
  // In all cases, the first line should be laid out (not truncated)

  graphics_text_layout_get_max_used_size(&gcontext, "JR Whopper", font, box, GTextOverflowModeFill, GTextAlignmentLeft, (void*)&layout);

  cl_assert(layout.box.size.w == box.size.w);
  cl_assert_equal_i(layout.max_used_size.w, 5 * HORIZ_ADVANCE_PX); // "JR..."

  cl_assert_equal_i(layout.max_used_size.h, FONT_HEIGHT);

  graphics_text_layout_get_max_used_size(&gcontext, "JR Whopper", font, box, GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, (void*)&layout);

  cl_assert(layout.box.size.w == box.size.w);
  cl_assert_equal_i(layout.max_used_size.w, 5 * HORIZ_ADVANCE_PX); // "JR..."

  cl_assert_equal_i(layout.max_used_size.h, FONT_HEIGHT);

  graphics_text_layout_get_max_used_size(&gcontext, "JR Whopper", font, box, GTextOverflowModeWordWrap, GTextAlignmentLeft, (void*)&layout);

  cl_assert_equal_i(layout.max_used_size.w, 2 * HORIZ_ADVANCE_PX); // "JR\nWhopper"

  cl_assert_equal_i(layout.max_used_size.h, FONT_HEIGHT);
}

void test_text_layout__cache_vert_overflow_with_newline(void) {
  GContext gcontext = (GContext) { };
  GFont font = (GFont) { 0 };
  GRect box = (GRect) { (GPoint) { 0, 0 }, (GSize) { 5 * HORIZ_ADVANCE_PX + 1, 2 * FONT_HEIGHT + 1 } };
  TextLayoutExtended layout = (TextLayoutExtended) {
    .hash = 0,
    .box = (GRect) { (GPoint) { 0, 0 }, (GSize) { 7 * HORIZ_ADVANCE_PX + 1, FONT_HEIGHT - 1 } },
    .font = (GFont) { 0 },
    .overflow_mode = GTextOverflowModeWordWrap,
    .alignment = GTextAlignmentLeft,
    .max_used_size = (GSize) { 0, 0 }
  };

  graphics_text_layout_get_max_used_size(&gcontext, "JR\n\nWhop", font, box, GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, (void*)&layout);

  cl_assert_equal_i(layout.box.size.w, box.size.w);
  cl_assert_equal_i(layout.max_used_size.w, 2 * HORIZ_ADVANCE_PX); // only the JR, since Whop is not being laid out

  cl_assert_equal_i(layout.max_used_size.h, 2 * FONT_HEIGHT); // Nothing - save for the first line - will be rendered below the box

  graphics_text_layout_get_max_used_size(&gcontext, "JR\n\nWhop", font, box, GTextOverflowModeWordWrap, GTextAlignmentLeft, (void*)&layout);

  cl_assert_equal_i(layout.box.size.w, box.size.w);
  cl_assert_equal_i(layout.max_used_size.w, 4 * HORIZ_ADVANCE_PX); // Includes Whop - as it may be partially rendered at the bottom of the box

  cl_assert_equal_i(layout.max_used_size.h, 3 * FONT_HEIGHT); // The blank line before Whop is still being laid out, however, so it is still included in the height

  graphics_text_layout_get_max_used_size(&gcontext, "JR\n\n\nWhop", font, box, GTextOverflowModeWordWrap, GTextAlignmentLeft, (void*)&layout);

  cl_assert_equal_i(layout.box.size.w, box.size.w);
  cl_assert_equal_i(layout.max_used_size.w, 2 * HORIZ_ADVANCE_PX); // Back to only JR - as the line being laid out from y=20-30px is empty (and the line from 30-40, Whop, is truncated as it can never appear)

  cl_assert_equal_i(layout.max_used_size.h, 3 * FONT_HEIGHT); // Same as above - the blank line is still laid out

  graphics_text_layout_get_max_used_size(&gcontext, "JR\n\n\nWhop", font, box, GTextOverflowModeFill, GTextAlignmentLeft, (void*)&layout);

  cl_assert_equal_i(layout.box.size.w, box.size.w);
  cl_assert_equal_i(layout.max_used_size.w, 4 * HORIZ_ADVANCE_PX); // Fill replaces \n's with spaces, so we will always fill the full horizontal width ("JR   Whop" wraps to "JR\nWhop")

  cl_assert_equal_i(layout.max_used_size.h, 2 * FONT_HEIGHT); // Same behaviour as TrailingEllipsis in this regard
}

void test_text_layout__pathological_1(void) {
  GContext gcontext;
  FrameBuffer *fb = malloc(sizeof(FrameBuffer));
  framebuffer_init(fb, &(GSize) { DISP_COLS, DISP_ROWS });

  GFont font = (GFont) { 0 };
  GRect box = (GRect) { (GPoint) { 0, 0 }, (GSize) {40, 250 * FONT_HEIGHT} };

  graphics_context_init(&gcontext, fb, GContextInitializationMode_App);
  framebuffer_clear(fb);
  graphics_draw_text(&gcontext, "\n", font, box,
                     GTextOverflowModeFill, GTextAlignmentLeft, NULL);
  graphics_draw_text(&gcontext, "\n\n", font, box,
                     GTextOverflowModeFill, GTextAlignmentLeft, NULL);
  graphics_draw_text(&gcontext, "\1\n", font, box,
                     GTextOverflowModeFill, GTextAlignmentLeft, NULL);
  graphics_draw_text(&gcontext, "", font, box,
                     GTextOverflowModeFill, GTextAlignmentLeft, NULL);
}

void test_text_layout__max_used_size(void) {
  char *empty_string = "";
  char *singleton = "A";
  char *doubleton = "AA";
  GFont font = (GFont){ 0 };
  GRect box = (GRect) { (GPoint) { 0, 0 }, (GSize) { 3 * HORIZ_ADVANCE_PX + 1, FONT_HEIGHT + 1 } };
  TextLayoutExtended layout = (TextLayoutExtended) { };
  GContext gcontext = (GContext) { };

  layout.hash = 0;
  layout.box = (GRect) { (GPoint) { 0, 0 }, (GSize) { 7 * HORIZ_ADVANCE_PX + 1, FONT_HEIGHT - 1} };
  layout.font = (GFont) { 0 };
  layout.overflow_mode = GTextOverflowModeWordWrap;
  layout.alignment = GTextAlignmentLeft;
  layout.max_used_size = (GSize) { 0, 0 };

  // Ensure that the empty string properly resets our sized boundaries
  graphics_text_layout_get_max_used_size(&gcontext, empty_string, font, box,
      GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, (void*)&layout);

  cl_assert_equal_i(layout.box.size.w, box.size.w);
  cl_assert_equal_i(layout.max_used_size.h, 0);
  cl_assert_equal_i(layout.max_used_size.w, 0);

  graphics_text_layout_get_max_used_size(&gcontext, singleton, font, box,
      GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, (void*)&layout);

  cl_assert_equal_i(layout.box.size.w, box.size.w);
  cl_assert_equal_i(layout.max_used_size.w, 1 * HORIZ_ADVANCE_PX);
  cl_assert_equal_i(layout.max_used_size.h, FONT_HEIGHT);

  // Ensure that the empty string properly resets our sized boundaries
  graphics_text_layout_get_max_used_size(&gcontext, empty_string, font, box,
      GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, (void*)&layout);

  cl_assert_equal_i(layout.box.size.w, box.size.w);
  cl_assert_equal_i(layout.max_used_size.h, 0);
  cl_assert_equal_i(layout.max_used_size.w, 0);

  graphics_text_layout_get_max_used_size(&gcontext, doubleton, font, box,
      GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, (void*)&layout);

  cl_assert_equal_i(layout.box.size.w, box.size.w);
  cl_assert_equal_i(layout.max_used_size.w, 2 * HORIZ_ADVANCE_PX);
  cl_assert_equal_i(layout.max_used_size.h, FONT_HEIGHT);
}

void test_text_layout__disable_paging(void) {
  TextLayoutExtended l = {.flow_data.paging.page_on_screen.size_h = 123};
  graphics_text_attributes_restore_default_paging((GTextLayoutCacheRef) &l);
  cl_assert_equal_i(l.flow_data.paging.page_on_screen.size_h, 0);
}

void test_text_layout__enable_paging(void) {
  TextLayoutExtended l = {};
  graphics_text_attributes_enable_paging((GTextLayoutCacheRef) &l, GPoint(1, 2), GRect(3, 4, 5, 6));

  cl_assert_equal_i(l.flow_data.paging.origin_on_screen.x, 1);
  cl_assert_equal_i(l.flow_data.paging.origin_on_screen.y, 2);
  cl_assert_equal_i(l.flow_data.paging.page_on_screen.origin_y, 4);
  cl_assert_equal_i(l.flow_data.paging.page_on_screen.size_h, 6);
}

void test_text_layout__disable_text_flow(void) {
  TextLayoutExtended l = {.flow_data.perimeter.impl = (const GPerimeter *)(1234)};
  graphics_text_attributes_restore_default_text_flow((GTextLayoutCacheRef) &l);
  cl_assert_equal_p(l.flow_data.perimeter.impl, NULL);
}

// just a fake value to have something to compare against
const GPerimeter * const g_perimeter_for_display = (const GPerimeter *) &g_perimeter_for_display;

void test_text_layout__enable_text_flow(void) {
  TextLayoutExtended l = {};
  graphics_text_attributes_enable_screen_text_flow((GTextLayoutCacheRef) &l, 123);
  cl_assert_equal_p(l.flow_data.perimeter.impl, g_perimeter_for_display);
  cl_assert_equal_i(l.flow_data.perimeter.inset, 123);
}

void test_text_layout__create_destroy(void) {
  GTextAttributes *attributes = graphics_text_attributes_create();
  cl_assert_equal_p(attributes->font, NULL);
  cl_assert_equal_i(attributes->hash, 0);
  graphics_text_attributes_destroy(attributes);
}

void test_text_layout__get_default_flow_data(void) {
  const TextLayoutFlowData *data1 = graphics_text_layout_get_flow_data(NULL);
  cl_assert(data1 != NULL);
  cl_assert_equal_p(data1->perimeter.impl, NULL);
  cl_assert_equal_i(data1->paging.page_on_screen.size_h, 0);

  // change SP so that we can make sure that graphics_text_layout_get_flow_data doesn't rely on it
  uint8_t change_stack[data1->paging.page_on_screen.size_h + 500];
  memset(change_stack, 0xff, 500);

  const TextLayoutFlowData *data2 = graphics_text_layout_get_flow_data(NULL);
  cl_assert_equal_p(data1, data2);

  // values are still 0
  cl_assert_equal_p(data2->perimeter.impl, NULL);
  cl_assert_equal_i(data2->paging.page_on_screen.size_h, 0);
}

#include "applib/legacy2/ui/text_layer_legacy2.h"
void test_text_layout__delta(void) {
  GContext gcontext = (GContext) { };
  GFont font = (GFont) { 0 };
  GRect box = (GRect) { (GPoint) { 0, 0 }, (GSize) { 4 * HORIZ_ADVANCE_PX + 1, 2 * (FONT_HEIGHT + FONT_LINE_DELTA) + 1 } };
  TextLayoutExtended layout = (TextLayoutExtended) {
    .hash = 0,
    .box = (GRect) { (GPoint) { 0, 0 }, (GSize) { 7 * HORIZ_ADVANCE_PX + 1, FONT_HEIGHT - 1 } },
    .font = (GFont) { 0 },
    .overflow_mode = GTextOverflowModeWordWrap,
    .alignment = GTextAlignmentLeft,
    .max_used_size = (GSize) { 0, 0 }
  };

  if (!process_manager_compiled_with_legacy2_sdk()) {
    graphics_text_layout_set_line_spacing_delta((void*)&layout, FONT_LINE_DELTA);
  }

  graphics_text_layout_get_max_used_size(&gcontext, "JR Whopper", font, box, GTextOverflowModeFill, GTextAlignmentLeft, (void*)&layout);
  cl_assert(layout.box.size.w == box.size.w);
  cl_assert_equal_i(layout.max_used_size.w, 4 * HORIZ_ADVANCE_PX);
  // 2 lines - all that will completely fit in the box ("Jr\nWho-")
  cl_assert_equal_i(layout.max_used_size.h, 2 * (FONT_HEIGHT + FONT_LINE_DELTA));

  graphics_text_layout_get_max_used_size(&gcontext, "JR Whopper", font, box, GTextOverflowModeWordWrap, GTextAlignmentLeft, (void*)&layout);
  cl_assert(layout.box.size.w == box.size.w);
  cl_assert_equal_i(layout.max_used_size.w, 4 * HORIZ_ADVANCE_PX);
  // 3 lines - one line extra being laid out so that it will clip ("Jr\nWho-\npper")
  cl_assert_equal_i(layout.max_used_size.h, 3 * (FONT_HEIGHT + FONT_LINE_DELTA));

  graphics_text_layout_get_max_used_size(&gcontext, "JR Whopper 123", font, box, GTextOverflowModeWordWrap, GTextAlignmentLeft, (void*)&layout);
  cl_assert_equal_i(layout.max_used_size.w, 4 * HORIZ_ADVANCE_PX);
  // 3 lines - but not 4, since the fourth has no chance of appearing ("Jr\nWho-\npper\n 123")
  cl_assert_equal_i(layout.max_used_size.h, 3 * (FONT_HEIGHT + FONT_LINE_DELTA));

  // Update line spacing and ensure the text layout gets updated
  if (!process_manager_compiled_with_legacy2_sdk()) {
    graphics_text_layout_set_line_spacing_delta((void*)&layout, FONT_LINE_DELTA - 1);
    cl_assert_equal_i(graphics_text_layout_get_line_spacing_delta((void*)&layout), (FONT_LINE_DELTA - 1));
    cl_assert_equal_i(layout.max_used_size.h, 3 * (FONT_HEIGHT + FONT_LINE_DELTA));
    cl_assert_equal_i(layout.hash, 0);
  }
  graphics_text_layout_get_max_used_size(&gcontext, "JR Whopper 123", font, box, GTextOverflowModeWordWrap, GTextAlignmentLeft, (void*)&layout);
  cl_assert(layout.hash != 0);
  cl_assert_equal_i(layout.max_used_size.w, 4 * HORIZ_ADVANCE_PX);
  if (!process_manager_compiled_with_legacy2_sdk()) {
    // 3 lines - but not 4, since the fourth has no chance of appearing ("Jr\nWho-\npper\n 123")
    cl_assert_equal_i(layout.max_used_size.h, 3 * (FONT_HEIGHT + (FONT_LINE_DELTA - 1)));
  } else {
    cl_assert_equal_i(layout.max_used_size.h, 3 * FONT_HEIGHT);
  }

  if (!process_manager_compiled_with_legacy2_sdk()) {
    // Test negative spacing
    graphics_text_layout_set_line_spacing_delta((void*)&layout, (-FONT_HEIGHT));
    graphics_text_layout_get_max_used_size(&gcontext, "JR Whopper 123", font, box, GTextOverflowModeWordWrap, GTextAlignmentLeft, (void*)&layout);
    cl_assert_equal_i(layout.max_used_size.w, 4 * HORIZ_ADVANCE_PX);
    // 4 lines - all four show up but all overlapped so 0 height is returned ("Jr\nWho-\npper\n 123")
    cl_assert_equal_i(layout.max_used_size.h, 0);

    graphics_text_layout_set_line_spacing_delta((void*)&layout, (1 - FONT_HEIGHT));
    graphics_text_layout_get_max_used_size(&gcontext, "JR Whopper 123", font, box, GTextOverflowModeWordWrap, GTextAlignmentLeft, (void*)&layout);
    // 4 lines - all four show up but 1 pixel height per line is returned ("Jr\nWho-\npper\n 123")
    cl_assert_equal_i(layout.max_used_size.h, 4);

    graphics_text_layout_set_line_spacing_delta((void*)&layout, (-4 * FONT_HEIGHT));
    graphics_text_layout_get_max_used_size(&gcontext, "JR Whopper 123", font, box, GTextOverflowModeWordWrap, GTextAlignmentLeft, (void*)&layout);
    // 4 lines spaced out at 10-40 = -30 pixels each ("Jr\nWho-\npper\n 123")
    cl_assert_equal_i(layout.max_used_size.h, -120);
  }
}

void test_text_layout__special_codepoints(void) {
  GContext gcontext;
  FrameBuffer *fb = malloc(sizeof(FrameBuffer));
  framebuffer_init(fb, &(GSize) { DISP_COLS, DISP_ROWS });

  graphics_context_init(&gcontext, fb, GContextInitializationMode_App);
  framebuffer_clear(fb);

  GFont font = (GFont) { 0 };
  GRect box = (GRect) { (GPoint) { 0, 0 }, (GSize) { 20 * HORIZ_ADVANCE_PX + 1, 13 } };
  TextLayoutExtended layout = (TextLayoutExtended) {
    .hash = 0,
    .box = (GRect) { (GPoint) { 0, 0 }, (GSize) { 20 * HORIZ_ADVANCE_PX + 1, 13 } },
    .font = (GFont) { 0 },
    .overflow_mode = GTextOverflowModeWordWrap,
    .alignment = GTextAlignmentLeft,
    .max_used_size = (GSize) { 0, 0 }
  };
  layout.box = box;

  graphics_draw_text(&gcontext,
                     "\xE2\x80\x8F" // Left-To-Right mark
                     "\xEF\xB8\x8E" // Variation Selector 1
                     "\xF0\x9F\x8F\xBB", // White skin tone codepoint
                     font, box,
                     GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft, (void*)&layout);
  cl_assert_equal_i(layout.box.size.w, box.size.w);
  cl_assert_equal_i(layout.max_used_size.w, 0 * HORIZ_ADVANCE_PX);
}
