/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "pbl/services/imaging.h"

#include "applib/graphics/gtypes.h"
#include "pbl/services/comm_session/session.h"

#include <stdlib.h>
#include <string.h>

// Stubs & Fakes
///////////////////////////////////////////////////////////

#include "fake_session.h"
#include "fake_system_task.h"

#include "stubs_bt_lock.h"
#include "stubs_hexdump.h"
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"

// imaging.c only needs the row-size rules from the graphics code; provide them here so the test
// controls the expected sizes without pulling in the renderer.
uint16_t gbitmap_format_get_row_size_bytes(int16_t width, GBitmapFormat format) {
  switch (format) {
    case GBitmapFormat1Bit:
      return ((width + 31) / 32) * 4;
    case GBitmapFormat8Bit:
      return width;
    case GBitmapFormat4BitPalette:
      return (width * 4 + 7) / 8;
    case GBitmapFormat2BitPalette:
      return (width * 2 + 7) / 8;
    case GBitmapFormat1BitPalette:
      return (width + 7) / 8;
    default:
      return 0;
  }
}

// Delivery capture
///////////////////////////////////////////////////////////

static int s_deliveries;
static uint8_t s_last_token;
static GBitmap *s_last_bitmap;

static void prv_free_last_bitmap(void) {
  if (s_last_bitmap) {
    kernel_free(s_last_bitmap->addr);
    kernel_free(s_last_bitmap->palette);
    kernel_free(s_last_bitmap);
    s_last_bitmap = NULL;
  }
}

static void prv_art_handler(uint8_t token, GBitmap *bitmap) {
  s_deliveries++;
  s_last_token = token;
  prv_free_last_bitmap();
  s_last_bitmap = bitmap;
}

// Helpers
///////////////////////////////////////////////////////////

#define TEST_TOKEN (42)

//! Build an ImageResponse into `out`: header, then (on First) the image header + palette, then
//! `pixel_len` pixel bytes. `chunk_len_field` is what goes on the wire, which tests may set to a
//! lie; pass the real pixel count for well-formed messages.
static size_t prv_build_response(uint8_t *out, uint8_t token, uint8_t flags, uint32_t offset,
                                 uint16_t chunk_len_field, uint16_t width, uint16_t height,
                                 uint8_t format, const uint8_t *palette, uint8_t palette_count,
                                 const uint8_t *pixels, size_t pixel_len) {
  ImagingResponseHeader *hdr = (ImagingResponseHeader *)out;
  *hdr = (ImagingResponseHeader) {
    .cmd = ImagingCmdIDResponse,
    .token = token,
    .flags = flags,
    .offset = offset,
    .chunk_len = chunk_len_field,
  };
  uint8_t *cursor = out + sizeof(*hdr);
  if (flags & ImagingResponseFlagFirst) {
    *cursor++ = (uint8_t)(width & 0xff);
    *cursor++ = (uint8_t)(width >> 8);
    *cursor++ = (uint8_t)(height & 0xff);
    *cursor++ = (uint8_t)(height >> 8);
    *cursor++ = format;
    *cursor++ = palette_count;
    memcpy(cursor, palette, palette_count);
    cursor += palette_count;
  }
  memcpy(cursor, pixels, pixel_len);
  cursor += pixel_len;
  return cursor - out;
}

static void prv_receive(const uint8_t *msg, size_t length) {
  imaging_protocol_msg_callback(NULL, msg, length);
}

//! A well-formed 4x2 4-bpp image: row size 2, total 4 pixel bytes, 3 palette entries.
static const uint8_t s_palette[] = { 0xC0, 0xF0, 0xFF };
static const uint8_t s_pixels[] = { 0x01, 0x20, 0x12, 0x01 };

static void prv_receive_valid_image(uint8_t token) {
  uint8_t buf[64];
  const size_t len = prv_build_response(
      buf, token, ImagingResponseFlagFirst | ImagingResponseFlagLast, 0, sizeof(s_pixels),
      4, 2, ImagingFormat4BitPalette, s_palette, sizeof(s_palette), s_pixels, sizeof(s_pixels));
  prv_receive(buf, len);
}

// Tests
///////////////////////////////////////////////////////////

static Transport *s_transport;

void test_imaging__initialize(void) {
  fake_comm_session_init();
  s_transport = fake_transport_create(TransportDestinationSystem, NULL, NULL);
  fake_transport_set_connected(s_transport, true);
  imaging_register_handler(ImagingImageTypeAlbumArt, prv_art_handler);
  s_deliveries = 0;
  s_last_token = 0;
  s_last_bitmap = NULL;
  // Reset any latched state left over from a previous test
  const PebbleCommSessionEvent closed_event = {
    .is_open = false,
    .is_system = true,
  };
  imaging_handle_comm_session_event(&closed_event);
}

void test_imaging__cleanup(void) {
  prv_free_last_bitmap();
  fake_comm_session_cleanup();
}

void test_imaging__single_chunk_image(void) {
  prv_receive_valid_image(TEST_TOKEN);
  cl_assert_equal_i(s_deliveries, 1);
  cl_assert_equal_i(s_last_token, TEST_TOKEN);
  cl_assert(s_last_bitmap != NULL);
  cl_assert_equal_i(s_last_bitmap->bounds.size.w, 4);
  cl_assert_equal_i(s_last_bitmap->bounds.size.h, 2);
  cl_assert_equal_i(s_last_bitmap->row_size_bytes, 2);
  cl_assert_equal_i(s_last_bitmap->info.format, GBitmapFormat4BitPalette);
  cl_assert(memcmp(s_last_bitmap->addr, s_pixels, sizeof(s_pixels)) == 0);
  cl_assert_equal_i(((GColor *)s_last_bitmap->palette)[1].argb, s_palette[1]);
}

void test_imaging__multi_chunk_image(void) {
  uint8_t buf[64];
  size_t len = prv_build_response(buf, TEST_TOKEN, ImagingResponseFlagFirst, 0, 2,
                                  4, 2, ImagingFormat4BitPalette,
                                  s_palette, sizeof(s_palette), s_pixels, 2);
  prv_receive(buf, len);
  cl_assert_equal_i(s_deliveries, 0);
  len = prv_build_response(buf, TEST_TOKEN, ImagingResponseFlagLast, 2, 2,
                           0, 0, 0, NULL, 0, s_pixels + 2, 2);
  prv_receive(buf, len);
  cl_assert_equal_i(s_deliveries, 1);
  cl_assert(s_last_bitmap != NULL);
  cl_assert(memcmp(s_last_bitmap->addr, s_pixels, sizeof(s_pixels)) == 0);
}

void test_imaging__no_image(void) {
  uint8_t buf[32];
  const size_t len = prv_build_response(buf, TEST_TOKEN, ImagingResponseFlagNoImage, 0, 0,
                                        0, 0, 0, NULL, 0, NULL, 0);
  prv_receive(buf, len);
  cl_assert_equal_i(s_deliveries, 1);
  cl_assert(s_last_bitmap == NULL);
}

void test_imaging__truncated_header_rejected(void) {
  uint8_t buf[64];
  const size_t len = prv_build_response(
      buf, TEST_TOKEN, ImagingResponseFlagFirst | ImagingResponseFlagLast, 0, sizeof(s_pixels),
      4, 2, ImagingFormat4BitPalette, s_palette, sizeof(s_palette), s_pixels, sizeof(s_pixels));
  // Truncate inside the image header, inside the palette, and inside the response header
  prv_receive(buf, sizeof(ImagingResponseHeader) + 3);
  prv_receive(buf, sizeof(ImagingResponseHeader) + 6 + 1);
  prv_receive(buf, sizeof(ImagingResponseHeader) - 1);
  cl_assert_equal_i(s_deliveries, 0);
}

void test_imaging__bad_dimensions_rejected(void) {
  uint8_t buf[64];
  // Width over the cap
  size_t len = prv_build_response(buf, TEST_TOKEN,
                                  ImagingResponseFlagFirst | ImagingResponseFlagLast, 0, 4,
                                  301, 2, ImagingFormat4BitPalette,
                                  s_palette, sizeof(s_palette), s_pixels, 4);
  prv_receive(buf, len);
  // Zero height
  len = prv_build_response(buf, TEST_TOKEN,
                           ImagingResponseFlagFirst | ImagingResponseFlagLast, 0, 4,
                           4, 0, ImagingFormat4BitPalette,
                           s_palette, sizeof(s_palette), s_pixels, 4);
  prv_receive(buf, len);
  cl_assert_equal_i(s_deliveries, 0);
}

void test_imaging__bad_palette_rejected(void) {
  uint8_t big_palette[17] = { 0 };
  uint8_t buf[64];
  // More palette entries than a 4-bpp image can have
  size_t len = prv_build_response(buf, TEST_TOKEN,
                                  ImagingResponseFlagFirst | ImagingResponseFlagLast, 0, 4,
                                  4, 2, ImagingFormat4BitPalette,
                                  big_palette, sizeof(big_palette), s_pixels, 4);
  prv_receive(buf, len);
  // A palettized format with no palette at all
  len = prv_build_response(buf, TEST_TOKEN,
                           ImagingResponseFlagFirst | ImagingResponseFlagLast, 0, 4,
                           4, 2, ImagingFormat4BitPalette, NULL, 0, s_pixels, 4);
  prv_receive(buf, len);
  cl_assert_equal_i(s_deliveries, 0);
}

void test_imaging__oversized_image_rejected(void) {
  uint8_t buf[64];
  // 300x300 8-bit = 90000 bytes, over IMAGING_MAX_BYTES
  const size_t len = prv_build_response(buf, TEST_TOKEN,
                                        ImagingResponseFlagFirst | ImagingResponseFlagLast, 0, 4,
                                        300, 300, ImagingFormat8BitColor, NULL, 0, s_pixels, 4);
  prv_receive(buf, len);
  cl_assert_equal_i(s_deliveries, 0);
}

void test_imaging__non_contiguous_chunk_resets(void) {
  uint8_t buf[64];
  size_t len = prv_build_response(buf, TEST_TOKEN, ImagingResponseFlagFirst, 0, 2,
                                  4, 2, ImagingFormat4BitPalette,
                                  s_palette, sizeof(s_palette), s_pixels, 2);
  prv_receive(buf, len);
  // Wrong offset: skips a byte
  len = prv_build_response(buf, TEST_TOKEN, ImagingResponseFlagLast, 3, 1,
                           0, 0, 0, NULL, 0, s_pixels + 3, 1);
  prv_receive(buf, len);
  cl_assert_equal_i(s_deliveries, 0);
  // The transfer was reset: a well-formed follow-up chunk must also be ignored
  len = prv_build_response(buf, TEST_TOKEN, ImagingResponseFlagLast, 2, 2,
                           0, 0, 0, NULL, 0, s_pixels + 2, 2);
  prv_receive(buf, len);
  cl_assert_equal_i(s_deliveries, 0);
}

void test_imaging__lying_chunk_len_resets(void) {
  uint8_t buf[64];
  size_t len = prv_build_response(buf, TEST_TOKEN, ImagingResponseFlagFirst, 0, 2,
                                  4, 2, ImagingFormat4BitPalette,
                                  s_palette, sizeof(s_palette), s_pixels, 2);
  prv_receive(buf, len);
  // chunk_len claims more pixel bytes than the message carries
  len = prv_build_response(buf, TEST_TOKEN, ImagingResponseFlagLast, 2, 60,
                           0, 0, 0, NULL, 0, s_pixels + 2, 2);
  prv_receive(buf, len);
  cl_assert_equal_i(s_deliveries, 0);
}

void test_imaging__incomplete_transfer_not_delivered(void) {
  uint8_t buf[64];
  // Last chunk arrives before all pixel bytes were received
  const size_t len = prv_build_response(buf, TEST_TOKEN,
                                        ImagingResponseFlagFirst | ImagingResponseFlagLast, 0, 2,
                                        4, 2, ImagingFormat4BitPalette,
                                        s_palette, sizeof(s_palette), s_pixels, 2);
  prv_receive(buf, len);
  cl_assert_equal_i(s_deliveries, 0);
}

void test_imaging__token_mismatch_resets(void) {
  uint8_t buf[64];
  size_t len = prv_build_response(buf, TEST_TOKEN, ImagingResponseFlagFirst, 0, 2,
                                  4, 2, ImagingFormat4BitPalette,
                                  s_palette, sizeof(s_palette), s_pixels, 2);
  prv_receive(buf, len);
  // Continuation with a different token must not complete the transfer
  len = prv_build_response(buf, TEST_TOKEN + 1, ImagingResponseFlagLast, 2, 2,
                           0, 0, 0, NULL, 0, s_pixels + 2, 2);
  prv_receive(buf, len);
  cl_assert_equal_i(s_deliveries, 0);
}

void test_imaging__unsupported_latches_until_session_close(void) {
  cl_assert(imaging_is_type_supported(ImagingImageTypeAlbumArt));

  uint8_t buf[32];
  const size_t len = prv_build_response(buf, TEST_TOKEN, ImagingResponseFlagUnsupported, 0, 0,
                                        0, 0, 0, NULL, 0, NULL, 0);
  prv_receive(buf, len);
  cl_assert_equal_i(s_deliveries, 1);
  cl_assert(s_last_bitmap == NULL);
  cl_assert(!imaging_is_type_supported(ImagingImageTypeAlbumArt));

  // Closing the system session clears the latch
  const PebbleCommSessionEvent closed_event = {
    .is_open = false,
    .is_system = true,
  };
  imaging_handle_comm_session_event(&closed_event);
  cl_assert(imaging_is_type_supported(ImagingImageTypeAlbumArt));
}

void test_imaging__request_payload_format(void) {
  cl_assert(imaging_request_album_art(7, ImagingFormat4BitPalette, 166, 166, "Title", "Artist"));
  fake_comm_session_process_send_next();
  const uint8_t expected[] = {
    0x01, 7, 0x00, 0x02, 166, 0, 166, 0,
    5, 'T', 'i', 't', 'l', 'e',
    6, 'A', 'r', 't', 'i', 's', 't',
  };
  fake_transport_assert_sent(s_transport, 0, 0x35, expected, sizeof(expected));
}
