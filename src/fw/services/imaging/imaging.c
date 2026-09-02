/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/imaging.h"

#include "applib/graphics/gtypes.h"
#include "kernel/pbl_malloc.h"
#include "pbl/kernel/mutex.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"

#include <string.h>

static const uint16_t IMAGING_ENDPOINT = 0x35;

// Full-screen 4-bpp on the largest supported display (260x260) is ~34 KB. Cap generously and reject
// anything larger to bound kernel-heap use against a malformed or hostile phone.
#define IMAGING_MAX_BYTES (40 * 1024)
#define IMAGING_MAX_DIM (300)
#define IMAGING_PALETTE_ENTRIES (16)

static ImagingReceivedHandler s_handlers[ImagingImageTypeCount];

//! Guards the latch state below: requests come in on the requesting task (e.g. the Music app)
//! while responses are handled on KernelMain. The reassembly state (s_rx) is deliberately not
//! covered — it is only ever touched on KernelMain (endpoint receiver and comm-session events).
static PBL_MUTEX_DEFINE(s_lock);

// Image types the phone told us it can't serve (ImagingResponseFlagUnsupported), so we stop asking.
// Latched per session: the connected phone doesn't change what it supports mid-connection, and a
// reconnect (possibly to a different phone) clears it via prv_session_types.
static uint32_t s_unsupported_types;
static CommSession *s_latched_session;

static struct {
  bool active;
  uint8_t token;
  GBitmapFormat format;
  uint16_t width;
  uint16_t height;
  uint16_t row_size_bytes;
  uint32_t total_bytes;
  uint32_t received_bytes;
  uint8_t *pixels;
  GColor *palette;  // NULL for non-palette formats
} s_rx;

static void prv_rx_reset(void) {
  kernel_free(s_rx.pixels);
  kernel_free(s_rx.palette);
  s_rx = (__typeof__(s_rx)) { 0 };
}

void imaging_register_handler(ImagingImageType image_type, ImagingReceivedHandler handler) {
  if (image_type < ARRAY_LENGTH(s_handlers)) {
    s_handlers[image_type] = handler;
  }
}

//! The type a response answers, from the top nibble of its flags byte. Zero — which is what a phone
//! that doesn't set those bits sends — is album art. A value we don't know is left out of range so
//! it routes nowhere rather than to the wrong consumer.
static uint8_t prv_response_type(const ImagingResponseHeader *hdr) {
  return (hdr->flags & IMAGING_RESPONSE_FLAG_TYPE_MASK) >> IMAGING_RESPONSE_FLAG_TYPE_SHIFT;
}

static void prv_deliver(uint8_t token, uint8_t type, GBitmap *bitmap) {
  ImagingReceivedHandler handler = (type < ARRAY_LENGTH(s_handlers)) ? s_handlers[type] : NULL;
  if (handler) {
    handler(token, bitmap);
  } else if (bitmap) {
    kernel_free(bitmap->addr);
    kernel_free(bitmap->palette);
    kernel_free(bitmap);
  }
}

// Clear the unsupported-type latch when the system session changes (reconnect / different phone).
// The latch is also cleared explicitly when the session closes (imaging_handle_comm_session_event)
// so a recycled session pointer can't be mistaken for the old one. s_lock held by the caller.
static bool prv_type_latched_unsupported(CommSession *session, ImagingImageType image_type) {
  if (session != s_latched_session) {
    s_latched_session = session;
    s_unsupported_types = 0;
  }
  return (s_unsupported_types & (1u << image_type)) != 0;
}

bool imaging_is_type_supported(ImagingImageType image_type) {
  CommSession *session = comm_session_get_system_session();
  if (!session ||
      !comm_session_has_capability(session, CommSessionImagingSupport)) {
    return false;
  }
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  const bool latched = prv_type_latched_unsupported(session, image_type);
  pbl_mutex_unlock(&s_lock);
  return !latched;
}

bool imaging_request_album_art(uint8_t token, ImagingFormat format, uint16_t width, uint16_t height,
                               const char *title, const char *artist) {
  if (!imaging_is_type_supported(ImagingImageTypeAlbumArt)) {
    return false;
  }
  CommSession *session = comm_session_get_system_session();
  const size_t title_len = title ? MIN(strlen(title), 255) : 0;
  const size_t artist_len = artist ? MIN(strlen(artist), 255) : 0;
  uint8_t payload[sizeof(ImagingRequestHeader) + 2 + 255 + 255];
  ImagingRequestHeader *hdr = (ImagingRequestHeader *)payload;
  hdr->cmd = ImagingCmdIDRequest;
  hdr->token = token;
  hdr->image_type = ImagingImageTypeAlbumArt;
  hdr->format = format;
  hdr->width = width;
  hdr->height = height;
  uint8_t *cursor = payload + sizeof(*hdr);
  *cursor++ = (uint8_t)title_len;
  memcpy(cursor, title, title_len);
  cursor += title_len;
  *cursor++ = (uint8_t)artist_len;
  memcpy(cursor, artist, artist_len);
  cursor += artist_len;

  comm_session_send_data(session, IMAGING_ENDPOINT, payload, cursor - payload,
                         COMM_SESSION_DEFAULT_TIMEOUT);
  return true;
}

bool imaging_request_notification_image(uint8_t token, ImagingFormat format, uint16_t width,
                                        uint16_t height, const Uuid *item_id) {
  if (!item_id || !imaging_is_type_supported(ImagingImageTypeNotification)) {
    return false;
  }
  CommSession *session = comm_session_get_system_session();
  uint8_t payload[sizeof(ImagingRequestHeader) + UUID_SIZE];
  ImagingRequestHeader *hdr = (ImagingRequestHeader *)payload;
  hdr->cmd = ImagingCmdIDRequest;
  hdr->token = token;
  hdr->image_type = ImagingImageTypeNotification;
  hdr->format = format;
  hdr->width = width;
  hdr->height = height;
  memcpy(payload + sizeof(*hdr), item_id, UUID_SIZE);

  comm_session_send_data(session, IMAGING_ENDPOINT, payload, sizeof(payload),
                         COMM_SESSION_DEFAULT_TIMEOUT);
  return true;
}

static uint16_t prv_gbitmap_format_for(ImagingFormat format, GBitmapFormat *out) {
  switch (format) {
    case ImagingFormat8BitColor:
      *out = GBitmapFormat8Bit;
      return 0;  // no palette
    case ImagingFormat4BitPalette:
      *out = GBitmapFormat4BitPalette;
      return IMAGING_PALETTE_ENTRIES;
    case ImagingFormat1Bit:
    default:
      *out = GBitmapFormat1Bit;
      return 0;
  }
}

void imaging_protocol_msg_callback(CommSession *session, const uint8_t *msg, size_t length) {
  if (length < sizeof(ImagingResponseHeader)) {
    return;
  }
  const ImagingResponseHeader *hdr = (const ImagingResponseHeader *)msg;
  if (hdr->cmd != ImagingCmdIDResponse) {
    return;
  }
  const uint8_t *cursor = msg + sizeof(*hdr);
  const uint8_t *msg_end = msg + length;

  const uint8_t type = prv_response_type(hdr);

  if (hdr->flags & ImagingResponseFlagUnsupported) {
    // Phone can't serve this type: latch it off so we don't ask again this connection, and deliver
    // NULL so the current request resolves (the app treats it like "no image").
    if (type < ImagingImageTypeCount) {
      pbl_mutex_lock(&s_lock, PBL_FOREVER);
      s_unsupported_types |= (1u << type);
      pbl_mutex_unlock(&s_lock);
    }
    prv_rx_reset();
    prv_deliver(hdr->token, type, NULL);
    return;
  }

  if (hdr->flags & ImagingResponseFlagNoImage) {
    prv_rx_reset();
    prv_deliver(hdr->token, type, NULL);
    return;
  }

  if (hdr->flags & ImagingResponseFlagFirst) {
    prv_rx_reset();
    if (cursor + 6 > msg_end) {
      return;
    }
    const uint16_t width = cursor[0] | (cursor[1] << 8);
    const uint16_t height = cursor[2] | (cursor[3] << 8);
    const uint8_t format = cursor[4];
    const uint8_t palette_count = cursor[5];
    cursor += 6;
    GBitmapFormat gformat;
    const uint16_t max_palette = prv_gbitmap_format_for(format, &gformat);
    if (width == 0 || height == 0 || width > IMAGING_MAX_DIM || height > IMAGING_MAX_DIM ||
        palette_count > max_palette || (max_palette > 0 && palette_count == 0)) {
      return;
    }
    if (cursor + palette_count > msg_end) {
      return;
    }
    const uint16_t row_size = gbitmap_format_get_row_size_bytes(width, gformat);
    const uint32_t total = (uint32_t)row_size * height;
    if (total == 0 || total > IMAGING_MAX_BYTES) {
      return;
    }
    s_rx.pixels = kernel_zalloc(total);
    if (!s_rx.pixels) {
      prv_rx_reset();
      return;
    }
    if (max_palette > 0) {
      s_rx.palette = kernel_zalloc(IMAGING_PALETTE_ENTRIES * sizeof(GColor));
      if (!s_rx.palette) {
        prv_rx_reset();
        return;
      }
      for (uint8_t i = 0; i < palette_count; ++i) {
        s_rx.palette[i] = (GColor) { .argb = cursor[i] };
      }
      cursor += palette_count;
    }
    s_rx.active = true;
    s_rx.token = hdr->token;
    s_rx.format = gformat;
    s_rx.width = width;
    s_rx.height = height;
    s_rx.row_size_bytes = row_size;
    s_rx.total_bytes = total;
    s_rx.received_bytes = 0;
  }

  if (!s_rx.active || s_rx.token != hdr->token) {
    // ponytail: one reassembly slot, so two consumers fetching at once costs one of them a retry.
    // Add a per-token slot array if that ever matters.
    prv_rx_reset();
    return;
  }

  // Reliable, ordered transport: require contiguous in-order chunks with an exact declared length.
  const size_t avail = (cursor <= msg_end) ? (size_t)(msg_end - cursor) : 0;
  if (hdr->offset != s_rx.received_bytes || hdr->chunk_len != avail ||
      (uint32_t)hdr->offset + hdr->chunk_len > s_rx.total_bytes) {
    prv_rx_reset();
    return;
  }
  memcpy(s_rx.pixels + hdr->offset, cursor, hdr->chunk_len);
  s_rx.received_bytes += hdr->chunk_len;

  if (hdr->flags & ImagingResponseFlagLast) {
    if (s_rx.received_bytes != s_rx.total_bytes) {
      prv_rx_reset();
      return;
    }
    GBitmap *bmp = kernel_zalloc(sizeof(GBitmap));
    if (!bmp) {
      prv_rx_reset();
      return;
    }
    bmp->addr = s_rx.pixels;
    bmp->row_size_bytes = s_rx.row_size_bytes;
    bmp->info.format = s_rx.format;
    bmp->info.version = GBITMAP_VERSION_CURRENT;
    bmp->bounds = (GRect) { { 0, 0 }, { s_rx.width, s_rx.height } };
    bmp->palette = s_rx.palette;
    // Ownership of the pixel and palette buffers moves into the bitmap.
    const uint8_t token = s_rx.token;
    s_rx.pixels = NULL;
    s_rx.palette = NULL;
    prv_rx_reset();
    prv_deliver(token, type, bmp);
  }
}

void imaging_handle_comm_session_event(const PebbleCommSessionEvent *event) {
  if (!event->is_system || event->is_open) {
    return;
  }
  // The system session closed: free any partially received image so an aborted transfer doesn't
  // hold its pixel buffer until the next one starts. This runs on KernelMain, the same task as
  // the endpoint receiver, so touching s_rx is safe. Also clear the unsupported-type latch here
  // rather than relying solely on the pointer comparison in prv_type_latched_unsupported: a
  // future session could be allocated at the address of the freed one.
  prv_rx_reset();
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  s_latched_session = NULL;
  s_unsupported_types = 0;
  pbl_mutex_unlock(&s_lock);
}
