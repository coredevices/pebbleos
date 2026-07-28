/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include "pbl/util/attributes.h"

//! Generic image-fetch endpoint (0x0035). The watch pulls an image from the phone: it sends an
//! ImageRequest naming what it wants (type, encoding, dimensions and type-specific parameters), and
//! the phone streams back the pixels in one or more ImageResponse chunks. Album art is the first
//! consumer; notification images and others can reuse the same endpoint.

typedef enum {
  // Watch -> Phone
  ImagingCmdIDRequest = 0x01,
  // Phone -> Watch
  ImagingCmdIDResponse = 0x02,

  ImagingCmdIDInvalid = 0xff,
} ImagingCmdID;

//! What the image is for. Determines the type-specific parameters in the request tail.
typedef enum {
  ImagingImageTypeAlbumArt = 0x00,
  // Future: ImagingImageTypeNotification = 0x01, ...
} ImagingImageType;

//! Pixel encoding the watch is asking for (and that the response is packed in).
typedef enum {
  ImagingFormat1Bit = 0x00,        //!< 1-bpp black & white.
  ImagingFormat8BitColor = 0x01,   //!< 8-bpp GColor8.
  ImagingFormat4BitPalette = 0x02, //!< 4-bpp palettized GColor8 (up to 16 colours).
} ImagingFormat;

//! Watch -> Phone. Fixed head, then type-specific parameters.
typedef struct PACKED {
  uint8_t cmd;         //!< ImagingCmdIDRequest.
  uint8_t token;       //!< Opaque; echoed in the response so the watch can match it to a request.
  uint8_t image_type;  //!< ImagingImageType.
  uint8_t format;      //!< ImagingFormat the watch wants.
  uint16_t width;      //!< Desired width in pixels.
  uint16_t height;     //!< Desired height in pixels.
  // Type-specific parameters follow. For ImagingImageTypeAlbumArt:
  //   uint8_t title_len;  char title[title_len];
  //   uint8_t artist_len; char artist[artist_len];
  // The phone returns art for the named track (so it can't race a track change), or NO_IMAGE.
} ImagingRequestHeader;

//! Flags byte in an ImageResponse chunk.
typedef enum {
  ImagingResponseFlagFirst = (1 << 0),   //!< First chunk; the image header precedes the pixels.
  ImagingResponseFlagLast = (1 << 1),    //!< Last chunk of the transfer.
  ImagingResponseFlagNoImage = (1 << 2), //!< Phone has no image; no pixels follow.
} ImagingResponseFlags;

//! Phone -> Watch, chunked. `chunk_len` pixel bytes follow this header (after the image header on
//! the first chunk).
typedef struct PACKED {
  uint8_t cmd;         //!< ImagingCmdIDResponse.
  uint8_t token;       //!< Echo of the request token.
  uint8_t flags;       //!< ImagingResponseFlags bitset.
  uint32_t offset;     //!< Byte offset of this chunk's pixels into the pixel stream.
  uint16_t chunk_len;  //!< Number of pixel bytes in this chunk.
  // First chunk only, before the pixel data:
  //   uint16_t width;
  //   uint16_t height;
  //   uint8_t  format;          // ImagingFormat
  //   uint8_t  palette_count;   // palette formats only: 1..16 (0 for non-palette)
  //   uint8_t  palette[palette_count];  // GColor8 entries
} ImagingResponseHeader;
