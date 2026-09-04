/*
   uPNG -- derived from LodePNG version 20100808

   Copyright (c) 2005-2010 Lode Vandevenne
   Copyright (c) 2010 Sean Middleditch
   Copyright (c) 2013-2014 Matthew Hungerford
   Copyright (c) 2015 by Pebble Inc.

   This software is provided 'as-is', without any express or implied
   warranty. In no event will the authors be held liable for any damages
   arising from the use of this software.

   Permission is granted to anyone to use this software for any purpose,
   including commercial applications, and to alter it and redistribute it
   freely, subject to the following restrictions:

   1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.

   2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.

   3. This notice may not be removed or altered from any source
   distribution.
   */

/* Version history:
   0.0   8 Aug 2010  LodePNG 20100808 release
   1.0  19 Oct 2010  Initial uPNG based on LodePNG 20100808, huffman decoder, SDL/GL viewer
   1.1  11 Dec 2013  Reduced huffman data overhead and moved huffman tables to heap
   1.2  10 Mar 2014  Support non-byte-aligned images (fixes 1,2,4 bit PNG8 support)
   1.3  11 Feb 2015  Add PNG8 alpha_palette support.  Add APNG support (iterative frame decoding)
   1.4  14 Dec 2015  Replace built-in huffman inflate with tinflate (tiny inflate)
 */

#include "upng.h"

#include "kernel/pbl_malloc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "tinflate.h"

// Used to allow direct character buffer access with offsets and byteswap in place
#define MAKE_WORD_PTR(p) MAKE_WORD((p)[0], (p)[1], (p)[2], (p)[3])
#define MAKE_SHORT_PTR(p) (uint16_t)((MAKE_BYTE((p)[0]) << 8) | MAKE_BYTE((p)[1]))

#define FIRST_LENGTH_CODE_INDEX 257
#define LAST_LENGTH_CODE_INDEX 285

/*256 literals, the end code, some length codes, and 2 unused codes */
#define NUM_DEFLATE_CODE_SYMBOLS 288
/*the distance codes have their own symbols, 30 used, 2 unused */
#define NUM_DISTANCE_SYMBOLS 32
/* the code length codes. 0-15: code lengths, 16: copy previous 3-6 times,
 * 17: 3-10 zeros, 18: 11-138 zeros */
#define NUM_CODE_LENGTH_CODES 19
/* largest number of symbols used by any tree type */
#define MAX_SYMBOLS 288

#define DEFLATE_CODE_BITLEN 15
#define DISTANCE_BITLEN 15
#define CODE_LENGTH_BITLEN 7
#define MAX_BIT_LENGTH 15 // bug? 15 /* largest bitlen used by any tree type */

#define DEFLATE_CODE_BUFFER_SIZE (NUM_DEFLATE_CODE_SYMBOLS * 2)
#define DISTANCE_BUFFER_SIZE (NUM_DISTANCE_SYMBOLS * 2)
#define CODE_LENGTH_BUFFER_SIZE (NUM_DISTANCE_SYMBOLS * 2)

#define SET_ERROR(upng, code) do {(upng)->error = (code); (upng)->error_line = __LINE__;} while (0)

#define upng_chunk_data_length(chunk) MAKE_WORD_PTR(chunk)
#define upng_chunk_type(chunk) MAKE_WORD_PTR((chunk) + 4)
#define upng_chunk_data(chunk) ((chunk) + 8)

#define upng_chunk_type_critical(chunk_type) (((chunk_type) & 0x20000000) == 0)

typedef enum upng_state {
  UPNG_ERROR   = -1,
  UPNG_DECODED = 0,
  UPNG_LOADED  = 1, // Global data loaded (Palette) (APNG control data)
  UPNG_HEADER  = 2,
  UPNG_NEW     = 3
} upng_state;

typedef enum upng_color {
  UPNG_LUM  = 0,
  UPNG_RGB  = 2,
  UPNG_PLT  = 3,
  UPNG_LUMA = 4,
  UPNG_RGBA = 6
} upng_color;

typedef struct upng_source {
  const uint8_t* buffer;
  uint32_t   size;
  char     owning;
} upng_source;

struct upng_t {
  uint32_t  width;
  uint32_t  height;

  rgb *palette;
  uint16_t palette_entries;

  uint8_t *alpha_palette;
  uint16_t alpha_palette_entries;

  upng_color  color_type;
  uint32_t  color_depth;
  upng_format  format;

  const uint8_t* cursor; // data cursor for parsing linearly
  uint8_t* buffer;
  uint32_t size;

  // APNG information for image at current frame
  bool is_apng;
  apng_fctl* apng_frame_control;
  uint32_t apng_num_frames;
  uint32_t apng_num_plays;  // 0 indicates infinite looping
  uint32_t apng_duration_ms;

  upng_error  error;
  uint32_t  error_line;

  upng_state  state;
  upng_source  source;
};

static uint8_t read_bit(uint32_t *bitpointer, const uint8_t *bitstream) {
  uint8_t result =
    (uint8_t)((bitstream[(*bitpointer) >> 3] >> ((*bitpointer) & 0x7)) & 1);
  (*bitpointer)++;
  return result;
}

static void inflate_uncompressed(upng_t* upng, uint8_t* out, uint32_t outsize,
    const uint8_t *in, uint32_t *bp, uint32_t *pos, uint32_t inlength) {
  uint32_t p;
  uint16_t len, nlen, n;

  /* go to first boundary of byte */
  while (((*bp) & 0x7) != 0) {
    (*bp)++;
  }
  p = (*bp) / 8;  /*byte position */

  /* read len (2 bytes) and nlen (2 bytes) */
  if (p >= inlength - 4) {
    SET_ERROR(upng, UPNG_EMALFORMED);
    return;
  }

  len = in[p] + 256 * in[p + 1];
  p += 2;
  nlen = in[p] + 256 * in[p + 1];
  p += 2;

  /* check if 16-bit nlen is really the one's complement of len */
  if (len + nlen != UINT16_MAX) {
    SET_ERROR(upng, UPNG_EMALFORMED);
    return;
  }

  if ((*pos) + len > outsize) {
    SET_ERROR(upng, UPNG_EMALFORMED);
    return;
  }

  /* read the literal data: len bytes are now stored in the out buffer */
  if (p + len > inlength) {
    SET_ERROR(upng, UPNG_EMALFORMED);
    return;
  }

  for (n = 0; n < len; n++) {
    out[(*pos)++] = in[p++];
  }

  (*bp) = p * 8;
}

/*inflate the deflated data (cfr. deflate spec); return value is the error*/
static upng_error uz_inflate_data(upng_t* upng, uint8_t* out, uint32_t outsize,
    const uint8_t *in, uint32_t insize, uint32_t inpos) {
  /*bit pointer in the "in" data, current byte is bp >> 3,
   * current bit is bp & 0x7 (from lsb to msb of the byte) */
  uint32_t bp = 0;
  uint32_t pos = 0; /*byte position in the out buffer */

  uint16_t done = 0;

  while (done == 0) {
    uint16_t btype;

    /* ensure next bit doesn't point past the end of the buffer */
    if ((bp >> 3) >= insize) {
      SET_ERROR(upng, UPNG_EMALFORMED);
      return upng->error;
    }

    /* read block control bits */
    done = read_bit(&bp, &in[inpos]);
    btype = read_bit(&bp, &in[inpos]) | (read_bit(&bp, &in[inpos]) << 1);

    /* process control type appropriateyly */
    if (btype == 3) {
      SET_ERROR(upng, UPNG_EMALFORMED);
      return upng->error;
    } else if (btype == 0) {
      inflate_uncompressed(upng, out, outsize, &in[inpos], &bp, &pos, insize); /*no compression */
    } else {
      /*compression, btype 01 or 10 */
      int tinflate_status = tinflate_uncompress(out, (unsigned int *)&outsize, &in[inpos], insize);
      if (tinflate_status < 0) {
        SET_ERROR(upng, UPNG_EMALFORMED);
        return upng->error;
      }
      done = 1;  // No need to increment bp, tinflate handles end-of-bounds marker
    }

    /* stop if an error has occured */
    if (upng->error != UPNG_EOK) {
      return upng->error;
    }
  }

  return upng->error;
}

static upng_error uz_inflate(upng_t* upng, uint8_t *out, uint32_t outsize,
    const uint8_t *in, uint32_t insize) {
  /* we require two bytes for the zlib data header */
  if (insize < 2) {
    SET_ERROR(upng, UPNG_EMALFORMED);
    return upng->error;
  }

  /* 256 * in[0] + in[1] must be a multiple of 31,
   * the FCHECK value is supposed to be made that way */
  if ((in[0] * 256 + in[1]) % 31 != 0) {
    SET_ERROR(upng, UPNG_EMALFORMED);
    return upng->error;
  }

  /*error: only compression method 8: inflate with sliding window of 32k
   * is supported by the PNG spec */
  if ((in[0] & 15) != 8 || ((in[0] >> 4) & 15) > 7) {
    SET_ERROR(upng, UPNG_EMALFORMED);
    return upng->error;
  }

  /* the specification of PNG says about the zlib stream:
   * "The additional flags shall not specify a preset dictionary." */
  if (((in[1] >> 5) & 1) != 0) {
    SET_ERROR(upng, UPNG_EMALFORMED);
    return upng->error;
  }

  /* create output buffer */
  uz_inflate_data(upng, out, outsize, in, insize, 2);

  return upng->error;
}

/*Paeth predicter, used by PNG filter type 4*/
static int32_t paeth_predictor(int32_t a, int32_t b, int32_t c) {
  int32_t p = a + b - c;
  int32_t pa = p > a ? p - a : a - p;
  int32_t pb = p > b ? p - b : b - p;
  int32_t pc = p > c ? p - c : c - p;

  if (pa <= pb && pa <= pc)
    return a;
  else if (pb <= pc)
    return b;
  else
    return c;
}

static void unfilter_scanline(upng_t* upng, uint8_t *recon, const uint8_t *scanline,
    const uint8_t *precon, uint32_t bytewidth, uint8_t filterType,
    uint32_t length) {
  /*
     For PNG filter method 0
     unfilter a PNG image scanline by scanline.
     When the pixels are smaller than 1 byte, the filter works byte per byte (bytewidth = 1)
     precon is the previous unfiltered scanline, recon the result, scanline the current one
     the incoming scanlines do NOT include the filtertype byte,
     that one is given in the parameter filterType instead
     recon and scanline MAY be the same memory address! precon must be disjoint.
     */

  uint32_t i;
  switch (filterType) {
    case 0:
      for (i = 0; i < length; i++)
        recon[i] = scanline[i];
      break;
    case 1:
      for (i = 0; i < bytewidth; i++)
        recon[i] = scanline[i];
      for (i = bytewidth; i < length; i++)
        recon[i] = scanline[i] + recon[i - bytewidth];
      break;
    case 2:
      if (precon)
        for (i = 0; i < length; i++)
          recon[i] = scanline[i] + precon[i];
      else
        for (i = 0; i < length; i++)
          recon[i] = scanline[i];
      break;
    case 3:
      if (precon) {
        for (i = 0; i < bytewidth; i++)
          recon[i] = scanline[i] + precon[i] / 2;
        for (i = bytewidth; i < length; i++)
          recon[i] = scanline[i] + ((recon[i - bytewidth] + precon[i]) / 2);
      } else {
        for (i = 0; i < bytewidth; i++)
          recon[i] = scanline[i];
        for (i = bytewidth; i < length; i++)
          recon[i] = scanline[i] + recon[i - bytewidth] / 2;
      }
      break;
    case 4:
      if (precon) {
        for (i = 0; i < bytewidth; i++)
          recon[i] = (uint8_t)(scanline[i] + paeth_predictor(0, precon[i], 0));
        for (i = bytewidth; i < length; i++)
          recon[i] = (uint8_t)(scanline[i] + paeth_predictor(recon[i - bytewidth],
                precon[i], precon[i - bytewidth]));
      } else {
        for (i = 0; i < bytewidth; i++)
          recon[i] = scanline[i];
        for (i = bytewidth; i < length; i++)
          recon[i] = (uint8_t)(scanline[i] + paeth_predictor(recon[i - bytewidth], 0, 0));
      }
      break;
    default:
      SET_ERROR(upng, UPNG_EMALFORMED);
      break;
  }
}

static void unfilter(upng_t* upng, uint8_t *out, const uint8_t *in,
    uint32_t w, uint32_t h, uint32_t bpp) {
  /*
     For PNG filter method 0
     this function unfilters a single image
     (e.g. without interlacing this is called once, with Adam7 it's called 7 times)
     out must have enough bytes allocated already,
     in must have the scanlines + 1 filtertype byte per scanline
     w and h are image dimensions or dimensions of reduced image, bpp is bpp per pixel
     in and out are allowed to be the same memory address!
     */

  uint32_t y;
  uint8_t *prevline = 0;

  /*bytewidth is used for filtering, is 1 when bpp < 8, number of bytes per pixel otherwise */
  uint32_t bytewidth = (bpp + 7) / 8;
  uint32_t linebytes = (w * bpp + 7) / 8;

  for (y = 0; y < h; y++) {
    uint32_t outindex = linebytes * y;
    uint32_t inindex = (1 + linebytes) * y; /*the extra filterbyte added to each row */
    uint8_t filterType = in[inindex];

    unfilter_scanline(upng, &out[outindex], &in[inindex + 1], prevline, bytewidth, filterType,
        linebytes);
    if (upng->error != UPNG_EOK) {
      return;
    }

    prevline = &out[outindex];
  }
}

static void remove_padding_bits(uint8_t *out, const uint8_t *in,
    uint32_t olinebits, uint32_t ilinebits, uint32_t h) {
  /*
     After filtering there are still padding bpp if scanlines have non multiple of 8 bit amounts.
     They need to be removed (except at last scanline of (Adam7-reduced) image)
     before working with pure image buffers for the Adam7 code,
     the color convert code and the output to the user.
     in and out are allowed to be the same buffer, in may also be higher but still overlapping;
     in must have >= ilinebits*h bpp, out must have >= olinebits*h bpp,
     olinebits must be <= ilinebits
     also used to move bpp after earlier such operations happened,
     e.g. in a sequence of reduced images from Adam7
     only useful if (ilinebits - olinebits) is a value in the range 1..7
     */
  uint32_t y;
  uint32_t diff = ilinebits - olinebits;
  uint32_t obp = 0, ibp = 0; /*bit pointers */
  for (y = 0; y < h; y++) {
    uint32_t x;
    for (x = 0; x < olinebits; x++) {
      uint8_t bit = (uint8_t)((in[(ibp) >> 3] >> (7 - ((ibp) & 0x7))) & 1);
      ibp++;

      if (bit == 0)
        out[(obp) >> 3] &= (uint8_t)(~(1 << (7 - ((obp) & 0x7))));
      else
        out[(obp) >> 3] |= (1 << (7 - ((obp) & 0x7)));
      ++obp;
    }
    ibp += diff;
  }
}

/*out must be buffer big enough to contain full image,
 * and in must contain the full decompressed data from the IDAT chunks*/
static void post_process_scanlines(upng_t* upng, uint8_t *out, uint8_t *in,
    uint32_t bpp, uint32_t w, uint32_t h) {
  if (bpp == 0) {
    SET_ERROR(upng, UPNG_EMALFORMED);
    return;
  }

  if (bpp < 8 && w * bpp != ((w * bpp + 7) / 8) * 8) {
    unfilter(upng, in, in, w, h, bpp);
    if (upng->error != UPNG_EOK) {
      return;
    }
    // remove_padding_bits(out, in, w * bpp, ((w * bpp + 7) / 8) * 8, h);
    // fix for non-byte-aligned images
    uint32_t aligned_width = ((w * bpp + 7) / 8) * 8;
    remove_padding_bits(in, in, aligned_width, aligned_width, h);
  } else {
    /*we can immediatly filter into the out buffer, no other steps needed */
    unfilter(upng, in, in, w, h, bpp);
  }
}

static upng_format determine_format(upng_t* upng) {
  switch (upng->color_type) {
    case UPNG_PLT:
      switch (upng->color_depth) {
        case 1:
          return UPNG_INDEXED1;
        case 2:
          return UPNG_INDEXED2;
        case 4:
          return UPNG_INDEXED4;
        case 8:
          return UPNG_INDEXED8;
        default:
          return UPNG_BADFORMAT;
      }
    case UPNG_LUM:
      switch (upng->color_depth) {
        case 1:
          return UPNG_LUMINANCE1;
        case 2:
          return UPNG_LUMINANCE2;
        case 4:
          return UPNG_LUMINANCE4;
        case 8:
          return UPNG_LUMINANCE8;
        default:
          return UPNG_BADFORMAT;
      }
    case UPNG_RGB:
      switch (upng->color_depth) {
        case 8:
          return UPNG_RGB8;
        case 16:
          return UPNG_RGB16;
        default:
          return UPNG_BADFORMAT;
      }
    case UPNG_LUMA:
      switch (upng->color_depth) {
        case 1:
          return UPNG_LUMINANCE_ALPHA1;
        case 2:
          return UPNG_LUMINANCE_ALPHA2;
        case 4:
          return UPNG_LUMINANCE_ALPHA4;
        case 8:
          return UPNG_LUMINANCE_ALPHA8;
        default:
          return UPNG_BADFORMAT;
      }
    case UPNG_RGBA:
      switch (upng->color_depth) {
        case 8:
          return UPNG_RGBA8;
        case 16:
          return UPNG_RGBA16;
        default:
          return UPNG_BADFORMAT;
      }
    default:
      return UPNG_BADFORMAT;
  }
}

static void upng_free_source(upng_t* upng) {
  if (!upng) {
    return;
  }

  if (upng->source.owning != 0) {
    task_free((void*)upng->source.buffer);
  }

  upng->source.buffer = NULL;
  upng->source.size = 0;
  upng->source.owning = 0;
}

/*read the information from the header and store it in the upng_Info. return value is error*/
upng_error upng_header(upng_t* upng) {
  /* if we have an error state, bail now */
  if (upng->error != UPNG_EOK) {
    return upng->error;
  }

  /* if the state is not NEW (meaning we are ready to parse the header), stop now */
  if (upng->state != UPNG_NEW) {
    return upng->error;
  }

  // verify minimum length for a valid PNG file
  if (upng->source.size < PNG_HEADER_SIZE) {
    SET_ERROR(upng, UPNG_ENOTPNG);
    return upng->error;
  }

  /* check that PNG header matches expected value */
  if (MAKE_WORD_PTR(upng->source.buffer) != PNG_SIGNATURE) {
    SET_ERROR(upng, UPNG_ENOTPNG);
    return upng->error;
  }

  /* check that the first chunk is the IHDR chunk */
  if (MAKE_WORD_PTR(upng->source.buffer + CHUNK_META_SIZE) != CHUNK_IHDR) {
    SET_ERROR(upng, UPNG_EMALFORMED);
    return upng->error;
  }

  /* read the values given in the header */
  upng->width = MAKE_WORD_PTR(upng->source.buffer + 16);
  upng->height = MAKE_WORD_PTR(upng->source.buffer + 20);
  upng->color_depth = upng->source.buffer[24];
  upng->color_type = (upng_color)upng->source.buffer[25];

  /* determine our color format */
  upng->format = determine_format(upng);
  if (upng->format == UPNG_BADFORMAT) {
    SET_ERROR(upng, UPNG_EUNFORMAT);
    return upng->error;
  }

  /* check that the compression method (byte 27) is 0 (only allowed value in spec) */
  if (upng->source.buffer[26] != 0) {
    SET_ERROR(upng, UPNG_EMALFORMED);
    return upng->error;
  }

  /* check that the compression method (byte 27) is 0 (only allowed value in spec) */
  if (upng->source.buffer[27] != 0) {
    SET_ERROR(upng, UPNG_EMALFORMED);
    return upng->error;
  }

  /* check that the compression method (byte 27) is 0
   * (spec allows 1, but uPNG does not support it) */
  if (upng->source.buffer[28] != 0) {
    SET_ERROR(upng, UPNG_EUNINTERLACED);
    return upng->error;
  }

  upng->state = UPNG_HEADER;
  return upng->error;
}


upng_error upng_decode_metadata(upng_t* upng) {
  /* if we have an error state, bail now */
  if (upng->error != UPNG_EOK) {
    return upng->error;
  }

  /* parse the main header, if necessary */
  if (upng->state != UPNG_HEADER) {
    upng_header(upng);
    if ((upng->error != UPNG_EOK) || (upng->state != UPNG_HEADER)) {
      return upng->error;
    }
  }

  /* first byte of the first chunk after the header */
  upng->cursor = upng->source.buffer + PNG_HEADER_SIZE;

  /* scan through the chunks, finding the size of all IDAT chunks, and also
   * verify general well-formed-ness */
  while (upng->cursor < upng->source.buffer + upng->source.size) {
    uint32_t chunk_type = upng_chunk_type(upng->cursor);
    uint32_t data_length = upng_chunk_data_length(upng->cursor);
    const uint8_t *data = upng_chunk_data(upng->cursor);

    /* make sure chunk header+paylaod is not larger than the total compressed */
    if ((uint32_t)(upng->cursor - upng->source.buffer + data_length + CHUNK_META_SIZE) >
        upng->source.size) {
      SET_ERROR(upng, UPNG_EMALFORMED);
      return upng->error;
    }

    /* parse chunks */
    switch (chunk_type) {
      case CHUNK_PLTE:
        task_free(upng->palette);
        upng->palette_entries = data_length / 3; // 3 bytes per color entry
        upng->palette = task_malloc(data_length);
        if (upng->palette == NULL) {
          SET_ERROR(upng, UPNG_ENOMEM);
          return upng->error;
        }
        memcpy(upng->palette, data, data_length);
        break;
      case CHUNK_TRNS:
        task_free(upng->alpha_palette);
        upng->alpha_palette_entries = data_length; // 1 byte per color entry
        // protect against tools that create a tRNS chunk with 0 entries
        if (data_length) {
          upng->alpha_palette = task_malloc(data_length);
          if (upng->alpha_palette == NULL) {
            SET_ERROR(upng, UPNG_ENOMEM);
            return upng->error;
          }
          memcpy(upng->alpha_palette, data, data_length);
        }
        break;
      case CHUNK_FCTL:
        if (upng->apng_frame_control == NULL) {
          upng->apng_frame_control = task_malloc(sizeof(apng_fctl));
          if (upng->apng_frame_control == NULL) {
            SET_ERROR(upng, UPNG_ENOMEM);
            return upng->error;
          }
        }

        /* FCTL - Frame Control CHUNK
         byte
         0    sequence_number (uint32_t) Sequence number of the animation chunk, starting from 0
         4    width           (uint32_t) Width of the following frame
         8    height          (uint32_t) Height of the following frame
         12    x_offset       (uint32_t) X position at which to render the following frame
         16    y_offset       (uint32_t) Y position at which to render the following frame
         20    delay_num      (uint16_t) Frame delay fraction numerator
         22    delay_den      (uint16_t) Frame delay fraction denominator
         24    dispose_op     (uint8_t)  Frame area disposal to be done after rendering this frame
         25    blend_op       (uint8_t)  Type of frame area rendering for this frame
         */

        upng->apng_frame_control->sequence_number = MAKE_WORD_PTR(data);
        upng->apng_frame_control->width = MAKE_WORD_PTR(data + 4);
        upng->apng_frame_control->height = MAKE_WORD_PTR(data + 8);
        upng->apng_frame_control->x_offset = MAKE_WORD_PTR(data + 12);
        upng->apng_frame_control->y_offset = MAKE_WORD_PTR(data + 16);
        upng->apng_frame_control->delay_num = MAKE_SHORT_PTR(data + 20);
        upng->apng_frame_control->delay_den = MAKE_SHORT_PTR(data + 22);
        upng->apng_frame_control->dispose_op = *(data + 24);
        upng->apng_frame_control->blend_op = *(data + 25);
        break;
      case CHUNK_ACTL:
        upng->is_apng = true;
        upng->apng_num_frames = MAKE_WORD_PTR(data);
        upng->apng_num_plays = MAKE_WORD_PTR(data + 4);
        break;
      case CHUNK_IDAT:
        // Stop at these chunks and leave for another stage
        upng->state = UPNG_LOADED;
        return upng->error;
        break;
      case CHUNK_IEND:
        SET_ERROR(upng, UPNG_EMALFORMED);
        upng->state = UPNG_ERROR;
        return upng->error;
        break;
      default:
        if (upng_chunk_type_critical(chunk_type)) {
          SET_ERROR(upng, UPNG_EUNSUPPORTED);
          upng->cursor += data_length + CHUNK_META_SIZE; // forward cursor to next chunk
          return upng->error;
        }
        break;
    }
    upng->cursor += data_length + CHUNK_META_SIZE; // forward cursor to next chunk
  }

  upng->state = UPNG_LOADED;
  return upng->error;
}

/*read a PNG, the result will be in the same color type as the PNG (hence "generic")*/
upng_error upng_decode_image(upng_t* upng) {
  uint8_t* compressed = NULL;
  uint8_t* inflated = NULL;
  uint32_t compressed_size = 0;
  uint32_t inflated_size = 0;
  bool cursor_at_next_frame = false;

  /* if we have an error state, bail now */
  if (upng->error != UPNG_EOK) {
    return upng->error;
  }

  /* parse the main header and additional global data, if necessary */
  if (upng->state != UPNG_LOADED && upng->state != UPNG_DECODED) {
    upng_decode_metadata(upng);
    if (upng->error != UPNG_EOK || upng->state != UPNG_LOADED) {
      return upng->error;
    }
  }


  /* release old result, if any */
  if (upng->buffer) {
    task_free(upng->buffer);
    upng->buffer = NULL;
    upng->size = 0;
  }


  /* scan through the chunks, finding the size of all IDAT chunks, and also
   * verify general well-formed-ness */
  while ((upng->cursor < upng->source.buffer + upng->source.size) && !cursor_at_next_frame) {
    uint32_t chunk_type = upng_chunk_type(upng->cursor);
    uint32_t data_length = upng_chunk_data_length(upng->cursor);
    const uint8_t *data = upng_chunk_data(upng->cursor);

    /* make sure chunk header+payload is not larger than the total compressed */
    if ((uint32_t)(upng->cursor - upng->source.buffer + data_length + CHUNK_META_SIZE) >
        upng->source.size) {
      SET_ERROR(upng, UPNG_EMALFORMED);
      return upng->error;
    }

    /* parse chunks */
    switch (chunk_type) {
      case CHUNK_FCTL:
        if (upng->apng_frame_control == NULL) {
          upng->apng_frame_control = task_malloc(data_length);
          if (upng->apng_frame_control == NULL) {
            SET_ERROR(upng, UPNG_ENOMEM);
            return upng->error;
          }
        }

        /* FCTL - Frame Control CHUNK
         byte
         0    sequence_number (uint32_t) Sequence number of the animation chunk, starting from 0
         4    width           (uint32_t) Width of the following frame
         8    height          (uint32_t) Height of the following frame
         12    x_offset       (uint32_t) X position at which to render the following frame
         16    y_offset       (uint32_t) Y position at which to render the following frame
         20    delay_num      (uint16_t) Frame delay fraction numerator
         22    delay_den      (uint16_t) Frame delay fraction denominator
         24    dispose_op     (uint8_t)  Frame area disposal to be done after rendering this frame
         25    blend_op       (uint8_t)  Type of frame area rendering for this frame
         */

        upng->apng_frame_control->sequence_number = MAKE_WORD_PTR(data);
        upng->apng_frame_control->width = MAKE_WORD_PTR(data + 4);
        upng->apng_frame_control->height = MAKE_WORD_PTR(data + 8);
        upng->apng_frame_control->x_offset = MAKE_WORD_PTR(data + 12);
        upng->apng_frame_control->y_offset = MAKE_WORD_PTR(data + 16);
        upng->apng_frame_control->delay_num = MAKE_SHORT_PTR(data + 20);
        upng->apng_frame_control->delay_den = MAKE_SHORT_PTR(data + 22);
        upng->apng_frame_control->dispose_op = *(data + 24);
        upng->apng_frame_control->blend_op = *(data + 25);
        break;
      case CHUNK_FDAT:
        /* first 4 bytes in fdAT is sequence number, so skip 4 bytes */
        // TODO : fix for multiple consecutive fdAT chunks (PBL-14294)
        compressed = (uint8_t*)(data + 4);
        compressed_size = (data_length - 4);
        cursor_at_next_frame = true; // stop processing chunks at the IDAT/fdAT chunks
        break;
      case CHUNK_IDAT:
        // TODO : fix for multiple consecutive IDAT chunks (PBL-14294)
        compressed = (uint8_t*)(data);
        compressed_size = data_length;
        cursor_at_next_frame = true; // stop processing chunks at the IDAT/fdAT chunks
        break;
      case CHUNK_IEND:
        SET_ERROR(upng, UPNG_EDONE);
        upng->state = UPNG_ERROR; // force future calls to fail
        return upng->error;
        break;
      default:
        if (upng_chunk_type_critical(chunk_type)) {
          SET_ERROR(upng, UPNG_EUNSUPPORTED);
          upng->cursor += data_length + CHUNK_META_SIZE; // forward cursor to next chunk
          return upng->error;
        }
        break;
    }
    upng->cursor += data_length + CHUNK_META_SIZE; // forward cursor to next chunk
  }


  uint32_t width = upng->width;
  uint32_t height = upng->height;
  if (upng->apng_frame_control) {
    width = upng->apng_frame_control->width;
    height = upng->apng_frame_control->height;
  }

  /* allocate space to store inflated (but still filtered) data */
  int32_t width_aligned_bytes = (width * upng_get_bpp(upng) + 7) / 8;
  inflated_size = (width_aligned_bytes * height) + height; // pad byte
  inflated = (uint8_t*)task_malloc(inflated_size);
  if (inflated == NULL) {
    SET_ERROR(upng, UPNG_ENOMEM);
    return upng->error;
  }

  /* decompress image data */
  if (uz_inflate(upng, inflated, inflated_size, compressed, compressed_size) != UPNG_EOK) {
    task_free(inflated);
    return upng->error;
  }

  /* unfilter scanlines */
  post_process_scanlines(upng, inflated, inflated, upng_get_bpp(upng), width, height);
  upng->buffer = inflated;
  upng->size = inflated_size;

  if (upng->error != UPNG_EOK) {
    task_free(upng->buffer);
    upng->buffer = NULL;
    upng->size = 0;
  } else {
    upng->state = UPNG_DECODED;
  }

  return upng->error;
}

upng_t* upng_create(void) {
  upng_t* upng = (upng_t*)task_malloc(sizeof(upng_t));
  if (upng == NULL) {
    return NULL;
  }

  memset(upng, 0, sizeof(upng_t));

  upng->color_type = UPNG_RGBA;
  upng->color_depth = 8;
  upng->format = UPNG_RGBA8;

  upng->state = UPNG_NEW;

  return upng;
}


void upng_load_bytes(upng_t *upng, const uint8_t *buffer, uint32_t size) {
  upng->cursor = buffer;
  upng->source.buffer = buffer;
  upng->source.size = size;
  upng->source.owning = 0;
}


void upng_destroy(upng_t* upng, bool free_image_buffer) {
  if (!upng) {
    return;
  }

  /* deallocate image buffer */
  if (free_image_buffer) {
    task_free(upng->buffer);
  }

  /* deallocate palette buffer */
  task_free(upng->palette);

  /* deallocate alpha_palette buffer */
  task_free(upng->alpha_palette);
  
  /* deallocate apng_frame_control struct */
  task_free(upng->apng_frame_control);

  /* deallocate source buffer */
  upng_free_source(upng);

  /* deallocate struct itself */
  task_free(upng);
}

upng_error upng_get_error(const upng_t* upng) {
  return upng->error;
}

uint32_t upng_get_error_line(const upng_t* upng) {
  return upng->error_line;
}

uint32_t upng_get_width(const upng_t* upng) {
  return upng->width;
}

uint32_t upng_get_height(const upng_t* upng) {
  return upng->height;
}

uint16_t upng_get_palette(const upng_t* upng, rgb **palette) {
  if (palette) {
    *palette = upng->palette;
  }
  return upng->palette_entries;
}

uint16_t upng_get_alpha_palette(const upng_t* upng, uint8_t **alpha_palette) {
  if (alpha_palette) {
    *alpha_palette = upng->alpha_palette;
  }
  return upng->alpha_palette_entries;
}

uint32_t upng_get_bpp(const upng_t* upng) {
  return upng_get_bitdepth(upng) * upng_get_components(upng);
}

uint32_t upng_get_components(const upng_t* upng) {
  switch (upng->color_type) {
    case UPNG_PLT:
      return 1;
    case UPNG_LUM:
      return 1;
    case UPNG_RGB:
      return 3;
    case UPNG_LUMA:
      return 2;
    case UPNG_RGBA:
      return 4;
    default:
      return 0;
  }
}

uint32_t upng_get_bitdepth(const upng_t* upng) {
  return upng->color_depth;
}

uint32_t upng_get_pixelsize(const upng_t* upng) {
  return (upng_get_bitdepth(upng) * upng_get_components(upng));
}

upng_format upng_get_format(const upng_t* upng) {
  return upng->format;
}

const uint8_t* upng_get_buffer(const upng_t* upng) {
  return upng->buffer;
}

uint32_t upng_get_size(const upng_t* upng) {
  return upng->size;
}

// returns if the png is an apng after the upng_load() function
bool upng_is_apng(const upng_t* upng) {
  return upng->is_apng;
}

// retuns the apng num_frames
uint32_t upng_apng_num_frames(const upng_t* upng) {
  uint32_t num_frames = 1;  //default to 1 frame for png images used as apng
  if (upng->is_apng) {
    num_frames = upng->apng_num_frames;
  }
  return num_frames;
}

// retuns the apng num_plays
uint32_t upng_apng_num_plays(const upng_t* upng) {
  uint32_t num_plays = 1;  // default to 1 play for png images used as apng
  if (upng->is_apng) {
    num_plays = upng->apng_num_plays;
  }
  return num_plays;
}

// Pass in a apng_fctl to get the next frames frame control information
bool upng_get_apng_fctl(const upng_t* upng, apng_fctl *apng_frame_control) {
  bool retval = false;
  if (upng->is_apng && apng_frame_control != NULL) {
    *apng_frame_control = *upng->apng_frame_control;
    retval = true;
  }
  return retval;
}
