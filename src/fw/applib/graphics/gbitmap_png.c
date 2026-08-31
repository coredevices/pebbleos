/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "gbitmap_png.h"

#include "applib/app_logging.h"
#include "applib/applib_malloc.auto.h"
#include "syscall/syscall.h"
#include "util/net.h"

#define PNG_DECODE_ERROR "PNG decoding failed"
#define PNG_MEMORY_ERROR "PNG memory allocation failed"
#define PNG_FORMAT_ERROR "Unsupported PNG format, only PNG8 is supported!"
#define PNG_LOAD_ERROR "Failed to load PNG"

static GBitmapFormat prv_get_format_for_bpp(uint8_t bits_per_pixel) {
  if (bits_per_pixel == 1)  return GBitmapFormat1BitPalette;
  if (bits_per_pixel == 2)  return GBitmapFormat2BitPalette;
  if (bits_per_pixel == 4)  return GBitmapFormat4BitPalette;
  return GBitmapFormat8Bit;
}

bool gbitmap_png_data_is_png(const uint8_t *data, size_t data_size) {
  if (data_size >= sizeof(PNG_SIGNATURE)) {
    // PNG files start with [137, 'P', 'N', 'G']
    return (ntohl(*(uint32_t*)data) == PNG_SIGNATURE);
  }
  return false;
}

// ! Distance from current resource cursor to next IDAT/fdAT chunk including that chunks data
int32_t png_seek_chunk_in_resource(uint32_t resource_id, uint32_t offset,
                                   bool seek_framedata, bool *found_actl) {
  ResAppNum app_num = sys_get_current_resource_num();
  return png_seek_chunk_in_resource_system(app_num, resource_id, offset, seek_framedata,
                                           found_actl);
}

int32_t png_seek_chunk_in_resource_system(ResAppNum app_num, uint32_t resource_id, uint32_t offset,
                                          bool seek_framedata, bool *found_actl) {
  uint32_t current_offset = offset;
  bool actl_chunk_found = false;  // ACTL chunk indicates PNG is an APNG

  struct png_chunk_marker {
    uint32_t length;
    uint32_t chunk_type;
  } marker;

  // we are assuming the current_offset is always left at the start of the next chunk
  // for alignment purposes
  size_t max_size = sys_resource_size(app_num, resource_id);

  while (current_offset + sizeof(marker) < max_size) {
    if (sizeof(marker) != sys_resource_load_range(app_num, resource_id, current_offset,
                            (uint8_t*)&marker, sizeof(marker))) {
      return -1;
    }

    // Need to byte swap it
    marker.length = ntohl(marker.length);
    marker.chunk_type = ntohl(marker.chunk_type);

    if (marker.chunk_type == CHUNK_ACTL) {
      actl_chunk_found = true;
    }

    if (seek_framedata) {
      if (marker.chunk_type == CHUNK_FDAT || marker.chunk_type == CHUNK_IDAT) {
        if (found_actl) {
          *found_actl = actl_chunk_found;
        }
        // current distance + data_length + chunk_parts
        return (current_offset - offset + marker.length + CHUNK_META_SIZE);
      }
    } else {  // Seeking for data up to but not including FCTL or IDAT chunk (ie. image metadata)
      if (marker.chunk_type == CHUNK_IDAT || marker.chunk_type == CHUNK_FCTL) {
        if (found_actl) {
          *found_actl = actl_chunk_found;
        }
        // current distance to the beginning of this chunk
        return (current_offset - offset);
      }
    }
    current_offset += CHUNK_META_SIZE + marker.length;
  }
  return -1; // Error
}

GBitmap* gbitmap_create_from_png_data(const uint8_t *png_data, size_t png_data_size) {
  GBitmap *bitmap = applib_type_malloc(GBitmap);
  if (bitmap) {
    memset(bitmap, 0, sizeof(GBitmap));
    gbitmap_init_with_png_data(bitmap, png_data, png_data_size);
  }
  return bitmap;
}

bool gbitmap_init_with_png_data(GBitmap *bitmap, const uint8_t *data, size_t data_size) {
  GColor8 *palette = NULL;
  bool retval = false;

  upng_t *upng = upng_create();
  if (!upng) {
    goto cleanup;
  }
  upng_load_bytes(upng, data, data_size);
  upng_error upng_state = upng_decode_image(upng);
  if (upng_state != UPNG_EOK) {
    APP_LOG(APP_LOG_LEVEL_ERROR, (upng_state == UPNG_ENOMEM) ? PNG_MEMORY_ERROR : PNG_DECODE_ERROR);
    goto cleanup;
  }

  // Use UPNG to decode image and get data
  uint32_t width = upng_get_width(upng);
  uint32_t height = upng_get_height(upng);
  uint8_t *upng_buffer = (uint8_t*)upng_get_buffer(upng);
  uint32_t bpp = upng_get_bpp(upng);
  uint16_t palette_size = 0;

  if (!gbitmap_png_is_format_supported(upng)) {
    APP_LOG(APP_LOG_LEVEL_ERROR, PNG_FORMAT_ERROR);
    goto cleanup;
  }

  // Create a color palette in GColor8 format from RGB24 + ALPHA8 PNG Palettes (or Grayscale)
  palette_size = gbitmap_png_load_palette(upng, &palette);
  if (palette_size == 0) {
    goto cleanup;
  }

  // Get the GBitmap format based on the bit depth of the raw data
  GBitmapFormat format = prv_get_format_for_bpp(bpp);

  // Convert 8-bit palettized PNGs to raw ARGB color images in-place
  // as we don't support palettized bitdepths above 4
  if (format == GBitmapFormat8Bit) {
    for (uint32_t i = 0; i < width * height; i++) {
      upng_buffer[i] = palette[upng_buffer[i]].argb;  // De-palettize the image data
    }
    applib_free(palette);  // Free the palette to avoid storing it as part of GBitmap
    palette = NULL;
  }

  // Set the image or pixel data
  gbitmap_set_data(bitmap, upng_buffer, format,
      gbitmap_format_get_row_size_bytes(width, format), true);
  gbitmap_set_bounds(bitmap, (GRect){.origin = {0, 0}, .size = {width, height}});
  bitmap->info.version = GBITMAP_VERSION_CURRENT;

  if (palette) {
    gbitmap_set_palette(bitmap, palette, true);
  }

  retval = true;

cleanup:
  if (!retval) {
    // bitmap init failed, free palette
    APP_LOG(APP_LOG_LEVEL_ERROR, PNG_LOAD_ERROR);
    applib_free(palette);
  }

  // we are keeping the image data to avoid copying it
  upng_destroy(upng, !retval);
  return retval;
}

static uint16_t prv_gbitmap_png_create_palette_for_grayscale(upng_t *upng, GColor8 **palette_out) {
  uint16_t palette_entries = 0;
  uint32_t bpp = upng_get_bpp(upng);
  // Convert Luminance format from Grayscale to palette
  // Pebble only has 4 grayscale shades + 1 transparent value, max bpp == 4
  if (bpp > 4) {
    return 0;
  }

  int32_t transparent_gray = gbitmap_png_get_transparent_gray_value(upng);

  // Palette will be size required to hold count of shades of gray
  palette_entries = 0x1 << bpp;
  GColor8 *palette = (GColor8*)applib_malloc(palette_entries * sizeof(GColor8));
  if (!palette) {
    return 0;
  }
  memset(palette, 0, palette_entries * sizeof(GColor8));

  for (uint16_t i = 0; i < palette_entries; i ++) {
    // If the color value matches transparent_gray, color is transparent
    if (transparent_gray >= 0 && i == transparent_gray) {
      palette[i] = GColorClear;
    } else {
      // Only have 2 bits per channel, but attempt to make grayscale 4-bit work
        // which occurs with black, white, gray1, gray2 and a transparent color
        uint8_t luminance = 0;
        if (bpp > 2) {
          luminance = (i >> (bpp - 2));
        } else if (bpp == 2) {
          // For bitdepth 2, use bits directly
          luminance = i;
        } else if (bpp == 1) {
          // For bitdepth 1, need max and minimal values
          luminance = i ? 0x3 : 0x0;
        }
        palette[i] = (GColor8){.a = 0x3, .r = luminance, .g = luminance, .b = luminance};
      }
    }

  // Return the converted palette and number of entries
  *palette_out = palette;
  return palette_entries;
}

static uint16_t prv_gbitmap_png_create_palette_for_color(upng_t *upng, GColor8 **palette_out) {
  if (!palette_out) {
    return 0;
  }

  rgb *rgb_palette = NULL;
  uint16_t palette_entries = upng_get_palette(upng, &rgb_palette);

  uint8_t *alpha_palette = NULL;
  uint16_t alpha_palette_entries = upng_get_alpha_palette(upng, &alpha_palette);

  // To make palette entries consistent with PBI, pad to the bitdepth number of colors
  uint32_t padded_palette_size = (1 << upng_get_bpp(upng));

  GColor8 *palette = (GColor8*)applib_malloc(padded_palette_size * sizeof(GColor8));
  if (palette == NULL) {
    return 0;
  }
  memset(palette, 0, padded_palette_size * sizeof(GColor8));

  // Convert rgb + alpha palette to GColor8 palette
  for (int i = 0; i < palette_entries; i++) {
    (palette)[i] = GColorFromRGBA(
        rgb_palette[i].r, rgb_palette[i].g, rgb_palette[i].b,  // RGB
        (i < alpha_palette_entries) ? alpha_palette[i] : UINT8_MAX); // Conditional A value
  }

  // Return the converted palette and number of entries
  *palette_out = palette;
  return palette_entries;
}

uint16_t gbitmap_png_load_palette(upng_t *upng, GColor8 **palette_out) {
  if (upng) {
    upng_format png_format = upng_get_format(upng);
    // Create a color palette in RGBA8 format from RGB24 + ALPHA8 PNG Palettes
    if (png_format >= UPNG_INDEXED1 && png_format <= UPNG_INDEXED8) {
      return prv_gbitmap_png_create_palette_for_color(upng, palette_out);
    } else if (png_format >= UPNG_LUMINANCE1 && png_format <= UPNG_LUMINANCE8) {
      return prv_gbitmap_png_create_palette_for_grayscale(upng, palette_out);
    }
  }
  return 0;
}

bool gbitmap_png_is_format_supported(upng_t *upng) {
  if (upng) {
    upng_format png_format = upng_get_format(upng);
    if ((png_format >= UPNG_INDEXED1 && png_format <= UPNG_INDEXED8) ||
        (png_format >= UPNG_LUMINANCE1 && png_format <= UPNG_LUMINANCE8)) {
      return true;
    }
  }
  return false;
}


int32_t gbitmap_png_get_transparent_gray_value(upng_t *upng) {
  int32_t transparent_gray = -1;  // default to invalid value
  // Handle grayscale transparency value (1 single transparent gray)
  uint8_t *alpha_palette = NULL;
  uint16_t alpha_palette_entries = upng_get_alpha_palette(upng, &alpha_palette);
  if (alpha_palette_entries == 2) {
    transparent_gray = ntohs(*(uint16_t*)alpha_palette);
  }
  return transparent_gray;
}
