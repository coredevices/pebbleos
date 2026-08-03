/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "weather_types.h"
#include "resource_ids.pin.h"

static int weather_type_slot_index(WeatherType weather_type) {
  int value = (int)weather_type;
  return (value >= 0 && value <= WeatherType_RainAndSnow) ? value : 9;
}

GColor weather_type_bg_color(WeatherType weather_type) {
  static const GColor s_colors[] = {
    GColorChromeYellow,   // PartlyCloudy
    GColorLightGray,      // CloudyDay
    GColorElectricBlue,   // LightSnow
    GColorPictonBlue,     // LightRain
    GColorBlueMoon,       // HeavyRain
    GColorTiffanyBlue,    // HeavySnow
    GColorLightGray,      // Generic
    GColorOrange,         // Sun
    GColorMidnightGreen,  // RainAndSnow
    GColorLightGray,      // Unknown
  };
  return s_colors[weather_type_slot_index(weather_type)];
}

uint32_t weather_type_icon_tiny_resource(WeatherType weather_type) {
  static const uint16_t s_resources[] = {
    RESOURCE_ID_IMAGE_PARTLY_CLOUDY_TINY,
    RESOURCE_ID_IMAGE_CLOUDY_DAY_TINY,
    RESOURCE_ID_IMAGE_LIGHT_SNOW_TINY,
    RESOURCE_ID_IMAGE_LIGHT_RAIN_TINY,
    RESOURCE_ID_IMAGE_HEAVY_RAIN_TINY,
    RESOURCE_ID_IMAGE_HEAVY_SNOW_TINY,
    RESOURCE_ID_IMAGE_GENERIC_WEATHER_TINY,
    RESOURCE_ID_IMAGE_SUNNY_DAY_TINY,
    RESOURCE_ID_IMAGE_RAIN_AND_SNOW_TINY,
    RESOURCE_ID_IMAGE_GENERIC_WEATHER_TINY,
  };
  return s_resources[weather_type_slot_index(weather_type)];
}

uint32_t weather_type_icon_small_resource(WeatherType weather_type) {
#if PBL_DISPLAY_HEIGHT >= 200
  // Every SMALL id sits exactly one above its TINY twin — except Sun, whose
  // SMALL is a separate legacy asset. Asserts pin the id-order invariant.
  _Static_assert(RESOURCE_ID_IMAGE_PARTLY_CLOUDY_SMALL == RESOURCE_ID_IMAGE_PARTLY_CLOUDY_TINY + 1, "id order");
  _Static_assert(RESOURCE_ID_IMAGE_CLOUDY_DAY_SMALL == RESOURCE_ID_IMAGE_CLOUDY_DAY_TINY + 1, "id order");
  _Static_assert(RESOURCE_ID_IMAGE_LIGHT_SNOW_SMALL == RESOURCE_ID_IMAGE_LIGHT_SNOW_TINY + 1, "id order");
  _Static_assert(RESOURCE_ID_IMAGE_LIGHT_RAIN_SMALL == RESOURCE_ID_IMAGE_LIGHT_RAIN_TINY + 1, "id order");
  _Static_assert(RESOURCE_ID_IMAGE_HEAVY_RAIN_SMALL == RESOURCE_ID_IMAGE_HEAVY_RAIN_TINY + 1, "id order");
  _Static_assert(RESOURCE_ID_IMAGE_HEAVY_SNOW_SMALL == RESOURCE_ID_IMAGE_HEAVY_SNOW_TINY + 1, "id order");
  _Static_assert(RESOURCE_ID_IMAGE_GENERIC_WEATHER_SMALL == RESOURCE_ID_IMAGE_GENERIC_WEATHER_TINY + 1, "id order");
  _Static_assert(RESOURCE_ID_IMAGE_RAIN_AND_SNOW_SMALL == RESOURCE_ID_IMAGE_RAIN_AND_SNOW_TINY + 1, "id order");
  if (weather_type_slot_index(weather_type) == 7) {
    return RESOURCE_ID_IMAGE_SUNNY_DAY_SMALL;   // the one exception (slot 7 = Sun)
  }
  return weather_type_icon_tiny_resource(weather_type) + 1;
#else
  return weather_type_icon_tiny_resource(weather_type);
#endif
}

GColor weather_type_disc_color(WeatherType weather_type) {
#if PBL_BW
  // 1-bit cannot carry the condition in the disc tone: nearly every colour
  // quantizes to the same 50% dither and a couple to black — an arbitrary
  // split that reads as noise. The icon alone carries the condition, so BW
  // draws no condition discs at all.
  (void)weather_type;
  return GColorClear;
#else
  return weather_type_bg_color(weather_type);
#endif
}

void weather_icon_invert_ink(GBitmap *icon) {
  if (!icon) return;
  GColor *palette = gbitmap_get_palette(icon);
  uint32_t entries = 0;
  switch (gbitmap_get_format(icon)) {
    case GBitmapFormat1BitPalette: entries = 2; break;
    case GBitmapFormat2BitPalette: entries = 4; break;
    case GBitmapFormat4BitPalette: entries = 16; break;
    default: break;   // non-palettized: leave as-is
  }
  if (!palette) return;
  for (uint32_t i = 0; i < entries; i++) {
    if (palette[i].a) {   // leave transparency alone
      palette[i].r = (uint8_t)(0b11 - palette[i].r);
      palette[i].g = (uint8_t)(0b11 - palette[i].g);
      palette[i].b = (uint8_t)(0b11 - palette[i].b);
    }
  }
}

void weather_icon_draw(GContext *ctx, GBitmap *icon, GRect rect, bool invert_ink) {
  if (!icon) return;
  graphics_context_set_compositing_mode(ctx, GCompOpSet);
  if (invert_ink) {
    weather_icon_invert_ink(icon);
  }
  graphics_draw_bitmap_in_rect(ctx, icon, rect);
  if (invert_ink) {
    weather_icon_invert_ink(icon);   // self-inverse restore — bitmaps are shared
  }
}

uint32_t weather_type_icon_large_resource(WeatherType weather_type) {
  static const uint16_t s_resources[] = {
    RESOURCE_ID_IMAGE_PARTLY_CLOUDY_LARGE,    // 0 PartlyCloudy
    RESOURCE_ID_IMAGE_CLOUDY_DAY_LARGE,       // 1 CloudyDay
    RESOURCE_ID_IMAGE_LIGHT_SNOW_LARGE,       // 2 LightSnow
    RESOURCE_ID_IMAGE_LIGHT_RAIN_LARGE,       // 3 LightRain
    RESOURCE_ID_IMAGE_HEAVY_RAIN_LARGE,       // 4 HeavyRain
    RESOURCE_ID_IMAGE_HEAVY_SNOW_LARGE,       // 5 HeavySnow
    RESOURCE_ID_IMAGE_GENERIC_WEATHER_LARGE,  // 6 Generic
    RESOURCE_ID_IMAGE_SUNNY_DAY_LARGE,        // 7 Sun
    RESOURCE_ID_IMAGE_RAIN_AND_SNOW_LARGE,    // 8 RainAndSnow
    RESOURCE_ID_IMAGE_GENERIC_WEATHER_LARGE,  // 9 Unknown
  };
  return s_resources[weather_type_slot_index(weather_type)];
}

