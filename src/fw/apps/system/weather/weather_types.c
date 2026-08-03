/* SPDX-FileCopyrightText: 2024 Google LLC */
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

#if defined(PBL_PLATFORM_GABBRO)
uint32_t weather_type_icon_clock_resource(WeatherType weather_type) {
  // The CLOCK id set is numerically identical to TINY, slot for slot — pinned:
  _Static_assert(RESOURCE_ID_IMAGE_PARTLY_CLOUDY_CLOCK == RESOURCE_ID_IMAGE_PARTLY_CLOUDY_TINY, "clock==tiny");
  _Static_assert(RESOURCE_ID_IMAGE_SUNNY_DAY_CLOCK == RESOURCE_ID_IMAGE_SUNNY_DAY_TINY, "clock==tiny");
  _Static_assert(RESOURCE_ID_IMAGE_RAIN_AND_SNOW_CLOCK == RESOURCE_ID_IMAGE_RAIN_AND_SNOW_TINY, "clock==tiny");
  return weather_type_icon_tiny_resource(weather_type);
}
#endif
