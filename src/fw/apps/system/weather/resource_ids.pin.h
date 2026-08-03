/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
//! Weather app resource-id aliases. FIRMWARE builds alias the generated symbolic ids
//! (resource_ids.auto.h, included via pebble_compat.h) so the numbers can never drift
//! when the system pack reorders — the raw-number pinning this header used on the v4.18
//! stored-app build silently broke on v4.32 (the WX_* block shifted 492->493 and every
//! bitmap drew its neighbour). If the stored-app variant is ever revived, regenerate its
//! numeric pins from that build's resource_ids.auto.h.
#pragma once

// ---- TINY (25x25 PNG / WX_*_TINY) ----
#define RESOURCE_ID_IMAGE_SUNNY_DAY_TINY        RESOURCE_ID_WX_SUNNY_TINY
#define RESOURCE_ID_IMAGE_PARTLY_CLOUDY_TINY    RESOURCE_ID_WX_PARTLY_TINY
#define RESOURCE_ID_IMAGE_CLOUDY_DAY_TINY       RESOURCE_ID_WX_CLOUDY_TINY
#define RESOURCE_ID_IMAGE_LIGHT_RAIN_TINY       RESOURCE_ID_WX_LIGHTRAIN_TINY
#define RESOURCE_ID_IMAGE_HEAVY_RAIN_TINY       RESOURCE_ID_WX_HEAVYRAIN_TINY
#define RESOURCE_ID_IMAGE_LIGHT_SNOW_TINY       RESOURCE_ID_WX_LIGHTSNOW_TINY
#define RESOURCE_ID_IMAGE_HEAVY_SNOW_TINY       RESOURCE_ID_WX_HEAVYSNOW_TINY
#define RESOURCE_ID_IMAGE_RAIN_AND_SNOW_TINY    RESOURCE_ID_WX_RAINSNOW_TINY
#define RESOURCE_ID_IMAGE_GENERIC_WEATHER_TINY  RESOURCE_ID_WX_GENERIC_TINY

// ---- SMALL (50x50 PNG / WX_*_SMALL) ----
#define RESOURCE_ID_IMAGE_SUNNY_DAY_SMALL       RESOURCE_ID_WX_SUNNY_SMALL
#define RESOURCE_ID_IMAGE_PARTLY_CLOUDY_SMALL   RESOURCE_ID_WX_PARTLY_SMALL
#define RESOURCE_ID_IMAGE_CLOUDY_DAY_SMALL      RESOURCE_ID_WX_CLOUDY_SMALL
#define RESOURCE_ID_IMAGE_LIGHT_RAIN_SMALL      RESOURCE_ID_WX_LIGHTRAIN_SMALL
#define RESOURCE_ID_IMAGE_HEAVY_RAIN_SMALL      RESOURCE_ID_WX_HEAVYRAIN_SMALL
#define RESOURCE_ID_IMAGE_LIGHT_SNOW_SMALL      RESOURCE_ID_WX_LIGHTSNOW_SMALL
#define RESOURCE_ID_IMAGE_HEAVY_SNOW_SMALL      RESOURCE_ID_WX_HEAVYSNOW_SMALL
#define RESOURCE_ID_IMAGE_RAIN_AND_SNOW_SMALL   RESOURCE_ID_WX_RAINSNOW_SMALL
#define RESOURCE_ID_IMAGE_GENERIC_WEATHER_SMALL RESOURCE_ID_WX_GENERIC_SMALL

// ---- LARGE (80x80 PDC) weather icons — the SYSTEM pack's own big weather icons
//      (rasterised from Pebble_80x80_*.svg). These are the exact icons the Timeline
//      weather pin card renders, so the expanded card matches it. Aliased to the
//      generated RESOURCE_ID_*_LARGE names (resource_ids.auto.h, included via
//      pebble_compat.h) — the numeric ids are PER-PLATFORM, so raw numbers would
//      silently drift on gabbro. Drawn via gdraw_command_image_* (PDC vectors). ----
//      Sunny/partly/cloudy/generic use the app's own restyled copies (WX_*_LARGE);
//      the rain/snow types render the shared system PDCs unchanged.
#define RESOURCE_ID_IMAGE_PARTLY_CLOUDY_LARGE   RESOURCE_ID_WX_PARTLY_LARGE
#define RESOURCE_ID_IMAGE_CLOUDY_DAY_LARGE      RESOURCE_ID_WX_CLOUDY_LARGE
#define RESOURCE_ID_IMAGE_LIGHT_SNOW_LARGE      RESOURCE_ID_LIGHT_SNOW_LARGE
#define RESOURCE_ID_IMAGE_LIGHT_RAIN_LARGE      RESOURCE_ID_LIGHT_RAIN_LARGE
#define RESOURCE_ID_IMAGE_HEAVY_RAIN_LARGE      RESOURCE_ID_HEAVY_RAIN_LARGE
#define RESOURCE_ID_IMAGE_HEAVY_SNOW_LARGE      RESOURCE_ID_HEAVY_SNOW_LARGE
#define RESOURCE_ID_IMAGE_RAIN_AND_SNOW_LARGE   RESOURCE_ID_RAINING_AND_SNOWING_LARGE
#define RESOURCE_ID_IMAGE_GENERIC_WEATHER_LARGE RESOURCE_ID_WX_GENERIC_LARGE
#define RESOURCE_ID_IMAGE_SUNNY_DAY_LARGE       RESOURCE_ID_WX_SUNNY_LARGE

// ---- CLOCK icons: gabbro-only (compiled out on emery). Mapped to the TINY
//      (25x25) ids so the source matches CLOCK_ICON_SIZE (25) — previously
//      aliased to the 50x50 SMALL ids, which made the round clock draw a
//      cropped/off-centre top-left 25-of-50 icon. ----
#define RESOURCE_ID_IMAGE_SUNNY_DAY_CLOCK       RESOURCE_ID_WX_SUNNY_TINY
#define RESOURCE_ID_IMAGE_PARTLY_CLOUDY_CLOCK   RESOURCE_ID_WX_PARTLY_TINY
#define RESOURCE_ID_IMAGE_CLOUDY_DAY_CLOCK      RESOURCE_ID_WX_CLOUDY_TINY
#define RESOURCE_ID_IMAGE_LIGHT_RAIN_CLOCK      RESOURCE_ID_WX_LIGHTRAIN_TINY
#define RESOURCE_ID_IMAGE_HEAVY_RAIN_CLOCK      RESOURCE_ID_WX_HEAVYRAIN_TINY
#define RESOURCE_ID_IMAGE_LIGHT_SNOW_CLOCK      RESOURCE_ID_WX_LIGHTSNOW_TINY
#define RESOURCE_ID_IMAGE_HEAVY_SNOW_CLOCK      RESOURCE_ID_WX_HEAVYSNOW_TINY
#define RESOURCE_ID_IMAGE_RAIN_AND_SNOW_CLOCK   RESOURCE_ID_WX_RAINSNOW_TINY
#define RESOURCE_ID_IMAGE_GENERIC_WEATHER_CLOCK RESOURCE_ID_WX_GENERIC_TINY

// RESULT_SHREDDED_LARGE intentionally NOT pinned: the animated shredder PDC is a
// real pack entry on this family (resource_ids.auto.h, via pebble_compat.h) and
// the saved-locations "Location Deleted" screen plays it.

// ---- PDC sequences (round/gabbro only). The main weather-icons sequence is now
//      shipped in the pack as WX_WEATHER_ICONS_PDC; alias the app's name to the
//      real auto id (the auto header is included via pebble_compat before this).
//      WEATHER_CLOCK_ICONS_PDC has no asset yet (forecast_list null-checks it). ----
#define RESOURCE_ID_WEATHER_ICONS_PDC           RESOURCE_ID_WX_WEATHER_ICONS_PDC
#define RESOURCE_ID_WEATHER_CLOCK_ICONS_PDC     0

// ---- Globe resources (type raw): pinned to the real ids from the built pack. ----
