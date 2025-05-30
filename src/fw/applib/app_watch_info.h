/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <inttypes.h>

//! @addtogroup Foundation
//! @{
//!   @addtogroup WatchInfo
//! \brief Provides information about the watch itself.
//!
//! This API provides access to information such as the watch model, watch color
//! and watch firmware version.
//!   @{

//! The different watch models.
typedef enum {
  WATCH_INFO_MODEL_UNKNOWN, //!< Unknown model
  WATCH_INFO_MODEL_PEBBLE_ORIGINAL, //!< Original Pebble
  WATCH_INFO_MODEL_PEBBLE_STEEL, //!< Pebble Steel
  WATCH_INFO_MODEL_PEBBLE_TIME, //!< Pebble Time
  WATCH_INFO_MODEL_PEBBLE_TIME_STEEL, //!< Pebble Time Steel
  WATCH_INFO_MODEL_PEBBLE_TIME_ROUND_14, //!< Pebble Time Round, 14mm lug size
  WATCH_INFO_MODEL_PEBBLE_TIME_ROUND_20, //!< Pebble Time Round, 20mm lug size
  WATCH_INFO_MODEL_PEBBLE_2_HR, //!< Pebble 2 HR
  WATCH_INFO_MODEL_PEBBLE_2_SE, //!< Pebble 2 SE
  WATCH_INFO_MODEL_PEBBLE_TIME_2, //!< Pebble Time 2
  WATCH_INFO_MODEL_COREDEVICES_C2D, //!< CoreDevices C2D (Core 2 Duo)
  WATCH_INFO_MODEL_COREDEVICES_CT2, //!< CoreDevices CT2 (Core Time 2)

  WATCH_INFO_MODEL__MAX
} WatchInfoModel;

//! The different watch colors.
// This color enum is programmed by the factory into the factory registry. Therefore these
// numbers must not change.
typedef enum {
  WATCH_INFO_COLOR_UNKNOWN = 0, //!< Unknown color
  WATCH_INFO_COLOR_BLACK = 1, //!< Black
  WATCH_INFO_COLOR_WHITE = 2, //!< White
  WATCH_INFO_COLOR_RED = 3, //!< Red
  WATCH_INFO_COLOR_ORANGE = 4, //!< Orange
  WATCH_INFO_COLOR_GRAY = 5, //!< Gray

  WATCH_INFO_COLOR_STAINLESS_STEEL = 6, //!< Stainless Steel
  WATCH_INFO_COLOR_MATTE_BLACK = 7, //!< Matte Black

  WATCH_INFO_COLOR_BLUE = 8, //!< Blue
  WATCH_INFO_COLOR_GREEN = 9, //!< Green
  WATCH_INFO_COLOR_PINK = 10, //!< Pink

  WATCH_INFO_COLOR_TIME_WHITE = 11, //!< Time White
  WATCH_INFO_COLOR_TIME_BLACK = 12, //!< Time Black
  WATCH_INFO_COLOR_TIME_RED = 13, //!< Time Red

  WATCH_INFO_COLOR_TIME_STEEL_SILVER = 14, //!< Time Steel Silver
  WATCH_INFO_COLOR_TIME_STEEL_BLACK = 15, //!< Time Steel Black
  WATCH_INFO_COLOR_TIME_STEEL_GOLD = 16, //!< Time Steel Gold

  WATCH_INFO_COLOR_TIME_ROUND_SILVER_14 = 17, //!< Time Round 14mm lug size, Silver
  WATCH_INFO_COLOR_TIME_ROUND_BLACK_14 = 18, //!< Time Round 14mm lug size, Black
  WATCH_INFO_COLOR_TIME_ROUND_SILVER_20 = 19, //!< Time Round 20mm lug size, Silver
  WATCH_INFO_COLOR_TIME_ROUND_BLACK_20 = 20, //!< Time Round 20mm lug size, Black
  WATCH_INFO_COLOR_TIME_ROUND_ROSE_GOLD_14 = 21, //!< Time Round 14mm lug size, Rose Gold

  WATCH_INFO_COLOR_PEBBLE_2_HR_BLACK = 25, //!< Pebble 2 HR, Black / Charcoal
  WATCH_INFO_COLOR_PEBBLE_2_HR_LIME = 27, //!< Pebble 2 HR, Charcoal / Sorbet Green
  WATCH_INFO_COLOR_PEBBLE_2_HR_FLAME = 28, //!< Pebble 2 HR, Charcoal / Red
  WATCH_INFO_COLOR_PEBBLE_2_HR_WHITE = 29, //!< Pebble 2 HR, White / Gray
  WATCH_INFO_COLOR_PEBBLE_2_HR_AQUA = 30, //!< Pebble 2 HR, White / Turquoise

  WATCH_INFO_COLOR_PEBBLE_2_SE_BLACK = 24, //!< Pebble 2 SE, Black / Charcoal
  WATCH_INFO_COLOR_PEBBLE_2_SE_WHITE = 26, //!< Pebble 2 SE, White / Gray

  WATCH_INFO_COLOR_PEBBLE_TIME_2_BLACK = 31, //!< Pebble Time 2, Black
  WATCH_INFO_COLOR_PEBBLE_TIME_2_SILVER = 32, //!< Pebble Time 2, Silver
  WATCH_INFO_COLOR_PEBBLE_TIME_2_GOLD = 33, //!< Pebble Time 2, Gold

  WATCH_INFO_COLOR_COREDEVICES_C2D_BLACK = 34, //!< CoreDevices C2D, Black
  WATCH_INFO_COLOR_COREDEVICES_C2D_WHITE = 35, //!< CoreDevices C2D, White

  WATCH_INFO_COLOR_COREDEVICES_CT2_BLACK = 36, //!< CoreDevices CT2, Black
  WATCH_INFO_COLOR__MAX
} WatchInfoColor;

//! Data structure containing the version of the firmware running on the watch.
//! The version of the firmware has the form X.[X.[X]]. If a version number is not present it will be 0.
//! For example: the version numbers of 2.4.1 are 2, 4, and 1. The version numbers of 2.4 are 2, 4, and 0.
typedef struct {
  uint8_t major; //!< Major version number
  uint8_t minor; //!< Minor version number
  uint8_t patch; //!< Patch version number
} WatchInfoVersion;

//! Provides the model of the watch.
//! @return {@link WatchInfoModel} representing the model of the watch.
WatchInfoModel watch_info_get_model(void);

//! Provides the version of the firmware running on the watch.
//! @return {@link WatchInfoVersion} representing the version of the firmware running on the watch.
WatchInfoVersion watch_info_get_firmware_version(void);

//! @internal Get the watch color from mfg info
WatchInfoColor sys_watch_info_get_color(void);

//!   @} // end addtogroup WatchInfo
//! @} // end addtogroup Foundation

