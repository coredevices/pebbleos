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

#include "app_watch_info.h"

#include "syscall/syscall_internal.h"
#include "system/version.h"
#include "mfg/mfg_info.h"

#include "git_version.auto.h"

DEFINE_SYSCALL(WatchInfoColor, sys_watch_info_get_color, void) {
  return mfg_info_get_watch_color();
}

WatchInfoModel watch_info_get_model() {
  // Pull the model for pebble time steel from the factory set model color bits.
  switch (sys_watch_info_get_color()) {
    // Pebble Original Colors
    case WATCH_INFO_COLOR_BLACK:
    case WATCH_INFO_COLOR_WHITE:
    case WATCH_INFO_COLOR_RED:
    case WATCH_INFO_COLOR_ORANGE:
    case WATCH_INFO_COLOR_GRAY:
    case WATCH_INFO_COLOR_BLUE:
    case WATCH_INFO_COLOR_GREEN:
    case WATCH_INFO_COLOR_PINK:
      return WATCH_INFO_MODEL_PEBBLE_ORIGINAL;
    // Pebble Steel Colors
    case WATCH_INFO_COLOR_STAINLESS_STEEL:
    case WATCH_INFO_COLOR_MATTE_BLACK:
      return WATCH_INFO_MODEL_PEBBLE_STEEL;
    // Pebble Time Colors
    case WATCH_INFO_COLOR_TIME_WHITE:
    case WATCH_INFO_COLOR_TIME_BLACK:
    case WATCH_INFO_COLOR_TIME_RED:
      return WATCH_INFO_MODEL_PEBBLE_TIME;
    // Pebble Time Steel Colors
    case WATCH_INFO_COLOR_TIME_STEEL_SILVER:
    case WATCH_INFO_COLOR_TIME_STEEL_BLACK:
    case WATCH_INFO_COLOR_TIME_STEEL_GOLD:
      return WATCH_INFO_MODEL_PEBBLE_TIME_STEEL;
    case WATCH_INFO_COLOR_TIME_ROUND_BLACK_14:
    case WATCH_INFO_COLOR_TIME_ROUND_SILVER_14:
    case WATCH_INFO_COLOR_TIME_ROUND_ROSE_GOLD_14:
      return WATCH_INFO_MODEL_PEBBLE_TIME_ROUND_14;
    case WATCH_INFO_COLOR_TIME_ROUND_BLACK_20:
    case WATCH_INFO_COLOR_TIME_ROUND_SILVER_20:
      return WATCH_INFO_MODEL_PEBBLE_TIME_ROUND_20;
    case WATCH_INFO_COLOR_PEBBLE_2_HR_BLACK:
    case WATCH_INFO_COLOR_PEBBLE_2_HR_LIME:
    case WATCH_INFO_COLOR_PEBBLE_2_HR_FLAME:
    case WATCH_INFO_COLOR_PEBBLE_2_HR_WHITE:
    case WATCH_INFO_COLOR_PEBBLE_2_HR_AQUA:
      return WATCH_INFO_MODEL_PEBBLE_2_HR;
    case WATCH_INFO_COLOR_PEBBLE_2_SE_BLACK:
    case WATCH_INFO_COLOR_PEBBLE_2_SE_WHITE:
      return WATCH_INFO_MODEL_PEBBLE_2_SE;
    case WATCH_INFO_COLOR_PEBBLE_TIME_2_BLACK:
    case WATCH_INFO_COLOR_PEBBLE_TIME_2_SILVER:
    case WATCH_INFO_COLOR_PEBBLE_TIME_2_GOLD:
      return WATCH_INFO_MODEL_PEBBLE_TIME_2;
    case WATCH_INFO_COLOR_COREDEVICES_C2D_BLACK:
    case WATCH_INFO_COLOR_COREDEVICES_C2D_WHITE:
      return WATCH_INFO_MODEL_COREDEVICES_C2D;
    case WATCH_INFO_COLOR_COREDEVICES_CT2_BLACK:
      return WATCH_INFO_MODEL_COREDEVICES_CT2;
    case WATCH_INFO_COLOR_UNKNOWN:
    case WATCH_INFO_COLOR__MAX:
      return WATCH_INFO_MODEL_UNKNOWN;
  }
  // Should never be reached
  return WATCH_INFO_MODEL_UNKNOWN;
}

WatchInfoVersion watch_info_get_firmware_version(void) {
  return (WatchInfoVersion) {
    .major = GIT_MAJOR_VERSION,
    .minor = GIT_MINOR_VERSION,
    .patch = GIT_PATCH_VERSION
  };
}
