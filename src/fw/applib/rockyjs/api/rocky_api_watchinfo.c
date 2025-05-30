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

#include "rocky_api_watchinfo.h"

#include "applib/app_watch_info.h"
#include "applib/i18n.h"
#include "mfg/mfg_info.h"
#include "rocky_api_util.h"
#include "syscall/syscall.h"
#include "system/passert.h"
#include "system/version.h"

// rocky.watchInfo = {
//     platform: "basalt",
//     model: "pebble_time_red",
//     language: "en_US",
//     firmware: {
//         major: 4,
//         minor: 0,
//         patch: 1,
//         suffix: "beta3"
//     }
// }

#define ROCKY_WATCHINFO "watchInfo"
#define ROCKY_WATCHINFO_PLATFORM "platform"
#define ROCKY_WATCHINFO_MODEL "model"
#define ROCKY_WATCHINFO_LANG "language"
#define ROCKY_WATCHINFO_FW "firmware"
#define ROCKY_WATCHINFO_FW_MAJOR "major"
#define ROCKY_WATCHINFO_FW_MINOR "minor"
#define ROCKY_WATCHINFO_FW_PATCH "patch"
#define ROCKY_WATCHINFO_FW_SUFFIX "suffix"

static const char *prv_get_platform_name_string(void) {
  const PlatformType platform = sys_get_current_app_sdk_platform();
  return platform_type_get_name(platform);
}

static jerry_value_t prv_get_platform_name(void) {
  return jerry_create_string(
      (const jerry_char_t *)prv_get_platform_name_string());
}

#if PLATFORM_TINTIN
#  define TINTIN_MODEL(model_str) model_str
#else
#  define TINTIN_MODEL(model_str) NULL
#endif // PLATFORM_TINTIN

#if PLATFORM_SNOWY
#  define SNOWY_MODEL(model_str) model_str
#else
#  define SNOWY_MODEL(model_str) NULL
#endif // PLATFORM_SNOWY

#if PLATFORM_SPALDING
#  define SPALDING_MODEL(model_str) model_str
#else
#  define SPALDING_MODEL(model_str) NULL
#endif // PLATFORM_SPALDING

#if PLATFORM_SILK
#  define SILK_MODEL(model_str) model_str
#else
#  define SILK_MODEL(model_str) NULL
#endif // PLATFORM_SPALDING

#if PLATFORM_ASTERIX
#  define ASTERIX_MODEL(model_str) model_str
#else
#  define ASTERIX_MODEL(model_str) NULL
#endif // PLATFORM_ASTERIX

#if PLATFORM_OBELIX
#  define OBELIX_MODEL(model_str) model_str
#else
#  define OBELIX_MODEL(model_str) NULL
#endif // PLATFORM_OBELIX

#if PLATFORM_ROBERT
#  define ROBERT_MODEL(model_str) model_str
#else
#  define ROBERT_MODEL(model_str) NULL
#endif // PLATFORM_ROBERT


static jerry_value_t prv_get_model_name(void) {
  WatchInfoColor color = sys_watch_info_get_color();
  char *model_name = NULL;

  if (color < WATCH_INFO_COLOR__MAX) {
    switch (color) {
      case WATCH_INFO_COLOR_BLACK:
        model_name = TINTIN_MODEL("pebble_black");
        break;
      case WATCH_INFO_COLOR_WHITE:
        model_name = TINTIN_MODEL("pebble_white");
        break;
      case WATCH_INFO_COLOR_RED:
        model_name = TINTIN_MODEL("pebble_red");
        break;
      case WATCH_INFO_COLOR_ORANGE:
        model_name = TINTIN_MODEL("pebble_orange");
        break;
      case WATCH_INFO_COLOR_GRAY:
        model_name = TINTIN_MODEL("pebble_gray");
        break;
      case WATCH_INFO_COLOR_STAINLESS_STEEL:
        model_name = TINTIN_MODEL("pebble_steel_silver");
        break;
      case WATCH_INFO_COLOR_MATTE_BLACK:
        model_name = TINTIN_MODEL("pebble_steel_black");
        break;
      case WATCH_INFO_COLOR_BLUE:
        model_name = TINTIN_MODEL("pebble_blue");
        break;
      case WATCH_INFO_COLOR_GREEN:
        model_name = TINTIN_MODEL("pebble_green");
        break;
      case WATCH_INFO_COLOR_PINK:
        model_name = TINTIN_MODEL("pebble_pink");
        break;
      case WATCH_INFO_COLOR_TIME_WHITE:
        model_name = SNOWY_MODEL("pebble_time_white");
        break;
      case WATCH_INFO_COLOR_TIME_BLACK:
        model_name = SNOWY_MODEL("pebble_time_black");
        break;
      case WATCH_INFO_COLOR_TIME_RED:
        model_name = SNOWY_MODEL("pebble_time_red");
        break;
      case WATCH_INFO_COLOR_TIME_STEEL_SILVER:
        model_name = SNOWY_MODEL("pebble_time_steel_silver");
        break;
      case WATCH_INFO_COLOR_TIME_STEEL_BLACK:
        model_name = SNOWY_MODEL("pebble_time_steel_black");
        break;
      case WATCH_INFO_COLOR_TIME_STEEL_GOLD:
        model_name = SNOWY_MODEL("pebble_time_steel_gold");
        break;
      case WATCH_INFO_COLOR_TIME_ROUND_SILVER_14:
        model_name = SPALDING_MODEL("pebble_time_round_silver_14mm");
        break;
      case WATCH_INFO_COLOR_TIME_ROUND_BLACK_14:
        model_name = SPALDING_MODEL("pebble_time_round_black_14mm");
        break;
      case WATCH_INFO_COLOR_TIME_ROUND_SILVER_20:
        model_name = SPALDING_MODEL("pebble_time_round_silver_20mm");
        break;
      case WATCH_INFO_COLOR_TIME_ROUND_BLACK_20:
        model_name = SPALDING_MODEL("pebble_time_round_black_20mm");
        break;
      case WATCH_INFO_COLOR_TIME_ROUND_ROSE_GOLD_14:
        model_name = SPALDING_MODEL("pebble_time_round_rose_gold_14mm");
        break;
      case WATCH_INFO_COLOR_PEBBLE_2_HR_BLACK:
        model_name = SILK_MODEL("pebble_2_hr_black");
        break;
      case WATCH_INFO_COLOR_PEBBLE_2_HR_LIME:
        model_name = SILK_MODEL("pebble_2_hr_lime");
        break;
      case WATCH_INFO_COLOR_PEBBLE_2_HR_FLAME:
        model_name = SILK_MODEL("pebble_2_hr_flame");
        break;
      case WATCH_INFO_COLOR_PEBBLE_2_HR_WHITE:
        model_name = SILK_MODEL("pebble_2_hr_white");
        break;
      case WATCH_INFO_COLOR_PEBBLE_2_HR_AQUA:
        model_name = SILK_MODEL("pebble_2_hr_aqua");
        break;
      case WATCH_INFO_COLOR_PEBBLE_2_SE_BLACK:
        model_name = SILK_MODEL("pebble_2_se_black");
        break;
      case WATCH_INFO_COLOR_PEBBLE_2_SE_WHITE:
        model_name = SILK_MODEL("pebble_2_se_white");
        break;
      case WATCH_INFO_COLOR_PEBBLE_TIME_2_BLACK:
        model_name = ROBERT_MODEL("pebble_time_2_black");
        break;
      case WATCH_INFO_COLOR_PEBBLE_TIME_2_SILVER:
        model_name = ROBERT_MODEL("pebble_time_2_silver");
        break;
      case WATCH_INFO_COLOR_PEBBLE_TIME_2_GOLD:
        model_name = ROBERT_MODEL("pebble_time_2_gold");
        break;
      case WATCH_INFO_COLOR_COREDEVICES_C2D_BLACK:
        model_name = ASTERIX_MODEL("coredevices_c2d_black");
        break;
      case WATCH_INFO_COLOR_COREDEVICES_C2D_WHITE:
        model_name = ASTERIX_MODEL("coredevices_c2d_white");
        break;
      case WATCH_INFO_COLOR_COREDEVICES_CT2_BLACK:
        model_name = OBELIX_MODEL("coredevices_ct2_black");
        break;
      case WATCH_INFO_COLOR_UNKNOWN:
      case WATCH_INFO_COLOR__MAX:
        model_name = NULL;
        break;
    }
  }

  if (model_name) {
    return jerry_create_string((const jerry_char_t *)model_name);
  } else {
    // Assume we're running on QEMU
    const char *platform_name = prv_get_platform_name_string();
    const char *qemu_prefix = "qemu_platform_";
    char combined_string[strlen(qemu_prefix) + strlen(platform_name) + 1];
    strcpy(combined_string, qemu_prefix);
    strcat(combined_string, platform_name);
    return jerry_create_string((const jerry_char_t *)combined_string);
  }
}

static jerry_value_t prv_get_language(void) {
  return jerry_create_string((const jerry_char_t *)app_get_system_locale());
}

static jerry_value_t prv_get_fw_version(void) {
  WatchInfoVersion fw_version = watch_info_get_firmware_version();
  JS_VAR version_major = jerry_create_number(fw_version.major);
  JS_VAR version_minor = jerry_create_number(fw_version.minor);
  JS_VAR version_patch = jerry_create_number(fw_version.patch);

  // Parse the suffix out of the version tag
  FirmwareMetadata md;
  version_copy_running_fw_metadata(&md);
  char *suffix_str = strstr(md.version_tag, "-");
  if (suffix_str) {
    // Skip the '-' character
    suffix_str++;
  } else {
    suffix_str = "";
  }
  JS_VAR version_suffix = jerry_create_string((const jerry_char_t *)suffix_str);

  JS_VAR version_object = jerry_create_object();
  jerry_set_object_field(version_object, ROCKY_WATCHINFO_FW_MAJOR, version_major);
  jerry_set_object_field(version_object, ROCKY_WATCHINFO_FW_MINOR, version_minor);
  jerry_set_object_field(version_object, ROCKY_WATCHINFO_FW_PATCH, version_patch);
  jerry_set_object_field(version_object, ROCKY_WATCHINFO_FW_SUFFIX, version_suffix);

  // TODO: PBL-40413: Support .toString() on the fwversion field
  return jerry_acquire_value(version_object);
}

static void prv_fill_watchinfo(jerry_value_t watchinfo) {
  JS_VAR platform_name = prv_get_platform_name();
  JS_VAR model_name = prv_get_model_name();
  JS_VAR language = prv_get_language();
  JS_VAR fw_version = prv_get_fw_version();

  jerry_set_object_field(watchinfo, ROCKY_WATCHINFO_PLATFORM, platform_name);
  jerry_set_object_field(watchinfo, ROCKY_WATCHINFO_MODEL, model_name);
  jerry_set_object_field(watchinfo, ROCKY_WATCHINFO_LANG, language);
  jerry_set_object_field(watchinfo, ROCKY_WATCHINFO_FW, fw_version);
}

static void prv_init_apis(void) {
  bool was_created = false;
  JS_VAR rocky = rocky_get_rocky_singleton();
  JS_VAR watchinfo =
      rocky_get_or_create_object(rocky, ROCKY_WATCHINFO, rocky_creator_object, NULL, &was_created);
  PBL_ASSERTN(was_created);
  prv_fill_watchinfo(watchinfo);
}

const RockyGlobalAPI WATCHINFO_APIS = {
  .init = prv_init_apis,
};
