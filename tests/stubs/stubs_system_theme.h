/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "shell/system_theme.h"
#include "pbl/util/attributes.h"

#include "stubs_ambient_light.h"
#include "stubs_pebble_process_md.h"

#include <stdlib.h>

const char *WEAK system_theme_get_font_key(TextStyleFont font) {
  return NULL;
}

const char *WEAK system_theme_get_font_key_for_size(PreferredContentSize size,
                                                    TextStyleFont font) {
  return NULL;
}

GFont WEAK system_theme_get_font_for_default_size(TextStyleFont font) {
  return NULL;
}

PreferredContentSize WEAK system_theme_get_default_content_size_for_runtime_platform(void) {
  return PreferredContentSizeDefault;
}

PreferredContentSize WEAK system_theme_convert_host_content_size_to_runtime_platform(
    PreferredContentSize size) {
  return size;
}

GColor WEAK system_theme_get_bg_color(void) {
  return GColorWhite;
}

GColor WEAK system_theme_get_fg_color(void) {
  return GColorBlack;
}

bool WEAK system_theme_is_dark_mode(void) {
  return false;
}