/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "shell/prefs.h"
#include "pbl/util/attributes.h"

static bool s_clock_24h;

bool WEAK shell_prefs_get_clock_24h_style(void) {
  return s_clock_24h;
}

void WEAK shell_prefs_set_clock_24h_style(bool is_24h) {
  s_clock_24h= is_24h;
}

static bool s_clock_timeezone_manual;

bool WEAK shell_prefs_is_timezone_source_manual(void) {
  return s_clock_timeezone_manual;
}

void WEAK shell_prefs_set_timezone_source_manual(bool manual) {
  s_clock_timeezone_manual = manual;
}

static bool s_clock_time_source_manual;

bool WEAK shell_prefs_is_time_source_manual(void) {
  return s_clock_time_source_manual;
}

void WEAK shell_prefs_set_time_source_manual(bool manual) {
  s_clock_time_source_manual = manual;
}

static int16_t s_timezone_id;

int16_t shell_prefs_get_automatic_timezone_id(void) {
  return s_timezone_id;
}

void shell_prefs_set_automatic_timezone_id(int16_t timezone_id) {
  s_timezone_id = timezone_id;
}

UnitsDistance WEAK shell_prefs_get_units_distance(void) {
  return UnitsDistance_Miles;
}

AppInstallId WEAK worker_preferences_get_default_worker(void) {
  return 0;
}

PreferredContentSize s_content_size = PreferredContentSizeDefault;

void WEAK system_theme_set_content_size(PreferredContentSize content_size) {
  s_content_size = content_size;
}

PreferredContentSize WEAK system_theme_get_content_size(void) {
  return (PreferredContentSize)s_content_size;
}

static bool s_menu_scroll_enable = false;

bool WEAK shell_prefs_get_menu_scroll_wrap_around_enable(void) {
  return s_menu_scroll_enable;
}

void WEAK shell_prefs_set_menu_scroll_wrap_around_enable(bool enable) {
  s_menu_scroll_enable = enable;
}

static MenuScrollVibeBehavior s_menu_scroll_vibe_behavior = MenuScrollNoVibe;

MenuScrollVibeBehavior WEAK shell_prefs_get_menu_scroll_vibe_behavior(void) {
  return s_menu_scroll_vibe_behavior;
}

void WEAK shell_prefs_set_menu_scroll_vibe_behavior(MenuScrollVibeBehavior behavior) {
  s_menu_scroll_vibe_behavior = behavior;
}

#ifdef CONFIG_ORIENTATION_MANAGER
static bool s_display_orientation_left = false;

bool WEAK display_orientation_is_left(void) {
  return s_display_orientation_left;
}

void WEAK display_orientation_set_left(bool left) {
  s_display_orientation_left = left;
}
#endif