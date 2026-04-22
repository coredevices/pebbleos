/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

//! @file stubs.c
//!
//! This file stubs out functionality that we don't want in PRF. Ideally this file wouldn't have
//! to exist because systems that were common to both PRF and normal firmware wouldn't try to
//! use something that only exists in normal, but we're not quite there yet.

#include "pbl/util/uuid.h"
#include "board/board.h"
#include <pbl/drivers/backlight.h>
#include "kernel/events.h"
#include "popups/crashed_ui.h"
#include "popups/notifications/notification_window.h"
#include "process_management/app_install_manager.h"
#include "process_management/pebble_process_md.h"
#include "resource/resource_ids.auto.h"
#include "resource/resource_storage.h"
#include "resource/resource_storage_file.h"
#include "pbl/services/light.h"
#include "pbl/services/notifications/do_not_disturb.h"
#include "pbl/services/notifications/alerts_private.h"
#include "pbl/services/persist.h"
#include "shell/prefs.h"

void app_fetch_binaries(const Uuid *uuid, AppInstallId app_id, bool has_worker) {
}

const char *app_custom_get_title(AppInstallId app_id) {
  return NULL;
}

void crashed_ui_show_worker_crash(AppInstallId install_id) {
}

void crashed_ui_show_forced_core_dump(void) {
}

void app_idle_timeout_stop(void) {
}

void wakeup_popup_window(uint8_t missed_apps_count, uint8_t *missed_apps_banks) {
}

void watchface_set_default_install_id(AppInstallId id) {
}

void watchface_handle_button_event(PebbleEvent *e) {
}

void app_idle_timeout_refresh(void) {
}

void app_idle_timeout_touch_down(void) {
}

void app_idle_timeout_touch_up(void) {
}

PebblePhoneCaller* phone_call_util_create_caller(const char *number, const char *name) {
  return NULL;
}

void alarm_set_snooze_delay(int delay_ms) {
}

const void* const g_pbl_system_tbl[] = {};

const FileResourceData g_file_resource_stores[] = {};
const uint32_t g_num_file_resource_stores = 0;

void persist_service_client_open(const Uuid *uuid) {
}

void persist_service_client_close(const Uuid *uuid) {
}

SettingsFile * persist_service_lock_and_get_store(const Uuid *uuid) {
  return NULL;
}

status_t persist_service_delete_file(const Uuid *uuid) {
  return E_INVALID_OPERATION;
}

void wakeup_enable(bool enable) {
}

bool phone_call_is_using_ANCS(void) {
  return true;
}

#include "pbl/services/notifications/alerts.h"

#include "pbl/services/blob_db/app_db.h"
#include "pbl/services/app_cache.h"
#include "pbl/services/blob_db/pin_db.h"

status_t pin_db_delete_with_parent(const TimelineItemId *parent_id) {
  return E_INVALID_OPERATION;
}

status_t app_cache_add_entry(AppInstallId app_id, uint32_t total_size) {
  return E_INVALID_OPERATION;
}

status_t app_cache_remove_entry(AppInstallId id) {
  return E_INVALID_OPERATION;
}

status_t app_db_insert(const uint8_t *key, int key_len, const uint8_t *val, int val_len) {
  return E_INVALID_OPERATION;
}

status_t app_db_delete(const uint8_t *key, int key_len) {
  return E_INVALID_OPERATION;
}

AppInstallId app_db_check_next_unique_id(void) {
  return 0;
}

void app_db_enumerate_entries(AppDBEnumerateCb cb, void *data) {
}

AppInstallId app_db_get_install_id_for_uuid(const Uuid *uuid) {
  return 0;
}

status_t app_db_get_app_entry_for_install_id(AppInstallId app_id, AppDBEntry *entry) {
  return E_INVALID_OPERATION;
}

bool app_db_exists_install_id(AppInstallId app_id) {
  return false;
}

void timeline_item_destroy(TimelineItem* item) {
}

AppInstallId worker_preferences_get_default_worker(void) {
  return INSTALL_ID_INVALID;
}

#include "process_management/process_loader.h"
void * process_loader_load(const PebbleProcessMd *app_md, PebbleTask task,
                         MemorySegment *destination) {
  return app_md->main_func;
}

#include "pbl/services/process_management/app_storage.h"
AppStorageGetAppInfoResult app_storage_get_process_info(PebbleProcessInfo* app_info,
                                                        uint8_t *build_id_out,
                                                        AppInstallId app_id,
                                                        PebbleTask task) {
  return GET_APP_INFO_COULD_NOT_READ_FORMAT;
}

void app_storage_get_file_name(char *name, size_t buf_length, AppInstallId app_id,
                               PebbleTask task) {
  // Empty string
  *name = 0;
}

bool shell_prefs_get_clock_24h_style(void) {
  return true;
}

void shell_prefs_set_clock_24h_style(bool is_24h_style) {
}

bool shell_prefs_is_timezone_source_manual(void) {
  return false;
}

void shell_prefs_set_timezone_source_manual(bool manual) {
}

void shell_prefs_set_automatic_timezone_id(int16_t timezone_id) {
}

int16_t shell_prefs_get_automatic_timezone_id(void) {
  return -1;
}

bool shell_prefs_can_coredump_on_request() {
  // it would be good to have a core dump escape hatch in PRF
  return true;
}

AlertMask alerts_get_mask(void) {
  return AlertMaskAllOff;
}

bool do_not_disturb_is_active(void) {
  return true;
}

BacklightBehaviour backlight_get_behaviour(void) {
  return BacklightBehaviour_On;
}

bool backlight_is_enabled(void) {
  return true;
}

bool backlight_is_ambient_sensor_enabled(void) {
  return false;
}

bool backlight_is_motion_enabled(void) {
  return false;
}

BacklightTouchWake backlight_get_touch_wake(void) {
  return BacklightTouchWake_Off;
}

void backlight_set_touch_wake(BacklightTouchWake wake) {
}

bool touch_is_globally_enabled(void) {
  return true;
}

void touch_set_globally_enabled(bool enable) {
}


bool touch_navigation_menu_is_enabled(void) {
  return false;
}

void touch_set_navigation_menu_enabled(bool enable) {
}

bool bt_persistent_storage_get_airplane_mode_enabled(void) {
  return false;
}

void bt_persistent_storage_set_airplane_mode_enabled(bool *state) {
}

uint32_t backlight_get_timeout_ms(void) {
  return DEFAULT_BACKLIGHT_TIMEOUT_MS;
}

uint8_t backlight_get_intensity(void) {
  return 100U;
}

#ifdef CONFIG_BACKLIGHT_HAS_COLOR
uint32_t backlight_get_default_color(void) {
  return BOARD_CONFIG.backlight_default_color;
}

void backlight_set_default_color(uint32_t rgb_color) {
}
#endif

bool shell_prefs_get_language_english(void) {
  return true;
}
void shell_prefs_set_language_english(bool english) {
}
void shell_prefs_toggle_language_english(void) {
}
ShellLanguage shell_prefs_get_language(void) {
  return ShellLanguageEnglish;
}
uint32_t shell_prefs_get_language_resource_id(void) {
  return RESOURCE_ID_INVALID;
}
void shell_prefs_set_language(ShellLanguage language) {
}

GColor shell_prefs_get_theme_highlight_color(void) {
  return PBL_IF_COLOR_ELSE(GColorVividCerulean, GColorBlack);
}

DarkMode shell_prefs_get_dark_mode(void) {
  return DarkModeOff;
}

void shell_prefs_set_dark_mode(DarkMode mode) {
}

void shell_prefs_get_dark_mode_schedule(DarkModeSchedule *schedule_out) {
  if (schedule_out) {
    *schedule_out = (DarkModeSchedule){ .from_hour = 19, .from_minute = 0, .to_hour = 7, .to_minute = 0 };
  }
}

void shell_prefs_set_dark_mode_schedule(const DarkModeSchedule *schedule) {
}

FontInfo *fonts_get_system_emoji_font_for_size(unsigned int font_height) {
  return NULL;
}

int16_t timeline_peek_get_origin_y(void) {
  return DISP_ROWS;
}

int16_t timeline_peek_get_obstruction_origin_y(void) {
  return DISP_ROWS;
}

void timeline_peek_handle_process_start(void) { }

void timeline_peek_handle_process_kill(void) { }

void pbl_analytics_external_collect_pfs_stats(void) {
}

void pbl_analytics_external_collect_settings(void) {
}
