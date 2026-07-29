/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

// Shell Preferences
//
// These are preferences which must be available for querying across all shells
// and which must be implemented differently depending on the shell compiled in.
// Only preferences which are used by common services and kernel code meet these
// criteria.
//
// NEW PREFERENCES DO __NOT__ BELONG HERE WITHOUT A VERY GOOD REASON.

#include <stdbool.h>

#include "applib/graphics/gtypes.h"
#include "process_management/app_install_types.h"
#include "shell/system_theme.h"
#include "pbl/util/uuid.h"

#if defined(CONFIG_APP_SCALING) && \
    (defined(CONFIG_BOARD_OBELIX) || defined(CONFIG_BOARD_QEMU_EMERY))
#define TIMELINE_PEEK_WATCHFACE_FIT_SUPPORTED 1
#else
#define TIMELINE_PEEK_WATCHFACE_FIT_SUPPORTED 0
#endif


// The clock 12h/24h setting is required by services/clock.c.
bool shell_prefs_get_clock_24h_style(void);
void shell_prefs_set_clock_24h_style(bool is24h);

// The timezone source setting is required by services/clock.c.
// When the source is manual, we don't override our timezone with the phone's timezone info
bool shell_prefs_is_timezone_source_manual(void);
void shell_prefs_set_timezone_source_manual(bool manual);

// The time source setting is required by services/clock.c.
// When the source is manual, we don't override our time with the phone's time info
bool shell_prefs_is_time_source_manual(void);
void shell_prefs_set_time_source_manual(bool manual);

// The timezone id setting is required by services/clock.c.
// The automatic timezone id is what we get from the phone
int16_t shell_prefs_get_automatic_timezone_id(void);
void shell_prefs_set_automatic_timezone_id(int16_t timezone_id);

// Preferences for choosing the units that are displayed in various places in the UI
typedef enum UnitsDistance {
  UnitsDistance_KM,
  UnitsDistance_Miles,
  UnitsDistanceCount
} UnitsDistance;

UnitsDistance shell_prefs_get_units_distance(void);
void shell_prefs_set_units_distance(UnitsDistance newUnit);

// The backlight preferences are required in all shells, but the settings are
// hardcoded when running PRF.

// The backlight behaviour enum value is used by the light service analytics.
// This type has been deprecated for any other use, replaced by the enabled
// and ambient_sensor_enabled booleans.
typedef enum BacklightBehaviour {
  BacklightBehaviour_On = 0,
  BacklightBehaviour_Off = 1,
  BacklightBehaviour_Auto = 2,
  NumBacklightBehaviours
} BacklightBehaviour;

bool backlight_is_enabled(void);
void backlight_set_enabled(bool enabled);

bool backlight_is_ambient_sensor_enabled(void);
void backlight_set_ambient_sensor_enabled(bool enabled);

#define DEFAULT_BACKLIGHT_TIMEOUT_MS 3000
uint32_t backlight_get_timeout_ms(void);
void backlight_set_timeout_ms(uint32_t timeout_ms);

uint8_t backlight_get_intensity(void);

void backlight_set_intensity(uint8_t intensity);

#ifdef CONFIG_BACKLIGHT_HAS_COLOR
// Packed 0x00RRGGBB. Matches the BACKLIGHT_COLOR_* constants in drivers/backlight.h.
uint32_t backlight_get_default_color(void);
void backlight_set_default_color(uint32_t rgb_color);
#endif

// The backlight motion enabled setting is used by the kernel event loop.
bool backlight_is_motion_enabled(void);
void backlight_set_motion_enabled(bool enable);

// The backlight touch wake setting is used by the kernel event loop to decide
// whether touch gestures wake the backlight, and if so which gesture (single
// tap or double tap).
typedef enum BacklightTouchWake {
  BacklightTouchWake_DoubleTap = 0,
  BacklightTouchWake_Tap = 1,
  BacklightTouchWake_Off = 2,
} BacklightTouchWake;

BacklightTouchWake backlight_get_touch_wake(void);
void backlight_set_touch_wake(BacklightTouchWake wake);

// Global touch input kill-switch. When false, the kernel touch service
// drops events at the source, powers the sensor down, and the applib
// touch_service_is_enabled() query returns false to apps.
bool touch_is_globally_enabled(void);
void touch_set_globally_enabled(bool enable);

// Touch-navigation sub-pref, ANDed with the master "Touch" kill switch above
// (touch_is_globally_enabled) for the SYSTEM experience: menus, notifications
// and the button bridge are active only while BOTH are on; with this off
// nothing system-side subscribes to touch. Only the wake-gesture pref, SDK
// raw-touch apps, and third-party apps that explicitly opted in via
// app_touch_navigation_enable() (they follow the master switch alone) still
// consume the sensor. Defaults to off. Toggling either pref runs the
// enable/disable transaction when the effective (ANDed) state changes and
// re-evaluates the running app's twin otherwise.
bool touch_navigation_menu_is_enabled(void);
void touch_set_navigation_menu_enabled(bool enable);

#ifdef CONFIG_DYNAMIC_BACKLIGHT
// Dynamic backlight: how aggressively brightness ramps with ambient light.
// Every mode keeps the same dim floor; the mode selects the lux level at
// which the ramp reaches the user's max intensity (Bright = earliest).
typedef enum BacklightDynamicMode {
  BacklightDynamicMode_Off = 0,
  BacklightDynamicMode_Bright = 1,
  BacklightDynamicMode_Standard = 2,
  BacklightDynamicMode_Dim = 3,
  BacklightDynamicModeCount,
} BacklightDynamicMode;

BacklightDynamicMode backlight_get_dynamic_mode(void);
void backlight_set_dynamic_mode(BacklightDynamicMode mode);
// Convenience: mode != Off
bool backlight_is_dynamic_intensity_enabled(void);
#endif

// Backlight presets bundle the ambient sensor, dynamic mode (where dynamic
// backlight is available) and intensity settings into one user-facing mode;
// Advanced exposes the settings individually. A stored preset only reports
// as active while the underlying settings still match its values, otherwise
// Advanced is reported.
typedef enum BacklightPreset {
  BacklightPreset_MaxBrightness = 0,
  BacklightPreset_Standard = 1,
  BacklightPreset_BatterySaver = 2,
  BacklightPreset_Advanced = 3,
  BacklightPresetCount,
} BacklightPreset;

BacklightPreset backlight_get_preset(void);
void backlight_set_preset(BacklightPreset preset);

// Motion sensitivity for accelerometer shake detection (0-100, lower = less sensitive)
// Only available on platforms with LSM6DSO (Asterix, Obelix)
uint8_t shell_prefs_get_motion_sensitivity(void);
void shell_prefs_set_motion_sensitivity(uint8_t sensitivity);

// The backlight ambient light threshold setting
uint32_t backlight_get_ambient_threshold(void);
void backlight_set_ambient_threshold(uint32_t threshold);

// Stationary mode will put the watch in a low power state. Disabling will
// prevent the watch from turning off any features.
bool shell_prefs_get_stationary_enabled(void);
void shell_prefs_set_stationary_enabled(bool enabled);

// The default worker setting is used by process_management.
AppInstallId worker_preferences_get_default_worker(void);
void worker_preferences_set_default_worker(AppInstallId id);

bool shell_prefs_get_language_english(void);
void shell_prefs_set_language_english(bool english);
void shell_prefs_toggle_language_english(void);

typedef enum ShellLanguage {
  ShellLanguageInstalledPack = 0,
  ShellLanguageEnglish,
  ShellLanguageCatalan,
  ShellLanguageGerman,
  ShellLanguageSpanish,
  ShellLanguageFrench,
  ShellLanguageItalian,
  ShellLanguageDutch,
  ShellLanguagePortuguese,
  ShellLanguagePolish,
  ShellLanguageCount,
} ShellLanguage;

ShellLanguage shell_prefs_get_language(void);
uint32_t shell_prefs_get_language_resource_id(void);
void shell_prefs_set_language(ShellLanguage language);

uint8_t timeline_prefs_get_settings_opened(void);
void timeline_prefs_set_settings_opened(uint8_t version);
void timeline_peek_prefs_set_enabled(bool enabled);
bool timeline_peek_prefs_get_enabled(void);
void timeline_peek_prefs_set_before_time(uint16_t before_time_m);
uint16_t timeline_peek_prefs_get_before_time(void);
#if TIMELINE_PEEK_WATCHFACE_FIT_SUPPORTED
typedef enum TimelinePeekUnsupportedFaceMode {
  TimelinePeekUnsupportedFaceMode_None = 0,
  TimelinePeekUnsupportedFaceMode_SquishUp = 1,
  TimelinePeekUnsupportedFaceMode_ShiftUp = 2,
  TimelinePeekUnsupportedFaceModeCount,
} TimelinePeekUnsupportedFaceMode;

void timeline_peek_prefs_set_unsupported_face_mode(TimelinePeekUnsupportedFaceMode mode);
TimelinePeekUnsupportedFaceMode timeline_peek_prefs_get_unsupported_face_mode(void);
#endif

bool shell_prefs_can_coredump_on_request(void);
void shell_prefs_set_coredump_on_request(bool enabled);

// When enabled, accel shake detection logs are emitted at INFO level instead of DEBUG.
bool shell_prefs_get_accel_shake_log_info_enabled(void);
void shell_prefs_set_accel_shake_log_info_enabled(bool enabled);

// When enabled, vibration pattern logs are emitted at INFO level instead of DEBUG.
bool shell_prefs_get_vibe_log_info_enabled(void);
void shell_prefs_set_vibe_log_info_enabled(bool enabled);

// One-time migration flag: set to true once we have force-compacted every
// growable settings DB at boot. Devices that pre-date the growable-files
// change still have full-size settings files and only shrink to fit after
// this migration runs.
bool shell_prefs_get_settings_dbs_compacted_v1(void);
void shell_prefs_set_settings_dbs_compacted_v1(bool done);

#ifdef CONFIG_APP_SCALING
// Legacy app rendering mode - whether to use bezel or scaling for legacy apps
typedef enum LegacyAppRenderMode {
  LegacyAppRenderMode_Bezel = 0,    // Center with black bezel (original behavior)
  LegacyAppRenderMode_ScalingNearest = 1,  // Scale to fill screen (nearest-neighbor)
  LegacyAppRenderMode_ScalingBilinear = 2,  // Scale to fill screen (bilinear)
  LegacyAppRenderModeCount
} LegacyAppRenderMode;

LegacyAppRenderMode shell_prefs_get_legacy_app_render_mode(void);
void shell_prefs_set_legacy_app_render_mode(LegacyAppRenderMode mode);
#endif

#ifdef CONFIG_ORIENTATION_MANAGER
bool display_orientation_is_left(void);
void display_orientation_set_left(bool left);
#endif

GColor shell_prefs_get_theme_highlight_color(void);
void shell_prefs_set_theme_highlight_color(GColor color);

bool shell_prefs_get_menu_scroll_wrap_around_enable(void);
void shell_prefs_set_menu_scroll_wrap_around_enable(bool enable);

typedef enum MenuScrollVibeBehavior {
  MenuScrollNoVibe,
  MenuScrollVibeOnWrapAround,
  MenuScrollVibeOnLocked,
  MenuScrollVibeBehaviorsCount,
} MenuScrollVibeBehavior;

MenuScrollVibeBehavior shell_prefs_get_menu_scroll_vibe_behavior(void);
void shell_prefs_set_menu_scroll_vibe_behavior(MenuScrollVibeBehavior behavior);

bool shell_prefs_get_music_show_volume_controls(void);
void shell_prefs_set_music_show_volume_controls(bool enable);

bool shell_prefs_get_music_show_progress_bar(void);
void shell_prefs_set_music_show_progress_bar(bool enable);
