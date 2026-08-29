// Included by prefs.c to declare the list of preferences and their keys
  PREFS_MACRO(PREF_KEY_CLOCK_24H, s_clock_24h)
  PREFS_MACRO(PREF_KEY_CLOCK_TIMEZONE_SOURCE_IS_MANUAL, s_clock_timezone_source_is_manual)
  PREFS_MACRO(PREF_KEY_CLOCK_TIME_SOURCE_IS_MANUAL, s_clock_time_source_is_manual)
  PREFS_MACRO(PREF_KEY_CLOCK_PHONE_TIMEZONE_ID, s_clock_phone_timezone_id)
  PREFS_MACRO(PREF_KEY_UNITS_DISTANCE, s_units_distance)
  PREFS_MACRO(PREF_KEY_BACKLIGHT_ENABLED, s_backlight_enabled)
  PREFS_MACRO(PREF_KEY_BACKLIGHT_AMBIENT_SENSOR_ENABLED, s_backlight_ambient_sensor_enabled)
  PREFS_MACRO(PREF_KEY_BACKLIGHT_TIMEOUT_MS, s_backlight_timeout_ms)
  PREFS_MACRO(PREF_KEY_BACKLIGHT_INTENSITY, s_backlight_intensity)
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
  PREFS_MACRO(PREF_KEY_BACKLIGHT_COLOR, s_backlight_color)
#endif
  PREFS_MACRO(PREF_KEY_BACKLIGHT_MOTION, s_backlight_motion_enabled)
  PREFS_MACRO(PREF_KEY_BACKLIGHT_TOUCH, s_backlight_touch_wake)
  PREFS_MACRO(PREF_KEY_TOUCH_ENABLED, s_touch_enabled)
  PREFS_MACRO(PREF_KEY_TOUCH_NAVIGATION_MENU, s_touch_navigation_menu_enabled)
  PREFS_MACRO(PREF_KEY_MOTION_SENSITIVITY, s_motion_sensitivity)
#ifdef CONFIG_DYNAMIC_BACKLIGHT
  PREFS_MACRO(PREF_KEY_BACKLIGHT_DYNAMIC_MODE, s_backlight_dynamic_mode)
#endif
  PREFS_MACRO(PREF_KEY_BACKLIGHT_PRESET, s_backlight_preset)
  PREFS_MACRO(PREF_KEY_BACKLIGHT_AMBIENT_THRESHOLD, s_backlight_ambient_threshold)
  PREFS_MACRO(PREF_KEY_STATIONARY, s_stationary_mode_enabled)
  PREFS_MACRO(PREF_KEY_CHARGE_LIMIT_PCT, s_charge_limit_pct)
  PREFS_MACRO(PREF_KEY_DEFAULT_WORKER, s_default_worker)
  PREFS_MACRO(PREF_KEY_TEXT_STYLE, s_text_style)
  PREFS_MACRO(PREF_KEY_LANG_ENGLISH, s_language_english)
  PREFS_MACRO(PREF_KEY_LANGUAGE, s_language)
  PREFS_MACRO(PREF_KEY_QUICK_LAUNCH_UP, s_quick_launch_up)
  PREFS_MACRO(PREF_KEY_QUICK_LAUNCH_DOWN, s_quick_launch_down)
  PREFS_MACRO(PREF_KEY_QUICK_LAUNCH_SELECT, s_quick_launch_select)
  PREFS_MACRO(PREF_KEY_QUICK_LAUNCH_BACK, s_quick_launch_back)
  PREFS_MACRO(PREF_KEY_QUICK_LAUNCH_SINGLE_CLICK_UP, s_quick_launch_single_click_up)
  PREFS_MACRO(PREF_KEY_QUICK_LAUNCH_SINGLE_CLICK_DOWN, s_quick_launch_single_click_down)
  PREFS_MACRO(PREF_KEY_QUICK_LAUNCH_COMBO_BACK_UP, s_quick_launch_combo_back_up)
  PREFS_MACRO(PREF_KEY_QUICK_LAUNCH_COMBO_UP_DOWN, s_quick_launch_combo_up_down)
  PREFS_MACRO(PREF_KEY_QUICK_LAUNCH_SETUP_OPENED, s_quick_launch_setup_opened)
  PREFS_MACRO(PREF_KEY_DEFAULT_WATCHFACE, s_default_watchface)
  PREFS_MACRO(PREF_KEY_WELCOME_VERSION, s_welcome_version)
#ifdef CONFIG_ORIENTATION_MANAGER
  PREFS_MACRO(PREF_KEY_DISPLAY_ORIENTATION_LEFT_HANDED, s_display_orientation_left)
#endif
  PREFS_MACRO(PREF_KEY_ACTIVITY_PREFERENCES, s_activity_preferences)
  PREFS_MACRO(PREF_KEY_ACTIVITY_ACTIVATED_TIMESTAMP, s_activity_activation_timestamp)
  PREFS_MACRO(PREF_KEY_ACTIVITY_ACTIVATION_DELAY_INSIGHT, s_activity_activation_delay_insight)
  PREFS_MACRO(PREF_KEY_ACTIVITY_HEALTH_APP_OPENED, s_activity_prefs_health_app_opened)
  PREFS_MACRO(PREF_KEY_ACTIVITY_WORKOUT_APP_OPENED, s_activity_prefs_workout_app_opened)
  PREFS_MACRO(PREF_KEY_ALARMS_APP_OPENED, s_alarms_app_opened)
  PREFS_MACRO(PREF_KEY_ACTIVITY_HRM_PREFERENCES, s_activity_hrm_preferences)
  PREFS_MACRO(PREF_KEY_ACTIVITY_HEART_RATE_PREFERENCES, s_activity_hr_preferences)
  PREFS_MACRO(PREF_KEY_TIMELINE_SETTINGS_OPENED, s_timeline_settings_opened)
  PREFS_MACRO(PREF_KEY_TIMELINE_PEEK_ENABLED, s_timeline_peek_enabled)
  PREFS_MACRO(PREF_KEY_TIMELINE_PEEK_BEFORE_TIME_M, s_timeline_peek_before_time_m)
#if TIMELINE_PEEK_WATCHFACE_FIT_SUPPORTED
  PREFS_MACRO(PREF_KEY_TIMELINE_PEEK_WATCHFACE_FIT, s_timeline_peek_unsupported_face_mode)
#endif
  PREFS_MACRO(PREF_KEY_COREDUMP_ON_REQUEST, s_coredump_on_request_enabled)
  PREFS_MACRO(PREF_KEY_ACCEL_SHAKE_LOG_INFO, s_accel_shake_log_info_enabled)
  PREFS_MACRO(PREF_KEY_VIBE_LOG_INFO, s_vibe_log_info_enabled)
  PREFS_MACRO(PREF_KEY_SETTINGS_DBS_COMPACTED_V1, s_settings_dbs_compacted_v1)
  PREFS_MACRO(PREF_KEY_ALS_THRESHOLD_MIGRATED_V1, s_als_threshold_migrated_v1)
  PREFS_MACRO(PREF_KEY_ALS_THRESHOLD_MIGRATED_V2, s_als_threshold_migrated_v2)
#ifdef CONFIG_APP_SCALING
  PREFS_MACRO(PREF_KEY_LEGACY_APP_RENDER_MODE, s_legacy_app_render_mode)
#endif

#ifdef CONFIG_THEMING
  PREFS_MACRO(PREF_KEY_THEME_HIGHLIGHT_COLOR, s_theme_highlight_color)
#endif

  PREFS_MACRO(PREF_KEY_MENU_SCROLL_WRAP_AROUND, s_menu_scroll_wrap_around)
  PREFS_MACRO(PREF_KEY_MENU_SCROLL_VIBE_BEHAVIOR, s_menu_scroll_vibe_behavior)
  PREFS_MACRO(PREF_KEY_MUSIC_SHOW_VOLUME_CONTROLS, s_music_show_volume_controls)
  PREFS_MACRO(PREF_KEY_MUSIC_SHOW_PROGRESS_BAR, s_music_show_progress_bar)
  PREFS_MACRO(PREF_KEY_MUSIC_SHOW_ALBUM_ART, s_music_show_album_art)
