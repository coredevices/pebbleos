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

#include "settings_notifications_private.h"
#include "settings_menu.h"
#include "settings_option_menu.h"
#include "settings_window.h"

#include "applib/event_service_client.h"
#include "applib/fonts/fonts.h"
#include "applib/ui/action_menu_window_private.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/option_menu_window.h"
#include "applib/ui/ui.h"
#include "drivers/battery.h"
#include "kernel/pbl_malloc.h"
#include "popups/notifications/notification_window.h"
#include "services/common/analytics/analytics.h"
#include "services/common/i18n/i18n.h"
#include "services/normal/notifications/alerts_preferences_private.h"
#include "services/normal/notifications/alerts_private.h"
#include "services/normal/vibes/vibe_intensity.h"
#include "shell/prefs.h"
#include "shell/system_theme.h"
#include "system/logging.h"
#include "system/passert.h"
#include "util/size.h"
#include "util/time/time.h"

#include <stdio.h>

// Offset between vibe intensity menu item index and vibe intensity enum values
#define INTENSITY_ROW_OFFSET 1

typedef struct {
  SettingsCallbacks callbacks;
  EventServiceInfo battery_connection_event_info;
} SettingsNotificationsData;

enum NotificationsItem {
  NotificationsItemFilter,
#if !CAPABILITY_HAS_VIBE_SCORES
  NotificationsItemVibration,
#endif
  NotificationsItemTextSize,
  NotificationsItemWindowTimeout,
  NotificationsItemDelayedVibration,
  NotificationsItem_Count,
};

// Filter Alerts
//////////////////////////

#define NUM_ALERT_MODES_IN_LIST 3

// These aren't all of the values of AlertMask, so to add extra ones you have to update both of
// these arrays

static const AlertMask s_alert_mode_values[NUM_ALERT_MODES_IN_LIST] = {
  AlertMaskAllOn,
  AlertMaskPhoneCalls,
  AlertMaskAllOff,
};

static const char *s_alert_mode_labels[NUM_ALERT_MODES_IN_LIST] = {
  i18n_noop("Allow All Notifications"),
  i18n_noop("Allow Phone Calls Only"),
  i18n_noop("Mute All Notifications"),
};

static const char *prv_alert_mask_to_label(AlertMask mask) {
  for (uint32_t i = 0; i < NUM_ALERT_MODES_IN_LIST; i++) {
    if (s_alert_mode_values[i] == mask) {
      return s_alert_mode_labels[i];
    }
  }
  return "???";
}

static void prv_filter_menu_select(OptionMenu *option_menu, int selection, void *context) {
  alerts_set_mask(s_alert_mode_values[selection]);
  app_window_stack_remove(&option_menu->window, true /* animated */);
}

static void prv_filter_menu_push(SettingsNotificationsData *data) {
  AlertMask mask = alerts_get_mask();
  size_t cycle_len = ARRAY_LENGTH(s_alert_mode_values);
  size_t index = 0;
  // TODO PBL-24306: update once AlertMask logic is made safer
  for (size_t i = 0; i < cycle_len; i++) {
    if (s_alert_mode_values[i] == mask) {
      index = i;
      break;
    }
  }
  const OptionMenuCallbacks callbacks = {
    .select = prv_filter_menu_select,
  };
  /// The option in the Settings app for filtering notifications by type.
  const char *title = i18n_noop("Filter");
  settings_option_menu_push(
      title, OptionMenuContentType_DoubleLine, index, &callbacks, cycle_len,
      true /* icons_enabled */, s_alert_mode_labels, data);
}

// Vibe Settings (If vibes scores disabled for this model)
//////////////////////////
#if !CAPABILITY_HAS_VIBE_SCORES
static const char *strings_for_vibe_intensities[] = {
  i18n_ctx_noop("NotifVibe", "Disabled"),
  i18n_ctx_noop("NotifVibe", "Low"),
  i18n_ctx_noop("NotifVibe", "Medium"),
  i18n_ctx_noop("NotifVibe", "High")
};

static void prv_vibe_menu_select(OptionMenu *option_menu, int selection, void *context) {
  const bool enable_vibration = (selection != 0);
  const VibeIntensity new_vibe_intensity = enable_vibration ? (selection - INTENSITY_ROW_OFFSET) :
                                           DEFAULT_VIBE_INTENSITY;

  alerts_set_vibrate(enable_vibration);
  alerts_preferences_set_vibe_intensity(new_vibe_intensity);
  vibe_intensity_set(new_vibe_intensity);

  if (enable_vibration) {
    vibes_short_pulse();
  }

  app_window_stack_remove(&option_menu->window, true /* animated */);
}

static void prv_vibe_menu_push(SettingsNotificationsData *data) {
  const OptionMenuCallbacks callbacks = {
    .select = prv_vibe_menu_select,
  };
  /// The option in the Settings app for choosing a vibration intensity for notifications.
  const char *title = i18n_noop("Vibration");
  uint32_t selected = vibe_intensity_get() + INTENSITY_ROW_OFFSET;
  if (!alerts_get_vibrate()) {
    selected = 0;
  }
  settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, selected, &callbacks,
      ARRAY_LENGTH(strings_for_vibe_intensities), true /* icons_enabled */,
      strings_for_vibe_intensities, data);
}
#endif  /* !CAPABILITY_HAS_VIBE_SCORES */

// Text Size
////////////////////////

static const char *s_text_size_names[] = {
  [SettingsContentSize_Small]   = i18n_noop("Smaller"),
  [SettingsContentSize_Default] = i18n_noop("Default"),
  [SettingsContentSize_Large]   = i18n_noop("Larger"),
};

static void prv_text_size_menu_select(OptionMenu *option_menu, int selection, void *context) {
  system_theme_set_content_size(settings_content_size_to_preferred_size(selection));
  app_window_stack_remove(&option_menu->window, true /* animated */);
}

static void prv_text_size_menu_push(SettingsNotificationsData *data) {
  const OptionMenuCallbacks callbacks = {
    .select = prv_text_size_menu_select,
  };
  /// The option in the Settings app for choosing the text size of notifications.
  const char *title = i18n_noop("Text Size");
  const SettingsContentSize index =
      settings_content_size_from_preferred_size(system_theme_get_content_size());
  settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, index, &callbacks, SettingsContentSizeCount,
      true /* icons_enabled */, s_text_size_names, data);
}

// Window Timeout
////////////////////////

// NOTE: Keep the following two arrays in sync and with the same size.
static const uint32_t s_window_timeouts_ms[] = {
  15 * MS_PER_SECOND,
  1  * MS_PER_MINUTE,
  NOTIF_WINDOW_TIMEOUT_DEFAULT,
  10 * MS_PER_MINUTE,
  NOTIF_WINDOW_TIMEOUT_INFINITE
};

static const char *s_window_timeouts_labels[] = {
  /// 15 Second Notification Window Timeout
  i18n_noop("15 Seconds"),
  /// 1 Minute Notification Window Timeout
  i18n_noop("1 Minute"),
  /// 3 Minute Notification Window Timeout
  i18n_noop("3 Minutes"),
  /// 10 Minute Notification Window Timeout
  i18n_noop("10 Minutes"),
  /// No Notification Window Timeout
  i18n_noop("None"),
};

_Static_assert(ARRAY_LENGTH(s_window_timeouts_ms) == ARRAY_LENGTH(s_window_timeouts_labels), "");

static int prv_window_timeout_get_selection_index(void) {
  const int DEFAULT_IDX = 2;
  // Double check no one has fudged with the order and the fallback/default
  PBL_ASSERTN(s_window_timeouts_ms[DEFAULT_IDX] == NOTIF_WINDOW_TIMEOUT_DEFAULT);

  const uint32_t timeout_ms = alerts_preferences_get_notification_window_timeout_ms();
  for (size_t i = 0; i < ARRAY_LENGTH(s_window_timeouts_ms); i++) {
    if (s_window_timeouts_ms[i] == timeout_ms) {
      return i;
    }
  }
  // Should never happen (only should happen if we remove a timeout and don't migrate the user
  // to a new setting automatically
  return DEFAULT_IDX;
}

static void prv_window_timeout_menu_select(OptionMenu *option_menu, int selection, void *context) {
  alerts_preferences_set_notification_window_timeout_ms(s_window_timeouts_ms[selection]);
  app_window_stack_remove(&option_menu->window, true /* animated */);
}

static void prv_window_timeout_menu_push(SettingsNotificationsData *data) {
  const int index = prv_window_timeout_get_selection_index();
  const OptionMenuCallbacks callbacks = {
    .select = prv_window_timeout_menu_select,
  };
  /// Status bar title for the Notification Window Timeout settings screen
  const char *title = i18n_noop("Timeout");
  settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, index, &callbacks,
      ARRAY_LENGTH(s_window_timeouts_labels), true /* icons_enabled */, s_window_timeouts_labels,
      data);
}

// Vibration Timing
////////////////////////

static const char *s_vibration_timing_options[] = {
  i18n_noop("Immediately"),
  i18n_noop("After animation")
};

static void prv_delayed_vibration_menu_select(OptionMenu *option_menu, int selection, void *context) {
  alerts_set_delayed_vibration(selection == 1);  // 0 = Immediately, 1 = After animation
  app_window_stack_remove(&option_menu->window, true /* animated */);
}

static void prv_delayed_vibration_menu_push(SettingsNotificationsData *data) {
  const OptionMenuCallbacks callbacks = {
    .select = prv_delayed_vibration_menu_select,
  };
  
  const char *title = i18n_noop("Vibrate...");
  const int index = alerts_get_delayed_vibration() ? 1 : 0;
  
  settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, index, &callbacks,
      ARRAY_LENGTH(s_vibration_timing_options), true /* icons_enabled */, 
      s_vibration_timing_options, data);
}

// Menu Layer Callbacks
////////////////////////

static uint16_t prv_num_rows_cb(SettingsCallbacks *context) {
  return NotificationsItem_Count;
}

static void prv_draw_row_cb(SettingsCallbacks *context, GContext *ctx,
                            const Layer *cell_layer, uint16_t row, bool selected) {
  SettingsNotificationsData *data = ((SettingsOptionMenuData *)context)->context;
  const char *subtitle = NULL;
  const char *title = NULL;

  switch (row) {
    case NotificationsItemFilter:
      title = i18n_noop("Filter");
      subtitle = prv_alert_mask_to_label(alerts_get_mask());
      break;
#if !CAPABILITY_HAS_VIBE_SCORES
    case NotificationsItemVibration:
      title = i18n_noop("Vibration");
      if (battery_is_usb_connected()) {
        subtitle = i18n_noop("Disabled (Plugged In)");
      } else if (alerts_get_vibrate()) {
        subtitle = strings_for_vibe_intensities[vibe_intensity_get() + INTENSITY_ROW_OFFSET];
      } else {
        subtitle = strings_for_vibe_intensities[0];
      }
      break;
#endif  /* !CAPABILITY_HAS_VIBE_SCORES */
    case NotificationsItemTextSize: {
      /// String within Settings->Notifications that describes the text font size
      title = i18n_noop("Text Size");
      const SettingsContentSize index =
          settings_content_size_from_preferred_size(system_theme_get_content_size());
      subtitle = (index < SettingsContentSizeCount) ? s_text_size_names[index] : "";
      break;
    }
    case NotificationsItemWindowTimeout: {
      /// String within Settings->Notifications that describes the window timeout setting
      title = i18n_noop("Timeout");
      subtitle = s_window_timeouts_labels[prv_window_timeout_get_selection_index()];
      break;
    }
    case NotificationsItemDelayedVibration: {
      title = i18n_noop("Vibrate...");
      subtitle = alerts_get_delayed_vibration() ? i18n_noop("After animation") : i18n_noop("Immediately");
      break;
    }
    default:
      WTF;
  }

  menu_cell_basic_draw(ctx, cell_layer, i18n_get(title, data), i18n_get(subtitle, data), NULL);
}

static void prv_deinit_cb(SettingsCallbacks *context) {
  SettingsNotificationsData *data = (SettingsNotificationsData *)context;
  i18n_free_all(data);
  app_free(data);
}

static void prv_select_click_cb(SettingsCallbacks *context, uint16_t row) {
  SettingsNotificationsData *data = (SettingsNotificationsData *) context;

  switch (row) {
    case NotificationsItemFilter:
      prv_filter_menu_push(data);
      break;
#if !CAPABILITY_HAS_VIBE_SCORES
    case NotificationsItemVibration:
      if (battery_is_usb_connected()) {
        return;
      }
      prv_vibe_menu_push(data);
      break;
#endif  /* !CAPABILITY_HAS_VIBE_SCORES */
    case NotificationsItemTextSize:
      prv_text_size_menu_push(data);
      break;
    case NotificationsItemWindowTimeout:
      prv_window_timeout_menu_push(data);
      break;
    case NotificationsItemDelayedVibration:
      prv_delayed_vibration_menu_push(data);
      break;
    default:
      WTF;
  }
  settings_menu_reload_data(SettingsMenuItemNotifications);
}

static void prv_settings_notifications_event_handler(PebbleEvent *event, void *context) {
  switch (event->type) {
    case PEBBLE_BATTERY_CONNECTION_EVENT:
      // Redraw the menu so that the Vibration status will be re-rendered.
      settings_menu_mark_dirty(SettingsMenuItemNotifications);
      break;
    default:
      break;
  }
}

static void prv_expand_cb(SettingsCallbacks *context) {
  SettingsNotificationsData *data = (SettingsNotificationsData *) context;

  data->battery_connection_event_info = (EventServiceInfo) {
    .type = PEBBLE_BATTERY_CONNECTION_EVENT,
    .handler = prv_settings_notifications_event_handler,
  };
  event_service_client_subscribe(&data->battery_connection_event_info);

}

static void prv_hide_cb(SettingsCallbacks *context) {
  SettingsNotificationsData *data = (SettingsNotificationsData *) context;

  event_service_client_unsubscribe(&data->battery_connection_event_info);
}

static Window *prv_init(void) {
  SettingsNotificationsData* data = app_malloc_check(sizeof(*data));
  *data = (SettingsNotificationsData){};

  data->callbacks = (SettingsCallbacks) {
    .deinit = prv_deinit_cb,
    .draw_row = prv_draw_row_cb,
    .select_click = prv_select_click_cb,
    .num_rows = prv_num_rows_cb,
    .expand = prv_expand_cb,
    .hide = prv_hide_cb,
  };

  return settings_window_create(SettingsMenuItemNotifications, &data->callbacks);
}

const SettingsModuleMetadata *settings_notifications_get_info(void) {
  static const SettingsModuleMetadata s_module_info = {
    .name = i18n_noop("Notifications"),
    .init = prv_init,
  };

  return &s_module_info;
}

void analytics_external_collect_notification_settings(void) {
  const uint8_t strength = get_strength_for_intensity(vibe_intensity_get());
  analytics_set(ANALYTICS_DEVICE_METRIC_SETTING_VIBRATION_STRENGTH,
                strength, AnalyticsClient_System);
}
