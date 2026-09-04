/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "menu.h"
#include "option_menu.h"
#include "window.h"

#include "applib/event_service_client.h"
#include "applib/ui/action_menu_window_private.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/option_menu_window.h"
#include "applib/ui/ui.h"
#include "kernel/pbl_malloc.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/notifications/alerts_preferences_private.h"
#include "pbl/services/notifications/alerts_private.h"
#include "system/passert.h"
#include "pbl/util/size.h"
#include "util/time/time.h"

// Offset between vibe intensity menu item index and vibe intensity enum values
#define INTENSITY_ROW_OFFSET 1

typedef struct {
  SettingsCallbacks callbacks;
  EventServiceInfo battery_connection_event_info;
} SettingsNotificationsData;

enum NotificationsItem {
  NotificationsItemFilter,
  NotificationsItemWindowTimeout,
#if PBL_BW
  NotificationsItemDesignStyle,
#endif
  NotificationsItemVibeDelay,
  NotificationsItemBacklight,
  NotificationsItemStatusBarStyle,
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

// Window Timeout
////////////////////////

// NOTE: Keep the following two arrays in sync and with the same size.
static const uint32_t s_window_timeouts_ms[] = {
  15 * MS_PER_SECOND,
  30 * MS_PER_SECOND,
  1  * MS_PER_MINUTE,
  NOTIF_WINDOW_TIMEOUT_DEFAULT,
  10 * MS_PER_MINUTE,
  NOTIF_WINDOW_TIMEOUT_INFINITE
};

static const char *s_window_timeouts_labels[] = {
  /// 15 Second Notification Window Timeout
  i18n_noop("15 Seconds"),
  /// 30 Second Notification Window Timeout
  i18n_noop("30 Seconds"),
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
  const int DEFAULT_IDX = 3;
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

// Design Style
////////////////////////

#if PBL_BW
static const char *s_design_style_labels[] = {
  /// Standard notification design option (default)
  i18n_noop("Classic"),
  /// Alternative notification design option
  i18n_noop("Flat Black"),
};

static int prv_design_style_get_selection_index(void) {
  return alerts_preferences_get_notification_alternative_design() ? 1 : 0;
}

static void prv_design_style_menu_select(OptionMenu *option_menu, int selection, void *context) {
  alerts_preferences_set_notification_alternative_design(selection == 1);
  app_window_stack_remove(&option_menu->window, true /* animated */);
}

static void prv_design_style_menu_push(SettingsNotificationsData *data) {
  const int index = prv_design_style_get_selection_index();
  const OptionMenuCallbacks callbacks = {
    .select = prv_design_style_menu_select,
  };
  /// Status bar title for the Notification Design Style settings screen
  const char *title = i18n_noop("Banner Style");
  settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, index, &callbacks,
      ARRAY_LENGTH(s_design_style_labels), true /* icons_enabled */, s_design_style_labels,
      data);
}
#endif /* PBL_BW */

// Vibe Delay
////////////////////////

static const char *s_vibe_delay_labels[] = {
  /// Vibrate at the beginning of notification animation (immediate)
  i18n_noop("Beginning"),
  /// Vibrate at the end of notification animation (delayed)
  i18n_noop("End"),
};

static int prv_vibe_delay_get_selection_index(void) {
  return alerts_preferences_get_notification_vibe_delay() ? 1 : 0;
}

static void prv_vibe_delay_menu_select(OptionMenu *option_menu, int selection, void *context) {
  alerts_preferences_set_notification_vibe_delay(selection == 1);
  app_window_stack_remove(&option_menu->window, true /* animated */);
}

static void prv_vibe_delay_menu_push(SettingsNotificationsData *data) {
  const int index = prv_vibe_delay_get_selection_index();
  const OptionMenuCallbacks callbacks = {
    .select = prv_vibe_delay_menu_select,
  };
  /// Status bar title for the Notification Vibe Timing settings screen
  const char *title = i18n_noop("Vibe Timing");
  settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, index, &callbacks,
      ARRAY_LENGTH(s_vibe_delay_labels), true /* icons_enabled */, s_vibe_delay_labels,
      data);
}

// Status Bar Style
////////////////////////

static const char *s_status_bar_style_labels[] = {
  [NotificationStatusBarStyle_Default]   = i18n_ctx_noop("StatusBar", "Default"),
  [NotificationStatusBarStyle_Bold]      = i18n_noop("Bold"),
  [NotificationStatusBarStyle_LargeBold] = i18n_noop("Big & Bold"),
};

_Static_assert(ARRAY_LENGTH(s_status_bar_style_labels) == NotificationStatusBarStyleCount, "");

static int prv_status_bar_style_get_selection_index(void) {
  const NotificationStatusBarStyle style =
      alerts_preferences_get_notification_status_bar_style();
  return (style < NotificationStatusBarStyleCount) ? (int)style : 0;
}

static void prv_status_bar_style_menu_select(OptionMenu *option_menu, int selection,
                                             void *context) {
  alerts_preferences_set_notification_status_bar_style((NotificationStatusBarStyle)selection);
  app_window_stack_remove(&option_menu->window, true /* animated */);
}

static void prv_status_bar_style_menu_push(SettingsNotificationsData *data) {
  const int index = prv_status_bar_style_get_selection_index();
  const OptionMenuCallbacks callbacks = {
    .select = prv_status_bar_style_menu_select,
  };
  /// Status bar title for the Notification Status Bar Style settings screen
  const char *title = i18n_noop("Clock Style");
  settings_option_menu_push(
      title, OptionMenuContentType_SingleLine, index, &callbacks,
      ARRAY_LENGTH(s_status_bar_style_labels), true /* icons_enabled */,
      s_status_bar_style_labels, data);
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
    case NotificationsItemWindowTimeout: {
      /// String within Settings->Notifications that describes the window timeout setting
      title = i18n_noop("Timeout");
      subtitle = s_window_timeouts_labels[prv_window_timeout_get_selection_index()];
      break;
    }
  #if PBL_BW
    case NotificationsItemDesignStyle: {
      /// String within Settings->Notifications that describes the notification design style
      title = i18n_noop("Banner Style");
      subtitle = s_design_style_labels[prv_design_style_get_selection_index()];
      break;
    }
  #endif /* PBL_BW */
    case NotificationsItemVibeDelay: {
      /// String within Settings->Notifications that describes when vibration happens
      title = i18n_noop("Vibe Timing");
      subtitle = s_vibe_delay_labels[prv_vibe_delay_get_selection_index()];
      break;
    }
    case NotificationsItemBacklight: {
      /// String within Settings->Notifications that describes backlight setting
      title = i18n_noop("Backlight");
      subtitle = alerts_preferences_get_notification_backlight() ?
                 i18n_noop("On") : i18n_noop("Off");
      break;
    }
    case NotificationsItemStatusBarStyle: {
      /// String within Settings->Notifications that selects the notification status bar style
      title = i18n_noop("Status Bar");
      subtitle = s_status_bar_style_labels[prv_status_bar_style_get_selection_index()];
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
    case NotificationsItemWindowTimeout:
      prv_window_timeout_menu_push(data);
      break;
#if PBL_BW
    case NotificationsItemDesignStyle:
      prv_design_style_menu_push(data);
      break;
#endif /* PBL_BW */
    case NotificationsItemVibeDelay:
      prv_vibe_delay_menu_push(data);
      break;
    case NotificationsItemBacklight:
      // Toggle backlight directly without submenu
      alerts_preferences_set_notification_backlight(
          !alerts_preferences_get_notification_backlight());
      break;
    case NotificationsItemStatusBarStyle:
      prv_status_bar_style_menu_push(data);
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
