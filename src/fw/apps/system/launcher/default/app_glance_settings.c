/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "app_glance_settings.h"

#include "app_glance_structured.h"
#include "menu_layer.h"

#include "applib/battery_state_service.h"
#include "applib/graphics/gpath.h"
#include "apps/system/timeline/text_node.h"
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "process_management/app_install_manager.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/battery/battery_state.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/services/bluetooth/ble_hrm.h"
#include "pbl/services/notifications/alerts_private.h"
#include "pbl/services/notifications/do_not_disturb.h"
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/size.h"
#include "pbl/util/struct.h"

#include <stdio.h>

// These dimensions are separate defines so we can use them to statically define the battery points
#define BATTERY_SILHOUETTE_ICON_WIDTH (16)
#define BATTERY_SILHOUETTE_ICON_HEIGHT (9)

typedef struct LauncherAppGlanceSettingsState {
  BatteryChargeState battery_charge_state;
  bool is_pebble_app_connected;
  bool is_airplane_mode_enabled;
  bool is_quiet_time_enabled;
#ifdef CONFIG_HRM
  bool is_sharing_hrm;
#endif
} LauncherAppGlanceSettingsState;

typedef struct LauncherAppGlanceSettings {
  char title[APP_NAME_SIZE_BYTES];
  char battery_percent_text[5]; //!< longest string is "100%" (4 characters + 1 for NULL terminator)
  KinoReel *icon;
  uint32_t icon_resource_id;
  KinoReel *charging_indicator_icon;
  uint8_t subtitle_font_height;
  LauncherAppGlanceSettingsState glance_state;
  EventServiceInfo battery_state_event_info;
  EventServiceInfo pebble_app_event_info;
  EventServiceInfo airplane_mode_event_info;
  EventServiceInfo quiet_time_event_info;
#ifdef CONFIG_HRM
  EventServiceInfo hrm_sharing_event_info;
#endif
} LauncherAppGlanceSettings;

static KinoReel *prv_get_icon(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceSettings *settings_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  return NULL_SAFE_FIELD_ACCESS(settings_glance, icon, NULL);
}

static const char *prv_get_title(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceSettings *settings_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  return NULL_SAFE_FIELD_ACCESS(settings_glance, title, NULL);
}

static void prv_charging_icon_node_draw_cb(GContext *ctx, const GRect *rect,
                                           PBL_UNUSED const GTextNodeDrawConfig *config, bool render,
                                           GSize *size_out, void *user_data) {
  LauncherAppGlanceStructured *structured_glance = user_data;
  LauncherAppGlanceSettings *settings_glance =
      launcher_app_glance_structured_get_data(structured_glance);

  KinoReel *charging_indicator_icon = NULL_SAFE_FIELD_ACCESS(settings_glance,
                                                             charging_indicator_icon, NULL);
  PBL_ASSERTN(charging_indicator_icon);


  if (render && charging_indicator_icon) {
    launcher_app_glance_structured_draw_icon(structured_glance, ctx, charging_indicator_icon,
                                             rect->origin);
  }

  if (size_out) {
    *size_out = GSize(kino_reel_get_size(charging_indicator_icon).w,
                      settings_glance->subtitle_font_height);
  }
}

static void prv_battery_icon_node_draw_cb(GContext *ctx, const GRect *rect,
                                          PBL_UNUSED const GTextNodeDrawConfig *config, bool render,
                                          GSize *size_out, void *user_data) {
  LauncherAppGlanceStructured *structured_glance = user_data;
  LauncherAppGlanceSettings *settings_glance =
      launcher_app_glance_structured_get_data(structured_glance);

  const GSize battery_silhouette_icon_size = GSize(BATTERY_SILHOUETTE_ICON_WIDTH,
                                                   BATTERY_SILHOUETTE_ICON_HEIGHT);

  if (render) {
    // This points array is static to help conserve stack usage
    static const GPoint s_battery_silhouette_path_points[] = {
      {0, 0},
      {BATTERY_SILHOUETTE_ICON_WIDTH - 1, 0},
      {BATTERY_SILHOUETTE_ICON_WIDTH - 1, 1},
      {BATTERY_SILHOUETTE_ICON_WIDTH + 1, 2},
      {BATTERY_SILHOUETTE_ICON_WIDTH + 1, BATTERY_SILHOUETTE_ICON_HEIGHT - 3},
      {BATTERY_SILHOUETTE_ICON_WIDTH - 1, BATTERY_SILHOUETTE_ICON_HEIGHT - 3},
      {BATTERY_SILHOUETTE_ICON_WIDTH - 1, BATTERY_SILHOUETTE_ICON_HEIGHT - 1},
      {0, BATTERY_SILHOUETTE_ICON_HEIGHT - 1},
    };
    GPath battery_silhouette_path = (GPath) {
      .num_points = ARRAY_LENGTH(s_battery_silhouette_path_points),
      .points = (GPoint *)s_battery_silhouette_path_points,
      .offset = rect->origin,
    };

    const GColor battery_silhouette_color =
        launcher_app_glance_structured_get_highlight_color(structured_glance);
    const GColor battery_fill_color =
        PBL_IF_COLOR_ELSE(gcolor_legible_over(battery_silhouette_color), GColorWhite);

    graphics_context_set_fill_color(ctx, battery_silhouette_color);

    // Draw the battery silhouette
    const GRect battery_silhouette_frame = (GRect) {
      .origin = rect->origin,
      .size = battery_silhouette_icon_size,
    };
    gpath_draw_filled(ctx, &battery_silhouette_path);

    // Inset the filled area
    GRect battery_fill_rect = grect_inset_internal(battery_silhouette_frame, 3, 2);
#if !PBL_COLOR
    // Fill the battery silhouette all the way for B&W, in order to make the BG black always.
    graphics_context_set_fill_color(ctx, GColorBlack);
    graphics_fill_rect(ctx, &battery_fill_rect);
#endif

    // Adjust fill width for charge percentage, never filling below 10%
    uint8_t clipped_charge_percent =
        settings_glance->glance_state.battery_charge_state.charge_percent;
    clipped_charge_percent = CLIP(clipped_charge_percent, (uint8_t)10, (uint8_t)100);
    battery_fill_rect.size.w = battery_fill_rect.size.w * clipped_charge_percent / (int16_t)100;
    // Fill the battery silhouette based on the charge percent
    graphics_context_set_fill_color(ctx, battery_fill_color);
    graphics_fill_rect(ctx, &battery_fill_rect);
  }

  if (size_out) {
    *size_out = GSize(battery_silhouette_icon_size.w, settings_glance->subtitle_font_height);
  }
}

static void prv_battery_percent_dynamic_text_node_update(
    PBL_UNUSED GContext *ctx, PBL_UNUSED GTextNode *node, PBL_UNUSED const GRect *box,
    PBL_UNUSED const GTextNodeDrawConfig *config, PBL_UNUSED bool render, char *buffer, size_t buffer_size,
    void *user_data) {
  LauncherAppGlanceStructured *structured_glance = user_data;
  LauncherAppGlanceSettings *settings_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  if (settings_glance) {
    buffer_size = MIN(sizeof(settings_glance->battery_percent_text), buffer_size);
    strncpy(buffer, settings_glance->battery_percent_text, buffer_size);
    buffer[buffer_size - 1] = '\0';
  }
}

static GTextNode *prv_wrap_text_node_in_vertically_centered_container(GTextNode *node) {
  const size_t max_vertical_container_nodes = 1;
  GTextNodeVertical *vertical_container_node =
      graphics_text_node_create_vertical(max_vertical_container_nodes);
  vertical_container_node->vertical_alignment = GVerticalAlignmentCenter;

  graphics_text_node_container_add_child(&vertical_container_node->container, node);

  return &vertical_container_node->container.node;
}

static GTextNode *prv_create_subtitle_node(LauncherAppGlanceStructured *structured_glance) {
  PBL_ASSERTN(structured_glance);
  LauncherAppGlanceSettings *settings_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  PBL_ASSERTN(settings_glance);

  // Battery text (if not plugged in), battery icon, and (if plugged in) a lightning bolt icon
  const size_t max_horizontal_nodes = 3;
  GTextNodeHorizontal *horizontal_container_node =
      graphics_text_node_create_horizontal(max_horizontal_nodes);
  horizontal_container_node->horizontal_alignment = GTextAlignmentLeft;

  if (!settings_glance->glance_state.battery_charge_state.is_plugged) {
    GTextNode *battery_percent_text_node =
        launcher_app_glance_structured_create_subtitle_text_node(
            structured_glance, prv_battery_percent_dynamic_text_node_update);
    // Achieves the design spec'd 6 px horizontal spacing b/w the percent text and battery icon
    battery_percent_text_node->margin.w = 4;
    GTextNode *vertically_centered_battery_percent_text_node =
        prv_wrap_text_node_in_vertically_centered_container(battery_percent_text_node);
    graphics_text_node_container_add_child(&horizontal_container_node->container,
                                           vertically_centered_battery_percent_text_node);
  }

#if PBL_DISPLAY_HEIGHT >= 200
  const int16_t subtitle_icon_offset_y = 5;
#else
  const int16_t subtitle_icon_offset_y = 2;
#endif

  GTextNodeCustom *battery_icon_node =
      graphics_text_node_create_custom(prv_battery_icon_node_draw_cb, structured_glance);
  // Push the battery icon down to center it properly
  battery_icon_node->node.offset.y += subtitle_icon_offset_y;

  // Achieves the design spec'd 6 px horizontal spacing b/w the battery icon and charging icon
  battery_icon_node->node.margin.w = 7;
  GTextNode *vertically_centered_battery_icon_node =
      prv_wrap_text_node_in_vertically_centered_container(&battery_icon_node->node);
  graphics_text_node_container_add_child(&horizontal_container_node->container,
                                         vertically_centered_battery_icon_node);

  if (settings_glance->glance_state.battery_charge_state.is_plugged) {
    GTextNodeCustom *charging_icon_node =
        graphics_text_node_create_custom(prv_charging_icon_node_draw_cb, structured_glance);
    // Push the charging icon down to center it properly
    charging_icon_node->node.offset.y += subtitle_icon_offset_y;
    GTextNode *vertically_centered_charging_icon_node =
        prv_wrap_text_node_in_vertically_centered_container(&charging_icon_node->node);
    graphics_text_node_container_add_child(&horizontal_container_node->container,
                                           vertically_centered_charging_icon_node);
  }

  return &horizontal_container_node->container.node;
}

static void prv_destructor(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceSettings *settings_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  if (settings_glance) {
    event_service_client_unsubscribe(&settings_glance->battery_state_event_info);
    event_service_client_unsubscribe(&settings_glance->pebble_app_event_info);
    event_service_client_unsubscribe(&settings_glance->airplane_mode_event_info);
    event_service_client_unsubscribe(&settings_glance->quiet_time_event_info);
#ifdef CONFIG_HRM
    event_service_client_unsubscribe(&settings_glance->hrm_sharing_event_info);
#endif
    kino_reel_destroy(settings_glance->icon);
    kino_reel_destroy(settings_glance->charging_indicator_icon);
  }
  app_free(settings_glance);
}

static void prv_set_glance_icon(LauncherAppGlanceSettings *settings_glance,
                                uint32_t new_icon_resource_id) {
  if (settings_glance->icon_resource_id == new_icon_resource_id) {
    // Nothing to do, bail out
    return;
  }

  // Destroy the existing icon
  kino_reel_destroy(settings_glance->icon);

  // Set the new icon and record its resource ID
  settings_glance->icon = kino_reel_create_with_resource(new_icon_resource_id);
  PBL_ASSERTN(settings_glance->icon);
  settings_glance->icon_resource_id = new_icon_resource_id;
}

static bool prv_mute_notifications_allow_calls_only(void) {
  return (alerts_get_mask() == AlertMaskPhoneCalls);
}

static uint32_t prv_get_resource_id_for_connectivity_status(
    LauncherAppGlanceSettings *settings_glance) {
#ifdef CONFIG_HRM
  if (settings_glance->glance_state.is_sharing_hrm) {
    return RESOURCE_ID_CONNECTIVITY_SHARING_HRM;
  }
#endif
  if (settings_glance->glance_state.is_airplane_mode_enabled) {
    return RESOURCE_ID_CONNECTIVITY_BLUETOOTH_AIRPLANE_MODE;
  } else if (!settings_glance->glance_state.is_pebble_app_connected) {
    return RESOURCE_ID_CONNECTIVITY_BLUETOOTH_DISCONNECTED;
  } else if (settings_glance->glance_state.is_quiet_time_enabled) {
    return RESOURCE_ID_CONNECTIVITY_BLUETOOTH_DND;
  } else if (prv_mute_notifications_allow_calls_only()) {
    return RESOURCE_ID_CONNECTIVITY_BLUETOOTH_CALLS_ONLY;
  } else if (settings_glance->glance_state.is_pebble_app_connected) {
    return RESOURCE_ID_CONNECTIVITY_BLUETOOTH_CONNECTED;
  } else {
    WTF;
  }
}

static void prv_refresh_glance_content(LauncherAppGlanceSettings *settings_glance) {
  // Update the battery percent text in the glance
  const size_t battery_percent_text_size = sizeof(settings_glance->battery_percent_text);
  snprintf(settings_glance->battery_percent_text, battery_percent_text_size, "%"PRIu8"%%",
           settings_glance->glance_state.battery_charge_state.charge_percent);

  // Update the icon
  const uint32_t new_icon_resource_id =
      prv_get_resource_id_for_connectivity_status(settings_glance);
  prv_set_glance_icon(settings_glance, new_icon_resource_id);
}

static bool prv_is_pebble_app_connected(void) {
  return (comm_session_get_system_session() != NULL);
}

static void prv_event_handler(PebbleEvent *event, void *context) {
  LauncherAppGlanceStructured *structured_glance = context;
  PBL_ASSERTN(structured_glance);

  LauncherAppGlanceSettings *settings_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  PBL_ASSERTN(settings_glance);

  switch (event->type) {
    case PEBBLE_BATTERY_STATE_CHANGE_EVENT:
      settings_glance->glance_state.battery_charge_state = battery_state_service_peek();
      break;
    case PEBBLE_COMM_SESSION_EVENT:
      if (event->bluetooth.comm_session_event.is_system) {
        settings_glance->glance_state.is_pebble_app_connected =
            event->bluetooth.comm_session_event.is_open;
      }
      break;
    case PEBBLE_BT_STATE_EVENT:
      settings_glance->glance_state.is_airplane_mode_enabled = bt_ctl_is_airplane_mode_on();
      break;
    case PEBBLE_DO_NOT_DISTURB_EVENT:
      settings_glance->glance_state.is_quiet_time_enabled = do_not_disturb_is_active();
      break;
#ifdef CONFIG_HRM
    case PEBBLE_BLE_HRM_SHARING_STATE_UPDATED_EVENT: {
      const bool prev_is_sharing = settings_glance->glance_state.is_sharing_hrm;
      const bool is_sharing = (event->bluetooth.le.hrm_sharing_state.subscription_count > 0);
      if (prev_is_sharing == is_sharing) {
        return;
      }
      settings_glance->glance_state.is_sharing_hrm = is_sharing;
      break;
    }
#endif
    default:
      WTF;
  }

  // Refresh the content in the glance
  prv_refresh_glance_content(settings_glance);

  // Broadcast to the service that we changed the glance
  launcher_app_glance_structured_notify_service_glance_changed(structured_glance);
}

static void prv_subscribe_to_event(EventServiceInfo *event_service_info, PebbleEventType type,
                                   LauncherAppGlanceStructured *structured_glance) {
  PBL_ASSERTN(event_service_info);

  *event_service_info = (EventServiceInfo) {
    .type = type,
    .handler = prv_event_handler,
    .context = structured_glance,
  };

  event_service_client_subscribe(event_service_info);
}

static const LauncherAppGlanceStructuredImpl s_settings_structured_glance_impl = {
  .get_icon = prv_get_icon,
  .get_title = prv_get_title,
  .create_subtitle_node = prv_create_subtitle_node,
  .destructor = prv_destructor,
};

LauncherAppGlance *launcher_app_glance_settings_create(const AppMenuNode *node) {
  PBL_ASSERTN(node);

  LauncherAppGlanceSettings *settings_glance = app_zalloc_check(sizeof(*settings_glance));

  // Copy the name of the Settings app as the title
  const size_t title_size = sizeof(settings_glance->title);
  strncpy(settings_glance->title, node->name, title_size);
  settings_glance->title[title_size - 1] = '\0';

  // Load the charging indicator icon
  settings_glance->charging_indicator_icon =
      kino_reel_create_with_resource(RESOURCE_ID_BATTERY_CHARGING_ICON);

  // Cache the subtitle font height for simplifying layout calculations
  settings_glance->subtitle_font_height =
      fonts_get_font_height(fonts_get_system_font(LAUNCHER_MENU_LAYER_SUBTITLE_FONT));

  const bool should_consider_slices = false;
  LauncherAppGlanceStructured *structured_glance =
      launcher_app_glance_structured_create(&node->uuid, &s_settings_structured_glance_impl,
                                            should_consider_slices, settings_glance);
  PBL_ASSERTN(structured_glance);
  // Disable selection animations for the settings glance
  structured_glance->selection_animation_disabled = true;

  // Set the first state of the glance
  settings_glance->glance_state = (LauncherAppGlanceSettingsState) {
    .battery_charge_state = battery_state_service_peek(),
    .is_pebble_app_connected = prv_is_pebble_app_connected(),
    .is_airplane_mode_enabled = bt_ctl_is_airplane_mode_on(),
    .is_quiet_time_enabled = do_not_disturb_is_active(),
#ifdef CONFIG_HRM
    .is_sharing_hrm = ble_hrm_is_sharing(),
#endif
  };

  // Refresh the glance now that we have set the first state of the glance
  prv_refresh_glance_content(settings_glance);

  // Subscribe to the various events we care about
  prv_subscribe_to_event(&settings_glance->battery_state_event_info,
                         PEBBLE_BATTERY_STATE_CHANGE_EVENT, structured_glance);
  prv_subscribe_to_event(&settings_glance->pebble_app_event_info, PEBBLE_COMM_SESSION_EVENT,
                         structured_glance);
  prv_subscribe_to_event(&settings_glance->airplane_mode_event_info, PEBBLE_BT_STATE_EVENT,
                         structured_glance);
  prv_subscribe_to_event(&settings_glance->quiet_time_event_info, PEBBLE_DO_NOT_DISTURB_EVENT,
                         structured_glance);
#ifdef CONFIG_HRM
  prv_subscribe_to_event(&settings_glance->hrm_sharing_event_info,
                         PEBBLE_BLE_HRM_SHARING_STATE_UPDATED_EVENT,
                         structured_glance);
#endif

  return &structured_glance->glance;
}
