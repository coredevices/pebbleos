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
#include "pbl/util/math.h"
#include "pbl/util/size.h"
#include "pbl/util/string.h"
#include "pbl/util/struct.h"

#include <stdio.h>

typedef struct {
  int16_t width;
  int16_t height;
} IconSize;

static const IconSize s_battery_silhouette_icon_size[NumPreferredContentSizes] = {
  //! @note this is the same as Medium until Small is designed
  [PreferredContentSizeSmall] = { .width = 12, .height = 7 },
  [PreferredContentSizeMedium] = { .width = 16, .height = 9 },
  [PreferredContentSizeLarge] = { .width = 16, .height = 9 },
  [PreferredContentSizeExtraLarge] = { .width = 21, .height = 12 },
};

static IconSize prv_get_battery_silhouette_icon_size(void) {
  return s_battery_silhouette_icon_size[launcher_menu_layer_get_clamped_content_size()];
}

static const IconSize s_charging_icon_size[NumPreferredContentSizes] = {
  //! @note this is the same as Medium until Small is designed
  [PreferredContentSizeSmall] = { .width = 5, .height = 7 },
  [PreferredContentSizeMedium] = { .width = 7, .height = 9 },
  [PreferredContentSizeLarge] = { .width = 7, .height = 9 },
  [PreferredContentSizeExtraLarge] = { .width = 9, .height = 12 },
};

static IconSize prv_get_charging_icon_size(void) {
  return s_charging_icon_size[launcher_menu_layer_get_clamped_content_size()];
}

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

  const IconSize icon_size = prv_get_charging_icon_size();
  const int16_t w = icon_size.width;
  const int16_t h = icon_size.height;

  if (render) {
    const GColor bolt_color =
        launcher_app_glance_structured_get_highlight_color(structured_glance);
    graphics_context_set_fill_color(ctx, bolt_color);

    // Minimum 1px crossbar height keeps the bolt recognizable at small sizes.
    const int16_t bar_height = MAX((int16_t)1, (int16_t)ROUND(h, 9));
    const int16_t bar_top = (int16_t)ROUND(h * 4, 9);
    const int16_t bar_bottom = bar_top + bar_height;

    const int16_t apex_left = (int16_t)ROUND(w * 4, 7);
    const int16_t apex_right = (int16_t)ROUND(w * 5, 7);
    const int16_t upper_right_at_bar = apex_left;
    for (int16_t y = 0; y < bar_top; y++) {
      const int16_t x0 = (int16_t)ROUND(apex_left * (bar_top - y), bar_top);
      const int16_t x1 =
          MAX((int16_t)(x0 + 1),
              (int16_t)ROUND(apex_right * (bar_top - y) + upper_right_at_bar * y, bar_top));
      graphics_fill_rect(ctx, &GRect(rect->origin.x + x0, rect->origin.y + y, x1 - x0, 1));
    }

    graphics_fill_rect(ctx, &GRect(rect->origin.x, rect->origin.y + bar_top, w, bar_height));

    const int16_t lower_left_at_bar = (int16_t)ROUND(w * 3, 7);
    const int16_t lower_right_at_bar = (int16_t)ROUND(w * 6, 7);
    const int16_t bottom_point = (int16_t)ROUND(w * 5, 14);  // 2.5/7, kept as a 14ths fraction
    const int16_t rows_below = h - bar_bottom;
    for (int16_t i = 0; i < rows_below; i++) {
      int16_t x0, x1;
      if (rows_below > 1) {
        const int16_t denom = rows_below - 1;
        x0 = (int16_t)ROUND(lower_left_at_bar * (denom - i) + bottom_point * i, denom);
        x1 = MAX((int16_t)(x0 + 1),
                 (int16_t)ROUND(lower_right_at_bar * (denom - i) + (bottom_point + 1) * i,
                                denom));
      } else {
        x0 = bottom_point;
        x1 = bottom_point + 1;
      }
      graphics_fill_rect(ctx, &GRect(rect->origin.x + x0, rect->origin.y + bar_bottom + i,
                                     x1 - x0, 1));
    }
  }

  if (size_out) {
    *size_out = GSize(w, settings_glance->subtitle_font_height);
  }
}

static void prv_battery_icon_node_draw_cb(GContext *ctx, const GRect *rect,
                                          PBL_UNUSED const GTextNodeDrawConfig *config, bool render,
                                          GSize *size_out, void *user_data) {
  LauncherAppGlanceStructured *structured_glance = user_data;
  LauncherAppGlanceSettings *settings_glance =
      launcher_app_glance_structured_get_data(structured_glance);

  const IconSize icon_size = prv_get_battery_silhouette_icon_size();
  const GSize battery_silhouette_icon_size = GSize(icon_size.width, icon_size.height);

  if (render) {
    const GPoint battery_silhouette_path_points[] = {
      {0, 0},
      {icon_size.width - 1, 0},
      {icon_size.width - 1, 1},
      {icon_size.width + 1, 2},
      {icon_size.width + 1, icon_size.height - 3},
      {icon_size.width - 1, icon_size.height - 3},
      {icon_size.width - 1, icon_size.height - 1},
      {0, icon_size.height - 1},
    };
    GPath battery_silhouette_path = (GPath) {
      .num_points = ARRAY_LENGTH(battery_silhouette_path_points),
      .points = (GPoint *)battery_silhouette_path_points,
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

  const IconSize battery_icon_size = prv_get_battery_silhouette_icon_size();
  const int16_t battery_icon_offset_y =
      (settings_glance->subtitle_font_height - battery_icon_size.height) / 2;

  GTextNodeCustom *battery_icon_node =
      graphics_text_node_create_custom(prv_battery_icon_node_draw_cb, structured_glance);
  // Push the battery icon down to center it properly
  battery_icon_node->node.offset.y += battery_icon_offset_y;

  // Achieves the design spec'd 6 px horizontal spacing b/w the battery icon and charging icon
  battery_icon_node->node.margin.w = 7;
  GTextNode *vertically_centered_battery_icon_node =
      prv_wrap_text_node_in_vertically_centered_container(&battery_icon_node->node);
  graphics_text_node_container_add_child(&horizontal_container_node->container,
                                         vertically_centered_battery_icon_node);

  if (settings_glance->glance_state.battery_charge_state.is_plugged) {
    const int16_t charging_icon_offset_y =
        (settings_glance->subtitle_font_height - prv_get_charging_icon_size().height) / 2;
    GTextNodeCustom *charging_icon_node =
        graphics_text_node_create_custom(prv_charging_icon_node_draw_cb, structured_glance);
    // Push the charging icon down to center it properly
    charging_icon_node->node.offset.y += charging_icon_offset_y;
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

  // Cache the subtitle font height for simplifying layout calculations
  settings_glance->subtitle_font_height =
      fonts_get_font_height(launcher_menu_layer_get_subtitle_font());

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
