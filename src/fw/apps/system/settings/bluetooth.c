/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "bluetooth.h"
#include "menu.h"
#include "remote.h"
#include "window.h"

#include "applib/app.h"
#include "applib/app_focus_service.h"
#include "applib/event_service_client.h"
#include "applib/fonts/fonts.h"
#include "applib/graphics/graphics.h"
#include "applib/graphics/gtypes.h"
#include "applib/ui/ui.h"
#include "comm/bt_lock.h"
#include "comm/ble/gap_le_connection.h"
#include "comm/ble/gap_le_device_name.h"
#include "kernel/pbl_malloc.h"
#include "kernel/ui/system_icons.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/bluetooth/bluetooth_persistent_storage.h"
#include "pbl/services/bluetooth/local_id.h"
#include "pbl/services/bluetooth/pairability.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/bluetooth/ble_hrm.h"
#include "shell/system_theme.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/string.h"

#include <bluetooth/bluetooth_types.h>
#include <bluetooth/sm_types.h>
#include <pbl/btutil/bt_device.h>

#include <stdio.h>
#include <string.h>

#define HEADER_BUFFER_SIZE 22

#define SHARING_HEART_RATE_EXTRA_HEIGHT_PX (18)

typedef enum SettingsBluetooth {
  SettingsBluetoothAirplaneMode,
  SettingsBluetoothTotal,
} SettingsBluetooth;

enum {
  BluetoothIconIdx,
  BluetoothAltIconIdx,
  AirplaneIconIdx,
  NumIcons,
};

static const uint32_t ICON_RESOURCE_ID[NumIcons] = {
  RESOURCE_ID_SETTINGS_ICON_BLUETOOTH,
  RESOURCE_ID_SETTINGS_ICON_BLUETOOTH_ALT,
  RESOURCE_ID_SETTINGS_ICON_AIRPLANE,
};

typedef enum {
  ToggleStateIdle,
  ToggleStateEnablingBluetooth,
  ToggleStateDisablingBluetooth,
} ToggleState;

typedef struct SettingsBluetoothData {
  SettingsCallbacks callbacks;

  GBitmap icon_heap_bitmap[NumIcons];

  ListNode* remote_list_head;

  char header_buffer[HEADER_BUFFER_SIZE];
  ToggleState toggle_state;
  bool did_enable_pairability;

  EventServiceInfo bt_airplane_event_info;
  EventServiceInfo bt_connection_event_info;
  EventServiceInfo bt_pairing_event_info;
  EventServiceInfo ble_device_name_updated_event_info;
#ifdef CONFIG_HRM
  EventServiceInfo ble_hrm_sharing_event_info;
#endif
} SettingsBluetoothData;

// BT stack interaction stuff
///////////////////////////

static void settings_bluetooth_toggle_airplane_mode(SettingsBluetoothData* data) {
  const bool airplane_mode = bt_ctl_is_airplane_mode_on();
  bt_ctl_set_airplane_mode_async(!airplane_mode);
  data->toggle_state =
      airplane_mode ? ToggleStateEnablingBluetooth : ToggleStateDisablingBluetooth;
  settings_menu_mark_dirty(SettingsMenuItemBluetooth);
}

bool is_remote_connected(StoredRemote* remote) {
  return (remote->ble.connection != NULL);
}

static int remote_comparator(StoredRemote* remote, StoredRemote* other) {
  if (is_remote_connected(remote) != is_remote_connected(other)) {
    return is_remote_connected(remote) ? -1 : 1;
  } else {
    return strncmp(remote->name, other->name, sizeof(remote->name));
  }
}

static void add_remote(SettingsBluetoothData* data, StoredRemote* remote) {
  const bool ascending = false;
  data->remote_list_head = list_sorted_add(data->remote_list_head, &remote->list_node,
      (Comparator) remote_comparator, ascending);
}

static StoredRemote* stored_remote_create(void) {
  StoredRemote* remote = task_malloc_check(sizeof(*remote));
  *remote = (StoredRemote){};
  return remote;
}

static void prv_copy_device_name_with_fallback(StoredRemote *remote, const char *name) {
  if (!name || strlen(name) == 0) {
    i18n_get_with_buffer("<Untitled>", remote->name, sizeof(remote->name));
  } else {
    strncpy(remote->name, name, sizeof(remote->name));
  }
}

static void prv_add_ble_remote(BTDeviceInternal *device, SMIdentityResolvingKey *irk,
                               const char *name, BTBondingID *id, void *context) {
  SettingsBluetoothData *data = (SettingsBluetoothData*) context;
  if (!data) {
    return;
  }

  StoredRemote* remote = stored_remote_create();
  remote->ble.bonding = *id;
  prv_copy_device_name_with_fallback(remote, name);
  add_remote(data, remote);
}

static void prv_add_ble_remotes(SettingsBluetoothData *data) {
  bt_persistent_storage_for_each_ble_pairing(prv_add_ble_remote, data);

  StoredRemote *remote = (StoredRemote *)data->remote_list_head;
  while (remote) {
    SMIdentityResolvingKey irk;
    BTDeviceInternal device;

    if (bt_persistent_storage_get_ble_pairing_by_id(remote->ble.bonding, &irk, &device, NULL)) {
      bt_lock();
      GAPLEConnection *connection = gap_le_connection_find_by_irk(&irk);
      if (!connection) {
        connection = gap_le_connection_by_device(&device);
      }
      remote->ble.connection = connection;
#ifdef CONFIG_HRM
      remote->ble.is_sharing_heart_rate = ble_hrm_is_sharing_to_connection(connection);
#endif
      bt_unlock();
    }
    remote = (StoredRemote *)remote->list_node.next;
  }
}

static void prv_clear_remote_list(SettingsBluetoothData* data) {
  while (data->remote_list_head) {
    StoredRemote* remote = (StoredRemote*) data->remote_list_head;
    data->remote_list_head = list_pop_head(&remote->list_node);
    task_free(remote);
  }
}

static void prv_reload_remote_list(SettingsBluetoothData* data) {
  prv_clear_remote_list(data);
  prv_add_ble_remotes(data);
}

static void settings_bluetooth_update_remotes_private(SettingsBluetoothData* data) {
  prv_reload_remote_list(data);

  if (!data->remote_list_head) {
    strncpy(data->header_buffer, i18n_get("Pairing Instructions", data), HEADER_BUFFER_SIZE);
  } else {
    const unsigned int num_remotes = list_count(data->remote_list_head);
    sniprintf(data->header_buffer, HEADER_BUFFER_SIZE,
              (num_remotes != 1) ? i18n_get("%u Paired Phones", data) :
                                   i18n_get("%u Paired Phone", data),
              num_remotes);
  }
}

void settings_bluetooth_update_remotes(SettingsBluetoothData *data) {
  settings_bluetooth_update_remotes_private(data);
  settings_menu_reload_data(SettingsMenuItemBluetooth);
}

//////////

static void prv_settings_bluetooth_event_handler(PebbleEvent *event, void *context) {
  SettingsBluetoothData* settings_data = (SettingsBluetoothData *) context;
  PBL_LOG_DBG("BT EVENT");
  switch (event->type) {
    case PEBBLE_BT_CONNECTION_EVENT:
      // If BT Settings is open, update BLE device name upon connecting device:
      if (event->bluetooth.connection.is_ble &&
          event->bluetooth.connection.state == PebbleBluetoothConnectionEventStateConnected) {
        // https://pebbletechnology.atlassian.net/browse/PBL-22176
        // iOS seems to respond with 0x0E (Unlikely Error) when performing this request while
        // the encryption set up is going on. For non-bonded devices it will work fine though.
        gap_le_device_name_request(&event->bluetooth.connection.device);
      }
      // fall-through!
    case PEBBLE_BT_PAIRING_EVENT:
#ifdef CONFIG_HRM
    case PEBBLE_BLE_HRM_SHARING_STATE_UPDATED_EVENT:
#endif
    case PEBBLE_BLE_DEVICE_NAME_UPDATED_EVENT: {
      bool had_remotes = (settings_data->remote_list_head != NULL);
      settings_bluetooth_update_remotes_private(settings_data);
      bool has_remotes = (settings_data->remote_list_head != NULL);
      
      // Handle single phone pairing policy: enable/disable advertising based on pairing state
      if (had_remotes && !has_remotes) {
        // Device was removed, enable advertising
        if (!settings_data->did_enable_pairability) {
          bt_pairability_use();
          settings_data->did_enable_pairability = true;
          PBL_LOG_INFO("Enabled advertising - no paired devices");
        }
      } else if (!had_remotes && has_remotes) {
        // Device was added, disable advertising
        if (settings_data->did_enable_pairability) {
          bt_pairability_release();
          settings_data->did_enable_pairability = false;
          PBL_LOG_INFO("Disabled advertising - device paired");
        }
      }
      
      settings_menu_mark_dirty(SettingsMenuItemBluetooth);
      break;
    }

    case PEBBLE_BT_STATE_EVENT: {
      settings_data->toggle_state = ToggleStateIdle;
      settings_menu_mark_dirty(SettingsMenuItemBluetooth);
      break;
    }

    default:
      break;
  }
}

// UI Stuff
/////////////////////////////
// Menu Layer Callbacks
/////////////////////////////
//-- Address
//   ...
//|  Airplane Mode: Off
//-- Paired Devices
//|  Device Name
//   Connected
//|  Device Name
//

#if PBL_RECT
static void prv_draw_stored_remote_item_rect(GContext *ctx, const Layer *cell_layer,
                                             const char *remote_name, const char *connected_string,
                                             const char *le_string,
                                             const char *is_sharing_heart_rate_string) {
  const GFont font = ((le_string || is_sharing_heart_rate_string) ?
                      fonts_get_system_font(FONT_KEY_GOTHIC_18) : NULL);

  if (le_string) {
    GRect box = cell_layer->bounds;
    box.size.w -= 5;
    box.origin.y += 20;
    box.size.h = 24;

    graphics_draw_text(ctx, le_string, font, box, GTextOverflowModeFill, GTextAlignmentRight, NULL);
  }

  if (is_sharing_heart_rate_string) {
    const int horizontal_margin = menu_cell_basic_horizontal_inset();
    GRect box = grect_inset(cell_layer->bounds, GEdgeInsets(0, horizontal_margin));
    box.origin.y += 38;
    box.size.h = 24;

    graphics_draw_text(ctx, is_sharing_heart_rate_string, font, box,
                       GTextOverflowModeFill, GTextAlignmentLeft, NULL);

    // Gross hack to avoid centering the title / subtitle labels in the entire cell:
    ((Layer *)cell_layer)->bounds.size.h -= SHARING_HEART_RATE_EXTRA_HEIGHT_PX;
  }

  menu_cell_basic_draw(ctx, cell_layer, remote_name, connected_string, NULL);

  if (is_sharing_heart_rate_string) {
    // Restore original height:
    ((Layer *)cell_layer)->bounds.size.h += SHARING_HEART_RATE_EXTRA_HEIGHT_PX;
  }
}
#endif

bool settings_bluetooth_is_sharing_heart_rate_for_stored_remote(StoredRemote* remote) {
#ifdef CONFIG_HRM
  return remote->ble.is_sharing_heart_rate;
#else
  return false;
#endif  // CONFIG_HRM
}

#if PBL_ROUND
static void prv_draw_stored_remote_item_round(GContext *ctx, const Layer *cell_layer,
                                              const char *remote_name, const char *connected_string,
                                              const char *le_string,
                                              const char *is_sharing_heart_rate_string) {
#  ifdef CONFIG_HRM
  _Static_assert(false, "FIXME: Implement round drawing code to show heart rate sharing status!");
#  endif  // CONFIG_HRM
  menu_cell_basic_draw(ctx, cell_layer, remote_name, connected_string, NULL);
}
#endif  // PBL_ROUND

static void draw_stored_remote_item(GContext *ctx, const Layer *cell_layer,
                                    uint16_t device_index, SettingsBluetoothData *data) {
  const uint32_t num_remotes = list_count(data->remote_list_head);
  PBL_ASSERT(device_index < num_remotes, "Got index %" PRId16 " only have %" PRId32,
             device_index, num_remotes);
  StoredRemote* remote = (StoredRemote*) list_get_at(data->remote_list_head, device_index);
  bool connected = is_remote_connected(remote);

  const char *le_string = NULL;

  const char *connected_string = connected ? i18n_get("Connected", data) :
                                 PBL_IF_RECT_ELSE("", NULL);

  // Add ellipsis if the name might have been cut off by the mobile
  const char ellipsis[] = UTF8_ELLIPSIS_STRING;
  const size_t max_name_size = BT_DEVICE_NAME_BUFFER_SIZE - 2;
  const size_t name_size = strnlen(remote->name, BT_DEVICE_NAME_BUFFER_SIZE);
  char *remote_name = task_zalloc_check(max_name_size + sizeof(ellipsis));
  strncpy(remote_name, remote->name, name_size);
  if (name_size > max_name_size) {
    const size_t ellipsis_start_offset = utf8_get_size_truncate(remote_name, name_size);
    strncpy(&remote_name[ellipsis_start_offset], ellipsis, sizeof(ellipsis));
  }

  const char *is_sharing_heart_rate =
      (settings_bluetooth_is_sharing_heart_rate_for_stored_remote(remote) ?
       i18n_get("Sharing Heart Rate ❤", data) : NULL);

  PBL_IF_RECT_ELSE(prv_draw_stored_remote_item_rect,
                   prv_draw_stored_remote_item_round)(ctx, cell_layer, remote_name,
                                                      connected_string, le_string,
                                                      is_sharing_heart_rate);

  task_free(remote_name);
}


static uint16_t prv_num_rows_cb(SettingsCallbacks *context) {
  SettingsBluetoothData *data = (SettingsBluetoothData *) context;
  return list_count(data->remote_list_head) + 1;
}

static int16_t prv_row_height_cb(SettingsCallbacks *context, uint16_t row, bool is_selected) {
#if PBL_RECT
#  ifdef CONFIG_HRM
  int heart_rate_sharing_text_height = 0;
  if (row > 0) {
    SettingsBluetoothData *data = (SettingsBluetoothData *) context;
    const uint16_t device_index = row - 1;
    StoredRemote* remote = (StoredRemote*) list_get_at(data->remote_list_head, device_index);
    if (settings_bluetooth_is_sharing_heart_rate_for_stored_remote(remote)) {
      heart_rate_sharing_text_height = SHARING_HEART_RATE_EXTRA_HEIGHT_PX;
    }
  }
#  else
  const int heart_rate_sharing_text_height = 0;
#  endif  // CONFIG_HRM
  return menu_cell_basic_cell_height() + heart_rate_sharing_text_height;
#elif PBL_ROUND
  return (is_selected ? MENU_CELL_ROUND_FOCUSED_TALL_CELL_HEIGHT :
          MENU_CELL_ROUND_UNFOCUSED_SHORT_CELL_HEIGHT);
#else
#endif
}

static void prv_draw_row_cb(SettingsCallbacks *context, GContext *ctx,
                            const Layer *cell_layer, uint16_t row, bool selected) {
  SettingsBluetoothData *data = (SettingsBluetoothData *) context;
  if (row == 0) {
      char device_name_buffer[BT_DEVICE_NAME_BUFFER_SIZE];
      const char *subtitle = NULL;
      const char *title = i18n_get("Connection", data);
      GBitmap *icon = NULL;
      if (data->toggle_state == ToggleStateIdle) {
        if (bt_ctl_is_airplane_mode_on()) {
          subtitle = i18n_get("Airplane Mode", data);
          icon = &data->icon_heap_bitmap[AirplaneIconIdx];
        } else {
          // Always show device name as subtitle
          bt_local_id_copy_device_name(device_name_buffer, false);
          subtitle = device_name_buffer;
          icon = &data->icon_heap_bitmap[BluetoothIconIdx];
        }
      } else {
        subtitle = (data->toggle_state == ToggleStateDisablingBluetooth)
            ? i18n_get("Disabling...", data) : i18n_get("Enabling...", data);
        icon = &data->icon_heap_bitmap[BluetoothAltIconIdx];
      }

      menu_cell_basic_draw(ctx, cell_layer, title, subtitle, icon);

    // TODO PBL-23111: Decide how we should show these strings on round displays
#if PBL_RECT
      // Hack: the pairing instruction is drawn in the cell callback, but outside of the cell...
      const GDrawState draw_state = ctx->draw_state;
      // Enable drawing outside of the cell:
      ctx->draw_state.clip_box = ctx->dest_bitmap.bounds;

      graphics_context_set_text_color(ctx, GColorBlack);
      GFont font = system_theme_get_font_for_default_size(TextStyleFont_MenuCellSubtitle);
      const int16_t horizontal_inset = menu_cell_basic_horizontal_inset() * 3;
      GRect box = cell_layer->bounds;
      box.origin.x = horizontal_inset;
      box.origin.y = menu_cell_basic_cell_height() + (int16_t)9;
      box.size.w -= horizontal_inset * 2;
      box.size.h = 83;

      if (!data->remote_list_head) {
        if (bt_ctl_is_airplane_mode_on()) {
          graphics_draw_text(ctx, i18n_get("Disable Airplane Mode to connect.", data), font,
                             box, GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter, NULL);
        } else {
          graphics_draw_text(ctx, i18n_get("Open the Pebble app on your phone to connect.", data),
                             font, box, GTextOverflowModeTrailingEllipsis,
                             GTextAlignmentCenter, NULL);
        }
      } else {
        // Show message when any phone is paired (even if disconnected)
        // Position the message lower to appear below the paired phone row
        GRect msg_box = box;
        msg_box.origin.y += menu_cell_basic_cell_height() - 10;
        graphics_draw_text(ctx, i18n_get("Forget this device to pair a new device.", data),
                           font, msg_box, GTextOverflowModeTrailingEllipsis,
                           GTextAlignmentCenter, NULL);
      }

      ctx->draw_state = draw_state;
#endif
  } else {
    const uint16_t device_index = row - 1;
    draw_stored_remote_item(ctx, cell_layer, device_index, data);
  }
}

static void prv_select_click_cb(SettingsCallbacks *context, uint16_t row) {
  SettingsBluetoothData *data = (SettingsBluetoothData *) context;
  if (row == 0) {
    settings_bluetooth_toggle_airplane_mode(data);
    return;
  }
  if (!data->remote_list_head) {
    return;
  }
  prv_reload_remote_list(data);
  StoredRemote* remote = (StoredRemote*) list_get_at(data->remote_list_head, row - 1);
  settings_remote_menu_push(data, remote);
}

static void prv_focus_handler(bool in_focus) {
  if (!in_focus) {
    return;
  }
  settings_menu_reload_data(SettingsMenuItemBluetooth);
}

static void prv_expand_cb(SettingsCallbacks *context) {
  SettingsBluetoothData *data = (SettingsBluetoothData *) context;

  settings_bluetooth_update_remotes_private(data);

  // When entering the BT Settings, update device names of all connected devices:
  if (!bt_ctl_is_airplane_mode_on()) {
    gap_le_device_name_request_all();
  }

  data->bt_airplane_event_info = (EventServiceInfo) {
    .type = PEBBLE_BT_STATE_EVENT,
    .handler = prv_settings_bluetooth_event_handler,
    .context = data,
  };
  data->bt_connection_event_info = (EventServiceInfo) {
    .type = PEBBLE_BT_CONNECTION_EVENT,
    .handler = prv_settings_bluetooth_event_handler,
    .context = data,
  };
  data->bt_pairing_event_info = (EventServiceInfo) {
    .type = PEBBLE_BT_PAIRING_EVENT,
    .handler = prv_settings_bluetooth_event_handler,
    .context = data,
  };
  data->ble_device_name_updated_event_info = (EventServiceInfo) {
    .type = PEBBLE_BLE_DEVICE_NAME_UPDATED_EVENT,
    .handler = prv_settings_bluetooth_event_handler,
    .context = data,
  };
#ifdef CONFIG_HRM
  data->ble_hrm_sharing_event_info = (EventServiceInfo) {
    .type = PEBBLE_BLE_HRM_SHARING_STATE_UPDATED_EVENT,
    .handler = prv_settings_bluetooth_event_handler,
    .context = data,
  };
  event_service_client_subscribe(&data->ble_hrm_sharing_event_info);
#endif
  event_service_client_subscribe(&data->bt_airplane_event_info);
  event_service_client_subscribe(&data->bt_connection_event_info);
  event_service_client_subscribe(&data->bt_pairing_event_info);
  event_service_client_subscribe(&data->ble_device_name_updated_event_info);
  
  // Only enable pairing/advertising if there are no paired devices (single phone policy)
  data->did_enable_pairability = false;
  if (!data->remote_list_head) {
    bt_pairability_use();
    data->did_enable_pairability = true;
  }
  
  // Reload & redraw after pairing popup
  app_focus_service_subscribe_handlers((AppFocusHandlers) { .did_focus = prv_focus_handler });
}

// Turns off services that are part of the bluetooth settings menu such as enabling
// discovery. We don't want to keep these services running longer than necessary because
// they consume a fair amount of power
static void prv_hide_cb(SettingsCallbacks *context) {
  SettingsBluetoothData *data = (SettingsBluetoothData *) context;
  
  // Only release pairability if we enabled it
  if (data->did_enable_pairability) {
    bt_pairability_release();
  }
  
#ifdef CONFIG_HRM
  event_service_client_unsubscribe(&data->ble_hrm_sharing_event_info);
#endif
  event_service_client_unsubscribe(&data->bt_airplane_event_info);
  event_service_client_unsubscribe(&data->bt_connection_event_info);
  event_service_client_unsubscribe(&data->bt_pairing_event_info);
  event_service_client_unsubscribe(&data->ble_device_name_updated_event_info);
  app_focus_service_unsubscribe();
}

static void prv_deinit_cb(SettingsCallbacks *context) {
  SettingsBluetoothData *data = (SettingsBluetoothData *) context;

  i18n_free_all(data);

  prv_clear_remote_list(data);
  for (unsigned int idx = 0; idx < NumIcons; ++idx) {
    gbitmap_deinit(&data->icon_heap_bitmap[idx]);
  }
  app_free(data);
}

static Window *prv_init(void) {
  SettingsBluetoothData *data = app_malloc_check(sizeof(SettingsBluetoothData));
  *data = (SettingsBluetoothData){};

  for (unsigned int idx = 0; idx < NumIcons; ++idx) {
    gbitmap_init_with_resource(&data->icon_heap_bitmap[idx], ICON_RESOURCE_ID[idx]);
  }

  data->callbacks = (SettingsCallbacks) {
    .deinit = prv_deinit_cb,
    .draw_row = prv_draw_row_cb,
    .select_click = prv_select_click_cb,
    .num_rows = prv_num_rows_cb,
    .row_height = prv_row_height_cb,
    .expand = prv_expand_cb,
    .hide = prv_hide_cb,
  };

  return settings_window_create(SettingsMenuItemBluetooth, &data->callbacks);
}

const SettingsModuleMetadata *settings_bluetooth_get_info(void) {
  static const SettingsModuleMetadata s_module_info = {
    .name = i18n_noop("Bluetooth"),
    .init = prv_init,
  };

  return &s_module_info;
}

#undef HEADER_BUFFER_SIZE
