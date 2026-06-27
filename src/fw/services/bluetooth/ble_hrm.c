/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/bluetooth/ble_hrm.h"

#include "applib/event_service_client.h"
#include "comm/ble/gap_le_connection.h"
#include "comm/ble/gap_le_slave_reconnect.h"
#include "comm/bt_lock.h"
#include "kernel/pbl_malloc.h"
#include "kernel/event_loop.h"
#include "kernel/events.h"
#include "popups/ble_hrm/ble_hrm_reminder_popup.h"
#include "popups/ble_hrm/ble_hrm_sharing_popup.h"
#include "process_management/app_manager.h"
#include "pbl/services/analytics/analytics.h"
#include "pbl/services/regular_timer.h"
#include "pbl/services/activity/activity.h"
#include "pbl/services/activity/workout_service.h"
#include "shell/system_app_ids.auto.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

#include <bluetooth/gap_le_connect.h>
#include <bluetooth/hrm_service.h>
#include <pbl/btutil/bt_device.h>
#include <pbl/util/list.h>
#include <pbl/util/size.h>

PBL_LOG_MODULE_DECLARE(service_bluetooth, CONFIG_SERVICE_BLUETOOTH_LOG_LEVEL);

#ifdef CONFIG_HRM

#define BLE_HRM_UPDATE_INTERVAL_SEC (1)

typedef struct BLEHRMSharingRequest {
  GAPLEConnection *connection;
} BLEHRMSharingRequest;

static bool s_ble_hrm_is_inited;
static int s_ble_hrm_subscription_count;
static RegularTimerInfo s_ble_hrm_timer;
static EventServiceInfo s_ble_hrm_event_info;

typedef enum {
  HrmSharingPermission_Unknown,
  HrmSharingPermission_Granted,
  HrmSharingPermission_Declined,
} HrmSharingPermission;

typedef struct BLEHRMSharingPermission {
  ListNode node;
  BTDeviceInternal device;

  //! Whether the user has confirmed that sharing HRM data to this device is permitted.
  HrmSharingPermission permission;
} BLEHRMSharingPermission;

static BLEHRMSharingPermission *s_permissions_head;

static bool prv_hw_and_sw_supports_hrm(void) {
  return (bt_driver_is_hrm_service_supported() &&
          sys_hrm_manager_is_hrm_present());
}

bool ble_hrm_is_supported_and_enabled(void) {
  return (prv_hw_and_sw_supports_hrm() &&
          activity_prefs_ble_hrm_sharing_is_enabled());
}

static void prv_reset_subscriptions(void);
static void prv_update_subscription(GAPLEConnection *connection, bool is_subscribed);

static bool prv_free_permission_for_each_cb(ListNode *node, void *unused) {
  kernel_free(node);
  return true;  // continue iteration
}

static void prv_free_all_permissions(void) {
  list_foreach((ListNode *)s_permissions_head, prv_free_permission_for_each_cb, NULL);
  s_permissions_head = NULL;
}

static bool prv_find_permission_by_device_filter_cb(ListNode *found_node, void *data) {
  const BTDeviceInternal *device = data;
  BLEHRMSharingPermission *permission = (BLEHRMSharingPermission *)found_node;
  return bt_device_internal_equal(device, &permission->device);
}

static BLEHRMSharingPermission *prv_find_permission_by_device(const BTDeviceInternal *device) {
  return (BLEHRMSharingPermission *)list_find((ListNode *)s_permissions_head,
                                              prv_find_permission_by_device_filter_cb,
                                              (void *)device);
}

static void prv_upsert_permission(const BTDeviceInternal *device, HrmSharingPermission permission) {
  BLEHRMSharingPermission *p = prv_find_permission_by_device(device);
  if (!p) {
    p = kernel_zalloc_check(sizeof(*p));
    p->device = *device;
    s_permissions_head = (BLEHRMSharingPermission *)list_prepend((ListNode *)s_permissions_head,
                                                                 (ListNode *)p);
  }
  p->permission = permission;
}

static HrmSharingPermission prv_get_permission_by_device(const BTDeviceInternal *device) {
  BLEHRMSharingPermission *p = prv_find_permission_by_device(device);
  if (!p) {
    return HrmSharingPermission_Unknown;
  }
  return p->permission;
}

void ble_hrm_handle_ble_hrm_sharing_enabled(bool is_enabled) {
  if (!prv_hw_and_sw_supports_hrm()) {
    return;
  }
  PBL_LOG_INFO("BLE HRM sharing prefs updated: is_enabled=%u", is_enabled);

  if (is_enabled && workout_service_is_workout_ongoing()) {
    gap_le_slave_reconnect_hrm_restart();
  } else {
    prv_reset_subscriptions();
    gap_le_slave_reconnect_hrm_stop();
  }
  bt_driver_hrm_service_enable(is_enabled);
}

static bool prv_is_sharing(const GAPLEConnection *const connection) {
  return (connection->hrm_service_is_subscribed &&
          (prv_get_permission_by_device(&connection->device) == HrmSharingPermission_Granted));
}

bool ble_hrm_is_sharing_to_connection(const GAPLEConnection *const connection) {
  bt_lock_assert_held(true);
  if (!connection) {
    return false;
  }
  return prv_is_sharing(connection);
}

bool ble_hrm_is_sharing(void) {
  return (s_ble_hrm_subscription_count > 0);
}

typedef struct {
  BTDeviceInternal *next_permitted_device;
  size_t slots_left;
} CopySharingDevicesCtx;

static void prv_copy_sharing_devices_for_each_connection_cb(GAPLEConnection *connection,
                                                              void *data) {
  CopySharingDevicesCtx *ctx = data;
  if (ctx->slots_left && prv_is_sharing(connection)) {
    *ctx->next_permitted_device = connection->device;
    ++ctx->next_permitted_device;
    --ctx->slots_left;
  }
}

static size_t prv_copy_sharing_devices(BTDeviceInternal *devices_out,
                                       size_t max_devices) {
  bt_lock();
  CopySharingDevicesCtx ctx = {
    .next_permitted_device = devices_out,
    .slots_left = max_devices,
  };
  gap_le_connection_for_each(prv_copy_sharing_devices_for_each_connection_cb, &ctx);
  bt_unlock();
  return (max_devices - ctx.slots_left);
}

static void prv_handle_health_event(PebbleEvent *e, void *context) {
  if (!s_ble_hrm_is_inited) {
    return;
  }
  if (s_ble_hrm_subscription_count == 0) {
    return;
  }
  PBL_ASSERTN(e->type == PEBBLE_HEALTH_SERVICE_EVENT);
  const PebbleHealthEvent *const health_event = &e->health_event;
  if (health_event->type != HealthEventHeartRateUpdate) {
    return;
  }
  const BleHrmServiceMeasurement measurement = {
    .bpm = health_event->data.heart_rate_update.current_bpm,
    .is_on_wrist = (health_event->data.heart_rate_update.quality >= HRMQuality_Worst),
  };

  BTDeviceInternal sharing_to_devices[4];
  const size_t num_devices = prv_copy_sharing_devices(sharing_to_devices,
                                                      ARRAY_LENGTH(sharing_to_devices));
  bt_driver_hrm_service_handle_measurement(&measurement, sharing_to_devices, num_devices);
}

static void prv_start_hrm_kernel_main(void *unused) {
  PBL_LOG_INFO("BLE HRM sharing started");
  s_ble_hrm_event_info = (EventServiceInfo) {
    .type = PEBBLE_HEALTH_SERVICE_EVENT,
    .handler = prv_handle_health_event,
  };
  event_service_client_subscribe(&s_ble_hrm_event_info);
}

static void prv_stop_hrm_kernel_main(void *unused) {
  PBL_LOG_INFO("BLE HRM sharing stopped");
  event_service_client_unsubscribe(&s_ble_hrm_event_info);
}

static void prv_execute_on_kernel_main(CallbackEventCallback cb) {
  if (pebble_task_get_current() != PebbleTask_KernelMain) {
    launcher_task_add_callback(cb, NULL);
  } else {
    cb(NULL);
  }
}

static void prv_push_sharing_request_window_kernel_main_cb(void *ctx) {
  ble_hrm_push_sharing_request_window((BLEHRMSharingRequest *)ctx);
}

static void prv_request_sharing_permission(GAPLEConnection *const connection) {
  PBL_LOG_INFO("Requesting BLE HRM sharing permission");
  BLEHRMSharingRequest *const sharing_request = kernel_zalloc_check(sizeof(*sharing_request));
  sharing_request->connection = connection;
  launcher_task_add_callback(prv_push_sharing_request_window_kernel_main_cb, sharing_request);
}

static void prv_put_sharing_state_updated_event(int subscription_count) {
  // 2 purposes of this event:
  // - refresh the Settings/Bluetooth UI whenever a device (un)subscribes.
  // - present a "Sharing HRM" icon in the Settings app glance.
  PebbleEvent e = {
    .type = PEBBLE_BLE_HRM_SHARING_STATE_UPDATED_EVENT,
    .bluetooth = {
      .le = {
        .hrm_sharing_state = {
          .subscription_count = subscription_count,
        },
      },
    },
  };
  event_put(&e);
}

static void prv_reschedule_popup_timer(void);

static void prv_push_reminder_popup_kernel_main_cb(void *unused) {
  bt_lock();
  if (s_ble_hrm_subscription_count > 0) {
    // Reschedule to show again after BLE_HRM_REMINDER_POPUP_DELAY_MINS
    prv_reschedule_popup_timer();
  }
  bt_unlock();

  ble_hrm_push_reminder_popup();

  PBL_LOG_INFO("BLE HRM sharing timeout fired!");
}

//! @note executes on timer task
static void prv_reminder_popup_timer_cb(void *unused) {
  prv_execute_on_kernel_main(prv_push_reminder_popup_kernel_main_cb);
}

static void prv_stop_popup_timer(void) {
  if (regular_timer_is_scheduled(&s_ble_hrm_timer)) {
    regular_timer_remove_callback(&s_ble_hrm_timer);
  }
}

static void prv_reschedule_popup_timer(void) {
  prv_stop_popup_timer();
  s_ble_hrm_timer = (RegularTimerInfo) {
    .cb = prv_reminder_popup_timer_cb,
  };
  regular_timer_add_multiminute_callback(&s_ble_hrm_timer, BLE_HRM_REMINDER_POPUP_DELAY_MINS);
}

static void prv_update_is_sharing(GAPLEConnection *connection, bool prev_is_sharing) {
  const bool is_sharing = prv_is_sharing(connection);
  if (is_sharing == prev_is_sharing) {
    return;
  }

  if (is_sharing) {
    if (s_ble_hrm_subscription_count == 0) {
      prv_reschedule_popup_timer();
      prv_execute_on_kernel_main(prv_start_hrm_kernel_main);
    }
    ++s_ble_hrm_subscription_count;
  } else {
    --s_ble_hrm_subscription_count;
    if (s_ble_hrm_subscription_count == 0) {
      prv_stop_popup_timer();
      prv_execute_on_kernel_main(prv_stop_hrm_kernel_main);
    }
  }

  // Emit for every subscription, so Settings/Bluetooth menu can update accordingly.
  prv_put_sharing_state_updated_event(s_ble_hrm_subscription_count);
}

static void prv_update_permission(GAPLEConnection *connection, HrmSharingPermission permission) {
  bt_lock_assert_held(true);
  if (prv_get_permission_by_device(&connection->device) == permission) {
    return;
  }
  const bool prev_is_sharing = prv_is_sharing(connection);
  prv_upsert_permission(&connection->device, permission);
  prv_update_is_sharing(connection, prev_is_sharing);
}

static void prv_disconnect_to_kill_subscription(GAPLEConnection *connection) {
  // Unfortunately, GATT does not offer a way to remove a subscription from the server side.
  // Only clients (subscribers) themselves can change the subscription state (write the CCCD).
  // When stopping sharing, we're disconnecting the LE link just to reset the remote subscription
  // state. Yes, a pretty big hammer... :( If we don't do this, the other end will stay subscribed.
  // Then when an app on the phone uses the HRM service "again", there won't be a new CCCD write
  // because the phone was already subscribed...
  // For declining to share up-front, we'll just leave the client subscribed and don't disconnect
  // to prevent reconnection-loops.
  bt_driver_gap_le_disconnect(&connection->device);
}

void ble_hrm_revoke_sharing_permission_for_connection(GAPLEConnection *connection) {
  PBL_LOG_INFO("BLE HRM sharing: revoked for conn %p", connection);
  bt_lock();
  if (gap_le_connection_is_valid(connection)) {
    prv_update_permission(connection, HrmSharingPermission_Declined);
    prv_disconnect_to_kill_subscription(connection);
  }
  bt_unlock();

}

static void prv_disconnect_hrm_subscriber_cb(GAPLEConnection *connection, void *unused) {
  if (prv_is_sharing(connection)) {
    prv_update_subscription(connection, false);
    prv_disconnect_to_kill_subscription(connection);
  }
}

static void prv_clear_connection_subscription_cb(GAPLEConnection *connection, void *unused) {
  connection->hrm_service_is_subscribed = false;
}

static void prv_revoke_gap_le_connection_for_each_cb(GAPLEConnection *connection, void *unused) {
  prv_update_permission(connection, HrmSharingPermission_Declined);
  prv_disconnect_to_kill_subscription(connection);
}

void ble_hrm_revoke_all(void) {
  bt_lock();
  gap_le_connection_for_each(prv_revoke_gap_le_connection_for_each_cb, NULL);
  bt_unlock();

  // Counting as one -- it's one user action.
  PBL_LOG_INFO("BLE HRM sharing: all revoked");
}

static void prv_update_subscription(GAPLEConnection *connection, bool is_subscribed) {
  bt_lock_assert_held(true);
  if (connection->hrm_service_is_subscribed == is_subscribed) {
    return;
  }
  PBL_LOG_INFO("BLE HRM sharing: conn <%p> is_subscribed=%u", connection, is_subscribed);

  const bool prev_is_sharing = prv_is_sharing(connection);
  connection->hrm_service_is_subscribed = is_subscribed;
  prv_update_is_sharing(connection, prev_is_sharing);

  if (is_subscribed) {
    const HrmSharingPermission current_permission =
        prv_get_permission_by_device(&connection->device);
    switch (current_permission) {
      case HrmSharingPermission_Unknown:
        prv_request_sharing_permission(connection);
        break;
      case HrmSharingPermission_Granted:
        // Stop advertising the with the HR service in the adv payload.
        // Note: we're assuming this is the only device we were advertising for.
        gap_le_slave_reconnect_hrm_stop();
        break;
      default:
        break;
    }
  }
}

static void prv_reset_subscriptions(void) {
  bt_lock();
  if (s_ble_hrm_subscription_count) {
    gap_le_connection_for_each(prv_clear_connection_subscription_cb, NULL);
    s_ble_hrm_subscription_count = 0;
    prv_stop_popup_timer();
    prv_put_sharing_state_updated_event(0);
    bt_unlock();

    prv_execute_on_kernel_main(prv_stop_hrm_kernel_main);
  } else {
    bt_unlock();
  }
}

void ble_hrm_handle_sharing_request_response(bool is_granted,
                                             BLEHRMSharingRequest *sharing_request) {
  PBL_LOG_INFO("BLE HRM sharing permission is_granted=%u", is_granted);

  bt_lock();
  GAPLEConnection *connection = sharing_request->connection;
  if (gap_le_connection_is_valid(connection)) {
    const HrmSharingPermission permission =
        (is_granted ? HrmSharingPermission_Granted : HrmSharingPermission_Declined);
    prv_update_permission(connection, permission);
  }
  bt_unlock();

  kernel_free(sharing_request);

}

void bt_driver_cb_hrm_service_update_subscription(const BTDeviceInternal *device,
                                                  bool is_subscribed) {
  bt_lock();
  if (!s_ble_hrm_is_inited) {
    goto unlock;
  }
  GAPLEConnection *connection = gap_le_connection_by_device(device);
  if (!connection) {
    PBL_LOG_ERR("Subscription update but no connection?");
    goto unlock;
  }
  prv_update_subscription(connection, is_subscribed);
unlock:
  bt_unlock();
}

void ble_hrm_handle_disconnection(GAPLEConnection *connection) {
  if (!s_ble_hrm_is_inited) {
    return;
  }
  if (prv_is_sharing(connection)) {
    // Certain phone apps require the HR device to advertise with the HR service in the adv payload
    // in order to make reconnection work, regardless of whether the Pebble mobile app already takes
    // care of reconnecting... Therefore, advertise with the HR service for up to 60 seconds:
    gap_le_slave_reconnect_hrm_restart();
  }
  prv_update_subscription(connection, false /* is_subscribed */);

  // Just leave the permission until we reboot, toggle airplane mode or the user manually revokes.
}

static EventServiceInfo s_ble_hrm_workout_event_info;

static void prv_handle_workout_event(PebbleEvent *e, void *context) {
  if (!s_ble_hrm_is_inited) {
    return;
  }
  const PebbleWorkoutEvent *workout_event = &e->workout;
  bt_lock();
  {
    if (workout_event->type == PebbleWorkoutEvent_Started) {
      if (ble_hrm_is_supported_and_enabled()) {
        gap_le_slave_reconnect_hrm_restart();
      }
    } else if (workout_event->type == PebbleWorkoutEvent_Stopped) {
      gap_le_slave_reconnect_hrm_stop();
      gap_le_connection_for_each(prv_disconnect_hrm_subscriber_cb, NULL);
    }
  }
  bt_unlock();
}

static void prv_subscribe_workout_events(void *unused) {
  event_service_client_subscribe(&s_ble_hrm_workout_event_info);
  if (ble_hrm_is_supported_and_enabled() && workout_service_is_workout_ongoing()) {
    bt_lock();
    gap_le_slave_reconnect_hrm_restart();
    bt_unlock();
  }
}

static void prv_unsubscribe_workout_events(void *unused) {
  event_service_client_unsubscribe(&s_ble_hrm_workout_event_info);
}

void ble_hrm_init(void) {
  s_ble_hrm_is_inited = true;
  s_ble_hrm_timer = (RegularTimerInfo) {};

  s_ble_hrm_workout_event_info = (EventServiceInfo) {
    .type = PEBBLE_WORKOUT_EVENT,
    .handler = prv_handle_workout_event,
  };
  prv_execute_on_kernel_main(prv_subscribe_workout_events);
}

void ble_hrm_deinit(void) {
  s_ble_hrm_is_inited = false;
  gap_le_slave_reconnect_hrm_stop();
  prv_reset_subscriptions();
  prv_free_all_permissions();
  prv_execute_on_kernel_main(prv_unsubscribe_workout_events);
}

// For unit testing
RegularTimerInfo *ble_hrm_timer(void) {
  return &s_ble_hrm_timer;
}

#endif
