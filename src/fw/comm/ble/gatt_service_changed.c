/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "gatt_service_changed.h"

#include "gap_le_connection.h"

#include "comm/bt_lock.h"

#include "kernel/pbl_malloc.h"

#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/system_task.h"
#include <pbl/logging/logging.h>

#include <bluetooth/gatt.h>

extern BTErrno gatt_client_discovery_rediscover_all(const BTDeviceInternal *device);
extern void gatt_client_discovery_handle_service_range_change(GAPLEConnection *connection,
                                                              ATTHandleRange *range);
extern void gatt_client_discovery_discover_range(GAPLEConnection *connection,
                                                 ATTHandleRange *hdl_range);


////////////////////////////////////////////////////////////////////////////////////////////////////
// Client -- Pebble consuming the remote's "Service Changed" characteristic

static void prv_rediscover_kernelbg_cb(void *data) {
  // Rediscover the world:
  BTDeviceInternal *device = (BTDeviceInternal *) data;
  const BTErrno e = gatt_client_discovery_rediscover_all(device);
  kernel_free(device);
  if (e != BTErrnoOK) {
    PBL_LOG_ERR("Service Changed couldn't restart discovery: %i", e);
  }
}

//! @note bt_lock is assumed to be taken by the caller
bool gatt_service_changed_client_handle_indication(struct GAPLEConnection *connection,
                                                   uint16_t att_handle, const uint8_t *value,
                                                   uint16_t value_length) {
  if (connection->gatt_service_changed_att_handle != att_handle) {
    return false;
  }
  if (value_length != sizeof(ATTHandleRange)) {
    PBL_LOG_ERR("Service Changed Indication incorrect length: %u", value_length);
      // Pretend we ate the indication. There will be no GAPLECharacteristic in the system that will
      // match this ATT handle anyway.
    return true;
  }
  ATTHandleRange *range = (ATTHandleRange *) value;
  PBL_LOG_DBG("Service Changed Indication: %x - %x", range->start, range->end);

  // Initiate rediscovery on KernelBG if the Server is asking us to rediscover everything
  // (See "2.5.2 Attribute Caching" in BT Core Specification)
  if ((range->start == 0x001 && range->end == 0xFFFF)) {
    BTDeviceInternal *device = (BTDeviceInternal *) kernel_malloc_check(sizeof(BTDeviceInternal));
    *device = connection->device;
    system_task_add_callback(prv_rediscover_kernelbg_cb, device);
    return true;
  }

  // My understanding is if we get here we will receive a range of handles for
  // _one_ service.  "The start Attribute Handle shall be the start Attribute
  // Handle of the service definition containing the change and the end
  // Attribute Handle shall be the last Attribute Handle of the service
  // definition containing the change" (Core Spec 2.5.2 Attribute Caching)

  // Send an event to notify us that service was removed/added
  gatt_client_discovery_handle_service_range_change(connection, range);

  // Let's spawn a new discovery
  gatt_client_discovery_discover_range(connection, range);
  return true;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// Server -- Pebble serving up the "Service Changed" characteristic to the remote

// Work-around for iOS issue where sending the indication immediately when iOS subscribes to the
// characteristic is causing problems:
// BTServer: ATT Failed to locate GAP primary service on device ...
#define GATT_SERVICE_CHANGED_INDICATION_DELAY_MS (10000)

#define GATT_SERVICE_CHANGED_INDICATION_MAX_TIMES (5)

static uint32_t s_service_changed_indications_left;

// For unit testing
void gatt_service_changed_server_init(void) {
  s_service_changed_indications_left = 0;
}

void gatt_service_changed_server_handle_fw_update(void) {
  // Once set, just keep it set until the next "normal" reboot.
  // It will cause Pebble to send the "Service Change" indication to be sent every time the other
  // end subscribes to it, causing the remote cache to be invalidated each time and force the
  // remote to do discover services again. However, cap the total number of times we send the
  // "Service Change" indication:
  s_service_changed_indications_left = GATT_SERVICE_CHANGED_INDICATION_MAX_TIMES;
}

void bt_driver_cb_gatt_service_changed_server_confirmation(
    const GattServerChangedConfirmationEvent *event) {
  if (event->status_code != HciStatusCode_Success) {
    PBL_LOG_ERR("Service Changed indication confirmation failure (timed out?) %"PRIu32,
            (uint32_t)event->status_code);
  }
}

void gatt_service_changed_server_cleanup_by_connection(GAPLEConnection *connection) {
  if (connection->gatt_service_changed_indication_timer != TIMER_INVALID_ID) {
    new_timer_delete(connection->gatt_service_changed_indication_timer);
    connection->gatt_service_changed_indication_timer = TIMER_INVALID_ID;
  }
}

static void prv_send_service_changed_indication(void *ctx) {
  GAPLEConnection *connection = (GAPLEConnection *)ctx;

  BTDeviceInternal device;
  bt_lock();
  {
    // The connection may have been torn down between the timer firing and this
    // callback running, leaving a dangling pointer. Bail if it is no longer live.
    if (!connection || !gap_le_connection_is_valid(connection)) {
      bt_unlock();
      return;
    }
    connection->gatt_service_changed_indication_timer = TIMER_INVALID_ID;
    // Copy the device address while holding bt_lock — the driver resolves the
    // NimBLE connection handle through its own connection table, so no bt_lock
    // is needed in the driver path.
    device = connection->device;
  }
  bt_unlock();

  // Invalidate the remote's entire attribute cache so it rediscovers all of our
  // services (see "2.5.2 Attribute Caching" in the BT Core Specification). This
  // mirrors what the client side treats as a "rediscover everything" request.
  const ATTHandleRange range = {
      .start = 0x0001,
      .end = 0xFFFF,
  };
  bt_driver_gatt_send_changed_indication(&device, &range);
}

static void prv_send_indication_timer_cb(void *ctx) {
  GAPLEConnection *connection = (GAPLEConnection *)ctx;
  system_task_add_callback(prv_send_service_changed_indication, connection);
}

void bt_driver_cb_gatt_service_changed_server_subscribe(
    const GattServerSubscribeEvent *event) {
  // Create timer outside of bt_lock to avoid deadlock with NimbleHost.
  // new_timer_create() acquires TaskTimerManager mutex, which may be held by NimbleHost
  // when it's trying to acquire bt_lock, leading to a lock ordering deadlock.
  TimerID timer = TIMER_INVALID_ID;
  const bool subscribed = event->is_subscribing;
  if (subscribed) {
    timer = new_timer_create();
    if (!timer) {
      return;
    }
  }

  bt_lock();
  {
    if (subscribed) {
      PBL_LOG_DBG("Remote subscribed to Service Changed characteristic");

      GAPLEConnection *connection = gap_le_connection_by_addr(&event->dev_address);
      if (!connection || connection->has_sent_gatt_service_changed_indication) {
        // Already sent indication once during the lifetime of this connection, don't send again.
        goto unlock;
      }

      // PRF will always send a "Service Changed" indication:
#if !defined(CONFIG_RECOVERY_FW)
      if (s_service_changed_indications_left <= 0) {
        goto unlock;
      }
#endif

      PBL_LOG_INFO("Indicating Service Changed to remote device");

      // Work-around for iOS issue (see comment above), send the indication after a short delay:
      connection->gatt_service_changed_indication_timer = timer;
      new_timer_start(connection->gatt_service_changed_indication_timer,
                      GATT_SERVICE_CHANGED_INDICATION_DELAY_MS, prv_send_indication_timer_cb,
                      connection, 0);
      timer = TIMER_INVALID_ID; // Timer now owned by connection, don't delete it below
      --s_service_changed_indications_left;
      // Don't send again for this connection:
      connection->has_sent_gatt_service_changed_indication = true;
    }
  }
unlock:
  bt_unlock();
  
  // Clean up timer if we didn't use it
  if (timer != TIMER_INVALID_ID) {
    new_timer_delete(timer);
  }
}

void bt_driver_cb_gatt_service_changed_server_read_subscription(
    const GattServerReadSubscriptionEvent *event) {
  bt_lock();
  {
    bt_driver_gatt_respond_read_subscription(event->transaction_id, 0 /* not subscribed */);
  }
  bt_unlock();
}

void bt_driver_cb_gatt_client_discovery_handle_service_changed(GAPLEConnection *connection,
                                                               uint16_t handle) {
  bt_lock();
  {
    connection->gatt_service_changed_att_handle = handle;
  }
  bt_unlock();
}

//////////////////////////////////
// Prompt commands
//////////////////////////////////

void command_ble_send_service_changed_indication(void) {
  prv_send_service_changed_indication(gap_le_connection_any());
}

void command_ble_rediscover(void) {
  // assume we only have one connection for debug
  GAPLEConnection *conn_hdl = gap_le_connection_any();
  BTDeviceInternal *device = (BTDeviceInternal *) kernel_malloc_check(sizeof(BTDeviceInternal));
  *device = conn_hdl->device;
  system_task_add_callback(prv_rediscover_kernelbg_cb, device);
}
