/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kernel_le_client.h"

#if defined(CONFIG_BT_ANCS_CLIENT)
#include "ancs/ancs_definition.h"
#endif
#if defined(CONFIG_BT_AMS_CLIENT)
#include "ams/ams_definition.h"
#endif
#include "app_launch/app_launch_definition.h"
#include "dis/dis_definition.h"
#include "ppogatt/ppogatt_definition.h"
#if UNITTEST
#include "test/test_definition.h"
#endif

#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"

#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/likely.h"

#include "comm/ble/gap_le_connect.h"
#include "comm/ble/gap_le_slave_reconnect.h"
#include "comm/ble/gatt_client_accessors.h"
#include "comm/ble/gatt_client_discovery.h"
#include "comm/ble/gatt_client_operations.h"
#include "comm/ble/gatt_client_subscriptions.h"

#define MAX_SERVICE_INSTANCES (8)

//! Array indices for the different client "classes"
enum {
#if UNITTEST
  KernelLEClientUnitTest = 0,
#else
  KernelLEClientPPoGATT = 0,
#if defined(CONFIG_BT_ANCS_CLIENT)
  KernelLEClientANCS,
#endif
#if defined(CONFIG_BT_AMS_CLIENT)
  KernelLEClientAMS,
#endif
  KernelLEClientAppLaunch,
  KernelLEClientDIS,
#endif
  KernelLEClientNum,
};

typedef struct {
  //! Name of the GATT profile that will be used in debug logs
  const char * const debug_name;
  //! The Service UUID of the remote GATT service
  const Uuid * const service_uuid;
  //! Array of Characteristic UUIDs that are expected to be part of the remote GATT service
  const Uuid * const characteristic_uuids;
  //! The number of elements in the `characteristic_uuids` array
  const uint8_t num_characteristics;
  //! Callback executed every time a BT LE service matching 'service_uuid' is discovered
  //!
  //! @param characteristics - handles for the characteristics discovered.
  //!    The array will be 'num_characteristics' size and ordered the same way
  //!    as the characteristics_uuids array provided
  void (*handle_service_discovered)(BLECharacteristic *characteristics);
  //! Callback executed every time a BT LE service matching 'service_uuid' is removed
  //!
  //!  @param characteristics - An array of all the characteristic handles that
  //!    have been invalidated
  //!  @param num_characteristics - The length of the array
  void (*handle_service_removed)(BLECharacteristic *characteristics, uint8_t num_characteristics);
  //! Invoked when all handles should be flushed by the connection
  //! (events such as a disconnect or full re-discovery will trigger this)
  void (*invalidate_all_references)(void);
  //! Function that is called to test whether the client handles the characteristic, in which case
  //! write/read responses/notifications will be routed to this client (can be NULL)
  bool (*can_handle_characteristic)(BLECharacteristic characteristic);
  //! Handler for GATT read responses and notifications / indications (can be NULL)
  void (*handle_read_or_notification)(BLECharacteristic characteristic, const uint8_t *value,
                                      size_t value_length, BLEGATTError error);
  //! Handler for GATT write responses (can be NULL)
  void (*handle_write_response)(BLECharacteristic characteristic, BLEGATTError error);
  //! Handler for GATT subscription confirmations (can be NULL)
  void (*handle_subscribe)(BLECharacteristic subscribed_characteristic,
                           BLESubscription subscription_type, BLEGATTError error);
} KernelLEClient;

static const KernelLEClient s_clients[KernelLEClientNum] = {
#if UNITTEST
  [KernelLEClientUnitTest] = {
    .debug_name = "TEST",
    .service_uuid = &s_test_service_uuid,
    .characteristic_uuids = s_test_characteristic_uuids,
    .num_characteristics = TestCharacteristicCount,
    .handle_service_discovered = test_client_handle_service_discovered,
    .handle_service_removed = test_client_handle_service_removed,
    .invalidate_all_references = test_client_invalidate_all_references,
    .can_handle_characteristic = test_client_can_handle_characteristic,
    .handle_write_response = test_client_handle_write_response,
    .handle_subscribe = test_client_handle_subscribe,
    .handle_read_or_notification = test_client_handle_read_or_notification,
  },
#else
  [KernelLEClientPPoGATT] = {
    .debug_name = "PPoG",
    .service_uuid = &s_ppogatt_service_uuid,
    .characteristic_uuids = s_ppogatt_characteristic_uuids,
    .num_characteristics = PPoGATTCharacteristicNum,
    .handle_service_discovered = ppogatt_handle_service_discovered,
    .handle_service_removed = ppogatt_handle_service_removed,
    .invalidate_all_references = ppogatt_invalidate_all_references,
    .can_handle_characteristic = ppogatt_can_handle_characteristic,
    .handle_write_response = NULL,
    .handle_subscribe = ppogatt_handle_subscribe,
    .handle_read_or_notification = ppogatt_handle_read_or_notification,
  },
#if defined(CONFIG_BT_ANCS_CLIENT)
  [KernelLEClientANCS] = {
    .debug_name = "ANCS",
    .service_uuid = &s_ancs_service_uuid,
    .characteristic_uuids = s_ancs_characteristic_uuids,
    .num_characteristics = NumANCSCharacteristic,
    .handle_service_discovered = ancs_handle_service_discovered,
    .handle_service_removed = ancs_handle_service_removed,
    .invalidate_all_references = ancs_invalidate_all_references,
    .can_handle_characteristic = ancs_can_handle_characteristic,
    .handle_write_response = ancs_handle_write_response,
    .handle_subscribe = ancs_handle_subscribe,
    .handle_read_or_notification = ancs_handle_read_or_notification,
  },
#endif
#if defined(CONFIG_BT_AMS_CLIENT)
  [KernelLEClientAMS] = {
    .debug_name = "AMS",
    .service_uuid = &s_ams_service_uuid,
    .characteristic_uuids = s_ams_characteristic_uuids,
    .num_characteristics = NumAMSCharacteristic,
    .handle_service_discovered = ams_handle_service_discovered,
    .handle_service_removed = ams_handle_service_removed,
    .invalidate_all_references = ams_invalidate_all_references,
    .can_handle_characteristic = ams_can_handle_characteristic,
    .handle_write_response = ams_handle_write_response,
    .handle_subscribe = ams_handle_subscribe,
    .handle_read_or_notification = ams_handle_read_or_notification,
  },
#endif
  [KernelLEClientAppLaunch] = {
    .debug_name = "Lnch",
    .service_uuid = &s_app_launch_service_uuid,
    .characteristic_uuids = s_app_launch_characteristic_uuids,
    .num_characteristics = AppLaunchCharacteristicNum,
    .handle_service_discovered = app_launch_handle_service_discovered,
    .handle_service_removed = app_launch_handle_service_removed,
    .invalidate_all_references = app_launch_invalidate_all_references,
    .can_handle_characteristic = app_launch_can_handle_characteristic,
    .handle_read_or_notification = NULL,
  },
  [KernelLEClientDIS] = {
    .debug_name = "DIS",
    .service_uuid = &s_dis_service_uuid,
    .characteristic_uuids = s_dis_characteristic_uuids,
    .num_characteristics = NumDISCharacteristic,
    .handle_service_discovered = dis_handle_service_discovered,
    .handle_service_removed = dis_handle_service_removed,
    .invalidate_all_references = dis_invalidate_all_references,
    .can_handle_characteristic = NULL,
    .handle_read_or_notification = NULL,
  },
#endif // UNITTEST
};

static void prv_handle_services_removed(PebbleBLEGATTClientServicesRemoved *services_removed) {
  PebbleBLEGATTClientServiceHandles *service_remove_info = &services_removed->handles[0];
  for (int s = 0; s < services_removed->num_services_removed; s++) {
    for (int c = 0; c < KernelLEClientNum; c++) {
      const KernelLEClient * const client = &s_clients[c];
      if (uuid_equal(&service_remove_info->uuid, client->service_uuid)) {
        client->handle_service_removed(
            (BLECharacteristic *)&service_remove_info->char_and_desc_handles[0],
                                       service_remove_info->num_characteristics);
      }
    }

    int num_hdls = service_remove_info->num_descriptors +
        service_remove_info->num_characteristics;
    service_remove_info =
        (PebbleBLEGATTClientServiceHandles *)&service_remove_info->char_and_desc_handles[num_hdls];
  }
}

static void prv_handle_all_services_invalidated(void) {
  for (int c = 0; c < KernelLEClientNum; c++) {
    const KernelLEClient * const client = &s_clients[c];
    client->invalidate_all_references();
  }
}

static void prv_handle_services_added(
    PebbleBLEGATTClientServicesAdded *added_services, BTDeviceInternal *device) {
  // loop through the new services
  for (int s = 0; s < added_services->num_services_added; s++) {
    // get the uuid for the service
    Uuid service_uuid = gatt_client_service_get_uuid(added_services->services[s]);

    // are any clients looking for this uuid?
    for (int c = 0; c < KernelLEClientNum; c++) {
      const KernelLEClient * const client = &s_clients[c];

      if (!uuid_equal(&service_uuid, (const Uuid *)client->service_uuid)) {
        continue;
      }

      // We have found a service that a client is looking for. Make sure the
      // characteristics we want are present and if so notify the interested client about it
      BLECharacteristic characteristics[client->num_characteristics];
      const uint8_t num_characteristics =
          gatt_client_service_get_characteristics_matching_uuids(
              added_services->services[s], &characteristics[0], client->characteristic_uuids,
              client->num_characteristics);

      if (num_characteristics != client->num_characteristics) {
        PBL_LOG_ERR("Found %s, but only %u characteristics...",
                client->debug_name, num_characteristics);
        continue;
      }

      client->handle_service_discovered(characteristics);
    }
  }
}

static void prv_handle_gatt_service_discovery_event(const PebbleBLEGATTClientServiceEvent *event) {
  PebbleBLEGATTClientServiceEventInfo *event_info = event->info;
  if (event_info->status == BTErrnoServiceDiscoveryDisconnected) {
    // TODO: In the past we'd disconnect when service discovery
    // failed (not due to a disconnection)
    return;
  }
  if (event_info->status != BTErrnoServiceDiscoveryDatabaseChanged &&
      event_info->status != BTErrnoOK) {
    // gatt_client_discovery.c already logs errors for this condition
    return;
  }

  if (event_info->type != PebbleServicesRemoved) {
    // For removals, we log info in the handler routine
    PBL_LOG_INFO("Service changed Indication: type: %d status: %d",
            event_info->type, event_info->status);
  }

  switch (event_info->type) {
    case PebbleServicesRemoved:
      prv_handle_services_removed(&event_info->services_removed_data);
      break;
    case PebbleServicesInvalidateAll:
      prv_handle_all_services_invalidated();
      break;
    case PebbleServicesAdded:
      prv_handle_services_added(&event_info->services_added_data, &event_info->device);
      break;
    default:
      WTF;
  }
}

static const KernelLEClient * prv_client_for_characteristic(BLECharacteristic characteristic) {
  for (int c = 0; c < KernelLEClientNum; ++c) {
    const KernelLEClient * const client = &s_clients[c];
    if (client->can_handle_characteristic && client->can_handle_characteristic(characteristic)) {
      return client;
    }
  }
  return NULL;
}

typedef void (*ConsumeFuncPtr)(BLECharacteristic characteristic_ref,
                            uint8_t *value_out, uint16_t value_length, GAPLEClient client);

typedef void (*ReadNotifyHandler)(BLECharacteristic characteristic, const uint8_t *value,
                                  size_t value_length, BLEGATTError error);

static void prv_consume_read_response(const PebbleBLEGATTClientEvent *event,
                                      const KernelLEClient *client) {
  const uint16_t value_length = event->value_length;
  uint8_t *buffer = NULL;

  if (value_length) {
    // This is ugly and causes double-copying the data...
    // TODO: https://pebbletechnology.atlassian.net/browse/PBL-14164
    buffer = (uint8_t *) kernel_malloc(value_length);
    if (UNLIKELY(!buffer)) {
      PBL_LOG_ERR("OOM for GATT read response - %d bytes", (int)value_length);
      return;
    }
    gatt_client_consume_read_response(event->object_ref,
                                      buffer, value_length, GAPLEClientKernel);
  }

  if (client->handle_read_or_notification) {
    client->handle_read_or_notification(event->object_ref, buffer,
                                        value_length, event->gatt_error);
  }
  kernel_free(buffer);
}

static void prv_consume_notifications(const PebbleBLEGATTClientEvent *event) {
  GATTBufferedNotificationHeader header = {};
  bool has_more = gatt_client_subscriptions_get_notification_header(GAPLEClientKernel, &header);
  const RtcTicks start_ticks = rtc_get_ticks();
  while (has_more) {
    const uint32_t ticks_spent = rtc_get_ticks() - start_ticks;

    // Don't spend more than ~33ms (or one 30fps animation frame interval) processing the pending
    // GATT notifications:
    if (ticks_spent >= ((RTC_TICKS_HZ * 33) / 1000)) {
      // Doing this might actually cause an issue if the characteristic(s) for which there are still
      // notifications pending in the buffer become invalid before the time they are processed.
      // Probably not a big deal.
      gatt_client_subscriptions_reschedule(GAPLEClientKernel);
      return;  // yield
    }

    // This is ugly and causes double-copying the data...
    // TODO: https://pebbletechnology.atlassian.net/browse/PBL-14164
    uint8_t *buffer = (uint8_t *) kernel_malloc(header.value_length);
    if (UNLIKELY(header.value_length && !buffer)) {
      PBL_LOG_ERR("OOM for GATT notification");
      return;
    }

    const uint16_t next_value_length =
            gatt_client_subscriptions_consume_notification(&header.characteristic,
                                                           buffer, &header.value_length,
                                                           GAPLEClientKernel, &has_more);

    const KernelLEClient * const client = prv_client_for_characteristic(header.characteristic);
    if (client->handle_read_or_notification) {
      client->handle_read_or_notification(header.characteristic, buffer, header.value_length,
                                          BLEGATTErrorSuccess);
    } else {
      PBL_LOG_DBG("No client to handle GATT notification from characteristic %p",
              (void*) header.characteristic);
    }
    kernel_free(buffer);
    header.value_length = next_value_length;
  }
}

static void prv_handle_gatt_event(const PebbleBLEGATTClientEvent *event) {
  if (event->subtype == PebbleBLEGATTClientEventTypeBufferEmpty) {
    // Taking a shortcut here:
    ppogatt_handle_buffer_empty();
    return;
  } else if (event->subtype == PebbleBLEGATTClientEventTypeNotification) {
    prv_consume_notifications(event);
    return;
  }

  const KernelLEClient * const client = prv_client_for_characteristic(event->object_ref);
  if (!client) {
    // Read responses still need to be consumed, even if the client has disappeared:
    if (event->subtype == PebbleBLEGATTClientEventTypeCharacteristicRead && event->value_length) {
      gatt_client_consume_read_response(event->object_ref,
                                        NULL, event->value_length, GAPLEClientKernel);
    }
    goto log_error;
  }

  switch (event->subtype) {
    case PebbleBLEGATTClientEventTypeCharacteristicWrite:
      if (!client->handle_write_response) {
        goto log_error;
      }
      client->handle_write_response(event->object_ref, event->gatt_error);
      return;

    case PebbleBLEGATTClientEventTypeCharacteristicSubscribe:
      if (!client->handle_subscribe) {
        goto log_error;
      }
      client->handle_subscribe(event->object_ref, event->subscription_type, event->gatt_error);
      return;

    case PebbleBLEGATTClientEventTypeCharacteristicRead:
      if (!client->handle_read_or_notification) {
        goto log_error;
      }
      prv_consume_read_response(event, client);
      return;

    default:
      break;
  }

log_error:
  PBL_LOG_ERR("Unhandled GATT event:%u ref:%"PRIu32" err:%"PRIu16" len:%"PRIu16" cl:%p",
          event->subtype,
          (uint32_t)event->object_ref,
          (uint16_t)event->gatt_error,
          (uint16_t)event->value_length,
          client);
}

static void prv_handle_connection_event(const PebbleBLEConnectionEvent *event) {
  PBL_LOG_DBG("PEBBLE_BLE_CONNECTION_EVENT: reason=0x%x, conn=%u, bond=%u",
          event->hci_reason, event->connected, event->bonding_id);

  const bool connected = event->connected;
  // FIXME: When PPoGATT is supported add a check for active gateway
  // https://pebbletechnology.atlassian.net/browse/PBL-15277
  //
  // For now, we just assume that the Kernel LE client is _always_ bonded for
  // ANCS. Note that we cannot use bt_persistent_storage calls in this routine because
  // we could be getting this call as a result of a disconnect due to
  // forgetting a pairing key

  const BTDeviceInternal device = PebbleEventToBTDeviceInternal(event);
  if (connected) {
    PBL_LOG_DBG("Connected to Gateway!");

#if defined(CONFIG_BT_ANCS_CLIENT)
    ancs_create();
#endif
#if defined(CONFIG_BT_AMS_CLIENT)
    ams_create();
#endif
    ppogatt_create();

    gap_le_slave_reconnect_stop();
    gatt_client_discovery_discover_all(&device);

  } else {
    PBL_LOG_DBG("Disconnected from Gateway!");
    ppogatt_destroy();
#if defined(CONFIG_BT_AMS_CLIENT)
    ams_destroy();
#endif
#if defined(CONFIG_BT_ANCS_CLIENT)
    ancs_destroy();
#endif
    app_launch_handle_disconnection();
    gap_le_slave_reconnect_start();
    gatt_client_op_cleanup(GAPLEClientKernel);
  }
}

// -------------------------------------------------------------------------------------------------
void kernel_le_client_handle_event(const PebbleEvent *e) {
  switch (e->type) {
    case PEBBLE_BLE_SCAN_EVENT:
      PBL_LOG_DBG("PEBBLE_BLE_SCAN_EVENT");
      return;

    case PEBBLE_BLE_CONNECTION_EVENT:
      prv_handle_connection_event(&e->bluetooth.le.connection);
      return;

    case PEBBLE_BLE_GATT_CLIENT_EVENT:
      if (e->bluetooth.le.gatt_client.subtype == PebbleBLEGATTClientEventTypeServiceChange) {
        prv_handle_gatt_service_discovery_event(&e->bluetooth.le.gatt_client_service);
      } else {
        prv_handle_gatt_event(&e->bluetooth.le.gatt_client);
      }
      return;

    default:
      return;
  }
}

// -------------------------------------------------------------------------------------------------
static void prv_connect_gateway_bonding(BTBondingID gateway_bonding) {
  gap_le_slave_reconnect_start();
  gap_le_connect_connect_by_bonding(gateway_bonding, true /* auto_reconnect */,
                                 true /* is_pairing_required */, GAPLEClientKernel);
}

// -------------------------------------------------------------------------------------------------
static void prv_cancel_connect_gateway_bonding(BTBondingID gateway_bonding) {
  gap_le_slave_reconnect_stop();
  // FIXME: Redundant? since gap_le_connect will also clean up?
  gap_le_connect_cancel_by_bonding(gateway_bonding, GAPLEClientKernel);
}

// -------------------------------------------------------------------------------------------------
static void prv_cleanup_clients_kernel_main_cb(void *unused) {
#if defined(CONFIG_BT_ANCS_CLIENT)
  ancs_destroy();
#endif
#if defined(CONFIG_BT_AMS_CLIENT)
  ams_destroy();
#endif
}

// -------------------------------------------------------------------------------------------------
void kernel_le_client_handle_bonding_change(BTBondingID bonding, BtPersistBondingOp op) {
  if (bt_persistent_storage_is_ble_ancs_bonding(bonding)) {
    if (op == BtPersistBondingOpWillDelete) {
      prv_cancel_connect_gateway_bonding(bonding);
    } else if (op == BtPersistBondingOpDidAdd) {
      prv_connect_gateway_bonding(bonding);
    }
  }
}

// -------------------------------------------------------------------------------------------------
void kernel_le_client_init(void) {
  // Reset analytics
  ppogatt_reset_disconnect_counter();

  BTBondingID gateway_bonding = bt_persistent_storage_get_ble_ancs_bonding();
  if (gateway_bonding != BT_BONDING_ID_INVALID) {
    prv_connect_gateway_bonding(gateway_bonding);
  }
}

// -------------------------------------------------------------------------------------------------
void kernel_le_client_deinit(void) {
  // Cleanup clients: their code must execute on KernelMain, so add callback:
  launcher_task_add_callback(prv_cleanup_clients_kernel_main_cb, NULL);

  gap_le_slave_reconnect_stop();
  gap_le_connect_cancel_all(GAPLEClientKernel);
  ppogatt_destroy();
}
