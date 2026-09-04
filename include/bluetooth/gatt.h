/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <bluetooth/bluetooth_types.h>
#include <bluetooth/gatt_discovery.h>
#include <bluetooth/hci_types.h>

#include "comm/ble/gap_le_connection.h"

// -- Gatt Device/Server Events

#define GATT_SERVICE_UUID ((uint16_t) 0x1801)
#define GATT_SERVICE_CHANGED_CHARACTERISTIC_UUID ((uint16_t) 0x2A05)
#define GATT_CCCD_UUID ((uint16_t) 0x2902)

//! Using BTDeviceAddress instead of BTDeviceInternal, with all these events, because Bluetopia's
//! events doesn't contain the address type.

typedef struct GattDeviceConnectionEvent {
  BTDeviceAddress dev_address;
  uint32_t connection_id;
  uint16_t mtu;
} GattDeviceConnectionEvent;

typedef struct GattDeviceDisconnectionEvent {
  BTDeviceAddress dev_address;
} GattDeviceDisconnectionEvent;

typedef struct GattDeviceBufferEmptyEvent {
  BTDeviceAddress dev_address;
} GattDeviceBufferEmptyEvent;

typedef struct GattServerNotifIndicEvent {
  BTDeviceAddress dev_address;
  uint16_t attr_handle;
  uint16_t attr_val_len;
  uint8_t *attr_val;
  void *context;
} GattServerNotifIndicEvent;

typedef struct GattDeviceMtuUpdateEvent {
  BTDeviceAddress dev_address;
  uint16_t mtu;
} GattDeviceMtuUpdateEvent;

// -- Service Changed Events

typedef struct GattServerChangedConfirmationEvent {
  BTDeviceAddress dev_address;
  uint32_t connection_id;
  uint32_t transaction_id;
  HciStatusCode status_code;
} GattServerChangedConfirmationEvent;

typedef struct GattServerReadSubscriptionEvent {
  BTDeviceAddress dev_address;
  uint32_t connection_id;
  uint32_t transaction_id;
} GattServerReadSubscriptionEvent;

typedef struct GattServerSubscribeEvent {
  BTDeviceAddress dev_address;
  uint32_t connection_id;
  bool is_subscribing;
} GattServerSubscribeEvent;

// -- Gatt Client Operations

typedef enum GattClientOpResponseType {
  GattClientOpResponseRead,
  GattClientOpResponseWrite,
} GattClientOpResponseType;

typedef struct GattClientOpResponseHdr {
  GattClientOpResponseType type;
  BLEGATTError error_code;
  void *context;
} GattClientOpResponseHdr;

typedef struct GattClientOpReadReponse {
  GattClientOpResponseHdr hdr;
  uint16_t value_length;
  uint8_t *value;
} GattClientOpReadReponse;

typedef struct GattClientOpWriteReponse {
  GattClientOpResponseHdr hdr;
} GattClientOpWriteReponse;

// -- Gatt Data Structures

void bt_driver_gatt_acknowledge_indication(uint32_t connection_id, uint32_t transaction_id);

// TODO: This will probably need to be changed for the Dialog chip (doesn't have transaction ids)
void bt_driver_gatt_respond_read_subscription(uint32_t transaction_id, uint16_t response_code);

void bt_driver_gatt_send_changed_indication(const BTDeviceInternal *device,
                                            const ATTHandleRange *data);


BTErrno bt_driver_gatt_write_without_response(GAPLEConnection *connection,
                                              const uint8_t *value,
                                              size_t value_length,
                                              uint16_t att_handle);

BTErrno bt_driver_gatt_write(GAPLEConnection *connection,
                             const uint8_t *value,
                             size_t value_length,
                             uint16_t att_handle,
                             void *context);

BTErrno bt_driver_gatt_read(GAPLEConnection *connection,
                                   uint16_t att_handle,
                                   void *context);

//! The following are callbacks that the bt_driver implementation will call when handling events.

//! gatt callbacks
extern void bt_driver_cb_gatt_handle_connect(const GattDeviceConnectionEvent *event);

extern void bt_driver_cb_gatt_handle_disconnect(const GattDeviceDisconnectionEvent *event);

extern void bt_driver_cb_gatt_handle_buffer_empty(const GattDeviceBufferEmptyEvent *event);

extern void bt_driver_cb_gatt_handle_mtu_update(const GattDeviceMtuUpdateEvent *event);

extern void bt_driver_cb_gatt_handle_notification(const GattServerNotifIndicEvent *event);

//! @note The indication is unconditionally confirmed within the bt_driver as soon as one is
//!        received.
extern void bt_driver_cb_gatt_handle_indication(const GattServerNotifIndicEvent *event);

//! gatt_service_changed callbacks
extern void bt_driver_cb_gatt_service_changed_server_confirmation(
    const GattServerChangedConfirmationEvent *event);

extern void bt_driver_cb_gatt_service_changed_server_subscribe(
    const GattServerSubscribeEvent *event);

extern void bt_driver_cb_gatt_service_changed_server_read_subscription(
    const GattServerReadSubscriptionEvent *event);

extern void bt_driver_cb_gatt_client_discovery_handle_service_changed(GAPLEConnection *connection,
                                                                      uint16_t handle);

extern void bt_driver_cb_gatt_client_operations_handle_response(GattClientOpResponseHdr *event);
