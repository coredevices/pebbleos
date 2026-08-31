/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "syscall/syscall_internal.h"

#include "applib/bluetooth/ble_client.h"

#include "comm/ble/gap_le_advert.h"
#include "comm/ble/gap_le_connection.h"
#include "comm/ble/gap_le_connect.h"
#include "comm/ble/gap_le_scan.h"

#include "comm/ble/gatt_client_accessors.h"
#include "comm/ble/gatt_client_discovery.h"
#include "comm/ble/gatt_client_operations.h"
#include "comm/ble/gatt_client_subscriptions.h"

// -----------------------------------------------------------------------------
// ble_scan.h

DEFINE_SYSCALL(bool, sys_ble_scan_start, void) {
  return gap_le_start_scan();
}

DEFINE_SYSCALL(bool, sys_ble_scan_stop, void) {
  return gap_le_stop_scan();
}

DEFINE_SYSCALL(bool, sys_ble_scan_is_scanning, void) {
  return gap_le_is_scanning();
}

DEFINE_SYSCALL(bool, sys_ble_consume_scan_results, uint8_t *buffer, uint16_t *size_in_out) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(size_in_out, sizeof(*size_in_out));
    syscall_assert_userspace_buffer(buffer, *size_in_out);
  }

  return gap_le_consume_scan_results(buffer, size_in_out);
}

// -----------------------------------------------------------------------------
// ble_ad_parse.h

DEFINE_SYSCALL(int8_t, sys_ble_get_advertising_tx_power, void) {
  return gap_le_advert_get_tx_power();
}

// -----------------------------------------------------------------------------
// ble_central.h

DEFINE_SYSCALL(BTErrno, sys_ble_central_connect, BTDeviceInternal device,
               bool auto_reconnect, bool is_pairing_required) {
  return gap_le_connect_connect(&device, auto_reconnect, is_pairing_required,
                               GAPLEClientApp);
}

DEFINE_SYSCALL(BTErrno, sys_ble_central_cancel_connect, BTDeviceInternal device) {
  return gap_le_connect_cancel(&device, GAPLEClientApp);
}

// -----------------------------------------------------------------------------
// ble_client.h

DEFINE_SYSCALL(BTErrno, sys_ble_client_discover_services_and_characteristics,
                        BTDeviceInternal device) {
  return gatt_client_discovery_discover_all(&device);
}

DEFINE_SYSCALL(uint8_t, sys_ble_client_copy_services, BTDeviceInternal device,
                        BLEService services[], uint8_t num_services) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(services, sizeof(BLEService) * num_services);
  }

  return gatt_client_copy_service_refs(&device, services, num_services);
}

DEFINE_SYSCALL(uint16_t, sys_ble_client_get_maximum_value_length, BTDeviceInternal device) {
  return gap_le_connection_get_gatt_mtu(&device);
}

DEFINE_SYSCALL(BTErrno, sys_ble_client_read, BLECharacteristic characteristic) {
  return gatt_client_op_read(characteristic, GAPLEClientApp);
}

DEFINE_SYSCALL(bool, sys_ble_client_get_notification_value_length,
                     BLECharacteristic *characteristic_out,
                     uint16_t *value_length_out) {
  if (PRIVILEGE_WAS_ELEVATED) {
    if (characteristic_out) {
      syscall_assert_userspace_buffer(characteristic_out, sizeof(*characteristic_out));
    }
    if (value_length_out) {
      syscall_assert_userspace_buffer(value_length_out, sizeof(*value_length_out));
    }
  }
  GATTBufferedNotificationHeader header;
  const bool has_notification = gatt_client_subscriptions_get_notification_header(GAPLEClientApp,
                                                                                  &header);
  if (has_notification) {
    if (characteristic_out) {
      *characteristic_out = header.characteristic;
    }
    if (value_length_out) {
      *value_length_out = header.value_length;
    }
  }
  return has_notification;
}

DEFINE_SYSCALL(void, sys_ble_client_consume_read, uintptr_t object_ref,
                                                  uint8_t value_out[],
                                                  uint16_t *value_length_in_out) {
  // Snapshot the user-supplied length once so the buffer-size used for
  // validation is the same one passed down to gatt_client_consume_read_response.
  // Re-reading *value_length_in_out a second time would let a racing app thread
  // grow it after we validated the buffer.
  uint16_t value_length;
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(value_length_in_out, sizeof(*value_length_in_out));
    value_length = *value_length_in_out;
    syscall_assert_userspace_buffer(value_out, value_length);
  } else {
    value_length = *value_length_in_out;
  }

  gatt_client_consume_read_response(object_ref, value_out, value_length, GAPLEClientApp);
}

DEFINE_SYSCALL(bool, sys_ble_client_consume_notification, uintptr_t *object_ref_out,
                                                          uint8_t value_out[],
                                                          uint16_t *value_length_in_out,
                                                          bool *has_more_out) {

  // Same snapshot pattern as sys_ble_client_consume_read: the inner function
  // dereferences value_length_in_out multiple times and gates its memcpy() on
  // it, so a racing app could let validation pass on a small value and the
  // memcpy land past the validated buffer. Use a kernel-local copy, then write
  // back the result the inner function would have stored through the pointer.
  uint16_t value_length;
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(object_ref_out, sizeof(*object_ref_out));
    syscall_assert_userspace_buffer(value_length_in_out, sizeof(*value_length_in_out));
    value_length = *value_length_in_out;
    syscall_assert_userspace_buffer(value_out, value_length);
    syscall_assert_userspace_buffer(has_more_out, sizeof(*has_more_out));
  } else {
    value_length = *value_length_in_out;
  }

  bool result = gatt_client_subscriptions_consume_notification(
      object_ref_out, value_out, &value_length, GAPLEClientApp, has_more_out);
  *value_length_in_out = value_length;
  return result;
}

DEFINE_SYSCALL(BTErrno, sys_ble_client_write, BLECharacteristic characteristic,
                                              const uint8_t *value,
                                              size_t value_length) {
  // gatt_client_op_write hands `value`/`value_length` straight to the BLE
  // driver, which transmits those bytes to the peer. Without this check an
  // app can point `value` at any kernel address and use the remote device as
  // a sink for arbitrary kernel memory.
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(value, value_length);
  }
  return gatt_client_op_write(characteristic, value, value_length, GAPLEClientApp);
}

DEFINE_SYSCALL(BTErrno, sys_ble_client_write_without_response,
                        BLECharacteristic characteristic,
                        const uint8_t *value,
                        size_t value_length) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(value, value_length);
  }
  return gatt_client_op_write_without_response(characteristic,
                                               value, value_length, GAPLEClientApp);
}

DEFINE_SYSCALL(BTErrno, sys_ble_client_subscribe, BLECharacteristic characteristic,
                                                  BLESubscription subscription_type) {
  return gatt_client_subscriptions_subscribe(characteristic,
                                             subscription_type,
                                             GAPLEClientApp);
}

DEFINE_SYSCALL(BTErrno, sys_ble_client_write_descriptor, BLEDescriptor descriptor,
                                                         const uint8_t *value,
                                                         size_t value_length) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(value, value_length);
  }
  return gatt_client_op_write_descriptor(descriptor,
                                         value, value_length, GAPLEClientApp);
}

DEFINE_SYSCALL(BTErrno, sys_ble_client_read_descriptor, BLEDescriptor descriptor) {
  return gatt_client_op_read_descriptor(descriptor, GAPLEClientApp);
}

// -----------------------------------------------------------------------------
// ble_service.h

DEFINE_SYSCALL(uint8_t, sys_ble_service_get_characteristics, BLEService service_ref,
                                            BLECharacteristic characteristics_out[],
                                            uint8_t num_characteristics) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(characteristics_out,
                                    sizeof(BLECharacteristic) * num_characteristics);
  }

  return gatt_client_service_get_characteristics(service_ref,
                                                 characteristics_out,
                                                 num_characteristics);
}

DEFINE_SYSCALL(void, sys_ble_service_get_uuid, Uuid *uuid, BLEService service_ref) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(uuid, sizeof(*uuid));
  }
  *uuid = gatt_client_service_get_uuid(service_ref);
}

DEFINE_SYSCALL(void, sys_ble_service_get_device, BTDeviceInternal *device, BLEService service) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(device, sizeof(*device));
  }
  *device = gatt_client_service_get_device(service);
}

DEFINE_SYSCALL(uint8_t, sys_ble_service_get_included_services, BLEService service_ref,
                                              BLEService included_services_out[],
                                              uint8_t num_services) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(included_services_out,
                                    sizeof(BLEService) * num_services);
  }

  return gatt_client_service_get_included_services(service_ref,
                                                 included_services_out,
                                                 num_services);
}

// -----------------------------------------------------------------------------
// ble_characteristic.h

DEFINE_SYSCALL(void, sys_ble_characteristic_get_uuid,
               Uuid *uuid, BLECharacteristic characteristic) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(uuid, sizeof(*uuid));
  }
  *uuid = gatt_client_characteristic_get_uuid(characteristic);
}

DEFINE_SYSCALL(BLEAttributeProperty, sys_ble_characteristic_get_properties,
                                     BLECharacteristic characteristic) {
  return gatt_client_characteristic_get_properties(characteristic);
}

DEFINE_SYSCALL(BLEService, sys_ble_characteristic_get_service, BLECharacteristic characteristic) {
  return gatt_client_characteristic_get_service(characteristic);
}

DEFINE_SYSCALL(void, sys_ble_characteristic_get_device,
               BTDevice *device, BLECharacteristic characteristic) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(device, sizeof(*device));
  }
  *device = gatt_client_characteristic_get_device(characteristic).opaque;
}

DEFINE_SYSCALL(uint8_t, sys_ble_characteristic_get_descriptors,
                        BLECharacteristic characteristic,
                        BLEDescriptor descriptors_out[],
                        uint8_t num_descriptors) {

  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(descriptors_out,
                                    sizeof(BLEDescriptor) * num_descriptors);
  }
  return gatt_client_characteristic_get_descriptors(characteristic,
                                                    descriptors_out,
                                                    num_descriptors);
}

// -----------------------------------------------------------------------------
// ble_descriptor.h

DEFINE_SYSCALL(void, sys_ble_descriptor_get_uuid, Uuid *uuid, BLEDescriptor descriptor) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(uuid, sizeof(*uuid));
  }
  *uuid = gatt_client_descriptor_get_uuid(descriptor);
}

DEFINE_SYSCALL(BLECharacteristic, sys_ble_descriptor_get_characteristic, BLEDescriptor descriptor) {
  return gatt_client_descriptor_get_characteristic(descriptor);
}
