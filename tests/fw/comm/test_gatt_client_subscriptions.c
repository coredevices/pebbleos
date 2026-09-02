/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <bluetooth/gatt.h>

#include "comm/ble/gatt_client_subscriptions.h"
#include "comm/ble/gap_le_connection.h"
#include "comm/ble/gap_le_task.h"
#include "comm/ble/gatt_service_changed.h"

#include "clar.h"

#include <pbl/btutil/bt_device.h>
#include <pbl/btutil/bt_uuid.h>

#include "pbl/kernel/sem.h"

// Fakes
///////////////////////////////////////////////////////////

#include "fake_events.h"
#include "fake_pbl_malloc.h"
#include "fake_bt_driver_gatt.h"
#include "fake_new_timer.h"
#include "fake_queue.h"
#include "fake_sem.h"
#include "fake_system_task.h"

#include "fake_event_gatt_service_buffer.h"

#include "stubs_regular_timer.h"

static BTErrno s_write_descriptor_cccd_result;
static BLEDescriptor s_last_cccd_ref;
static uint16_t s_last_cccd_value;

BTErrno gatt_client_op_write_descriptor_cccd(BLEDescriptor cccd, const uint16_t *value) {
  s_last_cccd_ref = cccd;
  s_last_cccd_value = *value;
  return s_write_descriptor_cccd_result;
}

// FIXME: PBL-23945
void fake_kernel_malloc_mark(void) { }
void fake_kernel_malloc_mark_assert_equal(void) { }

// Stubs
///////////////////////////////////////////////////////////

#include "stubs_analytics.h"
#include "stubs_bt_lock.h"
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_rand_ptr.h"
#include "stubs_tick.h"

void core_dump_reset(bool is_forced) {
}

void launcher_task_add_callback(void (*callback)(void *data), void *data) {
  callback(data);
}

uint16_t gaps_get_starting_att_handle(void) {
  return 4;
}

// Helpers
///////////////////////////////////////////////////////////

extern void gatt_client_subscriptions_handle_server_notification(GAPLEConnection *connection,
                                                                 uint16_t att_handle,
                                                                 const uint8_t *att_value,
                                                                 uint16_t att_length);

extern void gatt_client_subscriptions_handle_write_cccd_response(BLEDescriptor cccd,
                                                                 BLEGATTError error);

extern uint16_t gatt_client_characteristic_get_handle_and_connection(
                                                               BLECharacteristic characteristic_ref,
                                                               GAPLEConnection **connection);

extern struct pbl_sem * gatt_client_subscription_get_semaphore(void);
extern void gatt_client_subscription_cleanup(void);

#define TEST_GATT_CONNECTION_ID (1234)
#define BOGUS_CHARACTERISTIC ((BLECharacteristic) 888)
#define BOGUS_ATT_HANDLE ((uint16_t) 0xffff)

static BTDeviceInternal s_device;
static GAPLEConnection *s_connection;
static uint16_t s_handle;

static BTDeviceInternal prv_dummy_device(uint8_t octet) {
  BTDeviceAddress address = {
    .octets = {
      [0] = octet,
      [1] = octet,
      [2] = octet,
      [3] = octet,
      [4] = octet,
      [5] = octet,
    },
  };
  BTDevice device = bt_device_init_with_address(address, true /* is_random */);
  return *(BTDeviceInternal *)(&device);
}

static BTDeviceInternal prv_connected_dummy_device(uint8_t octet) {
  BTDeviceInternal device = prv_dummy_device(octet);
  gap_le_connection_add(&device, NULL, true /* local_is_master */, TIMER_INVALID_ID);
  s_connection = gap_le_connection_by_device(&device);
  s_connection->gatt_connection_id = TEST_GATT_CONNECTION_ID;
  return device;
}

static void prv_assert_no_event(void) {
  PebbleEvent event = fake_event_get_last();
  cl_assert_equal_i(event.type, PEBBLE_NULL_EVENT);
}

static void prv_assert_subscription_event(BLECharacteristic characteristic,
                                          BLESubscription subscription_type,
                                          BLEGATTError error, bool kernel, bool app) {
  PebbleEvent event = fake_event_get_last();
  cl_assert_equal_i(event.type, PEBBLE_BLE_GATT_CLIENT_EVENT);
  cl_assert_equal_i(event.bluetooth.le.gatt_client.subtype,
                    PebbleBLEGATTClientEventTypeCharacteristicSubscribe);
  cl_assert_equal_i(event.bluetooth.le.gatt_client.subscription_type, subscription_type);
  cl_assert_equal_i(event.bluetooth.le.gatt_client.object_ref, characteristic);
  cl_assert_equal_i(event.bluetooth.le.gatt_client.gatt_error, error);
  PebbleTaskBitset task_mask = ~0;
  if (kernel) {
    task_mask &= ~gap_le_pebble_task_bit_for_client(GAPLEClientKernel);
  }
  if (app) {
    task_mask &= ~gap_le_pebble_task_bit_for_client(GAPLEClientApp);
  }
  cl_assert_equal_i(event.task_mask, task_mask);
}

static void prv_assert_notification_event_ext(BLECharacteristic characteristic,
                                              const uint8_t *value, uint16_t assert_value_length,
                                              bool kernel, bool app, bool should_consume) {
  PebbleEvent event = fake_event_get_last();
  cl_assert_equal_i(event.type, PEBBLE_BLE_GATT_CLIENT_EVENT);
  cl_assert_equal_i(event.bluetooth.le.gatt_client.subtype,
                    PebbleBLEGATTClientEventTypeNotification);
  PebbleTaskBitset task_mask = ~0;
  if (kernel) {
    task_mask &= ~gap_le_pebble_task_bit_for_client(GAPLEClientKernel);
  }
  if (app) {
    task_mask &= ~gap_le_pebble_task_bit_for_client(GAPLEClientApp);
  }
  cl_assert_equal_i(event.task_mask, task_mask);

  BLECharacteristic characteristic_out;
  uint8_t *buffer = (uint8_t *) malloc(assert_value_length);
  memset(buffer, 0, assert_value_length);
  if (kernel) {
    GATTBufferedNotificationHeader header = {};
    gatt_client_subscriptions_get_notification_header(GAPLEClientKernel, &header);
    cl_assert_equal_i(header.value_length, assert_value_length);
    cl_assert_equal_i(header.characteristic, characteristic);

    if (should_consume) {
      uint16_t value_length = assert_value_length;
      gatt_client_subscriptions_consume_notification(&characteristic_out, buffer, &value_length,
                                                     GAPLEClientKernel, NULL);
      cl_assert_equal_i(memcmp(buffer, value, value_length), 0);
      cl_assert_equal_i(characteristic_out, characteristic);
    }
  }
  memset(buffer, 0, assert_value_length);
  if (app) {
    GATTBufferedNotificationHeader header = {};
    gatt_client_subscriptions_get_notification_header(GAPLEClientApp, &header);
    cl_assert_equal_i(header.value_length, assert_value_length);
    cl_assert_equal_i(header.characteristic, characteristic);

    if (should_consume) {
      uint16_t value_length = assert_value_length;
      gatt_client_subscriptions_consume_notification(&characteristic_out, buffer, &value_length,
                                                     GAPLEClientApp, NULL);
      cl_assert_equal_i(memcmp(buffer, value, value_length), 0);
      cl_assert_equal_i(characteristic_out, characteristic);
    }
  }
  free(buffer);
}

static void prv_assert_notification_event(BLECharacteristic characteristic,
                                          const uint8_t *value, uint16_t assert_value_length,
                                          bool kernel, bool app) {
  prv_assert_notification_event_ext(characteristic, value, assert_value_length, kernel, app,
                                    true /* should_consume */);
}

static void prv_simulate_and_assert_discovery_of_one_service(const BTDeviceInternal *device) {
  // Simulate discovery of Blood Pressure service:
  fake_gatt_put_discovery_indication_blood_pressure_service(TEST_GATT_CONNECTION_ID);
  // Simulate discovery of random 128-bit service:
  fake_gatt_put_discovery_indication_random_128bit_uuid_service(TEST_GATT_CONNECTION_ID);
  fake_gatt_put_discovery_complete_event(GATT_SERVICE_DISCOVERY_STATUS_SUCCESS,
                                         TEST_GATT_CONNECTION_ID);
}

static BLECharacteristic prv_get_indicatable_characteristic(void) {
  // The Blood Pressure Service UUID:
  Uuid service_uuid = bt_uuid_expand_16bit(0x1810);
  BLEService service;
  uint8_t num_copied = gatt_client_copy_service_refs_matching_uuid(&s_device, &service, 1,
                                                                   &service_uuid);
  cl_assert_equal_i(num_copied, 1);

  // UUID for indicatable Pressure Measurement characteristic:
  Uuid characteristic_uuid = bt_uuid_expand_16bit(0x2a35);
  BLECharacteristic characteristic;
  num_copied = gatt_client_service_get_characteristics_matching_uuids(service, &characteristic,
                                                                      &characteristic_uuid, 1);
  cl_assert_equal_i(num_copied, 1);

  GAPLEConnection *connection;
  s_handle = gatt_client_characteristic_get_handle_and_connection(characteristic, &connection);

  return characteristic;
}

static void prv_confirm_cccd_write(BLEGATTError error) {
  gatt_client_subscriptions_handle_write_cccd_response(s_last_cccd_ref, error);
}

// Tests
///////////////////////////////////////////////////////////

void test_gatt_client_subscriptions__initialize(void) {
  s_write_descriptor_cccd_result = BTErrnoOK;

  fake_pbl_malloc_clear_tracking();
  fake_event_init();
  gap_le_connection_init();
  gatt_client_subscription_boot();

  // Prepare connected device with Blood Pressure GATT service discovered:
  s_device = prv_connected_dummy_device(1);
  cl_assert_equal_i(gatt_client_discovery_discover_all(&s_device), BTErrnoOK);
  prv_simulate_and_assert_discovery_of_one_service(&s_device);

  fake_event_clear_last();
}

void test_gatt_client_subscriptions__cleanup(void) {
  for (GAPLEClient c = 0; c < GAPLEClientNum; ++c) {
    gatt_client_subscriptions_cleanup_by_client(c);
  }
  gap_le_connection_deinit();
  gatt_client_subscription_cleanup();

  fake_pbl_malloc_check_net_allocs();
  fake_pbl_malloc_clear_tracking();
}

// -------------------------------------------------------------------------------------------------
// gatt_client_subscriptions_subscribe

void test_gatt_client_subscriptions__subscribe_invalid_characteristic(void) {
  fake_kernel_malloc_mark();
  BTErrno e = gatt_client_subscriptions_subscribe(BOGUS_CHARACTERISTIC,
                                                  BLESubscriptionAny, GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoInvalidParameter);
  fake_kernel_malloc_mark_assert_equal();
  prv_assert_no_event();
}

void test_gatt_client_subscriptions__subscribe_no_cccd(void) {
  // The random 128-bit Service UUID:
  Uuid service_uuid = UuidMake(0xF7, 0x68, 0x09, 0x5B, 0x1B, 0xFA, 0x4F, 0x63,
                               0x97, 0xEE, 0xFD, 0xED, 0xAC, 0x66, 0xF9, 0xB0);
  BLEService service;
  uint8_t num_copied = gatt_client_copy_service_refs_matching_uuid(&s_device, &service, 1,
                                                                   &service_uuid);
  cl_assert_equal_i(num_copied, 1);

  // UUID for Characteristic that has no CCCD:
  Uuid characteristic_uuid = UuidMake(0xF7, 0x68, 0x09, 0x5B, 0x1B, 0xFA, 0x4F, 0x63,
                                      0x97, 0xEE, 0xFD, 0xED, 0xAC, 0x66, 0xF9, 0xB1);
  BLECharacteristic characteristic;
  num_copied = gatt_client_service_get_characteristics_matching_uuids(service, &characteristic,
                                                                      &characteristic_uuid, 1);
  cl_assert_equal_i(num_copied, 1);

  fake_kernel_malloc_mark();
  // Try to subscribe to the non-subscribe-able characteristic:
  BTErrno e = gatt_client_subscriptions_subscribe(characteristic,
                                                  BLESubscriptionAny, GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoInvalidParameter);

  fake_kernel_malloc_mark_assert_equal();
  prv_assert_no_event();
}

void test_gatt_client_subscriptions__subscribe_unsupported_subscription_type(void) {
  BTErrno e;
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();

  fake_kernel_malloc_mark();
  // Try to subscribe for notifications to the indicatable (but not notify-able) characteristic:
  e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionNotifications,
                                          GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoInvalidParameter);
  fake_kernel_malloc_mark_assert_equal();
  prv_assert_no_event();

  // Try to subscribe for indications to the indicatable characteristic:
  e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                          GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoOK);
}

void test_gatt_client_subscriptions__subscribe_already_subscribed(void) {
  BTErrno e;
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();

  e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                                  GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoOK);

  fake_event_clear_last();
  fake_kernel_malloc_mark();
  // Subscribe again:
  e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                          GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoInvalidState);
  fake_kernel_malloc_mark_assert_equal();
  prv_assert_no_event();
}

void test_gatt_client_subscriptions__unsubscribe_pending_subscription(void) {
  BTErrno e;
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();

  e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                          GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoOK);

  fake_event_clear_last();
  fake_kernel_malloc_mark();
  // Un-subscribe, while subscribing process is still pending:
  e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionNone, GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoInvalidState);
  fake_kernel_malloc_mark_assert_equal();
  prv_assert_no_event();
}

void test_gatt_client_subscriptions__subscribe_oom_for_subscription_allocation(void) {
  fake_kernel_malloc_mark();
  fake_malloc_set_largest_free_block(sizeof(GATTClientSubscriptionNode) - 1);
  BTErrno e = gatt_client_subscriptions_subscribe(prv_get_indicatable_characteristic(),
                                          BLESubscriptionIndications,
                                          GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoNotEnoughResources);
  fake_kernel_malloc_mark_assert_equal();
  prv_assert_no_event();
}

void test_gatt_client_subscriptions__subscribe_oom_for_buffer_allocation(void) {
  fake_kernel_malloc_mark();
  fake_malloc_set_largest_free_block(sizeof(GATTClientSubscriptionNode));
  BTErrno e = gatt_client_subscriptions_subscribe(prv_get_indicatable_characteristic(),
                                                  BLESubscriptionIndications,
                                                  GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoNotEnoughResources);
  fake_kernel_malloc_mark_assert_equal();
  prv_assert_no_event();
}

void test_gatt_client_subscriptions__subscribe_cccd_write_error(void) {
  s_write_descriptor_cccd_result = -1;

  fake_kernel_malloc_mark();
  // Try to subscribe for indications to the indicatable characteristic:
  BTErrno e = gatt_client_subscriptions_subscribe(prv_get_indicatable_characteristic(),
                                                  BLESubscriptionIndications,
                                                  GAPLEClientKernel);
  cl_assert_equal_i(e, -1);
  fake_kernel_malloc_mark_assert_equal();
  prv_assert_no_event();
}

void test_gatt_client_subscriptions__unsubscribe_no_clients_subscribed(void) {
  fake_kernel_malloc_mark();
  BTErrno e;
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();
  e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionNone, GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoInvalidState);
  fake_kernel_malloc_mark_assert_equal();
  prv_assert_no_event();
}

void test_gatt_client_subscriptions__unsubscribe_not_subscribed_but_other_client_is(void) {
  BTErrno e;
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();
  e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications, GAPLEClientApp);
  cl_assert_equal_i(e, BTErrnoOK);

  fake_kernel_malloc_mark();
  e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionNone, GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoInvalidState);
  fake_kernel_malloc_mark_assert_equal();
  prv_assert_no_event();
}

void test_gatt_client_subscriptions__subscribe_first_subscriber(void) {
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();
  BTErrno e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                                  GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoOK);
  cl_assert_equal_i(s_last_cccd_value, BLESubscriptionIndications);

  prv_confirm_cccd_write(BLEGATTErrorSuccess);
  prv_assert_subscription_event(characteristic, BLESubscriptionIndications, BLEGATTErrorSuccess,
                   true /* kernel */, false /* app */);
}

void test_gatt_client_subscriptions__subscribe_not_first_subscriber(void) {
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();
  // Subscribe kernel:
  BTErrno e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                                  GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoOK);
  cl_assert_equal_i(s_last_cccd_value, BLESubscriptionIndications);

  // Simulate getting confirmation from remote:
  prv_confirm_cccd_write(BLEGATTErrorSuccess);
  fake_event_clear_last();

  // Subscribe app:
  e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                          GAPLEClientApp);
  // App should get event immediately:
  prv_assert_subscription_event(characteristic, BLESubscriptionIndications, BLEGATTErrorSuccess,
                   false /* kernel */, true /* app */);
}

void test_gatt_client_subscriptions__two_subscribers_before_cccd_write_response(void) {
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();
  // Subscribe kernel:
  BTErrno e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                                  GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoOK);
  cl_assert_equal_i(s_last_cccd_value, BLESubscriptionIndications);

  s_last_cccd_value = ~0;

  // Subscribe app:
  e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                          GAPLEClientApp);
  cl_assert_equal_i(e, BTErrnoOK);
  // Should not be written again, because BLESubscriptionIndications was already written:
  cl_assert_equal_i(s_last_cccd_value, (uint16_t) ~0);

  fake_event_clear_last();

  // Simulate getting confirmation from remote:
  prv_confirm_cccd_write(BLEGATTErrorSuccess);

  // Expect one event for both Kernel + App:
  prv_assert_subscription_event(characteristic, BLESubscriptionIndications, BLEGATTErrorSuccess,
                   true /* kernel */, true /* app */);
}

void test_gatt_client_subscriptions__unsubscribe_last_subscriber(void) {
  BTErrno e;
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();

  fake_kernel_malloc_mark();

  // Subscribe kernel:
  e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                          GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoOK);

  // Simulate getting confirmation from remote:
  prv_confirm_cccd_write(BLEGATTErrorSuccess);
  fake_event_clear_last();

  // Unsubscribe kernel:
  e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionNone, GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoOK);

  // Kernel should get event immediately:
  prv_assert_subscription_event(characteristic, BLESubscriptionNone, BLEGATTErrorSuccess,
                   true /* kernel */, false /* app */);

  // CCCD value should be 0 now:
  cl_assert_equal_i(s_last_cccd_value, 0);

  // Verify everything is cleaned up:
  fake_kernel_malloc_mark_assert_equal();
}

void test_gatt_client_subscriptions__unsubscribe_not_last_subscriber(void) {
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();
  // Subscribe kernel:
  BTErrno e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                                  GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoOK);
  cl_assert_equal_i(s_last_cccd_value, BLESubscriptionIndications);

  // Simulate getting confirmation from remote:
  prv_confirm_cccd_write(BLEGATTErrorSuccess);
  fake_event_clear_last();

  // Subscribe app:
  e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                          GAPLEClientApp);

  s_last_cccd_value = ~0;

  // Unsubscribe kernel:
  e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionNone, GAPLEClientKernel);
  // Kernel should get event immediately:
  prv_assert_subscription_event(characteristic, BLESubscriptionNone, BLEGATTErrorSuccess,
                   true /* kernel */, false /* app */);

  // CCCD value should not be written, because app is still subscribed:
  cl_assert_equal_i(s_last_cccd_value, (uint16_t) ~0);
}

void test_gatt_client_subscriptions__subscribe_failed_cccd_write(void) {
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();

  fake_kernel_malloc_mark();

  // Subscribe kernel:
  BTErrno e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                                  GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoOK);
  cl_assert_equal_i(s_last_cccd_value, BLESubscriptionIndications);

  // Simulate getting confirmation from remote:
  prv_confirm_cccd_write(BLEGATTErrorInvalidHandle);

  // Expect kernel event with 'BLESubscriptionNone' type and error bubbled up:
  prv_assert_subscription_event(characteristic, BLESubscriptionNone, BLEGATTErrorInvalidHandle,
                   true /* kernel */, false /* app */);

  // Verify everything is cleaned up:
  fake_kernel_malloc_mark_assert_equal();
}

// -------------------------------------------------------------------------------------------------
// gatt_client_subscriptions_handle_server_notification &
// gatt_client_subscriptions_consume_notification

void test_gatt_client_subscriptions__notification_but_no_subscribers(void) {
  const uint8_t value = 0xAA;
  gatt_client_subscriptions_handle_server_notification(s_connection, s_handle,
                                                       &value, sizeof(value));
  prv_assert_no_event();
}

void test_gatt_client_subscriptions__cccd_write_confirmation_but_no_subscription(void) {
  s_last_cccd_ref = 1;
  prv_confirm_cccd_write(BLEGATTErrorSuccess);

  // This used to cause a crash:
  // https://pebbletechnology.atlassian.net/browse/PBL-23455
}

void test_gatt_client_subscriptions__notification_single_subscriber(void) {
  // Subscribe app:
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();
  BTErrno e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                                  GAPLEClientApp);
  cl_assert_equal_i(e, BTErrnoOK);

  // Simulate getting confirmation from remote:
  prv_confirm_cccd_write(BLEGATTErrorSuccess);
  fake_event_clear_last();

  // Nothing to be read before getting the notification:
  bool has_notification = gatt_client_subscriptions_get_notification_header(GAPLEClientApp,
                                                                            NULL);
  cl_assert_equal_b(has_notification, false);

  const uint8_t value[] = {0xAA, 0xBB, 0xCC};
  gatt_client_subscriptions_handle_server_notification(s_connection, s_handle,
                                                       value, sizeof(value));

  prv_assert_notification_event(characteristic, value, sizeof(value),
                                false /* kernel */, true /* app */);

  // Nothing to be read after "consuming" it:
  has_notification = gatt_client_subscriptions_get_notification_header(GAPLEClientApp, NULL);
  cl_assert_equal_b(has_notification, false);
}

void test_gatt_client_subscriptions__zero_length_notification(void) {
  // Subscribe app:
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();
  BTErrno e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                                  GAPLEClientApp);
  cl_assert_equal_i(e, BTErrnoOK);

  // Simulate getting confirmation from remote:
  prv_confirm_cccd_write(BLEGATTErrorSuccess);
  fake_event_clear_last();

  gatt_client_subscriptions_handle_server_notification(s_connection, s_handle, NULL, 0);

  prv_assert_notification_event(characteristic, NULL, 0, false /* kernel */, true /* app */);

  // Nothing to be read after "consuming" it:
  bool has_notification = gatt_client_subscriptions_get_notification_header(GAPLEClientApp, NULL);
  cl_assert_equal_b(has_notification, false);
}

void test_gatt_client_subscriptions__notification_app_and_kernel_subscribers(void) {
  // Subscribe app:
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();
  BTErrno e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                                  GAPLEClientApp);
  cl_assert_equal_i(e, BTErrnoOK);

  // Subscribe kernel:
  e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                          GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoOK);

  // Simulate getting confirmation from remote:
  prv_confirm_cccd_write(BLEGATTErrorSuccess);
  fake_event_clear_last();
  fake_event_reset_count();

  const uint8_t value[] = {0xAA, 0xBB, 0xCC};
  gatt_client_subscriptions_handle_server_notification(s_connection, s_handle,
                                                       value, sizeof(value));
  cl_assert_equal_i(fake_event_get_count(), 1);

  // Send another notification, before reading out the previous one:
  gatt_client_subscriptions_handle_server_notification(s_connection, s_handle,
                                                       value, sizeof(value));
  // Only one event should have been put on the queue:
  cl_assert_equal_i(fake_event_get_count(), 1);

  // Assert 2 events can be read out:
  prv_assert_notification_event(characteristic, value, sizeof(value),
                                true /* kernel */, true /* app */);
  prv_assert_notification_event(characteristic, value, sizeof(value),
                                true /* kernel */, true /* app */);

  // Send the 3rd notification:
  gatt_client_subscriptions_handle_server_notification(s_connection, s_handle,
                                                       value, sizeof(value));
  cl_assert_equal_i(fake_event_get_count(), 2);
  prv_assert_notification_event(characteristic, value, sizeof(value),
                                true /* kernel */, true /* app */);
}

static pbl_tick_t prv_taking_too_long_to_consume_yield_cb(struct pbl_sem *queue) {
  return milliseconds_to_ticks(1000);
}

static pbl_tick_t prv_consume_in_time_yield_cb(struct pbl_sem *queue) {
  // Consume while BT task is waiting for buffer to be freed up:
  BLECharacteristic characteristic_out;
  uint8_t *value_out = (uint8_t *) malloc(GATT_CLIENT_SUBSCRIPTIONS_BUFFER_SIZE);
  uint16_t value_length = GATT_CLIENT_SUBSCRIPTIONS_BUFFER_SIZE;
  gatt_client_subscriptions_consume_notification(&characteristic_out, value_out, &value_length,
                                                 GAPLEClientApp, NULL);
  free(value_out);
  return milliseconds_to_ticks(5);
}

void test_gatt_client_subscriptions__notification_buffer_full(void) {
  // Subscribe app:
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();
  BTErrno e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                                  GAPLEClientApp);
  cl_assert_equal_i(e, BTErrnoOK);

  // Simulate getting confirmation from remote:
  prv_confirm_cccd_write(BLEGATTErrorSuccess);
  fake_event_clear_last();

  // We had a bug at some point where the header size was not taken into account correctly, take
  // a value that won't fit by 1 byte, so this bug does not happen again:
  const size_t too_big = GATT_CLIENT_SUBSCRIPTIONS_BUFFER_SIZE -
                         sizeof(GATTBufferedNotificationHeader) + 1;
  uint8_t *value = (uint8_t *) malloc(too_big);
  memset(value, 0x55, too_big);
  gatt_client_subscriptions_handle_server_notification(s_connection, s_handle,
                                                       value, too_big);
  prv_assert_no_event();

  // Receive a GATT notification that is supposed to fill up the buffer entirely:
  const size_t fill_entirely_size = (GATT_CLIENT_SUBSCRIPTIONS_BUFFER_SIZE -
                                     sizeof(GATTBufferedNotificationHeader));
  gatt_client_subscriptions_handle_server_notification(s_connection, s_handle,
                                                       value, fill_entirely_size);
  prv_assert_notification_event_ext(characteristic, value, fill_entirely_size,
                                    false /* kernel */, true /* app */, false /* should_consume */);

  // Receive another GATT notification. Won't fit until consumed. Consuming is taking to long:
  fake_sem_set_yield_callback(gatt_client_subscription_get_semaphore(),
                                prv_taking_too_long_to_consume_yield_cb);
  fake_event_clear_last();
  gatt_client_subscriptions_handle_server_notification(s_connection, s_handle,
                                                       value, 1 /* one byte */);
  // Data will be dropped, no event :(
  prv_assert_no_event();

  // Receive another GATT notification. Won't fit until consumed.
  // Consuming is happening before the timeout hits (in the yield callback):
  fake_sem_set_yield_callback(gatt_client_subscription_get_semaphore(),
                                prv_consume_in_time_yield_cb);
  gatt_client_subscriptions_handle_server_notification(s_connection, s_handle,
                                                       value, 1 /* one byte */);
  prv_assert_notification_event(characteristic, value, 1 /* one byte */,
                                false /* kernel */, true /* app */);

  free(value);
}

void test_gatt_client_subscriptions__consume_but_too_small_buffer(void) {
  // Subscribe app:
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();
  BTErrno e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                                  GAPLEClientApp);
  cl_assert_equal_i(e, BTErrnoOK);

  // Simulate getting confirmation from remote:
  prv_confirm_cccd_write(BLEGATTErrorSuccess);
  fake_event_clear_last();

  const uint8_t value[] = {0xAA, 0xBB, 0xCC};
  gatt_client_subscriptions_handle_server_notification(s_connection, s_handle,
                                                       value, sizeof(value));

  BLECharacteristic handle_out = ~0;
  uint8_t out[1];
  uint16_t value_length = sizeof(out);
  bool has_more = true;
  const uint16_t next_length =
          gatt_client_subscriptions_consume_notification(&handle_out, out, &value_length,
                                                         GAPLEClientApp, &has_more);
  cl_assert_equal_i(handle_out, BLE_CHARACTERISTIC_INVALID);
  cl_assert_equal_i(value_length, 0);
  // Notification will be eaten, regardless of whether it was copied:
  cl_assert_equal_b(has_more, false);
  cl_assert_equal_i(next_length, 0);
}

void test_gatt_client_subscriptions__consume_but_nothing_in_buffer(void) {
  // Subscribe app:
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();
  BTErrno e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                                  GAPLEClientApp);
  cl_assert_equal_i(e, BTErrnoOK);

  // Simulate getting confirmation from remote:
  prv_confirm_cccd_write(BLEGATTErrorSuccess);
  fake_event_clear_last();

  BLECharacteristic handle_out = ~0;
  uint8_t out[1];
  uint16_t value_length = sizeof(out);
  bool has_more = true;
  const uint16_t next_length =
        gatt_client_subscriptions_consume_notification(&handle_out, out, &value_length,
                                                       GAPLEClientApp, &has_more);
  cl_assert_equal_i(handle_out, BLE_CHARACTERISTIC_INVALID);
  cl_assert_equal_i(value_length, 0);
  cl_assert_equal_b(has_more, false);
  cl_assert_equal_i(next_length, 0);
}

void test_gatt_client_subscriptions__consume_but_buffer_client_buffer_null(void) {
  bool has_more = true;
  BLECharacteristic handle_out = ~0;
  uint8_t out[1];
  uint16_t value_length = sizeof(out);
  
  const uint16_t next_length =
      gatt_client_subscriptions_consume_notification(&handle_out, out, &value_length,
                                                     GAPLEClientApp, &has_more);

  cl_assert_equal_i(next_length, 0);
  cl_assert_equal_b(has_more, false);
}

void test_gatt_client_subscriptions__notification_consume_without_notification(void) {
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();

  uint16_t value_length;
  GATTBufferedNotificationHeader header = {
    .characteristic = BLE_CHARACTERISTIC_INVALID,
    .value_length = 0,
  };
  bool has_more = gatt_client_subscriptions_get_notification_header(GAPLEClientKernel,
                                                                    &header);
  cl_assert_equal_i(header.value_length, 0);
  cl_assert_equal_i(header.characteristic, BLE_CHARACTERISTIC_INVALID);
  cl_assert_equal_b(has_more, false);

  BLECharacteristic characteristic_out = ~0;
  uint8_t value = 0xff;
  value_length = sizeof(value);
  gatt_client_subscriptions_consume_notification(&characteristic_out, &value, &value_length,
                                                 GAPLEClientKernel, &has_more);
  // Expect untouched:
  cl_assert_equal_i(characteristic_out, ~0);
  cl_assert_equal_i(value, 0xff);
  cl_assert_equal_b(has_more, false);
}

// -------------------------------------------------------------------------------------------------
// gatt_client_subscriptions_cleanup_by_client

void test_gatt_client_subscriptions__cleanup_by_client(void) {
  fake_kernel_malloc_mark();

  // Subscribe app:
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();
  BTErrno e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                                  GAPLEClientApp);
  cl_assert_equal_i(e, BTErrnoOK);

  gatt_client_subscriptions_cleanup_by_client(GAPLEClientApp);
  prv_assert_no_event();

  fake_kernel_malloc_mark_assert_equal();
}

// -------------------------------------------------------------------------------------------------
// gatt_client_subscriptions_cleanup_by_connection

bool gatt_client_get_event_pending_state(GAPLEClient);

static void prv_pend_events_to_kernel_and_app(void) {
  // fake pend an event to the kernel
  gatt_client_subscriptions_reschedule(GAPLEClientKernel);
  cl_assert_equal_b(gatt_client_get_event_pending_state(GAPLEClientKernel), true);
  fake_event_clear_last();

  // fake pend an event to the app
  gatt_client_subscriptions_reschedule(GAPLEClientApp);
  cl_assert_equal_b(gatt_client_get_event_pending_state(GAPLEClientApp), true);
  fake_event_clear_last();
}

static void prv_assert_no_pending_events_to_kernel_and_app(void) {
  cl_assert_equal_b(gatt_client_get_event_pending_state(GAPLEClientKernel), false);
  cl_assert_equal_b(gatt_client_get_event_pending_state(GAPLEClientApp), false);
}

void test_gatt_client_subscriptions__cleanup_by_connection(void) {
  fake_kernel_malloc_mark();

  // Subscribe app and kernel:
  BLECharacteristic characteristic = prv_get_indicatable_characteristic();
  BTErrno e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                                  GAPLEClientApp);
  cl_assert_equal_i(e, BTErrnoOK);
  e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                          GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoOK);

  prv_pend_events_to_kernel_and_app();

  gatt_client_subscriptions_cleanup_by_connection(s_connection, false /* should_unsubscribe */);
  prv_assert_no_event();

  // there should be no more subscriptions
  cl_assert(s_connection->gatt_subscriptions == NULL);
  prv_assert_no_pending_events_to_kernel_and_app();

  fake_kernel_malloc_mark_assert_equal();
}

extern void gatt_client_subscription_cleanup_by_att_handle_range(
    struct GAPLEConnection *connection, ATTHandleRange *range);

void test_gatt_client_subscriptions__cleanup_by_att_handle_range(void) {
  fake_kernel_malloc_mark();

  BLECharacteristic characteristic = prv_get_indicatable_characteristic();
  BTErrno e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                                  GAPLEClientApp);
  cl_assert_equal_i(e, BTErrnoOK);
  e = gatt_client_subscriptions_subscribe(characteristic, BLESubscriptionIndications,
                                                  GAPLEClientKernel);
  cl_assert_equal_i(e, BTErrnoOK);

  cl_assert(s_connection->gatt_subscriptions != NULL);

  ATTHandleRange range;
  fake_gatt_get_bp_att_handle_range(&range.start, &range.end);

  ATTHandleRange bogus_range = {
    .start = range.end + 1,
    .end = range.end + 5
  };

  prv_pend_events_to_kernel_and_app();

  // should have no effect since service is not in this range
  gatt_client_subscription_cleanup_by_att_handle_range(s_connection, &bogus_range);
  prv_assert_no_event();
  cl_assert(s_connection->gatt_subscriptions != NULL);
  
  // should actually remove everything
  gatt_client_subscription_cleanup_by_att_handle_range(s_connection, &range);
  prv_assert_no_event();

  // there should be no more subscriptions
  cl_assert(s_connection->gatt_subscriptions == NULL);
  prv_assert_no_pending_events_to_kernel_and_app();

  fake_kernel_malloc_mark_assert_equal();
}

// -------------------------------------------------------------------------------------------------
// TODO: Write tests that exercise applib/bluetooth/ble_client.c
