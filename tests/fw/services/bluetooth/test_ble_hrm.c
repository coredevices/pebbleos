/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/bluetooth/ble_hrm.h"

#include "comm/ble/gap_le_connection.h"
#include "pbl/services/activity/activity.h"

#include <bluetooth/hrm_service.h>
#include <pbl/btutil/bt_device.h>
#include <pbl/util/size.h>

#include <clar.h>


////////////////////////////////////////////////////////////////////////////////////////////////////
// Stubs & Fakes

#include "fake_event_service.h"
#include "fake_pebble_tasks.h"
#include "fake_pbl_malloc.h"
#include "fake_regular_timer.h"

#include "fake_workout_service.h"
#include "stubs_analytics.h"
#include "stubs_bt_lock.h"
#include "stubs_logging.h"
#include "stubs_passert.h"

static int s_gap_le_slave_reconnect_hrm_restart_call_count;
void gap_le_slave_reconnect_hrm_restart(void) {
  ++s_gap_le_slave_reconnect_hrm_restart_call_count;
}

static int s_gap_le_slave_reconnect_hrm_stop_call_count;
void gap_le_slave_reconnect_hrm_stop(void) {
  ++s_gap_le_slave_reconnect_hrm_stop_call_count;
}

static bool s_activity_prefs_ble_hrm_sharing_is_enabled;
bool activity_prefs_ble_hrm_sharing_is_enabled(void) {
  return s_activity_prefs_ble_hrm_sharing_is_enabled;
}

static bool s_bt_driver_hrm_service_is_enabled;
static int s_bt_driver_hrm_service_enable_call_count;
void bt_driver_hrm_service_enable(bool enable) {
  s_bt_driver_hrm_service_enable_call_count++;
  s_bt_driver_hrm_service_is_enabled = enable;
}

static BleHrmServiceMeasurement s_last_ble_hrm_measurement;
static int s_bt_driver_hrm_service_handle_measurement_call_count;
static BTDeviceInternal s_last_permitted_devices[10];
static size_t s_last_num_permitted_devices;
void bt_driver_hrm_service_handle_measurement(const BleHrmServiceMeasurement *measurement,
                                              const BTDeviceInternal *permitted_devices,
                                              size_t num_permitted_devices) {
  ++s_bt_driver_hrm_service_handle_measurement_call_count;
  s_last_ble_hrm_measurement = *measurement;
  s_last_num_permitted_devices = num_permitted_devices;
  memcpy(s_last_permitted_devices, permitted_devices,
         sizeof(*permitted_devices) * num_permitted_devices);
}

static BLEHRMSharingRequest *s_last_sharing_request;
static int s_ble_hrm_push_sharing_request_window_call_count;
void ble_hrm_push_sharing_request_window(BLEHRMSharingRequest *sharing_request) {
  ++s_ble_hrm_push_sharing_request_window_call_count;
  cl_assert_equal_p(s_last_sharing_request, NULL);
  s_last_sharing_request = sharing_request;
}

bool bt_driver_is_hrm_service_supported(void) {
  return true;
}

static BTDeviceInternal s_last_disconnected;
int bt_driver_gap_le_disconnect(const BTDeviceInternal *peer_address) {
  s_last_disconnected = *peer_address;
  return 0;
}

static void prv_assert_last_disconnected(const BTDeviceInternal *peer_address) {
  cl_assert_equal_b(bt_device_internal_equal(peer_address, &s_last_disconnected), true);
}

static int s_ble_hrm_push_reminder_popup_call_count;
void ble_hrm_push_reminder_popup(void) {
  s_ble_hrm_push_reminder_popup_call_count++;
}

static GAPLEConnection *s_connections[2];

GAPLEConnection *gap_le_connection_by_device(const BTDeviceInternal *device) {
  for (int i = 0; i < ARRAY_LENGTH(s_connections); ++i) {
    if (bt_device_internal_equal(device, &s_connections[i]->device)) {
      return s_connections[i];
    }
  }
  return NULL;
}
BTDeviceInternal *device_from_le_connection(GAPLEConnection *conn) {
  return &conn->device;
}

bool gap_le_connection_is_valid(const GAPLEConnection *conn) {
  for (int i = 0; i < ARRAY_LENGTH(s_connections); ++i) {
    if (s_connections[i] == conn) {
      return true;
    }
  }
  return false;
}

void gap_le_connection_for_each(GAPLEConnectionForEachCallback cb, void *data) {
  for (int i = 0; i < ARRAY_LENGTH(s_connections); ++i) {
    cb(s_connections[i], data);
  }
}

void launcher_task_add_callback(CallbackEventCallback callback, void *data) {
  callback(data);
}

bool sys_hrm_manager_is_hrm_present(void) {
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Tests

static void prv_assert_event_service_subscribed(bool is_subscribed) {
  const EventServiceInfo *const info = fake_event_service_get_info(PEBBLE_HEALTH_SERVICE_EVENT);
  if (is_subscribed) {
    cl_assert(info->handler);
  } else {
    cl_assert_equal_p(NULL, info->handler);
  }
}

void test_ble_hrm__cleanup(void) {
  ble_hrm_deinit();

  prv_assert_event_service_subscribed(false);

  fake_pbl_malloc_check_net_allocs();

  // Assert all regular timers are deregistered:
  cl_assert_equal_p(s_seconds_callbacks.next, NULL);
  cl_assert_equal_p(s_minutes_callbacks.next, NULL);
}

#define TEST_DEVICE_NAME "iPhone Martijn"

static GAPLEConnection s_conn_a;
static GAPLEConnection s_conn_b;
static const BTDeviceInternal *s_device_a;
static const BTDeviceInternal *s_device_b;

void test_ble_hrm__initialize(void) {
  fake_pbl_malloc_clear_tracking();
  for (int i = 0; i < ARRAY_LENGTH(s_connections); ++i) {
    s_connections[i] = NULL;
  }
  s_activity_prefs_ble_hrm_sharing_is_enabled = true;
  s_bt_driver_hrm_service_is_enabled = true;
  s_last_num_permitted_devices = 0;
  memset(s_last_permitted_devices, 0, sizeof(s_last_permitted_devices));
  s_bt_driver_hrm_service_enable_call_count = 0;
  s_bt_driver_hrm_service_handle_measurement_call_count = 0;
  s_ble_hrm_push_sharing_request_window_call_count = 0;
  s_ble_hrm_push_reminder_popup_call_count = 0;
  s_gap_le_slave_reconnect_hrm_restart_call_count = 0;
  s_gap_le_slave_reconnect_hrm_stop_call_count = 0;
  workout_service_stop_workout();
  s_last_disconnected = (BTDeviceInternal) {};
  s_last_sharing_request = NULL;
  s_last_ble_hrm_measurement = (BleHrmServiceMeasurement) {};
  fake_event_service_init();

  // Set up fake devices/connections:
  s_conn_a = (GAPLEConnection) {
    .device_name = TEST_DEVICE_NAME,
    .device = {
      .address = {
        .octets = {1, 2, 3, 4, 5, 6},
      },
    },
  };
  s_conn_b = (GAPLEConnection) {
    .device_name = TEST_DEVICE_NAME,
    .device = {
      .address = {
        .octets = {6, 5, 4, 3, 2, 1},
      },
    },
  };
  s_connections[0] = &s_conn_a;
  s_connections[1] = &s_conn_b;
  s_device_a = device_from_le_connection(&s_conn_a);
  s_device_b = device_from_le_connection(&s_conn_b);

  ble_hrm_init();
}

void test_ble_hrm__init_deinit_no_subscriptions(void) {
  // let cleanup & initialize do the work :)
}

static void prv_assert_permissions_ui_and_respond(bool is_granted) {
  cl_assert(s_last_sharing_request);
  ble_hrm_handle_sharing_request_response(is_granted, s_last_sharing_request);
  s_last_sharing_request = NULL;
}

void test_ble_hrm__sub_unsub(void) {
  prv_assert_event_service_subscribed(false);

  // Device A subscribes:
  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);

  // Expect health event subscription NOT to be active yet, need to grant permission first:
  prv_assert_event_service_subscribed(false);
  cl_assert_equal_b(false, ble_hrm_is_sharing_to_connection(&s_conn_a));

  // Expect permissions UI to be presented:
  prv_assert_permissions_ui_and_respond(true /* is_granted */);

  // Expect health event subscription to be active:
  prv_assert_event_service_subscribed(true);
  cl_assert_equal_b(true, ble_hrm_is_sharing_to_connection(&s_conn_a));

  // Device A subscribes again, should be a no-op, no new permissions prompt:
  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);
  prv_assert_event_service_subscribed(true);

  // Device B subscribes, shouldn't resubscribe, but should present a new
  // permission prompt, because it's a different device:
  bt_driver_cb_hrm_service_update_subscription(s_device_b, true);
  cl_assert_equal_b(false, ble_hrm_is_sharing_to_connection(&s_conn_b));
  prv_assert_permissions_ui_and_respond(true /* is_granted */);
  cl_assert_equal_b(true, ble_hrm_is_sharing_to_connection(&s_conn_b));
  prv_assert_event_service_subscribed(true);

  // Device A disconnects, shouldn't unsubscribe because B is still subscribed:
  ble_hrm_handle_disconnection(&s_conn_a);
  cl_assert_equal_b(false, ble_hrm_is_sharing_to_connection(&s_conn_a));
  prv_assert_event_service_subscribed(true);

  // Device B unsubscribes, expect to unsubscribe because there are no more
  // devices subscribed to the BLE HRM service:
  bt_driver_cb_hrm_service_update_subscription(s_device_b, false);
  cl_assert_equal_b(false, ble_hrm_is_sharing_to_connection(&s_conn_a));
  prv_assert_event_service_subscribed(false);

  // Device B unsubscribes again, should be no-op
  bt_driver_cb_hrm_service_update_subscription(s_device_b, false);
  prv_assert_event_service_subscribed(false);
}

void test_ble_hrm__sub_unsub_resub(void) {
  // Device A subscribes:
  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);

  // Expect permissions UI to be presented:
  prv_assert_permissions_ui_and_respond(true /* is_granted */);

  // Device A unsubscribes:
  bt_driver_cb_hrm_service_update_subscription(s_device_a, false);

  prv_assert_event_service_subscribed(false);

  // Device A re-subscribes, permission should still be valid:
  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);

  prv_assert_event_service_subscribed(true);
}

void test_ble_hrm__revoke(void) {
  // Device A subscribes:
  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);

  // Expect permissions UI to be presented:
  prv_assert_permissions_ui_and_respond(true /* is_granted */);

  cl_assert_equal_b(true, ble_hrm_is_sharing_to_connection(&s_conn_a));
  cl_assert_equal_b(true, ble_hrm_is_sharing());
  prv_assert_event_service_subscribed(true);

  // Revoke:
  ble_hrm_revoke_sharing_permission_for_connection(&s_conn_a);

  cl_assert_equal_b(false, ble_hrm_is_sharing_to_connection(&s_conn_a));
  cl_assert_equal_b(false, ble_hrm_is_sharing());
  prv_assert_event_service_subscribed(false);
  prv_assert_last_disconnected(s_device_a);
}

void test_ble_hrm__revoke_all(void) {
  // Device A subscribes:
  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);

  // Expect permissions UI to be presented:
  prv_assert_permissions_ui_and_respond(true /* is_granted */);

  // Device B subscribes:
  bt_driver_cb_hrm_service_update_subscription(s_device_b, true);

  // Expect permissions UI to be presented:
  prv_assert_permissions_ui_and_respond(true /* is_granted */);

  ble_hrm_revoke_all();

  cl_assert_equal_b(false, ble_hrm_is_sharing_to_connection(&s_conn_a));
  cl_assert_equal_b(false, ble_hrm_is_sharing_to_connection(&s_conn_b));
  cl_assert_equal_b(false, ble_hrm_is_sharing());
  prv_assert_event_service_subscribed(false);
}

void test_ble_hrm__revoke_after_disconnection(void) {
  ble_hrm_revoke_sharing_permission_for_connection(NULL);

  s_connections[0] = NULL;
  ble_hrm_revoke_sharing_permission_for_connection(&s_conn_a);

  cl_assert_equal_b(false, ble_hrm_is_sharing_to_connection(NULL));

  // Shouldn't crash or anything
}

void test_ble_hrm__grant_after_disconnection(void) {
  // Device A subscribes:
  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);

  // Fake disconnection:
  s_connections[0] = NULL;

  // Grabt permission after disconnection.
  // Request object should be freed and thing shouldn't crash.
  prv_assert_permissions_ui_and_respond(true /* is_granted */);

  cl_assert_equal_b(false, ble_hrm_is_sharing_to_connection(&s_conn_a));
}

void test_ble_hrm__decline_permission_dont_ask_again_even_after_reconnecting(void) {
  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);

  // Decline:
  prv_assert_permissions_ui_and_respond(false /* is_granted */);

  // Unsub, resub:
  bt_driver_cb_hrm_service_update_subscription(s_device_a, false);
  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);

  // No sharing request UI:
  cl_assert_equal_p(NULL, s_last_sharing_request);

  // Fake disconnection:
  ble_hrm_handle_disconnection(&s_conn_a);

  // Fake reconn & subscribe:
  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);

  // No sharing request UI:
  cl_assert_equal_p(NULL, s_last_sharing_request);

  // Still declined:
  cl_assert_equal_b(false, ble_hrm_is_sharing_to_connection(&s_conn_a));
}

void test_ble_hrm__unsub_upon_deinit(void) {
  // Device A subscribes:
  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);
  prv_assert_permissions_ui_and_respond(true /* is_granted */);

  // __cleanup() will do the deinit and also assert that the event service is unsubscribed.
}

// Test that we handle a races where a subscription/disconnection callback happens in after
// deiniting the stack:
void test_ble_hrm__sub_after_deinit(void) {
  ble_hrm_deinit();

  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);
  prv_assert_event_service_subscribed(false);

  ble_hrm_handle_disconnection(&s_conn_a);
  prv_assert_event_service_subscribed(false);

  ble_hrm_init(); // reinit, __cleanup() will deinit again
}

static void prv_put_and_assert_health_event(HealthEventType type, uint8_t bpm,
                                             HRMQuality quality, bool expect_bt_driver_cb,
                                             bool expected_is_on_wrist) {
  int call_count_before = s_bt_driver_hrm_service_handle_measurement_call_count;

  PebbleEvent event = {
    .type = PEBBLE_HEALTH_SERVICE_EVENT,
    .health_event = {
      .type = type,
      .data.heart_rate_update = {
        .current_bpm = bpm,
        .quality = quality,
      },
    },
  };
  event_put(&event);
  fake_event_service_handle_last();

  if (expect_bt_driver_cb) {
    cl_assert_equal_i(call_count_before + 1, s_bt_driver_hrm_service_handle_measurement_call_count);
    cl_assert_equal_i(bpm, s_last_ble_hrm_measurement.bpm);
    cl_assert_equal_b(expected_is_on_wrist, s_last_ble_hrm_measurement.is_on_wrist);
  } else {
    cl_assert_equal_i(call_count_before, s_bt_driver_hrm_service_handle_measurement_call_count);
  }
}

static void prv_put_workout_event(PebbleWorkoutEventType type) {
  PebbleEvent e = {
    .type = PEBBLE_WORKOUT_EVENT,
    .workout = { .type = type },
  };
  event_put(&e);
  fake_event_service_handle_last();
}

void test_ble_hrm__handle_health_event(void) {
  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);
  cl_assert_equal_i(0, s_bt_driver_hrm_service_handle_measurement_call_count);
  prv_assert_permissions_ui_and_respond(true /* is_granted */);

  // Don't grant permission to device B:
  bt_driver_cb_hrm_service_update_subscription(s_device_b, true);
  prv_assert_permissions_ui_and_respond(false /* is_granted */);

  prv_put_and_assert_health_event(HealthEventHeartRateUpdate, 80, HRMQuality_Excellent,
                                  true /* expect bt driver cb */, true /* expected_is_on_wrist */);

  // Assert only device A is listed as "permitted device" and B is not:
  cl_assert_equal_i(1, s_last_num_permitted_devices);
  cl_assert_equal_m(&s_last_permitted_devices[0], s_device_a, sizeof(*s_device_a));

  prv_put_and_assert_health_event(HealthEventHeartRateUpdate, 80, HRMQuality_OffWrist,
                                  true /* expect bt driver cb */, false /* expected_is_on_wrist */);

  // Ignore non-heart-rate health event:
  prv_put_and_assert_health_event(HealthEventMovementUpdate, 0, HRMQuality_OffWrist,
                                  false /* expect bt driver cb */, false /* expected_is_on_wrist */);
}

void test_ble_hrm__handle_ble_hrm_sharing_enabled_changes(void) {
  cl_assert_equal_b(true, s_bt_driver_hrm_service_is_enabled);
  cl_assert_equal_i(0, s_bt_driver_hrm_service_enable_call_count);
  ble_hrm_handle_ble_hrm_sharing_enabled(false);
  cl_assert_equal_i(1, s_bt_driver_hrm_service_enable_call_count);
  cl_assert_equal_b(false, s_bt_driver_hrm_service_is_enabled);

  // Disabled, again -- would lead to another call to bt_driver_hrm_service_enable(),
  // the BT driver lib keeps track of whether it's enabled and is expected to ignore the call.
  ble_hrm_handle_ble_hrm_sharing_enabled(false);
  cl_assert_equal_i(2, s_bt_driver_hrm_service_enable_call_count);
  cl_assert_equal_b(false, s_bt_driver_hrm_service_is_enabled);

  // Enable
  ble_hrm_handle_ble_hrm_sharing_enabled(true);
  cl_assert_equal_i(3, s_bt_driver_hrm_service_enable_call_count);
  cl_assert_equal_b(true, s_bt_driver_hrm_service_is_enabled);
}

void test_ble_hrm__popup_after_long_continuous_use(void) {
  extern RegularTimerInfo *ble_hrm_timer(void);
  RegularTimerInfo *timer = ble_hrm_timer();

  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);
  prv_assert_permissions_ui_and_respond(true /* is_granted */);

  cl_assert_equal_b(true, regular_timer_is_scheduled(timer));

  bt_driver_cb_hrm_service_update_subscription(s_device_a, false);
  cl_assert_equal_b(false, regular_timer_is_scheduled(timer));

  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);
  cl_assert_equal_b(true, regular_timer_is_scheduled(timer));

  cl_assert_equal_i(0, s_ble_hrm_push_reminder_popup_call_count);
  fake_regular_timer_trigger(timer);
  cl_assert_equal_i(1, s_ble_hrm_push_reminder_popup_call_count);

  // Except timer to be rescheduled again:
  cl_assert_equal_b(true, regular_timer_is_scheduled(timer));

  bt_driver_cb_hrm_service_update_subscription(s_device_a, false);
  cl_assert_equal_b(false, regular_timer_is_scheduled(timer));
}

void test_ble_hrm__workout_start_restarts_advert(void) {
  cl_assert_equal_i(0, s_gap_le_slave_reconnect_hrm_restart_call_count);
  prv_put_workout_event(PebbleWorkoutEvent_Started);
  cl_assert_equal_i(1, s_gap_le_slave_reconnect_hrm_restart_call_count);
}

void test_ble_hrm__workout_start_skipped_when_sharing_disabled(void) {
  s_activity_prefs_ble_hrm_sharing_is_enabled = false;
  prv_put_workout_event(PebbleWorkoutEvent_Started);
  cl_assert_equal_i(0, s_gap_le_slave_reconnect_hrm_restart_call_count);
}

void test_ble_hrm__workout_stop_stops_advert_and_disconnects_subscribers(void) {
  // Device A subscribes and is granted permission:
  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);
  prv_assert_permissions_ui_and_respond(true /* is_granted */);

  prv_put_workout_event(PebbleWorkoutEvent_Stopped);

  cl_assert_equal_i(1, s_gap_le_slave_reconnect_hrm_stop_call_count);
  prv_assert_last_disconnected(s_device_a);
}

void test_ble_hrm__workout_stop_skips_non_sharers(void) {
  // Device A subscribes and is granted; device B subscribes but is declined:
  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);
  prv_assert_permissions_ui_and_respond(true /* is_granted */);
  bt_driver_cb_hrm_service_update_subscription(s_device_b, true);
  prv_assert_permissions_ui_and_respond(false /* is_granted */);

  prv_put_workout_event(PebbleWorkoutEvent_Stopped);

  cl_assert_equal_i(1, s_gap_le_slave_reconnect_hrm_stop_call_count);
  // Only device A was sharing, so it's the one that got disconnected:
  prv_assert_last_disconnected(s_device_a);
}

void test_ble_hrm__toggle_off_emits_sharing_state_event(void) {
  // Device A subscribes and is granted (subscription count = 1):
  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);
  prv_assert_permissions_ui_and_respond(true /* is_granted */);

  fake_event_reset_count();
  ble_hrm_handle_ble_hrm_sharing_enabled(false);

  cl_assert(fake_event_get_count() >= 1);
  PebbleEvent e = fake_event_get_last();
  cl_assert_equal_i(e.type, PEBBLE_BLE_HRM_SHARING_STATE_UPDATED_EVENT);
  cl_assert_equal_i(e.bluetooth.le.hrm_sharing_state.subscription_count, 0);
  cl_assert_equal_i(1, s_gap_le_slave_reconnect_hrm_stop_call_count);
}

void test_ble_hrm__toggle_off_no_event_when_no_subs(void) {
  fake_event_reset_count();
  ble_hrm_handle_ble_hrm_sharing_enabled(false);

  // No active subscriptions, so no sharing-state event should be emitted:
  cl_assert_equal_i(0, fake_event_get_count());
}

void test_ble_hrm__init_with_ongoing_workout_restarts_advert(void) {
  ble_hrm_deinit();
  workout_service_start_workout(ActivitySessionType_Run);
  s_gap_le_slave_reconnect_hrm_restart_call_count = 0;
  ble_hrm_init();

  cl_assert_equal_i(1, s_gap_le_slave_reconnect_hrm_restart_call_count);
  cl_assert(fake_event_service_get_info(PEBBLE_WORKOUT_EVENT)->handler);
}

void test_ble_hrm__toggle_off_then_disconnect_no_count_corruption(void) {
  // Device A subscribes and is granted:
  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);
  prv_assert_permissions_ui_and_respond(true /* is_granted */);
  cl_assert_equal_b(true, ble_hrm_is_sharing());

  // User toggles sharing off:
  ble_hrm_handle_ble_hrm_sharing_enabled(false);
  cl_assert_equal_b(false, ble_hrm_is_sharing());

  // Device A disconnects -- must not corrupt the subscription count:
  ble_hrm_handle_disconnection(&s_conn_a);
  cl_assert_equal_b(false, ble_hrm_is_sharing());

  // Re-subscribe should work correctly (count 0 -> 1):
  bt_driver_cb_hrm_service_update_subscription(s_device_a, true);
  // Permission was not cleared by toggle-off, so no new prompt:
  cl_assert_equal_p(NULL, s_last_sharing_request);
  cl_assert_equal_b(true, ble_hrm_is_sharing_to_connection(&s_conn_a));
  cl_assert_equal_b(true, ble_hrm_is_sharing());
}
