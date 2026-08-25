/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "hrm_manager.h"

#include "applib/event_service_client.h"
#include <pbl/drivers/rtc.h>
#include "freertos_types.h"
#include "kernel/events.h"
#include "pbl/os/mutex.h"
#include "process_management/app_install_types.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/util/list.h"
#include "pbl/util/circular_buffer.h"

#include <stdint.h>

typedef void (*HRMSubscriberCallback)(PebbleHRMEvent *event, void *context);

// We need roughly this many seconds of "spin up" time to get a good reading from the HR sensor
// right after turning it on
#define HRM_SENSOR_SPIN_UP_SEC 20

typedef struct AccelServiceState AccelServiceState;

typedef struct HRMSubscriberState {
  ListNode list_node;
  HRMSessionRef session_ref;  // The session ref assigned to this subscriber
  AppInstallId app_id;        // The subscriber's app_id
  PebbleTask task;            // The subscriber's task
  QueueHandle_t queue;        // Queue to send events to. If NULL, then this is for KernelBG

  HRMSubscriberCallback callback_handler;  // only used for KernelBG subscribers
  void *callback_context;                  // only used for KernelBG subscribers

  uint32_t update_interval_s; // How often to send updates to this subscriber
  time_t expire_utc;          // This subscription will expire at this time
  bool sent_expiration_event; // true after we've sent a HRMEvent_SubscriptionExpiring event
  HRMFeature features;        // what features the subscriber is interested in

  RtcTicks last_valid_bpm_ticks; // tick count the last time this subscriber received valid HR reading
} HRMSubscriberState;

// HRM manager expects to be update at 1Hz. To the system task, we can currently
// expect up to 2 events / second. 8 items in the queue allows for up to a 4s stall if subscribed
// to both BPM and LEDCurrent.
#define NUM_EVENTS_TO_QUEUE (8)
#define EVENT_STORAGE_SIZE  (sizeof(PebbleHRMEvent) * NUM_EVENTS_TO_QUEUE)

#define HRM_MANAGER_ACCEL_MANAGER_SAMPLES_PER_UPDATE 4

// After every HRM_CHECK_SENSOR_DISABLE_COUNT calls to hrm_manager_new_data_cb(), we check to see
// if we should disable the sensor.
#define HRM_CHECK_SENSOR_DISABLE_COUNT 10

// After this many consecutive hrm_enable failures, stop trying until reboot
#define HRM_MAX_ENABLE_FAILURES 3

// If the sensor has been on this long and a subscriber still hasn't received a Good-quality
// reading, the subscriber is deferred to its next interval instead of holding the sensor on.
// Comfortably above a normal serve cycle (spin-up plus a few seconds), far below the battery
// impact threshold.
#define HRM_MAX_UNSERVED_TIME_SEC 120

struct HRMManagerState {
  PebbleRecursiveMutex *lock;
  ListNode *subscribers;

  CircularBuffer system_task_event_buffer;
  uint32_t dropped_events; //!< Count of how many events for the system task have been dropped
  HRMSessionRef next_session_ref;
  uint8_t system_task_event_storage[EVENT_STORAGE_SIZE];

  AccelManagerState *accel_state;
  AccelRawData accel_manager_buffer[HRM_MANAGER_ACCEL_MANAGER_SAMPLES_PER_UPDATE];
  PebbleMutex *accel_data_lock;
  HRMAccelData accel_data;

  // Event Service to keep track of whether the charger is connected
  EventServiceInfo charger_subscription;

  TimerID update_enable_timer_id;  // used for re-enabling the HRM sensor

  uint8_t check_disable_counter;   // increments to HRM_CHECK_SENSOR_DISABLE_COUNT
  uint8_t enable_failure_count;    // counts consecutive hrm_enable failures, stops retrying after max

  HRMFeature enabled_features;     // feature union the sensor was last enabled with

  RtcTicks sensor_on_since_ticks;  // tick count when the sensor was last turned on; 0 while off
  bool unserved_timeout_logged;    // limits the unserved-timeout warning to once per on-stretch

  bool enabled_run_level;          // True if the current run_level (LowPower, Stationary,
                                   // Normal, etc.) allows the sensor to be turned on
  bool enabled_charging_state;     // Ture if we aren't plugged in / charging

};

//! Subscription for KernelBG or KernelMain clients.
//! When called by KernelBG clients a callback is mandatory. When called by KernelMain clients,
//! a callback is optional because the event_service can be used to subscribe to events.
//! For other clients, please see \ref sys_hrm_manager_app_subscribe
//! @param app_id the AppInstallId if this is an app or worker. If this is a system subscriber
//!   use INSTALL_ID_INVALID
//! @param update_interval_s requested update interval
//! @param expire_s after this many seconds, this subscription will automatically expire. Pass 0
//!   for no expiration.
//! @param features A bitfield of the features the subscriber would like updates for
//! @param callback the KernelBG callback to call when an HRM event is available
//! @param context the context pointer for the callback
//! @return the HRMSessionRef for this subscription. NULL on failure
HRMSessionRef hrm_manager_subscribe_with_callback(AppInstallId app_id, uint32_t update_interval_s,
                                                  uint16_t expire_s, HRMFeature features,
                                                  HRMSubscriberCallback callback, void *context);
