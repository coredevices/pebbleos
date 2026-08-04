/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "gap_le_connect_params.h"
#include "gap_le_connection.h"

#include "bluetooth/gap_le_connect.h"
#include "bluetooth/responsiveness.h"
#include "comm/bluetooth_analytics.h"
#include "comm/bt_conn_mgr.h"
#include "comm/bt_lock.h"
#include <pbl/drivers/rtc.h>
#include "kernel/pbl_malloc.h"
#include "pbl/services/analytics/analytics.h"
#include "pbl/services/new_timer/new_timer.h"
#include <pbl/logging/logging.h>
#include "util/time/time.h"

#include <bluetooth/bluetooth_types.h>
#include <stdint.h>

// [MT] See page 129 of BLE Developer's Handbook (R. Heydon) and also
// http://www.ti.com/lit/ug/swru271f/swru271f.pdf
//
// Connection Event – In a BLE connection between two devices, a
// frequency-hopping scheme is used, in that the two devices each send and
// receive data from one another on a specific channel, then “meet” at a new
// channel (the link layer of the BLE stack handles the channel switching) at a
// specific amount of time later. This “meeting” where the two devices send and
// receive data is known as a “connection event”. Even if there is no
// application data to be sent or received, the two devices will still exchange
// link layer data to maintain the connection.
//
// Connection Interval - The connection interval is the amount of time between
// two connection events, in units of 1.25ms. The connection interval can range
// from a minimum value of 6 (7.5ms) to a maximum of 3200 (4.0s).
//
// Slave Latency (SL): the number of connection events that the slave can
// ignore. This allows the slave save power. When needed, the slave can respond
// to a connection event. Therefore the slave gets (SL+1) opportunities to
// send data back to the master. In other words, this enables lower latency
// responses from the slave, at the cost of the master's energy budget.
// Valid values: 0 - 499, however the maximum value must not make the effective
// connection interval (see below) greater than 16.0s
//
// Supervision timeout: This is the maximum amount of time between two
// successful connection events. If this amount of time passes without a
// successful connection event, the device is to consider the connection lost,
// and return to an unconnected (standby) state.
// Valid values: 100ms to 32,000ms. In addition, the timeout must be larger
// than the effective connection interval (explained below).
// Rule of thumb: the slave should be given at least 6 opportunities
// to resynchronize.
//
// Effective connection interval: is equal to the amount of time between two
// connection events, assuming that the slave skips the maximum number of
// possible events if slave latency is allowed (the effective connection
// interval is equal to the actual connection interval if slave latency is set
// to zero). It can be calculated using the formula:
// Effective Connection Interval = (Connection Interval) * (1+(Slave Latency))

#define REQUEST_TIMEOUT_MS (5 * 1000)

//! See v4.2 "9.3.12 Connection Interval Timing Parameters":
//! "The Peripheral device should not perform a Connection Parameter Update procedure
//! within TGAP(conn_pause_peripheral = 5 seconds) after establishing a connection."
//! We deliberately deviate from this recommendation ("should", not "shall"): iOS
//! creates the connection with a 720ms supervision timeout and never raises it on
//! its own, so the link is fragile until our first update request is granted. See
//! also the note below about Apple's relaxed handling of TGAP timings.
#define REQUIRED_INIT_PAUSE_S (1)
#define REQUIRED_INIT_PAUSE_TICKS (REQUIRED_INIT_PAUSE_S * RTC_TICKS_HZ)

//! Try 3 times before giving up.
#define MAX_UPDATE_REQUEST_ATTEMPTS (3)

static const GAPLEConnectRequestParams s_default_connection_params_table[NumResponseTimeState] = {
  [ResponseTimeMax] = {
    .slave_latency_events = 3,
    .connection_interval_min_1_25ms = 24, // 30ms
    .connection_interval_max_1_25ms = 36, // 45ms
    .supervision_timeout_10ms = 600, // 6s
  },
  [ResponseTimeMiddle] = {
    .slave_latency_events = 3,
    .connection_interval_min_1_25ms = 24, // 30ms
    .connection_interval_max_1_25ms = 36, // 45ms
    .supervision_timeout_10ms = 600, // 6s
  },
  [ResponseTimeMin] = {
    .slave_latency_events = 0,
    .connection_interval_min_1_25ms = 12, // 15ms
    .connection_interval_max_1_25ms = 12, // 15ms
    .supervision_timeout_10ms = 600, // 6s
  },
};

extern void conn_mgr_handle_desired_state_granted(GAPLEConnection *hdl,
                                                  ResponseTimeState granted_state);

static void prv_watchdog_timer_callback(void *ctx);

// -----------------------------------------------------------------------------
//! Analytics helpers for tracking connection interval time.

static void prv_analytics_stop_conn_interval_timers(void) {
  PBL_ANALYTICS_TIMER_STOP(ble_conn_itvl_min_time_ms);
  PBL_ANALYTICS_TIMER_STOP(ble_conn_itvl_mid_time_ms);
  PBL_ANALYTICS_TIMER_STOP(ble_conn_itvl_max_time_ms);
  PBL_ANALYTICS_TIMER_STOP(ble_conn_itvl_other_time_ms);
  PBL_ANALYTICS_TIMER_STOP(ble_conn_slave_lat0_time_ms);
}

//! Classify the actual connection interval into a ResponseTimeState based on
//! the default connection params table ranges.
static ResponseTimeState prv_classify_conn_interval(uint16_t conn_interval_1_25ms) {
  // Check from fastest (Min) to slowest (Max) so we pick the tightest match
  for (int state = ResponseTimeMin; state >= ResponseTimeMax; --state) {
    const GAPLEConnectRequestParams *params = &s_default_connection_params_table[state];
    if (conn_interval_1_25ms >= params->connection_interval_min_1_25ms &&
        conn_interval_1_25ms <= params->connection_interval_max_1_25ms) {
      return (ResponseTimeState)state;
    }
  }
  // Outside all known ranges (only used for analytics bucketing)
  return ResponseTimeInvalid;
}

static void prv_analytics_update_conn_params(uint16_t conn_interval_1_25ms,
                                             uint16_t slave_latency_events) {
  prv_analytics_stop_conn_interval_timers();

  switch (prv_classify_conn_interval(conn_interval_1_25ms)) {
    case ResponseTimeMin:
      PBL_ANALYTICS_TIMER_START(ble_conn_itvl_min_time_ms);
      break;
    case ResponseTimeMiddle:
      PBL_ANALYTICS_TIMER_START(ble_conn_itvl_mid_time_ms);
      break;
    case ResponseTimeMax:
      PBL_ANALYTICS_TIMER_START(ble_conn_itvl_max_time_ms);
      break;
    default:
      PBL_ANALYTICS_TIMER_START(ble_conn_itvl_other_time_ms);
      break;
  }

  // Interval buckets alone cannot distinguish a link that never got its slave
  // latency applied (~Nx the connection-event duty at the same interval).
  if (slave_latency_events == 0U) {
    PBL_ANALYTICS_TIMER_START(ble_conn_slave_lat0_time_ms);
  }
}

static const GAPLEConnectRequestParams *prv_params_for_state(const GAPLEConnection *connection,
                                                             ResponseTimeState state) {
  if (connection->connection_parameter_sets) {
    return &connection->connection_parameter_sets[state];
  }
  return &s_default_connection_params_table[state];
}

static bool prv_do_actual_params_match_desired_state(const GAPLEConnection *connection,
                                                     ResponseTimeState state,
                                                     uint16_t *actual_conn_interval_ms_out) {
  const BleConnectionParams *actual_params = &connection->conn_params;

  if (actual_conn_interval_ms_out) {
    *actual_conn_interval_ms_out = actual_params->conn_interval_1_25ms;
  }
  const GAPLEConnectRequestParams *desired_params = prv_params_for_state(connection, state);

  // When the fastest state is desired, ignore the minimum bound:
  bool is_interval_min_acceptable;
  if (state == ResponseTimeMin) {
    is_interval_min_acceptable = true;
  } else {
    is_interval_min_acceptable =
        (actual_params->conn_interval_1_25ms >= desired_params->connection_interval_min_1_25ms);
  }

  return (is_interval_min_acceptable &&
          actual_params->conn_interval_1_25ms <= desired_params->connection_interval_max_1_25ms &&
          actual_params->slave_latency_events == desired_params->slave_latency_events);
}

static void prv_request_params_update(GAPLEConnection *connection,
                                      ResponseTimeState state) {
  if (connection->is_remote_device_managing_connection_parameters ||
      connection->param_update_info.is_request_pending) {
    return;
  }

  // We need to wait at least REQUIRED_INIT_PAUSE_TICKS after a connection before
  // requesting new parameters.
  uint32_t retry_ms = REQUEST_TIMEOUT_MS;
  if ((rtc_get_ticks() - connection->ticks_since_connection) < REQUIRED_INIT_PAUSE_TICKS) {
    retry_ms = (REQUIRED_INIT_PAUSE_S * MS_PER_SECOND);
    goto retry;
  }

  // Fall-back:
  uint16_t actual_connection_interval_ms =
      prv_params_for_state(connection, ResponseTimeMax)->connection_interval_max_1_25ms;
  if (prv_do_actual_params_match_desired_state(connection, state, &actual_connection_interval_ms)) {
    return;
  }
  if (connection->param_update_info.attempts++ >= MAX_UPDATE_REQUEST_ATTEMPTS) {
    // [MT]: I've hit this once now. When this happened the TI CC2564B became unresponsive.
    // From the iOS side, it appeared as a connection timeout. A little while after this happened,
    // the BT chip auto-reset work-around kicked in.
    PBL_LOG_ERR("Max attempts reached, giving up. desired_state=%u", state);
    bluetooth_analytics_handle_param_update_failed();
    return;
  }

  // Note: the spec recommends waiting for a 30 second Tgap timeout before issuing a new update
  // request. Bluetopia does not enforce this. However, Sriram Hariharan of Apple confirmed we
  // do not need to do this with Apple devices: "As long as your stack ensures that connection
  // update requests are sent only after the previous request is completed, you can ignore the
  // 30 second Tgap timeout."

  const GAPLEConnectRequestParams *desired_params = prv_params_for_state(connection, state);
  BleConnectionParamsUpdateReq req = {
    .interval_min_1_25ms = desired_params->connection_interval_min_1_25ms,
    .interval_max_1_25ms = desired_params->connection_interval_max_1_25ms,
    .slave_latency_events = desired_params->slave_latency_events,
    .supervision_timeout_10ms = desired_params->supervision_timeout_10ms,
  };

  const bool success = bt_driver_le_connection_parameter_update(&connection->device, &req);
  if (success) {
    connection->param_update_info.is_request_pending = true;
  }

retry:
  // Restart watchdog timer:
  new_timer_start(connection->param_update_info.watchdog_timer, retry_ms,
                  prv_watchdog_timer_callback, connection, 0);
}

static void prv_watchdog_timer_callback(void *ctx) {
  // This should all take very little time, so just execute on NewTimer task:
  bt_lock();
  GAPLEConnection *connection = (GAPLEConnection *)ctx;
  if (gap_le_connection_is_valid(connection)) {
    // Override the flag:
    connection->param_update_info.is_request_pending = false;
    // Retry with most recently requested latency:
    const ResponseTimeState state = conn_mgr_get_latency_for_le_connection(connection, NULL);
    if (connection->param_update_info.attempts > 0) {
      PBL_LOG_INFO("Conn param request timed out: re-requesting %u", state);
    }
    prv_request_params_update(connection, state);
  }
  bt_unlock();
}

void gap_le_connect_params_request(GAPLEConnection *connection,
                                   ResponseTimeState desired_state) {
  // A new desired state is requested by the FW, start afresh:
  connection->param_update_info.attempts = 0;

  prv_request_params_update(connection, desired_state);
}

void gap_le_connect_params_setup_connection(GAPLEConnection *connection, TimerID timer) {
  connection->param_update_info.watchdog_timer = timer;
}

void gap_le_connect_params_cleanup_by_connection(GAPLEConnection *connection) {
  new_timer_delete(connection->param_update_info.watchdog_timer);
  prv_analytics_stop_conn_interval_timers();
}

// -------------------------------------------------------------------------------------------------
//! Extern'd for and used by bt_conn_mgr.c
ResponseTimeState gap_le_connect_params_get_actual_state(GAPLEConnection *connection) {
  for (ResponseTimeState state = 0; state < NumResponseTimeState; ++state) {
    if (prv_do_actual_params_match_desired_state(connection, state, NULL)) {
      return state;
    }
  }
  return ResponseTimeInvalid;
}

static void prv_evaluate(GAPLEConnection *connection, ResponseTimeState desired_state) {
  if (prv_do_actual_params_match_desired_state(connection, desired_state, NULL)) {
    conn_mgr_handle_desired_state_granted(connection, desired_state);

    // If the timer callback is executing (waiting on bt_lock) at this point, it's not a problem
    // because the actual vs desired state gets checked in the timer callback path as well.
    new_timer_stop(connection->param_update_info.watchdog_timer);
    return;
  }

  // Connection parameters are updated, but they don't match the desired parameters.
  // (Re)request a parameter update:
  PBL_LOG_INFO("Connection parameters do not match desired state: %u", desired_state);
  prv_request_params_update(connection, desired_state);
}

// -------------------------------------------------------------------------------------------------
//! Extern'd for and used by services/bluetooth/pebble_pairing_service.c
//! bt_lock is assumed to be taken before calling this function.
//! Forces the module to re-evaluate whether the current parameters match the desired ones.
//! This is used when the set of desired request params are changed through Pebble Pairing Service.
void gap_le_connect_params_re_evaluate(GAPLEConnection *connection) {
  const ResponseTimeState desired_state = conn_mgr_get_latency_for_le_connection(connection, NULL);
  prv_evaluate(connection, desired_state);
}

// -------------------------------------------------------------------------------------------------
//! Extern'd for and used by gap_le_connect.c
//! Handles Bluetopia's Connection Parameter Updated event.
//! This event is sent by our BT controller when the updated parameters have actually been applied
//! and taken effect.
//! bt_lock is assumed to be taken before calling this function.
void bt_driver_handle_le_conn_params_update_event(const BleConnectionUpdateCompleteEvent *event) {
  bt_lock();
  const BleConnectionParams *params = &event->conn_params;
  if (event->status != HciStatusCode_Success) {
    goto unlock;
  }

  GAPLEConnection *connection = gap_le_connection_by_addr(&event->dev_address);
  if (!connection) {
    PBL_LOG_DBG("Receiving conn param update but connection is no longer open");
    goto unlock;
  }

  const ResponseTimeState desired_state = conn_mgr_get_latency_for_le_connection(connection, NULL);

  // Cache the BLE connection parameters
  connection->conn_params = *params;
  connection->param_update_info.is_request_pending = false;

  const bool local_is_master = connection->local_is_master;
  if (!local_is_master) {
     bluetooth_analytics_handle_connection_params_update(params);
     prv_analytics_update_conn_params(params->conn_interval_1_25ms, params->slave_latency_events);
     PBL_ANALYTICS_ADD(ble_conn_param_update_count, 1);
  }

  prv_evaluate(connection, desired_state);
unlock:
  bt_unlock();
}
