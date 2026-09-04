/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "bluetooth_analytics.h"

#include "comm/ble/gap_le_connection.h"
#include "comm/bt_lock.h"
#include <pbl/drivers/rtc.h>
#include "pbl/services/analytics/analytics.h"
#include "pbl/services/comm_session/session.h"
#include <pbl/logging/logging.h>

#include <bluetooth/analytics.h>
#include <bluetooth/gap_le_connect.h>

typedef struct {
  uint32_t slave_latency_events;
  uint32_t supervision_to_ms;
  int num_samps;
} LeConnectionParams;

static LeConnectionParams s_le_conn_params = { 0 };

void bluetooth_analytics_get_param_averages(uint16_t *params) {
  int num_samps = s_le_conn_params.num_samps;
  if (num_samps != 0) {
    params[0] = s_le_conn_params.slave_latency_events / num_samps;
    params[1] = s_le_conn_params.supervision_to_ms / num_samps;
  }

  s_le_conn_params = (LeConnectionParams){};
}

static void prv_update_conn_params(uint16_t slave_latency_events,
                                   uint16_t supervision_to_10ms) {
  bt_lock();
  s_le_conn_params.slave_latency_events += slave_latency_events;
  s_le_conn_params.supervision_to_ms += (supervision_to_10ms * 10);
  s_le_conn_params.num_samps++;
  bt_unlock();
}

void bluetooth_analytics_handle_param_update_failed(void) {
}

//! only called when we are connected as a slave
void bluetooth_analytics_handle_connection_params_update(const BleConnectionParams *params) {
  // When connected as a slave device, the 'Slave Latency' connection parameter allows
  // the controller to skip the connection sync for that number of connection events.
  prv_update_conn_params(params->slave_latency_events, params->supervision_timeout_10ms);
}

void bluetooth_analytics_handle_connection_disconnection_event(
    uint8_t reason, const BleRemoteVersionInfo *vers_info) {
  static uint32_t last_reset_counter_ticks = 0;
  static uint8_t num_events_logged = 0;

  const uint32_t ticks_per_hour = RTC_TICKS_HZ * 60 * 60;

  if ((rtc_get_ticks() - last_reset_counter_ticks) > ticks_per_hour) {
    num_events_logged = 0;
    last_reset_counter_ticks = rtc_get_ticks();
  }

  // Per-reason counters — see nimble/ble.h ble_error_codes. The NimBLE BLE_HS_HCI_ERR
  // (0x200) prefix is dropped by the uint8_t narrowing, so the raw HCI status remains.
  switch (reason) {
    case 0x08:  // BLE_ERR_CONN_SPVN_TMO
      PBL_ANALYTICS_ADD(ble_disconnect_conn_spvn_tmo_count, 1);
      break;
    case 0x13:  // BLE_ERR_REM_USER_CONN_TERM
      PBL_ANALYTICS_ADD(ble_disconnect_rem_user_term_count, 1);
      break;
    case 0x16:  // BLE_ERR_CONN_TERM_LOCAL
      PBL_ANALYTICS_ADD(ble_disconnect_conn_term_local_count, 1);
      break;
    case 0x22:  // BLE_ERR_LMP_LL_RSP_TMO
      PBL_ANALYTICS_ADD(ble_disconnect_lmp_ll_rsp_tmo_count, 1);
      break;
    case 0x3e:  // BLE_ERR_CONN_ESTABLISHMENT
      PBL_ANALYTICS_ADD(ble_disconnect_conn_establishment_count, 1);
      break;
    default:
      PBL_ANALYTICS_ADD(ble_disconnect_other_count, 1);
      break;
  }

  if (num_events_logged > 100) { // don't log a ridiculous amount of tightly looped disconnects
    return;
  }

  // It's okay to log to analytics directly from the BT02 callback thread
  // because flash writes are dispatched to KernelBG if the datalogging session
  // is buffered
  if (!vers_info) { // We expect version info
    PBL_LOG_WRN("Le Disconnect but no version info?");
  }

  num_events_logged++;
}

void bluetooth_analytics_handle_connect(
    const BTDeviceInternal *peer_addr, const BleConnectionParams *conn_params) {
  bluetooth_analytics_handle_connection_params_update(conn_params);
}

void bluetooth_analytics_handle_disconnect(bool local_is_master) {
  if (!local_is_master) {
  }
}

void bluetooth_analytics_handle_encryption_change(void) {
}

void bluetooth_analytics_handle_no_intent_for_connection(void) {
}

void bluetooth_analytics_handle_ble_pairing_request(void) {
}

void bluetooth_analytics_handle_ble_pairing_error(uint32_t error) {
}

static bool prv_calc_stats_and_print(const SlaveConnEventStats *orig_stats,
                                           SlaveConnEventStats *stats_buf, bool is_putbytes) {
  return false;
}

void bluetooth_analytics_handle_put_bytes_stats(bool successful, uint8_t type, uint32_t total_size,
                                                uint32_t elapsed_time_ms,
                                                const SlaveConnEventStats *orig_stats) {
  SlaveConnEventStats new_stats = {};
  prv_calc_stats_and_print(orig_stats, &new_stats, true /* is_putbytes */);

}

void bluetooth_analytics_handle_get_bytes_stats(uint8_t type, uint32_t total_size,
                                                uint32_t elapsed_time_ms,
                                                const SlaveConnEventStats *orig_stats) {
  SlaveConnEventStats new_stats = {};
  prv_calc_stats_and_print(orig_stats, &new_stats, false /* is_putbytes */);

}
