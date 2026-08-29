/* SPDX-FileCopyrightText: 2025 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stdio.h>

#include <bluetooth/bonding_sync.h>
#include <bluetooth/bt_driver_advert.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hrm_service.h>
#include <bluetooth/pairing_confirm.h>
#include <comm/bt_lock.h>
#include <host/ble_gap.h>
#include <host/ble_hs_hci.h>
#include <kernel/pbl_malloc.h>
#include <os/os_mbuf.h>
#include <pbl/logging/logging.h>
#include <system/passert.h>
#include <pbl/util/math.h>

#include "hrm_service.h"
#include "nimble_gattc_op_queue.h"
#include "nimble_type_conversions.h"

PBL_LOG_MODULE_DECLARE(bt, CONFIG_BT_LOG_LEVEL);

static const ble_uuid16_t s_device_name_chr_uuid = BLE_UUID16_INIT(0x2A00);
static char s_device_name[BT_DEVICE_NAME_BUFFER_SIZE];
static bool s_pairing_in_progress;

static int prv_device_name_read_event_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                                         struct ble_gatt_attr *attr, void *arg) {
  if (error->status != 0) {
    nimble_gattc_op_queue_complete();
    return 0;
  }

  size_t len = MIN(OS_MBUF_PKTLEN(attr->om), sizeof(s_device_name) - 1);
  os_mbuf_copydata(attr->om, 0, len, s_device_name);
  s_device_name[len] = '\0';

  return 0;
}

static int prv_device_name_read_op_start(void *ctx) {
  const uint16_t conn_handle = *(uint16_t *)ctx;

  int rc = ble_gattc_read_by_uuid(conn_handle, 1, UINT16_MAX,
                                  (ble_uuid_t *)&s_device_name_chr_uuid,
                                  prv_device_name_read_event_cb, NULL);
  if (rc != 0) {
    PBL_LOG_ERR("Pairing device name read failed to start (rc=0x%04x)", (uint16_t)rc);
  }

  return rc;
}

void bt_driver_advert_advertising_disable(void) {
  int rc;

  if (ble_gap_adv_active() == 0) {
    return;
  }

  rc = ble_gap_adv_stop();
  PBL_ASSERT(rc == 0, "Failed to stop advertising (0x%04x)", (uint16_t)rc);
}

bool bt_driver_advert_client_get_tx_power(int8_t *tx_power) { return false; }

bool bt_driver_advert_set_advertising_data(const BLEAdData *ad_data) {
  int rc;

  rc = ble_gap_adv_set_data((uint8_t *)&ad_data->data, ad_data->ad_data_length);
  if (rc != 0) {
    PBL_LOG_ERR("Failed to set advertising data (0x%04x)", (uint16_t)rc);
    return false;
  }

  rc = ble_gap_adv_rsp_set_data((uint8_t *)&ad_data->data[ad_data->ad_data_length],
                                ad_data->scan_resp_data_length);
  if (rc != 0) {
    PBL_LOG_ERR("Failed to set scan response data (0x%04x)", (uint16_t)rc);
    return false;
  }

  return true;
}

static void prv_handle_connection_event(struct ble_gap_event *event) {
  // we only want to notify on a successful connection
  if (event->connect.status != 0) return;

  struct ble_gap_conn_desc desc;
  if (ble_gap_conn_find(event->connect.conn_handle, &desc) != 0) {
    PBL_LOG_ERR("prv_handle_connection_event: Failed to find connection descriptor");
    return;
  }

  struct BleConnectionCompleteEvent complete_event = {
      .handle = event->connect.conn_handle,
      .is_master = desc.role == BLE_GAP_ROLE_MASTER,
      .status = HciStatusCode_Success,
      .mtu = ble_att_mtu(event->connect.conn_handle),
  };

  // If OTA address != ID address, then the address must be resolved.
  // This happens for an already paired devices.
  complete_event.is_resolved = ble_addr_cmp(&desc.peer_id_addr, &desc.peer_ota_addr) != 0;

  {
    BTDeviceAddress ota_addr, id_addr;
    nimble_addr_to_pebble_addr(&desc.peer_ota_addr, &ota_addr);
    nimble_addr_to_pebble_addr(&desc.peer_id_addr, &id_addr);
    PBL_LOG_DBG("Conn compl: ota=" BT_DEVICE_ADDRESS_FMT " atype=%u",
                BT_DEVICE_ADDRESS_XPLODE(ota_addr), desc.peer_ota_addr.type);
    PBL_LOG_DBG("Conn compl: id=" BT_DEVICE_ADDRESS_FMT " atype=%u",
                BT_DEVICE_ADDRESS_XPLODE(id_addr), desc.peer_id_addr.type);
  }

  if (complete_event.is_resolved) {
    int rc;
    struct ble_store_key_sec key_sec;
    struct ble_store_value_sec value_sec;

    key_sec.idx = 0;
    key_sec.peer_addr = desc.peer_id_addr;

    rc = ble_store_read_peer_sec(&key_sec, &value_sec);
    if (rc != 0) {
      // We can get a resolved address in case of a repeated pairing event,
      // where peer security is deleted. An identity resolved event will be
      // received later after the new pairing is completed.
      PBL_LOG_INFO("Address resolved but no stored peer security (rc=%d)", rc);
      complete_event.is_resolved = false;
    } else {
      memcpy(complete_event.irk.data, value_sec.irk, 16);
    }
  } else {
    // If the address is not resolved, pairing is gonna happen.
    // Trigger name read to have it ready for the pairing confirmation.
    memset(s_device_name, 0, sizeof(s_device_name));
    uint16_t *conn_handle = kernel_malloc_check(sizeof(*conn_handle));
    *conn_handle = event->connect.conn_handle;
    nimble_gattc_op_queue_push(prv_device_name_read_op_start, conn_handle);
  }

  nimble_conn_params_to_pebble(&desc, &complete_event.conn_params);
  nimble_addr_to_pebble_device(&desc.peer_id_addr, &complete_event.peer_address);

  s_pairing_in_progress = false;

  bt_driver_handle_le_connection_complete_event(&complete_event);
}

static void prv_handle_disconnection_event(struct ble_gap_event *event) {
  GattDeviceDisconnectionEvent gatt_event;
  nimble_addr_to_pebble_addr(&event->disconnect.conn.peer_id_addr, &gatt_event.dev_address);
  bt_driver_cb_gatt_handle_disconnect(&gatt_event);

  struct BleDisconnectionCompleteEvent disconnection_event = {
      .handle = event->disconnect.conn.conn_handle,
      .reason = event->disconnect.reason,
      .status = HciStatusCode_Success,
  };
  nimble_addr_to_pebble_device(&event->disconnect.conn.peer_id_addr,
                               &disconnection_event.peer_address);
  bt_driver_handle_le_disconnection_complete_event(&disconnection_event);
}

static void prv_handle_enc_change_event(struct ble_gap_event *event) {
  struct ble_gap_conn_desc desc;
  if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) != 0) {
    PBL_LOG_ERR("prv_handle_enc_change_event: Failed to find connection descriptor");
    return;
  }

  PBL_LOG_INFO("Encryption change: status=0x%04x encrypted=%u bonded=%u",
               (uint16_t)event->enc_change.status, desc.sec_state.encrypted,
               desc.sec_state.bonded);

  struct BleEncryptionChange enc_change_event = {
      .encryption_enabled = desc.sec_state.encrypted,
      .status =
          event->enc_change.status,  // doesn't technically match but only logged so this is fine
  };
  nimble_addr_to_pebble_addr(&desc.peer_id_addr, &enc_change_event.dev_address);
  bt_driver_handle_le_encryption_change_event(&enc_change_event);
}

static void prv_handle_conn_params_updated_event(struct ble_gap_event *event) {
  if (event->conn_update.status != 0) {
    PBL_LOG_ERR("Connection parameters update failed: 0x%04x",
              (uint16_t)event->conn_update.status);
    return;
  }

  struct ble_gap_conn_desc desc;
  if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) != 0) {
    PBL_LOG_ERR("prv_handle_conn_params_updated_event: Failed to find connection descriptor");
    return;
  }

  PBL_LOG_INFO("Connection parameters updated: "
            "itvl=%u ms, latency=%u, spvn timeout=%u ms",
            desc.conn_itvl * BLE_HCI_CONN_ITVL / 1000, desc.conn_latency,
            desc.supervision_timeout * BLE_HCI_CONN_SPVN_TMO_UNITS);

  struct BleConnectionUpdateCompleteEvent conn_params_update_event = {
      .status = HciStatusCode_Success,
  };
  nimble_conn_params_to_pebble(&desc, &conn_params_update_event.conn_params);
  nimble_addr_to_pebble_addr(&desc.peer_id_addr, &conn_params_update_event.dev_address);

  bt_driver_handle_le_conn_params_update_event(&conn_params_update_event);
}

static void prv_handle_conn_update_req_event(struct ble_gap_event *event) {
  *event->conn_update_req.self_params = *event->conn_update_req.peer_params;

  PBL_LOG_INFO("Connection update request: "
            "itvl=(%u, %u) ms, latency=%u, spvn timeout=%u ms",
            event->conn_update_req.self_params->itvl_min * BLE_HCI_CONN_ITVL / 1000,
            event->conn_update_req.self_params->itvl_max * BLE_HCI_CONN_ITVL / 1000,
            event->conn_update_req.self_params->latency,
            event->conn_update_req.self_params->supervision_timeout * BLE_HCI_CONN_SPVN_TMO_UNITS);
}

static void prv_handle_passkey_event(struct ble_gap_event *event) {
  char passkey_str[7];
  uint32_t passkey = 0;
  const char *device_name = NULL;
  PairingUserConfirmationCtx *ctx =
      (PairingUserConfirmationCtx *)((uintptr_t)event->passkey.conn_handle);

  if (event->passkey.params.action == BLE_SM_IOACT_NUMCMP) {
    passkey = event->passkey.params.numcmp;
  }

  if (s_device_name[0] != '\0') {
    device_name = s_device_name;
  }

  snprintf(passkey_str, sizeof(passkey_str), "%06lu", passkey);
  bt_driver_cb_pairing_confirm_handle_request(ctx, device_name, passkey_str);
  s_pairing_in_progress = true;
}

static void prv_handle_pairing_complete_event(struct ble_gap_event *event) {
  PBL_LOG_INFO("Pairing complete: status=0x%04x", (uint16_t)event->pairing_complete.status);

  if (!s_pairing_in_progress) {
    return;
  }

  PairingUserConfirmationCtx *ctx =
      (PairingUserConfirmationCtx *)((uintptr_t)event->pairing_complete.conn_handle);
  bt_driver_cb_pairing_confirm_handle_completed(ctx, event->pairing_complete.status == 0);
  s_pairing_in_progress = false;
}

static void prv_handle_identity_resolved_event(struct ble_gap_event *event) {
  struct ble_gap_conn_desc desc;
  if (ble_gap_conn_find(event->identity_resolved.conn_handle, &desc) != 0) {
    PBL_LOG_ERR("prv_handle_identity_resolved_event: Failed to find connection descriptor");
    return;
  }

  BleAddressChange addr_change_event;
  nimble_addr_to_pebble_device(&desc.peer_ota_addr, &addr_change_event.device);
  nimble_addr_to_pebble_device(&desc.peer_id_addr, &addr_change_event.new_device);
  bt_driver_handle_le_connection_handle_update_address(&addr_change_event);
}

static void prv_handle_mtu_change_event(struct ble_gap_event *event) {
  struct ble_gap_conn_desc desc;
  if (ble_gap_conn_find(event->mtu.conn_handle, &desc) != 0) {
    PBL_LOG_ERR("prv_handle_mtu_change_event: Failed to find connection descriptor");
    return;
  }

  GattDeviceMtuUpdateEvent mtu_update_event = {.mtu = event->mtu.value};
  nimble_addr_to_pebble_addr(&desc.peer_id_addr, &mtu_update_event.dev_address);
  bt_driver_cb_gatt_handle_mtu_update(&mtu_update_event);
}

extern int pebble_pairing_service_get_connectivity_send_notification(uint16_t conn_handle,
                                                                     uint16_t attr_handle);
static void prv_handle_subscription_event(struct ble_gap_event *event) {
  PBL_LOG_DBG("prv_handle_subscription_event: connhandle: %d attr:%d notify:%d/%d indicate:%d/%d",
            event->subscribe.conn_handle, event->subscribe.attr_handle,
            event->subscribe.prev_notify, event->subscribe.cur_notify,
            event->subscribe.prev_indicate, event->subscribe.cur_indicate);

#ifdef CONFIG_BLE_HRM_SERVICE
  if (event->subscribe.attr_handle == hrm_service_measurement_val_handle()) {
    // NimBLE reports CCCD writes using the characteristic value handle.
    bool is_subscribed = (bool)event->subscribe.cur_notify;
    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find(event->subscribe.conn_handle, &desc) == 0) {
      BTDeviceInternal device;
      nimble_addr_to_pebble_device(&desc.peer_id_addr, &device);
      bt_driver_cb_hrm_service_update_subscription(&device, is_subscribed);
    } else {
      PBL_LOG_WRN("HRM subscribe: conn %u not found",
                   event->subscribe.conn_handle);
    }
  }
#endif // CONFIG_BLE_HRM_SERVICE
}

static void prv_handle_notification_rx_event(struct ble_gap_event *event) {
  struct ble_gap_conn_desc desc;
  if (ble_gap_conn_find(event->notify_rx.conn_handle, &desc) != 0) {
    PBL_LOG_ERR("prv_handle_notification_rx_event: Failed to find connection descriptor");
    return;
  }

  GattServerNotifIndicEvent notification_event = {
      .attr_handle = event->notify_rx.attr_handle,
      .attr_val = event->notify_rx.om->om_data,
      .attr_val_len = event->notify_rx.om->om_len,
  };
  nimble_addr_to_pebble_addr(&desc.peer_id_addr, &notification_event.dev_address);

  if (event->notify_rx.indication == 1) {
    bt_driver_cb_gatt_handle_indication(&notification_event);
  } else {
    bt_driver_cb_gatt_handle_notification(&notification_event);
  }
}

static void prv_handle_notification_tx_event(struct ble_gap_event *event) {
  PBL_LOG_DBG("notification tx event; status=%d attr_handle=%d indication=%d\n",
            event->notify_tx.status,
            event->notify_tx.attr_handle,
            event->notify_tx.indication);
}

static int prv_handle_repeat_pairing_event(struct ble_gap_event *event) {
  // In recovery mode there is no UI that allows to manually delete a pairing,
  // so we unconditionally enable repeat pairing. In main firmware, only allow
  // repeat pairing if using secure connections and we support user confirmation.
#if defined(CONFIG_RECOVERY_FW) || \
    (MYNEWT_VAL(BLE_SM_SC_ONLY) && (MYNEWT_VAL(BLE_SM_IO_CAP) == BLE_HS_IO_DISPLAY_YESNO))
  struct ble_gap_conn_desc desc;
  int ret;

  ret = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
  if (ret != 0) {
    return ret;
  }

  PBL_LOG_INFO("Repeat pairing: deleting stored peer keys and retrying");
  ble_store_util_delete_peer(&desc.peer_id_addr);

  return BLE_GAP_REPEAT_PAIRING_RETRY;
#else
  PBL_LOG_WRN("BLE_GAP_EVENT_REPEAT_PAIRING ignored");
  return BLE_GAP_REPEAT_PAIRING_IGNORE;
#endif
}

static void prv_handle_phy_update_event(struct ble_gap_event *event) {
  if (event->phy_updated.status != 0) {
    PBL_LOG_ERR("PHY update failed: 0x%04x",
              (uint16_t)event->phy_updated.status);
    return;
  }

  PBL_LOG_DBG("PHY update complete; conn_handle=%d, tx_phy=%d, rx_phy=%d",
          event->phy_updated.conn_handle, event->phy_updated.tx_phy, event->phy_updated.rx_phy);
}

static int prv_handle_gap_event(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
      PBL_LOG_DBG("BLE_GAP_EVENT_CONNECT");
      prv_handle_connection_event(event);
      break;
    case BLE_GAP_EVENT_DISCONNECT:
      PBL_LOG_DBG("BLE_GAP_EVENT_DISCONNECT reason=0x%x",
              event->disconnect.reason);
      prv_handle_disconnection_event(event);
      break;
    case BLE_GAP_EVENT_ENC_CHANGE:
      PBL_LOG_DBG("BLE_GAP_EVENT_ENC_CHANGE");
      prv_handle_enc_change_event(event);
      break;
    case BLE_GAP_EVENT_CONN_UPDATE:
      PBL_LOG_DBG("BLE_GAP_EVENT_CONN_UPDATE");
      prv_handle_conn_params_updated_event(event);
      break;
    case BLE_GAP_EVENT_CONN_UPDATE_REQ:
      PBL_LOG_DBG("BLE_GAP_EVENT_CONN_UPDATE_REQ");
      prv_handle_conn_update_req_event(event);
      break;
    case BLE_GAP_EVENT_PASSKEY_ACTION:
      PBL_LOG_DBG("BLE_GAP_EVENT_PASSKEY_ACTION");
      prv_handle_passkey_event(event);
      break;
    case BLE_GAP_EVENT_IDENTITY_RESOLVED:
      PBL_LOG_DBG("BLE_GAP_EVENT_IDENTITY_RESOLVED");
      prv_handle_identity_resolved_event(event);
      break;
    case BLE_GAP_EVENT_PAIRING_COMPLETE:
      PBL_LOG_DBG("BLE_GAP_EVENT_PAIRING_COMPLETE");
      prv_handle_pairing_complete_event(event);
      break;
    case BLE_GAP_EVENT_MTU:
      PBL_LOG_DBG("BLE_GAP_EVENT_MTU");
      prv_handle_mtu_change_event(event);
      break;
    case BLE_GAP_EVENT_SUBSCRIBE:
      PBL_LOG_DBG("BLE_GAP_EVENT_SUBSCRIBE");
      prv_handle_subscription_event(event);
      break;
    case BLE_GAP_EVENT_NOTIFY_RX:
      // no log here because it's incredibly noisy
      prv_handle_notification_rx_event(event);
      break;
    case BLE_GAP_EVENT_NOTIFY_TX:
      PBL_LOG_DBG("BLE_GAP_EVENT_NOTIFY_TX");
      prv_handle_notification_tx_event(event);
      break;
    case BLE_GAP_EVENT_REPEAT_PAIRING:
      PBL_LOG_DBG("BLE_GAP_EVENT_REPEAT_PAIRING");
      return prv_handle_repeat_pairing_event(event);
    case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
      PBL_LOG_DBG("BLE_GAP_EVENT_PHY_UPDATE_COMPLETE");
      prv_handle_phy_update_event(event);
      break;
    default:
      PBL_LOG_WRN("Unhandled GAP event: %d", event->type);
      break;
  }
  return 0;
}

bool bt_driver_advert_advertising_enable(uint32_t min_interval_ms, uint32_t max_interval_ms) {
  int rc;
  uint8_t own_addr_type;
  struct ble_gap_adv_params advp = {
      .conn_mode = BLE_GAP_CONN_MODE_UND,
      .disc_mode = BLE_GAP_DISC_MODE_GEN,
      .itvl_min = BLE_GAP_ADV_ITVL_MS(min_interval_ms),
      .itvl_max = BLE_GAP_ADV_ITVL_MS(max_interval_ms),
  };

  rc = ble_hs_id_infer_auto(0, &own_addr_type);
  if (rc != 0) {
    PBL_LOG_ERR("Failed to infer own address type (%d)", rc);
    return false;
  }

  rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &advp, prv_handle_gap_event, NULL);
  if (rc != 0) {
    PBL_LOG_ERR("Failed to start advertising (0x%04x)", (uint16_t)rc);
    return false;
  }

  return true;
}
