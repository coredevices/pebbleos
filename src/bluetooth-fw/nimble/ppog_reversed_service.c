/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "ppog_reversed_service.h"

#include <bluetooth/bt_driver_ppog_reversed.h>
#include <bluetooth/pebble_bt.h>
#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_hs.h>
#include <host/ble_uuid.h>
#include <kernel/pbl_malloc.h>
#include <os/os_mbuf.h>
#include <pbl/logging/logging.h>
#include <system/passert.h>

#include "nimble_type_conversions.h"

PBL_LOG_MODULE_DECLARE(bt, CONFIG_BT_LOG_LEVEL);

static uint16_t s_data_notify_handle;
static uint16_t s_data_write_handle;

static int prv_access_data_notify(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg) {
  // Notify-only — refuse explicit reads.
  return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

static int prv_access_data_write(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
    return BLE_ATT_ERR_UNLIKELY;
  }
  // Flatten into a kernel-heap buffer; ownership passes to the kernel callback.
  const uint16_t pkt_len = OS_MBUF_PKTLEN(ctxt->om);
  if (pkt_len == 0) {
    return 0;
  }
  uint8_t *buf = kernel_malloc(pkt_len);
  if (!buf) {
    return BLE_ATT_ERR_INSUFFICIENT_RES;
  }
  uint16_t out_len = 0;
  int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, pkt_len, &out_len);
  if (rc != 0) {
    PBL_LOG_ERR("Reversed PPoG write flatten failed: 0x%04x", (uint16_t) rc);
    kernel_free(buf);
    return BLE_ATT_ERR_UNLIKELY;
  }
  bt_driver_cb_ppog_reversed_data_written(conn_handle, buf, out_len);
  return 0;
}

static const struct ble_gatt_svc_def s_ppog_reversed_svc[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID128_DECLARE(BLE_UUID_SWIZZLE(
            PEBBLE_BT_UUID_EXPAND(PEBBLE_BT_PPOGATT_WATCH_SERVER_SERVICE_UUID_32BIT))),
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = BLE_UUID128_DECLARE(BLE_UUID_SWIZZLE(PEBBLE_BT_UUID_EXPAND(
                        PEBBLE_BT_PPOGATT_WATCH_SERVER_DATA_CHARACTERISTIC_UUID_32BIT))),
                    .access_cb = prv_access_data_notify,
                    // READ_ENC (without READ) gates the CCCD: subscribing
                    // requires an encrypted link, explicit reads stay blocked.
                    .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ_ENC,
                    .val_handle = &s_data_notify_handle,
                },
                {
                    .uuid = BLE_UUID128_DECLARE(BLE_UUID_SWIZZLE(PEBBLE_BT_UUID_EXPAND(
                        PEBBLE_BT_PPOGATT_WATCH_SERVER_DATA_WR_CHARACTERISTIC_UUID_32BIT))),
                    .access_cb = prv_access_data_write,
                    .flags = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE_ENC,
                    .val_handle = &s_data_write_handle,
                },
                {
                    0,
                },
            },
    },
    {
        0,
    },
};

static void prv_handle_subscribe_event(struct ble_gap_event *event) {
  if (event->subscribe.attr_handle != s_data_notify_handle) {
    return;
  }
  if (event->subscribe.cur_notify == event->subscribe.prev_notify) {
    return;
  }
  if (event->subscribe.cur_notify) {
    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find(event->subscribe.conn_handle, &desc) != 0) {
      PBL_LOG_ERR("Reversed PPoG subscribe: no conn descriptor for handle %u",
                  event->subscribe.conn_handle);
      return;
    }
    BTDeviceInternal device;
    nimble_addr_to_pebble_device(&desc.peer_id_addr, &device);
    bt_driver_cb_ppog_reversed_subscribed(&device, event->subscribe.conn_handle);
  } else {
    bt_driver_cb_ppog_reversed_unsubscribed(event->subscribe.conn_handle);
  }
}

static int prv_handle_gap_event(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
    case BLE_GAP_EVENT_SUBSCRIBE:
      prv_handle_subscribe_event(event);
      break;
    default:
      break;
  }
  return 0;
}

static struct ble_gap_event_listener s_gap_event_listener;

void ppog_reversed_service_init(void) {
  int rc = ble_gatts_count_cfg(s_ppog_reversed_svc);
  PBL_ASSERTN(rc == 0);
  rc = ble_gatts_add_svcs(s_ppog_reversed_svc);
  PBL_ASSERTN(rc == 0);
  // Called on every bt_driver_start, but the listener list is only cleared on
  // nimble_port_init, so tolerate EALREADY.
  rc = ble_gap_event_listener_register(&s_gap_event_listener, prv_handle_gap_event, NULL);
  PBL_ASSERTN(rc == 0 || rc == BLE_HS_EALREADY);
}

BTErrno bt_driver_ppog_reversed_notify(uint16_t conn_handle,
                                       const uint8_t *buf, uint16_t len) {
  struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, len);
  if (!om) {
    return BTErrnoNotEnoughResources;
  }
  int rc = ble_gatts_notify_custom(conn_handle, s_data_notify_handle, om);
  // ble_gatts_notify_custom always consumes the mbuf, even on error.
  switch (rc) {
    case 0:
      return BTErrnoOK;
    case BLE_HS_ENOMEM:
      return BTErrnoNotEnoughResources;
    case BLE_HS_ENOTCONN:
      return BTErrnoInvalidState;
    default:
      PBL_LOG_ERR("ble_gatts_notify_custom failed: 0x%04x", (uint16_t) rc);
      return (BTErrno)(BTErrnoInternalErrorBegin + rc);
  }
}
