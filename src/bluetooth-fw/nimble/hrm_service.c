/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <bluetooth/hrm_service.h>

#include <comm/bt_lock.h>
#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_hs.h>
#include <os/os_mbuf.h>
#include <pbl/logging/logging.h>
#include <system/passert.h>

#include "nimble_type_conversions.h"

PBL_LOG_MODULE_DECLARE(bt, CONFIG_BT_LOG_LEVEL);

// BLE Heart Rate Service UUIDs
#define BLE_SVC_HRS_UUID16                     0x180D
#define BLE_SVC_HRS_CHR_UUID16_MEASUREMENT     0x2A37
#define BLE_SVC_HRS_CHR_UUID16_BODY_SENSOR_LOC 0x2A38

// Heart Rate Measurement format flags (BLUETOOTH SPEC Vol 3, Part G, §3.106)
#define HRM_FLAG_FORMAT_UINT8       0x00
#define HRM_FLAG_FORMAT_UINT16      0x01
#define HRM_FLAG_SENSOR_CONTACT_DET 0x04
#define HRM_FLAG_SENSOR_CONTACT_SUP 0x02

// Body Sensor Location values
#define HRM_BODY_SENSOR_LOC_WRIST  0x02

// Saved value handle for the HRM Measurement characteristic (set at registration)
static uint16_t s_hrm_measurement_val_handle;

static const uint8_t s_body_sensor_loc = HRM_BODY_SENSOR_LOC_WRIST;

static int prv_hrs_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {
  int rc;

  if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
    uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    if (uuid16 == BLE_SVC_HRS_CHR_UUID16_BODY_SENSOR_LOC) {
      rc = os_mbuf_append(ctxt->om, &s_body_sensor_loc, sizeof(s_body_sensor_loc));
      return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
  }

  return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def s_hrs_svc[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_HRS_UUID16),
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    // Heart Rate Measurement: notify only
                    .uuid = BLE_UUID16_DECLARE(BLE_SVC_HRS_CHR_UUID16_MEASUREMENT),
                    .access_cb = prv_hrs_access,
                    .val_handle = &s_hrm_measurement_val_handle,
                    .flags = BLE_GATT_CHR_F_NOTIFY,
                },
                {
                    // Body Sensor Location: read only
                    .uuid = BLE_UUID16_DECLARE(BLE_SVC_HRS_CHR_UUID16_BODY_SENSOR_LOC),
                    .access_cb = prv_hrs_access,
                    .flags = BLE_GATT_CHR_F_READ,
                },
                {0},
            },
    },
    {0},
};

void hrm_service_init(void) {
  int rc;

  rc = ble_gatts_count_cfg(s_hrs_svc);
  PBL_ASSERTN(rc == 0);
  rc = ble_gatts_add_svcs(s_hrs_svc);
  PBL_ASSERTN(rc == 0);
}

uint16_t hrm_service_measurement_val_handle(void) {
  return s_hrm_measurement_val_handle;
}

bool bt_driver_is_hrm_service_supported(void) {
  return true;
}

void bt_driver_hrm_service_enable(bool enable) {
  // Service is always registered; enable/disable handled via advertising + CCCD.
}

void bt_driver_hrm_service_handle_measurement(const BleHrmServiceMeasurement *measurement,
                                              const BTDeviceInternal *permitted_devices,
                                              size_t num_permitted_devices) {
  if (num_permitted_devices == 0) {
    return;
  }

  // Pack Heart Rate Measurement value per BLUETOOTH SPEC Vol 3, Part G, §3.106:
  //   Flags (1 octet) + Heart Rate Measurement Value (1 or 2 octets)
  uint8_t hrm_data[3];
  size_t len = 0;
  uint8_t flags = HRM_FLAG_FORMAT_UINT8;
  if (measurement->is_on_wrist) {
    flags |= HRM_FLAG_SENSOR_CONTACT_SUP | HRM_FLAG_SENSOR_CONTACT_DET;
  } else {
    flags |= HRM_FLAG_SENSOR_CONTACT_SUP;
  }
  hrm_data[len++] = flags;
  if (measurement->bpm > UINT8_MAX) {
    hrm_data[0] = flags | HRM_FLAG_FORMAT_UINT16;
    hrm_data[len++] = measurement->bpm & 0xff;
    hrm_data[len++] = measurement->bpm >> 8;
  } else {
    hrm_data[len++] = (uint8_t)measurement->bpm;
  }

  bt_lock();
  {
    for (size_t i = 0; i < num_permitted_devices; i++) {
      uint16_t conn_handle;
      if (!pebble_device_to_nimble_conn_handle(&permitted_devices[i], &conn_handle)) {
        PBL_LOG_DBG("HRM notify: device not connected, skipping");
        continue;
      }

      struct os_mbuf *om = ble_hs_mbuf_from_flat(hrm_data, len);
      if (om == NULL) {
        PBL_LOG_ERR("HRM notify: failed to allocate mbuf");
        break;
      }

      int rc = ble_gatts_notify_custom(conn_handle, s_hrm_measurement_val_handle, om);
      if (rc != 0) {
        PBL_LOG_DBG("HRM notify: failed for conn_handle=%u rc=0x%04x",
                     conn_handle, (uint16_t)rc);
      }
    }
  }
  bt_unlock();
}
