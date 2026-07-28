/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "dis_service.h"

#include <board/board.h>
#include <host/ble_gatt.h>
#include <host/ble_hs.h>
#include <host/ble_uuid.h>
#include <os/os_mbuf.h>
#include <string.h>
#include <system/passert.h>

#define DIS_SERVICE_UUID               (0x180A)
#define DIS_CHR_SYSTEM_ID_UUID         (0x2A23)
#define DIS_CHR_MODEL_NUMBER_UUID      (0x2A24)
#define DIS_CHR_SERIAL_NUMBER_UUID     (0x2A25)
#define DIS_CHR_FIRMWARE_REVISION_UUID (0x2A26)
#define DIS_CHR_HARDWARE_REVISION_UUID (0x2A27)
#define DIS_CHR_SOFTWARE_REVISION_UUID (0x2A28)
#define DIS_CHR_MANUFACTURER_NAME_UUID (0x2A29)
#define DIS_CHR_PNP_ID_UUID            (0x2A50)

// BT_VENDOR_ID is a Bluetooth SIG company identifier (it is what we put in the
// company id field of our manufacturer-specific advertising data), so the PnP
// vendor id source says "Bluetooth SIG" rather than "USB Implementer's Forum".
#define DIS_PNP_VENDOR_ID_SOURCE_BT_SIG (0x01)
#define DIS_PNP_PRODUCT_ID              (0x0001)
#define DIS_PNP_PRODUCT_VERSION         (0x0100)

typedef enum {
  DisFieldModelNumber = 0,
  DisFieldSerialNumber,
  DisFieldFirmwareRevision,
  DisFieldSoftwareRevision,
  DisFieldManufacturerName,
  //! Registered but never populated, exactly as ble_svc_dis left them: both
  //! read back empty. Kept so the pre-existing DIS characteristics keep their
  //! ATT handles.
  DisFieldHardwareRevision,
  DisFieldSystemId,
} DisField;

// PnP ID, 7 bytes little-endian.
static const uint8_t s_pnp_id[] = {
    DIS_PNP_VENDOR_ID_SOURCE_BT_SIG,
    (uint8_t)(BT_VENDOR_ID & 0xFFU),
    (uint8_t)((BT_VENDOR_ID >> 8) & 0xFFU),
    (uint8_t)(DIS_PNP_PRODUCT_ID & 0xFFU),
    (uint8_t)((DIS_PNP_PRODUCT_ID >> 8) & 0xFFU),
    (uint8_t)(DIS_PNP_PRODUCT_VERSION & 0xFFU),
    (uint8_t)((DIS_PNP_PRODUCT_VERSION >> 8) & 0xFFU),
};

static const DisInfo *s_dis_info;

static const char *prv_field_value(DisField field) {
  if (!s_dis_info) {
    return "";
  }
  switch (field) {
    case DisFieldModelNumber:
      return s_dis_info->model_number;
    case DisFieldSerialNumber:
      return s_dis_info->serial_number;
    case DisFieldFirmwareRevision:
      return s_dis_info->fw_revision;
    case DisFieldSoftwareRevision:
      return s_dis_info->sw_revision;
    case DisFieldManufacturerName:
      return s_dis_info->manufacturer;
    case DisFieldHardwareRevision:
    case DisFieldSystemId:
    default:
      return "";
  }
}

static int prv_access_string(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
    return BLE_ATT_ERR_UNLIKELY;
  }
  const char *value = prv_field_value((DisField)(uintptr_t)arg);
  // Not zero terminated by design.
  return (os_mbuf_append(ctxt->om, value, strlen(value)) == 0) ? 0
                                                              : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int prv_access_pnp_id(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
    return BLE_ATT_ERR_UNLIKELY;
  }
  return (os_mbuf_append(ctxt->om, s_pnp_id, sizeof(s_pnp_id)) == 0)
             ? 0
             : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// Characteristic order is exactly the one of ble_svc_dis_defs[], with PnP ID
// appended, so every pre-existing DIS characteristic keeps its ATT handle. DIS
// still grows from 15 to 17 attributes, which shifts every service registered
// after it by +2. A normal OTA update reboots with
// RebootReasonCode_SoftwareUpdate, which arms the Service Changed indication so
// bonded peers rediscover; a firmware flashed over SWD/DFU does not, and leaves
// those peers with a stale GATT cache.
static const struct ble_gatt_svc_def s_dis_svc[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(DIS_SERVICE_UUID),
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = BLE_UUID16_DECLARE(DIS_CHR_MODEL_NUMBER_UUID),
                    .access_cb = prv_access_string,
                    .arg = (void *)(uintptr_t)DisFieldModelNumber,
                    .flags = BLE_GATT_CHR_F_READ,
                },
                {
                    .uuid = BLE_UUID16_DECLARE(DIS_CHR_SERIAL_NUMBER_UUID),
                    .access_cb = prv_access_string,
                    .arg = (void *)(uintptr_t)DisFieldSerialNumber,
                    .flags = BLE_GATT_CHR_F_READ,
                },
                {
                    .uuid = BLE_UUID16_DECLARE(DIS_CHR_HARDWARE_REVISION_UUID),
                    .access_cb = prv_access_string,
                    .arg = (void *)(uintptr_t)DisFieldHardwareRevision,
                    .flags = BLE_GATT_CHR_F_READ,
                },
                {
                    .uuid = BLE_UUID16_DECLARE(DIS_CHR_FIRMWARE_REVISION_UUID),
                    .access_cb = prv_access_string,
                    .arg = (void *)(uintptr_t)DisFieldFirmwareRevision,
                    .flags = BLE_GATT_CHR_F_READ,
                },
                {
                    .uuid = BLE_UUID16_DECLARE(DIS_CHR_SOFTWARE_REVISION_UUID),
                    .access_cb = prv_access_string,
                    .arg = (void *)(uintptr_t)DisFieldSoftwareRevision,
                    .flags = BLE_GATT_CHR_F_READ,
                },
                {
                    .uuid = BLE_UUID16_DECLARE(DIS_CHR_MANUFACTURER_NAME_UUID),
                    .access_cb = prv_access_string,
                    .arg = (void *)(uintptr_t)DisFieldManufacturerName,
                    .flags = BLE_GATT_CHR_F_READ,
                },
                {
                    .uuid = BLE_UUID16_DECLARE(DIS_CHR_SYSTEM_ID_UUID),
                    .access_cb = prv_access_string,
                    .arg = (void *)(uintptr_t)DisFieldSystemId,
                    .flags = BLE_GATT_CHR_F_READ,
                },
                {
                    // Mandated by HOGP for a HID device. Appended last so the
                    // characteristics above keep their handles.
                    .uuid = BLE_UUID16_DECLARE(DIS_CHR_PNP_ID_UUID),
                    .access_cb = prv_access_pnp_id,
                    .flags = BLE_GATT_CHR_F_READ,
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

void dis_service_init(const DisInfo *info) {
  s_dis_info = info;

  int rc = ble_gatts_count_cfg(s_dis_svc);
  PBL_ASSERTN(rc == 0);
  rc = ble_gatts_add_svcs(s_dis_svc);
  PBL_ASSERTN(rc == 0);
}
