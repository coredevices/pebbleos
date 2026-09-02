/* SPDX-FileCopyrightText: 2025 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "system/passert.h"
#include "nimble_type_conversions.h"

#include <pbl/btutil/bt_uuid.h>
#include <host/ble_gap.h>
#include <string.h>
#include <pbl/logging/logging.h>

PBL_LOG_MODULE_DECLARE(bt, CONFIG_BT_LOG_LEVEL);

void nimble_addr_to_pebble_addr(const ble_addr_t *addr, BTDeviceAddress *addr_out) {
  memcpy(&addr_out->octets, &addr->val, BLE_DEV_ADDR_LEN);
}

void pebble_device_to_nimble_addr(const BTDeviceInternal *device, ble_addr_t *addr_out) {
  addr_out->type = device->is_random_address ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
  memcpy(&addr_out->val, &device->address.octets, BLE_DEV_ADDR_LEN);
}

void nimble_addr_to_pebble_device(const ble_addr_t *stack_addr, BTDeviceInternal *host_addr) {
  nimble_addr_to_pebble_addr(stack_addr, &host_addr->address);
  host_addr->is_random_address = stack_addr->type == BLE_ADDR_RANDOM;
  host_addr->is_classic = false;
}

bool pebble_device_to_nimble_conn_handle(const BTDeviceInternal *device, uint16_t *handle) {
  ble_addr_t addr;
  struct ble_gap_conn_desc desc;

  pebble_device_to_nimble_addr(device, &addr);

  int rc = ble_gap_conn_find_by_addr(&addr, &desc);
  if (rc != 0) {
    PBL_LOG_ERR("failed to find connection handle for addr" BT_DEVICE_ADDRESS_FMT,
      BT_DEVICE_ADDRESS_XPLODE(device->address));
  } else {
    *handle = desc.conn_handle;
  }

  return rc == 0;
}

void nimble_conn_params_to_pebble(struct ble_gap_conn_desc *desc, BleConnectionParams *params) {
  params->conn_interval_1_25ms = desc->conn_itvl;
  params->slave_latency_events = desc->conn_latency;
  params->supervision_timeout_10ms = desc->supervision_timeout;
}

void pebble_conn_update_to_nimble(const BleConnectionParamsUpdateReq *req,
                                  struct ble_gap_upd_params *params) {
  params->itvl_min = req->interval_min_1_25ms;
  params->itvl_max = req->interval_max_1_25ms;
  params->latency = req->slave_latency_events;
  params->supervision_timeout = req->supervision_timeout_10ms;
  params->min_ce_len = 0U;
  params->max_ce_len = 0U;
}

void nimble_uuid_to_pebble(const ble_uuid_any_t *stack_uuid, Uuid *uuid) {
  switch (stack_uuid->u.type) {
    case BLE_UUID_TYPE_16:
      *uuid = bt_uuid_expand_16bit(stack_uuid->u16.value);
      break;
    case BLE_UUID_TYPE_32:
      *uuid = bt_uuid_expand_32bit(stack_uuid->u32.value);
      break;
    case BLE_UUID_TYPE_128:
      *uuid = UuidMakeFromLEBytes(stack_uuid->u128.value);
      break;
    default:
      PBL_ASSERTN(0);
  }
}
