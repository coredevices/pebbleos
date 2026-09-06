/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "bf0_hal_efuse.h"

#define NVDS_BUFF_START 0x2040FE00

static const uint8_t s_ble_slp_default[] = {
    // Control pre-wakeup time for the sleep of BT subsytem in LCPU.
    // See SiFli-SDK EXT_WAKEUP_TIME_LXT32K/EXT_WAKEUP_TIME_RC10K for details.
    // RC10K -> 4500us (0x1194)
    // LXT32K -> 3500us (0x0DAC)
#ifdef LXT_DISABLE
    0x0D, 0x02, 0x19, 0x11,
#else
    0x0D, 0x02, 0xAC, 0x0D,
#endif
    // Control maximum sleep duration of BT subsystem.
    // The last 0x01 means 10s in BLE only and 30s in dual mode. 0 means 500ms
    0x12, 0x01, 0x01,
    // Control the log in controller
    // Changed to 0x20, 0x00, 0x09, 0x00 will enable HCI logs by default
    0x2F, 0x04, 0x20, 0x00, 0x00, 0x00,
    // Internal usage, for scheduling
    0x15, 0x01, 0x01,
};

static int prv_bt_mac_addr_generate(uint8_t mac_addr[6]) {
  uint8_t uid[8];
  int32_t ret;
  uint8_t chksum;
  uint8_t i;

  ret = HAL_EFUSE_Read(0, uid, 8U);
  if (ret != 8) {
    return false;
  }

  for (i = 0U; i < 8U; i++) {
    if (uid[i] != 0U) {
      break;
    }
  }

  if (i >= 8U) {
    return false;
  }

  if (uid[7] != 0xA5) {
    return false;
  }

  chksum = uid[0] + uid[1] + uid[2] + uid[3] + uid[4] + uid[5];
  if (chksum != uid[6]) {
    return false;
  }

  memcpy(mac_addr, uid, 6);

  return true;
}

void lcpu_custom_nvds_config(void) {
  uint8_t *nvds_addr = (uint8_t *)NVDS_BUFF_START;
  uint8_t mac_addr[6];
  bool res;

  res = prv_bt_mac_addr_generate(mac_addr);
  assert(res);

  *(uint32_t *)nvds_addr = 0x4E564453;
  *(uint16_t *)(nvds_addr + 4) = sizeof(s_ble_slp_default) + 8U;
  *(uint16_t *)(nvds_addr + 6) = 0;

  *(uint8_t *)(nvds_addr + 8) = 0x01;
  *(uint8_t *)(nvds_addr + 9) = 0x06;
  memcpy(nvds_addr + 10, mac_addr, 6);

  memcpy(nvds_addr + 16, s_ble_slp_default, sizeof(s_ble_slp_default));
}
