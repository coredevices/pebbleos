/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/bluetooth/bluetooth_persistent_storage_debug.h"

#include "console/prompt.h"
#include "pbl/services/shared_prf_storage/shared_prf_storage_debug.h"
#include "system/hexdump.h"
#include <pbl/logging/logging.h>
#include "pbl/util/string.h"

#include <bluetooth/bluetooth_types.h>
#include <bluetooth/sm_types.h>
#include <pbl/btutil/sm_util.h>

//
// Strictly for debug. Pretty-prints most of the pairing information saved
// in the gap bonding db and shared PRF.
//

void bluetooth_persistent_storage_debug_dump_ble_pairing_info(
    char *display_buf, const SMPairingInfo *info) {
  prompt_send_response(" Local Encryption Info: ");
  PBL_HEXDUMP_D_PROMPT(LOG_LEVEL_DEBUG,
                       (uint8_t *)&info->local_encryption_info,
                       sizeof(info->local_encryption_info));

  prompt_send_response(" Remote Encryption Info: ");
  PBL_HEXDUMP_D_PROMPT(LOG_LEVEL_DEBUG,
                       (uint8_t *)&info->remote_encryption_info,
                       sizeof(info->remote_encryption_info));

  prompt_send_response(" SMIdentityResolvingKey: ");
  PBL_HEXDUMP_D_PROMPT(LOG_LEVEL_DEBUG,
                       (uint8_t *)&info->irk,
                       sizeof(info->irk));

  prompt_send_response(" BTDeviceInternal: ");
  PBL_HEXDUMP_D_PROMPT(LOG_LEVEL_DEBUG,
                       (uint8_t *)&info->identity,
                       sizeof(BTDeviceInternal));

  prompt_send_response(" SMConnectionSignatureResolvingKey: ");
  PBL_HEXDUMP_D_PROMPT(LOG_LEVEL_DEBUG,
                       (uint8_t *)&info->csrk,
                       sizeof(SMConnectionSignatureResolvingKey));

  prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN,
                       " local encryption valid:  %s\n"
                       " remote encryption valid: %s\n"
                       " remote identity valid:   %s\n"
                       " remote signature valid:  %s\n",
                       bool_to_str(info->is_local_encryption_info_valid),
                       bool_to_str(info->is_remote_encryption_info_valid),
                       bool_to_str(info->is_remote_encryption_info_valid),
                       bool_to_str(info->is_remote_signing_info_valid));
}

void bluetooth_persistent_storage_debug_dump_classic_pairing_info(
    char *display_buf, BTDeviceAddress *addr, char *device_name, SM128BitKey *link_key,
    uint8_t platform_bits) {
  prompt_send_response(" Link Key:");
  PBL_HEXDUMP_D_PROMPT(LOG_LEVEL_DEBUG, (uint8_t *)link_key, sizeof(SM128BitKey));
  prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN, " BT ADDR: " BD_ADDR_FMT,
                       BT_DEVICE_ADDRESS_XPLODE(*addr));
  prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN, " Name: %s",
                       device_name);
  prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN, " Platform Bits: 0x%x",
                       (int)platform_bits);
}


void bluetooth_persistent_storage_debug_dump_root_keys(SM128BitKey *irk, SM128BitKey *erk) {
  prompt_send_response("Root Key hexdumps:");

  prompt_send_response(" IRK:");
  if (irk) {
    PBL_HEXDUMP_D_PROMPT(LOG_LEVEL_DEBUG, (uint8_t *)irk, sizeof(SM128BitKey));
  } else {
    prompt_send_response("  None");
  };

  prompt_send_response(" ERK:");
  if (erk) {
    PBL_HEXDUMP_D_PROMPT(LOG_LEVEL_DEBUG, (uint8_t *)erk, sizeof(SM128BitKey));
  } else {
    prompt_send_response("  None");
  };
}

extern void bluetooth_persistent_storage_dump_contents(void);
void command_gapdb_dump(void) {
#if !defined(CONFIG_RECOVERY_FW)
  bluetooth_persistent_storage_dump_contents();
#endif
  shared_prf_storage_dump_contents();
}
