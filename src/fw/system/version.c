/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/app_watch_info.h"
#include <pbl/drivers/flash.h>
#include "flash_region/flash_region.h"
#include "system/firmware_storage.h"
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/build_id.h"
#include "pbl/util/string.h"

#include <ctype.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "version.h"

#include "git_version.auto.h"

//! This symbol and its contents are provided by the linker script, see the
//! .note.gnu.build-id section in src/fw/fw_common.ld
extern const ElfExternalNote TINTIN_BUILD_ID;

const FirmwareMetadata TINTIN_METADATA SECTION(".pbl_fw_version") = {
  .version_timestamp = GIT_TIMESTAMP,
  .version_tag = GIT_TAG,
  .version_short = GIT_REVISION,

  .is_recovery_firmware = FIRMWARE_METADATA_IS_RECOVERY_FIRMWARE,
  .is_ble_firmware = false,
#ifdef CONFIG_PBLBOOT
  .is_dual_slot = true,
#else
  .is_dual_slot = false,
#endif
#if defined(CONFIG_PBLBOOT) && CONFIG_FIRMWARE_SLOT == 0 && !defined(CONFIG_RECOVERY_FW)
  .is_slot_0 = true,
#else
  .is_slot_0 = false,
#endif
  .reserved = 0,

  .hw_platform = FIRMWARE_METADATA_HW_PLATFORM,

  .metadata_version = FW_METADATA_CURRENT_STRUCT_VERSION,
};

bool version_copy_running_fw_metadata(FirmwareMetadata *out_metadata) {
  if (out_metadata == NULL) {
    return false;
  }
  memcpy(out_metadata, &TINTIN_METADATA, sizeof(FirmwareMetadata));
  return true;
}

static bool prv_version_copy_flash_fw_metadata(FirmwareMetadata *out_metadata,
                                               uint32_t flash_address, bool check_crc) {
#ifndef CONFIG_PBLBOOT
  FirmwareDescription firmware_description =
      firmware_storage_read_firmware_description(flash_address);

  if (check_crc &&
      !firmware_storage_check_valid_firmware_description(flash_address,
                                                         &firmware_description)) {
    *out_metadata = (FirmwareMetadata){};
    return false;
  }
  uint32_t fw_len = firmware_description.description_length + firmware_description.firmware_length;
#else
  FirmwareHeader header =
      firmware_storage_read_firmware_header(flash_address);
  if (check_crc &&
      !firmware_storage_check_valid_firmware_header(flash_address,
                                                    &header)) {
    *out_metadata = (FirmwareMetadata){};
    return false;
  }
  uint32_t fw_len = header.fw_length;
#endif
  // The FirmwareMetadata is stored at the end of the binary
  const uint32_t metadata_offset = flash_address + FIRMWARE_OFFSET + fw_len - sizeof(FirmwareMetadata);

  flash_read_bytes((uint8_t*)out_metadata, metadata_offset, sizeof(FirmwareMetadata));

  return true;
}

bool version_copy_recovery_fw_metadata(FirmwareMetadata *out_metadata) {
  const bool check_crc = true;
  return prv_version_copy_flash_fw_metadata(out_metadata, FLASH_REGION_SAFE_FIRMWARE_BEGIN,
                                            check_crc);
}

bool version_copy_update_fw_metadata(FirmwareMetadata *out_metadata) {
  const bool check_crc = false;
  uint32_t addr = FLASH_REGION_FIRMWARE_DEST_BEGIN;

  return prv_version_copy_flash_fw_metadata(out_metadata, addr, check_crc);
}

bool version_copy_recovery_fw_version(char* dest, const int dest_len_bytes) {
  FirmwareMetadata out_metadata;
  const bool check_crc = true;
  bool success = prv_version_copy_flash_fw_metadata(&out_metadata,
                                                    FLASH_REGION_SAFE_FIRMWARE_BEGIN,
                                                    check_crc);
  if (success) {
    strncpy(dest, out_metadata.version_tag, dest_len_bytes);
  }
  return success;
}

bool version_is_prf_installed(void) {
#ifndef CONFIG_PBLBOOT
  FirmwareDescription firmware_description =
      firmware_storage_read_firmware_description(FLASH_REGION_SAFE_FIRMWARE_BEGIN);

  return firmware_storage_check_valid_firmware_description(FLASH_REGION_SAFE_FIRMWARE_BEGIN,
                                                           &firmware_description);
#else
  FirmwareHeader header =
      firmware_storage_read_firmware_header(FLASH_REGION_SAFE_FIRMWARE_BEGIN);
  return firmware_storage_check_valid_firmware_header(FLASH_REGION_SAFE_FIRMWARE_BEGIN,
                                                      &header);
#endif
}

const uint8_t * version_get_build_id(size_t *out_len) {
  if (out_len) {
    *out_len = TINTIN_BUILD_ID.data_length;
  }
  PBL_ASSERTN(TINTIN_BUILD_ID.data_length == BUILD_ID_EXPECTED_LEN);

  return &TINTIN_BUILD_ID.data[TINTIN_BUILD_ID.name_length];
}

void version_copy_build_id_hex_string(char *buffer, size_t buffer_bytes_left,
                                      const ElfExternalNote *elf_build_id) {
  size_t build_id_bytes_left = elf_build_id->data_length;
  const uint8_t *build_id = &elf_build_id->data[elf_build_id->name_length];
  byte_stream_to_hex_string(buffer, buffer_bytes_left, build_id,
                            build_id_bytes_left, false);
}

void version_copy_current_build_id_hex_string(char *buffer, size_t buffer_bytes_left) {
  version_copy_build_id_hex_string(buffer, buffer_bytes_left, &TINTIN_BUILD_ID);
}

void version_get_major_minor_patch(unsigned int *major, unsigned int *minor,
                                   char const **patch_ptr) {
  *major = GIT_MAJOR_VERSION;
  *minor = GIT_MINOR_VERSION;
  *patch_ptr = GIT_PATCH_VERBOSE_STRING;
}

bool version_is_release_build(void) {
  const char *tag = GIT_TAG;

  if (*tag++ != 'v') {
    return false;
  }

  // Expect: digits, '.', digits, '.', digits, '\0'
  for (int i = 0; i < 3; i++) {
    if (!isdigit((unsigned char)*tag)) {
      return false;
    }
    while (isdigit((unsigned char)*tag)) {
      tag++;
    }
    if (i < 2) {
      if (*tag++ != '.') {
        return false;
      }
    }
  }

  return *tag == '\0';
}
