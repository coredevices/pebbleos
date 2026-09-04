/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "firmware_storage.h"

#include <pbl/drivers/flash.h>
#include "flash_region/flash_region.h"
#include <pbl/logging/logging.h>

#ifndef CONFIG_PBLBOOT
FirmwareDescription firmware_storage_read_firmware_description(uint32_t firmware_start_address) {
  FirmwareDescription firmware_description;
  flash_read_bytes((uint8_t*) &firmware_description, firmware_start_address,
                   sizeof(FirmwareDescription));


  return firmware_description;
}

bool firmware_storage_check_valid_firmware_description(
    uint32_t start_address, const FirmwareDescription *firmware_description) {

  if (firmware_description->description_length != sizeof(FirmwareDescription)) {
    // Corrupted description
    return false;
  }

  // Log around this operation, as it can take some time (hundreds of ms)
  PBL_LOG_DBG("CRCing recovery...");

  start_address += sizeof(FirmwareDescription);
  const uint32_t calculated_crc = flash_crc32(start_address, firmware_description->firmware_length);

  PBL_LOG_DBG("CRCing recovery... done");

  return calculated_crc == firmware_description->checksum;
}
#else
FirmwareHeader firmware_storage_read_firmware_header(uint32_t address) {
  FirmwareHeader header;
  flash_read_bytes((uint8_t*) &header, address, sizeof(FirmwareHeader));
  return header;
}

bool firmware_storage_check_valid_firmware_header(
    uint32_t address, const FirmwareHeader* header) {

  if (header->magic != FIRMWARE_HEADER_MAGIC ||
      header->header_length != sizeof(FirmwareHeader)) {
    // Corrupted header
    return false;
  }

  // Log around this operation, as it can take some time (hundreds of ms)
  PBL_LOG_DBG("CRCing recovery...");

  const uint32_t calculated_crc = flash_crc32(address + header->fw_start, header->fw_length);

  PBL_LOG_DBG("CRCing recovery... done");

  return calculated_crc == header->fw_crc;
}

void firmware_storage_invalidate_firmware_slot(uint8_t slot) {
  uint32_t slot_start;
  
  if (slot == 0U) {
    slot_start = FLASH_REGION_FIRMWARE_SLOT_0_BEGIN;
  } else {
    slot_start = FLASH_REGION_FIRMWARE_SLOT_1_BEGIN;
  }

  flash_region_erase_optimal_range(slot_start,
                                   slot_start,
                                   slot_start + SUBSECTOR_SIZE_BYTES,
                                   slot_start + SUBSECTOR_SIZE_BYTES);
}

#endif