/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

/*
 * firmware_metadata.h
 *
 * This file specifies the Firmware Metadata structure used in the .elf & .bin files to
 * identify the build info, etc.
 */

#include "pbl/util/attributes.h"

#include <stdint.h>
#include <stdbool.h>


#define FW_METADATA_CURRENT_STRUCT_VERSION 0x1
#define FW_METADATA_VERSION_SHORT_BYTES 8
#define FW_METADATA_VERSION_TAG_BYTES 32

// NOTE: When adding new platforms, if they use the legacy defective CRC, the list in
// tools/fw_binary_info.py needs to be updated with the platform value.
typedef enum FirmwareMetadataPlatform {
  FirmwareMetadataPlatformUnknown = 0,
  FirmwareMetadataPlatformPebbleOneEV1 = 1,
  FirmwareMetadataPlatformPebbleOneEV2 = 2,
  FirmwareMetadataPlatformPebbleOneEV2_3 = 3,
  FirmwareMetadataPlatformPebbleOneEV2_4 = 4,
  FirmwareMetadataPlatformPebbleOnePointFive = 5,
  FirmwareMetadataPlatformPebbleTwoPointZero = 6,
  FirmwareMetadataPlatformPebbleSnowyEVT2 = 7,
  FirmwareMetadataPlatformPebbleSnowyDVT = 8,
  FirmwareMetadataPlatformPebbleSpaldingEVT = 9,
  FirmwareMetadataPlatformPebbleBobbyDVT = 10,
  FirmwareMetadataPlatformPebbleSpalding = 11,
  FirmwareMetadataPlatformPebbleSilkEVT = 12,
  FirmwareMetadataPlatformPebbleRobertEVT = 13,
  FirmwareMetadataPlatformPebbleSilk = 14,
  FirmwareMetadataPlatformPebbleAsterix = 15,
  FirmwareMetadataPlatformPebbleObelixEVT = 16,
  FirmwareMetadataPlatformPebbleObelixDVT = 17,
  FirmwareMetadataPlatformPebbleObelixPVT = 18,
  FirmwareMetadataPlatformPebbleGetafixEVT = 19,
  FirmwareMetadataPlatformPebbleGetafixDVT = 20,
  FirmwareMetadataPlatformPebbleGetafixDVT2 = 21,

  FirmwareMetadataPlatformPebbleOneBigboard = 0xff,
  FirmwareMetadataPlatformPebbleOneBigboard2 = 0xfe,
  FirmwareMetadataPlatformPebbleSnowyBigboard = 0xfd,
  FirmwareMetadataPlatformPebbleSnowyBigboard2 = 0xfc,
  FirmwareMetadataPlatformPebbleSpaldingBigboard = 0xfb,
  FirmwareMetadataPlatformPebbleSilkBigboard = 0xfa,
  FirmwareMetadataPlatformPebbleRobertBigboard = 0xf9,
  FirmwareMetadataPlatformPebbleSilkBigboard2 = 0xf8,
  FirmwareMetadataPlatformPebbleRobertBigboard2 = 0xf7,
  FirmwareMetadataPlatformPebbleFlintEmu = 0xf6,
  FirmwareMetadataPlatformPebbleEmeryEmu = 0xf5,
  FirmwareMetadataPlatformPebbleObelixBigboard = 0xf4,
  FirmwareMetadataPlatformPebbleObelixBigboard2 = 0xf3,
  FirmwareMetadataPlatformPebbleGabbroEmu = 0xf2,
} FirmwareMetadataPlatform;

// WARNING: changes in this struct must be reflected in:
// - iOS/PebblePrivateKit/PebblePrivateKit/PBBundle.m

struct PACKED FirmwareMetadata {
  uint32_t version_timestamp;
  char version_tag[FW_METADATA_VERSION_TAG_BYTES];
  char version_short[FW_METADATA_VERSION_SHORT_BYTES];
  bool is_recovery_firmware:1;
  bool is_ble_firmware:1;
  bool is_dual_slot:1;
  bool is_slot_0:1;
  uint8_t reserved:4;
  uint8_t hw_platform;
  //! This should be the last field, since we put the meta data struct at the end of the fw binary.
  uint8_t metadata_version;
};
typedef struct FirmwareMetadata FirmwareMetadata;

_Static_assert(sizeof(struct FirmwareMetadata) == (sizeof(uint32_t) +
               FW_METADATA_VERSION_SHORT_BYTES + FW_METADATA_VERSION_TAG_BYTES + sizeof(uint8_t) +
               sizeof(uint8_t) + sizeof(uint8_t)),
               "FirmwareMetadata bitfields not packed correctly");


// Shared defines. Let's not duplicate this everywhere.

#ifdef CONFIG_RECOVERY_FW
  #define FIRMWARE_METADATA_IS_RECOVERY_FIRMWARE (true)
#else
  #define FIRMWARE_METADATA_IS_RECOVERY_FIRMWARE (false)
#endif

#ifdef CONFIG_BOARD_ASTERIX
  #define FIRMWARE_METADATA_HW_PLATFORM (FirmwareMetadataPlatformPebbleAsterix)
#elif defined(CONFIG_BOARD_OBELIX_DVT)
  #define FIRMWARE_METADATA_HW_PLATFORM (FirmwareMetadataPlatformPebbleObelixDVT)
#elif defined(CONFIG_BOARD_OBELIX_PVT)
  #define FIRMWARE_METADATA_HW_PLATFORM (FirmwareMetadataPlatformPebbleObelixPVT)
#elif defined(CONFIG_BOARD_OBELIX_BB2)
  #define FIRMWARE_METADATA_HW_PLATFORM (FirmwareMetadataPlatformPebbleObelixBigboard2)
#elif defined(CONFIG_BOARD_GETAFIX_DVT)
  #define FIRMWARE_METADATA_HW_PLATFORM (FirmwareMetadataPlatformPebbleGetafixDVT)
#elif defined(CONFIG_BOARD_GETAFIX_DVT2)
  #define FIRMWARE_METADATA_HW_PLATFORM (FirmwareMetadataPlatformPebbleGetafixDVT2)
#elif defined(CONFIG_BOARD_QEMU_EMERY)
  #define FIRMWARE_METADATA_HW_PLATFORM (FirmwareMetadataPlatformPebbleEmeryEmu)
#elif defined(CONFIG_BOARD_QEMU_FLINT)
  #define FIRMWARE_METADATA_HW_PLATFORM (FirmwareMetadataPlatformPebbleFlintEmu)
#elif defined(CONFIG_BOARD_QEMU_GABBRO)
  #define FIRMWARE_METADATA_HW_PLATFORM (FirmwareMetadataPlatformPebbleGabbroEmu)
#else
  #define FIRMWARE_METADATA_HW_PLATFORM (FirmwareMetadataPlatformUnknown)
#endif
