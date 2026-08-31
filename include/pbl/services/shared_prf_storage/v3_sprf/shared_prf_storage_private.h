/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <bluetooth/bluetooth_types.h>
#include <pbl/util/attributes.h>

#define SPRF_PAGE_IDX_INVALID ((uint16_t)~0)

#define SPRF_MAX_NUM_PAGES_MULT(num) ((num) * 3 / 4)

typedef enum {
  SprfValidFields_LocalEncryptionInfoValid  = (1 << 0),
  SprfValidFields_RemoteEncryptionInfoValid = (1 << 1),
  SprfValidFields_RemoteIdentityInfoValid   = (1 << 2),
  SprfValidFields_RemoteSigningInfoValid    = (1 << 3),
} SprfValidFields;

#ifndef __clang__
_Static_assert(sizeof(SprfValidFields) == 1, "SprfValidFields unexpected size");
#endif


typedef enum {
  SprfMagic_ValidEntry = 0x46525053,
  SprfMagic_UnpopulatedEntry = 0xFFFFFFFF,
  SprfMagic_InvalidatedEntry = 0x0
} SprfMagic;

_Static_assert(sizeof(SprfMagic) == 4, "SprfMagic unexpected size");

//! This is the struct written out to the Shared PRF flash region
//!
//! It's composed of seven sub entries:
//!   root_keys: Root keys (only identity since that is all Dialog needs)
//!   ble_pairing_data: The pairing info for the device most recently paired to the watch
//!   ble_pairing_name: The name of the device most recently paired to the watch
//!   pinned_address: Pinned address of the device most recently paired to watch (may be empty)
//!   getting_started: Captures whether or not we have gone through onboarding
//!   local_name: Not used yet, but saved for future proofing
//!   main_fw_scratch: A region for normal fw to stash info in the future if needed
//!
//! Each entry, or field, has its own crc which is written once the write of the field is complete.
//! @note The CRC _must_ be the first member of a field. There are static asserts to catch
//! this for current, please add a static assert for this if you create a new field
//!
//! A field is 'valid' iff a CRC of its contents matches the crc in flash
//! A field is 'unpopulated' if a memcmp of its contents is all 0xff.
//! A field is 'deleted' if its header has the value of SprfMagic_InvalidatedEntry
//! A field is 'corrupted' or 'partially written' if the content CRC does not match the field CRC
//!
//! On flash, there is a rolling list of entries. If a field above needs to be rewritten,
//! 'valid' entries must be copied from the current flash area to the next adjacent one.
//!
//! The shared PRF struct itself is defined as 256 bytes. Flash architectures have sectors
//! which are some 2^n multiple so this size pretty much guarantees that a divisible number
//! of structs can fit in the region allocated

typedef struct PACKED SprfRootKeys {
  uint32_t crc;
  SM128BitKey keys[SMRootKeyTypeNum];
} SprfRootKeys;
_Static_assert(offsetof(SprfRootKeys, crc) == 0, "crc must be the first field");

typedef struct PACKED SprfBlePairingData {
  uint32_t crc; // CRC over the 'pairing_data' struct ('name' through 'fields')

  // local encryption data
  SMLongTermKey l_ltk; // 16 byte key
  uint64_t l_rand;
  uint16_t l_ediv;

  // remote encryption data
  uint16_t r_ediv;
  SMLongTermKey r_ltk;
  uint64_t r_rand;

  SMIdentityResolvingKey irk; // 16 byte key
  SMConnectionSignatureResolvingKey csrk; // 16 byte key
  BTDeviceInternal identity;

  SprfValidFields fields:8;
  bool is_mitm_protection_enabled;
  bool requires_address_pinning;

  //! Added in SPRF_CUR_VERSION 2. In SPRF_CUR_VERSION 1, this field is always 0x00.
  uint8_t flags;
} SprfBlePairingData;
_Static_assert(offsetof(SprfBlePairingData, crc) == 0, "crc must be the first field");

typedef struct PACKED SprfBlePairingName {
  uint32_t crc;
  char name[BT_DEVICE_NAME_BUFFER_SIZE];
} SprfBlePairingName;
_Static_assert(offsetof(SprfBlePairingName, crc) == 0, "crc must be the first field");

typedef struct PACKED SprfPinnedAddress {
  uint32_t crc;
  BTDeviceAddress pinned_address;
  uint8_t rsvd[2];
} SprfPinnedAddress;
_Static_assert(offsetof(SprfPinnedAddress, crc) == 0, "crc must be the first field");

typedef struct PACKED SprfGettingStarted {
  uint32_t crc;
  bool is_complete;
  uint8_t rsvd[3];
} SprfGettingStarted;
_Static_assert(offsetof(SprfGettingStarted, crc) == 0, "crc must be the first field");

typedef struct PACKED SprfLocalName {
  // Not used today, but in the future we could replace 'Pebble XXXX' with
  // a user friendly name, 'Chris' Pebble'
  uint32_t crc;
  char name[BT_DEVICE_NAME_BUFFER_SIZE];
} SprfLocalName;
_Static_assert(offsetof(SprfLocalName, crc) == 0, "crc must be the first field");

typedef struct PACKED SharedPRFData {
  SprfMagic magic;
  uint8_t version;
  uint8_t rsvd[3];

  SprfRootKeys root_keys;
  SprfBlePairingData ble_pairing_data;
  SprfBlePairingName ble_pairing_name;
  SprfPinnedAddress pinned_address;
  SprfGettingStarted getting_started;
  SprfLocalName local_name;

  // Occasions have arisen in the past where a region in sharedPRF that
  // main FW can stash info related to a pairing. That is the intent of this region.
  struct PACKED {
    uint8_t rsvd[44];
  } main_fw_scratch;
} SharedPRFData;

_Static_assert(BT_DEVICE_NAME_BUFFER_SIZE == 20, "Changing the length will break SharedPRF");
_Static_assert(sizeof(SharedPRFData) == 256, "SharedPRFData does not match expected size");
