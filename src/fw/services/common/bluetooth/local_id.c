/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "local_id.h"

#include "mfg/mfg_serials.h"
#include "services/common/bluetooth/bluetooth_persistent_storage.h"
#include "util/attributes.h"
#include "util/hash.h"
#include "util/size.h"
#include "util/string.h"

#include <bluetooth/features.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// Caches of the local address and device name.
// Some clients (i.e. Settings app) make a lot of calls to this module. By caching this info,
// we avoid having to reach out to the BT driver every time.
static BTDeviceAddress s_local_address;
static char s_local_device_name[BT_DEVICE_NAME_BUFFER_SIZE];
static char s_local_le_device_name[BT_DEVICE_NAME_BUFFER_SIZE];

static void prv_populate_name(char name[BT_DEVICE_NAME_BUFFER_SIZE], const char *name_fmt) {
  sprintf(name, name_fmt, s_local_address.octets[1], s_local_address.octets[0]);
}

static void prv_set_default_device_name(void) {
#if (PLATFORM_SNOWY || PLATFORM_SPALDING || PLATFORM_ROBERT)
  const char *s_local_default_device_name_format = "Pebble Time %02X%02X";
  const char *s_local_default_le_device_name_format = "Pebble Time LE %02X%02X";
#elif PLATFORM_ASTERIX || PLATFORM_OBELIX
  const char *s_local_default_device_name_format = "Core %02X%02X";
  const char *s_local_default_le_device_name_format = "Core %02X%02X";
#else
  const char *s_local_default_device_name_format = "Pebble %02X%02X";
  const char *s_local_default_le_device_name_format = "Pebble-LE %02X%02X";
#endif

  // Pebble + hex last 2 bytes of the device address:
  prv_populate_name(s_local_device_name, s_local_default_device_name_format);
  prv_populate_name(s_local_le_device_name, s_local_default_le_device_name_format);
}

static bool prv_has_device_name(void) {
  return (s_local_device_name[0] != '\0');
}

static void prv_configure_device_name(void) {
  bt_driver_id_set_local_device_name(s_local_device_name);
}

void bt_local_id_configure_driver(void) {
  // Request the local address from the BT driver and cache it:
  bt_driver_id_copy_local_identity_address(&s_local_address);

  if (!prv_has_device_name()) {
    if (!bt_persistent_storage_get_local_device_name(s_local_device_name,
                                                     sizeof(s_local_device_name))) {
      prv_set_default_device_name();
    }
  }

  prv_configure_device_name();
}

void bt_local_id_set_device_name(const char *device_name) {
  strncpy(s_local_device_name, device_name, sizeof(s_local_device_name));
  s_local_device_name[sizeof(s_local_device_name) - 1] = '\0';
  prv_configure_device_name();
}

void bt_local_id_copy_device_name(char name_out[BT_DEVICE_NAME_BUFFER_SIZE], bool is_le) {
  char *name = (is_le && bt_driver_supports_bt_classic()) ? s_local_le_device_name :
      s_local_device_name;
  strncpy(name_out, name, BT_DEVICE_NAME_BUFFER_SIZE);
}

void bt_local_id_copy_address(BTDeviceAddress *addr_out) {
  *addr_out = s_local_address;
}

void bt_local_id_copy_address_hex_string(char addr_hex_str_out[BT_ADDR_FMT_BUFFER_SIZE_BYTES]) {
  static const BTDeviceAddress null_addr = {};
  if (0 != memcmp(&null_addr, &s_local_address, sizeof(s_local_address))) {
    sniprintf(addr_hex_str_out, BT_DEVICE_ADDRESS_FMT_BUFFER_SIZE,
              BD_ADDR_FMT, BT_DEVICE_ADDRESS_XPLODE(s_local_address));
  } else {
    sniprintf(addr_hex_str_out, BT_DEVICE_ADDRESS_FMT_BUFFER_SIZE, "Unknown");
  }
}

void bt_local_id_copy_address_mac_string(char addr_mac_str_out[BT_DEVICE_ADDRESS_FMT_BUFFER_SIZE]) {
  sniprintf(addr_mac_str_out, BT_DEVICE_ADDRESS_FMT_BUFFER_SIZE,
            BT_DEVICE_ADDRESS_FMT, BT_DEVICE_ADDRESS_XPLODE(s_local_address));
}

T_STATIC void prv_generate_address(BTDeviceAddress *addr_out) {
  const char *serial = mfg_get_serial_number();
  const uint32_t full_len = strlen(serial);

  // Hash of the normal serial
  const uint32_t serial_hash = hash((uint8_t *)serial, full_len);

  // Hash of the serial reversed
  char tmp[full_len + 1];
  strncpy(tmp, serial, sizeof(tmp));
  string_reverse(tmp);
  const uint32_t reverse_hash = hash((uint8_t *)tmp, full_len);

  struct PACKED {
    union {
      BTDeviceAddress bt_addr;
      struct PACKED {
        uint16_t a;
        uint32_t b;
      };
    };
  } addr = {
    .a = (uint16_t) reverse_hash,
    .b = (serial_hash ^ reverse_hash),
  };

  *addr_out = addr.bt_addr;
}

void bt_local_id_generate_address_from_serial(BTDeviceAddress *addr_out) {
  prv_generate_address(addr_out);
  addr_out->octets[ARRAY_LENGTH(addr_out->octets) - 1] |= 0b11000000;

  // Addresses with all 0's or 1's
  const BTDeviceAddress zero_addr     = {.octets = {0x00, 0x00, 0x00, 0x00, 0x00, 0xC0}};
  const BTDeviceAddress one_addr      = {.octets = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}};
  // NOTE: It already has the two most sig. bits set.
  const BTDeviceAddress fallback_addr = {.octets = {0x3c, 0x08, 0x55, 0xaf, 0xd3, 0xc4}};

  // Compare (the first 5 bytes) the generated one with the invalid ones. If they are equal,
  // fall back to this address.
  if (!memcmp(addr_out, &zero_addr, sizeof(BTDeviceAddress))
      || !memcmp(addr_out, &one_addr, sizeof(BTDeviceAddress))) {
    *addr_out = fallback_addr;
  }

  /* memcpy(addr_out, (uint8_t *)addr_out, sizeof(*addr_out)); */
}
