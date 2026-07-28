/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "board/board.h"

#include "gap_le_slave_discovery.h"
#include "gap_le_advert.h"

#include "applib/bluetooth/ble_ad_parse.h"

#include "comm/bt_lock.h"

#include "git_version.auto.h"

#include "mfg/mfg_info.h"

#include "mfg/mfg_serials.h"

#include "pbl/services/bluetooth/local_id.h"
#include "pbl/services/bluetooth/ble_hrm.h"

#include "system/passert.h"
#include "system/version.h"

#include <bluetooth/pebble_bt.h>
#include <bluetooth/pebble_pairing_service.h>
#include <bluetooth/bluetooth_types.h>
#include <pbl/btutil/bt_uuid.h>
#include <pbl/logging/logging.h>
#include <pbl/util/attributes.h>
#include <pbl/util/size.h>

PBL_LOG_MODULE_DECLARE(bt, CONFIG_BT_LOG_LEVEL);

static GAPLEAdvertisingJobRef s_discovery_advert_job;

// -----------------------------------------------------------------------------
//! Handles unscheduling of the discovery advertisement job.
static void prv_job_unschedule_callback(GAPLEAdvertisingJobRef job,
                                        bool completed,
                                        void *cb_data) {
  // Cleanup:
  s_discovery_advert_job = NULL;
}

// -----------------------------------------------------------------------------
//! Builds the advertisement + scan response payload.
//! @return False if the manufacturer-specific data did not fit, which is the
//! one element the mobile app cannot do without.
static bool prv_build_ad_payload(BLEAdData *ad, bool include_hid_uuid) {
  // Advertisement part:
  // Centrals will be filtering on Service UUID first. Assuming that the
  // central is only doing a scan request if the Service UUID matches with their
  // interests, to save radio time / battery life we keep the advertisement part
  // as "small" as possible. With the default 11-character device name it is 25
  // bytes of the 31 available, or 27 with the HRM service also advertised.
  // A name longer than 17 characters (15 with HRM also advertised) overflows
  // the 31-byte advertisement, and elements that no longer fit spill into the
  // scan response, where they compete with the manufacturer-specific data.
  // These figures include the HID Service UUID; without CONFIG_BT_HID_REMOTE
  // everything is 2 bytes smaller and the name limits 2 characters longer.
  // Advertise "BR/EDR Not Supported" alongside General Discoverable: these are
  // BLE-only watches, so dual-mode hosts must connect over LE instead of attempting
  // a classic page (which would time out).
  if (!ble_ad_set_flags(ad, GAP_LE_AD_FLAGS_GEN_DISCOVERABLE_MASK |
                            GAP_LE_AD_FLAGS_BR_EDR_NOT_SUPPORTED_MASK)) {
    PBL_LOG_WRN("AD flags did not fit the advertising payload");
  }

  // *DO NOT* use pebble_bt_uuid_expand() here!
  // ble_ad_set_service_uuids() will be "smart" and include only the 16-bit UUID, but only if the
  // BT SIG Base UUID is used.
  Uuid service_uuids[3];
  size_t num_uuids = 0;

#if defined(CONFIG_HRM) && !defined(CONFIG_RECOVERY_FW)
  // NOTE: The HRM service has to be first in the list because otherwise the Pebble won't
  // show up as an HRM device in Strava for Android...
  if (ble_hrm_is_supported_and_enabled()) {
    service_uuids[num_uuids++] = bt_uuid_expand_16bit(0x180D);  // Heart Rate Service
  }
#endif

  // Pebble Pairing Service UUID:
  service_uuids[num_uuids++] = bt_uuid_expand_16bit(PEBBLE_BT_PAIRING_SERVICE_UUID_16BIT);

#ifdef CONFIG_BT_HID_REMOTE
  // Human Interface Device Service, so hosts see the watch as an input device.
  if (include_hid_uuid) {
    service_uuids[num_uuids++] = bt_uuid_expand_16bit(0x1812);
  }
#else
  (void)include_hid_uuid;
#endif

  if (!ble_ad_set_service_uuids(ad, service_uuids, num_uuids)) {
    PBL_LOG_WRN("Service UUIDs did not fit the advertising payload");
  }

  char device_name[BT_DEVICE_NAME_BUFFER_SIZE];
  bt_local_id_copy_device_name(device_name, true);
  if (!ble_ad_set_local_name(ad, device_name)) {
    PBL_LOG_WRN("Device name did not fit the advertising payload");
  }
  if (!ble_ad_set_tx_power_level(ad)) {
    PBL_LOG_WRN("TX power level did not fit the advertising payload");
  }

  // Scan response part:
  ble_ad_start_scan_response(ad);

  // Add serial number in a Manufacturer Specific AD Type:
  struct PACKED ManufacturerSpecificData {
    uint8_t payload_type;
    char serial_number[MFG_SERIAL_NUMBER_SIZE];
    uint8_t hw_platform;
    uint8_t color;
    struct {
      uint8_t major;
      uint8_t minor;
      uint8_t patch;
    } fw_version;
    union {
      uint8_t flags;
      struct {
        bool is_running_recovery_firmware:1;
        bool is_first_use:1;
      };
    };
  } mfg_data = {
    .payload_type = 0 /* For future proofing. Only one type for now.*/,
    .hw_platform = TINTIN_METADATA.hw_platform,
    .color = mfg_info_get_watch_color(),
    .fw_version = {
      .major = GIT_MAJOR_VERSION,
      .minor = GIT_MINOR_VERSION,
      .patch = GIT_PATCH_VERSION,
    },
    .is_running_recovery_firmware = TINTIN_METADATA.is_recovery_firmware,
    .is_first_use = false, // !getting_started_is_complete(), // TODO
  };
  memcpy(&mfg_data.serial_number,
         mfg_get_serial_number(),
         MFG_SERIAL_NUMBER_SIZE);

  return ble_ad_set_manufacturer_specific_data(ad,
                                               BT_VENDOR_ID,
                                               (const uint8_t *) &mfg_data,
                                               sizeof(struct ManufacturerSpecificData));
}

// -----------------------------------------------------------------------------
//! Schedules the discovery advertisement job.
//! We don't want to be advertising at a high rate infinitely. When duration
//! is 0, a short period of high-rate advertising will be used. When this short
//! period is completed, an indefinite, low-rate job will be scheduled.
static void prv_schedule_ad_job(void) {
  BLEAdData *ad = ble_ad_create();
  if (!ad) {
    PBL_LOG_ERR("Out of memory for the advertising payload");
    return;
  }

  if (!prv_build_ad_payload(ad, true /* include_hid_uuid */)) {
    PBL_LOG_WRN("Manufacturer specific data did not fit the advertising payload");
#ifdef CONFIG_BT_HID_REMOTE
    // That data is what the mobile app identifies the watch by, so drop the HID
    // UUID and rebuild: it is only a discovery-time hint, an already-bonded
    // phone finds the HID service over GATT regardless. ble_ad_create() starts
    // from an all-zero header, so this resets the payload the same way.
    PBL_LOG_WRN("Rebuilding it without the HID service UUID");
    memset(ad, 0, sizeof(BLEAdData));
    if (!prv_build_ad_payload(ad, false /* include_hid_uuid */)) {
      // Unreachable today: the longest name BT_DEVICE_NAME_BUFFER_SIZE allows
      // always frees enough room once the HID UUID is gone. Should that stop
      // holding, this ships a payload strictly worse than the first attempt.
      PBL_LOG_WRN("Manufacturer specific data still does not fit; dropped");
    }
#endif
  }

  // Values chosen according to Apple Accessory Design Guidelines.
  const GAPLEAdvertisingJobTerm advert_terms[] = {
      {
          // Extend this term from recommended 30s to 5min so user has e.g. time
          // to download or open mobile app.
          .duration_secs = 5 * 60,
          .interval = GAPLEAdvertisingInterval_Short,
      },
      {
          .duration_secs = GAPLE_ADVERTISING_DURATION_INFINITE,
          .interval = GAPLEAdvertisingInterval_Long,
      },
  };

  s_discovery_advert_job = gap_le_advert_schedule(
      ad, advert_terms, sizeof(advert_terms) / sizeof(GAPLEAdvertisingJobTerm),
      prv_job_unschedule_callback, NULL, GAPLEAdvertisingJobTagDiscovery);

  ble_ad_destroy(ad);
}

// -----------------------------------------------------------------------------
bool gap_le_slave_is_discoverable(void) {
  bool is_discoverable = false;
  bt_lock();
  {
    is_discoverable = (s_discovery_advert_job != NULL);
  }
  bt_unlock();
  return is_discoverable;
}

// -----------------------------------------------------------------------------
void gap_le_slave_set_discoverable(bool discoverable) {
  bt_lock();
  {
    // Always stop and re-start, so we start with the high rate again:
    gap_le_advert_unschedule(s_discovery_advert_job);
    if (discoverable) {
      prv_schedule_ad_job();
    }
  }
  bt_unlock();
}

// -----------------------------------------------------------------------------
void gap_le_slave_discovery_init(void) {
  bt_lock();
  {
    PBL_ASSERTN(!s_discovery_advert_job);
  }
  bt_unlock();
}

// -----------------------------------------------------------------------------
void gap_le_slave_discovery_deinit(void) {
  bt_lock();
  {
    gap_le_advert_unschedule(s_discovery_advert_job);
  }
  bt_unlock();
}
