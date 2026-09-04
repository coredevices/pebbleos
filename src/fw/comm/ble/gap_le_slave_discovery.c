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
#include <pbl/util/attributes.h>

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
//! Schedules the discovery advertisement job.
//! We don't want to be advertising at a high rate infinitely. When duration
//! is 0, a short period of high-rate advertising will be used. When this short
//! period is completed, an indefinite, low-rate job will be scheduled.
static void prv_schedule_ad_job(void) {
  BLEAdData *ad = ble_ad_create();

  // Advertisement part:
  // Centrals will be filtering on Service UUID first. Assuming that the
  // central is only doing a scan request if the Service UUID matches with their
  // interests, to save radio time / battery life we keep the advertisement part
  // as "small" as possible (21 bytes currently).
  // Advertise "BR/EDR Not Supported" alongside General Discoverable: these are
  // BLE-only watches, so dual-mode hosts must connect over LE instead of attempting
  // a classic page (which would time out).
  ble_ad_set_flags(ad, GAP_LE_AD_FLAGS_GEN_DISCOVERABLE_MASK |
                       GAP_LE_AD_FLAGS_BR_EDR_NOT_SUPPORTED_MASK);

  // *DO NOT* use pebble_bt_uuid_expand() here!
  // ble_ad_set_service_uuids() will be "smart" and include only the 16-bit UUID, but only if the
  // BT SIG Base UUID is used.
  Uuid service_uuids[2];
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

  ble_ad_set_service_uuids(ad, service_uuids, num_uuids);

  char device_name[BT_DEVICE_NAME_BUFFER_SIZE];
  bt_local_id_copy_device_name(device_name, true);
  ble_ad_set_local_name(ad, device_name);
  ble_ad_set_tx_power_level(ad);

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

  ble_ad_set_manufacturer_specific_data(ad,
                                       BT_VENDOR_ID,
                                       (const uint8_t *) &mfg_data,
                                       sizeof(struct ManufacturerSpecificData));

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
