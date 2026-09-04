# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_timeline
  SOURCES
    src/fw/services/blob_db/pin_db.c
    src/fw/services/blob_db/sync_util.c
    src/fw/services/blob_db/timeline_item_storage.c
    src/fw/services/timeline/attribute.c
    src/fw/services/timeline/attribute_group.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/item.c
    src/fw/services/timeline/timeline.c
    src/fw/util/crc8.c
    src/fw/util/time/mktime.c
    src/fw/util/time/time.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_settings_file.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_pin_db
  SOURCES
    src/fw/services/blob_db/pin_db.c
    src/fw/services/blob_db/sync_util.c
    src/fw/services/blob_db/timeline_item_storage.c
    src/fw/services/blob_db/util.c
    src/fw/services/timeline/attribute.c
    src/fw/services/timeline/attribute_group.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/item.c
    src/fw/util/crc8.c
    src/fw/util/time/mktime.c
    src/fw/util/time/time.c
    tests/fakes/fake_blobdb.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_session.c
    tests/fakes/fake_settings_file.c
    tests/fakes/ram_storage.c
    tests/fakes/test_db.c
)

pbl_clar_test(test_blob_db_sync
  SOURCES
    src/fw/services/blob_db/util.c
    src/fw/services/blob_db/sync.c
    tests/fakes/ram_storage.c
    tests/fakes/test_db.c
    tests/fakes/fake_blobdb.c
)

pbl_clar_test(test_blob_db_endpoint
  SOURCES
    src/fw/services/blob_db/api.c
    src/fw/services/blob_db/endpoint.c
    src/fw/services/blob_db/endpoint_private.c
    src/fw/services/blob_db/util.c
    tests/fakes/fake_session.c
)

pbl_clar_test(test_blob_db2_endpoint
  SOURCES
    src/fw/services/blob_db/api.c
    src/fw/services/blob_db/endpoint2.c
    src/fw/services/blob_db/endpoint_private.c
    src/fw/services/blob_db/util.c
)

pbl_clar_test(test_app_db
  SOURCES
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    tests/fakes/fake_spi_flash.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    tests/fakes/fake_rtc.c
    src/fw/flash_region/flash_region.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/services/settings/settings_file.c
    src/fw/services/settings/settings_raw_iter.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/blob_db/app_db.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_app_glance_db
  SOURCES
    src/fw/services/blob_db/app_glance_db.c
    src/fw/services/timeline/attribute.c
    src/fw/util/crc8.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_settings_file.c
    tests/fakes/fake_events.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_contacts_db
  SOURCES
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    tests/fakes/fake_spi_flash.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    tests/fakes/fake_rtc.c
    src/fw/flash_region/flash_region.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/services/settings/settings_file.c
    src/fw/services/settings/settings_raw_iter.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/blob_db/contacts_db.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_watch_app_prefs_db
  SOURCES
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    tests/fakes/fake_spi_flash.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    tests/fakes/fake_rtc.c
    src/fw/flash_region/flash_region.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/services/settings/settings_file.c
    src/fw/services/settings/settings_raw_iter.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/blob_db/watch_app_prefs_db.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_reminder_db
  SOURCES
    src/fw/util/crc8.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    tests/fakes/fake_rtc.c
    src/fw/util/time/time.c
    src/fw/services/blob_db/timeline_item_storage.c
    tests/fakes/fake_settings_file.c
    src/fw/services/blob_db/reminder_db.c
    src/fw/services/blob_db/sync_util.c
    src/fw/services/timeline/attribute.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/attribute_group.c
    src/fw/services/timeline/item.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_notif_db
  SOURCES
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    tests/fakes/fake_spi_flash.c
    tests/fakes/fake_kernel_services_notifications.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    tests/fakes/fake_rtc.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/flash_region/flash_region.c
    src/fw/util/time/time.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/notifications/notification_storage.c
    src/fw/services/timeline/attribute.c
    src/fw/services/timeline/item.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/attribute_group.c
    src/fw/services/blob_db/notif_db.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_weather_db
  SOURCES
    src/fw/util/pstring.c
    src/fw/util/time/time.c
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_spi_flash.c
    src/fw/flash_region/flash_region.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/services/settings/settings_file.c
    src/fw/services/settings/settings_raw_iter.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/blob_db/weather_db.c
    tests/fw/services/blob_db/weather_data_shared.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_prefs_db
  SOURCES
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    src/fw/util/time/time.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/flash_region/flash_region.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/settings/settings_file.c
    src/fw/services/settings/settings_raw_iter.c
    src/fw/services/blob_db/prefs_db.c
    src/fw/shell/normal/prefs.c
    tests/stubs/stubs_i18n.c
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    tests/fakes/fake_accel_service.c
    tests/fakes/fake_spi_flash.c
    tests/fakes/fake_rtc.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_ios_notif_pref_db
  SOURCES
    src/fw/util/crc8.c
    lib/util/uuid.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_settings_file.c
    src/fw/util/time/time.c
    src/fw/services/timeline/attribute.c
    src/fw/services/timeline/item.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/attribute_group.c
    src/fw/services/blob_db/sync_util.c
    src/fw/services/blob_db/ios_notif_pref_db.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_health_db
  SOURCES
    src/fw/util/crc8.c
    src/fw/util/time/time.c
    tests/fakes/fake_settings_file.c
    src/fw/services/blob_db/health_db.c
  OVERRIDES dummy_board
)
