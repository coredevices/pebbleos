# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_ancs_pebble_actions
  SOURCES
    src/fw/comm/ble/kernel_le_client/ancs/ancs_util.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/flash_region/flash_region.c
    src/fw/services/blob_db/ios_notif_pref_db.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/notifications/ancs/ancs_filtering.c
    src/fw/services/notifications/ancs/ancs_item.c
    src/fw/services/notifications/ancs/ancs_notifications.c
    src/fw/services/notifications/ancs/ancs_notifications_util.c
    src/fw/services/notifications/ancs/ancs_phone_call.c
    src/fw/services/settings/settings_file.c
    src/fw/services/settings/settings_raw_iter.c
    src/fw/services/timeline/actions_endpoint.c
    src/fw/services/timeline/attribute.c
    src/fw/services/timeline/attribute_group.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/item.c
    src/fw/services/timeline/timeline.c
    src/fw/services/timeline/timeline_actions.c
    src/fw/util/buffer.c
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    src/fw/util/pstring.c
    src/fw/util/rand/rand.c
    src/fw/util/time/time.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    tests/fakes/fake_spi_flash.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_timeline_actions
  SOURCES
    src/fw/services/timeline/actions_endpoint.c
    src/fw/services/timeline/attribute.c
    src/fw/services/timeline/attribute_group.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/item.c
    src/fw/services/timeline/timeline.c
    src/fw/services/timeline/timeline_actions.c
    src/fw/util/time/time.c
  OVERRIDES dummy_board
)
