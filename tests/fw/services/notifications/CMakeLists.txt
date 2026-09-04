# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_notification_storage
  SOURCES
    src/fw/services/notifications/notification_storage.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    src/fw/util/stringlist.c
    src/fw/util/time/time.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/timeline/item.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/attribute_group.c
    src/fw/services/timeline/attribute.c
    src/fw/flash_region/flash_region.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_spi_flash.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_ancs_notifications
  SOURCES
    src/fw/services/notifications/ancs/ancs_notifications.c
    src/fw/services/notifications/ancs/ancs_notifications_util.c
    src/fw/services/notifications/ancs/ancs_item.c
    src/fw/services/notifications/ancs/ancs_phone_call.c
    src/fw/services/timeline/attribute.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/attribute_group.c
    src/fw/services/timeline/item.c
    src/fw/services/phone_call/util.c
    src/fw/applib/fonts/codepoint.c
    src/fw/applib/graphics/utf8.c
    src/fw/util/pstring.c
    src/fw/util/time/time.c
    tests/fakes/fake_events.c
)

pbl_clar_test(test_ancs_notifications_util
  SOURCES
    src/fw/services/notifications/ancs/ancs_notifications_util.c
    src/fw/util/time/mktime.c
    src/fw/util/pstring.c
    src/fw/util/time/time.c
    tests/fakes/fake_rtc.c
)

pbl_clar_test(test_ancs_filtering
  SOURCES
    src/fw/services/notifications/ancs/ancs_filtering.c
    src/fw/services/timeline/attribute.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/attribute_group.c
    src/fw/util/pstring.c
    src/fw/util/time/time.c
)

pbl_clar_test(test_nexmo
  SOURCES
    src/fw/services/notifications/ancs/nexmo.c
    src/fw/services/notifications/ancs/ancs_notifications_util.c
    src/fw/services/timeline/attribute.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/attribute_group.c
    src/fw/util/pstring.c
    src/fw/util/time/time.c
    tests/fakes/fake_rtc.c
)
