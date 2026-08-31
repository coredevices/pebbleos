# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_app_message
  SOURCES
    src/fw/applib/app_message/app_message.c
    src/fw/applib/app_message/app_message_outbox.c
    src/fw/applib/app_message/app_message_inbox.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    src/fw/process_management/pebble_process_info.c
    src/fw/util/dict.c
)

pbl_clar_test(test_app_inbox
  SOURCES
    src/fw/applib/app_inbox.c
    src/fw/services/app_inbox_service/service.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_app_outbox
  SOURCES
    src/fw/applib/app_outbox.c
    src/fw/services/app_outbox_service/service.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_cpu_cache
  SOURCES
    src/fw/applib/cpu_cache.c
    lib/os/mcu/cache_arm.c
  DEFINES UNITTEST_WITH_SYSCALL_PRIVILEGES=1
  OVERRIDES dummy_board
)

pbl_clar_test(test_persist
  SOURCES
    src/fw/flash_region/filesystem_regions.c
    src/fw/flash_region/flash_region.c
    src/fw/util/dict.c
    src/fw/applib/persist.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    src/fw/services/filesystem/app_file.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/persist/service.c
    src/fw/services/settings/settings_file.c
    src/fw/services/settings/settings_raw_iter.c
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_spi_flash.c
    tests/fixtures/resources/builtin_resources.auto.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_pbl_std
  SOURCES
    src/fw/flash_region/flash_region.c
    src/fw/applib/pbl_std/pbl_std.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_spi_flash.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_strftime
  SOURCES
    src/fw/flash_region/flash_region.c
    src/fw/applib/pbl_std/pbl_std.c
    src/fw/applib/pbl_std/strftime.c
    src/fw/applib/pbl_std/timelocal.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_spi_flash.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_text_render
  SOURCES
    src/fw/applib/graphics/gbitmap.c
    src/fw/applib/graphics/gcolor_definitions.c
    src/fw/applib/graphics/gtypes.c
    src/fw/applib/graphics/text_render.c
    src/fw/applib/fonts/codepoint.c
    tests/fakes/fake_gbitmap_png.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_text_resources
  SOURCES
    src/fw/flash_region/flash_region.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/system/hexdump.c
    src/fw/applib/fonts/codepoint.c
    src/fw/applib/fonts/fonts.c
    src/fw/applib/graphics/text_resources.c
    src/fw/resource/resource.c
    src/fw/resource/resource_storage.c
    src/fw/resource/resource_storage_builtin.c
    src/fw/resource/resource_storage_file.c
    src/fw/resource/resource_storage_flash.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/filesystem/app_file.c
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    src/fw/drivers/flash/flash_crc.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_spi_flash.c
    tests/fixtures/resources/builtin_resources.auto.c
    tests/fixtures/resources/pfs_resource_table.c
    tests/fw/applib/test_text_resources_font_stub.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_bitmap_layer
  SOURCES
    src/fw/applib/ui/bitmap_layer.c
    src/fw/applib/ui/layer.c
    tests/fakes/fake_gbitmap_png.c
    src/fw/applib/graphics/gcolor_definitions.c
    src/fw/applib/graphics/gtypes.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_unobstructed_area_service
  SOURCES
    src/fw/applib/graphics/gtypes.c
    src/fw/applib/ui/layer.c
    src/fw/applib/unobstructed_area_service.c
    tests/fakes/fake_events.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_touch_service
  SOURCES
    src/fw/applib/touch_service.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_app_glance
  SOURCES
    src/fw/applib/app_glance.c
    src/fw/applib/template_string.c
    src/fw/services/app_glances/app_glance_service.c
    src/fw/services/blob_db/app_glance_db.c
    src/fw/services/timeline/attribute.c
    src/fw/util/crc8.c
    src/fw/util/lru_cache.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_settings_file.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_template_string
  SOURCES
    src/fw/applib/template_string.c
  OVERRIDES dummy_board
)
