# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_clar
  SOURCES
    src/fw/applib/graphics/gtypes.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
)

pbl_clar_test(test_utf8
  SOURCES
    src/fw/applib/graphics/utf8.c
    tests/fakes/fake_gbitmap_png.c
)

pbl_clar_test(test_i18n
  SOURCES
    src/fw/services/i18n/i18n.c
    src/fw/flash_region/flash_region.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/resource/resource.c
    src/fw/resource/resource_storage.c
    src/fw/resource/resource_storage_builtin.c
    src/fw/resource/resource_storage_file.c
    src/fw/resource/resource_storage_flash.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/filesystem/app_file.c
    tests/fakes/fake_bootbits.c
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    src/fw/drivers/flash/flash_crc.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_spi_flash.c
    tests/fixtures/resources/builtin_resources.auto.c
    tests/fixtures/resources/pfs_resource_table.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_utf8_iterator
  SOURCES
    src/fw/applib/graphics/utf8.c
)

pbl_clar_test(test_codepoint
  SOURCES
    src/fw/applib/fonts/codepoint.c
)

pbl_clar_test(test_rtl_support
  SOURCES
    src/fw/applib/graphics/rtl_support.c
    src/fw/applib/graphics/utf8.c
    src/fw/applib/fonts/codepoint.c
)

pbl_clar_test(test_arabic_shaping
  SOURCES
    src/fw/applib/graphics/arabic_shaping.c
    src/fw/applib/graphics/utf8.c
)

pbl_clar_test(test_char_iterator
  SOURCES
    src/fw/applib/graphics/utf8.c
    src/fw/applib/graphics/framebuffer.c
    src/fw/applib/graphics/gtypes.c
    src/fw/applib/graphics/text_layout.c
    src/fw/applib/graphics/rtl_support.c
    src/fw/applib/graphics/arabic_shaping.c
    src/fw/applib/fonts/codepoint.c
    tests/fakes/fake_gbitmap_png.c
)

pbl_clar_test(test_word_iterator
  SOURCES
    src/fw/applib/graphics/utf8.c
    src/fw/applib/graphics/framebuffer.c
    src/fw/applib/graphics/gtypes.c
    src/fw/applib/graphics/text_layout.c
    src/fw/applib/graphics/rtl_support.c
    src/fw/applib/graphics/arabic_shaping.c
    src/fw/applib/fonts/codepoint.c
    tests/fakes/fake_gbitmap_png.c
)

pbl_clar_test(test_launcher_app_message
  SOURCES
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    src/fw/process_management/launcher_app_message.c
    src/fw/util/dict.c
    tests/fakes/fake_session.c
)

pbl_clar_test(test_line_layout
  SOURCES
    src/fw/applib/graphics/utf8.c
    src/fw/applib/graphics/framebuffer.c
    src/fw/applib/graphics/gtypes.c
    src/fw/applib/graphics/text_layout.c
    src/fw/applib/graphics/rtl_support.c
    src/fw/applib/graphics/arabic_shaping.c
    src/fw/applib/fonts/codepoint.c
    tests/fakes/fake_gbitmap_png.c
)

pbl_clar_test(test_alarm
  SOURCES
    src/fw/services/alarms/alarm.c
    src/fw/services/alarms/alarm_pin.c
    src/fw/flash_region/flash_region.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/util/time/time.c
    src/fw/util/time/mktime.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    src/fw/services/cron/service.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/blob_db/timeline_item_storage.c
    src/fw/services/settings/settings_file.c
    src/fw/services/settings/settings_raw_iter.c
    src/fw/services/timeline/attribute.c
    src/fw/services/timeline/item.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/attribute_group.c
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    tests/fakes/fake_spi_flash.c
  DEFINES CAPABILITY_HAS_HEALTH_TRACKING=0
  OVERRIDES dummy_board
)

pbl_clar_test(test_alarm_smart
  SOURCES
    src/fw/services/alarms/alarm.c
    src/fw/services/alarms/alarm_pin.c
    src/fw/flash_region/flash_region.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/util/time/time.c
    src/fw/util/time/mktime.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    src/fw/services/cron/service.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/blob_db/timeline_item_storage.c
    src/fw/services/settings/settings_file.c
    src/fw/services/settings/settings_raw_iter.c
    src/fw/services/timeline/attribute.c
    src/fw/services/timeline/item.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/attribute_group.c
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    tests/fakes/fake_clock.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_spi_flash.c
  DEFINES CAPABILITY_HAS_HEALTH_TRACKING=1
  OVERRIDES dummy_board
)

pbl_clar_test(test_battery_monitor
  SOURCES
    src/fw/services/battery/voltage/battery_state.c
    src/fw/services/battery/voltage/battery_curve.c
    src/fw/services/battery/battery_monitor.c
    tests/fakes/fake_battery.c
    tests/fakes/fake_rtc.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_alerts
  SOURCES
    src/fw/flash_region/flash_region.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/notifications/alerts_preferences.c
    src/fw/services/notifications/alerts.c
    src/fw/services/settings/settings_file.c
    src/fw/services/settings/settings_raw_iter.c
    tests/fakes/fake_battery.c
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_spi_flash.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_tick)

pbl_clar_test(test_heap
  SOURCES
    src/fw/applib/app_heap_util.c
)

pbl_clar_test(test_flash_logging
  SOURCES
    src/fw/debug/flash_logging.c
    src/fw/flash_region/flash_region.c
    tests/fakes/fake_spi_flash.c
  PLATFORMS asterix
  DEFINES DUMA_DISABLED PLATFORM_ASTERIX
  OVERRIDES dummy_board
)

pbl_clar_test(test_data_logging
  SOURCES
    src/fw/services/data_logging/dls_main.c
    src/fw/services/data_logging/dls_list.c
    src/fw/services/data_logging/dls_storage.c
    src/fw/services/data_logging/dls_endpoint.c
    src/fw/services/data_logging/dls_syscalls.c
    src/fw/services/regular_timer/service.c
    src/fw/util/shared_circular_buffer.c
    tests/fakes/fake_spi_flash.c
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    src/fw/flash_region/flash_region.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/filesystem/flash_translation.c
    tests/fakes/fake_session.c
    tests/fakes/fake_rtc.c
    src/fw/applib/data_logging.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_memory_layout
  SOURCES
    src/fw/kernel/memory_layout.c
)

pbl_clar_test(test_app_manager
  SOURCES
    src/fw/applib/app_exit_reason.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    src/fw/process_management/process_manager.c
    src/fw/process_management/pebble_process_info.c
    src/fw/process_management/app_manager.c
    src/fw/process_management/pebble_process_md.c
    src/fw/kernel/memory_layout.c
    src/fw/kernel/util/segment.c
    tests/fakes/fake_rtc.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_process_manager
  SOURCES
    src/fw/process_management/pebble_process_info.c
    src/fw/process_management/process_manager.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_pebble_app_info
  SOURCES
    src/fw/process_management/pebble_process_info.c
)

pbl_clar_test(test_pebble_process_md
  SOURCES
    src/fw/process_management/pebble_process_info.c
    src/fw/process_management/pebble_process_md.c
  PLATFORMS obelix gabbro
)

pbl_clar_test(test_phone_formatting
  SOURCES
    src/fw/applib/graphics/utf8.c
    src/fw/popups/phone_formatting.c
)

pbl_clar_test(test_timeline_api
  SOURCES
    src/fw/flash_region/filesystem_regions.c
    src/fw/flash_region/flash_region.c
    src/fw/services/blob_db/pin_db.c
    src/fw/services/blob_db/timeline_item_storage.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/settings/settings_file.c
    src/fw/services/settings/settings_raw_iter.c
    src/fw/services/timeline/attribute.c
    src/fw/services/timeline/attribute_group.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/item.c
    src/fw/services/timeline/timeline.c
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    src/fw/util/rand/rand.c
    src/fw/util/time/time.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_spi_flash.c
    tests/stubs/stubs_blob_db.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_app_run_state
  SOURCES
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    src/fw/process_management/app_run_state.c
)

pbl_clar_test(test_text_layout_8_bit
  SOURCE test_text_layout.c
  SOURCES
    src/fw/applib/graphics/utf8.c
    src/fw/applib/graphics/gbitmap.c
    src/fw/applib/graphics/gtypes.c
    src/fw/applib/graphics/gcolor_definitions.c
    src/fw/applib/graphics/8_bit/framebuffer.c
    src/fw/applib/graphics/framebuffer.c
    src/fw/applib/graphics/graphics.c
    src/fw/applib/graphics/gtransform.c
    src/fw/applib/graphics/bitblt.c
    src/fw/applib/graphics/8_bit/bitblt_private.c
    src/fw/applib/graphics/text_layout.c
    src/fw/applib/graphics/rtl_support.c
    src/fw/applib/graphics/arabic_shaping.c
    tests/fakes/fake_gbitmap_png.c
    src/fw/applib/fonts/codepoint.c
    src/fw/applib/graphics/graphics_private.c
    src/fw/applib/graphics/graphics_private_raw.c
    src/fw/applib/graphics/graphics_circle.c
    src/fw/applib/graphics/graphics_line.c
  PLATFORMS obelix
  DEFINES USE_DISPLAY_PERIMETER_ON_FONT_LAYOUT=1
)

pbl_clar_test(test_getting_started_button_combo
  SOURCES
    src/fw/apps/prf/recovery_first_use/getting_started_button_combo.c
  DEFINES CONFIG_RECOVERY_FW
)
