# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_reminders
  SOURCES
    src/fw/util/legacy_checksum.c
    tests/fakes/fake_spi_flash.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    src/fw/flash_region/flash_region.c
    src/fw/util/time/time.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/services/blob_db/timeline_item_storage.c
    src/fw/services/settings/settings_file.c
    src/fw/services/settings/settings_raw_iter.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/blob_db/reminder_db.c
    src/fw/services/timeline/attribute.c
    src/fw/services/timeline/item.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/attribute_group.c
    src/fw/services/timeline/reminders.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_calendar
  SOURCES
    src/fw/services/blob_db/pin_db.c
    src/fw/services/blob_db/timeline_item_storage.c
    src/fw/services/timeline/attribute.c
    src/fw/services/timeline/attribute_group.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/calendar.c
    src/fw/services/timeline/event.c
    src/fw/services/timeline/item.c
    src/fw/services/timeline/timeline.c
    src/fw/util/crc8.c
    src/fw/util/time/time.c
    tests/fakes/fake_events.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_settings_file.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_timeline_peek_event
  SOURCES
    src/fw/services/blob_db/pin_db.c
    src/fw/services/blob_db/timeline_item_storage.c
    src/fw/services/timeline/attribute.c
    src/fw/services/timeline/attribute_group.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/event.c
    src/fw/services/timeline/item.c
    src/fw/services/timeline/peek.c
    src/fw/services/timeline/timeline.c
    src/fw/util/crc8.c
    src/fw/util/time/time.c
    tests/fakes/fake_events.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_settings_file.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_attribute
  SOURCES
    src/fw/services/timeline/attribute.c
)

pbl_clar_test(test_timeline_model
  SOURCES
    src/fw/util/time/time.c
    src/fw/util/time/mktime.c
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    tests/fakes/fake_spi_flash.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    tests/fakes/fake_rtc.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/flash_region/flash_region.c
    src/fw/services/blob_db/timeline_item_storage.c
    src/fw/services/settings/settings_file.c
    src/fw/services/settings/settings_raw_iter.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/timeline/attribute.c
    src/fw/services/timeline/item.c
    src/fw/services/timeline/attributes_actions.c
    src/fw/services/timeline/attribute_group.c
    src/fw/services/timeline/timeline.c
    src/fw/services/blob_db/pin_db.c
    src/fw/apps/system/timeline/model.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_timeline_resources
  SOURCES
    src/fw/process_management/pebble_process_info.c
    src/fw/services/timeline/timeline_resources.c
    tests/fixtures/resources/timeline_resource_table.auto.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_alarm_layout
  SOURCES
    src/fw/services/alarms/alarm.c
    src/fw/services/timeline/alarm_layout.c
    src/fw/services/timeline/attribute.c
  PLATFORMS obelix gabbro
  DEFINES CAPABILITY_HAS_HEALTH_TRACKING=0
  OVERRIDES dummy_board
)

pbl_clar_test(test_timeline_layouts
  SOURCES
    src/fw/applib/fonts/codepoint.c
    src/fw/applib/graphics/@BITDEPTH@_bit/bitblt_private.c
    src/fw/applib/graphics/@BITDEPTH@_bit/framebuffer.c
    src/fw/applib/graphics/bitblt.c
    src/fw/applib/graphics/framebuffer.c
    src/fw/applib/graphics/gbitmap.c
    src/fw/applib/graphics/gbitmap_png.c
    src/fw/applib/graphics/gbitmap_sequence.c
    src/fw/applib/graphics/gcolor_definitions.c
    src/fw/applib/graphics/gdraw_command.c
    src/fw/applib/graphics/gdraw_command_frame.c
    src/fw/applib/graphics/gdraw_command_image.c
    src/fw/applib/graphics/gdraw_command_list.c
    src/fw/applib/graphics/gdraw_command_sequence.c
    src/fw/applib/graphics/gpath.c
    src/fw/applib/graphics/graphics.c
    src/fw/applib/graphics/graphics_bitmap.c
    src/fw/applib/graphics/graphics_circle.c
    src/fw/applib/graphics/graphics_line.c
    src/fw/applib/graphics/graphics_private.c
    src/fw/applib/graphics/graphics_private_raw.c
    src/fw/applib/graphics/gtransform.c
    src/fw/applib/graphics/gtypes.c
    src/fw/applib/graphics/perimeter.c
    src/fw/applib/graphics/text_layout.c
    src/fw/applib/graphics/rtl_support.c
    src/fw/applib/graphics/arabic_shaping.c
    src/fw/applib/graphics/text_render.c
    src/fw/applib/graphics/text_resources.c
    src/fw/applib/graphics/utf8.c
    src/fw/applib/ui/action_button.c
    src/fw/applib/ui/animation_interpolate.c
    src/fw/applib/ui/content_indicator.c
    src/fw/applib/ui/inverter_layer.c
    src/fw/applib/ui/kino/kino_layer.c
    src/fw/applib/ui/kino/kino_player.c
    src/fw/applib/ui/kino/kino_reel.c
    src/fw/applib/ui/kino/kino_reel/transform.c
    src/fw/applib/ui/kino/kino_reel_custom.c
    src/fw/applib/ui/kino/kino_reel_gbitmap.c
    src/fw/applib/ui/kino/kino_reel_gbitmap_sequence.c
    src/fw/applib/ui/kino/kino_reel_pdci.c
    src/fw/applib/ui/kino/kino_reel_pdcs.c
    src/fw/applib/ui/layer.c
    src/fw/applib/ui/menu_layer.c
    src/fw/applib/ui/menu_layer_system_cells.c
    src/fw/applib/ui/scroll_layer.c
    src/fw/applib/ui/shadows.c
    src/fw/applib/ui/status_bar_layer.c
    src/fw/applib/ui/text_layer.c
    src/fw/applib/ui/text_layer_flow.c
    src/fw/applib/ui/window.c
    src/fw/applib/vendor/tinflate/tinflate.c
    src/fw/applib/vendor/uPNG/upng.c
    src/fw/apps/system/timeline/text_node.c
    src/fw/board/displays/display_getafix.c
    src/fw/drivers/flash/flash_crc.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/flash_region/flash_region.c
    src/fw/resource/resource.c
    src/fw/resource/resource_storage.c
    src/fw/resource/resource_storage_builtin.c
    src/fw/resource/resource_storage_file.c
    src/fw/resource/resource_storage_flash.c
    src/fw/services/filesystem/app_file.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/system/hexdump.c
    src/fw/util/buffer.c
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    src/fw/util/stringlist.c
    src/fw/util/time/time.c
    tests/fakes/fake_applib_resource.c
    tests/fakes/fake_clock.c
    tests/fakes/fake_display.c
    tests/fakes/fake_fonts.c
    tests/fakes/fake_gbitmap_get_data_row.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_spi_flash.c
    tests/fixtures/resources/builtin_resources.auto.c
    tests/fixtures/resources/pfs_resource_table.c
    tests/fixtures/resources/timeline_resource_table.auto.c
    tests/stubs/stubs_animation.c
    src/fw/apps/system/timeline/pin_window.c
    src/fw/popups/timeline/timeline_item_layer.c
    src/fw/services/timeline/attribute.c
    src/fw/services/timeline/generic_layout.c
    src/fw/services/timeline/layout_layer.c
    src/fw/services/timeline/layout_node.c
    src/fw/services/timeline/timeline_layout.c
    src/fw/services/timeline/timeline_resources.c
    src/fw/services/timeline/weather_layout.c
    src/fw/services/weather/weather_types.c
    src/fw/shell/system_theme.c
    tests/stubs/stubs_clock.c
    tests/stubs/stubs_timeline_layout.c
  PLATFORMS obelix gabbro
  DEFINES USE_DISPLAY_PERIMETER_ON_FONT_LAYOUT=1
  OVERRIDES dummy_board
  TEST_IMAGES
)

pbl_clar_test(test_swap_layer_touch
  SOURCES
    src/fw/applib/graphics/gcolor_definitions.c
    src/fw/applib/graphics/gtypes.c
    src/fw/applib/ui/layer.c
    src/fw/applib/ui/recognizer/recognizer.c
    src/fw/applib/ui/recognizer/recognizer_manager.c
    src/fw/applib/ui/recognizer/tap.c
    src/fw/applib/ui/recognizer/pan.c
    src/fw/applib/ui/recognizer/swipe.c
    src/fw/applib/ui/recognizer/touch_nav.c
    src/fw/services/timeline/swap_layer.c
    tests/fakes/fake_animation.c
    tests/fakes/fake_rtc.c
  DEFINES CONFIG_TOUCH
  OVERRIDES dummy_board
)
