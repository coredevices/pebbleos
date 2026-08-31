# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# Renders a PDC into a PNG, which is how the tests' PDC baselines are
# produced. It runs on the host and is built by the unit test build.

file(GLOB pdc2png_sources CONFIGURE_DEPENDS src/*.c)

add_executable(pdc2png
  ${pdc2png_sources}
  ${PBL_BASE}/src/fw/applib/vendor/uPNG/upng.c
  ${PBL_BASE}/src/fw/applib/vendor/tinflate/tinflate.c
  ${PBL_BASE}/src/fw/applib/graphics/8_bit/bitblt_private.c
  ${PBL_BASE}/src/fw/applib/graphics/8_bit/framebuffer.c
  ${PBL_BASE}/src/fw/applib/graphics/bitblt.c
  ${PBL_BASE}/src/fw/applib/graphics/framebuffer.c
  ${PBL_BASE}/src/fw/applib/graphics/gbitmap.c
  ${PBL_BASE}/src/fw/applib/graphics/gbitmap_png.c
  ${PBL_BASE}/src/fw/applib/graphics/gcolor_definitions.c
  ${PBL_BASE}/src/fw/applib/graphics/gdraw_command.c
  ${PBL_BASE}/src/fw/applib/graphics/gdraw_command_frame.c
  ${PBL_BASE}/src/fw/applib/graphics/gdraw_command_image.c
  ${PBL_BASE}/src/fw/applib/graphics/gdraw_command_list.c
  ${PBL_BASE}/src/fw/applib/graphics/gdraw_command_sequence.c
  ${PBL_BASE}/src/fw/applib/graphics/gpath.c
  ${PBL_BASE}/src/fw/applib/graphics/graphics.c
  ${PBL_BASE}/src/fw/applib/graphics/graphics_circle.c
  ${PBL_BASE}/src/fw/applib/graphics/graphics_line.c
  ${PBL_BASE}/src/fw/applib/graphics/graphics_private.c
  ${PBL_BASE}/src/fw/applib/graphics/graphics_private_raw.c
  ${PBL_BASE}/src/fw/applib/graphics/gtypes.c
  ${PBL_BASE}/lib/util/keyed_circular_cache.c
  ${PBL_BASE}/lib/util/math.c
  ${PBL_BASE}/lib/util/platform.c
  ${PBL_BASE}/lib/util/trig.c
)

target_include_directories(pdc2png PRIVATE
  # Ahead of the rest: this time.h deliberately shadows the host's.
  ${PBL_BASE}/src/fw/util/time
  ${PBL_BASE}/tests/stubs
  ${PBL_BASE}/tests/fw/graphics
  ${PBL_BASE}/src/fw/applib/vendor/uPNG
  ${PBL_BASE}/src/fw/applib/vendor/tinflate
  ${PBL_BASE}/tests/overrides/default
  ${PBL_BASE}/include
  ${PBL_BASE}/subsys
  ${PBL_BASE}/src/fw
  ${PBL_BASE}/third_party/freertos
  ${PBL_BASE}/third_party/freertos/FreeRTOS-Kernel/FreeRTOS/Source/include
  ${PBL_BASE}/third_party/freertos/FreeRTOS-Kernel/FreeRTOS/Source/portable/GCC/ARM_CM3
)

# pdc2png resolves pbi2png against its own directory, so it gets a copy.
add_custom_command(TARGET pdc2png POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${PBL_BASE}/tools/pbi2png.py $<TARGET_FILE_DIR:pdc2png>/pbi2png.py
  VERBATIM)

target_compile_definitions(pdc2png PRIVATE
  UNITTEST
  CONFIG_SCREEN_COLOR_DEPTH_BITS=8
  # TODO: PBL-21014 Support arbitrary display sizes
  DISP_COLS=144
  DISP_ROWS=168
  DISPLAY_FRAMEBUFFER_BYTES=24192
  PBL_COLOR
  PBL_RECT
  CONFIG_BOARD_OBELIX=1
  CONFIG_PLATFORM_EMERY=1
  CONFIG_FLASH_GD25Q256E=1
  # Check that our <time.h> is the one that got included.
  _TIME_H_
  "PBI2PNG_EXE=\"pbi2png.py\""
)
target_compile_options(pdc2png PRIVATE -Wno-gnu-variable-sized-type-not-at-end)
