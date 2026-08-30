# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(vendor/tinflate)
add_subdirectory(vendor/uPNG)

# The graphics code ships one implementation per screen colour depth.
if(CONFIG_SCREEN_COLOR_DEPTH_BITS EQUAL 1)
  set(excl 8_bit/**)
else()
  set(excl 1_bit/**)
endif()
list(APPEND excl vendor)

if(CONFIG_RECOVERY_FW)
  list(APPEND excl
    ui/dialogs/bt_conn_dialog.c
    ui/dialogs/expandable_dialog.c
    ui/option_menu_window.c
    ui/progress_window.c
    ui/selection_layer.c
    ui/date_selection_window.c
    ui/time_selection_window.c
    ui/time_range_selection_window.c
    ui/day_picker.c
    voice
  )
endif()

if(NOT CONFIG_BOARD_GETAFIX AND NOT CONFIG_BOARD_QEMU_GABBRO)
  list(APPEND excl ui/window_stack_animation_round*.c)
endif()

if(NOT CONFIG_MIC)
  list(APPEND excl voice/voice_window.c)
endif()

if(CONFIG_MAG)
  list(APPEND excl compass_service_stub.c)
else()
  list(APPEND excl compass_service.c)
endif()

if(CONFIG_TOUCH)
  list(APPEND excl touch_service_stub.c)
else()
  list(APPEND excl touch_service.c)
endif()

file(GLOB_RECURSE sources CONFIGURE_DEPENDS *.c)
pbl_filter_sources(sources EXCLUDE ${excl})

# applib_malloc.json describes the size of every applib object handed out
# to apps; the generator turns it into the allocator's tables.
set(applib_malloc_h ${CMAKE_CURRENT_BINARY_DIR}/applib_malloc.auto.h)
set(applib_malloc_c ${CMAKE_CURRENT_BINARY_DIR}/applib_malloc.auto.c)
file(GLOB applib_malloc_templates CONFIGURE_DEPENDS ${PBL_BASE}/tools/applib_malloc*)
add_custom_command(
  OUTPUT ${applib_malloc_h} ${applib_malloc_c}
  COMMAND ${PBL_TOOLCHAIN_ENV} ${PYTHON_EXECUTABLE} ${PBL_BASE}/tools/cmake/generate.py applib-malloc
          --input ${CMAKE_CURRENT_SOURCE_DIR}/applib_malloc.json
          --header ${applib_malloc_h} --impl ${applib_malloc_c}
          --min-sdk ${PBL_MIN_SDK_VERSION}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/applib_malloc.json ${applib_malloc_templates}
  WORKING_DIRECTORY ${PBL_BASE}
  COMMENT "Generating applib_malloc tables"
  VERBATIM
)
add_custom_target(pbl_applib_malloc DEPENDS ${applib_malloc_h})
add_dependencies(pbl_generated_headers pbl_applib_malloc)

pbl_gettext(${CMAKE_CURRENT_BINARY_DIR}/applib.pot ${sources})

pbl_library()
pbl_library_sources(${sources} ${applib_malloc_c})
