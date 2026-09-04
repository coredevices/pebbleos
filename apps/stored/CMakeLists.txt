# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# Apps that ship inside the firmware's resource pack. They are built like
# any third-party app -- against the generated SDK, position independent,
# with the app metadata header stamped into the binary -- and the result
# is wrapped as a resource.
#
# Adding an app here also needs a raw resource entry named
# STORED_APP_<APPNAME> in resources/normal/base/resource_map.json (file:
# normal/raw/app_<appname>.bin) and an entry in the system app registry.

# Apps are built the way a third-party app is: the SDK's own flags and
# headers, not the firmware's. Only the configuration is shared.
execute_process(
  COMMAND ${PYTHON_EXECUTABLE} -c
    "import re,sys
text = open(sys.argv[1]).read()
for part in ('MAJOR', 'MINOR'):
    print(int(re.search(r'PROCESS_INFO_CURRENT_SDK_VERSION_%s (\\S+)' % part, text).group(1), 16))"
    ${PBL_BASE}/src/fw/process_management/pebble_process_info.h
  OUTPUT_VARIABLE sdk_version
  OUTPUT_STRIP_TRAILING_WHITESPACE
  COMMAND_ERROR_IS_FATAL ANY
)
string(REPLACE "\n" ";" sdk_version "${sdk_version}")
list(GET sdk_version 0 sdk_version_major)
list(GET sdk_version 1 sdk_version_minor)

set(sdk_flags
  -std=c99
  -mcpu=cortex-m3
  -mthumb
  -ffunction-sections
  -fdata-sections
  -fcommon
  -g
  -fPIE
  -Os
)
if(sdk_version_major EQUAL 5 AND sdk_version_minor GREATER 19)
  # Newer toolchains default time_t to 64-bit; apps expect 32-bit.
  list(APPEND sdk_flags -D_TIME_H_ -Dtime_t=long)
endif()
list(APPEND sdk_flags
  -Wall
  -Wextra
  -Werror
  -Wno-unused-parameter
  -Wno-error=unused-function
  -Wno-error=unused-variable
  -Wno-error=builtin-declaration-mismatch
  -Wno-error=format-truncation
  -Wno-error=expansion-to-defined
  -Wno-error=zero-length-bounds
  -Wno-error=cast-function-type
  -Wno-error=unused-value
  "SHELL:-include ${PBL_AUTOCONF_H}"
)

# The platform's feature defines, as third-party apps see them.
execute_process(
  COMMAND ${PYTHON_EXECUTABLE} -c
    "import sys; sys.path.insert(0, 'tools')
from pebble_sdk_platform import pebble_platforms
print(';'.join(pebble_platforms['${PBL_PLATFORM_NAME}']['DEFINES']))"
  WORKING_DIRECTORY ${PBL_BASE}
  OUTPUT_VARIABLE platform_defines
  OUTPUT_STRIP_TRAILING_WHITESPACE
  COMMAND_ERROR_IS_FATAL ANY
)

set_directory_properties(PROPERTIES
  COMPILE_OPTIONS "${sdk_flags}"
  COMPILE_DEFINITIONS "MAX_FONT_GLYPH_SIZE=${PBL_MAX_FONT_GLYPH_SIZE}"
  INCLUDE_DIRECTORIES ""
  LINK_OPTIONS ""
)

# An app is one blob: the firmware loads it into app RAM and runs it there,
# so its single LOAD segment is both writable and executable by design.
# binutils 2.39 and later warn about that unless told otherwise.
execute_process(COMMAND ${CMAKE_LINKER} --help OUTPUT_VARIABLE ld_help ERROR_QUIET)
set(app_rwx_flags "")
if(ld_help MATCHES "--no-warn-rwx-segments")
  set(app_rwx_flags -Wl,--no-warn-rwx-segments)
endif()

function(pbl_stored_app name)
  set(app_bin ${CMAKE_CURRENT_BINARY_DIR}/${name})
  set(app_ldscript ${CMAKE_CURRENT_SOURCE_DIR}/pebble_app.ld)
  set(generate ${PBL_TOOLCHAIN_ENV} ${PYTHON_EXECUTABLE} ${PBL_BASE}/tools/cmake/generate.py)

  # appinfo.json describes the app to the launcher and the app store.
  add_custom_command(
    OUTPUT ${app_bin}/appinfo.auto.c
    COMMAND ${generate} appinfo
            --input ${CMAKE_CURRENT_SOURCE_DIR}/${name}/appinfo.json
            --output ${app_bin}/appinfo.auto.c
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${name}/appinfo.json
    WORKING_DIRECTORY ${PBL_BASE}
    VERBATIM
  )
  # Stored apps have no resources and no message keys of their own, but
  # the SDK headers expect both files to exist.
  file(WRITE ${app_bin}/src/resource_ids.auto.h "#define DEFAULT_MENU_ICON  0\n")
  file(WRITE ${app_bin}/message_keys.auto.h "")

  file(GLOB_RECURSE sources CONFIGURE_DEPENDS ${name}/src/*.c)

  add_executable(stored_app_${name} ${sources} ${app_bin}/appinfo.auto.c)
  set_target_properties(stored_app_${name} PROPERTIES
    OUTPUT_NAME pebble-app
    SUFFIX .elf
    RUNTIME_OUTPUT_DIRECTORY ${app_bin}
  )
  add_dependencies(stored_app_${name} pbl_native_sdk)
  target_compile_definitions(stored_app_${name} PRIVATE ${platform_defines})
  target_include_directories(stored_app_${name} PRIVATE
    ${PBL_SDK_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/${name}
    ${CMAKE_CURRENT_SOURCE_DIR}/${name}/src
    ${app_bin}
    ${app_bin}/src
  )
  target_link_libraries(stored_app_${name} PRIVATE ${PBL_SDK_DIR}/lib/libpebble.a)
  target_link_options(stored_app_${name} PRIVATE
    -mcpu=cortex-m3
    -mthumb
    -Wl,--gc-sections
    -Wl,--warn-common
    -fPIE
    -Os
    -Wl,--emit-relocs
    ${app_rwx_flags}
    -T${app_ldscript}
  )
  set_property(TARGET stored_app_${name} APPEND PROPERTY
    LINK_DEPENDS ${app_ldscript})

  set(elf ${app_bin}/pebble-app.elf)
  add_custom_command(
    OUTPUT ${app_bin}/pebble-app.raw.bin
    COMMAND ${CMAKE_OBJCOPY} -S -R .stack -R .priv_bss -R .bss -R .retained
            -O binary ${elf} ${app_bin}/pebble-app.raw.bin
    DEPENDS stored_app_${name}
    VERBATIM
  )
  add_custom_command(
    OUTPUT ${app_bin}/pebble-app.bin
    COMMAND ${generate} inject-metadata
            --input ${app_bin}/pebble-app.raw.bin
            --output ${app_bin}/pebble-app.bin --elf ${elf}
    DEPENDS ${app_bin}/pebble-app.raw.bin
    WORKING_DIRECTORY ${PBL_BASE}
    VERBATIM
  )

  set(reso ${PROJECT_BINARY_DIR}/resources/normal/base/raw/app_${name}.bin.reso)
  add_custom_command(
    OUTPUT ${reso}
    COMMAND ${generate} app-reso --input ${app_bin}/pebble-app.bin
            --output ${reso} --name ${name}
    DEPENDS ${app_bin}/pebble-app.bin
    WORKING_DIRECTORY ${PBL_BASE}
    COMMENT "Packaging the ${name} app as a resource"
    VERBATIM
  )
  # The resource pipeline lives in another directory, so the rule needs a
  # target here to hang off.
  add_custom_target(stored_app_${name}_resource DEPENDS ${reso})
  pbl_dynamic_resource(${reso})
endfunction()

file(GLOB apps RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} CONFIGURE_DEPENDS */)
foreach(app ${apps})
  string(REGEX REPLACE "/$" "" app ${app})
  if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${app})
    pbl_stored_app(${app})
  endif()
endforeach()
