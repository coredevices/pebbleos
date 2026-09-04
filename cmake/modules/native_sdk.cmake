# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# The native SDK shims. The same generator produces the firmware side of
# the syscall boundary (pebble.auto.c) and the app side (libpebble.a plus
# the SDK headers), so the firmware build depends on it as well.
# The rest of the shippable SDK is packaged by the `sdk` target; see
# cmake/modules/sdk.cmake and docs/development/sdk_export.md.

function(pbl_native_sdk)
  set(script ${PBL_BASE}/tools/generate_native_sdk/generate_pebble_native_sdk_files.py)
  set(symbols ${PBL_BASE}/tools/generate_native_sdk/exported_symbols.json)
  set(sdk_dir ${PROJECT_BINARY_DIR}/sdk/${PBL_PLATFORM_NAME})

  # The headers the generator parses, as listed in exported_symbols.json.
  execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -c
      "import json,os,sys
root = os.getcwd()
files = json.load(open(sys.argv[1]))['files']
for f in files:
    src = os.path.join(root, 'src', f)
    print(src if os.path.exists(src) else os.path.join(root, f))"
      ${symbols}
    WORKING_DIRECTORY ${PBL_BASE}
    OUTPUT_VARIABLE headers
    OUTPUT_STRIP_TRAILING_WHITESPACE
    COMMAND_ERROR_IS_FATAL ANY
  )
  string(REPLACE "\n" ";" headers "${headers}")

  set(PBL_PEBBLE_AUTO_C ${PROJECT_BINARY_DIR}/src/fw/pebble.auto.c)
  set(outputs
    ${PBL_PEBBLE_AUTO_C}
    ${sdk_dir}/include/pebble.h
    ${sdk_dir}/include/pebble_sdk_version.h
    ${sdk_dir}/include/pebble_process_info.h
    ${sdk_dir}/include/pebble_worker.h
    ${sdk_dir}/include/pebble_worker_sdk_version.h
    ${sdk_dir}/lib/libpebble.a
  )

  add_custom_command(
    OUTPUT ${outputs}
    COMMAND ${PBL_TOOLCHAIN_ENV} ${PYTHON_EXECUTABLE} ${script} --sdk-dir=${sdk_dir}
            ${symbols} ${PBL_BASE}/src ${PROJECT_BINARY_DIR}/src
            ${PBL_PLATFORM_NAME}
    DEPENDS ${script} ${symbols} ${headers}
    WORKING_DIRECTORY ${PBL_BASE}
    COMMENT "Generating native SDK shims"
    VERBATIM
  )
  # The font keys apps may refer to, from the same whitelist.
  set(fonts_header ${sdk_dir}/include/pebble_fonts.h)
  add_custom_command(
    OUTPUT ${fonts_header}
    COMMAND ${PBL_TOOLCHAIN_ENV} ${PYTHON_EXECUTABLE} ${PBL_BASE}/tools/cmake/generate.py sdk-fonts-header
            --input ${symbols} --output ${fonts_header}
            --platform ${PBL_PLATFORM_NAME}
    DEPENDS ${symbols} ${outputs}
    WORKING_DIRECTORY ${PBL_BASE}
    VERBATIM
  )

  add_custom_target(pbl_native_sdk DEPENDS ${outputs} ${fonts_header})

  set(PBL_SDK_DIR ${sdk_dir} PARENT_SCOPE)
  set(PBL_PEBBLE_AUTO_C ${PBL_PEBBLE_AUTO_C} PARENT_SCOPE)
endfunction()
