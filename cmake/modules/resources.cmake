# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# The resource pipeline: every entry of the board's resource maps becomes
# a .reso, the .reso files are collected into a resource ball, and the
# firmware's resource tables, headers and pbpack are derived from it.

define_property(GLOBAL PROPERTY PBL_DYNAMIC_RESOURCES
  BRIEF_DOCS "Resources produced by the build itself (stored apps, timezones)")

set(PBL_RESOURCES_PY ${PBL_BASE}/tools/cmake/resources.py)

# A .reso another build step produces, added to the resource ball after
# the ones the resource maps declare.
function(pbl_dynamic_resource path)
  set_property(GLOBAL APPEND PROPERTY PBL_DYNAMIC_RESOURCES ${path})
endfunction()

function(pbl_resource_command)
  cmake_parse_arguments(ARG "" "OUTPUT;COMMENT" "ARGS;DEPENDS" ${ARGN})
  add_custom_command(
    OUTPUT ${ARG_OUTPUT}
    COMMAND ${PYTHON_EXECUTABLE} ${PBL_RESOURCES_PY} ${ARG_ARGS}
    DEPENDS ${ARG_DEPENDS} ${PBL_RESOURCES_PY}
    WORKING_DIRECTORY ${PBL_BASE}
    COMMENT "${ARG_COMMENT}"
    VERBATIM
  )
endfunction()

# Called once, after every directory has been added, so the dynamic
# resources are all registered.
function(pbl_resources)
  set(fw_bin ${PROJECT_BINARY_DIR}/src/fw)
  set(manifest ${PROJECT_BINARY_DIR}/resources/manifest.pickle)
  set(ball ${PROJECT_BINARY_DIR}/system_resources.resball)
  set(pbpack ${PROJECT_BINARY_DIR}/system_resources.pbpack)

  get_property(dynamic GLOBAL PROPERTY PBL_DYNAMIC_RESOURCES)

  # The timezone database is built straight into a resource.
  if(NOT CONFIG_RECOVERY_FW)
    set(tzdata ${PROJECT_BINARY_DIR}/resources/normal/base/tzdata/tzdata.bin.reso)
    set(olson ${PBL_BASE}/resources/normal/base/tzdata/timezones_olson.txt)
    pbl_resource_command(
      OUTPUT ${tzdata}
      ARGS tzdata --input ${olson} --output ${tzdata}
      DEPENDS ${olson}
      COMMENT "Generating timezone database"
    )
    list(APPEND dynamic ${tzdata})
  endif()

  # Resolve the resource maps into per-resource build rules.
  execute_process(
    COMMAND ${PYTHON_EXECUTABLE} ${PBL_RESOURCES_PY} manifest
      --builddir ${PROJECT_BINARY_DIR}
      --platform ${PBL_PLATFORM_NAME}
      --board-name ${PBL_BOARD_NAME}
      --variant ${VARIANT}
      --output ${manifest}
      --cmake-output ${PROJECT_BINARY_DIR}/resources/resources.cmake
      --dynamic ${dynamic}
    WORKING_DIRECTORY ${PBL_BASE}
    OUTPUT_VARIABLE summary
    OUTPUT_STRIP_TRAILING_WHITESPACE
    COMMAND_ERROR_IS_FATAL ANY
  )
  message(STATUS "Resources: ${summary}")

  include(${PROJECT_BINARY_DIR}/resources/resources.cmake)
  set_property(DIRECTORY ${PBL_BASE} APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${PBL_RESOURCE_MAP_FILES} ${PBL_RESOURCES_PY})

  pbl_resource_command(
    OUTPUT ${ball}
    ARGS ball --manifest ${manifest} --output ${ball}
    DEPENDS ${PBL_RESO_FILES} ${dynamic} ${manifest}
    COMMENT "Building resource ball"
  )

  if(NOT CONFIG_RECOVERY_FW)
    add_custom_command(
      OUTPUT ${pbpack}
      COMMAND ${PYTHON_EXECUTABLE} ${PBL_RESOURCES_PY} pbpack
              --ball ${ball} --output ${pbpack} --system
      COMMAND ${PYTHON_EXECUTABLE} ${PBL_BASE}/tools/cmake/firmware.py
              size-resources --config ${PBL_DOTCONFIG} --pbpack ${pbpack}
      DEPENDS ${ball} ${PBL_RESOURCES_PY}
      WORKING_DIRECTORY ${PBL_BASE}
      COMMENT "Packing system resources"
      VERBATIM
    )
    set(PBL_PBPACK ${pbpack} PARENT_SCOPE)
    set(version_header_args --pbpack ${pbpack})
    set(version_header_depends ${pbpack})
  else()
    set(PBL_PBPACK "" PARENT_SCOPE)
    set(version_header_args "")
    set(version_header_depends "")
  endif()

  pbl_resource_command(
    OUTPUT ${fw_bin}/resource/resource_version.auto.h
    ARGS version-header --output ${fw_bin}/resource/resource_version.auto.h
         ${version_header_args}
    DEPENDS ${version_header_depends}
    COMMENT "Generating resource_version.auto.h"
  )

  pbl_resource_command(
    OUTPUT ${fw_bin}/resource/resource_ids.auto.h
    ARGS resource-ids --ball ${ball} --output ${fw_bin}/resource/resource_ids.auto.h
    DEPENDS ${ball}
    COMMENT "Generating resource_ids.auto.h"
  )

  pbl_resource_command(
    OUTPUT ${fw_bin}/builtin_resources.auto.c
    ARGS builtin --ball ${ball} --output ${fw_bin}/builtin_resources.auto.c
         --include resource/resource_ids.auto.h
    DEPENDS ${ball} ${fw_bin}/resource/resource_ids.auto.h
    COMMENT "Generating builtin_resources.auto.c"
  )

  pbl_resource_command(
    OUTPUT ${fw_bin}/font_resource_keys.auto.h
    ARGS font-header --ball ${ball} --output ${fw_bin}/font_resource_keys.auto.h
    DEPENDS ${ball}
    COMMENT "Generating font_resource_keys.auto.h"
  )
  pbl_resource_command(
    OUTPUT ${fw_bin}/font_resource_table.auto.h
    ARGS font-table --ball ${ball} --output ${fw_bin}/font_resource_table.auto.h
    DEPENDS ${ball}
    COMMENT "Generating font_resource_table.auto.h"
  )

  set(generated
    ${fw_bin}/resource/resource_version.auto.h
    ${fw_bin}/resource/resource_ids.auto.h
    ${fw_bin}/font_resource_keys.auto.h
    ${fw_bin}/font_resource_table.auto.h
  )
  set(sources ${fw_bin}/builtin_resources.auto.c)

  if(NOT CONFIG_RECOVERY_FW)
    pbl_resource_command(
      OUTPUT ${fw_bin}/resource/timeline_resource_table.auto.c
      ARGS timeline-table --manifest ${manifest}
           --output ${fw_bin}/resource/timeline_resource_table.auto.c
      DEPENDS ${manifest}
      COMMENT "Generating timeline_resource_table.auto.c"
    )
    pbl_resource_command(
      OUTPUT ${fw_bin}/resource/timeline_resource_ids.auto.h
      ARGS timeline-ids --manifest ${manifest}
           --output ${fw_bin}/resource/timeline_resource_ids.auto.h
      DEPENDS ${manifest}
      COMMENT "Generating timeline_resource_ids.auto.h"
    )
    pbl_resource_command(
      OUTPUT ${PROJECT_BINARY_DIR}/resources/layouts.json.auto
      ARGS layouts --manifest ${manifest}
           --template ${PBL_BASE}/resources/normal/base/layouts/layouts.json.in
           --output ${PROJECT_BINARY_DIR}/resources/layouts.json.auto
      DEPENDS ${manifest} ${PBL_BASE}/resources/normal/base/layouts/layouts.json.in
      COMMENT "Generating layouts.json"
    )
    list(APPEND generated
      ${fw_bin}/resource/timeline_resource_ids.auto.h
      ${PROJECT_BINARY_DIR}/resources/layouts.json.auto
    )
    list(APPEND sources ${fw_bin}/resource/timeline_resource_table.auto.c)

    pbl_resource_command(
      OUTPUT ${fw_bin}/resource/pfs_resource_table.auto.c
      ARGS pfs --manifest ${manifest}
           --output ${fw_bin}/resource/pfs_resource_table.auto.c
           --include resource/resource_ids.auto.h
      DEPENDS ${manifest} ${fw_bin}/resource/resource_ids.auto.h
      COMMENT "Generating pfs_resource_table.auto.c"
    )
    list(APPEND sources ${fw_bin}/resource/pfs_resource_table.auto.c)
  endif()

  add_custom_target(pbl_resource_headers DEPENDS ${generated})
  add_dependencies(pbl_generated_headers pbl_resource_headers)

  set(PBL_RESOURCE_SOURCES ${sources} PARENT_SCOPE)
  set(PBL_RESOURCE_BALL ${ball} PARENT_SCOPE)
  set(PBL_LAYOUTS ${PROJECT_BINARY_DIR}/resources/layouts.json.auto PARENT_SCOPE)
endfunction()
