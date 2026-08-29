# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_RECOVERY_FW)
  set(shell prf)
elseif(CONFIG_SHELL_SDK)
  set(shell sdk)
else()
  set(shell normal)
endif()

# The launcher's app registry, and the app id defines the sources use.
set(registry ${CMAKE_CURRENT_BINARY_DIR}/system_app_registry_list.auto.h)
set(app_ids ${CMAKE_CURRENT_BINARY_DIR}/system_app_ids.auto.h)
add_custom_command(
  OUTPUT ${registry} ${app_ids}
  COMMAND ${PYTHON_EXECUTABLE} ${PBL_BASE}/tools/cmake/generate.py app-registry
          --input ${CMAKE_CURRENT_SOURCE_DIR}/${shell}/system_app_registry_list.json
          --registry ${registry} --enum ${app_ids}
          --config ${PBL_DOTCONFIG} --board ${PBL_BOARD_NAME}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${shell}/system_app_registry_list.json
          ${PBL_DOTCONFIG}
  WORKING_DIRECTORY ${PBL_BASE}
  COMMENT "Generating the system app registry"
  VERBATIM
)
add_custom_target(pbl_app_registry DEPENDS ${registry} ${app_ids})
add_dependencies(pbl_generated_headers pbl_app_registry)

file(GLOB sources CONFIGURE_DEPENDS *.c)
file(GLOB_RECURSE shell_sources CONFIGURE_DEPENDS ${shell}/*.c)

pbl_library()
pbl_library_sources(${sources} ${shell_sources})
