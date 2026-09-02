# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_include_directories(. applib/vendor/uPNG applib/vendor/tinflate)

# The firmware reports the version it was built from.
set(git_version ${CMAKE_CURRENT_BINARY_DIR}/git_version.auto.h)
add_custom_command(
  OUTPUT ${git_version} ${CMAKE_CURRENT_BINARY_DIR}/git_version.stamp
  COMMAND ${PBL_TOOLCHAIN_ENV} ${PYTHON_EXECUTABLE} ${PBL_BASE}/tools/cmake/generate.py git-version
          --template ${CMAKE_CURRENT_SOURCE_DIR}/git_version.auto.h.in
          --output ${git_version}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/git_version.auto.h.in
  WORKING_DIRECTORY ${PBL_BASE}
  COMMENT "Checking the firmware version"
  VERBATIM
)
# The stamp is never written, so the version is re-checked on every build;
# the header itself is only rewritten when it actually changes.
add_custom_target(pbl_git_version DEPENDS ${git_version})
add_dependencies(pbl_generated_headers pbl_git_version)

add_subdirectory(startup)
add_subdirectory(drivers)
add_subdirectory(board)
add_subdirectory(shell)
add_subdirectory(services)
add_subdirectory(applib)
add_subdirectory(mfg)
add_subdirectory(comm)
add_subdirectory(console)
add_subdirectory(debug)
add_subdirectory(flash_region)
add_subdirectory(kernel)
add_subdirectory(popups)
add_subdirectory(process_management)
add_subdirectory(process_state)
add_subdirectory(resource)
add_subdirectory(syscall)
add_subdirectory(system)
add_subdirectory(util)
add_subdirectory(apps/core)

if(CONFIG_RECOVERY_FW)
  add_subdirectory(apps/prf)
else()
  add_subdirectory(apps)
endif()

file(GLOB sources CONFIGURE_DEPENDS *.c)
pbl_library()
pbl_library_sources(${sources})

if(NOT CONFIG_RECOVERY_FW)
  # Translatable strings from the firmware core; apps, services and applib
  # have their own catalogs, merged in here.
  file(GLOB_RECURSE pot_sources CONFIGURE_DEPENDS *.c *.h *.def)
  pbl_filter_sources(pot_sources EXCLUDE apps services applib)
  pbl_gettext(${PROJECT_BINARY_DIR}/fw.pot ${pot_sources})

  set(catalogs ${PROJECT_BINARY_DIR}/fw.pot)
  get_property(service_pots GLOBAL PROPERTY PBL_SERVICES_POT)
  if(service_pots)
    list(APPEND catalogs ${CMAKE_CURRENT_BINARY_DIR}/services/services.pot)
  endif()
  list(APPEND catalogs
    ${CMAKE_CURRENT_BINARY_DIR}/applib/applib.pot
    ${CMAKE_CURRENT_BINARY_DIR}/apps/apps.pot
  )
  pbl_firmware_pot(${PROJECT_BINARY_DIR}/pebbleos.pot ${catalogs})
endif()
