# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# Runs Kconfig for the selected board and imports the result: one CMake
# variable per symbol, plus autoconf.h force-included into every
# compilation. Re-runs whenever a Kconfig or defconfig file changes.

set(PBL_DOTCONFIG ${PROJECT_BINARY_DIR}/.config)
set(PBL_AUTOCONF_H ${PROJECT_BINARY_DIR}/autoconf.h)
set(PBL_CONFIG_CMAKE ${PROJECT_BINARY_DIR}/config.cmake)

set(kconfig_args
  --srcdir ${PBL_BASE}
  --builddir ${PROJECT_BINARY_DIR}
  --board ${BOARD}
  --variant ${VARIANT}
)
# Kept by `./pbl menuconfig`; delete it to go back to the board defaults.
if(EXISTS ${PROJECT_BINARY_DIR}/menuconfig.conf)
  list(APPEND kconfig_args --fragment ${PROJECT_BINARY_DIR}/menuconfig.conf)
endif()

foreach(override ${CONFIG_OVERRIDES})
  list(APPEND kconfig_args --override ${override})
endforeach()

execute_process(
  COMMAND ${PYTHON_EXECUTABLE} ${PBL_BASE}/tools/cmake/kconfig.py ${kconfig_args}
  WORKING_DIRECTORY ${PBL_BASE}
  RESULT_VARIABLE ret
  OUTPUT_VARIABLE summary
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT ret EQUAL 0)
  message(FATAL_ERROR "Kconfig failed")
endif()
message(STATUS "Kconfig: ${summary}")

include(${PBL_CONFIG_CMAKE})

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${PBL_KCONFIG_DEPENDS} ${PBL_BASE}/tools/cmake/kconfig.py)

# The recovery firmware is selected with -DVARIANT=prf, which loads
# prj_prf.conf; keep the two spellings in sync.
if(VARIANT STREQUAL "prf" AND NOT CONFIG_RECOVERY_FW)
  message(FATAL_ERROR "VARIANT=prf but CONFIG_RECOVERY_FW is not set")
endif()
