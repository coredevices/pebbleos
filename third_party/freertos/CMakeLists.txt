# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_CORTEX_M4)
  set(port ARM_CM4F)
elseif(CONFIG_CORTEX_M33)
  set(port ARM_CM33)
else()
  message(FATAL_ERROR "Unrecognized Cortex core for FreeRTOS port")
endif()

set(kernel FreeRTOS-Kernel/FreeRTOS/Source)
file(GLOB sources CONFIGURE_DEPENDS
  ${kernel}/*.c
  ${kernel}/portable/GCC/${port}/*.c
)

# Only kernel/freertos may see these; FreeRTOSConfig.h lives there too.
set(FREERTOS_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/${kernel}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/${kernel}/portable/GCC/${port}
  ${PBL_BASE}/kernel/freertos
  CACHE INTERNAL "FreeRTOS include directories, private to the kernel shim"
)

pbl_library_named(freertos)
pbl_library_kind(stlib)
pbl_library_sources(${sources})
pbl_library_include_directories(${FREERTOS_INCLUDE_DIRS})
