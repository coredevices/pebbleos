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

pbl_include_directories(
  .
  ${kernel}/include
  ${kernel}/portable/GCC/${port}
)

# SF32LB52: SiFli's HAL programs MPU regions 0..4 and Pebble's static MPU
# setup uses 5..7, so the per-task configurable regions live at 8..11
# (four slots: AppRAM, WorkerRAM and two NULL placeholders, to satisfy the
# FreeRTOS port's multiple-of-four invariant).
if(CONFIG_SOC_SF32LB52)
  pbl_compile_definitions(
    FREERTOS_FIRST_MPU_REGION=8
    FREERTOS_LAST_MPU_REGION=11
  )
endif()

# QEMU CM33 has no vendor-reserved MPU regions, so Pebble's static regions
# take 0..3 and the FreeRTOS task regions 4..7.
if(CONFIG_QEMU AND CONFIG_CORTEX_M33)
  pbl_compile_definitions(
    FREERTOS_FIRST_MPU_REGION=4
    FREERTOS_LAST_MPU_REGION=7
  )
endif()

pbl_library_named(freertos)
pbl_library_kind(stlib)
pbl_library_sources(${sources})
