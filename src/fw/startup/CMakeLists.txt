# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_library()
if(CONFIG_BOARD_ASTERIX)
  pbl_library_sources(
    ${PBL_BASE}/third_party/hal_nordic/nrfx/mdk/system_nrf52840.c
    startup_cortex_m.c
  )
elseif(CONFIG_BOARD_OBELIX OR CONFIG_BOARD_GETAFIX)
  pbl_library_sources(startup_sf32lb52.c)
elseif(CONFIG_QEMU)
  pbl_library_sources(startup_qemu.c)
else()
  message(FATAL_ERROR "No clock configuration file specified for this platform")
endif()
