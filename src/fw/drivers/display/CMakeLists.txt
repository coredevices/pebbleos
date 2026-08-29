# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources)
if(CONFIG_DISPLAY_QEMU)
  list(APPEND sources ../qemu/qemu_display_hal.c)
elseif(CONFIG_DISPLAY_JDI_SF32LB)
  list(APPEND sources sf32lb/display_jdi.c)
elseif(CONFIG_DISPLAY_SHARP_LS013B7DH01_NRF5)
  list(APPEND sources sharp_ls013b7dh01/sharp_ls013b7dh01_nrf5.c)
endif()
pbl_library()
pbl_library_sources(${sources})
