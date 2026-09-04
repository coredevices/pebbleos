# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources)
if(CONFIG_MIC_NRF5)
  list(APPEND sources nrf5/pdm.c)
elseif(CONFIG_MIC_SF32LB)
  list(APPEND sources sf32lb52/pdm.c)
elseif(CONFIG_MIC_QEMU)
  list(APPEND sources qemu/mic.c)
endif()
pbl_library()
pbl_library_sources(${sources})
