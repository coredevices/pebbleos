# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources common.c)
if(CONFIG_I2C_NRF5)
  list(APPEND sources nrf5.c)
elseif(CONFIG_I2C_SF32LB)
  list(APPEND sources sf32lb.c)
endif()
pbl_library()
pbl_library_sources(${sources})
