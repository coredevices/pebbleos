# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources battery_monitor.c)
if(CONFIG_NRF_FUEL_GAUGE)
  list(APPEND sources
    nrf_fuel_gauge/battery_curve.c
    nrf_fuel_gauge/battery_state.c
  )
else()
  list(APPEND sources
    voltage/battery_curve.c
    voltage/battery_state.c
  )
endif()
pbl_library()
pbl_library_sources(${sources})
