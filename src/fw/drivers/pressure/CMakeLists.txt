# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources)
if(CONFIG_PRESSURE_BMP390)
  list(APPEND sources bmp390.c)
endif()
pbl_library()
pbl_library_sources(${sources})
