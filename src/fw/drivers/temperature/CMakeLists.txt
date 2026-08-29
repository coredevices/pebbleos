# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources)
if(CONFIG_TEMPERATURE_SF32LB)
  list(APPEND sources sf32lb.c)
elseif(CONFIG_TEMPERATURE_STUB)
  list(APPEND sources stubs.c)
endif()
pbl_library()
pbl_library_sources(${sources})
