# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources analytics.c)
if(NOT CONFIG_RECOVERY_FW)
  list(APPEND sources native.c)
  pbl_compile_definitions(ANALYTICS_NATIVE)
endif()
pbl_library()
pbl_library_sources(${sources})
