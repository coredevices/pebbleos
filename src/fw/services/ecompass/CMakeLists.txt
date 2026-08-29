# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_MAG)
  pbl_library()
  pbl_library_sources(
    service.c
    correction.c
  )
endif()
