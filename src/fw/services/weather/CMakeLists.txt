# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  weather_service.c
  weather_types.c
)
pbl_library()
pbl_library_sources(${sources})
