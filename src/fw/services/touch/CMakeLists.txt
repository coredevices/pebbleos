# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  touch.c
  touch_nav_service.c
  touch_session.c
)
pbl_library()
pbl_library_sources(${sources})
