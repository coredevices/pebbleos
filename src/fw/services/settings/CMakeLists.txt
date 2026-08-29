# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  settings_file.c
  settings_raw_iter.c
)
pbl_library()
pbl_library_sources(${sources})
