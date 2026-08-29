# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  shared_prf_storage_debug.c
  shared_prf_storage.c
)
pbl_library()
pbl_library_sources(${sources})
