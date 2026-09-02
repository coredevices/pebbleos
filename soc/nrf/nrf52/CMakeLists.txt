# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_library()
pbl_library_sources(
  init.c
  sleep.c
  idle.c
)
pbl_linker_sources(memory linker.ld)
