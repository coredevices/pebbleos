# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  i18n.c
  syscalls.c
)
pbl_library()
pbl_library_sources(${sources})
