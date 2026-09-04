# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  dls_command.c
  dls_endpoint.c
  dls_list.c
  dls_main.c
  dls_storage.c
  dls_syscalls.c
)
pbl_library()
pbl_library_sources(${sources})
