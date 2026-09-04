# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  app_order_storage.c
  app_storage.c
  process_commands.c
  process_loader_storage.c
)
pbl_library()
pbl_library_sources(${sources})
