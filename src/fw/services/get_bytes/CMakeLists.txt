# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  get_bytes.c
  get_bytes_storage.c
  get_bytes_storage_coredump.c
  get_bytes_storage_file.c
  get_bytes_storage_flash.c
)
pbl_library()
pbl_library_sources(${sources})
