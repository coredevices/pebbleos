# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  put_bytes.c
  put_bytes_storage.c
  put_bytes_storage_file.c
  put_bytes_storage_raw.c
)
pbl_library()
pbl_library_sources(${sources})
