# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_library()
pbl_library_sources(
  assert.c
  syscalls.c
)
if(CONFIG_LIBC_NEWLIB_NANO)
  add_subdirectory(newlib)
endif()
