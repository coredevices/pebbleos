# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

# The only place in the tree that may include FreeRTOS headers.
file(GLOB sources CONFIGURE_DEPENDS *.c)

pbl_library_named(kernel_freertos)
pbl_library_kind(stlib)
pbl_library_sources(${sources})
pbl_library_include_directories(. ${FREERTOS_INCLUDE_DIRS})
