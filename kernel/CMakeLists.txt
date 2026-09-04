# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

file(GLOB sources CONFIGURE_DEPENDS *.c arch/arm/*.c)

pbl_library_named(kernel)
pbl_library_kind(stlib)
pbl_library_sources(${sources})
pbl_library_include_directories(. arch)

pbl_include_directories(arch/arm/include)
