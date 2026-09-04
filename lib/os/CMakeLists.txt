# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

file(GLOB_RECURSE sources CONFIGURE_DEPENDS *.c)

pbl_library_named(libos)
pbl_library_kind(stlib)
pbl_library_sources(${sources})
pbl_library_include_directories(.)
