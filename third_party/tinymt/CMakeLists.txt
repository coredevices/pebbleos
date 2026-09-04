# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(tinymt_sources TinyMT/tinymt/tinymt32.c)
set(tinymt_includes TinyMT/tinymt)
pbl_include_directories(${tinymt_includes})
pbl_library_named(tinymt32)
pbl_library_kind(stlib)
pbl_library_sources(${tinymt_sources})
