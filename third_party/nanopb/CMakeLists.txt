# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(nanopb_sources
  nanopb/pb_common.c
  nanopb/pb_encode.c
  nanopb/pb_decode.c
)
set(nanopb_includes nanopb)
pbl_include_directories(${nanopb_includes})
pbl_library_named(nanopb)
pbl_library_kind(stlib)
pbl_library_sources(${nanopb_sources})
