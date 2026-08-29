# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  circles.c
  gpath_masking.c
  rotated_bitmap.c
  single_line.c
  tests.c
  text.c
  text_clipping.c
)
pbl_library()
pbl_library_sources(${sources})
pbl_library_include_directories(.)
