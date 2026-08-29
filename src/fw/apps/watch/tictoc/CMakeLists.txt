# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_APPS_WATCH_TICTOC_ROUND)
  set(variant round)
elseif(CONFIG_APPS_WATCH_TICTOC_BW)
  set(variant bw)
else()
  set(variant default)
endif()

file(GLOB variant_sources CONFIGURE_DEPENDS ${variant}/*.c)

pbl_library()
pbl_library_sources(tictoc.c ${variant_sources})
pbl_library_include_directories(.)
