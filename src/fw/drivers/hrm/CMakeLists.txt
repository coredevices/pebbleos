# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources)
if(CONFIG_HRM_GH3X2X)
  list(APPEND sources gh3x2x.c)
elseif(CONFIG_HRM_STUB)
  list(APPEND sources stub.c)
endif()
pbl_library()
pbl_library_sources(${sources})
