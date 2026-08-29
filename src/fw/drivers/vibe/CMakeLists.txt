# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources)
if(CONFIG_VIBE_AW86225)
  list(APPEND sources vibe_aw86225.c)
elseif(CONFIG_VIBE_AW8623X)
  list(APPEND sources vibe_aw8623x.c)
elseif(CONFIG_VIBE_DRV2604)
  list(APPEND sources vibe_drv2604.c)
endif()
pbl_library()
pbl_library_sources(${sources})
