# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources)
if(CONFIG_AMBIENT_LIGHT_OPT3001)
  list(APPEND sources ambient_light_opt3001.c)
elseif(CONFIG_AMBIENT_LIGHT_W1160)
  list(APPEND sources ambient_light_w1160.c)
endif()
if(sources)
  list(APPEND sources ambient_light_common.c)
endif()
pbl_library()
pbl_library_sources(${sources})
