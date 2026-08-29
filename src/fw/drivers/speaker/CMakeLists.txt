# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources)
if(CONFIG_SPEAKER_DA7212)
  list(APPEND sources nrf5/da7212.c)
elseif(CONFIG_SPEAKER_SF32LB)
  list(APPEND sources
    sf32lb52/audec.c
    sf32lb52/audio.c
  )
elseif(CONFIG_SPEAKER_QEMU)
  list(APPEND sources qemu/audio.c)
endif()
pbl_library()
pbl_library_sources(${sources})
