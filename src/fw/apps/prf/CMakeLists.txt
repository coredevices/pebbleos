# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(excl)
if(NOT CONFIG_BOARD_ASTERIX)
  list(APPEND excl
    mfg_speaker_asterix.c
    mfg_mic_asterix.c
  )
endif()
if(NOT CONFIG_BOARD_OBELIX)
  list(APPEND excl
    mfg_speaker_obelix.c
    mfg_mic_obelix.c
  )
endif()
if(NOT (CONFIG_BOARD_OBELIX AND CONFIG_MFG))
  list(APPEND excl
    mfg_hrm_ctr_leakage_obelix.c
  )
endif()
if(NOT CONFIG_BOARD_GETAFIX)
  list(APPEND excl
    mfg_mic_getafix.c
  )
endif()
if(NOT CONFIG_MAG)
  list(APPEND excl
    mfg_mag.c
  )
endif()
if(NOT CONFIG_TOUCH)
  list(APPEND excl
    mfg_touch.c
  )
endif()
pbl_library()
file(GLOB_RECURSE sources CONFIGURE_DEPENDS *.c)
pbl_filter_sources(sources EXCLUDE ${excl})
pbl_library_sources(${sources})
