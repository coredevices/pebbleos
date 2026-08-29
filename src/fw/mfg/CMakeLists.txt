# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

# The active board's manufacturing sources.
if(CONFIG_QEMU)
  set(mfg_board qemu)
elseif(CONFIG_BOARD_ASTERIX)
  set(mfg_board asterix)
elseif(CONFIG_BOARD_OBELIX)
  set(mfg_board obelix)
elseif(CONFIG_BOARD_GETAFIX)
  set(mfg_board getafix)
else()
  message(FATAL_ERROR "No MFG configuration for board ${PBL_BOARD}")
endif()

file(GLOB sources CONFIGURE_DEPENDS *.c ${mfg_board}/*.c)
# Factory mode only goes into the recovery firmware.
if(CONFIG_RECOVERY_FW)
  file(GLOB mfg_mode_sources CONFIGURE_DEPENDS mfg_mode/*.c)
  list(APPEND sources ${mfg_mode_sources})
endif()

pbl_library()
pbl_library_sources(${sources})
