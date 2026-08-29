# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BOARD_ASTERIX)
  set(sources boards/board_asterix.c)
elseif(CONFIG_BOARD_OBELIX)
  set(sources boards/board_obelix.c)
elseif(CONFIG_BOARD_GETAFIX)
  set(sources boards/board_getafix.c displays/display_getafix.c)
elseif(CONFIG_BOARD_QEMU_EMERY)
  set(sources boards/board_qemu_emery.c)
elseif(CONFIG_BOARD_QEMU_FLINT)
  set(sources boards/board_qemu_flint.c)
elseif(CONFIG_BOARD_QEMU_GABBRO)
  # qemu_gabbro reuses the getafix round display row info data
  set(sources boards/board_qemu_gabbro.c displays/display_getafix.c)
else()
  message(FATAL_ERROR "Unknown board ${PBL_BOARD}")
endif()

pbl_library()
pbl_library_sources(${sources})
