# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  ble_bas.c
  bluetooth_ctl.c
  bluetooth_persistent_storage_debug.c
  bluetooth_prompt.c
  bonding.c
  dis.c
  local_addr.c
  local_id.c
  pairability.c
  pebble_pairing_service.c
  pp_ble_control.c
)
if(NOT CONFIG_RECOVERY_FW)
  list(APPEND sources bluetooth_persistent_storage_normal.c)
  if(CONFIG_HRM)
    list(APPEND sources ble_hrm.c)
  endif()
else()
  list(APPEND sources bluetooth_persistent_storage_prf.c)
endif()
pbl_library()
pbl_library_sources(${sources})
