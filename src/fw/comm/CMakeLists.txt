# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(excl)
if(NOT CONFIG_BT_ANCS_CLIENT)
  list(APPEND excl ble/kernel_le_client/ancs/*.c)
endif()
if(NOT CONFIG_BT_AMS_CLIENT)
  list(APPEND excl ble/kernel_le_client/ams/*.c)
endif()
pbl_library()
file(GLOB_RECURSE sources CONFIGURE_DEPENDS *.c)
pbl_filter_sources(sources EXCLUDE ${excl})
pbl_library_sources(${sources})
