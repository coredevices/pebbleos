# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources)
if(CONFIG_PMIC_NPM1300)
  list(APPEND sources npm1300.c)
endif()
pbl_library()
pbl_library_sources(${sources})
