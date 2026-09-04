# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_library_named(watch_app_kickstart)
pbl_library_sources(kickstart/kickstart.c)
pbl_library_include_directories(.)
pbl_library_named(watch_app_low_power)
pbl_library_sources(low_power/face.c)
pbl_library_include_directories(.)
add_subdirectory(tictoc)
