# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(demo)
add_subdirectory(system)
if(CONFIG_SHELL_SDK)
  add_subdirectory(sdk)
else()
  add_subdirectory(watch)
endif()
# Every app's translatable strings, whether or not this configuration
# builds the app. The recovery-only apps ship in their own firmware
# variant, so they are left out.
file(GLOB_RECURSE pot_sources CONFIGURE_DEPENDS *.c)
pbl_filter_sources(pot_sources EXCLUDE prf/**)
pbl_gettext(${CMAKE_CURRENT_BINARY_DIR}/apps.pot ${pot_sources})
