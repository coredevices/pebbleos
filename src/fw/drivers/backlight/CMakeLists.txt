# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources console.c)
if(CONFIG_BACKLIGHT_AW2016)
  list(APPEND sources aw2016.c)
elseif(CONFIG_BACKLIGHT_AW9364E)
  list(APPEND sources aw9364e.c)
elseif(CONFIG_BACKLIGHT_PWM)
  list(APPEND sources pwm.c)
elseif(CONFIG_BACKLIGHT_QEMU)
  list(APPEND sources qemu.c)
endif()
pbl_library()
pbl_library_sources(${sources})
