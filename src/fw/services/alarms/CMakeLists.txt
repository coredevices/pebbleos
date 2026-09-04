# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  alarm.c
  alarm_pin.c
  alarm_tones.c
)
pbl_library()
pbl_library_sources(${sources})
pbl_service_gettext(alarms ${sources})
