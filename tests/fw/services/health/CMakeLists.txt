# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_health
  SOURCES
    src/fw/applib/health_service.c
    src/fw/process_management/pebble_process_info.c
    src/fw/util/time/mktime.c
    src/fw/util/time/time.c
    src/fw/util/stats.c
    tests/fakes/fake_rtc.c
  DEFINES CAPABILITY_HAS_HEALTH_TRACKING=1
  OVERRIDES dummy_board
)
