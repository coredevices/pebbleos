# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_activity
  SOURCES
    src/fw/applib/health_service.c
    src/fw/flash_region/flash_region.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/util/base64.c
    src/fw/util/stats.c
    src/fw/util/time/time.c
    src/fw/util/time/mktime.c
    src/fw/services/regular_timer/service.c
    src/fw/services/activity/activity.c
    src/fw/services/activity/activity_metrics.c
    src/fw/services/activity/activity_sessions.c
    src/fw/services/activity/activity_calculators.c
    src/fw/services/activity/hr_util.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/settings/settings_file.c
    src/fw/services/settings/settings_raw_iter.c
    src/fw/shell/normal/prefs.c
    tests/stubs/stubs_i18n.c
    tests/fakes/fake_accel_service.c
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    tests/fakes/fake_events.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_spi_flash.c
  DEFINES DUMA_DISABLED CAPABILITY_HAS_HEALTH_TRACKING=1 CONFIG_HRM CONFIG_SERVICE_ACTIVITY_BATCH_SAMPLES=25
  OVERRIDES dummy_board
)

pbl_clar_test(test_activity_insights
  SOURCES
    src/fw/util/base64.c
    src/fw/util/crc8.c
    src/fw/util/stats.c
    src/fw/util/time/time.c
    src/fw/util/time/mktime.c
    src/fw/services/activity/activity_insights.c
    src/fw/services/activity/insights_settings.c
    src/fw/services/timeline/metricgroup.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    tests/fakes/fake_clock.c
    tests/fakes/fake_settings_file.c
    tests/fakes/fake_kernel_services_notifications.c
    tests/fakes/fake_rtc.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_kraepelin_algorithm
  SOURCES
    src/fw/services/activity/kraepelin/kraepelin_algorithm.c
    tests/fakes/fake_rtc.c
    tests/fw/services/activity/step_samples.c
    tests/fw/services/activity/sleep_samples_v1.c
    tests/fw/services/activity/kraepelin_reference/fourier.c
    tests/fw/services/activity/kraepelin_reference/helper_worker.c
    tests/fw/services/activity/kraepelin_reference/ProjectK_worker.c
    tests/fw/services/activity/kraepelin_reference/raw_stats.c
  DEFINES DUMA_DISABLED CONFIG_HRM
  OVERRIDES dummy_board
)

pbl_clar_test(test_activity_algorithm_kraepelin
  SOURCES
    src/fw/flash_region/flash_region.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/util/base64.c
    src/fw/util/crc8.c
    src/fw/util/shared_circular_buffer.c
    src/fw/util/time/time.c
    src/fw/util/time/mktime.c
    src/fw/services/regular_timer/service.c
    src/fw/services/activity/kraepelin/activity_algorithm_kraepelin.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/settings/settings_file.c
    src/fw/services/settings/settings_raw_iter.c
    tests/fakes/fake_accel_service.c
    src/fw/util/legacy_checksum.c
    tests/fakes/fake_events.c
    tests/fakes/fake_rtc.c
    tests/fakes/fake_spi_flash.c
  DEFINES DUMA_DISABLED
  OVERRIDES dummy_board
)

pbl_clar_test(test_health_util
  SOURCES
    src/fw/services/activity/health_util.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_hr_util
  SOURCES
    src/fw/services/activity/hr_util.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_workout_service
  SOURCES
    src/fw/services/activity/activity_calculators.c
    src/fw/services/activity/hr_util.c
    src/fw/services/activity/workout_service.c
    tests/fakes/fake_events.c
    tests/fakes/fake_rtc.c
  DEFINES CONFIG_HRM
  OVERRIDES dummy_board
)

pbl_clar_test(test_activity_calculators
  SOURCES
    src/fw/services/activity/activity_calculators.c
  OVERRIDES dummy_board
)
