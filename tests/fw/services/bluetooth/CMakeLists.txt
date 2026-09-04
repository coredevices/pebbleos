# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_ble_hrm
  SOURCES
    src/fw/services/bluetooth/ble_hrm.c
    tests/fakes/fake_events.c
  DEFINES CONFIG_HRM
)

pbl_clar_test(test_local_addr
  SOURCES
    src/fw/services/bluetooth/local_addr.c
)

pbl_clar_test(test_bluetooth_persistent_storage_prf
  SOURCES
    src/fw/services/bluetooth/bluetooth_persistent_storage_prf.c
    src/fw/services/shared_prf_storage/shared_prf_storage.c
    tests/fakes/fake_flash_region.c
    tests/fakes/fake_spi_flash.c
    tests/fakes/fake_events.c
  PLATFORMS obelix
  OVERRIDES shared_prf_storage_v3
)

pbl_clar_test(test_bluetooth_persistent_storage
  SOURCES
    src/fw/services/bluetooth/bluetooth_persistent_storage_normal.c
    src/fw/services/settings/settings_file.c
    src/fw/services/settings/settings_raw_iter.c
    src/fw/services/filesystem/pfs.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/flash_region/flash_region.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    src/fw/system/hexdump.c
    tests/fakes/fake_shared_prf_storage.c
    tests/fakes/fake_events.c
    tests/fakes/fake_spi_flash.c
    tests/fakes/fake_rtc.c
  OVERRIDES dummy_board bluetooth_persistent_storage_v2
)
