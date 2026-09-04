# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_settings_file
  SOURCES
    src/fw/util/dict.c
    src/fw/services/filesystem/flash_translation.c
    src/fw/services/filesystem/pfs.c
    tests/fakes/fake_spi_flash.c
    src/fw/util/crc8.c
    src/fw/util/legacy_checksum.c
    src/fw/flash_region/filesystem_regions.c
    src/fw/flash_region/flash_region.c
    tests/fakes/fake_rtc.c
    src/fw/system/hexdump.c
    src/fw/services/settings/settings_file.c
    src/fw/services/settings/settings_raw_iter.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
  DEFINES DUMA_DISABLED
  OVERRIDES dummy_board
)
