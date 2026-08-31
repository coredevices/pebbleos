# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_ble_ad_parse
  SOURCES
    src/fw/applib/bluetooth/ble_ad_parse.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    src/fw/system/hexdump.c
    src/fw/util/buffer.c
    tests/fakes/fake_rtc.c
)

pbl_clar_test(test_ble_ibeacon
  SOURCES
    src/fw/applib/bluetooth/ble_ibeacon.c
    src/fw/applib/bluetooth/ble_ad_parse.c
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    tests/fakes/fake_rtc.c
)
