# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_cobs_decode
  SOURCES
    src/fw/console/cobs.c
)

pbl_clar_test(test_cobs_encode
  SOURCES
    src/fw/console/cobs.c
)
