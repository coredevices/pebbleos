# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_mfg_serials
  SOURCES
    src/fw/mfg/mfg_serials.c
    tests/fakes/fake_otp.c
  PLATFORMS obelix
  OVERRIDES dummy_board
)
