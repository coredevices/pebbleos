# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_test_infra
  SOURCES
    tests/test_infra/header_overrides.c
  OVERRIDES test_infra_test
)
