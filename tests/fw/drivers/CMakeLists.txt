# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_qemu_serial
  SOURCES
    src/fw/drivers/qemu/qemu_serial_util.c
    src/fw/util/shared_circular_buffer.c
    tests/fakes/fake_rtc.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_flash_api
  SOURCES
    src/fw/drivers/flash/flash_api.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_flash_erase
  SOURCES
    src/fw/drivers/flash/flash_erase.c
  PLATFORMS obelix
  OVERRIDES dummy_board
)
