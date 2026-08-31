# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_compositor
  SOURCES
    src/fw/applib/graphics/gtypes.c
    src/fw/services/compositor/compositor.c
    tests/stubs/stubs_modal_manager.c
    tests/stubs/stubs_app_state.c
  OVERRIDES dummy_board
)
