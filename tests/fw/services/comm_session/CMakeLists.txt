# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_meta_endpoint
  SOURCES
    src/fw/services/comm_session/meta_endpoint.c
    tests/fakes/fake_session.c
)

pbl_clar_test(test_session
  SOURCES
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    src/fw/services/comm_session/session.c
    tests/fakes/fake_session_send_buffer.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_session_send_buffer
  SOURCES
    src/fw/services/comm_session/default_kernel_sender.c
    src/fw/services/comm_session/session_send_queue.c
    tests/fakes/fake_rtc.c
)

pbl_clar_test(test_session_send_queue
  SOURCES
    src/fw/services/comm_session/session_send_queue.c
)

pbl_clar_test(test_session_receive_router
  SOURCES
    src/fw/util/rand/rand.c
    third_party/tinymt/TinyMT/tinymt/tinymt32.c
    src/fw/services/comm_session/session.c
    src/fw/services/comm_session/session_receive_router.c
    tests/fakes/fake_session_send_buffer.c
  OVERRIDES dummy_board pp_endpoints
)

pbl_clar_test(test_session_remote_version
  SOURCES
    src/fw/services/comm_session/session_remote_version.c
    tests/fakes/fake_events.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_default_kernel_receiver
  SOURCES
    src/fw/services/comm_session/default_kernel_receiver.c
  OVERRIDES dummy_board
)
