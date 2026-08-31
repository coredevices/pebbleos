# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_app_message_receiver
  SOURCES
    src/fw/applib/app_inbox.c
    src/fw/applib/app_message/app_message_receiver.c
    src/fw/services/app_message/app_message_receiver.c
    src/fw/services/app_inbox_service/service.c
  OVERRIDES applib_malloc
)

pbl_clar_test(test_app_message_sender
  SOURCES
    src/fw/services/app_message/app_message_sender.c
    src/fw/services/comm_session/session_send_queue.c
)

pbl_clar_test(test_app_session_capabilities
  SOURCES
    src/fw/services/comm_session/app_session_capabilities.c
)
