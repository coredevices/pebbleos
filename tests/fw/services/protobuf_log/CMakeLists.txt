# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_protobuf_log
  SOURCES
    src/fw/services/protobuf_log/protobuf_log.c
    src/fw/services/protobuf_log/protobuf_log_hr.c
    src/fw/services/protobuf_log/protobuf_log_activity_sessions.c
    src/fw/services/protobuf_log/protobuf_log_test.c
    src/fw/services/protobuf_log/protobuf_log_util.c
    src/fw/util/time/mktime.c
    src/fw/util/time/time.c
    tests/fakes/fake_rtc.c
    tests/fw/services/protobuf_log/protobuf_log_test_helpers.c
  LIBS nanopb proto_schemas
)
