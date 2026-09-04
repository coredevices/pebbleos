# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  protobuf_log.c
  protobuf_log_activity_sessions.c
  protobuf_log_hr.c
  protobuf_log_test.c
  protobuf_log_util.c
)
pbl_library()
pbl_library_sources(${sources})
