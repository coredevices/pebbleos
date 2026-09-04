# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  app_message_receiver.c
  app_message_sender.c
)
pbl_library()
pbl_library_sources(${sources})
