# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_library()
pbl_library_sources(
  service.c
  util.c
)
pbl_service_gettext(phone_call service.c util.c)
