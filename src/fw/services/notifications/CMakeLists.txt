# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources
  action_chaining_window.c
  alerts.c
  alerts_preferences.c
  ancs/ancs_filtering.c
  ancs/ancs_item.c
  ancs/ancs_notifications.c
  ancs/ancs_notifications_util.c
  ancs/ancs_phone_call.c
  ancs/nexmo.c
  do_not_disturb.c
  do_not_disturb_toggle.c
  notification_image.c
  notification_storage.c
  notifications.c
)
pbl_library()
pbl_library_sources(${sources})
pbl_service_gettext(notifications ${sources})
