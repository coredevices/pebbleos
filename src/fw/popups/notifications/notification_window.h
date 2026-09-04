/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>

#include "applib/fonts/fonts.h"
#include "pbl/services/notifications/notifications.h"
#include "kernel/events.h"

void notification_window_service_init(void);

void notification_window_init(bool is_modal);

void notification_window_show(void);

bool notification_window_is_modal(void);

void notification_window_handle_notification(PebbleSysNotificationEvent *e);

void notification_window_handle_reminder(PebbleReminderEvent *e);

void notification_window_handle_dnd_event(PebbleDoNotDisturbEvent *e);

void notification_window_add_notification_by_id(Uuid *id);

void notification_window_focus_notification(Uuid *id, bool animated);

void notification_window_mark_focused_read(void);

void app_notification_window_add_new_notification_by_id(Uuid *id);

void app_notification_window_remove_notification_by_id(Uuid *id);

void app_notification_window_handle_notification_acted_upon_by_id(Uuid *id);
