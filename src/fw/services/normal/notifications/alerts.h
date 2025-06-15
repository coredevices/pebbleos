/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <stdbool.h>

#include "services/normal/notifications/notification_types.h"

typedef enum AlertType {
  AlertInvalid   = NotificationInvalid,
  AlertMobile    = NotificationMobile,
  AlertPhoneCall = NotificationPhoneCall,
  AlertOther     = NotificationOther,
  AlertReminder  = NotificationReminder
} AlertType;

// Service to determine how and if the user gets alerted on a call/notification

//! Call this function before alerting the user in any notification/call for the alerts service
//! to handle analytics operations.
void alerts_incoming_alert_analytics();

bool alerts_should_notify_for_type(AlertType type);

bool alerts_should_enable_backlight_for_type(AlertType type);

bool alerts_should_vibrate_for_type(AlertType type);

//! When vibrating for an incoming notification, call this function to prevent multiple vibes
//! within a short period of time.
void alerts_set_notification_vibe_timestamp();

//! Get whether delayed vibration for notifications is enabled
bool alerts_get_delayed_vibration(void);

//! Set whether delayed vibration for notifications is enabled
void alerts_set_delayed_vibration(bool enable);
