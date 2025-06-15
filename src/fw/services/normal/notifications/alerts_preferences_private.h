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

#include "services/normal/notifications/alerts_private.h"
#include "services/normal/notifications/do_not_disturb.h"
#include "services/normal/notifications/notification_types.h"
#include "services/normal/vibes/vibe_intensity.h"
#include "util/time/time.h"

#if CAPABILITY_HAS_VIBE_SCORES
#include "services/normal/vibes/vibe_client.h"
#include "services/normal/vibes/vibe_score_info.h"
#endif

#define NOTIF_WINDOW_TIMEOUT_INFINITE ((uint32_t)~0)
#define NOTIF_WINDOW_TIMEOUT_DEFAULT (3 * MS_PER_MINUTE)

void alerts_preferences_init(void);

AlertMask alerts_preferences_get_alert_mask(void);

void alerts_preferences_set_alert_mask(AlertMask mask);

AlertMask alerts_preferences_dnd_get_mask(void);

void alerts_preferences_dnd_set_mask(AlertMask mask);

uint32_t alerts_preferences_get_notification_window_timeout_ms(void);

void alerts_preferences_set_notification_window_timeout_ms(uint32_t timeout_ms);

bool alerts_preferences_get_vibrate(void);

void alerts_preferences_set_vibrate(bool enable);

VibeIntensity alerts_preferences_get_vibe_intensity(void);

void alerts_preferences_set_vibe_intensity(VibeIntensity intensity);

#if CAPABILITY_HAS_VIBE_SCORES
VibeScoreId alerts_preferences_get_vibe_score_for_client(VibeClient client);

void alerts_preferences_set_vibe_score_for_client(VibeClient client, VibeScoreId id);
#endif

bool alerts_preferences_dnd_is_manually_enabled(void);

void alerts_preferences_dnd_set_manually_enabled(bool enable);

void alerts_preferences_dnd_get_schedule(DoNotDisturbScheduleType type,
                                         DoNotDisturbSchedule *schedule_out);

void alerts_preferences_dnd_set_schedule(DoNotDisturbScheduleType type,
                                         const DoNotDisturbSchedule *schedule);

bool alerts_preferences_dnd_is_schedule_enabled(DoNotDisturbScheduleType type);

void alerts_preferences_dnd_set_schedule_enabled(DoNotDisturbScheduleType type, bool enable);

bool alerts_preferences_dnd_is_smart_enabled(void);

void alerts_preferences_dnd_set_smart_enabled(bool enable);

bool alerts_preferences_get_delayed_vibration(void);

void alerts_preferences_set_delayed_vibration(bool enable);
