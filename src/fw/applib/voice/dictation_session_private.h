/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "voice_window.h"
#include "dictation_session.h"
#include "applib/event_service_client.h"

#include <stdbool.h>

struct DictationSession {
  VoiceWindow *voice_window;
  DictationSessionStatusCallback callback;
  void *context;
  bool in_progress;
  bool destroy_pending;
  EventServiceInfo dictation_result_sub;
  EventServiceInfo app_focus_sub;
};
