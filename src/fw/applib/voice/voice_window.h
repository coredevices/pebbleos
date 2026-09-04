/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/util/uuid.h"
#include "applib/voice/dictation_session.h"
#include "pbl/services/voice_endpoint.h"

#include <stddef.h>
#include <stdbool.h>

typedef struct VoiceUiData VoiceWindow;

VoiceWindow *voice_window_create(char *buffer, size_t buffer_size,
                                 VoiceEndpointSessionType session_type);

void voice_window_destroy(VoiceWindow *voice_window);

// Push the voice window from App task or Main task
DictationSessionStatus voice_window_push(VoiceWindow *voice_window);

void voice_window_pop(VoiceWindow *voice_window);

void voice_window_set_confirmation_enabled(VoiceWindow *voice_window, bool enabled);

void voice_window_set_error_enabled(VoiceWindow *voice_window, bool enabled);

void voice_window_reset(VoiceWindow *voice_window);
