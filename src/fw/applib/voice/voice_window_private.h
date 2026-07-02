/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "loading_layer.h"
#include "transcription_dialog.h"

#include "applib/app_timer.h"
#include "applib/event_service_client.h"
#include "applib/ui/animation.h"
#include "applib/ui/layer.h"
#include "applib/ui/property_animation.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/status_bar_layer.h"
#include "applib/ui/window.h"
#include "applib/ui/dialogs/bt_conn_dialog.h"
#include "applib/ui/dialogs/simple_dialog.h"
#include "applib/ui/dialogs/expandable_dialog.h"
#include "applib/ui/kino/kino_layer.h"
#include "applib/voice/dictation_session.h"
#include "pbl/services/voice/voice_recording.h"

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  StateStart,                   // Start state. Nothing happens
  StateStartWaitForReady,       // Dot flies in
  StateWaitForReady,            // Progress bar shows and animates, dot pulses
  StateRecording,               // Microphone unfolds and text appears
  StateStopRecording,           // Microphone folds up again and text disappears
  StateWaitForResponse,         // Dot pulses, progress bar shown
  StateStopWaitForResponse,     // Progress bar shrinks
  StateTransitionToText,        // Dot flies out, text window pushed
  StateError,
  StateFinished,
  StateExiting,
} VoiceUiState;

typedef struct VoiceUiData {
  struct {
    Window window;
    KinoLayer icon_layer;
    Animation *mic_dot_anim;
    Layer mic_dot_layer;
    int16_t mic_dot_radius;
    TextLayer text_layer;
    char text_buffer[20];       // Larger than needed because i18n
    StatusBarLayer status_bar;
    LoadingLayer progress_bar;
    PropertyAnimation *progress_anim;
    PropertyAnimation *fly_anim;
  } mic_window;

  union{
    TranscriptionDialog transcription_dialog;
    ExpandableDialog long_error_dialog;
    SimpleDialog short_error_dialog;
    BtConnDialog bt_dialog;
    Dialog dialog;
  };

  VoiceUiState state;
  bool speech_detected;
  bool transcription_dialog_keep_alive_on_select;
  char *message;
  size_t message_len;
  time_t timestamp;
  uint8_t error_count;
  bool last_session_successful;
  uint8_t num_sessions;
  AppTimer *dictation_timeout;
  EventServiceInfo voice_event_sub;
  DictationSessionStatus error_exit_status;

  char error_text_buffer[150];

  // For API access
  size_t buffer_size;
  bool show_confirmation_dialog;
  bool show_error_dialog;

  // Used to keep track of total elapsed time of transcriptions
  uint64_t start_ms;
  uint64_t elapsed_ms;

  VoiceSessionId session_id;
  VoiceEndpointSessionType session_type;

  // When non-zero, audio is streamed from this stored recording for transcription instead of the
  // microphone: the mic capture phase is skipped and the progress bar tracks the real upload.
  VoiceRecordingId recording_id;

  // Tag carried by the PEBBLE_DICTATION_EVENTs this window emits, so subscribers can tell
  // concurrent voice windows apart.
  uint8_t event_id;
} VoiceUiData;

void voice_window_lose_focus(VoiceWindow *voice_window);

void voice_window_regain_focus(VoiceWindow *voice_window);

void voice_window_transcription_dialog_keep_alive_on_select(VoiceWindow *voice_window,
                                                            bool keep_alive_on_select);
