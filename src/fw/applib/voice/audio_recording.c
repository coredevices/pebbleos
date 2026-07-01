/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "audio_recording.h"

#include "syscall/syscall.h"

#ifdef CONFIG_MIC
#include "dictation_session.h"
#include "voice_window.h"
#include "voice_window_private.h"

#include "applib/applib_malloc.auto.h"
#include "applib/event_service_client.h"
#include "process_management/app_install_manager.h"
#include <pbl/logging/logging.h>
#endif

AudioRecordingId audio_recording_start(void) {
#ifdef CONFIG_MIC
  return sys_audio_recording_start();
#else
  return AUDIO_RECORDING_ID_INVALID;
#endif
}

void audio_recording_stop(AudioRecordingId recording_id) {
#ifdef CONFIG_MIC
  sys_audio_recording_stop(recording_id);
#endif
}

void audio_recording_cancel(AudioRecordingId recording_id) {
#ifdef CONFIG_MIC
  sys_audio_recording_cancel(recording_id);
#endif
}

bool audio_recording_is_active(void) {
#ifdef CONFIG_MIC
  return sys_audio_recording_is_active();
#else
  return false;
#endif
}

bool audio_recording_play(AudioRecordingId recording_id) {
#ifdef CONFIG_MIC
  return sys_audio_recording_play(recording_id);
#else
  return false;
#endif
}

void audio_recording_stop_playback(void) {
#ifdef CONFIG_MIC
  sys_audio_recording_stop_playback();
#endif
}

bool audio_recording_is_playing(void) {
#ifdef CONFIG_MIC
  return sys_audio_recording_is_playing();
#else
  return false;
#endif
}

#ifdef CONFIG_MIC

// One-shot transcription of a stored recording. Owns a headless-ish voice window (recording mode)
// and tears everything down once the result event fires.
typedef struct AudioTranscription {
  VoiceWindow *voice_window;
  AudioTranscriptionCallback callback;
  void *context;
  AudioRecordingId recording_id;
  EventServiceInfo result_sub;
  EventServiceInfo app_focus_sub;
} AudioTranscription;

static void prv_transcription_destroy(AudioTranscription *transcription) {
  event_service_client_unsubscribe(&transcription->result_sub);
  if (pebble_task_get_current() == PebbleTask_App) {
    event_service_client_unsubscribe(&transcription->app_focus_sub);
  }
  voice_window_destroy(transcription->voice_window);  // also frees the transcription buffer
  applib_free(transcription);
}

static void prv_handle_transcription_result(PebbleEvent *e, void *context) {
  AudioTranscription *transcription = context;
  transcription->callback(transcription->recording_id, e->dictation.result, e->dictation.text,
                          transcription->context);
  prv_transcription_destroy(transcription);
}

static void prv_app_focus_handler(PebbleEvent *e, void *context) {
  AudioTranscription *transcription = context;
  if (e->app_focus.in_focus) {
    event_service_client_subscribe(&transcription->result_sub);
    voice_window_regain_focus(transcription->voice_window);
  } else {
    event_service_client_unsubscribe(&transcription->result_sub);
    voice_window_lose_focus(transcription->voice_window);
  }
}

#endif  // CONFIG_MIC

bool audio_recording_transcribe(AudioRecordingId recording_id, uint32_t buffer_size,
                                AudioTranscriptionCallback callback, void *context) {
#ifdef CONFIG_MIC
  if (!callback || (recording_id == AUDIO_RECORDING_ID_INVALID)) {
    return false;
  }

  // App-initiated transcription needs a phone that advertises voice API support (same requirement
  // as app-initiated dictation).
  const bool from_app = (pebble_task_get_current() == PebbleTask_App) &&
      !app_install_id_from_system(sys_process_manager_get_current_process_id());
  if (from_app && !sys_system_pp_has_capability(CommSessionVoiceApiSupport)) {
    PBL_LOG_WRN("Phone does not support app-initiated voice sessions");
    return false;
  }

  AudioTranscription *transcription = applib_malloc(sizeof(AudioTranscription));
  if (!transcription) {
    return false;
  }

  char *buffer = NULL;
  if (buffer_size > 0) {
    buffer = applib_malloc(buffer_size);
    if (!buffer) {
      applib_free(transcription);
      return false;
    }
  }

  VoiceWindow *voice_window = voice_window_create_for_recording(buffer, buffer_size, recording_id);
  if (!voice_window) {
    applib_free(buffer);
    applib_free(transcription);
    return false;
  }

  *transcription = (AudioTranscription) {
    .voice_window = voice_window,
    .callback = callback,
    .context = context,
    .recording_id = recording_id,
    .result_sub = (EventServiceInfo) {
      .type = PEBBLE_DICTATION_EVENT,
      .handler = prv_handle_transcription_result,
      .context = transcription,
    },
  };
  if (pebble_task_get_current() == PebbleTask_App) {
    transcription->app_focus_sub = (EventServiceInfo) {
      .type = PEBBLE_APP_DID_CHANGE_FOCUS_EVENT,
      .handler = prv_app_focus_handler,
      .context = transcription,
    };
  }

  if (voice_window_push(voice_window) != DictationSessionStatusSuccess) {
    voice_window_destroy(voice_window);  // also frees the buffer
    applib_free(transcription);
    return false;
  }

  event_service_client_subscribe(&transcription->result_sub);
  if (pebble_task_get_current() == PebbleTask_App) {
    event_service_client_subscribe(&transcription->app_focus_sub);
  }
  return true;
#else
  return false;
#endif
}
