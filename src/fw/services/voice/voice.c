/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/voice/voice.h"

#include "bluetooth/responsiveness.h"
#include "board/board.h"
#include <pbl/drivers/mic.h>
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "pbl/os/mutex.h"
#include "process_management/app_install_manager.h"
#include "process_management/app_manager.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/system_task.h"
#include "pbl/services/audio_endpoint.h"
#include "pbl/services/voice/transcription.h"
#include "pbl/services/voice/voice_recording.h"
#include "pbl/services/voice/voice_speex.h"
#include "pbl/services/voice_endpoint.h"
#include "syscall/syscall_internal.h"
#include "voice_recording_storage.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "system/profiler.h"
#include "pbl/util/likely.h"
#include "pbl/util/uuid.h"

#include <string.h>

PBL_LOG_MODULE_DEFINE(service_voice, CONFIG_SERVICE_VOICE_LOG_LEVEL);

#define SPEEX_BITSTREAM_VERSION (4)

#define TIMEOUT_SESSION_SETUP (8000)
#define TIMEOUT_SESSION_RESULT  (15000)

// When streaming a stored recording for transcription, feed encoded frames at ~real-time
// (each frame holds 20 ms of audio) so the audio endpoint send buffer never overflows and
// drops frames, which would corrupt the transcription.
#define VOICE_REC_FEED_MS (40)
#define VOICE_REC_FRAMES_PER_FEED (2)

typedef enum {
  SessionState_Idle = 0,
  SessionState_StartSession,
  SessionState_VoiceEndpointSetupReceived,
  SessionState_AudioEndpointSetupReceived,
  SessionState_Recording,
  SessionState_WaitForSessionResult,
} SessionState;

static SessionState s_state = SessionState_Idle;

static PebbleMutex* s_lock = NULL;

// Handle requests from apps
static bool s_from_app;
static Uuid s_app_uuid;

static AudioEndpointSessionId s_session_id = AUDIO_ENDPOINT_SESSION_INVALID_ID;
static TimerID s_timeout = TIMER_INVALID_ID;

// Session generation & teardown guards to mitigate race conditions between explicit cancel
// and timeout callbacks executing on the timer thread.
static uint32_t s_session_generation = 0;      // Monotonic session counter
static uint32_t s_timeout_generation = 0;      // Generation tied to currently scheduled timeout
static bool s_teardown_in_progress = false;    // Debounce concurrent teardown paths

// Audio source for the current session: the live microphone (default), or a stored recording
// streamed to the phone for transcription.
typedef enum {
  VoiceAudioSource_Mic = 0,
  VoiceAudioSource_Recording,
} VoiceAudioSource;

static VoiceAudioSource s_audio_source;
static int s_rec_fd = -1;          // payload descriptor of the recording being streamed
static VoiceRecordingId s_rec_id = VOICE_RECORDING_ID_INVALID;  // valid while s_rec_fd is open
static uint32_t s_rec_total;       // total encoded payload bytes to stream
static uint32_t s_rec_sent;        // encoded payload bytes streamed so far
static uint8_t s_rec_last_pct;     // last reported upload progress (0-100)
static TimerID s_feed_timer = TIMER_INVALID_ID;

static void prv_send_event(VoiceEventType event_type, VoiceStatus status,
                           PebbleVoiceServiceEventData *data);
static void prv_send_progress_event(uint8_t progress);
static void prv_session_result_timeout(void * data);
static void prv_feed_recording(void *unused);

static void prv_audio_data_handler(int16_t *samples, size_t sample_count, void *context) {
  if (!voice_speex_is_initialized()) {
    PBL_LOG_DBG("Speex not initialized, dropping audio data");
    return;
  }

  if (s_state != SessionState_Recording) {
    PBL_LOG_DBG("Not recording, dropping audio data");
    return;
  }

  // Additional safety check - ensure we have a valid session
  if (s_session_id == AUDIO_ENDPOINT_SESSION_INVALID_ID) {
    return;
  }

  // Ensure we have the right amount of data for a frame
  size_t expected_samples = voice_speex_get_frame_size();
  if (sample_count != expected_samples) {
    PBL_LOG_DBG("Unexpected audio sample count: got %zu, expected %zu", sample_count, expected_samples);
    return;
  }

  // Encode the audio frame
  uint8_t encoded_buffer[VOICE_SPEEX_MAX_ENCODED_FRAME_SIZE];
  int encoded_bytes = voice_speex_encode_frame(samples, encoded_buffer, sizeof(encoded_buffer));
  
  if (encoded_bytes > 0) {
    // Send encoded data to audio endpoint
    audio_endpoint_add_frame(s_session_id, encoded_buffer, encoded_bytes);
  } else {
    PBL_LOG_DBG("Failed to encode audio frame");
  }
}

static void prv_teardown_session(void) {
  // Reset communication session responsiveness to default after voice session ends
  CommSession *comm_session = comm_session_get_system_session();
  if (comm_session) {
    comm_session_set_responsiveness(comm_session, BtConsumerPpVoiceEndpoint, ResponseTimeMax, 0);
  }
}

// Stop streaming a stored recording: cancel the feed timer and release the payload descriptor.
// Safe to call from within prv_feed_recording (stopping the currently-firing timer is a no-op).
static void prv_stop_feed(void) {
  if (s_feed_timer != TIMER_INVALID_ID) {
    new_timer_stop(s_feed_timer);
  }
  if (s_rec_fd >= 0) {
    pfs_close(s_rec_fd);
    s_rec_fd = -1;
    s_rec_id = VOICE_RECORDING_ID_INVALID;
  }
}

// Stop the active audio source (mic or recording feed) without touching the audio endpoint.
static void prv_stop_audio_source(void) {
  if (s_audio_source == VoiceAudioSource_Recording) {
    prv_stop_feed();
  } else {
    mic_stop(MIC);
  }
}

static void prv_stop_recording(void) {
  PBL_LOG_DBG("prv_stop_recording called - stopping audio source and endpoint transfer");

  // First, set state to non-recording to prevent any new audio processing
  s_state = SessionState_WaitForSessionResult;

  // Stop audio endpoint transfer BEFORE stopping the source
  // This prevents new frames from being added while the endpoint shuts down
  audio_endpoint_stop_transfer(s_session_id);

  prv_stop_audio_source();

  prv_teardown_session();

  // Speex cleanup will be handled by delayed cleanup to avoid race conditions
}

static void prv_cancel_recording(void) {
  PBL_LOG_DBG("prv_cancel_recording called - cancelling audio source and endpoint transfer");
  prv_stop_audio_source();

  audio_endpoint_cancel_transfer(s_session_id);
  prv_teardown_session();
}

static void prv_cancel_early_session(void) {
  // For early cancellation, only cancel the audio endpoint transfer
  // Don't call mic_stop() since the microphone was never started
  audio_endpoint_cancel_transfer(s_session_id);
  prv_teardown_session();
}

static void prv_reset(void) {
  s_state = SessionState_Idle;
  s_session_id = AUDIO_ENDPOINT_SESSION_INVALID_ID;
  if (s_rec_fd >= 0) {
    pfs_close(s_rec_fd);
    s_rec_fd = -1;
  }
  s_rec_id = VOICE_RECORDING_ID_INVALID;
  s_audio_source = VoiceAudioSource_Mic;
  s_rec_total = 0;
  s_rec_sent = 0;
  s_rec_last_pct = 0;
}

static void prv_cancel_session(void) {
  prv_cancel_recording();
  prv_reset();
}

static void prv_start_result_timeout(void) {
  new_timer_start(s_timeout, TIMEOUT_SESSION_RESULT, prv_session_result_timeout, NULL, 0);
}

static void prv_audio_transfer_stopped_handler(AudioEndpointSessionId session_id) {
  mutex_lock(s_lock);
  PBL_LOG_DBG("prv_audio_transfer_stopped_handler called with session_id=%d (current=%d)", 
              session_id, s_session_id);
  
  if (s_session_id != session_id) {
    PBL_LOG_WRN("Received audio transfer message when no session was in progress ("
            "%d)", session_id);
    mutex_unlock(s_lock);
    return;
  }

  if (s_state != SessionState_Recording) {
    PBL_LOG_WRN("Received stop message from phone after audio session "
        "stopped/cancelled");
    mutex_unlock(s_lock);
    return;
  }

  PBL_LOG_DBG("Stopping recording due to phone request");
  // TODO: Handle this better: there is no feedback to the UI that we've stopped recording
  s_state = SessionState_WaitForSessionResult;
  prv_stop_recording();
  s_timeout_generation = s_session_generation;
  prv_start_result_timeout();
  mutex_unlock(s_lock);
}

// System-task callback scheduled by prv_feed_recording once the whole payload has been fed:
// stops the transfer and starts the result timeout. The session generation guards against the
// session having ended (or been replaced) between scheduling and execution.
static void prv_finish_feed(void *data) {
  const uint32_t generation = (uint32_t)(uintptr_t)data;
  mutex_lock(s_lock);
  if ((generation != s_session_generation) || s_teardown_in_progress ||
      (s_state != SessionState_Recording) ||
      (s_audio_source != VoiceAudioSource_Recording)) {
    mutex_unlock(s_lock);
    return;
  }

  PBL_LOG_DBG("Finished streaming recording, awaiting transcription result");
  prv_stop_recording();
  s_timeout_generation = s_session_generation;
  prv_start_result_timeout();
  mutex_unlock(s_lock);
}

// Timer callback that streams the stored recording to the phone, one small batch of encoded
// frames per tick. Reports upload progress and, once the whole payload has been sent, stops the
// transfer and waits for the transcription result.
static void prv_feed_recording(void *unused) {
  mutex_lock(s_lock);
  if (s_teardown_in_progress ||
      (s_state != SessionState_Recording) ||
      (s_audio_source != VoiceAudioSource_Recording)) {
    mutex_unlock(s_lock);
    return;
  }

  bool eof = false;
  for (int i = 0; i < VOICE_REC_FRAMES_PER_FEED; i++) {
    uint8_t frame[VOICE_SPEEX_MAX_ENCODED_FRAME_SIZE];
    uint32_t remaining = s_rec_total - s_rec_sent;
    const int frame_len =
        voice_recording_storage_read_frame(s_rec_fd, &remaining, frame, sizeof(frame));
    s_rec_sent = s_rec_total - remaining;
    if (frame_len <= 0) {
      eof = true;
      break;
    }
    audio_endpoint_add_frame(s_session_id, frame, frame_len);
  }

  const uint8_t pct = (s_rec_total > 0)
                          ? (uint8_t)(((uint64_t)s_rec_sent * 100) / s_rec_total)
                          : 100;
  if (pct != s_rec_last_pct) {
    s_rec_last_pct = pct;
    prv_send_progress_event(pct);
  }

  if (eof) {
    // Stopping the transfer sends a BT message that can block for seconds when the send buffer
    // is congested (likely right after streaming the whole payload). Do it on the system task
    // instead of stalling the timer-service task with s_lock held.
    system_task_add_callback(prv_finish_feed, (void *)(uintptr_t)s_session_generation);
  } else {
    new_timer_start(s_feed_timer, VOICE_REC_FEED_MS, prv_feed_recording, NULL, 0);
  }
  mutex_unlock(s_lock);
}

static bool prv_start_recording(void) {
  PBL_LOG_DBG("prv_start_recording called");

  if (s_audio_source == VoiceAudioSource_Recording) {
    PBL_LOG_DBG("Streaming stored recording (%"PRIu32" bytes)", s_rec_total);
    if (s_feed_timer == TIMER_INVALID_ID) {
      s_feed_timer = new_timer_create();
    }
    s_rec_sent = 0;
    s_rec_last_pct = 0;
    new_timer_start(s_feed_timer, VOICE_REC_FEED_MS, prv_feed_recording, NULL, 0);
    return true;
  }

  // Start microphone with Speex frame buffer
  int16_t *frame_buffer = voice_speex_get_frame_buffer();
  size_t frame_size_samples = voice_speex_get_frame_size();  // Get frame size in samples

  PBL_LOG_DBG("Got Speex frame buffer: %p, frame_size_samples: %zu", frame_buffer, frame_size_samples);

  if (frame_buffer && frame_size_samples > 0) {
    PBL_LOG_DBG("Starting microphone with frame buffer");
    if (!mic_start(MIC, &prv_audio_data_handler, NULL, frame_buffer, frame_size_samples)) {
      PBL_LOG_ERR("Failed to start microphone for voice session");
      return false;
    }
    PBL_LOG_DBG("Microphone started successfully");
  } else {
    PBL_LOG_ERR("Invalid Speex frame buffer");
    return false;
  }

  return true;
}

static void prv_send_event(VoiceEventType event_type, VoiceStatus status,
                           PebbleVoiceServiceEventData *data) {
  PebbleEvent event = {
    .type = PEBBLE_VOICE_SERVICE_EVENT,
    .voice_service = {
      .type = event_type,
      .status = status,
      .data = data,
    }
  };
  event_put(&event);
}

static void prv_send_progress_event(uint8_t progress) {
  PebbleEvent event = {
    .type = PEBBLE_VOICE_SERVICE_EVENT,
    .voice_service = {
      .type = VoiceEventTypeSessionProgress,
      .status = VoiceStatusSuccess,
      .progress = progress,
      .data = NULL,
    }
  };
  event_put(&event);
}

//! Expects s_lock is held by caller
static void prv_handle_subsystem_started(SessionState transition_to_state) {
  PBL_LOG_DBG("prv_handle_subsystem_started called: transition_to_state=%d, current_state=%d", 
              transition_to_state, s_state);
            
  PBL_ASSERTN(transition_to_state == SessionState_VoiceEndpointSetupReceived ||
              transition_to_state == SessionState_AudioEndpointSetupReceived);

  if (s_state == SessionState_Idle) { // we error'ed out
    PBL_LOG_DBG("Session already errored out (state=Idle), ignoring subsystem start");
    return;
  }

  if (s_state == SessionState_StartSession) {
    // we are still waiting for one of the subsystems to be ready
    PBL_LOG_DBG("First subsystem ready, transitioning to state %d", transition_to_state);
    s_state = transition_to_state;
  } else {
    PBL_ASSERTN((s_state == SessionState_VoiceEndpointSetupReceived ||
                 s_state == SessionState_AudioEndpointSetupReceived) &&
                (transition_to_state != s_state));
    
    PBL_LOG_DBG("Both subsystems ready, transitioning to Recording state");
    s_state = SessionState_Recording;

    new_timer_stop(s_timeout);

    PBL_LOG_DBG("Starting recording now that both subsystems are ready");
    if (!prv_start_recording()) {
      PBL_LOG_ERR("Voice session setup failed while starting recording");
      prv_cancel_session();
      prv_send_event(VoiceEventTypeSessionSetup, VoiceStatusErrorGeneric, NULL);
      return;
    }

    // Indicate to the UI that we have started recording
    prv_send_event(VoiceEventTypeSessionSetup, VoiceStatusSuccess, NULL);
  }
}

static void prv_session_result_timeout(void * data) {
  mutex_lock(s_lock);

  if (s_teardown_in_progress || (s_timeout_generation != s_session_generation)) {
    PBL_LOG_DBG("Ignoring stale session result timeout (t_gen=%"PRIu32" cur=%"PRIu32" teardown=%d)",
                s_timeout_generation, s_session_generation, s_teardown_in_progress);
    mutex_unlock(s_lock);
    return;
  }

  PBL_ASSERTN(s_state == SessionState_WaitForSessionResult);

  prv_reset();
  PBL_LOG_WRN("Timeout waiting for session result");

  prv_send_event(VoiceEventTypeSessionResult, VoiceStatusTimeout, NULL);

  mutex_unlock(s_lock);
}

static void prv_session_setup_timeout(void * data) {
  mutex_lock(s_lock);
  if (s_teardown_in_progress || (s_timeout_generation != s_session_generation)) {
    PBL_LOG_DBG("Ignoring stale session setup timeout (t_gen=%"PRIu32" cur=%"PRIu32" teardown=%d)",
                s_timeout_generation, s_session_generation, s_teardown_in_progress);
    mutex_unlock(s_lock);
    return;
  }

  PBL_ASSERTN(s_state == SessionState_StartSession ||
              s_state == SessionState_VoiceEndpointSetupReceived ||
              s_state == SessionState_AudioEndpointSetupReceived);

  prv_cancel_session();
  PBL_LOG_WRN("Timeout waiting for session setup result ");

  prv_send_event(VoiceEventTypeSessionSetup, VoiceStatusTimeout, NULL);

  mutex_unlock(s_lock);
}

static VoiceStatus prv_get_status_from_result(VoiceEndpointResult result) {
  VoiceStatus status;
  switch (result) {
    case VoiceEndpointResultFailServiceUnavailable:
      status = VoiceStatusErrorConnectivity;
      break;
    case VoiceEndpointResultFailDisabled:
      status = VoiceStatusErrorDisabled;
      break;
    case VoiceEndpointResultFailInvalidRecognizerResponse:
      status = VoiceStatusRecognizerResponseError;
      break;
    case VoiceEndpointResultFailTimeout:
    case VoiceEndpointResultFailRecognizerError:
    case VoiceEndpointResultFailInvalidMessage:
    default:
      status = VoiceStatusErrorGeneric;
      break;
  }
  return status;
}

void voice_init(void) {
  s_lock = mutex_create();
  // Speex encoder is now initialized lazily when a dictation session starts
}

// Common session bring-up shared by the microphone and recording sources: allocates a session
// generation, sets up both endpoints, sends the setup message with the given Speex transfer info,
// and starts the setup timeout. Expects s_lock held and s_audio_source already set by the caller.
static VoiceSessionId prv_start_session(VoiceEndpointSessionType session_type,
                                        const AudioTransferInfoSpeex *transfer_info) {
  // Start new session generation and clear teardown guard
  s_session_generation++;
  if (UNLIKELY(s_session_generation == 0)) { // handle wrap-around (very unlikely)
    s_session_generation = 1;
  }
  s_teardown_in_progress = false;
  PBL_LOG_DBG("Setting state to StartSession");
  s_state = SessionState_StartSession;

  // check if we're being started from an app so we know to send the UUID when setting up a session
  s_from_app = ((pebble_task_get_current() == PebbleTask_App) &&
      !app_install_id_from_system(app_manager_get_current_app_id()));
  if (s_from_app) {
    s_app_uuid = app_manager_get_current_app_md()->uuid;
    char uuid_str[UUID_STRING_BUFFER_LENGTH];
    uuid_to_string(&s_app_uuid, uuid_str);
    PBL_LOG_DBG("Starting app-initiated voice session for app %s", uuid_str);
  } else {
    PBL_LOG_DBG("Starting system-initiated voice session");
  }

  // Set up communication session responsiveness for voice session
  CommSession *comm_session = comm_session_get_system_session();
  if (comm_session) {
    comm_session_set_responsiveness(comm_session, BtConsumerPpVoiceEndpoint, ResponseTimeMin,
                                    MIN_LATENCY_MODE_TIMEOUT_VOICE_SECS);
  }

  PBL_LOG_DBG("Setting up audio endpoint transfer");
  s_session_id = audio_endpoint_setup_transfer(prv_audio_transfer_stopped_handler);
  PBL_ASSERTN(s_session_id != AUDIO_ENDPOINT_SESSION_INVALID_ID);
  PBL_LOG_DBG("Audio endpoint transfer setup complete with session_id=%d", s_session_id);

  PBL_LOG_INFO("Send session setup message. Session type: %d", session_type);
  AudioTransferInfoSpeex info = *transfer_info;
  voice_endpoint_setup_session(session_type, s_session_id, &info,
      s_from_app ? &s_app_uuid : NULL);

  if (s_timeout == TIMER_INVALID_ID) {
    s_timeout = new_timer_create();
  }
  s_timeout_generation = s_session_generation;
  new_timer_start(s_timeout, TIMEOUT_SESSION_SETUP, prv_session_setup_timeout, NULL, 0);

  PBL_LOG_DBG("Audio transfer setup complete, handling subsystem started");
  prv_handle_subsystem_started(SessionState_AudioEndpointSetupReceived);

  return s_session_id;
}

// This will kick off a dictation session. After the setup session message is sent via the
// voice control endpoint, we wait for a session ready response via the
// voice_handle_session_setup_result call or a session setup timeout occurs (timer callback
// prv_session_setup_timeout)
VoiceSessionId voice_start_dictation(VoiceEndpointSessionType session_type) {
  PBL_LOG_DBG("voice_start_dictation called with session_type: %d", session_type);
  mutex_lock(s_lock);

  // Lazily initialize Speex encoder to avoid baseline memory usage when voice not used
  if (!voice_speex_is_initialized()) {
    if (!voice_speex_init()) {
      PBL_LOG_ERR("Failed to initialize Speex encoder");
      mutex_unlock(s_lock);
      return VOICE_SESSION_ID_INVALID;
    }
  }

  if (s_state != SessionState_Idle) {
    PBL_LOG_DBG("Voice service not idle (state: %d), returning invalid session", s_state);
    mutex_unlock(s_lock);
    return VOICE_SESSION_ID_INVALID;
  }

  // Dictation and on-device recording share the single microphone.
  if (mic_is_running(MIC)) {
    PBL_LOG_WRN("Microphone busy (recording?), cannot start dictation");
    mutex_unlock(s_lock);
    return VOICE_SESSION_ID_INVALID;
  }

  s_audio_source = VoiceAudioSource_Mic;

  AudioTransferInfoSpeex transfer_info;
  voice_speex_get_transfer_info(&transfer_info);
  PBL_LOG_DBG("Got Speex transfer info: sample_rate=%"PRIu32", bit_rate=%"PRIu16", frame_size=%"PRIu16,
              transfer_info.sample_rate, transfer_info.bit_rate, transfer_info.frame_size);

  const VoiceSessionId session_id = prv_start_session(session_type, &transfer_info);
  mutex_unlock(s_lock);
  return session_id;
}

// Kick off a transcription session sourced from a stored recording instead of the microphone.
// The stored Speex frames are streamed to the phone as-is (same protocol as live dictation); the
// result comes back through the same path (voice_handle_dictation_result).
VoiceSessionId voice_start_dictation_from_recording(VoiceRecordingId recording_id) {
  PBL_LOG_DBG("voice_start_dictation_from_recording called for id %u", (unsigned)recording_id);
  mutex_lock(s_lock);

  if (s_state != SessionState_Idle) {
    PBL_LOG_DBG("Voice service not idle (state: %d), cannot transcribe", s_state);
    mutex_unlock(s_lock);
    return VOICE_SESSION_ID_INVALID;
  }

  // Dictation, transcription and on-device recording share the single microphone/encoder.
  if (mic_is_running(MIC)) {
    PBL_LOG_WRN("Microphone busy (recording?), cannot transcribe");
    mutex_unlock(s_lock);
    return VOICE_SESSION_ID_INVALID;
  }

  VoiceRecordingStorageMetadata meta;
  if (!voice_recording_storage_get_metadata(recording_id, &meta)) {
    PBL_LOG_WRN("Recording %u not found", (unsigned)recording_id);
    mutex_unlock(s_lock);
    return VOICE_SESSION_ID_INVALID;
  }

  // Transcription (speech-to-text) is mono only.
  if (meta.channels != 1) {
    PBL_LOG_WRN("Cannot transcribe recording with %u channels", meta.channels);
    mutex_unlock(s_lock);
    return VOICE_SESSION_ID_INVALID;
  }

  uint32_t data_bytes = 0;
  const int fd = voice_recording_storage_open_payload(recording_id, &data_bytes);
  if (fd < 0) {
    PBL_LOG_WRN("Failed to open recording %u payload", (unsigned)recording_id);
    mutex_unlock(s_lock);
    return VOICE_SESSION_ID_INVALID;
  }

  s_audio_source = VoiceAudioSource_Recording;
  s_rec_fd = fd;
  s_rec_id = recording_id;
  s_rec_total = data_bytes;
  s_rec_sent = 0;
  s_rec_last_pct = 0;

  // The stored frames already use the system Speex parameters; describe them to the phone straight
  // from the recording header (no Speex encoder needed for a pre-encoded source).
  const VoiceSessionId session_id =
      prv_start_session(VoiceEndpointSessionTypeDictation, &meta.speex);
  mutex_unlock(s_lock);
  return session_id;
}

VoiceRecordingId voice_transcribing_recording_id(void) {
  mutex_lock(s_lock);
  const VoiceRecordingId id = (s_rec_fd >= 0) ? s_rec_id : VOICE_RECORDING_ID_INVALID;
  mutex_unlock(s_lock);
  return id;
}

// Calling this will end the recording, disable the mic and and stop the audio transfer session. We
// expect voice_handle_dictation_result to be called next with a dictation response
void voice_stop_dictation(VoiceSessionId session_id) {
  mutex_lock(s_lock);
  if ((s_state == SessionState_Idle) ||
      (session_id != s_session_id) ||
      (session_id == VOICE_SESSION_ID_INVALID)) {
    goto unlock;
  }

  if (s_state != SessionState_Recording) {
    mutex_unlock(s_lock);
    voice_cancel_dictation(session_id);
    return;
  }

  s_state = SessionState_WaitForSessionResult;
  prv_stop_recording();
  prv_start_result_timeout();

unlock:
  mutex_unlock(s_lock);
}

void voice_cancel_dictation(VoiceSessionId session_id) {
  mutex_lock(s_lock);
  if ((session_id != s_session_id) ||
      (session_id == VOICE_SESSION_ID_INVALID)) {
    goto unlock;
  }

  if (s_state != SessionState_Idle) {
    bool stopped = new_timer_stop(s_timeout);
    if (!stopped) {
      // Timer callback in progress or already firing; invalidate generation so callback no-ops
      s_timeout_generation = 0;
      PBL_LOG_DBG("Timeout stop race (cancel), invalidating timeout generation");
    }
    s_teardown_in_progress = true;
    if (s_state == SessionState_StartSession ||
        s_state == SessionState_VoiceEndpointSetupReceived ||
        s_state == SessionState_AudioEndpointSetupReceived) {
      prv_cancel_early_session();
    } else if (s_state == SessionState_Recording) {
      prv_stop_recording();
    }
    prv_reset();
  }

unlock:
  mutex_unlock(s_lock);
}

// This will trigger an event to be sent to the main task indicating success or failure to set up
// a session. If the session setup result was success, the microphone will be enabled and we'll
// start sending Speex encoded data via the audio endpoint to the phone. voice_stop_dictation will
// end the recording
void voice_handle_session_setup_result(VoiceEndpointResult result,
    VoiceEndpointSessionType session_type, bool app_initiated) {
  PBL_LOG_DBG("voice_handle_session_setup_result: result=%d, session_type=%d, app_initiated=%d", 
              result, session_type, app_initiated);
  PBL_LOG_DBG("Current state: %d", s_state);

  mutex_lock(s_lock);

  if (s_state == SessionState_Idle) {
    PBL_LOG_DBG("State is Idle, ignoring session setup result");
    goto unlock;
  }

  bool has_error = true;

  if (s_state != SessionState_StartSession &&
      s_state != SessionState_AudioEndpointSetupReceived &&
      s_state != SessionState_VoiceEndpointSetupReceived) {
    PBL_LOG_WRN("Session setup result received when not expected, state=%d",
            (int)s_state);
    prv_cancel_session();
    VoiceEventType event_type = (s_state <= SessionState_StartSession) ?
        VoiceEventTypeSessionSetup : VoiceEventTypeSessionResult;
    prv_send_event(event_type, VoiceStatusErrorGeneric, NULL);
    goto done;
  }

  // If we already received the voice endpoint setup, a duplicate is unexpected.
  // Log a warning and ignore it rather than asserting in prv_handle_subsystem_started().
  if (s_state == SessionState_VoiceEndpointSetupReceived) {
    PBL_LOG_WRN("Duplicate voice endpoint setup result received, ignoring");
    goto unlock;
  }

  if (session_type >= VoiceEndpointSessionTypeCount) {
    PBL_LOG_WRN("Session setup result for invalid session type received");
    goto done;
  }

  if (result != VoiceEndpointResultSuccess) {
    PBL_LOG_DBG("Session setup failed with result %d", result);
    prv_cancel_session();
    VoiceStatus status = prv_get_status_from_result(result);
    PBL_LOG_WRN("Error occurred setting up session: %d", result);
    prv_send_event(VoiceEventTypeSessionSetup, status, NULL);
    goto done;
  }

  if (app_initiated != s_from_app) {
    PBL_LOG_DBG("App initiated mismatch - received=%d, expected=%d", app_initiated, s_from_app);
    prv_cancel_session();
    if (app_initiated) {
      PBL_LOG_WRN("Received session setup result for app initiated session when it "
              "was not expected");
    } else {
      PBL_LOG_WRN("Received session setup result for non-app session when an app "
              "session result was expected");
    }
    prv_send_event(VoiceEventTypeSessionSetup, VoiceStatusErrorGeneric, NULL);
    goto done;
  }

  PBL_LOG_DBG("Session setup successful!");
  has_error = false;

done:
  if (has_error) {
    PBL_LOG_DBG("Session setup had errors, stopping timeout timer");
    new_timer_stop(s_timeout);
  } else {
    PBL_LOG_DBG("Session setup complete, notifying voice endpoint subsystem started");
    prv_handle_subsystem_started(SessionState_VoiceEndpointSetupReceived);
  }
unlock:
  mutex_unlock(s_lock);
}

static bool prv_get_string_size_cb(const TranscriptionWord *word, void *data) {
  size_t *size = data;
  *size += word->length + sizeof(char); // add 1 for space or null terminator
  return true;
}

static bool prv_build_string_cb(const TranscriptionWord *word, void *data) {
  char *sentence = data;

  // if the current word is a punctuation mark strip out backspace (phone app inserts backspace
  // before punctuation mark) and do not insert a space before the word
  if (word->data[0] == '\x08') {
    strncat(sentence, (char *) &word->data[1], word->length - 1);
  } else {
    // if this is not the beginning of the string, insert a space before the word
    if (strlen(sentence) != 0) {
      strcat(sentence, " ");
    }
    strncat(sentence, (char *) word->data, word->length);
  }

  return true;
}

static bool prv_handle_dictation_nlp_result_common(VoiceEndpointResult result,
                                                   AudioEndpointSessionId session_id,
                                                   bool app_initiated, Uuid *app_uuid) {
  if (s_state == SessionState_Idle) {
    return false;
  }

  // stop timer before changing state variable
  new_timer_stop(s_timeout);

  if (s_state != SessionState_WaitForSessionResult) {
    // This handles erroneous replies from the phone app (sometimes the phone app sends a session
    // result immediately after we start streaming
    PBL_LOG_WRN("Session result when not expected (result: %d, "
        "session_id: %d)", result, session_id);
    if (s_state == SessionState_Recording) {
      prv_stop_recording();
    } else {
      prv_cancel_recording();
    }
    VoiceEventType event_type = (s_state <= SessionState_StartSession) ?
        VoiceEventTypeSessionSetup : VoiceEventTypeSessionResult;
    prv_send_event(event_type, VoiceStatusErrorGeneric, NULL);
    return false;
  }

  if (s_session_id != session_id) {
    PBL_LOG_WRN("Received session result for wrong session (Expected: "
        "%"PRIu16"; Received: %"PRIu16, s_session_id, session_id);
    prv_send_event(VoiceEventTypeSessionResult, VoiceStatusErrorGeneric, NULL);
    return false;
  }

  if (result != VoiceEndpointResultSuccess) {
    VoiceStatus status = prv_get_status_from_result(result);
    PBL_LOG_WRN("Error occurred processing result: %d", result);
    prv_send_event(VoiceEventTypeSessionResult, status, NULL);
    return false;
  }

  // Make sure that if this is an app initiated session, we're expecting a response for an app
  // initiated session and that if this is an app initiated session, the app UUID matches the
  // expected UUID
  if ((app_initiated != s_from_app) || (s_from_app && !uuid_equal(&s_app_uuid, app_uuid))) {
    if (app_initiated) {
      PBL_LOG_WRN("Received session result for app initiated session when a "
              "non-app session result was expected");
    } else {
      PBL_LOG_WRN("Received session result for non-app session when an app "
              "session result was expected");
    }
    prv_send_event(VoiceEventTypeSessionResult, VoiceStatusErrorGeneric, NULL);
    return false;
  }

  return true;
}

// receiving this ends the session, sending an event to the main task with the result
void voice_handle_dictation_result(VoiceEndpointResult result, AudioEndpointSessionId session_id,
                                   Transcription *transcription, bool app_initiated,
                                   Uuid *app_uuid) {
  mutex_lock(s_lock);

  if (!prv_handle_dictation_nlp_result_common(result, session_id, app_initiated, app_uuid)) {
    goto unlock;
  }

  // Calculate size of string
  size_t sentence_size = 0;
  transcription_iterate_words(transcription->sentences[0].words,
      transcription->sentences[0].word_count, prv_get_string_size_cb, &sentence_size);

  const size_t event_size = sizeof(PebbleVoiceServiceEventData) + sentence_size;
  PebbleVoiceServiceEventData *event_data = kernel_zalloc_check(event_size);

  // TODO: Final UI will probably demand a more sophisticated input, but this service will be
  // updated to support additional features when the final UI is implemented
  // Build string by concatenating each word in the first sentence
  transcription_iterate_words(transcription->sentences[0].words,
      transcription->sentences[0].word_count, prv_build_string_cb, event_data->sentence);

  if (app_initiated) {
    char uuid_str[UUID_STRING_BUFFER_LENGTH];
    uuid_to_string(app_uuid, uuid_str);
    PBL_LOG_DBG("Transcription received (%"PRIu32" B) for app %s",
        (uint32_t)sentence_size, uuid_str);
  } else {
    PBL_LOG_DBG("Transcription received (%"PRIu32" B)", (uint32_t)sentence_size);
  }

  prv_send_event(VoiceEventTypeSessionResult, VoiceStatusSuccess, event_data);

unlock:
  prv_reset();
  mutex_unlock(s_lock);
}

// receiving this ends the session, sending an event to the main task with the result
void voice_handle_nlp_result(VoiceEndpointResult result, AudioEndpointSessionId session_id,
                             char *reminder, time_t timestamp) {
  mutex_lock(s_lock);

  const bool app_initiated = false;
  Uuid *app_uuid = NULL;
  if (!prv_handle_dictation_nlp_result_common(result, session_id, app_initiated, app_uuid)) {
    goto unlock;
  }

  const size_t sentence_size = strlen(reminder) + 1;
  const size_t event_size = sizeof(PebbleVoiceServiceEventData) + sentence_size;
  PebbleVoiceServiceEventData *event_data = kernel_zalloc_check(event_size);
  *event_data = (PebbleVoiceServiceEventData) {
    .timestamp = timestamp,
  };
  strncpy(event_data->sentence, reminder, sentence_size);

  prv_send_event(VoiceEventTypeSessionResult, VoiceStatusSuccess, event_data);

unlock:
  prv_reset();
  mutex_unlock(s_lock);
}

DEFINE_SYSCALL(VoiceSessionId, sys_voice_start_dictation, VoiceEndpointSessionType session_type) {
  if (session_type >= VoiceEndpointSessionTypeCount) {
    return AUDIO_ENDPOINT_SESSION_INVALID_ID;
  }
  return voice_start_dictation(session_type);
}

DEFINE_SYSCALL(VoiceSessionId, sys_voice_start_dictation_from_recording,
               VoiceRecordingId recording_id) {
  if (PRIVILEGE_WAS_ELEVATED) {
    const PebbleProcessMd *app_md = app_manager_get_current_app_md();
    if (!app_md || !voice_recording_is_owned_by(recording_id, &app_md->uuid)) {
      return VOICE_SESSION_ID_INVALID;
    }
  }
  return voice_start_dictation_from_recording(recording_id);
}

DEFINE_SYSCALL(void, sys_voice_stop_dictation, VoiceSessionId session_id) {
  voice_stop_dictation(session_id);
}

DEFINE_SYSCALL(void, sys_voice_cancel_dictation, VoiceSessionId session_id) {
  voice_cancel_dictation(session_id);
}

void voice_kill_app_session(PebbleTask task) {
  if (task != PebbleTask_App) {
    return;
  }
  mutex_lock(s_lock);
  if (s_from_app && (s_session_id != AUDIO_ENDPOINT_SESSION_INVALID_ID)) {
    prv_cancel_session();
  }
  mutex_unlock(s_lock);
}
