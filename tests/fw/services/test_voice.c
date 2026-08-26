/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "fake_mutex.h"
#include "fake_new_timer.h"
#include "stubs_logging.h"
#include "stubs_passert.h"

#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "pbl/drivers/mic.h"
#include "pbl/services/audio_endpoint.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/services/system_task.h"
#include "pbl/services/voice/transcription.h"
#include "pbl/services/voice/voice.h"
#include "pbl/services/voice/voice_recording.h"
#include "pbl/services/voice/voice_speex.h"
#include "pbl/services/voice_endpoint.h"
#include "process_management/app_install_manager.h"
#include "process_management/app_manager.h"
#include "services/voice/voice_recording_storage.h"

static bool s_recording_in_progress;
static bool s_reservation_succeeds;
static int s_reservation_release_calls;
static int s_mic_is_running_calls;
static int s_endpoint_setup_calls;
static int s_payload_open_calls;

bool voice_recording_in_progress(void) {
  return s_recording_in_progress;
}

bool voice_recording_transcription_reserve(VoiceRecordingId id) {
  return s_reservation_succeeds;
}

void voice_recording_transcription_release(VoiceRecordingId id) {
  s_reservation_release_calls++;
}

bool voice_recording_is_owned_by(VoiceRecordingId id, const Uuid *app_uuid) {
  return true;
}

bool voice_recording_storage_get_metadata(VoiceRecordingId id,
                                          VoiceRecordingStorageMetadata *out) {
  *out = (VoiceRecordingStorageMetadata){
      .channels = 1,
  };
  return true;
}

int voice_recording_storage_open_payload(VoiceRecordingId id, uint32_t *data_bytes_out) {
  s_payload_open_calls++;
  *data_bytes_out = 10;
  return 7;
}

void voice_recording_storage_close_payload(int fd) {
}

int voice_recording_storage_read_frame(int fd, uint32_t *remaining_bytes, uint8_t *frame_out,
                                       size_t frame_out_size) {
  return 0;
}

bool mic_is_running(MicDevice *device) {
  s_mic_is_running_calls++;
  return false;
}

bool mic_start(MicDevice *device, MicDataHandlerCB data_handler, void *context,
               int16_t *audio_buffer, size_t audio_buffer_len) {
  return true;
}

void mic_stop(MicDevice *device) {
}

bool voice_speex_is_initialized(void) {
  return true;
}

bool voice_speex_init(void) {
  return true;
}

int16_t *voice_speex_get_frame_buffer(void) {
  static int16_t s_frame[320];
  return s_frame;
}

int voice_speex_get_frame_size(void) {
  return 320;
}

int voice_speex_encode_frame(int16_t *samples, uint8_t *encoded_data, size_t max_encoded_size) {
  return 1;
}

void voice_speex_get_transfer_info(AudioTransferInfoSpeex *info) {
  *info = (AudioTransferInfoSpeex){};
}

AudioEndpointSessionId audio_endpoint_setup_transfer(AudioEndpointStopTransferCallback callback) {
  return 1;
}

void audio_endpoint_add_frame(AudioEndpointSessionId session_id, uint8_t *frame,
                              uint8_t frame_size) {
}

void audio_endpoint_stop_transfer(AudioEndpointSessionId session_id) {
}

void audio_endpoint_cancel_transfer(AudioEndpointSessionId session_id) {
}

void voice_endpoint_setup_session(VoiceEndpointSessionType session_type,
                                  AudioEndpointSessionId session_id,
                                  AudioTransferInfoSpeex *info, Uuid *app_uuid) {
  s_endpoint_setup_calls++;
}

CommSession *comm_session_get_system_session(void) {
  return NULL;
}

void comm_session_set_responsiveness(CommSession *session, BtConsumer consumer,
                                     ResponseTimeState state, uint16_t max_period_secs) {
}

bool system_task_add_callback(SystemTaskEventCallback callback, void *data) {
  return true;
}

void event_put(PebbleEvent *event) {
}

void *transcription_iterate_words(const TranscriptionWord *words, size_t count,
                                  TranscriptionWordIterateCb handle_word, void *data) {
  return (void *)(words + count);
}

PebbleTask pebble_task_get_current(void) {
  return PebbleTask_KernelMain;
}

bool app_install_id_from_system(AppInstallId id) {
  return true;
}

AppInstallId app_manager_get_current_app_id(void) {
  return 1;
}

const PebbleProcessMd *app_manager_get_current_app_md(void) {
  static PebbleProcessMd s_app_md;
  return &s_app_md;
}

void test_voice__initialize(void) {
  s_recording_in_progress = false;
  s_reservation_succeeds = true;
  s_reservation_release_calls = 0;
  s_mic_is_running_calls = 0;
  s_endpoint_setup_calls = 0;
  s_payload_open_calls = 0;
  voice_init();
}

void test_voice__cleanup(void) {
  stub_new_timer_cleanup();
  fake_mutex_reset(true);
}

void test_voice__active_recording_blocks_live_dictation_before_mic_check(void) {
  s_recording_in_progress = true;

  cl_assert_equal_i(voice_start_dictation(VoiceEndpointSessionTypeDictation),
                    VOICE_SESSION_ID_INVALID);
  cl_assert_equal_i(s_mic_is_running_calls, 0);
  cl_assert_equal_i(s_endpoint_setup_calls, 0);
}

void test_voice__recording_start_reservation_blocks_live_dictation(void) {
  cl_assert(voice_session_reserve_recording());

  cl_assert_equal_i(voice_start_dictation(VoiceEndpointSessionTypeDictation),
                    VOICE_SESSION_ID_INVALID);
  cl_assert_equal_i(s_mic_is_running_calls, 0);
  cl_assert_equal_i(s_endpoint_setup_calls, 0);

  voice_session_release_recording();
}

void test_voice__active_recording_blocks_stored_transcription_before_file_open(void) {
  s_recording_in_progress = true;

  cl_assert_equal_i(voice_start_dictation_from_recording(42), VOICE_SESSION_ID_INVALID);
  cl_assert_equal_i(s_payload_open_calls, 0);
  cl_assert_equal_i(s_mic_is_running_calls, 0);
  cl_assert_equal_i(s_reservation_release_calls, 1);
  cl_assert_equal_i(s_endpoint_setup_calls, 0);
}
