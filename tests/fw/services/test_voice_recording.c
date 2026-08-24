/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "services/voice/voice_recording_storage.h"

#include "fake_mutex.h"
#include "fake_new_timer.h"
#include "stubs_logging.h"
#include "stubs_passert.h"

#include "kernel/event_loop.h"
#include "pbl/drivers/mic.h"
#include "pbl/drivers/rtc.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/settings/settings_file.h"
#include "pbl/services/voice/voice.h"
#include "pbl/services/voice/voice_recording.h"
#include "pbl/services/voice/voice_speex.h"
#include "process_management/app_install_manager.h"
#include "process_management/app_manager.h"

static bool s_voice_session_active;
static int s_voice_session_release_calls;
static bool s_mic_running;
static int s_mic_start_calls;

static bool s_metadata_exists;
static Uuid s_metadata_owner;
static int s_storage_delete_calls;
static VoiceRecordingId s_storage_delete_id;
static int s_storage_delete_owned_by_calls;
static VoiceRecordingId s_storage_delete_owned_by_skip_id;
static bool s_storage_calls_are_locked;
static uint32_t s_storage_total_bytes;

static bool s_playback_active;
static bool s_playback_stop_was_locked;
static Uuid s_current_app_uuid;

static bool prv_any_mutex_locked(void) {
  for (FakePebbleMutex *mutex = s_mutex_list; mutex;
       mutex = (FakePebbleMutex *)mutex->node.next) {
    if (mutex->lock_count != 0) {
      return true;
    }
  }
  return false;
}

static void prv_note_storage_lock(void) {
  s_storage_calls_are_locked &= prv_any_mutex_locked();
}

bool voice_session_reserve_recording(void) {
  return !s_voice_session_active;
}

void voice_session_release_recording(void) {
  s_voice_session_release_calls++;
}

bool mic_is_running(MicDevice *device) {
  return s_mic_running;
}

bool mic_start(MicDevice *device, MicDataHandlerCB data_handler, void *context,
               int16_t *audio_buffer, size_t audio_buffer_len) {
  s_mic_start_calls++;
  s_mic_running = true;
  return true;
}

void mic_stop(MicDevice *device) {
  s_mic_running = false;
}

uint32_t mic_get_channels(MicDevice *device) {
  return 1;
}

time_t rtc_get_time(void) {
  return 1234;
}

uint32_t get_available_pfs_space(void) {
  return UINT32_MAX;
}

int pfs_write(int fd, const void *buf, size_t size) {
  return (int)size;
}

status_t pfs_close(int fd) {
  return S_SUCCESS;
}

status_t pfs_close_and_remove(int fd) {
  return S_SUCCESS;
}

status_t settings_file_open(SettingsFile *file, const char *name, int max_used_space) {
  return E_DOES_NOT_EXIST;
}

status_t settings_file_get(SettingsFile *file, const void *key, size_t key_len, void *val_out,
                           size_t val_out_len) {
  return E_DOES_NOT_EXIST;
}

status_t settings_file_set(SettingsFile *file, const void *key, size_t key_len, const void *val,
                           size_t val_len) {
  return S_SUCCESS;
}

void settings_file_close(SettingsFile *file) {
}

void launcher_task_add_callback(CallbackEventCallback callback, void *data) {
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

void voice_recording_storage_init(VoiceRecordingId *next_id_out) {
  *next_id_out = 1;
}

bool voice_recording_storage_id_in_use(VoiceRecordingId id) {
  return false;
}

uint32_t voice_recording_storage_header_size(void) {
  return 32;
}

int voice_recording_storage_open_temp(VoiceRecordingId id, uint32_t payload_capacity) {
  return 7;
}

void voice_recording_storage_remove_temp(VoiceRecordingId id) {
}

bool voice_recording_storage_finalize(VoiceRecordingId id,
                                      const VoiceRecordingStorageMetadata *metadata,
                                      VoiceRecordingError *error_out) {
  return true;
}

bool voice_recording_storage_get_metadata(VoiceRecordingId id,
                                          VoiceRecordingStorageMetadata *out) {
  if (!s_metadata_exists) {
    return false;
  }
  *out = (VoiceRecordingStorageMetadata){
      .app_uuid = s_metadata_owner,
  };
  return true;
}

uint32_t voice_recording_storage_list(VoiceRecordingInfo *out, uint32_t max) {
  prv_note_storage_lock();
  return 0;
}

uint32_t voice_recording_storage_list_summaries(VoiceRecordingSummary *out, uint32_t max,
                                                bool *has_more) {
  prv_note_storage_lock();
  return 0;
}

uint32_t voice_recording_storage_list_page(VoiceRecordingInfo *out, uint32_t max,
                                           uint32_t offset, bool *has_more) {
  prv_note_storage_lock();
  return 0;
}

uint32_t voice_recording_storage_list_owned_by(VoiceRecordingInfo *out, uint32_t max,
                                               const Uuid *app_uuid) {
  prv_note_storage_lock();
  return 0;
}

uint32_t voice_recording_storage_total_bytes(void) {
  prv_note_storage_lock();
  return s_storage_total_bytes;
}

bool voice_recording_storage_delete(VoiceRecordingId id) {
  prv_note_storage_lock();
  s_storage_delete_calls++;
  s_storage_delete_id = id;
  return true;
}

void voice_recording_storage_delete_owned_by(const Uuid *app_uuid, VoiceRecordingId skip_id) {
  prv_note_storage_lock();
  s_storage_delete_owned_by_calls++;
  s_storage_delete_owned_by_skip_id = skip_id;
}

void voice_recording_playback_init(void) {
}

bool voice_recording_playback_start(VoiceRecordingId id) {
  s_playback_active = true;
  return true;
}

void voice_recording_playback_stop(void) {
  s_playback_stop_was_locked = prv_any_mutex_locked();
  s_playback_active = false;
}

bool voice_recording_playback_is_active(void) {
  return s_playback_active;
}

bool voice_recording_playback_is_playing_id(VoiceRecordingId id) {
  return false;
}

VoiceRecordingId voice_recording_playback_get_active_id(void) {
  return VOICE_RECORDING_ID_INVALID;
}

PebbleTask pebble_task_get_current(void) {
  return PebbleTask_App;
}

bool app_install_id_from_system(AppInstallId id) {
  return false;
}

AppInstallId app_manager_get_current_app_id(void) {
  return 1;
}

const PebbleProcessMd *app_manager_get_current_app_md(void) {
  static PebbleProcessMd s_app_md;
  s_app_md.uuid = s_current_app_uuid;
  return &s_app_md;
}

void test_voice_recording__initialize(void) {
  s_voice_session_active = false;
  s_voice_session_release_calls = 0;
  s_mic_running = false;
  s_mic_start_calls = 0;

  s_metadata_exists = true;
  s_metadata_owner = (Uuid){.byte0 = 1};
  s_storage_delete_calls = 0;
  s_storage_delete_id = VOICE_RECORDING_ID_INVALID;
  s_storage_delete_owned_by_calls = 0;
  s_storage_delete_owned_by_skip_id = VOICE_RECORDING_ID_INVALID;
  s_storage_calls_are_locked = true;
  s_storage_total_bytes = 0;

  s_playback_active = false;
  s_playback_stop_was_locked = false;
  s_current_app_uuid = (Uuid){.byte0 = 2};

  voice_recording_init();
}

void test_voice_recording__cleanup(void) {
  stub_new_timer_cleanup();
  fake_mutex_reset(true);
}

void test_voice_recording__active_voice_session_blocks_capture(void) {
  s_voice_session_active = true;

  cl_assert_equal_i(voice_recording_start(), VOICE_RECORDING_ID_INVALID);
  cl_assert_equal_i(voice_recording_last_error(), VoiceRecordingError_Busy);
  cl_assert_equal_i(s_mic_start_calls, 0);
  cl_assert_equal_i(s_voice_session_release_calls, 0);
}

void test_voice_recording__all_list_readers_hold_the_recording_lock(void) {
  VoiceRecordingInfo info;
  VoiceRecordingSummary summary;
  bool has_more;

  voice_recording_list(&info, 1);
  voice_recording_list_summaries(&summary, 1, &has_more);
  voice_recording_list_page(&info, 1, 0, &has_more);
  voice_recording_list_owned_by(&info, 1, &s_metadata_owner);

  cl_assert(s_storage_calls_are_locked);
}

void test_voice_recording__transcription_reservation_blocks_delete(void) {
  cl_assert(voice_recording_transcription_reserve(42));

  cl_assert(!voice_recording_delete(42));
  cl_assert_equal_i(s_storage_delete_calls, 0);

  voice_recording_transcription_release(42);
}

void test_voice_recording__owner_cleanup_defers_reserved_recording_delete(void) {
  cl_assert(voice_recording_transcription_reserve(42));

  voice_recording_delete_owned_by(&s_metadata_owner);

  cl_assert_equal_i(s_storage_delete_owned_by_calls, 1);
  cl_assert_equal_i(s_storage_delete_owned_by_skip_id, 42);
  cl_assert_equal_i(s_storage_delete_calls, 0);

  voice_recording_transcription_release(42);

  cl_assert_equal_i(s_storage_delete_calls, 1);
  cl_assert_equal_i(s_storage_delete_id, 42);
  cl_assert(s_storage_calls_are_locked);
}

void test_voice_recording__stop_playback_updates_owner_under_recording_lock(void) {
  cl_assert(voice_recording_play(7));
  cl_assert(voice_recording_playback_owned_by(&s_current_app_uuid));

  voice_recording_stop_playback();

  cl_assert(s_playback_stop_was_locked);
  cl_assert(!voice_recording_playback_owned_by(&s_current_app_uuid));
}

void test_voice_recording__successful_capture_releases_voice_reservation(void) {
  const VoiceRecordingId id = voice_recording_start();

  cl_assert(id != VOICE_RECORDING_ID_INVALID);
  cl_assert_equal_i(s_voice_session_release_calls, 1);
  cl_assert(voice_recording_stop(id));
}

void test_voice_recording__storage_usage_reports_remaining_quota(void) {
  uint32_t used_bytes;
  uint32_t available_bytes;
  s_storage_total_bytes = 12345;

  voice_recording_get_storage_usage(&used_bytes, &available_bytes);

  cl_assert_equal_i(used_bytes, 12345);
  cl_assert_equal_i(available_bytes, 1024 * 1024 - 12345);
  cl_assert(s_storage_calls_are_locked);
}

void test_voice_recording__storage_usage_clamps_exhausted_quota(void) {
  uint32_t available_bytes;
  s_storage_total_bytes = 2 * 1024 * 1024;

  voice_recording_get_storage_usage(NULL, &available_bytes);

  cl_assert_equal_i(available_bytes, 0);
}
