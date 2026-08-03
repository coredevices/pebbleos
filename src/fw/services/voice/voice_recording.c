/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/voice/voice_recording.h"

#include "voice_recording_playback.h"
#include "voice_recording_storage.h"

#include "board/board.h"
#include <pbl/drivers/mic.h>
#include <pbl/drivers/rtc.h>
#include "kernel/event_loop.h"
#include "kernel/pebble_tasks.h"
#include <pbl/os/mutex.h>
#include <pbl/logging/logging.h>
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/settings/settings_file.h"
#include "pbl/services/voice/voice.h"
#include "pbl/services/voice/voice_speex.h"
#include "process_management/app_install_manager.h"
#include "process_management/app_manager.h"
#include "syscall/syscall_internal.h"
#include "system/passert.h"
#include <pbl/util/attributes.h>
#include "util/units.h"
#include <pbl/util/uuid.h>

#include <inttypes.h>
#include <stdint.h>
#include <string.h>

PBL_LOG_MODULE_DECLARE(service_voice, CONFIG_SERVICE_VOICE_LOG_LEVEL);

#define VOICE_REC_MAX_DURATION_MS (120 * 1000)
// Memos are encoded at a higher Speex quality than live dictation (whose bit rate is
// constrained by the BT link); the default quality is restored when the recording ends.
#define VOICE_REC_SETTINGS_FILE "voicerec"
#define VOICE_REC_SETTINGS_KEY "config"
#define VOICE_REC_SETTINGS_SIZE (128)
// ~27.8 kbps at quality 8, plus one length byte per 20 ms frame, rounded up.
#define VOICE_REC_BYTES_PER_SEC (3600)
#define VOICE_REC_TOTAL_STORAGE_BYTES (KiBYTES(1024))
#define VOICE_REC_STAGING_SIZE (1024)

static PebbleMutex *s_lock;
static VoiceRecordingId s_active_id = VOICE_RECORDING_ID_INVALID;
static VoiceRecordingId s_next_id = 1;
static PebbleTask s_owner_task = PebbleTask_Unknown;

static int s_temp_fd = -1;
static uint32_t s_data_bytes;
static uint32_t s_frame_count;
static uint32_t s_created;
static Uuid s_app_uuid;
static uint32_t s_samples_per_frame;
static uint32_t s_cap_data_bytes;
static bool s_capped;
static VoiceRecordingError s_last_error;
// Id of the most recently saved recording, so a stop() racing an auto-stop can be answered
// accurately without reporting success for an unrelated stored recording.
static VoiceRecordingId s_last_saved_id = VOICE_RECORDING_ID_INVALID;

typedef struct {
  VoiceRecordingQuality quality;
  uint16_t record_gain;
  uint16_t playback_gain;
} VoiceRecordingConfig;

static VoiceRecordingConfig s_config = {
    .quality = VoiceRecordingQuality_High,
    .record_gain = VOICE_RECORDING_GAIN_DEFAULT,
    .playback_gain = VOICE_RECORDING_GAIN_DEFAULT,
};

static uint8_t s_staging[VOICE_REC_STAGING_SIZE];
static size_t s_staging_used;
static TimerID s_max_timer = TIMER_INVALID_ID;

// UUID of the app that started the active playback, or UUID_INVALID when playback was started
// by the system (or is not app-owned). An app may only stop the playback it started itself.
static Uuid s_playback_owner = UUID_INVALID_INIT;

static void prv_stop_callback(void *data);

static void prv_schedule_stop(void) {
  s_capped = true;
  launcher_task_add_callback(prv_stop_callback, (void *)(uintptr_t)s_active_id);
}

static bool prv_flush_staging(void) {
  if (s_staging_used == 0) {
    return true;
  }

  const int written = pfs_write(s_temp_fd, s_staging, s_staging_used);
  if (written < (int)s_staging_used) {
    PBL_LOG_ERR("Failed to write recording staging buffer (%d)", written);
    return false;
  }
  s_staging_used = 0;
  return true;
}

static void prv_data_handler(int16_t *samples, size_t sample_count, void *context) {
  if ((s_active_id == VOICE_RECORDING_ID_INVALID) || s_capped) {
    return;
  }

  if (s_config.record_gain != VOICE_RECORDING_GAIN_DEFAULT) {
    for (size_t i = 0; i < sample_count; i++) {
      const int32_t amplified =
          (int32_t)samples[i] * s_config.record_gain / VOICE_RECORDING_GAIN_DEFAULT;
      samples[i] = (int16_t)MAX(INT16_MIN, MIN(INT16_MAX, amplified));
    }
  }

  uint8_t encoded[VOICE_SPEEX_MAX_ENCODED_FRAME_SIZE];
  const int encoded_bytes = voice_speex_encode_frame(samples, encoded, sizeof(encoded));
  if (encoded_bytes <= 0) {
    PBL_LOG_DBG("Failed to encode recording frame");
    return;
  }

  const size_t record_size = 1 + (size_t)encoded_bytes;
  if (s_data_bytes + record_size > s_cap_data_bytes) {
    PBL_LOG_DBG("Recording reached capacity, stopping");
    prv_schedule_stop();
    return;
  }

  if ((s_staging_used + record_size > sizeof(s_staging)) && !prv_flush_staging()) {
    prv_schedule_stop();
    return;
  }

  s_staging[s_staging_used++] = (uint8_t)encoded_bytes;
  memcpy(&s_staging[s_staging_used], encoded, encoded_bytes);
  s_staging_used += encoded_bytes;
  s_data_bytes += record_size;
  s_frame_count++;
}

static void prv_close_temp(bool remove) {
  if (remove) {
    pfs_close_and_remove(s_temp_fd);
  } else {
    pfs_close(s_temp_fd);
  }
  s_temp_fd = -1;
}

static void prv_reset(void) {
  voice_speex_set_quality(VOICE_SPEEX_QUALITY_DEFAULT);
  s_active_id = VOICE_RECORDING_ID_INVALID;
  s_owner_task = PebbleTask_Unknown;
  s_data_bytes = 0;
  s_frame_count = 0;
  s_staging_used = 0;
  s_capped = false;
}

static void prv_fill_metadata(VoiceRecordingStorageMetadata *metadata) {
  *metadata = (VoiceRecordingStorageMetadata){
      .channels = (uint16_t)mic_get_channels(MIC),
      .created = s_created,
      .app_uuid = s_app_uuid,
      .data_bytes = s_data_bytes,
  };
  voice_speex_get_transfer_info(&metadata->speex);
  if (s_samples_per_frame > 0) {
    metadata->duration_ms =
        (uint32_t)((uint64_t)s_frame_count * s_samples_per_frame * 1000 / MIC_SAMPLE_RATE);
  }
}

static bool prv_stop_locked(VoiceRecordingId id) {
  if ((id == VOICE_RECORDING_ID_INVALID) || (id != s_active_id)) {
    return false;
  }

  new_timer_stop(s_max_timer);
  s_active_id = VOICE_RECORDING_ID_INVALID;
  mic_stop(MIC);

  bool ok = prv_flush_staging();
  prv_close_temp(false);

  if (!ok) {
    s_last_error = VoiceRecordingError_Write;
  } else {
    VoiceRecordingStorageMetadata metadata;
    prv_fill_metadata(&metadata);
    ok = voice_recording_storage_finalize(id, &metadata, &s_last_error);
  }

  voice_recording_storage_remove_temp(id);
  prv_reset();
  if (ok) {
    s_last_error = VoiceRecordingError_None;
    s_last_saved_id = id;
  }
  PBL_LOG_DBG("Stopped recording id=%u (%s)", (unsigned)id, ok ? "saved" : "failed");
  return ok;
}

static void prv_cancel_locked(VoiceRecordingId id) {
  if ((id == VOICE_RECORDING_ID_INVALID) || (id != s_active_id)) {
    return;
  }

  new_timer_stop(s_max_timer);
  s_active_id = VOICE_RECORDING_ID_INVALID;
  mic_stop(MIC);
  prv_close_temp(true);
  prv_reset();
  PBL_LOG_DBG("Cancelled recording id=%u", (unsigned)id);
}

static void prv_stop_callback(void *data) {
  (void)voice_recording_stop((VoiceRecordingId)(uintptr_t)data);
}

static void prv_max_duration_timeout(void *data) {
  launcher_task_add_callback(prv_stop_callback, data);
}

void voice_recording_init(void) {
  s_lock = mutex_create();
  SettingsFile file = {{0}};
  VoiceRecordingConfig config;
  if ((settings_file_open(&file, VOICE_REC_SETTINGS_FILE, VOICE_REC_SETTINGS_SIZE) == S_SUCCESS)) {
    if (settings_file_get(&file, VOICE_REC_SETTINGS_KEY, strlen(VOICE_REC_SETTINGS_KEY), &config,
                          sizeof(config)) == S_SUCCESS &&
        config.quality <= VoiceRecordingQuality_High &&
        config.record_gain >= VOICE_RECORDING_GAIN_MIN &&
        config.record_gain <= VOICE_RECORDING_GAIN_MAX &&
        config.playback_gain >= VOICE_RECORDING_GAIN_MIN &&
        config.playback_gain <= VOICE_RECORDING_GAIN_MAX) {
      s_config = config;
    }
    settings_file_close(&file);
  }
  voice_recording_storage_init(&s_next_id);
  voice_recording_playback_init();
}

VoiceRecordingId voice_recording_start(void) {
  mutex_lock(s_lock);

  VoiceRecordingId id = VOICE_RECORDING_ID_INVALID;
  s_last_error = VoiceRecordingError_None;

  if ((s_active_id != VOICE_RECORDING_ID_INVALID) || voice_recording_playback_is_active()) {
    PBL_LOG_DBG("Recording or playback already in progress");
    s_last_error = VoiceRecordingError_Busy;
    goto unlock;
  }

  if (mic_is_running(MIC)) {
    PBL_LOG_WRN("Microphone busy, cannot start recording");
    s_last_error = VoiceRecordingError_MicBusy;
    goto unlock;
  }

  const uint32_t stored_bytes = voice_recording_storage_total_bytes();
  if (stored_bytes >= VOICE_REC_TOTAL_STORAGE_BYTES) {
    PBL_LOG_WRN("Recording storage budget exhausted");
    s_last_error = VoiceRecordingError_StorageFull;
    goto unlock;
  }

  if (!voice_speex_is_initialized() && !voice_speex_init()) {
    PBL_LOG_ERR("Failed to initialize Speex encoder for recording");
    s_last_error = VoiceRecordingError_Codec;
    goto unlock;
  }

  s_samples_per_frame = (uint32_t)voice_speex_get_frame_size() / mic_get_channels(MIC);
  const uint32_t max_data_bytes = (VOICE_REC_MAX_DURATION_MS / 1000) * VOICE_REC_BYTES_PER_SEC;
  const uint32_t header_size = voice_recording_storage_header_size();
  const uint32_t remaining_budget = VOICE_REC_TOTAL_STORAGE_BYTES - stored_bytes;
  if (remaining_budget <= header_size + VOICE_SPEEX_MAX_ENCODED_FRAME_SIZE + 1) {
    PBL_LOG_WRN("Recording storage budget exhausted");
    s_last_error = VoiceRecordingError_StorageFull;
    goto unlock;
  }

  const uint32_t budget_data_bytes = remaining_budget - header_size;
  s_cap_data_bytes = (budget_data_bytes < max_data_bytes) ? budget_data_bytes : max_data_bytes;
  const uint32_t prealloc = header_size + s_cap_data_bytes;
  const uint32_t finalize_space = prealloc * 2;
  if (get_available_pfs_space() < finalize_space) {
    PBL_LOG_WRN("Not enough flash to record and finalize (need %" PRIu32 ")", finalize_space);
    s_last_error = VoiceRecordingError_NoSpace;
    goto unlock;
  }

  // After the uint16 id space has wrapped, s_next_id can collide with a recording that is
  // still stored; probe until a free id is found so an old memo is never overwritten.
  id = s_next_id;
  uint32_t probes = 0;
  while (voice_recording_storage_id_in_use(id) && (++probes < UINT16_MAX)) {
    id = (id == UINT16_MAX) ? 1 : (id + 1);
  }
  if (probes >= UINT16_MAX) {
    PBL_LOG_ERR("No free recording id");
    s_last_error = VoiceRecordingError_StorageFull;
    id = VOICE_RECORDING_ID_INVALID;
    goto unlock;
  }

  s_temp_fd = voice_recording_storage_open_temp(id, s_cap_data_bytes);
  if (s_temp_fd < 0) {
    PBL_LOG_ERR("Failed to create temp recording file (%d)", s_temp_fd);
    s_last_error = VoiceRecordingError_FileOpen;
    id = VOICE_RECORDING_ID_INVALID;
    goto unlock;
  }

  const bool from_app = (pebble_task_get_current() == PebbleTask_App) &&
                        !app_install_id_from_system(app_manager_get_current_app_id());
  s_app_uuid = from_app ? app_manager_get_current_app_md()->uuid : UUID_INVALID;
  s_owner_task = from_app ? PebbleTask_App : PebbleTask_Unknown;
  s_created = (uint32_t)rtc_get_time();
  s_data_bytes = 0;
  s_frame_count = 0;
  s_staging_used = 0;
  s_capped = false;

  if (!mic_start(MIC, prv_data_handler, NULL, voice_speex_get_frame_buffer(),
                 voice_speex_get_frame_size())) {
    PBL_LOG_ERR("Failed to start microphone for recording");
    s_last_error = VoiceRecordingError_MicStart;
    prv_close_temp(true);
    id = VOICE_RECORDING_ID_INVALID;
    goto unlock;
  }

  static const uint8_t s_speex_quality[] = {4, 6, 8};
  voice_speex_set_quality(s_speex_quality[s_config.quality]);

  s_active_id = id;
  s_next_id = (id == UINT16_MAX) ? 1 : (id + 1);

  if (s_max_timer == TIMER_INVALID_ID) {
    s_max_timer = new_timer_create();
  }
  new_timer_start(s_max_timer, VOICE_REC_MAX_DURATION_MS, prv_max_duration_timeout,
                  (void *)(uintptr_t)id, 0);
  PBL_LOG_DBG("Started recording id=%u", (unsigned)id);

unlock:
  mutex_unlock(s_lock);
  return id;
}

bool voice_recording_stop(VoiceRecordingId id) {
  mutex_lock(s_lock);
  bool stopped;
  if (id == s_active_id) {
    stopped = prv_stop_locked(id);
  } else {
    // The recording may have just been auto-stopped (duration cap or storage full) before this
    // call landed: report success only for that recording, never for an older stored one.
    stopped = (id != VOICE_RECORDING_ID_INVALID) && (id == s_last_saved_id);
  }
  mutex_unlock(s_lock);
  return stopped;
}

void voice_recording_stop_active(void) {
  mutex_lock(s_lock);
  (void)prv_stop_locked(s_active_id);
  mutex_unlock(s_lock);
}

void voice_recording_cancel(VoiceRecordingId id) {
  mutex_lock(s_lock);
  prv_cancel_locked(id);
  mutex_unlock(s_lock);
}

void voice_recording_cleanup_task(PebbleTask task) {
  mutex_lock(s_lock);
  if (s_owner_task == task) {
    prv_cancel_locked(s_active_id);
  }
  if ((task == PebbleTask_App) && !uuid_is_invalid(&s_playback_owner)) {
    voice_recording_playback_stop();
    s_playback_owner = UUID_INVALID;
  }
  mutex_unlock(s_lock);
}

bool voice_recording_in_progress(void) {
  mutex_lock(s_lock);
  const bool recording = (s_active_id != VOICE_RECORDING_ID_INVALID);
  mutex_unlock(s_lock);
  return recording;
}

static bool prv_is_owned_by_locked(VoiceRecordingId id, const Uuid *app_uuid) {
  if ((s_active_id != VOICE_RECORDING_ID_INVALID) && (id == s_active_id)) {
    return uuid_equal(&s_app_uuid, app_uuid);
  }
  VoiceRecordingStorageMetadata metadata;
  return voice_recording_storage_get_metadata(id, &metadata) &&
         uuid_equal(&metadata.app_uuid, app_uuid);
}

bool voice_recording_is_owned_by(VoiceRecordingId id, const Uuid *app_uuid) {
  mutex_lock(s_lock);
  const bool owned = prv_is_owned_by_locked(id, app_uuid);
  mutex_unlock(s_lock);
  return owned;
}

uint32_t voice_recording_list(VoiceRecordingInfo *out, uint32_t max) {
  return voice_recording_storage_list(out, max);
}

uint32_t voice_recording_list_summaries(VoiceRecordingSummary *out, uint32_t max,
                                        bool *has_more) {
  return voice_recording_storage_list_summaries(out, max, has_more);
}

uint32_t voice_recording_list_page(VoiceRecordingInfo *out, uint32_t max, uint32_t offset,
                                   bool *has_more) {
  return voice_recording_storage_list_page(out, max, offset, has_more);
}

uint32_t voice_recording_list_owned_by(VoiceRecordingInfo *out, uint32_t max,
                                       const Uuid *app_uuid) {
  return voice_recording_storage_list_owned_by(out, max, app_uuid);
}

bool voice_recording_delete(VoiceRecordingId id) {
  mutex_lock(s_lock);
  if (voice_recording_playback_is_playing_id(id)) {
    voice_recording_playback_stop();
  }
  bool deleted = false;
  // Removing an open PFS file panics; the transcription stream keeps the recording open
  // for its whole (real-time) duration.
  if (voice_transcribing_recording_id() == id) {
    PBL_LOG_WRN("Recording %u is being transcribed, refusing to delete", (unsigned)id);
  } else {
    deleted = voice_recording_storage_delete(id);
    if (deleted && (id == s_last_saved_id)) {
      s_last_saved_id = VOICE_RECORDING_ID_INVALID;
    }
  }
  mutex_unlock(s_lock);
  return deleted;
}

void voice_recording_delete_owned_by(const Uuid *app_uuid) {
  if (!app_uuid) {
    return;
  }
  mutex_lock(s_lock);
  // Playback may hold one of this app's files open; removing an open PFS file panics.
  const VoiceRecordingId playing_id = voice_recording_playback_get_active_id();
  if ((playing_id != VOICE_RECORDING_ID_INVALID) &&
      prv_is_owned_by_locked(playing_id, app_uuid)) {
    voice_recording_playback_stop();
  }
  // Skip a recording held open by an active transcription stream (see voice_recording_delete).
  voice_recording_storage_delete_owned_by(app_uuid, voice_transcribing_recording_id());
  mutex_unlock(s_lock);
}

bool voice_recording_play(VoiceRecordingId id) {
  mutex_lock(s_lock);
  const bool started =
      (s_active_id == VOICE_RECORDING_ID_INVALID) && voice_recording_playback_start(id);
  if (started) {
    const bool from_app = (pebble_task_get_current() == PebbleTask_App) &&
                          !app_install_id_from_system(app_manager_get_current_app_id());
    s_playback_owner = from_app ? app_manager_get_current_app_md()->uuid : UUID_INVALID;
  }
  mutex_unlock(s_lock);
  return started;
}

bool voice_recording_playback_owned_by(const Uuid *app_uuid) {
  if (!app_uuid) {
    return false;
  }
  mutex_lock(s_lock);
  const bool owned = !uuid_is_invalid(&s_playback_owner) &&
                     uuid_equal(&s_playback_owner, app_uuid) &&
                     voice_recording_playback_is_active();
  mutex_unlock(s_lock);
  return owned;
}

void voice_recording_stop_playback(void) {
  voice_recording_playback_stop();
  mutex_lock(s_lock);
  s_playback_owner = UUID_INVALID;
  mutex_unlock(s_lock);
}

bool voice_recording_is_playing(void) {
  return voice_recording_playback_is_active();
}

VoiceRecordingError voice_recording_last_error(void) {
  mutex_lock(s_lock);
  const VoiceRecordingError error = s_last_error;
  mutex_unlock(s_lock);
  return error;
}

static void prv_save_config(void) {
  SettingsFile file = {{0}};
  if (settings_file_open(&file, VOICE_REC_SETTINGS_FILE, VOICE_REC_SETTINGS_SIZE) == S_SUCCESS) {
    settings_file_set(&file, VOICE_REC_SETTINGS_KEY, strlen(VOICE_REC_SETTINGS_KEY), &s_config,
                      sizeof(s_config));
    settings_file_close(&file);
  }
}

VoiceRecordingQuality voice_recording_get_quality(void) {
  return s_config.quality;
}

void voice_recording_set_quality(VoiceRecordingQuality quality) {
  if (quality <= VoiceRecordingQuality_High) {
    s_config.quality = quality;
    prv_save_config();
  }
}

uint16_t voice_recording_get_record_gain(void) {
  return s_config.record_gain;
}

void voice_recording_set_record_gain(uint16_t gain) {
  s_config.record_gain = MIN(VOICE_RECORDING_GAIN_MAX, MAX(VOICE_RECORDING_GAIN_MIN, gain));
  prv_save_config();
}

uint16_t voice_recording_get_playback_gain(void) {
  return s_config.playback_gain;
}

void voice_recording_set_playback_gain(uint16_t gain) {
  s_config.playback_gain = MIN(VOICE_RECORDING_GAIN_MAX, MAX(VOICE_RECORDING_GAIN_MIN, gain));
  prv_save_config();
}
