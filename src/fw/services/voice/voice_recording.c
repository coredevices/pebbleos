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
#define VOICE_REC_BYTES_PER_SEC (1600)
#define VOICE_REC_TOTAL_STORAGE_BYTES (KiBYTES(512))
#define VOICE_REC_MAX_ENCODED_FRAME (200)
#define VOICE_REC_STAGING_SIZE (1024)

typedef enum {
  RecState_Idle = 0,
  RecState_Recording,
} RecState;

static PebbleMutex *s_lock;
static RecState s_state = RecState_Idle;
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

static uint8_t s_staging[VOICE_REC_STAGING_SIZE];
static size_t s_staging_used;
static TimerID s_max_timer = TIMER_INVALID_ID;

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
  if ((s_state != RecState_Recording) || s_capped) {
    return;
  }

  uint8_t encoded[VOICE_REC_MAX_ENCODED_FRAME];
  const int encoded_bytes = voice_speex_encode_frame(samples, encoded, sizeof(encoded));
  if (encoded_bytes <= 0) {
    PBL_LOG_DBG("Failed to encode recording frame");
    return;
  }

  const size_t record_size = 1 + (size_t)encoded_bytes;
  if (s_data_bytes + record_size > s_cap_data_bytes) {
    PBL_LOG_DBG("Recording reached capacity, dropping further audio");
    s_capped = true;
    return;
  }

  if ((s_staging_used + record_size > sizeof(s_staging)) && !prv_flush_staging()) {
    s_capped = true;
    return;
  }

  s_staging[s_staging_used++] = (uint8_t)encoded_bytes;
  memcpy(&s_staging[s_staging_used], encoded, encoded_bytes);
  s_staging_used += encoded_bytes;
  s_data_bytes += record_size;
  s_frame_count++;
}

static void prv_close_temp(bool remove) {
  voice_recording_storage_close_temp(s_temp_fd, remove);
  s_temp_fd = -1;
}

static void prv_reset(void) {
  s_state = RecState_Idle;
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
      .frame_count = s_frame_count,
      .data_bytes = s_data_bytes,
  };
  voice_speex_get_transfer_info(&metadata->speex);
  if (s_samples_per_frame > 0) {
    metadata->duration_ms =
        (uint32_t)((uint64_t)s_frame_count * s_samples_per_frame * 1000 / MIC_SAMPLE_RATE);
  }
}

static void prv_stop_locked(VoiceRecordingId id) {
  if ((s_state != RecState_Recording) || (id != s_active_id)) {
    return;
  }

  new_timer_stop(s_max_timer);
  s_state = RecState_Idle;
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
  }
  PBL_LOG_DBG("Stopped recording id=%u (%s)", (unsigned)id, ok ? "saved" : "failed");
}

static void prv_cancel_locked(VoiceRecordingId id) {
  if ((s_state != RecState_Recording) || (id != s_active_id)) {
    return;
  }

  new_timer_stop(s_max_timer);
  s_state = RecState_Idle;
  mic_stop(MIC);
  prv_close_temp(true);
  prv_reset();
  PBL_LOG_DBG("Cancelled recording id=%u", (unsigned)id);
}

static void prv_max_duration_stop(void *data) {
  voice_recording_stop((VoiceRecordingId)(uintptr_t)data);
}

static void prv_max_duration_timeout(void *data) {
  launcher_task_add_callback(prv_max_duration_stop, data);
}

void voice_recording_init(void) {
  s_lock = mutex_create();
  voice_recording_storage_init(&s_next_id);
  voice_recording_playback_init();
}

VoiceRecordingId voice_recording_start(void) {
  mutex_lock(s_lock);

  VoiceRecordingId id = VOICE_RECORDING_ID_INVALID;
  s_last_error = VoiceRecordingError_None;

  if ((s_state != RecState_Idle) || voice_recording_playback_is_active()) {
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
  if (remaining_budget <= header_size + VOICE_REC_MAX_ENCODED_FRAME + 1) {
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

  id = s_next_id;
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

  s_state = RecState_Recording;
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

void voice_recording_stop(VoiceRecordingId id) {
  mutex_lock(s_lock);
  prv_stop_locked(id);
  mutex_unlock(s_lock);
}

void voice_recording_stop_active(void) {
  mutex_lock(s_lock);
  prv_stop_locked(s_active_id);
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
  mutex_unlock(s_lock);
}

bool voice_recording_in_progress(void) {
  mutex_lock(s_lock);
  const bool recording = (s_state == RecState_Recording);
  mutex_unlock(s_lock);
  return recording;
}

uint32_t voice_recording_list(VoiceRecordingInfo *out, uint32_t max) {
  return voice_recording_storage_list(out, max);
}

uint32_t voice_recording_total_bytes(void) {
  return voice_recording_storage_total_bytes();
}

bool voice_recording_delete(VoiceRecordingId id) {
  mutex_lock(s_lock);
  voice_recording_playback_stop();
  const bool deleted = voice_recording_storage_delete(id);
  mutex_unlock(s_lock);
  return deleted;
}

void voice_recording_delete_all(void) {
  mutex_lock(s_lock);
  voice_recording_playback_stop();
  voice_recording_storage_delete_all();
  mutex_unlock(s_lock);
}

bool voice_recording_play(VoiceRecordingId id) {
  mutex_lock(s_lock);
  const bool started = (s_state == RecState_Idle) && voice_recording_playback_start(id);
  mutex_unlock(s_lock);
  return started;
}

void voice_recording_stop_playback(void) {
  voice_recording_playback_stop();
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
