/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/voice/voice_recording.h"

#include "board/board.h"
#include "kernel/pbl_malloc.h"
#include <pbl/drivers/mic.h>
#include <pbl/drivers/rtc.h>
#include "kernel/pebble_tasks.h"
#include <pbl/os/mutex.h>
#include <pbl/logging/logging.h>
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/speaker/speaker_service.h"
#include "pbl/services/voice/voice_speex.h"
#include "pbl/services/voice_endpoint.h"
#include "process_management/app_install_manager.h"
#include "process_management/app_manager.h"
#include "syscall/syscall_internal.h"
#include "system/passert.h"
#include <pbl/util/attributes.h>
#include "util/units.h"
#include <pbl/util/uuid.h>

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

PBL_LOG_MODULE_DECLARE(service_voice, CONFIG_SERVICE_VOICE_LOG_LEVEL);

// Container format magic ("VRC1") and version.
#define VOICE_REC_MAGIC (0x56524331)
#define VOICE_REC_CONTAINER_VERSION (1)

// Stored files are named "vrec_<id>"; in-progress captures use "vrecT_<id>" so a
// crash mid-recording leaves an obvious temp file that init() can clean up.
#define VOICE_REC_PREFIX "vrec_"
#define VOICE_REC_TEMP_PREFIX "vrecT_"
#define VOICE_REC_NAME_MAX (16)

// Per-recording and aggregate limits.
#define VOICE_REC_MAX_DURATION_MS (120 * 1000)
#define VOICE_REC_BYTES_PER_SEC (1600)  // Speex 9.8kbps + per-frame overhead, with margin
#define VOICE_REC_TOTAL_STORAGE_BYTES (KiBYTES(512))
#define VOICE_REC_MAX_ENCODED_FRAME (200)
#define VOICE_REC_STAGING_SIZE (1024)

typedef struct PACKED {
  uint32_t magic;
  uint16_t container_version;
  uint16_t channels;
  AudioTransferInfoSpeex speex;  // codec parameters, for replaying frames to the phone later
  uint32_t created;              // unix time the recording started
  Uuid app_uuid;                 // creating app, or UUID_INVALID for system
  uint32_t frame_count;          // backfilled on stop
  uint32_t duration_ms;          // backfilled on stop
  uint32_t data_bytes;           // size of the encoded payload following the header
} VoiceRecordingHeader;

typedef enum {
  RecState_Idle = 0,
  RecState_Recording,
} RecState;

static PebbleMutex *s_lock;
static RecState s_state = RecState_Idle;
static VoiceRecordingId s_active_id = VOICE_RECORDING_ID_INVALID;
static VoiceRecordingId s_next_id = 1;

static int s_temp_fd = -1;
static uint32_t s_data_bytes;
static uint32_t s_frame_count;
static uint32_t s_created;
static Uuid s_app_uuid;
static uint32_t s_samples_per_frame;  // per channel, for duration computation
static uint32_t s_cap_data_bytes;
static bool s_capped;
static VoiceRecordingError s_last_error;

static uint8_t s_staging[VOICE_REC_STAGING_SIZE];
static size_t s_staging_used;

static TimerID s_max_timer = TIMER_INVALID_ID;

// Playback state
#define VOICE_REC_PLAY_FEED_MS (40)
#define VOICE_REC_PLAY_VOLUME (100)
static bool s_playing;
static int s_play_fd = -1;
static TimerID s_play_timer = TIMER_INVALID_ID;
static int16_t *s_play_pcm;        // decoded frame buffer
static uint32_t s_play_pcm_bytes;  // pending PCM bytes still to hand to the speaker
static uint32_t s_play_pcm_off;    // offset into s_play_pcm
static uint32_t s_play_remaining;  // encoded payload bytes left to read

// --------------------------------------------------------------------------------------
// Filename helpers

static void prv_make_name(char *buf, size_t len, const char *prefix, VoiceRecordingId id) {
  snprintf(buf, len, "%s%u", prefix, (unsigned)id);
}

static bool prv_name_has_prefix(const char *name, const char *prefix) {
  return strncmp(name, prefix, strlen(prefix)) == 0;
}

static bool prv_is_recording_file(const char *name) {
  return prv_name_has_prefix(name, VOICE_REC_PREFIX);
}

static bool prv_is_temp_file(const char *name) {
  return prv_name_has_prefix(name, VOICE_REC_TEMP_PREFIX);
}

static bool prv_parse_id(const char *name, VoiceRecordingId *id_out) {
  const char *digits = name + strlen(VOICE_REC_PREFIX);
  if (*digits == '\0') {
    return false;
  }
  char *end = NULL;
  unsigned long val = strtoul(digits, &end, 10);
  if ((end == digits) || (*end != '\0') || (val == 0) || (val > UINT16_MAX)) {
    return false;
  }
  *id_out = (VoiceRecordingId)val;
  return true;
}

// --------------------------------------------------------------------------------------
// Capture (microphone data handler runs on the system task, lock-free)

static bool prv_flush_staging(void) {
  if (s_staging_used == 0) {
    return true;
  }
  int written = pfs_write(s_temp_fd, s_staging, s_staging_used);
  if (written < (int)s_staging_used) {
    PBL_LOG_ERR("Failed to write recording staging buffer (%d)", written);
    return false;
  }
  s_staging_used = 0;
  return true;
}

static void prv_rec_data_handler(int16_t *samples, size_t sample_count, void *context) {
  if (s_state != RecState_Recording || s_capped) {
    return;
  }

  uint8_t encoded[VOICE_REC_MAX_ENCODED_FRAME];
  int encoded_bytes = voice_speex_encode_frame(samples, encoded, sizeof(encoded));
  if (encoded_bytes <= 0) {
    PBL_LOG_DBG("Failed to encode recording frame");
    return;
  }

  const size_t record_size = 1 + (size_t)encoded_bytes;  // 1-byte length prefix + frame

  // Stop accepting data once the pre-allocated file would overflow.
  if (s_data_bytes + record_size > s_cap_data_bytes) {
    PBL_LOG_DBG("Recording reached capacity, dropping further audio");
    s_capped = true;
    return;
  }

  if (s_staging_used + record_size > sizeof(s_staging)) {
    if (!prv_flush_staging()) {
      s_capped = true;
      return;
    }
  }

  s_staging[s_staging_used++] = (uint8_t)encoded_bytes;
  memcpy(&s_staging[s_staging_used], encoded, encoded_bytes);
  s_staging_used += encoded_bytes;

  s_data_bytes += record_size;
  s_frame_count++;
}

// --------------------------------------------------------------------------------------
// Finalize / teardown (caller holds s_lock; microphone already stopped)

static void prv_fill_header(VoiceRecordingHeader *hdr) {
  memset(hdr, 0, sizeof(*hdr));
  hdr->magic = VOICE_REC_MAGIC;
  hdr->container_version = VOICE_REC_CONTAINER_VERSION;
  hdr->channels = (uint16_t)mic_get_channels(MIC);
  voice_speex_get_transfer_info(&hdr->speex);
  hdr->created = s_created;
  hdr->app_uuid = s_app_uuid;
  hdr->frame_count = s_frame_count;
  hdr->data_bytes = s_data_bytes;
  if (s_samples_per_frame > 0) {
    hdr->duration_ms =
        (uint32_t)((uint64_t)s_frame_count * s_samples_per_frame * 1000 / MIC_SAMPLE_RATE);
  }
}

// Copy the encoded payload from the (oversized) temp file into a right-sized final
// file. The temp file must already be closed: PFS fds are single-mode, so the payload
// is read back through a fresh read-only handle.
static bool prv_compact_to_final(VoiceRecordingId id, const VoiceRecordingHeader *hdr) {
  char final_name[VOICE_REC_NAME_MAX];
  char temp_name[VOICE_REC_NAME_MAX];
  prv_make_name(final_name, sizeof(final_name), VOICE_REC_PREFIX, id);
  prv_make_name(temp_name, sizeof(temp_name), VOICE_REC_TEMP_PREFIX, id);

  int temp_fd = pfs_open(temp_name, OP_FLAG_READ, FILE_TYPE_STATIC, 0);
  if (temp_fd < 0) {
    PBL_LOG_ERR("Failed to reopen temp recording for read (%d)", temp_fd);
    s_last_error = VoiceRecordingError_FileOpen;
    return false;
  }

  const uint32_t total = sizeof(*hdr) + s_data_bytes;
  int final_fd = pfs_open(final_name, OP_FLAG_WRITE, FILE_TYPE_STATIC, total);
  if (final_fd < 0) {
    PBL_LOG_ERR("Failed to create recording file %s (%d)", final_name, final_fd);
    s_last_error = VoiceRecordingError_FileOpen;
    pfs_close(temp_fd);
    return false;
  }

  bool ok = (pfs_write(final_fd, hdr, sizeof(*hdr)) == (int)sizeof(*hdr));

  if (ok && (pfs_seek(temp_fd, sizeof(*hdr), FSeekSet) >= 0)) {
    uint8_t buf[256];
    uint32_t remaining = s_data_bytes;
    while (ok && (remaining > 0)) {
      const size_t chunk = (remaining < sizeof(buf)) ? remaining : sizeof(buf);
      if ((pfs_read(temp_fd, buf, chunk) != (int)chunk) ||
          (pfs_write(final_fd, buf, chunk) != (int)chunk)) {
        ok = false;
        break;
      }
      remaining -= chunk;
    }
  } else {
    ok = false;
  }

  pfs_close(final_fd);
  pfs_close(temp_fd);
  if (!ok) {
    s_last_error = VoiceRecordingError_Write;
    pfs_remove(final_name);
  }
  return ok;
}

static void prv_close_temp(bool remove) {
  if (s_temp_fd < 0) {
    return;
  }
  if (remove) {
    pfs_close_and_remove(s_temp_fd);
  } else {
    pfs_close(s_temp_fd);
  }
  s_temp_fd = -1;
}

static void prv_reset(void) {
  s_state = RecState_Idle;
  s_active_id = VOICE_RECORDING_ID_INVALID;
  s_data_bytes = 0;
  s_frame_count = 0;
  s_staging_used = 0;
  s_capped = false;
}

// --------------------------------------------------------------------------------------
// Public API

void voice_recording_init(void) {
  s_lock = mutex_create();

  // Clean up any temp file from an interrupted recording.
  pfs_remove_files(prv_is_temp_file);

  // Prime the id allocator past the highest existing recording.
  PFSFileListEntry *list = pfs_create_file_list(prv_is_recording_file);
  PFSFileListEntry *entry = list;
  while (entry) {
    VoiceRecordingId id;
    if (prv_parse_id(entry->name, &id) && (id >= s_next_id)) {
      s_next_id = id + 1;
    }
    entry = (PFSFileListEntry *)entry->list_node.next;
  }
  pfs_delete_file_list(list);
}

static void prv_max_duration_timeout(void *data) {
  voice_recording_stop((VoiceRecordingId)(uintptr_t)data);
}

VoiceRecordingId voice_recording_start(void) {
  mutex_lock(s_lock);

  VoiceRecordingId id = VOICE_RECORDING_ID_INVALID;
  s_last_error = VoiceRecordingError_None;

  if (s_state != RecState_Idle || s_playing) {
    PBL_LOG_DBG("Recording or playback already in progress");
    s_last_error = VoiceRecordingError_Busy;
    goto unlock;
  }

  // Recording and live dictation share the single microphone.
  if (mic_is_running(MIC)) {
    PBL_LOG_WRN("Microphone busy, cannot start recording");
    s_last_error = VoiceRecordingError_MicBusy;
    goto unlock;
  }

  if (voice_recording_total_bytes() >= VOICE_REC_TOTAL_STORAGE_BYTES) {
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
  s_cap_data_bytes = (VOICE_REC_MAX_DURATION_MS / 1000) * VOICE_REC_BYTES_PER_SEC;

  const uint32_t prealloc = sizeof(VoiceRecordingHeader) + s_cap_data_bytes;
  if (get_available_pfs_space() < prealloc) {
    PBL_LOG_WRN("Not enough flash to start recording (need %" PRIu32 ")", prealloc);
    s_last_error = VoiceRecordingError_NoSpace;
    goto unlock;
  }

  id = s_next_id;

  char temp_name[VOICE_REC_NAME_MAX];
  prv_make_name(temp_name, sizeof(temp_name), VOICE_REC_TEMP_PREFIX, id);
  s_temp_fd = pfs_open(temp_name, OP_FLAG_WRITE, FILE_TYPE_STATIC, prealloc);
  if (s_temp_fd < 0) {
    PBL_LOG_ERR("Failed to create temp recording file (%d)", s_temp_fd);
    s_last_error = VoiceRecordingError_FileOpen;
    id = VOICE_RECORDING_ID_INVALID;
    goto unlock;
  }

  // Determine the creating app (mirrors voice.c's app detection).
  bool from_app = (pebble_task_get_current() == PebbleTask_App) &&
                  !app_install_id_from_system(app_manager_get_current_app_id());
  s_app_uuid = from_app ? app_manager_get_current_app_md()->uuid : UUID_INVALID;
  s_created = (uint32_t)rtc_get_time();
  s_data_bytes = 0;
  s_frame_count = 0;
  s_staging_used = 0;
  s_capped = false;

  // Reserve the header region; it is rewritten with final counts on stop.
  VoiceRecordingHeader placeholder = {0};
  if (pfs_write(s_temp_fd, &placeholder, sizeof(placeholder)) != (int)sizeof(placeholder)) {
    PBL_LOG_ERR("Failed to write recording header placeholder");
    s_last_error = VoiceRecordingError_Write;
    prv_close_temp(true);
    id = VOICE_RECORDING_ID_INVALID;
    goto unlock;
  }

  if (!mic_start(MIC, prv_rec_data_handler, NULL, voice_speex_get_frame_buffer(),
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

  if ((s_state != RecState_Recording) || (id != s_active_id)) {
    goto unlock;
  }

  new_timer_stop(s_max_timer);

  // After mic_stop() returns the data handler will not run again, so the capture
  // state can be finalized without racing it.
  s_state = RecState_Idle;
  mic_stop(MIC);

  bool ok = prv_flush_staging();

  // Commit the temp file (close it) so prv_compact_to_final can reopen it read-only.
  prv_close_temp(false);

  if (!ok) {
    s_last_error = VoiceRecordingError_Write;
  } else {
    VoiceRecordingHeader hdr;
    prv_fill_header(&hdr);
    ok = prv_compact_to_final(id, &hdr);  // sets s_last_error on failure
  }

  // Remove the temp file now that the payload has been copied (or on failure).
  char temp_name[VOICE_REC_NAME_MAX];
  prv_make_name(temp_name, sizeof(temp_name), VOICE_REC_TEMP_PREFIX, id);
  pfs_remove(temp_name);

  prv_reset();

  if (ok) {
    s_last_error = VoiceRecordingError_None;
  }
  PBL_LOG_DBG("Stopped recording id=%u (%s)", (unsigned)id, ok ? "saved" : "failed");

unlock:
  mutex_unlock(s_lock);
}

void voice_recording_stop_active(void) {
  // s_active_id is re-validated under the lock by voice_recording_stop().
  voice_recording_stop(s_active_id);
}

void voice_recording_cancel(VoiceRecordingId id) {
  mutex_lock(s_lock);

  if ((s_state != RecState_Recording) || (id != s_active_id)) {
    goto unlock;
  }

  new_timer_stop(s_max_timer);
  s_state = RecState_Idle;
  mic_stop(MIC);
  prv_close_temp(true);
  prv_reset();

  PBL_LOG_DBG("Cancelled recording id=%u", (unsigned)id);

unlock:
  mutex_unlock(s_lock);
}

bool voice_recording_in_progress(void) {
  return s_state == RecState_Recording;
}

static bool prv_read_info(const char *name, VoiceRecordingInfo *info) {
  int fd = pfs_open(name, OP_FLAG_READ, FILE_TYPE_STATIC, 0);
  if (fd < 0) {
    return false;
  }

  VoiceRecordingHeader hdr;
  bool ok = (pfs_read(fd, &hdr, sizeof(hdr)) == (int)sizeof(hdr)) && (hdr.magic == VOICE_REC_MAGIC);
  if (ok) {
    VoiceRecordingId id;
    info->id = prv_parse_id(name, &id) ? id : VOICE_RECORDING_ID_INVALID;
    info->size_bytes = pfs_get_file_size(fd);
    info->duration_ms = hdr.duration_ms;
    info->created = (time_t)hdr.created;
    info->app_uuid = hdr.app_uuid;
  }
  pfs_close(fd);
  return ok;
}

uint32_t voice_recording_list(VoiceRecordingInfo *out, uint32_t max) {
  if (!out || (max == 0)) {
    return 0;
  }

  uint32_t count = 0;
  PFSFileListEntry *list = pfs_create_file_list(prv_is_recording_file);
  PFSFileListEntry *entry = list;
  while (entry && (count < max)) {
    if (prv_read_info(entry->name, &out[count])) {
      count++;
    }
    entry = (PFSFileListEntry *)entry->list_node.next;
  }
  pfs_delete_file_list(list);
  return count;
}

uint32_t voice_recording_total_bytes(void) {
  uint32_t total = 0;
  PFSFileListEntry *list = pfs_create_file_list(prv_is_recording_file);
  PFSFileListEntry *entry = list;
  while (entry) {
    int fd = pfs_open(entry->name, OP_FLAG_READ, FILE_TYPE_STATIC, 0);
    if (fd >= 0) {
      total += pfs_get_file_size(fd);
      pfs_close(fd);
    }
    entry = (PFSFileListEntry *)entry->list_node.next;
  }
  pfs_delete_file_list(list);
  return total;
}

bool voice_recording_delete(VoiceRecordingId id) {
  char name[VOICE_REC_NAME_MAX];
  prv_make_name(name, sizeof(name), VOICE_REC_PREFIX, id);
  return pfs_remove(name) == S_SUCCESS;
}

void voice_recording_delete_all(void) {
  pfs_remove_files(prv_is_recording_file);
}

// --------------------------------------------------------------------------------------
// Playback (decode Speex frames to PCM and stream them to the speaker)

static void prv_play_cleanup(void) {
  if (s_play_fd >= 0) {
    pfs_close(s_play_fd);
    s_play_fd = -1;
  }
  voice_speex_decoder_deinit();
  if (s_play_pcm) {
    kernel_free(s_play_pcm);
    s_play_pcm = NULL;
  }
  s_play_pcm_bytes = 0;
  s_play_pcm_off = 0;
  s_play_remaining = 0;
  s_playing = false;
}

static void prv_play_feed(void *data) {
  mutex_lock(s_lock);
  if (!s_playing) {
    mutex_unlock(s_lock);
    return;
  }

  bool eof = false;
  while (true) {
    // Hand any leftover decoded PCM to the speaker first.
    if (s_play_pcm_bytes > 0) {
      uint32_t written =
          speaker_service_stream_write((uint8_t *)s_play_pcm + s_play_pcm_off, s_play_pcm_bytes);
      s_play_pcm_off += written;
      s_play_pcm_bytes -= written;
      if (s_play_pcm_bytes > 0) {
        break;  // speaker ring buffer full; resume next tick
      }
    }

    if (s_play_remaining < 1) {
      eof = true;
      break;
    }

    // Read the next [len][frame] record.
    uint8_t len = 0;
    if (pfs_read(s_play_fd, &len, 1) != 1) {
      eof = true;
      break;
    }
    s_play_remaining -= 1;
    if ((len == 0) || (len > s_play_remaining) || (len > VOICE_REC_MAX_ENCODED_FRAME)) {
      eof = true;
      break;
    }

    uint8_t frame[VOICE_REC_MAX_ENCODED_FRAME];
    if (pfs_read(s_play_fd, frame, len) != (int)len) {
      eof = true;
      break;
    }
    s_play_remaining -= len;

    int samples = voice_speex_decode_frame(frame, len, s_play_pcm);
    if (samples > 0) {
      s_play_pcm_bytes = (uint32_t)samples * sizeof(int16_t);
      s_play_pcm_off = 0;
    }
  }

  if (eof && (s_play_pcm_bytes == 0)) {
    speaker_service_stream_close();  // drains remaining buffered audio
    prv_play_cleanup();
  } else {
    new_timer_start(s_play_timer, VOICE_REC_PLAY_FEED_MS, prv_play_feed, NULL, 0);
  }
  mutex_unlock(s_lock);
}

bool voice_recording_play(VoiceRecordingId id) {
  mutex_lock(s_lock);
  bool ok = false;

  if ((s_state == RecState_Recording) || s_playing) {
    goto unlock;
  }

  char name[VOICE_REC_NAME_MAX];
  prv_make_name(name, sizeof(name), VOICE_REC_PREFIX, id);
  s_play_fd = pfs_open(name, OP_FLAG_READ, FILE_TYPE_STATIC, 0);
  if (s_play_fd < 0) {
    goto unlock;
  }

  VoiceRecordingHeader hdr;
  if ((pfs_read(s_play_fd, &hdr, sizeof(hdr)) != (int)sizeof(hdr)) ||
      (hdr.magic != VOICE_REC_MAGIC)) {
    goto cleanup;
  }

  if (!voice_speex_decoder_init()) {
    goto cleanup;
  }

  int frame_size = voice_speex_get_decoder_frame_size();
  if (frame_size <= 0) {
    goto cleanup;
  }
  s_play_pcm = kernel_malloc(frame_size * sizeof(int16_t));
  if (!s_play_pcm) {
    goto cleanup;
  }

  if (!speaker_service_stream_open(SpeakerPriorityApp, VOICE_REC_PLAY_VOLUME,
                                   SpeakerPcmFormat_16kHz_16bit)) {
    goto cleanup;
  }

  s_play_remaining = hdr.data_bytes;
  s_play_pcm_bytes = 0;
  s_play_pcm_off = 0;
  s_playing = true;

  if (s_play_timer == TIMER_INVALID_ID) {
    s_play_timer = new_timer_create();
  }
  new_timer_start(s_play_timer, 1, prv_play_feed, NULL, 0);
  PBL_LOG_DBG("Playing recording id=%u (%" PRIu32 " bytes)", (unsigned)id, hdr.data_bytes);
  ok = true;
  goto unlock;

cleanup:
  voice_speex_decoder_deinit();
  if (s_play_pcm) {
    kernel_free(s_play_pcm);
    s_play_pcm = NULL;
  }
  if (s_play_fd >= 0) {
    pfs_close(s_play_fd);
    s_play_fd = -1;
  }
unlock:
  mutex_unlock(s_lock);
  return ok;
}

void voice_recording_stop_playback(void) {
  mutex_lock(s_lock);
  if (s_playing) {
    new_timer_stop(s_play_timer);
    speaker_service_stop();
    prv_play_cleanup();
  }
  mutex_unlock(s_lock);
}

bool voice_recording_is_playing(void) {
  return s_playing;
}

VoiceRecordingError voice_recording_last_error(void) {
  return s_last_error;
}

// --------------------------------------------------------------------------------------
// Syscalls

DEFINE_SYSCALL(VoiceRecordingId, sys_voice_recording_start, void) {
  return voice_recording_start();
}

DEFINE_SYSCALL(void, sys_voice_recording_stop, VoiceRecordingId id) {
  voice_recording_stop(id);
}

DEFINE_SYSCALL(void, sys_voice_recording_cancel, VoiceRecordingId id) {
  voice_recording_cancel(id);
}

DEFINE_SYSCALL(bool, sys_voice_recording_in_progress, void) {
  return voice_recording_in_progress();
}
