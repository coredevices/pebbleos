/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "voice_recording_playback.h"

#include "voice_recording_storage.h"

#include "kernel/pbl_malloc.h"
#include <pbl/os/mutex.h>
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/speaker/speaker_service.h"
#include "pbl/services/voice/voice_speex.h"
#include <pbl/logging/logging.h>
#include <pbl/util/math.h>

#include <inttypes.h>
#include <stdint.h>

PBL_LOG_MODULE_DECLARE(service_voice, CONFIG_SERVICE_VOICE_LOG_LEVEL);

#define VOICE_REC_PLAY_FEED_MS (40)
#define VOICE_REC_PLAY_VOLUME (100)

static PebbleMutex *s_lock;
static bool s_playing;
static int s_fd = -1;
static TimerID s_timer = TIMER_INVALID_ID;
static int16_t *s_pcm;
static uint32_t s_pcm_bytes;
static uint32_t s_pcm_offset;
static uint32_t s_remaining;
static SpeakerStreamId s_stream_id = SPEAKER_STREAM_ID_INVALID;

static void prv_cleanup(void) {
  if (s_fd >= 0) {
    pfs_close(s_fd);
    s_fd = -1;
  }
  voice_speex_decoder_deinit();
  if (s_pcm) {
    kernel_free(s_pcm);
    s_pcm = NULL;
  }
  s_pcm_bytes = 0;
  s_pcm_offset = 0;
  s_remaining = 0;
  s_stream_id = SPEAKER_STREAM_ID_INVALID;
  s_playing = false;
}

static void prv_feed(void *data) {
  mutex_lock(s_lock);
  if (!s_playing) {
    mutex_unlock(s_lock);
    return;
  }

  bool eof = false;
  while (true) {
    if (s_pcm_bytes > 0) {
      uint32_t written = 0;
      if (!speaker_service_stream_write_session(s_stream_id, (uint8_t *)s_pcm + s_pcm_offset,
                                                s_pcm_bytes, &written)) {
        PBL_LOG_DBG("Recording playback was preempted");
        prv_cleanup();
        mutex_unlock(s_lock);
        return;
      }
      s_pcm_offset += written;
      s_pcm_bytes -= written;
      if (s_pcm_bytes > 0) {
        break;
      }
    }

    if (s_remaining < 1) {
      eof = true;
      break;
    }

    uint8_t len = 0;
    if (pfs_read(s_fd, &len, 1) != 1) {
      eof = true;
      break;
    }
    s_remaining--;
    if ((len == 0) || (len > s_remaining) || (len > VOICE_SPEEX_MAX_ENCODED_FRAME_SIZE)) {
      eof = true;
      break;
    }

    uint8_t frame[VOICE_SPEEX_MAX_ENCODED_FRAME_SIZE];
    if (pfs_read(s_fd, frame, len) != (int)len) {
      eof = true;
      break;
    }
    s_remaining -= len;

    const int samples = voice_speex_decode_frame(frame, len, s_pcm);
    if (samples > 0) {
      s_pcm_bytes = (uint32_t)samples * sizeof(int16_t);
      s_pcm_offset = 0;
    }
  }

  if (eof && (s_pcm_bytes == 0)) {
    speaker_service_stream_close_session(s_stream_id);
    prv_cleanup();
  } else {
    new_timer_start(s_timer, VOICE_REC_PLAY_FEED_MS, prv_feed, NULL, 0);
  }
  mutex_unlock(s_lock);
}

void voice_recording_playback_init(void) {
  s_lock = mutex_create();
  s_playing = false;
  s_fd = -1;
  s_timer = TIMER_INVALID_ID;
  s_pcm = NULL;
  s_pcm_bytes = 0;
  s_pcm_offset = 0;
  s_remaining = 0;
  s_stream_id = SPEAKER_STREAM_ID_INVALID;
}

bool voice_recording_playback_start(VoiceRecordingId id) {
  mutex_lock(s_lock);
  bool ok = false;

  if (s_playing) {
    goto unlock;
  }

  s_fd = voice_recording_storage_open_payload(id, &s_remaining);
  if (s_fd < 0) {
    goto unlock;
  }

  if (!voice_speex_decoder_init()) {
    goto cleanup;
  }

  const int frame_size = voice_speex_get_decoder_frame_size();
  if (frame_size <= 0) {
    goto cleanup;
  }
  s_pcm = kernel_malloc(frame_size * sizeof(int16_t));
  if (!s_pcm) {
    goto cleanup;
  }

  s_stream_id = speaker_service_stream_open_session(SpeakerPriorityApp, VOICE_REC_PLAY_VOLUME,
                                                    SpeakerPcmFormat_16kHz_16bit);
  if (s_stream_id == SPEAKER_STREAM_ID_INVALID) {
    goto cleanup;
  }

  s_pcm_bytes = 0;
  s_pcm_offset = 0;
  s_playing = true;

  if (s_timer == TIMER_INVALID_ID) {
    s_timer = new_timer_create();
  }
  new_timer_start(s_timer, 1, prv_feed, NULL, 0);
  PBL_LOG_DBG("Playing recording id=%u (%" PRIu32 " bytes)", (unsigned)id, s_remaining);
  ok = true;
  goto unlock;

cleanup:
  prv_cleanup();
unlock:
  mutex_unlock(s_lock);
  return ok;
}

void voice_recording_playback_stop(void) {
  mutex_lock(s_lock);
  if (s_playing) {
    new_timer_stop(s_timer);
    speaker_service_stream_stop_session(s_stream_id);
    prv_cleanup();
  }
  mutex_unlock(s_lock);
}

bool voice_recording_playback_is_active(void) {
  mutex_lock(s_lock);
  const bool playing = s_playing;
  mutex_unlock(s_lock);
  return playing;
}
