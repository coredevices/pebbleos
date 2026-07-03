/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "services/voice/voice_recording_playback.h"

#include "fake_mutex.h"
#include "fake_new_timer.h"
#include "stubs_logging.h"
#include "stubs_passert.h"

#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/speaker/speaker_service.h"

#include <string.h>

static bool s_stream_active;
static bool s_preempt_on_write;
static SpeakerStreamId s_stream_id;
static int s_close_calls;
static int s_decoder_deinit_calls;
static int s_speaker_close_calls;
static int s_speaker_stop_calls;
int voice_recording_storage_open_payload(VoiceRecordingId id, uint32_t *data_bytes_out) {
  *data_bytes_out = 2;
  return 7;
}

int voice_recording_storage_read_frame(int fd, uint32_t *remaining_bytes, uint8_t *frame_out,
                                       size_t frame_out_size) {
  if (*remaining_bytes < 2) {
    return 0;
  }
  frame_out[0] = 0x42;
  *remaining_bytes -= 2;  // length byte + one payload byte
  return 1;
}

uint16_t voice_recording_get_playback_gain(void) {
  return VOICE_RECORDING_GAIN_DEFAULT;
}

status_t pfs_close(int fd) {
  s_close_calls++;
  return S_SUCCESS;
}

bool voice_speex_decoder_init(void) {
  return true;
}

void voice_speex_decoder_deinit(void) {
  s_decoder_deinit_calls++;
}

int voice_speex_get_decoder_frame_size(void) {
  return 2;
}

int voice_speex_decode_frame(const uint8_t *encoded, int len, int16_t *out_pcm) {
  out_pcm[0] = 1;
  out_pcm[1] = 2;
  return 2;
}

SpeakerStreamId speaker_service_stream_open_session(SpeakerPriority priority, uint8_t volume,
                                                    SpeakerPcmFormat format) {
  s_stream_active = true;
  s_stream_id++;
  return s_stream_id;
}

bool speaker_service_stream_write_session(SpeakerStreamId id, const void *data, uint32_t num_bytes,
                                          uint32_t *written_out) {
  if (s_preempt_on_write) {
    // Simulate immediate replacement by another PCM stream. The old reader
    // must release its resources without touching the replacement session.
    s_stream_id++;
    *written_out = 0;
    return false;
  }
  *written_out = num_bytes;
  return s_stream_active && (id == s_stream_id);
}

void speaker_service_stream_close_session(SpeakerStreamId id) {
  if (s_stream_active && (id == s_stream_id)) {
    s_speaker_close_calls++;
    s_stream_active = false;
  }
}

void speaker_service_stream_stop_session(SpeakerStreamId id) {
  if (s_stream_active && (id == s_stream_id)) {
    s_speaker_stop_calls++;
    s_stream_active = false;
  }
}

void test_voice_recording_playback__initialize(void) {
  s_stream_active = false;
  s_preempt_on_write = false;
  s_stream_id = 0;
  s_close_calls = 0;
  s_decoder_deinit_calls = 0;
  s_speaker_close_calls = 0;
  s_speaker_stop_calls = 0;
  voice_recording_playback_init();
}

void test_voice_recording_playback__cleanup(void) {
  voice_recording_playback_stop();
  stub_new_timer_cleanup();
  fake_mutex_reset(true);
}

void test_voice_recording_playback__stop_releases_active_playback(void) {
  cl_assert(voice_recording_playback_start(1));
  cl_assert(voice_recording_playback_is_active());

  voice_recording_playback_stop();

  cl_assert(!voice_recording_playback_is_active());
  cl_assert_equal_i(s_speaker_stop_calls, 1);
  cl_assert_equal_i(s_close_calls, 1);
  cl_assert_equal_i(s_decoder_deinit_calls, 1);
}

void test_voice_recording_playback__preemption_releases_file(void) {
  s_preempt_on_write = true;
  cl_assert(voice_recording_playback_start(1));

  cl_assert(stub_new_timer_fire(stub_new_timer_get_next()));

  cl_assert(!voice_recording_playback_is_active());
  cl_assert(s_stream_active);
  cl_assert_equal_i(s_speaker_stop_calls, 0);
  cl_assert_equal_i(s_close_calls, 1);
  cl_assert_equal_i(s_decoder_deinit_calls, 1);
}

void test_voice_recording_playback__completion_closes_stream(void) {
  cl_assert(voice_recording_playback_start(1));

  cl_assert(stub_new_timer_fire(stub_new_timer_get_next()));

  cl_assert(!voice_recording_playback_is_active());
  cl_assert_equal_i(s_speaker_close_calls, 1);
  cl_assert_equal_i(s_close_calls, 1);
}
