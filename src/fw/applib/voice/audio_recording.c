/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "audio_recording.h"

#include "syscall/syscall.h"

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
