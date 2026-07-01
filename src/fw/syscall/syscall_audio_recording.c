/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "syscall.h"
#include "syscall_internal.h"

#ifdef CONFIG_MIC
#include "pbl/services/voice/voice_recording.h"
#endif

DEFINE_SYSCALL(AudioRecordingId, sys_audio_recording_start, void) {
#ifdef CONFIG_MIC
  return voice_recording_start();
#else
  return AUDIO_RECORDING_ID_INVALID;
#endif
}

DEFINE_SYSCALL(void, sys_audio_recording_stop, AudioRecordingId recording_id) {
#ifdef CONFIG_MIC
  voice_recording_stop(recording_id);
#endif
}

DEFINE_SYSCALL(void, sys_audio_recording_cancel, AudioRecordingId recording_id) {
#ifdef CONFIG_MIC
  voice_recording_cancel(recording_id);
#endif
}

DEFINE_SYSCALL(bool, sys_audio_recording_is_active, void) {
#ifdef CONFIG_MIC
  return voice_recording_in_progress();
#else
  return false;
#endif
}

DEFINE_SYSCALL(bool, sys_audio_recording_play, AudioRecordingId recording_id) {
#ifdef CONFIG_MIC
  return voice_recording_play(recording_id);
#else
  return false;
#endif
}

DEFINE_SYSCALL(void, sys_audio_recording_stop_playback, void) {
#ifdef CONFIG_MIC
  voice_recording_stop_playback();
#endif
}

DEFINE_SYSCALL(bool, sys_audio_recording_is_playing, void) {
#ifdef CONFIG_MIC
  return voice_recording_is_playing();
#else
  return false;
#endif
}
