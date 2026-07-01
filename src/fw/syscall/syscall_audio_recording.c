/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "syscall.h"
#include "syscall_internal.h"

#ifdef CONFIG_MIC
#include "pbl/services/voice/voice_recording.h"
#include "process_management/app_manager.h"
#endif

#ifdef CONFIG_MIC
static bool prv_current_app_owns_recording(AudioRecordingId recording_id) {
  const PebbleProcessMd *app_md = app_manager_get_current_app_md();
  return app_md && voice_recording_is_owned_by(recording_id, &app_md->uuid);
}
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
  if (PRIVILEGE_WAS_ELEVATED && !prv_current_app_owns_recording(recording_id)) {
    return;
  }
  voice_recording_stop(recording_id);
#endif
}

DEFINE_SYSCALL(void, sys_audio_recording_cancel, AudioRecordingId recording_id) {
#ifdef CONFIG_MIC
  if (PRIVILEGE_WAS_ELEVATED && !prv_current_app_owns_recording(recording_id)) {
    return;
  }
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
  if (PRIVILEGE_WAS_ELEVATED && !prv_current_app_owns_recording(recording_id)) {
    return false;
  }
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
