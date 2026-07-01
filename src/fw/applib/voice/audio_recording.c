/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "audio_recording.h"

#include "pbl/services/voice/voice_recording.h"

AudioRecordingId audio_recording_start(void) {
#ifdef CONFIG_MIC
  return sys_voice_recording_start();
#else
  return AUDIO_RECORDING_ID_INVALID;
#endif
}

void audio_recording_stop(AudioRecordingId recording_id) {
#ifdef CONFIG_MIC
  sys_voice_recording_stop(recording_id);
#endif
}

void audio_recording_cancel(AudioRecordingId recording_id) {
#ifdef CONFIG_MIC
  sys_voice_recording_cancel(recording_id);
#endif
}

bool audio_recording_is_active(void) {
#ifdef CONFIG_MIC
  return sys_voice_recording_in_progress();
#else
  return false;
#endif
}
