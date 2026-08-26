/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "syscall.h"
#include "syscall_internal.h"

#include <stddef.h>

#ifdef CONFIG_MIC
#include "pbl/services/voice/voice_recording.h"
#include "process_management/app_manager.h"

// The applib struct is a copy of the service struct (the SDK-exported header must be
// self-contained); the list syscall casts between them, so their layouts must match exactly.
#define ASSERT_FIELD_MATCHES(field) \
  _Static_assert((offsetof(AudioRecordingInfo, field) == offsetof(VoiceRecordingInfo, field)) && \
                 (sizeof(((AudioRecordingInfo *)0)->field) == \
                  sizeof(((VoiceRecordingInfo *)0)->field)), \
                 "AudioRecordingInfo." #field " must match VoiceRecordingInfo." #field)
_Static_assert(sizeof(AudioRecordingInfo) == sizeof(VoiceRecordingInfo),
               "Recording info layouts must match");
ASSERT_FIELD_MATCHES(id);
ASSERT_FIELD_MATCHES(size_bytes);
ASSERT_FIELD_MATCHES(duration_ms);
ASSERT_FIELD_MATCHES(created);
ASSERT_FIELD_MATCHES(app_uuid);
#undef ASSERT_FIELD_MATCHES

static bool prv_current_app_owns_recording(AudioRecordingId recording_id) {
  // Recording IDs are global, so authorize each app operation against the stored creator UUID.
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

DEFINE_SYSCALL(bool, sys_audio_recording_stop, AudioRecordingId recording_id) {
#ifdef CONFIG_MIC
  if (PRIVILEGE_WAS_ELEVATED && !prv_current_app_owns_recording(recording_id)) {
    return false;
  }
  return voice_recording_stop(recording_id);
#else
  return false;
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

DEFINE_SYSCALL(uint32_t, sys_audio_recording_list, AudioRecordingInfo *recordings,
               uint32_t max_recordings) {
#ifdef CONFIG_MIC
  if (!recordings || (max_recordings == 0)) {
    return 0;
  }
  if (max_recordings > SIZE_MAX / sizeof(*recordings)) {
    syscall_failed();
  }
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(recordings, max_recordings * sizeof(*recordings));
    const PebbleProcessMd *app_md = app_manager_get_current_app_md();
    if (!app_md) {
      return 0;
    }
    return voice_recording_list_owned_by((VoiceRecordingInfo *)recordings, max_recordings,
                                         &app_md->uuid);
  }
  return voice_recording_list((VoiceRecordingInfo *)recordings, max_recordings);
#else
  return 0;
#endif
}

DEFINE_SYSCALL(bool, sys_audio_recording_delete, AudioRecordingId recording_id) {
#ifdef CONFIG_MIC
  if (PRIVILEGE_WAS_ELEVATED && !prv_current_app_owns_recording(recording_id)) {
    return false;
  }
  return voice_recording_delete(recording_id);
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
  // Apps may only stop the playback they started themselves, never the system's or another
  // app's.
  if (PRIVILEGE_WAS_ELEVATED) {
    const PebbleProcessMd *app_md = app_manager_get_current_app_md();
    if (!app_md || !voice_recording_playback_owned_by(&app_md->uuid)) {
      return;
    }
  }
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
