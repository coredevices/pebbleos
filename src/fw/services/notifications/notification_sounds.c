/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/notifications/notification_sounds.h"

#include "pbl/services/i18n/i18n.h"

#define NOTE(midi, wave, ms) \
  { .midi_note = (midi), .waveform = (wave), .duration_ms = (ms), .velocity = 0, .reserved = 0 }

// MIDI: C5=72, E5=76, G5=79, B5=83, C6=84.

// Ping — a single soft sine C6. The least intrusive option.
static const SpeakerNote s_ping[] = {
  NOTE(84, SpeakerWaveformSine, 200),
};

// Doorbell — classic two-note ding-dong, E5 down to C5.
static const SpeakerNote s_doorbell[] = {
  NOTE(76, SpeakerWaveformSine, 250),
  NOTE(0,  SpeakerWaveformSine, 40),
  NOTE(72, SpeakerWaveformSine, 400),
};

// Trill — SMS-style double beep on B5 squares.
static const SpeakerNote s_trill[] = {
  NOTE(83, SpeakerWaveformSquare, 80),
  NOTE(0,  SpeakerWaveformSquare, 50),
  NOTE(83, SpeakerWaveformSquare, 80),
};

// Ascent — quick rising triangle flourish, C5-E5-G5.
static const SpeakerNote s_ascent[] = {
  NOTE(72, SpeakerWaveformTriangle, 100),
  NOTE(76, SpeakerWaveformTriangle, 100),
  NOTE(79, SpeakerWaveformTriangle, 220),
};

#undef NOTE

static const struct {
  const SpeakerNote *notes;
  uint32_t count;
  const char *name;
} s_sounds[NotificationSound_Count] = {
  [NotificationSound_None]     = { NULL, 0, i18n_noop("Off") },
  [NotificationSound_Ping]     = { s_ping, sizeof(s_ping) / sizeof(s_ping[0]),
                                   i18n_noop("Ping") },
  [NotificationSound_Doorbell] = { s_doorbell, sizeof(s_doorbell) / sizeof(s_doorbell[0]),
                                   i18n_noop("Doorbell") },
  [NotificationSound_Trill]    = { s_trill, sizeof(s_trill) / sizeof(s_trill[0]),
                                   i18n_noop("Trill") },
  [NotificationSound_Ascent]   = { s_ascent, sizeof(s_ascent) / sizeof(s_ascent[0]),
                                   i18n_noop("Ascent") },
};

_Static_assert(NotificationSound_Ascent + 1 == NotificationSound_Count,
               "notification_sounds table must cover every NotificationSound enum value");

void notification_sounds_get(NotificationSound sound, const SpeakerNote **notes_out,
                             uint32_t *count_out) {
  if ((unsigned)sound >= NotificationSound_Count || sound == NotificationSound_None) {
    sound = NotificationSound_Ping;
  }
  *notes_out = s_sounds[sound].notes;
  *count_out = s_sounds[sound].count;
}

const char *notification_sounds_get_name(NotificationSound sound) {
  if ((unsigned)sound >= NotificationSound_Count) {
    sound = NotificationSound_Ping;
  }
  return s_sounds[sound].name;
}

NotificationSound notification_sounds_cycle_next(NotificationSound sound) {
  return (NotificationSound)(((unsigned)sound + 1) % NotificationSound_Count);
}
