/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/notifications/notification_sounds.h"

#include "pbl/services/i18n/i18n.h"
#include "pbl/services/speaker/speaker_service.h"
#include "pbl/services/speaker/track.h"

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

#include "notification_sounds_pcm.inc"

// Bell — sampled strike played through the track player. The note pitches the
// sample; at the sample's base note it plays unshifted.
static const SpeakerSample s_bell_sample = {
  .data = s_bell_pcm,
  .num_bytes = sizeof(s_bell_pcm),
  .format = SpeakerPcmFormat_16kHz_8bit,
  .base_midi_note = 84,  // C6, the sample's own fundamental
  .loop = false,
};

static const SpeakerNote s_bell_notes[] = {
  // Waveform is ignored for sampled tracks; duration covers the full strike.
  { .midi_note = 84, .waveform = 0, .duration_ms = 420, .velocity = 0, .reserved = 0 },
};

static const struct {
  const SpeakerNote *notes;
  uint32_t count;
  const char *name;
  const SpeakerSample *sample;  // non-NULL: play via the track player
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
  [NotificationSound_Bell]     = { s_bell_notes, sizeof(s_bell_notes) / sizeof(s_bell_notes[0]),
                                   i18n_noop("Bell"), &s_bell_sample },
};

_Static_assert(NotificationSound_Bell + 1 == NotificationSound_Count,
               "notification_sounds table must cover every NotificationSound enum value");

bool notification_sounds_play(NotificationSound sound, uint8_t volume) {
  if (sound == NotificationSound_None) {
    return false;
  }
  if ((unsigned)sound >= NotificationSound_Count) {
    sound = NotificationSound_Ping;
  }
  if (s_sounds[sound].sample) {
    const SpeakerTrack track = {
      .notes = s_sounds[sound].notes,
      .num_notes = s_sounds[sound].count,
      .sample = s_sounds[sound].sample,
    };
    return speaker_service_play_tracks(&track, 1, SpeakerPriorityNotification, volume);
  }
  return speaker_service_play_note_seq(s_sounds[sound].notes, s_sounds[sound].count,
                                       SpeakerPriorityNotification, volume);
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
