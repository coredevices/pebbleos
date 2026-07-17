# Voice recordings

PebbleOS can record microphone audio without a phone connection, keep the
encoded audio in persistent storage, play it through the watch speaker, and
send it to the phone for speech-to-text. Applications can manage only the
recordings they created. The system settings application and a compatible
companion can manage recordings from every source.

This feature requires a firmware build with `CONFIG_MIC`. Playback additionally
requires a speaker. The application API was added in SDK revision 106
(SDK version major `0x5`, minor `0x67`).

## Architecture

Audio is encoded as Speex frames while it is captured. The encoded frames are
written to a temporary PFS file and copied into an exact-sized recording
container when capture stops successfully.

```text
microphone -> Speex encoder -> temporary PFS file -> recording container
                                                       |       |       |
                                                       |       |       +-> delete
                                                       |       +----------> Speex decoder -> speaker
                                                       +------------------> audio endpoint -> phone STT
```

The valid container header is written last. An interrupted finalize therefore
does not make a partial file appear as a recording. Temporary files and files
with invalid headers are removed when the recording service starts.

Recording, playback, live dictation, and transcription share audio resources.
Only compatible operations can run concurrently; callers must handle a start
operation returning failure when another audio operation is active.

## Application API

Applications use the declarations exported by `voice/audio_recording.h`. A
recording ID is a non-zero `uint16_t`; zero is
`AUDIO_RECORDING_ID_INVALID`.

| API | Purpose |
| --- | --- |
| `audio_recording_start()` | Start capture and return its ID. |
| `audio_recording_stop()` | Finalize the active recording. |
| `audio_recording_cancel()` | Stop capture and discard the temporary audio. |
| `audio_recording_is_active()` | Report whether any capture is active. |
| `audio_recording_list()` | List recordings owned by the calling app. |
| `audio_recording_delete()` | Delete a recording owned by the calling app. |
| `audio_recording_play()` | Play an owned recording through the speaker. |
| `audio_recording_stop_playback()` | Stop app-owned playback. |
| `audio_recording_is_playing()` | Report whether playback is active. |
| `audio_recording_transcribe()` | Upload an owned recording for speech-to-text. |

`AudioRecordingInfo` contains the ID, complete container size, duration in
milliseconds, creation time, and creator UUID. `created` is a Unix timestamp in
seconds.

### Basic capture

Keep the returned ID until capture has either been stopped or cancelled.
Stopping can fail if the ID is not the active recording or if the temporary
audio cannot be finalized.

```c
#include <pebble.h>

static AudioRecordingId s_recording_id = AUDIO_RECORDING_ID_INVALID;

static void start_recording(void) {
  s_recording_id = audio_recording_start();
  if (s_recording_id == AUDIO_RECORDING_ID_INVALID) {
    APP_LOG(APP_LOG_LEVEL_ERROR, "Could not start recording");
  }
}

static void stop_recording(void) {
  if ((s_recording_id != AUDIO_RECORDING_ID_INVALID) &&
      audio_recording_stop(s_recording_id)) {
    s_recording_id = AUDIO_RECORDING_ID_INVALID;
  }
}
```

An application exit cancels an active application recording instead of saving
it. Applications should explicitly call `audio_recording_stop()` before they
exit when the audio must be kept.

### Listing and ownership

The app-facing list and mutating calls enforce the UUID of the current
application. An app cannot discover, play, transcribe, or delete another app's
recordings. Recordings are deleted automatically when their owning app is
uninstalled.

`audio_recording_list()` fills a caller-provided array and returns the number of
entries written. It does not return a total count and does not guarantee a sort
order.

```c
AudioRecordingInfo recordings[8];
const uint32_t count = audio_recording_list(recordings, ARRAY_LENGTH(recordings));

for (uint32_t i = 0; i < count; ++i) {
  APP_LOG(APP_LOG_LEVEL_INFO, "Recording %u: %lu ms",
          recordings[i].id, (unsigned long)recordings[i].duration_ms);
}
```

### Transcription

`audio_recording_transcribe()` opens the normal voice UI, uploads the stored
Speex frames over the audio endpoint, and delivers the speech-to-text result
asynchronously. It requires a connected phone that advertises voice API
support, and currently accepts only mono recordings.

The callback's transcription string is valid only for the duration of the
callback. Copy it before returning if it must be retained. Passing a buffer size
of zero lets the implementation allocate a result buffer when the result
arrives.

```c
static void transcription_callback(AudioRecordingId recording_id,
                                   DictationSessionStatus status,
                                   char *text, void *context) {
  if ((status == DictationSessionStatusSuccess) && text) {
    APP_LOG(APP_LOG_LEVEL_INFO, "Recording %u: %s", recording_id, text);
  }
}

static bool transcribe(AudioRecordingId recording_id) {
  return audio_recording_transcribe(recording_id, 0,
                                    transcription_callback, NULL);
}
```

The return value only reports whether the asynchronous operation started. The
callback reports its final status.

## Limits and system integration

The recording service currently applies these limits:

- A recording is finalized automatically after 120 seconds.
- Valid recording containers share a 1 MiB storage budget.
- Finalization needs enough free PFS space for both the preallocated temporary
  file and the final exact-sized file.
- IDs wrap after `UINT16_MAX`; the allocator probes for an unused ID and never
  overwrites a stored recording.

On microphone-equipped builds, the settings application exposes a temporary
**Voice Memos** module. It can create, list, play, transcribe, and delete system
recordings. It also configures recording quality and recording/playback gain.
These tuning controls are internal and are not part of the application SDK.

## Companion management protocol

A compatible phone can manage recordings over Pebble Protocol endpoint 11000
(`0x2af8`), which is also used by the existing voice control protocol. The
phone sends requests and the watch sends the corresponding response.

### Capability negotiation

The watch sets
`PebbleProtocolCapabilities.voice_recording_management_support` only in builds
with both `CONFIG_MIC` and `CONFIG_SERVICE_VOICE_ENDPOINT`. This is bit 17 of
the system capabilities bitfield. A companion must check this bit before
sending any recording-management message.

### Encoding rules

Messages are packed with no alignment padding. Multi-byte integers use the
firmware's little-endian wire representation. A UUID is sent as its 16 bytes in
canonical order: the first byte contains the first two hexadecimal digits of
the UUID string.

Every recording-management message begins with this six-byte header:

| Offset | Size | Field | Description |
| ---: | ---: | --- | --- |
| 0 | 1 | `msg_id` | Message ID from the table below. |
| 1 | 4 | `flags` | Reserved; send zero. |
| 5 | 1 | `transaction_id` | Chosen by the requester and echoed by the response. |

The recording-management message IDs are:

| ID | Direction | Message |
| ---: | --- | --- |
| `0x10` | phone to watch | List request |
| `0x11` | watch to phone | List response |
| `0x12` | phone to watch | Delete request |
| `0x13` | watch to phone | Delete response |
| `0x14` | phone to watch | Playback request |
| `0x15` | watch to phone | Playback response |
| `0x16` | phone to watch | Transcribe request |
| `0x17` | watch to phone | Transcribe response |

Request handlers accept trailing bytes for forward compatibility, but the
documented fixed fields must all be present.

### Result codes

List and command responses use the same one-byte result values:

| Value | Name | Meaning |
| ---: | --- | --- |
| `0x00` | Success | The request was accepted. |
| `0x01` | Not found | The recording could not be found or used. |
| `0x02` | Busy | Another recording, playback, or transcription operation conflicts. |
| `0x03` | Invalid request | The message is too short or contains an invalid field. |
| `0x04` | Unsupported | Reserved for an unsupported operation. |
| `0x05` | Failed | Reserved for another operation failure. |

`Unsupported` and `Failed` are defined for protocol evolution but are not
currently emitted. Some non-busy playback or transcription start failures are
reported as `Not found`, including an unreadable or unusable recording.

### List recordings

The list request is nine bytes:

| Offset | Size | Field | Description |
| ---: | ---: | --- | --- |
| 0 | 6 | common header | `msg_id` is `0x10`. |
| 6 | 2 | `offset` | Number of valid recordings to skip. |
| 8 | 1 | `limit` | Requested page size; must be non-zero. |

The firmware clamps `limit` to 24. The response has a nine-byte fixed part
followed by `count` metadata entries:

| Offset | Size | Field | Description |
| ---: | ---: | --- | --- |
| 0 | 6 | common header | `msg_id` is `0x11`. |
| 6 | 1 | `result` | Result code. |
| 7 | 1 | `has_more` | Non-zero when another valid recording follows this page. |
| 8 | 1 | `count` | Number of metadata entries that follow. |
| 9 | `count * 30` | `recordings` | Packed metadata entries. |

Each 30-byte metadata entry has this layout:

| Entry offset | Size | Field | Description |
| ---: | ---: | --- | --- |
| 0 | 2 | `id` | Non-zero recording ID. |
| 2 | 4 | `size_bytes` | Complete container size in bytes. |
| 6 | 4 | `duration_ms` | Captured duration in milliseconds. |
| 10 | 4 | `created` | Unix creation timestamp in seconds. |
| 14 | 16 | `app_uuid` | Creator UUID, or all `0xff` for a system recording. |

The response size must equal `9 + count * 30`. On a list error, the response
contains only the fixed nine bytes and `count` is zero.

Listing follows PFS enumeration order and does not provide snapshot semantics.
If recordings are added or removed between offset-based requests, entries can
move between pages. A companion that needs a consistent view should restart at
offset zero after it performs a mutation.

### Delete a recording

The delete request is eight bytes:

| Offset | Size | Field | Description |
| ---: | ---: | --- | --- |
| 0 | 6 | common header | `msg_id` is `0x12`. |
| 6 | 2 | `recording_id` | Recording to delete; zero is invalid. |

The response is the common header followed by `result` at offset 6, for a total
of seven bytes, with `msg_id` `0x13`. Deleting an ID stops active recording
playback before removing the file. A recording that is open for transcription
is not removed and returns `Busy`.

### Control playback

The playback request is nine bytes:

| Offset | Size | Field | Description |
| ---: | ---: | --- | --- |
| 0 | 6 | common header | `msg_id` is `0x14`. |
| 6 | 2 | `recording_id` | Recording to play. Ignored by the stop action. |
| 8 | 1 | `action` | `0x00` to play, `0x01` to stop. |

The seven-byte response uses `msg_id` `0x15` and stores `result` at offset 6.
Stop is idempotent and succeeds even when no recording is playing.

### Transcribe a recording

The transcribe request has the same eight-byte layout as delete, with
`msg_id` `0x16`. `recording_id` must be non-zero. The seven-byte command
response uses `msg_id` `0x17` and stores `result` at offset 6.

A successful response means that the existing dictation flow was started; it
is not the transcription result. The watch sends a voice session setup message
(`0x01`), streams the stored Speex frames over the audio endpoint, and completes
through the existing dictation result flow (`0x02`). The session setup may be
queued before the `0x17` command response, so companions must correlate and
handle both messages without assuming their receive order.

## Implementation map

| Area | Main files |
| --- | --- |
| Application API | `src/fw/applib/voice/audio_recording.c`, `audio_recording.h` |
| Syscall ownership checks | `src/fw/syscall/syscall_audio_recording.c` |
| Capture and lifecycle | `src/fw/services/voice/voice_recording.c` |
| Persistent container | `src/fw/services/voice/voice_recording_storage.c` |
| Speaker playback | `src/fw/services/voice/voice_recording_playback.c` |
| Stored-audio transcription | `src/fw/services/voice/voice.c` |
| Companion commands | `src/fw/services/voice_endpoint/service.c` |
| Protocol structures | `include/pbl/services/voice_endpoint_private.h` |
| System settings UI | `src/fw/apps/system/settings/recordings.c` |
