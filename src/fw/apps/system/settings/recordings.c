/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "recordings.h"
#include "menu.h"

#include "applib/app.h"
#include "applib/ui/option_menu_window.h"
#include "applib/ui/ui.h"
#include "kernel/pbl_malloc.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/voice/voice_recording.h"
#include "shell/prefs.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#ifdef CONFIG_MIC

// TODO(voice-recording): The top "Record/Stop" row is a temporary affordance so the
// feature can be exercised from the watch without a companion app. Remove it once
// recording is driven by its real trigger (SDK app / dedicated UI). Search this file
// for VOICE_REC_TEST_TRIGGER to delete every related bit.
#define VOICE_REC_TEST_TRIGGER 1

#define SETTINGS_RECORDINGS_MAX_SHOWN (64)

typedef struct {
  OptionMenu option_menu;
  VoiceRecordingInfo infos[SETTINGS_RECORDINGS_MAX_SHOWN];
  uint32_t count;
  char row_buf[40];
#if VOICE_REC_TEST_TRIGGER
  VoiceRecordingId active_id;
#endif
} SettingsRecordingsData;

#if VOICE_REC_TEST_TRIGGER
// Row 0 is the record/stop toggle; recordings (and the trailing "Delete all" row)
// follow it, so list indices are shifted by one.
#define VOICE_REC_ROW_OFFSET (1)
#else
#define VOICE_REC_ROW_OFFSET (0)
#endif

#if VOICE_REC_TEST_TRIGGER
// Short on-screen labels for diagnosing why a test recording failed.
static const char *prv_error_label(VoiceRecordingError e) {
  switch (e) {
    case VoiceRecordingError_Busy:
      return "Busy";
    case VoiceRecordingError_MicBusy:
      return "Mic busy";
    case VoiceRecordingError_StorageFull:
      return "Storage full";
    case VoiceRecordingError_Codec:
      return "Codec failed";
    case VoiceRecordingError_NoSpace:
      return "No flash space";
    case VoiceRecordingError_FileOpen:
      return "File open failed";
    case VoiceRecordingError_Write:
      return "Write failed";
    case VoiceRecordingError_MicStart:
      return "Mic start failed";
    case VoiceRecordingError_Save:
      return "Save failed";
    case VoiceRecordingError_None:
    default:
      return NULL;
  }
}
#endif

static void prv_reload(SettingsRecordingsData *data) {
  data->count = voice_recording_list(data->infos, SETTINGS_RECORDINGS_MAX_SHOWN);
  option_menu_reload_data(&data->option_menu);
}

// One trailing "Delete all" row is shown when there is at least one recording.
static bool prv_is_delete_all_row(SettingsRecordingsData *data, uint32_t row) {
  return (data->count > 0) && (row == VOICE_REC_ROW_OFFSET + data->count);
}

static uint16_t prv_get_num_rows_cb(OptionMenu *option_menu, void *context) {
  SettingsRecordingsData *data = context;
  const uint16_t list_rows = (data->count == 0) ? 1 : (data->count + 1);
  return VOICE_REC_ROW_OFFSET + list_rows;
}

static void prv_draw_row_cb(OptionMenu *option_menu, GContext *ctx, const Layer *cell_layer,
                            const GRect *text_frame, uint32_t row, bool selected, void *context) {
  SettingsRecordingsData *data = context;
  const char *title;

#if VOICE_REC_TEST_TRIGGER
  if (row == 0) {
    if (voice_recording_in_progress()) {
      title = i18n_get("Stop recording", data);
    } else {
      const char *err = prv_error_label(voice_recording_last_error());
      title = err ? err : i18n_get("Record", data);
    }
  } else
#endif
      if (data->count == 0) {
    title = i18n_get("No recordings", data);
  } else if (prv_is_delete_all_row(data, row)) {
    title = i18n_get("Delete all", data);
  } else {
    const VoiceRecordingInfo *info = &data->infos[row - VOICE_REC_ROW_OFFSET];
    const uint32_t secs = info->duration_ms / 1000;
    const uint32_t kb = (info->size_bytes + 1023) / 1024;
    snprintf(data->row_buf, sizeof(data->row_buf), "%" PRIu32 ":%02" PRIu32 " · %" PRIu32 " KB",
             secs / 60, secs % 60, kb);
    title = data->row_buf;
  }

  option_menu_system_draw_row(option_menu, ctx, cell_layer, text_frame, title, false, NULL);
}

static uint16_t prv_row_height_cb(OptionMenu *option_menu, uint16_t row, bool is_selected,
                                  void *context) {
  return option_menu_default_cell_height(option_menu->content_type, is_selected);
}

static void prv_select_cb(OptionMenu *option_menu, int row, void *context) {
  SettingsRecordingsData *data = context;

#if VOICE_REC_TEST_TRIGGER
  if (row == 0) {
    if (voice_recording_in_progress()) {
      voice_recording_stop(data->active_id);
      prv_reload(data);  // re-list: the finished recording now appears below
    } else {
      data->active_id = voice_recording_start();
      option_menu_reload_data(&data->option_menu);  // refresh the row 0 label
    }
    return;
  }
#endif

  if (data->count == 0) {
    return;
  }

  if (prv_is_delete_all_row(data, (uint32_t)row)) {
    voice_recording_delete_all();
    prv_reload(data);
  } else {
    // Tap a recording to play it back through the speaker.
    voice_recording_play(data->infos[row - VOICE_REC_ROW_OFFSET].id);
  }
}

static void prv_unload_cb(OptionMenu *option_menu, void *context) {
  SettingsRecordingsData *data = context;
#if VOICE_REC_TEST_TRIGGER
  // Finalize any in-progress recording so leaving the page doesn't leave the
  // service stuck in the recording state (the mic keeps running otherwise).
  if (voice_recording_in_progress()) {
    voice_recording_stop_active();
  }
#endif
  option_menu_deinit(&data->option_menu);
  i18n_free_all(data);
  app_free(data);
}

static Window *prv_init(void) {
  SettingsRecordingsData *data = app_zalloc_check(sizeof(SettingsRecordingsData));

  const OptionMenuCallbacks callbacks = {
      .unload = prv_unload_cb,
      .draw_row = prv_draw_row_cb,
      .select = prv_select_cb,
      .get_num_rows = prv_get_num_rows_cb,
      .get_cell_height = prv_row_height_cb,
  };

  option_menu_init(&data->option_menu);
  option_menu_set_status_colors(&data->option_menu, GColorWhite, GColorBlack);
  const GColor highlight_bg = shell_prefs_get_theme_highlight_color();
  option_menu_set_highlight_colors(&data->option_menu, highlight_bg,
                                   gcolor_legible_over(highlight_bg));
  option_menu_set_title(&data->option_menu, i18n_get("Record audio (beta)", data));
  option_menu_set_content_type(&data->option_menu, OptionMenuContentType_SingleLine);
  option_menu_set_callbacks(&data->option_menu, &callbacks, data);

  prv_reload(data);

  return &data->option_menu.window;
}

const SettingsModuleMetadata *settings_recordings_get_info(void) {
  static const SettingsModuleMetadata s_module_info = {
      .name = i18n_noop("Record audio (beta)"),
      .init = prv_init,
  };

  return &s_module_info;
}

#endif  // CONFIG_MIC
