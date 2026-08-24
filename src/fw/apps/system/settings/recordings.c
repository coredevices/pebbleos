/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "recordings.h"

#include "applib/ui/dialogs/expandable_dialog.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/voice/voice_recording.h"
#include "resource/resource_ids.auto.h"

#include <stdio.h>

#ifdef CONFIG_MIC

static Window *prv_init(void) {
  uint32_t used_bytes;
  uint32_t available_bytes;
  voice_recording_get_storage_usage(&used_bytes, &available_bytes);

  const void *i18n_owner = prv_init;
  char used[24];
  char available[24];
  char text[50];
  snprintf(used, sizeof(used), i18n_get("Used %u KB", i18n_owner),
           (unsigned)((used_bytes + 1023) / 1024));
  snprintf(available, sizeof(available), i18n_get("Free %u KB", i18n_owner),
           (unsigned)(available_bytes / 1024));
  snprintf(text, sizeof(text), "%s\n%s", used, available);

  ExpandableDialog *dialog = expandable_dialog_create_with_params(
      "Recordings Storage", RESOURCE_ID_VOICE_MICROPHONE_LARGE, text, GColorBlack, GColorWhite,
      NULL, RESOURCE_ID_ACTION_BAR_ICON_CHECK, expandable_dialog_close_cb);
  expandable_dialog_set_header(dialog, i18n_get("Recordings", i18n_owner));
  expandable_dialog_show_action_bar(dialog, true);
  i18n_free_all(i18n_owner);
  return &dialog->dialog.window;
}

const SettingsModuleMetadata *settings_recordings_get_info(void) {
  static const SettingsModuleMetadata s_module_info = {
      .name = i18n_noop("Recordings"),
      .init = prv_init,
  };

  return &s_module_info;
}

#endif  // CONFIG_MIC
