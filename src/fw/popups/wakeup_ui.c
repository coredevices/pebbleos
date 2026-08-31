/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "wakeup_ui.h"

#include "applib/fonts/fonts.h"
#include "applib/ui/dialogs/expandable_dialog.h"
#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"
#include "kernel/ui/kernel_ui.h"
#include "kernel/ui/modals/modal_manager.h"
#include "process_management/app_install_manager.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"

typedef void (*EachCb)(AppInstallEntry *entry, void *data);

void prv_find_len_helper(AppInstallEntry *entry, void *context) {
  uint32_t *len = context;
  *len += strlen(entry->name) + 1;
}

void prv_string_builder(AppInstallEntry *entry, void *context) {
  char *str = context;
  strcat(str, entry->name);
  strcat(str, "\n");
}

void prv_each_app_ids(uint32_t num_ids, AppInstallId *ids, EachCb cb, void *context) {
  for (uint32_t i = 0; i < num_ids; i++) {
    // app_title +1 to add newline per line
    AppInstallId app_id = ids[i];

    AppInstallEntry entry;
    if (!app_install_get_entry_for_install_id(app_id, &entry)) {
      continue;
    }

    cb(&entry, context);
  }
}

typedef struct {
  uint8_t count;
  AppInstallId *app_ids;
} WakeupUICbData;

static void prv_show_dialog(void *context) {
  WakeupUICbData *data = context;

  const char* missed_text_raw =
      i18n_noop("While your Pebble was off wakeup events occurred for:\n");
  const char* missed_text = i18n_get(missed_text_raw, data);

  // Find the size of all of the missed_apps names (no max length defined)
  int16_t missed_app_titles_len = 0;

  prv_each_app_ids(data->count, data->app_ids, prv_find_len_helper, &missed_app_titles_len);

  int16_t missed_message_len = strlen(missed_text) + missed_app_titles_len;
  char *missed_message = kernel_zalloc(missed_message_len + 1);

  strncpy(missed_message, missed_text, strlen(missed_text));
  i18n_free(missed_text, data);

  prv_each_app_ids(data->count, data->app_ids, prv_string_builder, missed_message);
  missed_message[missed_message_len] = '\0';

  // must free the buffer passed in
  kernel_free(data->app_ids);
  kernel_free(data);

  ExpandableDialog * ex_dialog = expandable_dialog_create(NULL);
  Dialog *dialog = expandable_dialog_get_dialog(ex_dialog);
  dialog_set_text_buffer(dialog, missed_message, true);
  dialog_set_icon(dialog, RESOURCE_ID_GENERIC_WARNING_TINY);
  dialog_show_status_bar_layer(dialog, true);
  expandable_dialog_push(ex_dialog, modal_manager_get_window_stack(ModalPriorityGeneric));
}

// ---------------------------------------------------------------------------
// Display our alert
void wakeup_popup_window(uint8_t missed_apps_count, AppInstallId *missed_app_ids) {
  WakeupUICbData *data = kernel_malloc(sizeof(WakeupUICbData));
  if (data) {
    *data = (WakeupUICbData) {
      .count = missed_apps_count,
      .app_ids = missed_app_ids,
    };
    launcher_task_add_callback(prv_show_dialog, data);
  }
  // don't really care if it fails.
}
