/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "remote.h"
#include "bluetooth.h"

#include "applib/ui/action_menu_window_private.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/ui.h"
#include "applib/ui/dialogs/dialog.h"
#include "applib/ui/dialogs/expandable_dialog.h"
#include "applib/ui/dialogs/simple_dialog.h"
#include "kernel/pbl_malloc.h"
#include "kernel/ui/system_icons.h"
#include "popups/ble_hrm/ble_hrm_stop_sharing_popup.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/bluetooth/bluetooth_persistent_storage.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/bluetooth/ble_hrm.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

enum {
  RemoteMenuForget = 0,
#ifdef CONFIG_HRM
  RemoteMenuStopSharingHeartRate,
#endif
  RemoteMenu_Count
};

typedef struct {
  StoredRemote remote;

  ActionMenuConfig action_menu;
  struct SettingsBluetoothData *bt_data;
} SettingsRemoteData;

static void prv_dialog_unload(void *context) {
  PBL_ASSERTN(context);
  i18n_free_all(context);
}

static void prv_show_dialog(void *i18n_owner) {
  DialogCallbacks callback = { .unload = prv_dialog_unload };
  const char *text = BT_FORGET_PAIRING_STR;
  ExpandableDialog *e_dialog = expandable_dialog_create_with_params(
      "Forget Remote", RESOURCE_ID_GENERIC_CONFIRMATION_TINY, i18n_get(text, i18n_owner),
      GColorWhite, GColorCobaltBlue, &callback, RESOURCE_ID_ACTION_BAR_ICON_CHECK,
      expandable_dialog_close_cb);
  i18n_free(text, i18n_owner);

  expandable_dialog_show_action_bar(e_dialog, true);
  expandable_dialog_set_header(e_dialog, i18n_get("You're all set", e_dialog));

  app_expandable_dialog_push(e_dialog);
}

static void prv_forget_ble_remote(int id) {
  bt_persistent_storage_delete_ble_pairing_by_id(id);
}

static void prv_remote_menu_cleanup(ActionMenu *action_menu,
                                    const ActionMenuItem *item,
                                    void *context) {
  ActionMenuLevel *root_level = action_menu_get_root_level(action_menu);
  SettingsRemoteData *data = (SettingsRemoteData *) context;
  i18n_free_all(data);
  task_free((void *) root_level);
  task_free(data);
}

static void prv_forget_item(ActionMenu *action_menu,
                            const ActionMenuItem *item,
                            void *context) {
  SettingsRemoteData* remote_data = (SettingsRemoteData*) context;
  StoredRemote* remote = &remote_data->remote;
  prv_forget_ble_remote(remote->ble.bonding);
  PBL_LOG_INFO("User Forgot BT Pairing");
  PBL_LOG_DBG("Name: %s", remote->name);
  settings_bluetooth_update_remotes(remote_data->bt_data);
  prv_show_dialog(context);
}

#ifdef CONFIG_HRM
static void prv_stop_sharing_heart_rate(ActionMenu *action_menu,
                                        const ActionMenuItem *item,
                                        void *context) {
  SettingsRemoteData *remote_data = (SettingsRemoteData*) context;
  StoredRemote *remote = &remote_data->remote;

  ble_hrm_revoke_sharing_permission_for_connection(remote->ble.connection);

  app_simple_dialog_push(ble_hrm_stop_sharing_popup_create());
}
#endif  // CONFIG_HRM

void settings_remote_menu_push(struct SettingsBluetoothData *bt_data, StoredRemote *stored_remote) {
  SettingsRemoteData *data = app_malloc_check(sizeof(SettingsRemoteData));

  PBL_LOG_DBG("NAME: %s", stored_remote->name);
  *data = (SettingsRemoteData){};
  data->remote = *stored_remote;
  data->bt_data = bt_data;

  data->action_menu = (ActionMenuConfig) {
    .context = data,
    .colors.background = shell_prefs_get_theme_highlight_color(),
    .did_close = prv_remote_menu_cleanup,
  };

#ifdef CONFIG_HRM
  const bool is_sharing_hr =
      settings_bluetooth_is_sharing_heart_rate_for_stored_remote(stored_remote);
  const size_t num_items = RemoteMenu_Count - (is_sharing_hr ? 0 : 1);
#else
  const size_t num_items = RemoteMenu_Count;
#endif
  ActionMenuLevel *level =
      task_zalloc_check(sizeof(ActionMenuLevel) + num_items * sizeof(ActionMenuItem));
  *level = (ActionMenuLevel) {
    .num_items = num_items,
    .display_mode = ActionMenuLevelDisplayModeWide,
  };

  level->items[RemoteMenuForget] = (ActionMenuItem) {
    .label = i18n_get("Forget", data),
    .perform_action = prv_forget_item,
    .action_data = data,
  };

#ifdef CONFIG_HRM
  if (is_sharing_hr) {
    level->items[RemoteMenuStopSharingHeartRate] = (ActionMenuItem) {
      .label = i18n_get("Stop Sharing Heart Rate", data),
      .perform_action = prv_stop_sharing_heart_rate,
      .action_data = data,
    };
  }
#endif

  data->action_menu.root_level = level;
  app_action_menu_open(&data->action_menu);
}
