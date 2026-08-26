/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "alarm_detail.h"
#include "alarm_editor.h"

#include "applib/ui/action_menu_window.h"
#include "applib/ui/action_menu_window_private.h"
#include "applib/ui/dialogs/actionable_dialog.h"
#include "applib/ui/dialogs/simple_dialog.h"
#include "kernel/pbl_malloc.h"
#include "popups/health_tracking_ui.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/activity/activity.h"
#include "pbl/services/alarms/alarm.h"
#include <pbl/logging/logging.h>

#ifdef CONFIG_SPEAKER
#include "popups/alarm_popup.h"
#include "services/alarms/alarm_tones.h"
#include "pbl/services/speaker/speaker_service.h"
#endif

#include <stdio.h>

#define NUM_SNOOZE_MENU_ITEMS 5

#ifdef CONFIG_SPEAKER
// Sound submenu: "Off" + one entry per AlarmTone (Reveille, Beacon, Bell, Chime).
#define NUM_SOUND_MENU_ITEMS 5
#endif

typedef enum DetailMenuItemIndex {
  DetailMenuItemIndexEnable = 0,
  DetailMenuItemIndexSnoozeNext,
  DetailMenuItemIndexDelete,
  DetailMenuItemIndexChangeTime,
  DetailMenuItemIndexChangeDays,
  DetailMenuItemIndexConvertSmart,
#ifdef CONFIG_SPEAKER
  DetailMenuItemIndexSound,
#endif
  DetailMenuItemIndexVibration,
  DetailMenuItemIndexSnooze,
  DetailMenuItemIndexNum,
} DetailMenuItemIndex;

typedef struct AlarmDetailData {
  ActionMenuConfig menu_config;

  AlarmId alarm_id;
  AlarmInfo alarm_info;

  AlarmEditorCompleteCallback alarm_editor_callback;
  void *callback_context;
} AlarmDetailData;


static SimpleDialog *prv_snooze_set_confirm_dialog(void) {
  SimpleDialog *simple_dialog = simple_dialog_create("AlarmSnoozeSet");
  Dialog *dialog = simple_dialog_get_dialog(simple_dialog);
  const char *snooze_text = i18n_noop("Snooze delay set to %d minutes");
  char snooze_buf[64];
  snprintf(snooze_buf, sizeof(snooze_buf), i18n_get(snooze_text, dialog), alarm_get_snooze_delay());
  i18n_free(snooze_text, dialog);
  dialog_set_text(dialog, snooze_buf);
  dialog_set_icon(dialog, RESOURCE_ID_GENERIC_CONFIRMATION_LARGE);
  dialog_set_background_color(dialog, GColorJaegerGreen);
  dialog_set_timeout(dialog, DIALOG_TIMEOUT_DEFAULT);
  return simple_dialog;
}

static SimpleDialog *prv_snooze_next_confirm_dialog(bool snoozed) {
  SimpleDialog *simple_dialog = simple_dialog_create("AlarmSnoozeNext");
  Dialog *dialog = simple_dialog_get_dialog(simple_dialog);
  const char *snooze_text = snoozed ? i18n_noop("Next Alarm Snoozed") : i18n_noop("Next Alarm Resumed");
  dialog_set_text(dialog, i18n_get(snooze_text, dialog));
  i18n_free(snooze_text, dialog);
  dialog_set_icon(dialog, RESOURCE_ID_GENERIC_CONFIRMATION_LARGE);
  dialog_set_background_color(dialog, GColorJaegerGreen);
  dialog_set_timeout(dialog, DIALOG_TIMEOUT_DEFAULT);
  return simple_dialog;
}

static void prv_edit_snooze_delay(ActionMenu *action_menu,
                                  const ActionMenuItem *item,
                                  void *context) {
  alarm_set_snooze_delay((uintptr_t)item->action_data);
  SimpleDialog *snooze_delay_dialog = prv_snooze_set_confirm_dialog();
  action_menu_set_result_window(action_menu, (Window *)snooze_delay_dialog);
}

static void prv_toggle_snooze_next_handler(ActionMenu *action_menu, const ActionMenuItem *item,
                                           void *context) {
  AlarmDetailData *data = (AlarmDetailData *) context;
  bool new_snooze = !data->alarm_info.is_snoozed_next;
  alarm_set_snoozed_next(data->alarm_id, new_snooze);
  if (data->alarm_editor_callback) {
    data->alarm_editor_callback(EDITED, data->alarm_id, data->callback_context);
  }
  SimpleDialog *snooze_next_dialog = prv_snooze_next_confirm_dialog(new_snooze);
  action_menu_set_result_window(action_menu, (Window *)snooze_next_dialog);
}

static void prv_toggle_enable_alarm_handler(ActionMenu *action_menu, const ActionMenuItem *item,
                                            void *context) {
  AlarmDetailData *data = (AlarmDetailData *) context;
  alarm_set_enabled(data->alarm_id, !data->alarm_info.enabled);
  if (data->alarm_editor_callback) {
    data->alarm_editor_callback(EDITED, data->alarm_id, data->callback_context);
  }
}

static void prv_toggle_smart_alarm_handler(ActionMenu *action_menu, const ActionMenuItem *item,
                                           void *context) {
  AlarmDetailData *data = context;

  if (!data->alarm_info.is_smart && !activity_prefs_tracking_is_enabled()) {
    // Notify about Health and keep the menu open
    health_tracking_ui_feature_show_disabled();
    return;
  }

  alarm_set_smart(data->alarm_id, !data->alarm_info.is_smart);
  if (data->alarm_editor_callback) {
    data->alarm_editor_callback(EDITED, data->alarm_id, data->callback_context);
  }
}

static void prv_edit_time_handler(ActionMenu *action_menu,
                                 const ActionMenuItem *item,
                                 void *context) {
  AlarmDetailData *data = (AlarmDetailData *) context;
  alarm_editor_update_alarm_time(data->alarm_id,
                                 data->alarm_info.is_smart ? AlarmType_Smart : AlarmType_Basic,
                                 data->alarm_editor_callback, data->callback_context);
}

static void prv_edit_day_handler(ActionMenu *action_menu,
                                 const ActionMenuItem *item,
                                 void *context) {
  AlarmDetailData *data = (AlarmDetailData *) context;
  alarm_editor_update_alarm_days(data->alarm_id, data->alarm_editor_callback,
                                 data->callback_context);
}

static void prv_delete_alarm_handler(ActionMenu *action_menu,
                                     const ActionMenuItem *item,
                                     void *context) {
  AlarmDetailData *data = (AlarmDetailData *) context;
  alarm_delete(data->alarm_id);
  if (data->alarm_editor_callback) {
    data->alarm_editor_callback(DELETED, data->alarm_id, data->callback_context);
  }
}

static void prv_toggle_vibrate_handler(ActionMenu *action_menu, const ActionMenuItem *item,
                                       void *context) {
  AlarmDetailData *data = (AlarmDetailData *) context;
  alarm_set_vibrate_enabled(data->alarm_id, !data->alarm_info.vibrate_enabled);
  if (data->alarm_editor_callback) {
    data->alarm_editor_callback(EDITED, data->alarm_id, data->callback_context);
  }
}

#ifdef CONFIG_SPEAKER
static void prv_disable_sound_handler(ActionMenu *action_menu, const ActionMenuItem *item,
                                      void *context) {
  AlarmDetailData *data = (AlarmDetailData *) context;
  alarm_set_sound_enabled(data->alarm_id, false);
  if (data->alarm_editor_callback) {
    data->alarm_editor_callback(EDITED, data->alarm_id, data->callback_context);
  }
}

static void prv_select_tone_handler(ActionMenu *action_menu, const ActionMenuItem *item,
                                    void *context) {
  AlarmDetailData *data = (AlarmDetailData *) context;
  AlarmTone tone = (AlarmTone)(uintptr_t)item->action_data;
  alarm_set_tone(data->alarm_id, tone);
  alarm_set_sound_enabled(data->alarm_id, true);
  if (data->alarm_editor_callback) {
    data->alarm_editor_callback(EDITED, data->alarm_id, data->callback_context);
  }
}

static void prv_stop_sound_preview(void) {
  speaker_service_stop_for_task(PebbleTask_App);
}

static void prv_sound_highlight_handler(const ActionMenuItem *item, void *context) {
  (void)context;

  if (!item || !item->is_leaf || item->perform_action != prv_select_tone_handler) {
    prv_stop_sound_preview();
    return;
  }

  const SpeakerNote *notes = NULL;
  uint32_t count = 0;
  alarm_tones_get((AlarmTone)(uintptr_t)item->action_data, &notes, &count);
  if (!notes || count == 0) {
    prv_stop_sound_preview();
    return;
  }

  prv_stop_sound_preview();
  speaker_service_set_owner_task(PebbleTask_App);
  speaker_service_play_note_seq(notes, count, SpeakerPriorityApp,
                                ALARM_SPEAKER_VOLUME);
}
#endif

static ActionMenuLevel *prv_create_main_menu(void) {
  ActionMenuLevel *level = task_malloc(sizeof(ActionMenuLevel) +
      DetailMenuItemIndexNum * sizeof(ActionMenuItem));
  if (!level) return NULL;
  *level = (ActionMenuLevel) {
    .num_items = DetailMenuItemIndexNum,
    .parent_level = NULL,
    .display_mode = ActionMenuLevelDisplayModeWide,
  };
  return level;
}

static ActionMenuLevel *prv_create_snooze_menu(ActionMenuLevel *parent_level) {
  ActionMenuLevel *level = task_malloc(sizeof(ActionMenuLevel) +
      NUM_SNOOZE_MENU_ITEMS * sizeof(ActionMenuItem));
  if (!level) return NULL;
  *level = (ActionMenuLevel) {
    .num_items = NUM_SNOOZE_MENU_ITEMS,
    .parent_level = parent_level,
    .display_mode = ActionMenuLevelDisplayModeWide,
  };
  return level;
}

#ifdef CONFIG_SPEAKER
static ActionMenuLevel *prv_create_sound_menu(ActionMenuLevel *parent_level) {
  ActionMenuLevel *level = task_malloc(sizeof(ActionMenuLevel) +
      NUM_SOUND_MENU_ITEMS * sizeof(ActionMenuItem));
  if (!level) return NULL;
  *level = (ActionMenuLevel) {
    .num_items = NUM_SOUND_MENU_ITEMS,
    .parent_level = parent_level,
    .display_mode = ActionMenuLevelDisplayModeWide,
    .selection_changed = prv_sound_highlight_handler,
  };
  return level;
}
#endif

static void prv_alarm_detail_menu_will_close(ActionMenu *action_menu,
                                             const ActionMenuItem *item,
                                             void *context) {
  (void)action_menu;
  (void)item;
  (void)context;

#ifdef CONFIG_SPEAKER
  prv_stop_sound_preview();
#endif
}

void prv_cleanup_alarm_detail_menu(ActionMenu *action_menu,
                                   const ActionMenuItem *item,
                                   void *context) {
  ActionMenuLevel *root_level = action_menu_get_root_level(action_menu);
  AlarmDetailData *data = (AlarmDetailData *) context;
  i18n_free_all(data);
  task_free((void *)root_level->items[DetailMenuItemIndexSnooze].next_level);
#ifdef CONFIG_SPEAKER
  task_free((void *)root_level->items[DetailMenuItemIndexSound].next_level);
#endif
  task_free((void *)root_level);
  task_free(data);
  data = NULL;
}

void alarm_detail_window_push(AlarmId alarm_id, AlarmInfo *alarm_info,
                              AlarmEditorCompleteCallback alarm_editor_callback,
                              void *callback_context) {
  AlarmDetailData* data = task_malloc_check(sizeof(AlarmDetailData));
  *data = (AlarmDetailData) {
    .alarm_id = alarm_id,
    .alarm_info = *alarm_info,
    .alarm_editor_callback = alarm_editor_callback,
    .callback_context = callback_context,
    .menu_config = {
      .context = data,
      .colors.background = ALARMS_APP_HIGHLIGHT_COLOR,
      .will_close = prv_alarm_detail_menu_will_close,
      .did_close = prv_cleanup_alarm_detail_menu,
    },
  };

  // Setup main menu items
  ActionMenuLevel *main_menu = prv_create_main_menu();

  main_menu->items[DetailMenuItemIndexDelete] = (ActionMenuItem) {
    .label = i18n_get("Delete", data),
    .perform_action = prv_delete_alarm_handler,
    .action_data = data,
  };
  main_menu->items[DetailMenuItemIndexEnable] = (ActionMenuItem) {
    .label = data->alarm_info.enabled ? i18n_get("Disable", data) : i18n_get("Enable", data),
    .perform_action = prv_toggle_enable_alarm_handler,
    .action_data = data,
  };
  main_menu->items[DetailMenuItemIndexSnoozeNext] = (ActionMenuItem) {
    .label = data->alarm_info.is_snoozed_next ?
        i18n_get("Resume Next", data) : i18n_get("Snooze Next", data),
    .perform_action = prv_toggle_snooze_next_handler,
    .action_data = data,
  };
  main_menu->items[DetailMenuItemIndexChangeTime] = (ActionMenuItem) {
    .label = i18n_get("Change Time", data),
    .perform_action = prv_edit_time_handler,
    .action_data = data,
  };
  main_menu->items[DetailMenuItemIndexChangeDays] = (ActionMenuItem) {
    .label = i18n_get("Change Days", data),
    .perform_action = prv_edit_day_handler,
    .action_data = data,
  };
  main_menu->items[DetailMenuItemIndexConvertSmart] = (ActionMenuItem) {
    .label = data->alarm_info.is_smart ? i18n_get("Convert to Basic Alarm", data) :
                                         i18n_get("Convert to Smart Alarm", data),
    .perform_action = prv_toggle_smart_alarm_handler,
    .action_data = data,
  };
#ifdef CONFIG_SPEAKER
  main_menu->items[DetailMenuItemIndexSound] = (ActionMenuItem) {
    .label = i18n_get("Sound", data),
    .is_leaf = 0,
    .next_level = prv_create_sound_menu(main_menu),
  };
#endif
  main_menu->items[DetailMenuItemIndexVibration] = (ActionMenuItem) {
    .label = data->alarm_info.vibrate_enabled ? i18n_get("Vibration: On", data)
                                              : i18n_get("Vibration: Off", data),
    .perform_action = prv_toggle_vibrate_handler,
    .action_data = data,
  };
  main_menu->items[DetailMenuItemIndexSnooze] = (ActionMenuItem) {
    .label = i18n_get("Snooze Delay", data),
    .is_leaf = 0,
    .next_level = prv_create_snooze_menu(main_menu),
  };
  main_menu->separator_index = DetailMenuItemIndexSnooze;
  data->menu_config.root_level = main_menu;

  // Setup snooze menu items
  ActionMenuLevel *snooze_level = main_menu->items[DetailMenuItemIndexSnooze].next_level;
  static const unsigned snooze_delays[NUM_SNOOZE_MENU_ITEMS] = {5, 10, 15, 30, 60};
  static const char *snooze_delay_strs[NUM_SNOOZE_MENU_ITEMS] = {
    i18n_noop("5 minutes"),
    i18n_noop("10 minutes"),
    i18n_noop("15 minutes"),
    i18n_noop("30 minutes"),
    i18n_noop("1 hour")
  };

  unsigned current_snooze_delay = alarm_get_snooze_delay();
  for (int i = 0; i < NUM_SNOOZE_MENU_ITEMS; i++) {
    snooze_level->items[i] = (ActionMenuItem) {
      .label = i18n_get(snooze_delay_strs[i], data),
      .perform_action = prv_edit_snooze_delay,
      .action_data = (void *) (uintptr_t) snooze_delays[i],
    };

    if (current_snooze_delay == snooze_delays[i]) {
      snooze_level->default_selected_item = i;
    }
  }

#ifdef CONFIG_SPEAKER
  // Setup sound menu items: Off + one entry per AlarmTone.
  ActionMenuLevel *sound_level = main_menu->items[DetailMenuItemIndexSound].next_level;
  static const AlarmTone tone_values[] = {
    AlarmTone_Reveille, AlarmTone_Beacon, AlarmTone_Bell, AlarmTone_Chime,
  };
  sound_level->items[0] = (ActionMenuItem) {
    .label = i18n_ctx_get("AlarmSound", "Off", data),
    .perform_action = prv_disable_sound_handler,
    .action_data = data,
  };
  const int num_tones = (int)(sizeof(tone_values) / sizeof(tone_values[0]));
  for (int i = 0; i < num_tones; i++) {
    sound_level->items[i + 1] = (ActionMenuItem) {
      .label = i18n_get(alarm_tones_get_name(tone_values[i]), data),
      .perform_action = prv_select_tone_handler,
      .action_data = (void *)(uintptr_t)tone_values[i],
    };
  }
  // Bounds-check the stored tone before indexing the menu. The 3-bit storage
  // field can hold 0..7 but the menu only has num_tones entries; defend
  // against corrupted storage or future tone-table changes.
  int selected = 0;
  if (data->alarm_info.sound_enabled && (int)data->alarm_info.tone < num_tones) {
    selected = (int)data->alarm_info.tone + 1;
  }
  sound_level->default_selected_item = selected;
#endif

  app_action_menu_open(&data->menu_config);
}
