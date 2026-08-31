/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "alarm_popup.h"

#include "applib/ui/dialogs/dialog.h"
#include "applib/ui/dialogs/simple_dialog.h"
#include "applib/ui/dialogs/actionable_dialog.h"
#include "applib/ui/vibes.h"
#include "kernel/event_loop.h"
#include "kernel/low_power.h"
#include "kernel/pbl_malloc.h"
#include "kernel/ui/modals/modal_manager.h"
#include "resource/resource_ids.auto.h"
#include <pbl/logging/logging.h>
#include "pbl/services/clock.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/light.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/alarms/alarm.h"
#include "util/time/time.h"

#include <stdio.h>

#include "pbl/services/vibes/vibe_client.h"
#include "pbl/services/vibes/vibe_score.h"

#ifdef CONFIG_SPEAKER
#include "applib/event_service_client.h"
#include "kernel/events.h"
#include "pbl/services/notifications/alerts_preferences_private.h"
#include "pbl/services/speaker/speaker_finish_reason.h"
#include "pbl/services/speaker/speaker_service.h"
#include "services/alarms/alarm_tones.h"
#endif

#define DIALOG_TIMEOUT_SNOOZE 2000
#define DIALOG_TIMEOUT_DISMISS DIALOG_TIMEOUT_SNOOZE

#define ALARM_PRIORITY (ModalPriorityAlarm)

static WindowStack *prv_get_window_stack(void) {
  return modal_manager_get_window_stack(ALARM_PRIORITY);
}

// ----------------------------------------------------------------------------------------------
//! Snooze confirm dialog

static void prv_show_snooze_confirm_dialog(void) {
  SimpleDialog *simple_dialog = simple_dialog_create("AlarmSnooze");
  Dialog *dialog = simple_dialog_get_dialog(simple_dialog);
  const char *snooze_text = i18n_noop("Snooze for %d minutes");
  char snooze_buf[32];
  snprintf(snooze_buf, sizeof(snooze_buf), i18n_get(snooze_text, dialog), alarm_get_snooze_delay());
  i18n_free(snooze_text, dialog);
  dialog_set_text(dialog, snooze_buf);
  dialog_set_icon(dialog, RESOURCE_ID_GENERIC_CONFIRMATION_LARGE);
  dialog_set_background_color(dialog, GColorJaegerGreen);
  dialog_set_timeout(dialog, DIALOG_TIMEOUT_SNOOZE);
  simple_dialog_push(simple_dialog, prv_get_window_stack());
}

// ----------------------------------------------------------------------------------------------
//! Dismiss confirm dialog

static void prv_show_dismiss_confirm_dialog(void) {
  SimpleDialog *simple_dialog = simple_dialog_create("AlarmSnooze");
  Dialog *dialog = simple_dialog_get_dialog(simple_dialog);
  const char *dismiss_text = i18n_noop("Alarm dismissed");
  dialog_set_text(dialog, i18n_get(dismiss_text, dialog));
  i18n_free(dismiss_text, dialog);
  dialog_set_icon(dialog, RESOURCE_ID_GENERIC_CONFIRMATION_LARGE);
  dialog_set_background_color(dialog, GColorJaegerGreen);
  dialog_set_timeout(dialog, DIALOG_TIMEOUT_DISMISS);
  simple_dialog_push(simple_dialog, prv_get_window_stack());
}

// ----------------------------------------------------------------------------------------------
//! Main Window

typedef struct {
  ActionableDialog *alarm_popup;
  GBitmap *bitmap;
  GBitmap *action_bar_dismiss;
  GBitmap *action_bar_snooze;
  ActionBarLayer action_bar;

  TimerID vibe_timer;
  int max_vibes;
  int vibe_count;
  VibeScore *vibe_score;

#ifdef CONFIG_SPEAKER
  // Speaker playback state. sound_active is the single source of truth: it must
  // be cleared *before* speaker_service_stop() so any in-flight finish event
  // bails out instead of re-triggering playback. When sound_driven_by_vibe_timer
  // is set, replays are issued from the vibe outer timer instead of the speaker
  // Done event, so audio and vibe stay anchored to the same drift-free clock.
  bool sound_active;
  bool sound_driven_by_vibe_timer;
  const SpeakerNote *sound_notes;
  uint32_t sound_count;
  EventServiceInfo speaker_event_info;
#endif
} AlarmPopupData;

AlarmPopupData *s_alarm_popup_data = NULL;


static void prv_stop_animation_kernel_main_cb(void *callback_context) {
  if (s_alarm_popup_data) {
    dialog_set_icon((Dialog *) s_alarm_popup_data->alarm_popup,
                    RESOURCE_ID_ALARM_CLOCK_LARGE_STATIC);
  }
}

static void prv_stop_vibes(void) {
  if (s_alarm_popup_data->vibe_timer != TIMER_INVALID_ID) {
    new_timer_stop(s_alarm_popup_data->vibe_timer);
    new_timer_delete(s_alarm_popup_data->vibe_timer);
    s_alarm_popup_data->vibe_timer = TIMER_INVALID_ID;
    if (s_alarm_popup_data->vibe_score) {
      vibe_score_destroy(s_alarm_popup_data->vibe_score);
      s_alarm_popup_data->vibe_score = NULL;
    }
  }
  vibes_cancel();
}

#ifdef CONFIG_SPEAKER
static void prv_play_sound_loop_iteration(void) {
  speaker_service_play_note_seq(s_alarm_popup_data->sound_notes,
                                s_alarm_popup_data->sound_count,
                                SpeakerPriorityCritical, ALARM_SPEAKER_VOLUME);
}

static void prv_handle_speaker_event(PebbleEvent *e, void *context) {
  if (!s_alarm_popup_data || !s_alarm_popup_data->sound_active) {
    return;
  }
  if (e->speaker.type != SpeakerEventFinished) {
    return;
  }
  if (s_alarm_popup_data->sound_driven_by_vibe_timer) {
    // Replays are issued from the vibe timer; ignore Done/Preempted here.
    return;
  }
  if ((SpeakerFinishReason)e->speaker.finish_reason == SpeakerFinishReasonDone) {
    // Sound-only mode: alarm rings continuously until dismiss/snooze/timeout.
    prv_play_sound_loop_iteration();
  }
}

static void prv_start_sound(AlarmId id, bool vibe_driving_loop) {
  if (low_power_is_active()) {
    return;  // Skip sound in LPM to conserve battery; vibration still runs.
  }

  AlarmInfo info;
  if (!alarm_get_info(id, &info) || !info.sound_enabled) {
    return;
  }

  alarm_tones_get(info.tone, &s_alarm_popup_data->sound_notes, &s_alarm_popup_data->sound_count);
  if (!s_alarm_popup_data->sound_notes || s_alarm_popup_data->sound_count == 0) {
    return;
  }

  s_alarm_popup_data->speaker_event_info = (EventServiceInfo) {
    .type = PEBBLE_SPEAKER_EVENT,
    .handler = prv_handle_speaker_event,
  };
  event_service_client_subscribe(&s_alarm_popup_data->speaker_event_info);
  speaker_service_register_finish(PebbleTask_KernelMain);

  // Only piggyback on the vibe outer timer when the selected tone has been
  // laid out beat-for-beat against the user's selected alarm vibe (see the
  // paired_vibe column in s_tones[]). For any other combination the cycle
  // lengths and beats won't line up, so each side loops on its own cadence.
  const VibeScoreId paired = alarm_tones_get_paired_vibe(info.tone);
  const bool tones_match = vibe_driving_loop &&
                           paired != VibeScoreId_Invalid &&
                           paired == alerts_preferences_get_vibe_score_for_client(VibeClient_Alarms);
  s_alarm_popup_data->sound_active = true;
  s_alarm_popup_data->sound_driven_by_vibe_timer = tones_match;
  if (!tones_match) {
    prv_play_sound_loop_iteration();
  }
  // When the vibe timer is driving the loop, the first audio play happens
  // inside the vibe timer's callback so both subsystems are anchored to the
  // same clock.
}

static void prv_stop_sound(void) {
  if (!s_alarm_popup_data->sound_active) {
    return;
  }
  // Order matters: clear the flag first so any in-flight finish event bails
  // out before stop() generates one.
  s_alarm_popup_data->sound_active = false;
  speaker_service_stop();
  event_service_client_unsubscribe(&s_alarm_popup_data->speaker_event_info);
}
#endif  // CONFIG_SPEAKER

// ----------------------------------------------------------------------------------------------
//! Vibe Timer
#define TINTIN_VIBE_REPEAT_INTERVAL_MS (1000)
#define TINTIN_MAX_VIBES (10 * 60) // 10 minutes at 1 vibe a second
#define TINTIN_LPM_VIBES_PER_MINUTE (10)
#define VIBE_DURATION (10 * SECONDS_PER_MINUTE * MS_PER_SECOND)
static void prv_vibe_kernel_main_cb(void *callback_context) {
  if (s_alarm_popup_data) {
    if (s_alarm_popup_data->vibe_count < s_alarm_popup_data->max_vibes) {
      s_alarm_popup_data->vibe_count++;
      vibes_cancel();
      if (s_alarm_popup_data->vibe_score) {
        vibe_score_do_vibe(s_alarm_popup_data->vibe_score);
      } else {
        vibes_long_pulse();
      }
#ifdef CONFIG_SPEAKER
      if (s_alarm_popup_data->sound_active &&
          s_alarm_popup_data->sound_driven_by_vibe_timer) {
        prv_play_sound_loop_iteration();
      }
#endif
    }
    else {
      prv_stop_vibes();
      launcher_task_add_callback(prv_stop_animation_kernel_main_cb, NULL);
      // Auto-dismiss the alarm after the vibration period ends
      alarm_dismiss_alarm();
      if (s_alarm_popup_data && s_alarm_popup_data->alarm_popup) {
        actionable_dialog_pop(s_alarm_popup_data->alarm_popup);
      }
    }
  }
}

static void prv_vibe(void *unused) {
  launcher_task_add_callback(prv_vibe_kernel_main_cb, NULL);
}

static void prv_start_vibes(void) {
  s_alarm_popup_data->vibe_count = 0;
  if (low_power_is_active()) {
    s_alarm_popup_data->vibe_score = vibe_client_get_score(VibeClient_AlarmsLPM);
  } else {
    s_alarm_popup_data->vibe_score = vibe_client_get_score(VibeClient_Alarms);
  }
  unsigned int vibe_repeat_interval_ms = 0;
  if (s_alarm_popup_data->vibe_score) {
    vibe_repeat_interval_ms = vibe_score_get_duration_ms(s_alarm_popup_data->vibe_score) +
        vibe_score_get_repeat_delay_ms(s_alarm_popup_data->vibe_score);
  }
  if (vibe_repeat_interval_ms == 0) {
    // Missing (e.g. resource load failure under memory pressure) or empty
    // score: an alarm must never be silent, so pulse on a fixed cadence.
    PBL_LOG_WRN("No usable alarm vibe score; falling back to fixed pulse");
    if (s_alarm_popup_data->vibe_score) {
      vibe_score_destroy(s_alarm_popup_data->vibe_score);
      s_alarm_popup_data->vibe_score = NULL;
    }
    vibe_repeat_interval_ms = low_power_is_active()
        ? (SECONDS_PER_MINUTE * MS_PER_SECOND / TINTIN_LPM_VIBES_PER_MINUTE)
        : TINTIN_VIBE_REPEAT_INTERVAL_MS;
  }
  s_alarm_popup_data->max_vibes = DIVIDE_CEIL(VIBE_DURATION, vibe_repeat_interval_ms);
  s_alarm_popup_data->vibe_timer = new_timer_create();
  prv_vibe(NULL);
  new_timer_start(s_alarm_popup_data->vibe_timer, vibe_repeat_interval_ms, prv_vibe,
                  NULL, TIMER_START_FLAG_REPEATING);
}

// ----------------------------------------------------------------------------------------------
//! Click Handler
static void prv_dismiss_click_handler(ClickRecognizerRef recognizer, void *data) {
  alarm_dismiss_alarm();
  actionable_dialog_pop(s_alarm_popup_data->alarm_popup);
  prv_show_dismiss_confirm_dialog();
}

static void prv_snooze_click_handler(ClickRecognizerRef recognizer, void *data) {
  alarm_set_snooze_alarm();
  actionable_dialog_pop(s_alarm_popup_data->alarm_popup);
  prv_show_snooze_confirm_dialog();
}

static void prv_click_provider(void *context) {
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_dismiss_click_handler);
  window_single_click_subscribe(BUTTON_ID_UP, prv_snooze_click_handler);
  window_single_click_subscribe(BUTTON_ID_BACK, prv_snooze_click_handler);
}

// ----------------------------------------------------------------------------------------------
//! Main Window Setup
static void prv_setup_action_bar(void) {
  ActionBarLayer *action_bar = &s_alarm_popup_data->action_bar;
  action_bar_layer_init(action_bar);
  action_bar_layer_set_background_color(action_bar, GColorBlack);

  s_alarm_popup_data->action_bar_snooze =
      gbitmap_create_with_resource(RESOURCE_ID_ACTION_BAR_ICON_SNOOZE);
  s_alarm_popup_data->action_bar_dismiss =
      gbitmap_create_with_resource(RESOURCE_ID_ACTION_BAR_ICON_X);
  action_bar_layer_set_icon(action_bar, BUTTON_ID_UP, s_alarm_popup_data->action_bar_snooze);
  action_bar_layer_set_icon(action_bar, BUTTON_ID_DOWN, s_alarm_popup_data->action_bar_dismiss);
  action_bar_layer_set_click_config_provider(action_bar, prv_click_provider);
}

static void prv_cleanup_alarm_popup(void *callback_context) {
  if (s_alarm_popup_data) {
    prv_stop_vibes();
#ifdef CONFIG_SPEAKER
    prv_stop_sound();
#endif
    // The action bar owns a redraw timer armed on button press; it must be
    // cancelled before the layer's memory goes away. actionable_dialog leaves
    // custom action bars to their owner.
    action_bar_layer_deinit(&s_alarm_popup_data->action_bar);
    gbitmap_destroy(s_alarm_popup_data->bitmap);
    gbitmap_destroy(s_alarm_popup_data->action_bar_snooze);
    gbitmap_destroy(s_alarm_popup_data->action_bar_dismiss);
    task_free(s_alarm_popup_data);
    s_alarm_popup_data = NULL;
  }
}

// ----------------------------------------------------------------------------------------------
//! API
void alarm_popup_push_window(PebbleAlarmClockEvent *event) {
  if (s_alarm_popup_data) {
    // The window is already visible, don't show another one
    return;
  }

  s_alarm_popup_data = task_malloc_check(sizeof(AlarmPopupData));
  *s_alarm_popup_data = (AlarmPopupData){};
  s_alarm_popup_data->vibe_timer = TIMER_INVALID_ID;

  prv_setup_action_bar();

  s_alarm_popup_data->alarm_popup = actionable_dialog_create("Alarm Popup");
  actionable_dialog_set_action_bar_type(s_alarm_popup_data->alarm_popup, DialogActionBarCustom,
      &s_alarm_popup_data->action_bar);

  Dialog *dialog = actionable_dialog_get_dialog(s_alarm_popup_data->alarm_popup);
  char display_time[16];
  struct tm alarm_tm;
  localtime_r(&event->alarm_time, &alarm_tm);
  if (clock_is_24h_style()) {
    strftime(display_time, 16, "%H:%M", &alarm_tm);
  } else {
    strftime(display_time, 16, "%I:%M %p", &alarm_tm);
  }
  dialog_set_text(dialog, display_time);
  dialog_set_icon(dialog, RESOURCE_ID_ALARM_CLOCK_LARGE);
  dialog_set_background_color(dialog, GColorJaegerGreen);
  DialogCallbacks callback = {
    .unload = prv_cleanup_alarm_popup,
  };
  dialog_set_callbacks(dialog, &callback, NULL);
  actionable_dialog_push(s_alarm_popup_data->alarm_popup, prv_get_window_stack());

  // The alarm id isn't carried in the event (PebbleEvent must stay <= 12 bytes
  // — see _Static_assert in events.c). Fall back to the most-recently-fired
  // alarm tracked in the alarm service. Synthetic events (trigger_alarm demo)
  // get ALARM_INVALID_ID and skip the per-alarm config lookup.
  const AlarmId alarm_id = alarm_get_most_recent_id();
  AlarmInfo info;
  const bool have_info = (alarm_id != ALARM_INVALID_ID) &&
                         alarm_get_info(alarm_id, &info);
  const bool vibe_on = !have_info || info.vibrate_enabled;
  if (vibe_on) {
    prv_start_vibes();
  }
#ifdef CONFIG_SPEAKER
  if (have_info) {
    // The paired-tone piggyback assumes the timer runs at the score's cycle
    // length; in fallback-pulse mode (score == NULL) it doesn't, so let the
    // sound loop on its own.
    prv_start_sound(alarm_id, vibe_on && (s_alarm_popup_data->vibe_score != NULL));
  }
#endif

  light_enable_interaction();
}
