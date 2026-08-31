/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "voice_window.h"
#include "voice_window_private.h"

#include "loading_layer.h"
#include "transcription_dialog.h"

#include "applib/applib_malloc.auto.h"
#include "applib/app_timer.h"
#include "applib/event_service_client.h"
#include "applib/fonts/fonts.h"
#include "applib/graphics/text_render.h"
#include "applib/graphics/utf8.h"
#include "applib/ui/animation.h"
#include "applib/ui/animation_interpolate.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/dialogs/bt_conn_dialog.h"
#include "applib/ui/dialogs/dialog.h"
#include "applib/ui/dialogs/dialog_private.h"
#include "applib/ui/dialogs/simple_dialog.h"
#include "applib/ui/dialogs/expandable_dialog.h"
#include "applib/ui/status_bar_layer.h"
#include "applib/ui/window_stack.h"
#include "applib/ui/layer.h"
#include "applib/ui/action_bar_layer.h"
#include "applib/ui/kino/kino_reel.h"
#include "applib/ui/kino/kino_reel/transform.h"
#include "applib/ui/kino/kino_reel/unfold.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/scroll_layer.h"
#if (0) // https://pebbletechnology.atlassian.net/browse/PBL-20406
#include "applib/ui/vibes.h"
#endif
#include "kernel/ui/kernel_ui.h"
#include "kernel/ui/modals/modal_manager.h"
#include "process_state/app_state/app_state.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/event_service.h"
#include "pbl/services/voice/voice.h"
#include "syscall/syscall.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "syscall/syscall.h"
#include "syscall/syscall_internal.h"
#include "pbl/util/size.h"

#include <string.h>

// TODO:
// mic hot before showing screen - needs robust beginning-of-speech detection - https://pebbletechnology.atlassian.net/browse/PBL-16474
// animated microphone icon - https://pebbletechnology.atlassian.net/browse/PBL-16481
// handle line wrapping - https://pebbletechnology.atlassian.net/browse/PBL-16475
// Brief vibration just before microphone is turned on https://pebbletechnology.atlassian.net/browse/PBL-20406


#define DICTATION_TIMEOUT (15 * 1000)     // 15s timeout for each dictation
#define SPEECH_DETECTION_TIMEOUT (3 * 1000)

// Session must last at least 600ms before reporting an error to the user
#define MIN_ELAPSED_DURATION (600)

#define TEXT_PADDING (4)
#define MIC_DOT_MAX_RADIUS  (9)
#define MIC_DOT_LAYER_RADIUS (MIC_DOT_MAX_RADIUS + 1)
#define MIC_DOT_LAYER_SIZE ((GSize){ .w = MIC_DOT_LAYER_RADIUS * 2, .h = MIC_DOT_LAYER_RADIUS * 2})

#define MAX_MESSAGE_LEN (500)

#define MAX_ERROR_COUNT (4)

#define ERROR_DIALOG_TIMEOUT (5000)

static const uint32_t UNFOLD_DURATION = 500;

static void prv_start_dictation(VoiceUiData *data);
static void prv_stop_dictation(VoiceUiData *data);
static void prv_cancel_dictation(VoiceUiData *data);
static void prv_set_mic_window_state(VoiceUiData *data, VoiceUiState state);
static PropertyAnimation *prv_create_int16_prop_anim(int16_t from, int16_t to,
                                                     uint32_t duration,
                                                     const PropertyAnimationImplementation *impl,
                                                     void *subject);
static void prv_handle_stop_transition(VoiceUiData *data);
static void prv_voice_window_push(VoiceUiData *data);
char *sys_voice_get_transcription_from_event(PebbleVoiceServiceEvent *e, char *buffer,
                                             size_t buffer_size, size_t *sentence_len);



static WindowStack *prv_get_window_stack(void) {
  if (pebble_task_get_current() == PebbleTask_App) {
    return app_state_get_window_stack();
  }
  return modal_manager_get_window_stack(ModalPriorityVoice);
}


static void prv_window_push(Window *window) {
  window_stack_push(prv_get_window_stack(), window, true /* animated */);
}

static void prv_window_pop(Window *window) {
  window_stack_remove(window, true);
}

static void prv_teardown(VoiceUiData *data) {
  // The state is only set to StateExiting in this function, so check that teardown has not already
  // been performed before carrying on
  if (data->state == StateExiting) {
    return;
  }
  prv_set_mic_window_state(data, StateExiting);

  if (window_is_loaded(&data->mic_window.window)) {
    prv_window_pop(&data->mic_window.window);
  } else {
    window_deinit(&data->mic_window.window);
  }
}

static void prv_exit_and_send_result_event(VoiceUiData *data, DictationSessionStatus result) {
  PBL_LOG_DBG("Send result");

  PebbleEvent event = {
    .type = PEBBLE_DICTATION_EVENT,
    .dictation = {
      .result = result,
      .text = (result == DictationSessionStatusSuccess) ? data->message : NULL,
      .timestamp = (result == DictationSessionStatusSuccess) ? data->timestamp : 0,
    }
  };
  sys_send_pebble_event_to_kernel(&event);

  sys_light_reset_to_timed_mode();

  prv_teardown(data);
}

static void prv_handle_error_retries(VoiceUiData *data) {
  if (data->error_count < MAX_ERROR_COUNT) {
    PBL_LOG_DBG("Restarting dictation after error");
    prv_start_dictation(data);
  } else {
    PBL_LOG_DBG("Too many errors! Exiting...");
    prv_exit_and_send_result_event(data, data->error_exit_status);
  }
}

static void prv_error_dialog_unload(void *context) {
  VoiceUiData *data = context;
  prv_handle_error_retries(data);
}

static void prv_init_dialog(VoiceUiData *data, Dialog *dialog, const char *text,
    uint32_t resource_id, bool has_timeout, GColor color) {
  dialog_set_callbacks(dialog, &(DialogCallbacks) {
    .unload = prv_error_dialog_unload,
  }, data);
  sys_i18n_get_with_buffer(text, data->error_text_buffer, sizeof(data->error_text_buffer));
  dialog_set_text(dialog, data->error_text_buffer);
  dialog_set_icon(dialog, resource_id);
  dialog_set_background_color(dialog, color);
  dialog_set_timeout(dialog, has_timeout ? ERROR_DIALOG_TIMEOUT : DIALOG_TIMEOUT_INFINITE);
  dialog_set_destroy_on_pop(dialog, false /* free_on_pop */);
}

static void prv_push_error_dialog(VoiceUiData *data, const char *text, uint32_t resource_id,
    GColor color) {
  prv_set_mic_window_state(data, StateError);

  SimpleDialog *simple_dialog = &data->short_error_dialog;
  simple_dialog_init(simple_dialog, "Dictation Error");
  Dialog *dialog = simple_dialog_get_dialog(simple_dialog);
  prv_init_dialog(data, dialog, text, resource_id, true, color);
  simple_dialog_push(simple_dialog, prv_get_window_stack());
}

static void prv_push_long_error_dialog(VoiceUiData *data, const char* header, const char *text,
    uint32_t resource_id) {
  prv_set_mic_window_state(data, StateError);

  ExpandableDialog *long_error_dialog = &data->long_error_dialog;
  expandable_dialog_init(long_error_dialog, "Error");
  Dialog *dialog = expandable_dialog_get_dialog(long_error_dialog);
  const GColor dialog_bg_color = PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite);
  prv_init_dialog(data, dialog, text, resource_id, false, dialog_bg_color);
  expandable_dialog_set_header(long_error_dialog, header);
  expandable_dialog_push(long_error_dialog, prv_get_window_stack());
}

static void prv_push_final_error_dialog(VoiceUiData *data) {
  prv_push_error_dialog(data, i18n_noop("Dictation is not available."),
                        RESOURCE_ID_GENERIC_WARNING_LARGE, GColorRed);
}

static void prv_show_error_dialog(VoiceUiData *data, const char *msg) {
  if (data->show_error_dialog) {
    if (data->error_count == MAX_ERROR_COUNT) {
      data->error_exit_status = DictationSessionStatusFailureSystemAborted;
      prv_push_final_error_dialog(data);
    }
    else {
      const GColor dialog_bg_color = PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite);
      prv_push_error_dialog(data, msg, RESOURCE_ID_GENERIC_WARNING_LARGE, dialog_bg_color);
    }
  } else {
    prv_exit_and_send_result_event(data, DictationSessionStatusFailureSystemAborted);
  }
}

static void prv_show_generic_error_dialog(VoiceUiData *data) {
  prv_show_error_dialog(data, i18n_noop("Error occurred. Try again."));
}

static void prv_show_connectivity_error_and_exit(VoiceUiData *data) {
  data->error_count = MAX_ERROR_COUNT;   // exit UI after the dialog is shown
  if (data->show_error_dialog) {
    const GColor dialog_bg_color = PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite);
    prv_push_error_dialog(data, i18n_noop("No internet connection"),
                          RESOURCE_ID_CHECK_INTERNET_CONNECTION_LARGE, dialog_bg_color);
    data->error_exit_status = DictationSessionStatusFailureConnectivityError;
  } else {
    prv_exit_and_send_result_event(data, DictationSessionStatusFailureConnectivityError);
  }
}

static void prv_handle_bt_conn_result(bool connected, void *context) {
  VoiceUiData *data = context;
  if (connected) {
    if (data->state == StateError) {
      // We got here after a dictation result timeout, so restart the dictation
      prv_start_dictation(data);
    } else {
      prv_voice_window_push(data);
    }
  } else {
    PBL_LOG_DBG("Bluetooth not restored! Exiting...");
    prv_exit_and_send_result_event(data, DictationSessionStatusFailureConnectivityError);
  }
}

static void prv_push_bt_dialog(VoiceUiData *data) {
  BtConnDialog *bt_dialog = &data->bt_dialog;
  bt_conn_dialog_init(bt_dialog, data->error_text_buffer, sizeof(data->error_text_buffer));

  Dialog *dialog = &bt_dialog->dialog.dialog;
  dialog_set_destroy_on_pop(dialog, false /* free_on_pop */);

  bt_conn_dialog_push(bt_dialog, prv_handle_bt_conn_result, data);
}

static uint64_t prv_get_time_ms(void) {
  time_t now_s;
  uint16_t now_ms;
  sys_get_time_ms(&now_s, &now_ms);
  uint64_t ms = (now_s * 1000) + now_ms;
  return ms;
}

static void prv_update_analytics_metrics(VoiceUiData *data) {
  data->elapsed_ms = prv_get_time_ms() - data->start_ms;
  data->num_sessions++;
}

static void prv_dictation_timeout_cb(void *context) {
  VoiceUiData *data = context;
  PBL_LOG_DBG("Single session timeout");
  prv_stop_dictation(data);
}

static void prv_handle_ready_error(VoiceUiData *data) {
  data->session_id = VOICE_SESSION_ID_INVALID;

  data->error_count++;
  if (data->error_count < MAX_ERROR_COUNT) {
    if (data->show_error_dialog) {
      prv_handle_error_retries(data);
    } else {
      prv_exit_and_send_result_event(data, DictationSessionStatusFailureSystemAborted);
    }
  } else {
    prv_show_generic_error_dialog(data);
  }
}

static void prv_handle_ready_event(VoiceUiData *data, PebbleVoiceServiceEvent *event) {
  PBL_LOG_DBG("Handling ready event");

  switch (event->status) {
    case VoiceStatusSuccess:
      PBL_LOG_DBG("Session setup successfully");
      data->start_ms = prv_get_time_ms();
      data->speech_detected = false;
      data->dictation_timeout = app_timer_register(DICTATION_TIMEOUT, prv_dictation_timeout_cb,
          data);

      // Update UI
      prv_set_mic_window_state(data, StateRecording);
      break;

    case VoiceStatusErrorConnectivity:
      // Subsequent attempts are probably going to result in the same error. Let the user sort
      // out the error and re-enter the dialog.
      prv_show_connectivity_error_and_exit(data);
      break;

    case VoiceStatusErrorDisabled:
      // This should happen before loading the window, but we currently do not have a mechanism to
      // tell the watch whether or not voice reply is enabled
      data->error_count = MAX_ERROR_COUNT;   // exit UI after the dialog is shown
      if (data->show_error_dialog) {
        prv_push_long_error_dialog(data, NULL, i18n_noop("Enable voice in the settings page of the Pebble app."),
                                   RESOURCE_ID_GENERIC_WARNING_TINY);
        data->error_exit_status = DictationSessionStatusFailureDisabled;
      } else {
        prv_exit_and_send_result_event(data, DictationSessionStatusFailureDisabled);
      }
      break;

    case VoiceStatusErrorGeneric:
    case VoiceStatusTimeout:
      PBL_LOG_DBG("Session setup error %d", event->status);
      prv_handle_ready_error(data);
      break;
    default:
      WTF;
  }
  if (event->status != VoiceStatusSuccess) {
    data->last_session_successful = false;
  }
}

static bool prv_handle_dictation_success(VoiceUiData *data, PebbleVoiceServiceEvent *event) {
  if (data->buffer_size == 0) {
    // If buffer size is set to 0, the buffer was allocated when the last transcription was received
    applib_free(data->message);
    data->message = NULL;
  }
  data->message = sys_voice_get_transcription_from_event(event, data->message, data->buffer_size,
      &data->message_len);
  if (data->session_type == VoiceEndpointSessionTypeNLP) {
    data->timestamp = event->data->timestamp;
  }

  if (!data->message) {
    PBL_LOG_DBG("Empty sentence received");
    return false;
  }

  PBL_LOG_DBG("New sentence: %s", data->message);
  return true;
}

static void prv_handle_dictation_error(VoiceUiData *data, VoiceStatus error_status) {
  uint64_t elapsed = prv_get_time_ms() - data->start_ms;
  const bool speech_detected = (data->speech_detected) || (elapsed < MIN_ELAPSED_DURATION);

  data->error_count++;

  if (data->show_error_dialog) {
    if (error_status == VoiceStatusRecognizerResponseError) {
      prv_show_error_dialog(data, i18n_noop("Missed that. Try again."));
    } else {
      prv_show_generic_error_dialog(data);
    }
  } else {
    if (!speech_detected) {
      PBL_LOG_DBG("No speech detected! Exiting...");
      prv_exit_and_send_result_event(data, DictationSessionStatusFailureNoSpeechDetected);
    } else {
      prv_exit_and_send_result_event(data, DictationSessionStatusFailureRecognizerError);
    }
  }
}

static void prv_handle_dictation_result(VoiceUiData *data, PebbleVoiceServiceEvent *event) {
  PBL_LOG_DBG("Handling result event");
  data->session_id = VOICE_SESSION_ID_INVALID;
  bool success = false;
  switch (event->status) {
    case VoiceStatusSuccess: {
      success = prv_handle_dictation_success(data, event);
      if (success) {
        if (data->state == StateRecording) {
          // Transition to unfold state (StateStopRecording) before pending a transition to the text
          // window
          prv_set_mic_window_state(data, StateTransitionToText);
        }
        prv_set_mic_window_state(data, StateTransitionToText);
      } else {
        prv_handle_dictation_error(data, event->status);
      }
      break;
    }

    case VoiceStatusErrorConnectivity:
      prv_show_connectivity_error_and_exit(data);
      break;

    case VoiceStatusErrorGeneric:
      PBL_LOG_DBG("Result: error %"PRId8, event->status);
      prv_handle_dictation_error(data, event->status);
      break;

    case VoiceStatusRecognizerResponseError:
      PBL_LOG_DBG("Result: speech not recognized");
      prv_handle_dictation_error(data, event->status);
      break;

    case VoiceStatusTimeout:
      PBL_LOG_DBG("Result: timeout");
      if (!connection_service_peek_pebble_app_connection()) {
        data->error_count++;
        if (data->error_count < MAX_ERROR_COUNT) {
          prv_set_mic_window_state(data, StateError);
          prv_push_bt_dialog(data);
        } else {
          prv_push_final_error_dialog(data);
          data->error_exit_status = DictationSessionStatusFailureConnectivityError;
        }
      } else {
        prv_handle_dictation_error(data, event->status);
      }
      break;

    default:
      WTF;
  }
  data->last_session_successful = success;
}

// Only use stable state for determining how to handle input events and voice service events
static VoiceUiState prv_get_simple_state(VoiceUiState state) {
  static const VoiceUiState state_map[] = {
    StateStart,
    StateWaitForReady,      // StateStartWaitForReady
    StateWaitForReady,
    StateRecording,
    StateWaitForResponse,   // StateStopRecording
    StateWaitForResponse,
    StateTransitionToText,  // StateStopWaitForResponse
    StateTransitionToText,
    StateError,
    StateFinished,
    StateExiting,
  };
  _Static_assert(StateExiting < ARRAY_LENGTH(state_map), "The number of states has grown, but state"
      "the simple state mapping has not been updated");
  PBL_ASSERTN(state < ARRAY_LENGTH(state_map));

  return state_map[state];
}

static void prv_voice_event_handler(PebbleEvent *e, void *context) {
  PebbleVoiceServiceEvent *event = (PebbleVoiceServiceEvent *) e;
  VoiceUiData *data = context;

  VoiceUiState simple_state = prv_get_simple_state(data->state);
  PBL_LOG_DBG("Event received: %"PRIu8"; state:%"PRIu8, event->type, simple_state);
  switch (simple_state) {
    case StateWaitForReady:
      if (event->type == VoiceEventTypeSessionSetup) {
        prv_handle_ready_event(data, event);
      }
      break;

    case StateWaitForResponse:
      if (event->type == VoiceEventTypeSessionResult) {
        prv_handle_dictation_result(data, event);
      }
      break;

    case StateRecording:
      if (event->type == VoiceEventTypeSilenceDetected) {
        PBL_LOG_DBG("Silence detected");
        prv_stop_dictation(data);
      }
      if (event->type == VoiceEventTypeSpeechDetected) {
        PBL_LOG_DBG("Speech detected");
        data->speech_detected = true;
      }
      if (event->type == VoiceEventTypeSessionResult) {
        // Recording stopped by voice service, capture time recording
        prv_update_analytics_metrics(data);

        app_timer_cancel(data->dictation_timeout);
        prv_handle_dictation_result(data, event);
      }
      break;

    case StateTransitionToText:
    case StateError:
    case StateFinished:
    case StateExiting:
      // Discard event
      PBL_LOG_DBG("Ignoring event");
      break;
    default:
      WTF;
  }
}

static void prv_start_dictation(VoiceUiData *data) {
  PBL_LOG_DBG("Start dictation session");
  PBL_ASSERTN(data->session_id == VOICE_SESSION_ID_INVALID);
  data->session_id = sys_voice_start_dictation(data->session_type);
  if (data->session_id == VOICE_SESSION_ID_INVALID) {
    PBL_LOG_ERR("Dictation session failed to start");
    prv_exit_and_send_result_event(data, DictationSessionStatusFailureInternalError);
    return;
  }
  if (data->state != StateStartWaitForReady) {
    // This is a bit of a hack to prevent jumps in the fly in animation when a session fail comes
    // back quickly
    prv_set_mic_window_state(data, StateWaitForReady);
  }
}

static void prv_stop_dictation(VoiceUiData *data) {
  PBL_LOG_DBG("Stop dictation and wait for result");
  sys_voice_stop_dictation(data->session_id);
  prv_set_mic_window_state(data, StateWaitForResponse);
  prv_update_analytics_metrics(data);
  app_timer_cancel(data->dictation_timeout);
}

static void prv_cancel_dictation(VoiceUiData *data) {
  if ((data->state != StateStart) && (data->state != StateFinished) &&
      (data->state != StateExiting) && (data->state != StateError)) {
    PBL_LOG_DBG("Cancel dictation session");
    sys_voice_cancel_dictation(data->session_id);
    data->session_id = VOICE_SESSION_ID_INVALID;
    app_timer_cancel(data->dictation_timeout);
    prv_set_mic_window_state(data, StateFinished);
  }
}

// Microphone Window
///////////////////////////////////////////////////////////////////////////

#if (0) // https://pebbletechnology.atlassian.net/browse/PBL-20406
static void prv_short_vibe(void) {
  static const uint32_t VIBE_DURATIONS[] = { 50 };
  VibePattern pattern = {
    .durations = VIBE_DURATIONS,
    .num_segments = ARRAY_LENGTH(VIBE_DURATIONS),
  };
  vibes_enqueue_custom_pattern(pattern);
}
#endif

///////////// CREATE DOT ANIMATIONS
static void prv_dot_layer_update_proc(Layer *layer, GContext *ctx) {
  VoiceUiData *data = window_get_user_data(layer_get_window(layer));
  // get frame to place dot in middle of layer
  graphics_context_set_fill_color(ctx, GColorBlack);
  graphics_fill_circle(ctx, grect_center_point(&layer->bounds), data->mic_window.mic_dot_radius);
}

static void prv_set_dot_width(void *subject, int16_t radius) {
  VoiceUiData *data = subject;
  data->mic_window.mic_dot_radius = radius;
  layer_mark_dirty(&data->mic_window.mic_dot_layer);
}

static PropertyAnimation *prv_create_int16_prop_anim(int16_t from, int16_t to,
                                                     uint32_t duration,
                                                     const PropertyAnimationImplementation *impl,
                                                     void *subject) {

  PropertyAnimation *anim = property_animation_create(impl, subject, NULL, NULL);
  if (!anim) {
    return NULL;
  }
  property_animation_set_from_int16(anim, &from);
  property_animation_set_to_int16(anim, &to);

  animation_set_duration((Animation *) anim, duration);
  animation_set_curve((Animation *) anim, AnimationCurveEaseInOut);

  return anim;
}


static const PropertyAnimationImplementation s_animated_dot_impl = {
    .base = {
        .update = (AnimationUpdateImplementation) property_animation_update_int16,
    },
    .accessors = {
        .setter = { .int16 = prv_set_dot_width, },
    },
  };

static Animation *prv_create_pulse_dot_anim(VoiceUiData *data, int16_t min, int16_t max,
                                            int16_t overshoot, uint32_t delay_duration,
                                            uint32_t pulse_duration) {

  uint32_t stage_duration = (pulse_duration / 3);

  // Declare these here so that if we goto cleanup, then the variables are initialized
  PropertyAnimation *expand = NULL;
  PropertyAnimation *shrink = NULL;
  PropertyAnimation *revert = NULL;

  // Do the overshoot animation first
  expand = prv_create_int16_prop_anim(max, overshoot + max, stage_duration, &s_animated_dot_impl,
      data);
  if (!expand) {
    goto cleanup;
  }
  animation_set_delay((Animation *) expand, delay_duration);

  // If overshoot > 0 shrink to min size, otherwise shrink from max to min
  int16_t current_size = (overshoot != 0) ? overshoot + max : max;
  shrink = prv_create_int16_prop_anim(current_size, min, stage_duration, &s_animated_dot_impl,
      data);
  if (!shrink) {
    goto cleanup;
  }

  revert =  prv_create_int16_prop_anim(min, max, stage_duration, &s_animated_dot_impl, data);
  if (!revert) {
    goto cleanup;
  }

  Animation *sequence  = animation_sequence_create((Animation *)expand, (Animation *)shrink,
        (Animation *) revert, NULL);

  if (!sequence) {
    goto cleanup;
  }

  animation_set_play_count(sequence, ANIMATION_PLAY_COUNT_INFINITE);
  return sequence;

cleanup:
  property_animation_destroy(shrink);
  property_animation_destroy(expand);
  property_animation_destroy(revert);
  return NULL;
}
///////////// END - CREATE DOT ANIMATIONS

static void prv_hide_mic_text(VoiceUiData *data) {
  text_layer_set_text(&data->mic_window.text_layer, "");
  layer_set_hidden((Layer *)&data->mic_window.text_layer, true);
}

static void prv_show_mic_text(VoiceUiData *data, const char *msg) {
  layer_set_hidden((Layer *)&data->mic_window.text_layer, false);
  sys_i18n_get_with_buffer(msg, data->mic_window.text_buffer, sizeof(data->mic_window.text_buffer));
  text_layer_set_text(&data->mic_window.text_layer, data->mic_window.text_buffer);
}

static void prv_kino_reel_stopped_handler(KinoLayer *layer, bool finished, void *context) {
  if (!finished) {
    return;
  }

  // This stopped handler is used to defer the transition from recording to the wait for response
  // screen until the folding animation is complete.
  prv_handle_stop_transition(context);
}

static void prv_show_unfold_animation(VoiceUiData *data, bool is_reversed) {
  KinoReel *reel = kino_layer_get_reel(&data->mic_window.icon_layer);

  // reel can be null if the image was not found in the init function.
  if (!reel) {
    return;
  }
  layer_set_hidden((Layer *) &data->mic_window.icon_layer, false);
  kino_layer_rewind(&data->mic_window.icon_layer);

  GRect from = kino_reel_transform_get_from_frame(reel);
  GRect to = kino_reel_transform_get_to_frame(reel);
  kino_reel_scale_segmented_set_from_stroke_width(reel, FIXED_S16_3_ONE, GStrokeWidthOpMultiply);
  kino_reel_scale_segmented_set_to_stroke_width(reel, FIXED_S16_3_ONE, GStrokeWidthOpMultiply);
  if (is_reversed) {
    if (to.size.w > from.size.w) {
      // swap frames so that we shrink to a dot during reverse
      kino_reel_transform_set_from_frame(reel, to);
      kino_reel_transform_set_to_frame(reel, from);
    }
    kino_reel_scale_segmented_set_end_as_dot(reel, data->mic_window.mic_dot_radius);
    kino_layer_play_section(&data->mic_window.icon_layer, 0, UNFOLD_DURATION);
  } else {
    if (to.size.w < from.size.w) {
      // swap frames so that we unfold from a dot
      kino_reel_transform_set_from_frame(reel, to);
      kino_reel_transform_set_to_frame(reel, from);
    }
    kino_reel_unfold_set_start_as_dot(reel, data->mic_window.mic_dot_radius);
    kino_layer_play(&data->mic_window.icon_layer);
  }
}

static void prv_hide_unfold_animation(VoiceUiData *data) {
  kino_layer_pause(&data->mic_window.icon_layer);
  layer_set_hidden((Layer *) &data->mic_window.icon_layer, true);
}

static void prv_show_mic_dot_pulse(VoiceUiData *data) {
  // Dot is already animating
  if (animation_is_scheduled(data->mic_window.mic_dot_anim)) {
    animation_unschedule(data->mic_window.mic_dot_anim);
  }

  const GRect *root_frame = &window_get_root_layer(&data->mic_window.window)->frame;
  GRect dot_frame = data->mic_window.mic_dot_layer.frame;
  grect_align(&dot_frame, root_frame, GAlignCenter, false);
  layer_set_frame(&data->mic_window.mic_dot_layer, &dot_frame);
  layer_set_hidden(&data->mic_window.mic_dot_layer, false);

  static const uint32_t ANIMATION_DURATION = 270;
  static const int16_t MIN_RADIUS = 7;
  static const int16_t OVERSHOOT = 4;
  static const uint32_t DELAY_DURATION = 1000;
  static const uint32_t START_ELAPSED = 800;  // show pulse just after the start of the animation

  layer_mark_dirty(&data->mic_window.mic_dot_layer);
  data->mic_window.mic_dot_radius = MIC_DOT_MAX_RADIUS;

  data->mic_window.mic_dot_anim = prv_create_pulse_dot_anim(data, MIN_RADIUS,
      MIC_DOT_MAX_RADIUS, OVERSHOOT, DELAY_DURATION, ANIMATION_DURATION);

  animation_schedule(data->mic_window.mic_dot_anim);
  animation_set_elapsed(data->mic_window.mic_dot_anim, START_ELAPSED);
}

static void prv_hide_mic_dot(VoiceUiData *data) {
  if (animation_is_scheduled(data->mic_window.mic_dot_anim)) {
    animation_unschedule(data->mic_window.mic_dot_anim);
  }
  layer_set_hidden(&data->mic_window.mic_dot_layer, true);
}

static void prv_handle_animation_stop(Animation *animation, bool finished, void *context) {
  if (!finished) {
    return;
  }
  prv_handle_stop_transition(context);
}

static const int32_t NUM_MOOOK_FRAMES_MID = 3;

static int64_t prv_interpolate_moook_soft(int32_t normalized, int64_t from, int64_t to) {
  return interpolate_moook_soft(normalized, from, to, NUM_MOOOK_FRAMES_MID);
}

// fly dot in or out of the window
static void prv_fly_dot(VoiceUiData *data, bool fly_in) {
  Layer *dot_layer = &data->mic_window.mic_dot_layer;
  const GRect *root_frame = &window_get_root_layer(layer_get_window(dot_layer))->frame;

  GRect dot_frame = {.size = MIC_DOT_LAYER_SIZE};
  grect_align(&dot_frame, root_frame, GAlignCenter, false);
  GRect from;
  GRect to;

  if (fly_in) {
    to = dot_frame;
    from = dot_frame;
    from.origin.x = -to.size.w;
  } else {
    from = dot_frame;
    to = dot_frame;
    to.origin.x = -from.size.w;
  }
  data->mic_window.mic_dot_radius = MIC_DOT_MAX_RADIUS;
  PropertyAnimation *anim = property_animation_create_layer_frame(dot_layer, &from, &to);

  if (!anim) {
    return;
  }

  layer_set_frame(dot_layer, &from);
  layer_set_hidden(dot_layer, false);

  animation_set_custom_interpolation((Animation *)anim, prv_interpolate_moook_soft);
  animation_set_duration((Animation *)anim, interpolate_moook_soft_duration(NUM_MOOOK_FRAMES_MID));
  animation_set_handlers((Animation *)anim, (AnimationHandlers) {
    .stopped = prv_handle_animation_stop,
  }, data);

  data->mic_window.fly_anim = anim;
  animation_schedule((Animation *)anim);
}

// cancel the flying animation
static void prv_stop_fly_dot(VoiceUiData *data) {
  if (animation_is_scheduled((Animation *)data->mic_window.fly_anim)) {
    GRect to;
    property_animation_get_to_grect(data->mic_window.fly_anim, &to);
    animation_unschedule((Animation *)data->mic_window.fly_anim);
    layer_set_frame(&data->mic_window.mic_dot_layer, &to);
  }
}

static void prv_set_percent(void *subject, int16_t percent) {
  VoiceUiData *data = subject;
  progress_layer_set_progress((ProgressLayer *)&data->mic_window.progress_bar, percent);
}

static const PropertyAnimationImplementation s_progress_bar_impl = {
  .base = {
    .update = (AnimationUpdateImplementation) property_animation_update_int16,
  },
  .accessors = {
    .setter = { .int16 = prv_set_percent, },
  },
};

// animate the progress bar in by growing it from the left (or just show it if animated == false)
static void prv_show_progress_bar(VoiceUiData *data, bool animated) {
  static const uint16_t MAX_PROGRESS_FUDGE_AMOUNT = 75;
  static const uint32_t PROGRESS_FUDGE_DURATION = 5000;
  static const uint32_t ANIMATE_IN_DURATION = 200;

  if (!layer_get_hidden((Layer *)&data->mic_window.progress_bar)) {
    return;
  }
  progress_layer_set_progress((ProgressLayer *)&data->mic_window.progress_bar, 0);

  animation_unschedule((Animation *)data->mic_window.progress_anim);
  data->mic_window.progress_anim = prv_create_int16_prop_anim(0, MAX_PROGRESS_FUDGE_AMOUNT,
      PROGRESS_FUDGE_DURATION, &s_progress_bar_impl, data);

  if (data->mic_window.progress_anim) {
    animation_schedule((Animation *)data->mic_window.progress_anim);
  }
  layer_set_hidden((Layer *)&data->mic_window.progress_bar, false);
  loading_layer_grow(&data->mic_window.progress_bar, 0,
      (animated ? ANIMATE_IN_DURATION : 0));
}

static void prv_progress_stop(Animation *animation, bool finished, void *context) {
  if (!finished) {
    return;
  }
  static const uint32_t SHRINK_DELAY = 100;
  static const uint32_t SHRINK_DURATION = 200;
  VoiceUiData *data = context;
  loading_layer_shrink(&data->mic_window.progress_bar, SHRINK_DELAY, SHRINK_DURATION,
      prv_handle_animation_stop, data);
}

// shrink the progress bar from the left after animating the progress % to 100%
static void prv_shrink_progress_bar(VoiceUiData *data) {
  animation_unschedule((Animation *)data->mic_window.progress_anim);

  uint16_t progress = data->mic_window.progress_bar.progress_layer.progress_percent;
  uint32_t duration = MAX_PROGRESS_PERCENT - progress;

  data->mic_window.progress_anim = prv_create_int16_prop_anim(progress, MAX_PROGRESS_PERCENT,
      duration, &s_progress_bar_impl, data);
  // use a stopped handler instead of a sequence animation because we need to be able to stop
  animation_set_handlers((Animation *)data->mic_window.progress_anim, (AnimationHandlers) {
    .stopped = prv_progress_stop,
  }, data);

  animation_schedule((Animation *)data->mic_window.progress_anim);
}

static void prv_mic_click_handler(ClickRecognizerRef recognizer, void *context) {
  VoiceUiData *data = context;
  ButtonId button_id = click_recognizer_get_button_id(recognizer);
  if (button_id == BUTTON_ID_BACK) {
    PBL_LOG_DBG("Exit UI");
    prv_cancel_dictation(data);
    DictationSessionStatus status = DictationSessionStatusFailureTranscriptionRejected;
    if ((data->error_count > 0) && !data->last_session_successful) {
      status = DictationSessionStatusFailureTranscriptionRejectedWithError;
    }
    prv_exit_and_send_result_event(context, status);
  } else {
    // Select button pressed
    prv_stop_dictation(context);
  }
}

static void prv_back_select_click_config_provider(void *context) {
  window_set_click_context(BUTTON_ID_BACK, context);
  window_single_click_subscribe(BUTTON_ID_BACK, &prv_mic_click_handler);
  window_set_click_context(BUTTON_ID_SELECT, context);
  window_single_click_subscribe(BUTTON_ID_SELECT, &prv_mic_click_handler);
}

static void prv_back_click_config_provider(void *context) {
  window_set_click_context(BUTTON_ID_BACK, context);
  window_single_click_subscribe(BUTTON_ID_BACK, &prv_mic_click_handler);
}

static void prv_enable_select_click(VoiceUiData *data) {
  window_set_click_config_provider_with_context(&data->mic_window.window,
      prv_back_select_click_config_provider, data);
}

static void prv_disable_select_click(VoiceUiData *data) {
  window_set_click_config_provider_with_context(&data->mic_window.window,
      prv_back_click_config_provider, data);
}

static void prv_hide_progress_bar(VoiceUiData *data) {
  animation_unschedule((Animation *)data->mic_window.progress_anim);
  loading_layer_pause(&data->mic_window.progress_bar);
  layer_set_hidden((Layer *)&data->mic_window.progress_bar, true);
}

static void prv_voice_confirm_cb(void *context) {
  prv_exit_and_send_result_event((VoiceUiData *)context, DictationSessionStatusSuccess);
}

// Initiates transitions triggered by animations finishing
static void prv_handle_stop_transition(VoiceUiData *data) {
  // Transition to next state
  switch (data->state) {
    case StateStartWaitForReady:
      prv_set_mic_window_state(data, StateWaitForReady);
      break;

    case StateRecording:
      // do nothing
      break;

    case StateStopRecording:
      prv_set_mic_window_state(data, StateWaitForResponse);
      break;

    case StateStopWaitForResponse:
      prv_set_mic_window_state(data, StateTransitionToText);
      break;

    case StateTransitionToText: {
      prv_set_mic_window_state(data, StateFinished);

      if (data->show_confirmation_dialog) {
        TranscriptionDialog *transcription_dialog = &data->transcription_dialog;
        transcription_dialog_init(transcription_dialog);
        transcription_dialog_update_text(transcription_dialog, data->message, data->message_len);
        transcription_dialog_set_callback(transcription_dialog, prv_voice_confirm_cb, data);
        transcription_dialog_keep_alive_on_select(transcription_dialog,
                                                  data->transcription_dialog_keep_alive_on_select);
        Dialog *dialog = expandable_dialog_get_dialog((ExpandableDialog *)transcription_dialog);
        dialog_set_destroy_on_pop(dialog, false /* free_on_pop */);

        transcription_dialog_push(transcription_dialog, prv_get_window_stack());
        sys_light_reset_to_timed_mode();
      } else {
        prv_exit_and_send_result_event(data, DictationSessionStatusSuccess);
      }
      break;
    }
    default:
      WTF;
  }
}

// This function gets the next state to transition to and whether that transition should be deferred
// until the current animation is complete.
static VoiceUiState prv_get_next_state(VoiceUiState current_state, VoiceUiState next_state,
                                       bool *defer_transition) {
  // StateWaitForReady is unique because it can be re-entered when session setup times
  // out
  if ((current_state == next_state) && (next_state == StateWaitForReady)) {
    *defer_transition = false;
    return next_state;
  }

  PBL_ASSERT(current_state != next_state, "Trying to transition to the same state %d",
             next_state);

  PBL_ASSERTN(next_state != StateStart); // Cannot transition to start state

  PBL_LOG_DBG("Transition: Current state: %d; new state: %d", current_state, next_state);

  // Transition will be handled by the animation stopped handler if defer_transition is set to true
  *defer_transition = false;

  // These transitions are always valid
  if ((next_state == StateFinished) ||
      (next_state == StateExiting) ||
      (next_state == StateError)) {
    return next_state;
  }

  // This determines whether a transition is valid and whether the transition should be deferred
  // until an animation completes. If a transition skips states, this will return the first
  // intermediate state to enter (or in the case of a deferred transition, which case to spoof so
  // that the correct transition occurs).
  switch (current_state) {
    case StateStart:
      if (next_state == StateWaitForReady) {
        return StateStartWaitForReady;
      }
      break;

    case StateStartWaitForReady:
      if (next_state == StateWaitForReady || next_state == StateRecording) {
        return next_state;
      }
      break;

    case StateWaitForReady:
      if (next_state == StateRecording) {
        return next_state;
      }
      break;

    case StateRecording:
      if ((next_state == StateWaitForResponse) || (next_state == StateTransitionToText)) {
        return StateStopRecording;
      }
      break;

    case StateStopRecording:
      if (next_state == StateTransitionToText) {
        *defer_transition = true;
        // Spoof the state to StateStopWaitForResponse so the next transition takes us to
        // StateTransitionToText
        return StateStopWaitForResponse;
      } else if (next_state == StateWaitForResponse) {
        return next_state;
      }
      break;

    case StateWaitForResponse:
      if (next_state == StateTransitionToText) {
        return StateStopWaitForResponse;
      }
      break;

    case StateStopWaitForResponse:
      if (next_state == StateTransitionToText) {
        return next_state;
      }
      break;

    case StateTransitionToText:
      if (next_state == StateFinished) {
        return next_state;
      }
      break;

    case StateFinished:
      if ((next_state == StateWaitForReady) || (next_state == StateStartWaitForReady)) {
        return StateStartWaitForReady;
      }
      break;

    case StateError:
    case StateExiting:
      if (next_state == StateWaitForReady) {
        return next_state;
      }
      break;

    default:
      WTF;
  }

  // No valid transition found!

  PBL_CROAK("Cannot transition from state %"PRIu16" to state %"PRIu16,
            current_state, next_state);
}

// This handles all the microphone UI transitions
static void prv_do_transition(VoiceUiData *data, VoiceUiState state) {
  PBL_LOG_DBG("Transition: %d -> %d", data->state, state);
  switch (state) {
    case StateStartWaitForReady:
      // Fly in dot
      prv_fly_dot(data, true /* fly in */);
      break;

    case StateWaitForReady:
      // Stop fly in animation. Start pulsing dot animation.
      prv_stop_fly_dot(data);
      prv_show_mic_dot_pulse(data);
      break;

    // TODO: Create an intermediate state where the microphone unfolds and the vibe plays before
    // turning the mic on - dependent on separating the setup session and recording stages of
    // voice state machine
    case StateRecording:
      // vibe briefly, enable clicking to end the transcription, unfold the mic from a dot
      // and show text
#if (0) // https://pebbletechnology.atlassian.net/browse/PBL-20406
      prv_short_vibe();
#endif
      prv_enable_select_click(data);
      prv_stop_fly_dot(data);
      prv_hide_mic_dot(data);
      prv_hide_progress_bar(data);
      prv_show_unfold_animation(data, false /* is_reverse */);
      prv_show_mic_text(data, i18n_noop("Listening"));
      break;

    case StateStopRecording:
      // Fold animation back
      prv_disable_select_click(data);
      prv_hide_mic_text(data);
      prv_show_unfold_animation(data, true /* is_reverse */);
      break;

    case StateWaitForResponse:
      // pulse the microphone dot
      prv_hide_unfold_animation(data);
      prv_show_mic_dot_pulse(data);
      prv_show_progress_bar(data, true /* animated */);
      break;

    case StateStopWaitForResponse:
      // shrink progress bar
      prv_shrink_progress_bar(data);
      break;

    case StateTransitionToText:
      // fly dot out
      prv_hide_unfold_animation(data);
      prv_hide_progress_bar(data);
      prv_hide_mic_dot(data);
      prv_fly_dot(data, false /* fly out */);

      break;

    case StateError:
    case StateFinished:
    case StateExiting:
      // hide all elements
      prv_disable_select_click(data);
      prv_hide_unfold_animation(data);
      prv_hide_mic_text(data);
      prv_hide_progress_bar(data);
      prv_hide_mic_dot(data);
      prv_stop_fly_dot(data);
      break;

    default:
      WTF;
  }
}

static void prv_set_mic_window_state(VoiceUiData *data, VoiceUiState state) {
  bool defer_transition;
  state = prv_get_next_state(data->state, state, &defer_transition);
  if (!defer_transition) {
    prv_do_transition(data, state);
  }

  data->state = state;
  PBL_LOG_DBG("State: %d", data->state);
}

static void prv_mic_window_load(Window *window) {
  VoiceUiData *data = window_get_user_data(window);

  const GColor window_bg_color = PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite);
  window_set_background_color(window, window_bg_color);
  Layer *root_layer = window_get_root_layer(window);
  const GRect *root_frame = &root_layer->frame;

  Layer *mic_dot_layer = &data->mic_window.mic_dot_layer;

  GRect dot_frame = {.size = MIC_DOT_LAYER_SIZE};
  grect_align(&dot_frame, root_frame, GAlignCenter, false);

  layer_init(mic_dot_layer, &dot_frame);
  layer_set_clips(mic_dot_layer, false);  //
  layer_set_update_proc(mic_dot_layer, prv_dot_layer_update_proc);
  layer_add_child(root_layer, mic_dot_layer);
  layer_set_hidden(mic_dot_layer, true);

  GFont font = fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD);
  int16_t font_height = fonts_get_font_height(font);
  static const int16_t TEXT_LAYER_Y_OFFSET = 50;

  TextLayer *text_layer = &data->mic_window.text_layer;
  text_layer_init_with_parameters(text_layer,
                                  &GRect(0, dot_frame.origin.y + TEXT_LAYER_Y_OFFSET,
                                         root_frame->size.w, font_height * 2),
                                  NULL, font,
                                  GColorBlack, window_bg_color, GTextAlignmentCenter,
                                  GTextOverflowModeTrailingEllipsis);
  layer_add_child(root_layer, (Layer *)text_layer);
  layer_set_hidden((Layer *)text_layer, true);

  static const int16_t LOADING_FRAME_OFFSET_Y = 27;
  GRect loading_frame = (GRect) { .size = LOADING_LAYER_DEFAULT_SIZE };
  grect_align(&loading_frame, root_frame, GAlignCenter, false);
  loading_frame.origin.y += LOADING_FRAME_OFFSET_Y;

  LoadingLayer *loading_layer = &data->mic_window.progress_bar;
  loading_layer_init(loading_layer, &loading_frame);
  progress_layer_set_foreground_color((ProgressLayer *)loading_layer, GColorBlack);

  const GColor progress_bg_color = PBL_IF_COLOR_ELSE(GColorDarkGray, GColorWhite);
  progress_layer_set_background_color((ProgressLayer *)loading_layer, progress_bg_color);
  layer_add_child(&window->layer, (Layer *)loading_layer);
  layer_set_hidden((Layer *)loading_layer, true);

  StatusBarLayer *status_bar = &data->mic_window.status_bar;
  status_bar_layer_init(status_bar);
  const GColor status_bg_color = PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite);
  status_bar_layer_set_colors(status_bar, status_bg_color, GColorBlack);
  layer_add_child(root_layer, (Layer *) status_bar);

  KinoReel *image = kino_reel_create_with_resource_system(SYSTEM_APP,
                                                          RESOURCE_ID_VOICE_MICROPHONE_LARGE);
  PBL_ASSERTN(image);

  GSize icon_size = kino_reel_get_size(image);
  // Center the icon resting position in the window
  GRect icon_frame = (GRect) {
    .size.w = icon_size.w,
    .size.h = icon_size.h
  };
  grect_align(&icon_frame, root_frame, GAlignCenter, false);

  static const int16_t UNFOLD_BOUNCE_AMOUNT = 10;
  static const int16_t UNFOLD_EXPAND_AMOUNT = 5;
  int16_t dot_size = data->mic_window.mic_dot_radius * 2;

  GRect icon_from = {
    .size = { dot_size, dot_size },
  };
  grect_align(&icon_from, &icon_frame, GAlignCenter, false);

  const bool take_ownership = true;
  KinoReel *icon_reel = kino_reel_unfold_create(image, take_ownership, icon_frame, 0,
    UNFOLD_DEFAULT_NUM_DELAY_GROUPS, UNFOLD_DEFAULT_GROUP_DELAY);

  if (icon_reel) {
    kino_reel_transform_set_from_frame(icon_reel, icon_from);
    kino_reel_transform_set_transform_duration(icon_reel, UNFOLD_DURATION);
    kino_reel_scale_segmented_set_deflate_effect(icon_reel, UNFOLD_EXPAND_AMOUNT);
    kino_reel_scale_segmented_set_bounce_effect(icon_reel, UNFOLD_BOUNCE_AMOUNT);

    kino_layer_init(&data->mic_window.icon_layer, &icon_frame);
    // do not clip bounds of window - animated icon will be hidden when it's not within the
    // visible bounds
    kino_layer_set_reel(&data->mic_window.icon_layer, icon_reel, true);
    kino_layer_set_callbacks(&data->mic_window.icon_layer, (KinoLayerCallbacks) {
      .did_stop = prv_kino_reel_stopped_handler,
    }, data);
    layer_add_child(root_layer, (Layer *)&data->mic_window.icon_layer);
    layer_set_hidden((Layer *)&data->mic_window.icon_layer, true);
  } else {
    kino_reel_destroy(image);
  }

  data->voice_event_sub = (EventServiceInfo) {
    .type = PEBBLE_VOICE_SERVICE_EVENT,
    .handler = prv_voice_event_handler,
    .context = data
  };
  event_service_client_subscribe(&data->voice_event_sub);

  prv_disable_select_click(data);
}

// Mic window unload called last when UI is exited. Unsubscribe from events and free UI data
static void prv_mic_window_unload(Window *window) {
  VoiceUiData *data = window_get_user_data(window);
  kino_layer_deinit(&data->mic_window.icon_layer);
  loading_layer_deinit(&data->mic_window.progress_bar);
  layer_deinit(&data->mic_window.mic_dot_layer);
  text_layer_deinit(&data->mic_window.text_layer);
  status_bar_layer_deinit(&data->mic_window.status_bar);
  event_service_client_unsubscribe(&data->voice_event_sub);
}

static void prv_mic_window_disappear(Window *window) {
  VoiceUiData *data = window_get_user_data(window);
  // prv_mic_window_appear pinned the backlight via sys_light_enable_respect_settings(true).
  // Release it here so the appear/disappear pair is symmetric, even if a later
  // exit path (prv_exit_and_send_result_event, voice_window_pop) never runs.
  sys_light_reset_to_timed_mode();
  if (data->state != StateError) {
    // Do not indicate that an error occurred when a session is interrupted by a window transition
    if (data->state != StateFinished) {
      data->last_session_successful = false;
    }
    prv_cancel_dictation(data);
  }
}

static void prv_mic_window_appear(Window *window) {
  VoiceUiData *data = window_get_user_data(window);
  if ((data->state == StateStart) || (data->state == StateFinished)) {
    sys_light_enable_respect_settings(true);
    prv_start_dictation(data);
  }
}

static void prv_voice_window_push(VoiceUiData *data) {
  Window *window = &data->mic_window.window;
  window_init(window, WINDOW_NAME("Voice Window"));
  window_set_window_handlers(window, &(WindowHandlers) {
    .load = prv_mic_window_load,
    .unload = prv_mic_window_unload,
    .appear = prv_mic_window_appear,
    .disappear = prv_mic_window_disappear
  });
  window_set_user_data(window, data);

  prv_window_push(window);
}

// External interface
/////////////////////////////////////////////////////////////////////////////////

VoiceWindow *voice_window_create(char *buffer, size_t buffer_size,
                                 VoiceEndpointSessionType session_type) {
  // if buffer is NULL, buffer_size must be 0 and if it is not NULL, buffer_size must be non-zero
  PBL_ASSERTN((buffer != NULL) == (buffer_size > 0));

  VoiceUiData *data = applib_type_malloc(VoiceWindow);
  if (!data) {
    return NULL;
  }
  *data = (VoiceUiData) {
    .state = StateStart,
    .show_confirmation_dialog = true,
    .show_error_dialog = true,
    .message = buffer,
    .buffer_size = buffer_size,
    .session_type = session_type,
  };

  return data;
}

void voice_window_destroy(VoiceWindow *voice_window) {
  voice_window_pop(voice_window);
  applib_free(voice_window->message);
  applib_free(voice_window);
}

DictationSessionStatus voice_window_push(VoiceWindow *voice_window) {
  if (!connection_service_peek_pebble_app_connection()) {
    if (voice_window->show_error_dialog) {
      prv_push_bt_dialog(voice_window);

      // we return success because the user could reconnect the phone and watch and resume the UI
      // flow
      return DictationSessionStatusSuccess;
    } else {
      return DictationSessionStatusFailureConnectivityError;
    }
  }
  voice_window->state = StateStart;
  prv_voice_window_push(voice_window);
  return DictationSessionStatusSuccess;
}

void voice_window_pop(VoiceWindow *voice_window) {
  sys_light_reset_to_timed_mode();
  prv_cancel_dictation(voice_window);

  // This relies on all dialogs having a dialog object as their first member
  if (window_is_loaded(&voice_window->dialog.window)) {
    dialog_pop(&voice_window->dialog);
  }

  prv_teardown(voice_window);
}

void voice_window_set_confirmation_enabled(VoiceWindow *voice_window, bool enabled) {
  voice_window->show_confirmation_dialog = enabled;
}

void voice_window_set_error_enabled(VoiceWindow *voice_window, bool enabled) {
  voice_window->show_error_dialog = enabled;
}

void voice_window_lose_focus(VoiceWindow *voice_window) {
  if (app_window_stack_get_top_window() == &voice_window->mic_window.window) {
    prv_mic_window_disappear(&voice_window->mic_window.window);
  }
}

void voice_window_regain_focus(VoiceWindow *voice_window) {
  if (app_window_stack_get_top_window() == &voice_window->mic_window.window) {
    prv_mic_window_appear(&voice_window->mic_window.window);
  }
}

void voice_window_transcription_dialog_keep_alive_on_select(VoiceWindow *voice_window,
                                                            bool keep_alive_on_select) {
  voice_window->transcription_dialog_keep_alive_on_select = keep_alive_on_select;
}

void voice_window_reset(VoiceWindow *voice_window) {
  if (voice_window->message) {
    if (voice_window->buffer_size == 0) {
      // If buffer size is set to 0, the buffer was allocated when the last transcription was
      // received
      applib_free(voice_window->message);
      voice_window->message = NULL;
    } else {
      voice_window->message[0] = '\0';
    }
  }
  voice_window->message_len = 0;
  voice_window->last_session_successful = false;
  voice_window->num_sessions = 0;
  voice_window->error_count = 0;
  voice_window->speech_detected = false;
  voice_window->state = StateStart;
  voice_window->session_id = VOICE_SESSION_ID_INVALID;
  voice_window->start_ms = 0;
  voice_window->elapsed_ms = 0;
  voice_window->error_exit_status = DictationSessionStatusSuccess;
}

// Syscalls
/////////////////////////////////////////////////////////////////////////////////

DEFINE_SYSCALL(char *, sys_voice_get_transcription_from_event, PebbleVoiceServiceEvent *e,
               char *buffer, size_t buffer_size, size_t *sentence_len) {

  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(e, sizeof(*e));
    if (buffer && (buffer_size > 0)) {
      syscall_assert_userspace_buffer(buffer, buffer_size);
    }
    syscall_assert_userspace_buffer(sentence_len, sizeof(*sentence_len));
    // `e->data` is a kernel pointer the event service handed to the app along
    // with the event. The app can rewrite the field in its copy before calling
    // us, so confirm it still references a buffer the kernel actually allocated
    // — otherwise an app could aim it at kernel memory and turn the strlen/memcpy
    // below into an arbitrary kernel read primitive.
    if (!event_service_is_known_buffer(e->data)) {
      PBL_LOG_ERR("Rejecting voice event with unknown data pointer %p", e->data);
      syscall_failed();
    }
  }

  size_t len;
  if (!buffer) {
    // if the is not allocated, allocate enough to contain the string
    len = strlen(e->data->sentence);
  } else {
    // if the buffer is allocated, truncate sentence to buffer size
    len = utf8_get_size_truncate(e->data->sentence, buffer_size);
  }

  if (len == 0) {
    return NULL;
  }

  char *sentence;
  if (buffer) {
    // do not allocate a buffer is one is allocated already
    sentence = buffer;
  } else {
    // allocate a buffer if one is not yet allocated
    sentence = applib_malloc(len + 1);
  }

  memcpy(sentence, e->data->sentence, len);
  sentence[len] = '\0'; // Ensure that string is NULL-terminated

  *sentence_len = len;

  return sentence;
}

