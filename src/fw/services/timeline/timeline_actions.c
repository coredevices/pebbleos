/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/timeline/timeline_actions.h"

#include "applib/applib_malloc.auto.h"
#include "applib/event_service_client.h"
#include "applib/ui/action_menu_hierarchy.h"
#include "applib/ui/action_menu_window_private.h"
#include "applib/ui/dialogs/dialog.h"
#include "applib/ui/dialogs/dialog_private.h"
#include "applib/ui/dialogs/expandable_dialog.h"
#include "applib/ui/dialogs/simple_dialog.h"
#include "applib/ui/progress_window.h"
#include "applib/ui/window.h"
#include "applib/ui/window_manager.h"
#include "applib/voice/voice_window.h"
#include "applib/voice/voice_window_private.h"
#include "comm/ble/kernel_le_client/ancs/ancs.h"
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "kernel/ui/kernel_ui.h"
#include "kernel/ui/modals/modal_manager.h"
#include "popups/ble_hrm/ble_hrm_stop_sharing_popup.h"
#include "popups/notifications/notification_window.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/services/event_service.h"
#include "pbl/services/evented_timer.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/blob_db/reminder_db.h"
#include "pbl/services/bluetooth/ble_hrm.h"
#include "pbl/services/notifications/action_chaining_window.h"
#include "pbl/services/notifications/alerts_preferences.h"
#include "pbl/services/notifications/ancs/ancs_notifications.h"
#include "pbl/services/notifications/notification_constants.h"
#include "pbl/services/notifications/notification_storage.h"
#include "pbl/services/timeline/timeline.h"
#include "pbl/services/timeline/timeline_resources.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/size.h"

PBL_LOG_MODULE_DECLARE(service_timeline, CONFIG_SERVICE_TIMELINE_LOG_LEVEL);

typedef struct {
  void *action_data;
  void *context;
  ActionMenu *action_menu;
#ifdef CONFIG_MIC
  VoiceWindow *voice_window;
#endif
  EventServiceInfo event_service_info;
} VoiceResponseData;

typedef struct ActionResultData {
  EventServiceInfo event_service_info;
  ActionMenu *action_menu;
  ProgressWindow *progress_window; //!< For showing progress of long requests
  VoiceResponseData *voice_data;
  bool standalone_action; //!< This action was performed without a previous action menu

  struct {
    EventedTimerID timer; //!< For timing out requests
    Attribute attribute; //!< used to persist the response value while waiting for result
    bool ignore_failures;
  } response;

  struct {
    Window *action_chaining_window;
    TimelineItem *notif;
  } chaining_data;

  struct {
    char *message;
    TimelineResourceId icon;
    bool success;
  } dialog;

  struct {
    ActionCompleteCallback callback;
    void *callback_data;
  } action_complete;
} ActionResultData;

typedef struct TimelineActionMenu {
  ActionMenu *action_menu;
  TimelineItem *item;
  ActionResultData *action_result;
  ActionMenuDidCloseCb did_close;
} TimelineActionMenu;

static void prv_subscribe_to_action_results_and_timeouts(ActionResultData *data,
                                                         bool ignore_failures);

static void prv_request_responsive_session(void) {
  // In anticipation of having to communicate with the phone, request the minimum latency for 10s:
  comm_session_set_responsiveness(comm_session_get_system_session(),
                                  BtConsumerTimelineActionMenu, ResponseTimeMin,
                                  MIN_LATENCY_MODE_TIMEOUT_TIMELINE_ACTION_MENU_SECS);
}

static void prv_reset_session_responsiveness(void) {
  comm_session_set_responsiveness(comm_session_get_system_session(),
                                  BtConsumerTimelineActionMenu, ResponseTimeMax, 0);
}

static WindowStack *prv_get_window_stack(ActionResultData *data) {
  return window_manager_get_window_stack(data->voice_data ? ModalPriorityVoice :
                                                            ModalPriorityNotification);
}

static void prv_cleanup_voice_data(VoiceResponseData *data) {
  event_service_client_unsubscribe(&data->event_service_info);
#ifdef CONFIG_MIC
  voice_window_destroy(data->voice_window);
#endif
  // Free the copied TimelineItem that was created in prv_start_voice_reply()
  timeline_item_destroy(data->context);
  applib_free(data);
}

static void prv_cancel_response_timer(ActionResultData *data) {
  if (evented_timer_exists(data->response.timer)) {
    evented_timer_cancel(data->response.timer);
  }
}

static void prv_cleanup_action_result(ActionResultData *data, bool succeeded) {
  if (!data) {
    return;
  }

  // report to analytics the result of the action
  if (data->response.attribute.id == AttributeIdTitle &&
      data->response.attribute.cstring) {
  }

  if (data->action_complete.callback) {
    data->action_complete.callback(succeeded, data->action_complete.callback_data);
  }

  if (data->action_menu) {
    action_menu_close(data->action_menu, true);
    data->action_menu = NULL;
  } else if (data->chaining_data.action_chaining_window) {
    window_stack_remove(data->chaining_data.action_chaining_window, true);
    data->chaining_data.action_chaining_window = NULL;
  }

  if (data->voice_data) {
    prv_cleanup_voice_data(data->voice_data);
    data->voice_data = NULL;
  }

  if (data->chaining_data.notif) {
    timeline_item_destroy(data->chaining_data.notif);
    data->chaining_data.notif = NULL;
  }

  if (data->progress_window) {
    progress_window_destroy(data->progress_window);
    data->progress_window = NULL;
  }

  applib_free(data->dialog.message);
  data->dialog.message = NULL;

  event_service_client_unsubscribe(&data->event_service_info);
  prv_cancel_response_timer(data);
  applib_free(data);
}

static void prv_show_result_window(ActionResultData *data, const TimelineResourceId timeline_res_id,
                                   const char *msg, bool succeeded) {
  const GSize simple_dialog_icon_size = timeline_resources_get_gsize(TimelineResourceSizeLarge);
  const bool use_status_bar = true;
  const bool use_simple_dialog = simple_dialog_does_text_fit(msg, DISP_FRAME.size,
                                                             simple_dialog_icon_size,
                                                             use_status_bar);

  SimpleDialog *simple_dialog = NULL;
  ExpandableDialog *expandable_dialog = NULL;
  Dialog *dialog = NULL;

  if (use_simple_dialog) {
    simple_dialog = simple_dialog_create("Action Result");
    if (!simple_dialog) {
      goto cleanup;
    }
    dialog = simple_dialog_get_dialog(simple_dialog);
  } else {
    expandable_dialog = expandable_dialog_create("Action Result");
    if (!expandable_dialog) {
      goto cleanup;
    }
    expandable_dialog_show_action_bar(expandable_dialog, false);
    dialog = expandable_dialog_get_dialog(expandable_dialog);
  }

  const TimelineResourceSize icon_format = use_simple_dialog ?
                                           TimelineResourceSizeLarge :
                                           TimelineResourceSizeTiny;
  TimelineResourceInfo timeline_res = {
    .res_id = timeline_res_id,
  };
  AppResourceInfo icon_res_info;
  timeline_resources_get_id(&timeline_res, icon_format, &icon_res_info);

  const char *i18n_msg = i18n_get(msg, dialog);
  dialog_set_text(dialog, i18n_msg);
  i18n_free(msg, dialog);
  dialog_set_icon(dialog, icon_res_info.res_id);
  dialog_set_icon_animate_direction(dialog, DialogIconAnimationFromLeft);
  const uint32_t dialog_timeout_ms = use_simple_dialog ? DIALOG_TIMEOUT_DEFAULT :
                                                         DIALOG_TIMEOUT_INFINITE;
  dialog_set_timeout(dialog, dialog_timeout_ms);

  if (data->action_menu) {
    action_menu_set_result_window(data->action_menu, &dialog->window);
  } else {
    dialog_push(dialog, prv_get_window_stack(data));
  }

cleanup:
  prv_cleanup_action_result(data, succeeded);
}

static bool prv_set_dialog_message(ActionResultData *data, TimelineResourceId timeline_res_id,
                                   const char *message, bool success) {
  const size_t message_buffer_size = strlen(message) + 1;
  data->dialog.message = applib_malloc(message_buffer_size);
  if (!data->dialog.message) {
    return false;
  }
  strncpy(data->dialog.message, message, message_buffer_size);
  data->dialog.icon = timeline_res_id;
  data->dialog.success = success;
  return true;
}

static bool prv_show_result_window_with_progress(ActionResultData *data,
                                                 TimelineResourceId timeline_res_id,
                                                 const char *message, bool success) {
  if (data->progress_window) {
    if (!prv_set_dialog_message(data, timeline_res_id, message, success)) {
      return false;
    }
    if (success) {
      progress_window_set_result_success(data->progress_window);
    } else {
      const uint32_t delay_ms = 100;
      progress_window_set_result_failure(data->progress_window, timeline_res_id, message,
                                         delay_ms);
    }
  } else {
    prv_show_result_window(data, timeline_res_id, message, success);
  }
  return true;
}

static void prv_timeout_handler(void *context) {
  ActionResultData *data = context;
  // we failed to perform action since we timed out.
  const char *msg = i18n_noop("Failed");
  const bool succeeded = false;
  PBL_LOG_WRN("Timed out waiting for action result");
  prv_show_result_window_with_progress(data, TIMELINE_RESOURCE_GENERIC_WARNING, msg, succeeded);
}

static void prv_progress_window_finished(ProgressWindow *window, bool success, void *context) {
  ActionResultData *data = context;
  if (success) {
    prv_show_result_window(data, data->dialog.icon, data->dialog.message, data->dialog.success);
  } else {
    prv_cleanup_action_result(data, success);
  }
}

static void prv_show_progress_window(void *data_ptr) {
  ActionResultData *data = data_ptr;
  data->progress_window = applib_zalloc(sizeof(ProgressWindow));
  if (!data->progress_window) {
    const bool success = false;
    prv_cleanup_action_result(data, success);
    return;
  }
  progress_window_init(data->progress_window);
  const int16_t max_fake_percent = 80;
  progress_window_set_max_fake_progress(data->progress_window, max_fake_percent);
  progress_window_set_callbacks(data->progress_window, (ProgressWindowCallbacks) {
    .finished = prv_progress_window_finished,
  }, data);
  progress_window_set_back_disabled(data->progress_window, true);
  progress_window_push(data->progress_window, prv_get_window_stack(data));

  const unsigned action_result_timeout_ms = 5 * MS_PER_SECOND;
  data->response.timer = evented_timer_register(action_result_timeout_ms,
                                                false,
                                                prv_timeout_handler,
                                                data);
}

static void prv_handle_success_fail_response(ActionResultData *data,
                                             AttributeList *attr_list,
                                             bool success) {
  const char *msg = attribute_get_string(attr_list, AttributeIdSubtitle,
                                         success ? "Success" : "Failed");
  const uint32_t icon = attribute_get_uint32(attr_list, AttributeIdIconLarge,
                                             success ? TIMELINE_RESOURCE_RESULT_SENT
                                                     : TIMELINE_RESOURCE_GENERIC_WARNING);
  if (!prv_show_result_window_with_progress(data, icon, msg, success)) {
    prv_cleanup_action_result(data, success);
  }
}

typedef struct {
  TimelineItem *item;
  void *event_ref;
  bool standalone_action;
} ChainingWindowCBData;

static void prv_cleanup_chaining_action_menu(void *context) {
  ChainingWindowCBData *data = context;
  timeline_item_destroy(data->item);
  event_service_free_claimed_buffer(data->event_ref);
  applib_free(data);
}

static void prv_invoke_chaining_action(Window *chaining_window,
                                       TimelineItemAction *action, void *context) {
  ChainingWindowCBData *cb_data = context;

  ActionResultData *data = applib_zalloc(sizeof(ActionResultData));
  if (!data) {
    return;
  }

  data->chaining_data.action_chaining_window = chaining_window;
  data->chaining_data.notif = timeline_item_copy(cb_data->item);
  data->standalone_action = cb_data->standalone_action;

  const bool ignore_failures = false;
  prv_subscribe_to_action_results_and_timeouts(data, ignore_failures);
  timeline_invoke_action(data->chaining_data.notif, action, NULL);
}

static void prv_handle_chaining_response(ActionResultData *data, PebbleEvent *event) {
  PebbleSysNotificationActionResult *action_result = event->sys_notification.action_result;

  TimelineItem *item = timeline_item_copy(data->chaining_data.notif);
  if (!item) {
    PBL_LOG_WRN("No notification in chaining data");
    prv_cleanup_action_result(data, false /* succeeded */);
    return;
  }

  ChainingWindowCBData *cb_data = applib_malloc(sizeof(ChainingWindowCBData));
  if (!cb_data) {
    timeline_item_destroy(item);
    prv_cleanup_action_result(data, false /* succeeded */);
    return;
  }
  *cb_data = (ChainingWindowCBData) {
    .item = item,
    // Claim the buffer so it doesn't get automatically free'd.
    // The action group needs to stick around
    .event_ref = event_service_claim_buffer(event),
    .standalone_action = data->standalone_action,
  };

  Attribute *title_attr = attribute_find(&action_result->attr_list, AttributeIdTitle);
  const char *title = title_attr ? title_attr->cstring : NULL;
  action_chaining_window_push(prv_get_window_stack(data), title, &action_result->action_group,
                              prv_invoke_chaining_action, cb_data, prv_cleanup_chaining_action_menu,
                              cb_data);

  prv_request_responsive_session();

  prv_cleanup_action_result(data, true /* succeeded */);
}

static void prv_cleanup_do_response_menu(ActionMenu *action_menu, const ActionMenuItem *item,
                                         void *context) {
  timeline_item_destroy(context);
}

static void prv_handle_do_response_response(ActionResultData *data) {
  TimelineItem *item = timeline_item_copy(data->chaining_data.notif);
  if (!item) {
    PBL_LOG_WRN("No notification in chaining data");
    prv_cleanup_action_result(data, false /* succeeded */);
    return;
  }

  TimelineItemAction *reply_action = timeline_item_find_action_by_type(
      item, TimelineItemActionTypeAncsResponse);
  // Update the type of the action to be of type TimelineItemActionTypeResponse. This will cause
  // us to send a slightly different message to the phone so it can tell the difference between
  // the start reply action and the send reply action.
  // This is okay because we are just modifying a copy of the original notification
  reply_action->type = TimelineItemActionTypeResponse;

  GColor color = (GColor) attribute_get_uint8(&item->attr_list, AttributeIdBgColor,
                                              SMS_REPLY_COLOR.argb);

  // Lack of an action menu means this was a standalone action, so adjust reply menu accordingly
  const TimelineItemActionSource current_item_source =
      kernel_ui_get_current_timeline_item_action_source();
  timeline_actions_push_response_menu(item, reply_action, color,
                                      prv_cleanup_do_response_menu, prv_get_window_stack(data),
                                      current_item_source, data->standalone_action);

  prv_cleanup_action_result(data, true /* succeeded */);
}

static void prv_action_handle_response(PebbleEvent *e, void *context) {
  ActionResultData *data = (ActionResultData *)context;

  if (e->sys_notification.type != NotificationActionResult) {
    // Not what we want
    return;
  }

  PebbleSysNotificationActionResult *action_result = e->sys_notification.action_result;

  if (action_result == NULL ||
      (data->response.ignore_failures && action_result->type != ActionResultTypeSuccess)) {
    const bool success = false;
    prv_cleanup_action_result(data, success);
    return;
  }

  char uuid_string[UUID_STRING_BUFFER_LENGTH];
  uuid_to_string(&action_result->id, uuid_string);
  PBL_LOG_INFO("Received action result: Item ID - %s; type - %"
          PRIu8, uuid_string, (uint8_t)action_result->type);

  // Each action result can only service one response event
  event_service_client_unsubscribe(&data->event_service_info);

  switch (action_result->type) {
    case ActionResultTypeSuccess:
    case ActionResultTypeFailure:
      prv_handle_success_fail_response(data, &action_result->attr_list,
                                       (action_result->type == ActionResultTypeSuccess));
      break;
    case ActionResultTypeSuccessANCSDismiss:
    {
      bool should_perform_dismiss = false;
      uint32_t ancs_uid = data->chaining_data.notif->header.ancs_uid;

      if (!data->chaining_data.notif->header.dismissed &&
          timeline_item_find_dismiss_action(data->chaining_data.notif) &&
          notification_window_is_modal()) {
        // Only perform the dismiss if:
        // 1) The notification has a dismiss action
        // 2) The notification has not already been dismissed
        // 3) The notification window is modal (we are not in the app). We can get repeat
        // ANCS UIDs across disconnections and we don't want to dismiss a random
        // notification that happens to get the same UID.
        should_perform_dismiss = true;
      }

      prv_handle_success_fail_response(data, &action_result->attr_list, true /* success */);

      // Perform the dismiss after showing the UI (to try and improve perceived responsiveness)
      if (should_perform_dismiss) {
        // Call this directly so that we don't get another action result
        ancs_perform_action(ancs_uid, ActionIDNegative);
      }
      break;
    }
    case ActionResultTypeChaining:
      prv_handle_chaining_response(data, e);
      break;
    case ActionResultTypeDoResponse:
      prv_handle_do_response_response(data);
      break;
    default:
      prv_cleanup_action_result(data, false /* success */);
      PBL_LOG_WRN("Unknown Action Response");
      break;
  }
}

static void prv_subscribe_to_action_results_and_timeouts(ActionResultData *data,
                                                         bool ignore_failures) {
  data->event_service_info = (EventServiceInfo) {
    .type = PEBBLE_SYS_NOTIFICATION_EVENT,
    .handler = prv_action_handle_response,
    .context = data,
  };
  event_service_client_subscribe(&data->event_service_info);

  data->response.ignore_failures = ignore_failures;
  const unsigned show_progress_timeout_ms = 1 * MS_PER_SECOND;
  data->response.timer = evented_timer_register(show_progress_timeout_ms,
                                                false,
                                                prv_show_progress_window,
                                                data);
}

static void prv_set_action_result(TimelineActionMenu *timeline_action_menu,
                                  ActionResultData *action_result) {
  PBL_ASSERTN(!timeline_action_menu->action_result);
  timeline_action_menu->action_result = action_result;
}

// invoke actions that require a response from a connected remote
static ActionResultData *prv_invoke_remote_action(ActionMenu *action_menu,
                                                  const TimelineItemAction *action,
                                                  const TimelineItem *pin,
                                                  void *context) {
  ActionResultData *data = applib_zalloc(sizeof(ActionResultData));
  if (!data) {
    return NULL;
  }

  data->action_menu = action_menu;
  data->chaining_data.notif = timeline_item_copy((TimelineItem *)pin);

  const bool ignore_failures = false;
  prv_subscribe_to_action_results_and_timeouts(data, ignore_failures);

  if (action_menu) {
    TimelineActionMenu *timeline_action_menu = action_menu_get_context(action_menu);
    prv_set_action_result(timeline_action_menu, data);

    action_menu_freeze(action_menu);
  }

  // perform the action
  switch (action->type) {
    case TimelineItemActionTypeAncsGeneric:
    case TimelineItemActionTypeAncsResponse: {
      // To give the iOS app some context (let it do lookups), give it all the
      // info about the notification

      // Copy every attribtue from the notification and add:
      // - Timestamp attribute
      const int num_extra_attributes = 1;
      const int num_attributes = pin->attr_list.num_attributes + num_extra_attributes;

      AttributeList response_attributes = (AttributeList) {
        .num_attributes = num_attributes,
        .attributes = kernel_zalloc_check(sizeof(Attribute) * num_attributes),
      };
      memcpy(response_attributes.attributes, pin->attr_list.attributes,
             sizeof(Attribute) * pin->attr_list.num_attributes);

      int cur_attribute = pin->attr_list.num_attributes;
      response_attributes.attributes[cur_attribute++] = (Attribute) {
        .id = AttributeIdTimestamp,
        .uint32 = pin->header.timestamp,
      };

      timeline_invoke_action(pin, action, &response_attributes);
      kernel_free(response_attributes.attributes);
      break;
    }
    case TimelineItemActionTypeResponse: {
      data->response.attribute = (Attribute) {
        .id = AttributeIdTitle,
        .cstring = (char *)context,
      };
      Attribute *sender_attr = attribute_find(&pin->attr_list, AttributeIdSender);
      const int16_t num_attributes = (sender_attr) ? 2 : 1;
      Attribute attributes[num_attributes];
      attributes[0] = data->response.attribute;
      if (sender_attr) {
        // Copy the sender attribute - note: this assumes the timeline item is not freed until
        // the message is sent
        attributes[1] = *sender_attr;
      }
      AttributeList response_attributes = {
        .num_attributes = num_attributes,
        .attributes = attributes,
      };
      timeline_invoke_action(pin, action, &response_attributes);
      break;
    }
    case TimelineItemActionTypePostpone: {
      Attribute timestamp_attr = {
        .id = AttributeIdTimestamp,
        .uint32 = (uint32_t)(uintptr_t)context,
      };

      AttributeList response_attributes = {
        .num_attributes = 1,
        .attributes = &timestamp_attr,
      };
      timeline_invoke_action(pin, action, &response_attributes);
      break;
    }
    case TimelineItemActionTypeOpenPin:
    case TimelineItemActionTypeOpenWatchApp:
      WTF;
    case TimelineItemActionTypeAncsNegative:
    case TimelineItemActionTypeAncsDelete:
    case TimelineItemActionTypeAncsPositive:
    case TimelineItemActionTypeAncsDial:
    case TimelineItemActionTypeInsightResponse:
    default:
      timeline_invoke_action(pin, action, NULL);
      break;
  }

  return data;
}

// invoke actions that are immediately handled locally
static void prv_invoke_local_action(const TimelineItemAction *action, const TimelineItem *pin) {
  timeline_invoke_action(pin, action, NULL);
}

static void prv_do_action_analytics(const TimelineItem *pin, const ActionMenuItem *item) {
  const TimelineItemAction *action = item->action_data;

  // Record action in the analytics
  if (action->type == TimelineItemActionTypeOpenWatchApp) {
  } else {
    Uuid app_uuid;
    timeline_get_originator_id(pin, &app_uuid);
  }

  const TimelineItemActionSource current_item_source =
      kernel_ui_get_current_timeline_item_action_source();
  if (current_item_source == TimelineItemActionSourceModalNotification) {
  } else if (current_item_source == TimelineItemActionSourceNotificationApp) {
  } else {
  }
}

#ifdef CONFIG_HRM
static void prv_invoke_ble_hrm_stop_sharing_action(ActionMenu *action_menu,
                                                   const TimelineItem *item) {
  ble_hrm_revoke_all();

  const TimelineItemAction *dismiss_action = timeline_item_find_dismiss_action(item);
  if (dismiss_action) {
    timeline_invoke_action(item, dismiss_action, NULL);
  }

  SimpleDialog *stopped_sharing_dialog = ble_hrm_stop_sharing_popup_create();
  action_menu_set_result_window(action_menu, &stopped_sharing_dialog->dialog.window);
}
#endif

T_STATIC ActionResultData *prv_invoke_action(ActionMenu *action_menu,
                                             const TimelineItemAction *action,
                                             const TimelineItem *pin,
                                             const char *label) {
  switch (action->type) {
    case TimelineItemActionTypeOpenPin:
    case TimelineItemActionTypeOpenWatchApp:
      prv_invoke_local_action(action, pin);
      return NULL;
    case TimelineItemActionTypeAncsResponse:
    case TimelineItemActionTypeAncsGeneric:
    case TimelineItemActionTypeAncsNegative:
    case TimelineItemActionTypeAncsPositive:
    case TimelineItemActionTypeAncsDelete:
    case TimelineItemActionTypeAncsDial:
    case TimelineItemActionTypeGeneric:
    case TimelineItemActionTypeResponse:
    case TimelineItemActionTypeDismiss:
    case TimelineItemActionTypeHttp:
    case TimelineItemActionTypeSnooze:
    case TimelineItemActionTypeRemove:
    case TimelineItemActionTypeInsightResponse:
    case TimelineItemActionTypeComplete:
    case TimelineItemActionTypePostpone:
    case TimelineItemActionTypeRemoteRemove:
      return prv_invoke_remote_action(action_menu, action, pin, (void *)label);
    case TimelineItemActionTypeEmpty:
    case TimelineItemActionTypeUnknown:
      break;
#ifdef CONFIG_HRM
    case TimelineItemActionTypeBLEHRMStopSharing:
      prv_invoke_ble_hrm_stop_sharing_action(action_menu, pin);
      return NULL;
#else
    case TimelineItemActionTypeBLEHRMStopSharing:
      break;
#endif
  }

  PBL_LOG_ERR("Unsupported action type %d", action->type);
  if (action_menu) {
    action_menu_close(action_menu, true);
  }
  return NULL;
}

void timeline_actions_invoke_action(const TimelineItemAction *action, const TimelineItem *pin,
                                    ActionCompleteCallback complete_cb, void *cb_data) {
  ActionResultData *data = prv_invoke_action(NULL /* action_menu */, action, pin, NULL /* label */);

  if (data) {
    data->action_complete.callback = complete_cb;
    data->action_complete.callback_data = cb_data;

    // We can assume that this is a standalone action since we have no action menu
    data->standalone_action = true;
  } else if (complete_cb) {
    // If data is NULL, something went wrong (or local action), so call callback in case caller
    // relies on it for cleanup
    complete_cb(false /* success */, cb_data);
  }
}

static void prv_push_dismiss_first_use_dialog(ActionMenu *action_menu) {
  if (alerts_preferences_check_and_set_first_use_complete(FirstUseSourceDismiss)) {
    return;
  }

  const char* tutorial_msg = i18n_get("Quickly dismiss all notifications by holding the " \
                                      "Select button for 2 seconds from any incoming " \
                                      "notification.", action_menu);

  ExpandableDialog *first_use_dialog = expandable_dialog_create_with_params(
      "Dismiss First Use", RESOURCE_ID_QUICK_DISMISS, tutorial_msg,
      gcolor_legible_over(GColorLightGray), GColorLightGray, NULL,
      RESOURCE_ID_ACTION_BAR_ICON_CHECK, expandable_dialog_close_cb);
  i18n_free(tutorial_msg, action_menu);
  expandable_dialog_push(first_use_dialog, action_menu->window.parent_window_stack);
}

static void prv_action_menu_cb(ActionMenu *action_menu, const ActionMenuItem *item, void *context) {
  TimelineActionMenu *timeline_action_menu = context;
  const TimelineItem *pin = timeline_action_menu->item;
  const TimelineItemAction *action = item->action_data;

  // Quickly make sure our TimelineItem is still (mostly) valid. It is shared with the notification
  // window's UI, and can be easily trampled if we aren't careful
  switch (pin->header.type) {
    case TimelineItemTypeNotification:
    case TimelineItemTypeReminder:
    case TimelineItemTypePin:
      // The item is valid!
      break;
    case TimelineItemTypeUnknown:
    case TimelineItemTypeOutOfRange:
    default:
      PBL_LOG_ERR("Performing action on invalid TimelineItem with type: %d",
              pin->header.type);
      action_menu_close(action_menu, true);
      return;
  }

  prv_do_action_analytics(pin, item);

  if (timeline_item_action_is_dismiss(action)) {
    prv_push_dismiss_first_use_dialog(action_menu);
  }

  prv_invoke_action(action_menu, action, pin, item->label);
}

#ifdef CONFIG_MIC
static void prv_invoke_voice_response(VoiceResponseData *voice_data, char *transcription) {
  // This is a bit of a hack, but we need all the behaviour of timeline_actions_invoke_action and
  // this allows voice responses to be used for other types of responses (i.e. ANCS in the future)
  ActionMenuItem item = {
    .label = transcription,
    .action_data = voice_data->action_data,
  };

  prv_do_action_analytics(voice_data->context, &item);

  ActionResultData *action_result = prv_invoke_remote_action(voice_data->action_menu,
                                                             voice_data->action_data,
                                                             voice_data->context,
                                                             transcription);
  if (action_result) {
    action_result->voice_data = voice_data;
  } else {
    prv_cleanup_voice_data(voice_data);
  }
}
#endif

static ActionMenuLevel *prv_create_level(uint16_t num_items, ActionMenuLevel *parent_level) {
  ActionMenuLevel *level = action_menu_level_create(num_items);
  level->parent_level = parent_level;
  return level;
}

static ActionMenuLevel *prv_create_template_level_from_action(ActionMenuLevel *parent_level,
                                                              TimelineItemAction *action,
                                                              void *i18n_owner) {
  StringList *responses_list =
     attribute_get_string_list(&action->attr_list,
          AttributeIdCannedResponses);
  uint16_t canned_responses_count = responses_list ?
      string_list_count(responses_list) : 0;

  ActionMenuLevel *template_level;
  if (canned_responses_count) {
    // responses as provided by the action
    template_level = prv_create_level(canned_responses_count, parent_level);
    for (size_t i = 0; i < canned_responses_count; i++) {
      const char *label = string_list_get_at(responses_list, i);
      action_menu_level_add_action(template_level, label, prv_action_menu_cb, action);
    }
  } else {
    // hard-wired default responses in case the phone app doesn't provide any
    static char *strings[] = {
      i18n_noop("Ok"),
      i18n_noop("Yes"),
      i18n_noop("No"),
      i18n_noop("Call me"),
      i18n_noop("Call you later"),
    };
    template_level = prv_create_level(ARRAY_LENGTH(strings), parent_level);
    for (size_t i = 0; i < (int)ARRAY_LENGTH(strings); i++) {
      const char *label = i18n_get(strings[i], i18n_owner);
      action_menu_level_add_action(template_level, label, prv_action_menu_cb, action);
    }
  }

  return template_level;
}

static ActionMenuLevel *prv_create_emoji_level_from_action(ActionMenuLevel *parent_level,
                                                           TimelineItemAction *action,
                                                           void *i18n_owner) {
  static const char *short_strings[] = {
    "😃", "😉", "😂", "😍", "😘", "\xe2\x9d\xa4",
    "😇", "😎", "😛", "😟", "😩", "😭", "😴", "😐",
    "😯", "👍", "👎", "👌", "💩", "🎉", "🍺"};
  const uint16_t num_items = ARRAY_LENGTH(short_strings);

  ActionMenuLevel *emoji_level = prv_create_level(num_items, parent_level);
  emoji_level->display_mode = ActionMenuLevelDisplayModeThin;

  for (size_t i = 0; i < num_items; i++) {
    action_menu_level_add_action(emoji_level, short_strings[i], prv_action_menu_cb,
                                 action);
  }

  return emoji_level;
}

#ifdef CONFIG_MIC
static void prv_handle_voice_transcription_result(PebbleEvent *e, void *context) {
  DictationSessionStatus status = e->dictation.result;
  char *transcription = e->dictation.text;
  VoiceResponseData *data = context;

  if (status == DictationSessionStatusSuccess) {
    prv_invoke_voice_response(data, transcription);
  } else {
    action_menu_unfreeze(data->action_menu);
    // TODO: [AS] set reply as selected option
    prv_cleanup_voice_data(data);
  }
}
#endif

#ifdef CONFIG_MIC
static void prv_start_voice_reply(ActionMenu *action_menu,
                                  const ActionMenuItem *item,
                                  void *context) {
  TimelineActionMenu *timeline_action_menu = context;
  action_menu_freeze(action_menu);

  // Make a copy of the TimelineItem to avoid use-after-free if the notification window's
  // swap layer is reloaded while the voice window is active
  TimelineItem *item_copy = timeline_item_copy(timeline_action_menu->item);
  PBL_ASSERTN(item_copy);

  VoiceResponseData *data = applib_malloc(sizeof(VoiceResponseData));
  *data = (VoiceResponseData) {
    .action_data = item->action_data,
    .context = item_copy,
    .action_menu = action_menu,
    .voice_window = voice_window_create(NULL, 0, VoiceEndpointSessionTypeDictation),
  };
  PBL_ASSERTN(data->voice_window);

  data->event_service_info = (EventServiceInfo) {
    .type = PEBBLE_DICTATION_EVENT,
    .handler = prv_handle_voice_transcription_result,
    .context = data,
  };
  event_service_client_subscribe(&data->event_service_info);

  voice_window_transcription_dialog_keep_alive_on_select(data->voice_window, true);

  voice_window_push(data->voice_window);
}
#endif

typedef enum ReplyOption {
  ReplyOption_Voice,
  ReplyOption_Template,
  ReplyOption_Emoji,
  ReplyOptionCount,
} ReplyOption;

static bool prv_is_reply_option_supported(ReplyOption option, TimelineItemAction *action) {
  switch (option) {
    case ReplyOption_Voice:
#ifdef CONFIG_MIC
      return true;
#else
      return false;
#endif
    case ReplyOption_Template:
      return true;
    case ReplyOption_Emoji:
      // If this attribute isn't found, we want to support emoji by default
      return attribute_get_uint8(&action->attr_list, AttributeIdEmojiSupported, true);
    default:
      PBL_LOG_WRN("Unknown reply option");
      return false;
  }
}

static ActionMenuLevel *prv_create_responses_level(TimelineItemAction *action,
                                                   ActionMenuLevel *root_level,
                                                   bool reply_prefix) {
  uint16_t num_items = 0;
  for (ReplyOption reply = 0; reply < ReplyOptionCount; reply++) {
    if (prv_is_reply_option_supported(reply, action)) {
      num_items++;
    }
  }
  ActionMenuLevel *responses_level = prv_create_level(num_items, NULL);
  responses_level->num_items = num_items;

  // If we weren't given a root, assume this is the root level for i18n ownership
  if (!root_level) {
    root_level = responses_level;
  }

  ActionMenuItem reply_options[ReplyOptionCount] = {
#ifdef CONFIG_MIC
    {
      .label = (reply_prefix ? i18n_get("Reply with Voice", root_level) :
                               i18n_get("Voice", root_level)),
      .perform_action = prv_start_voice_reply,
      .action_data = action,
    },
#else
    {
      // This should never get used because prv_is_reply_option_supported() will return false
    },
#endif
    {
      .label = i18n_get("Canned messages", root_level),
      .is_leaf = 0,
    },
    {
      .label = i18n_get("Emoji", root_level),
      .is_leaf = 0,
    },
  };
  ActionMenuLevel *(*level_getters[ReplyOptionCount])
      (ActionMenuLevel *, TimelineItemAction *, void *) = {
    NULL,
    prv_create_template_level_from_action,
    prv_create_emoji_level_from_action,
  };

  unsigned int item = 0;
  for (ReplyOption reply = 0; reply < ReplyOptionCount; reply++) {
    if (!prv_is_reply_option_supported(reply, action)) {
      continue;
    }
    responses_level->items[item] = reply_options[reply];
    // fill with the non-leaves with next level
    if (!reply_options[reply].is_leaf) {
      responses_level->items[item].next_level =
          level_getters[reply](responses_level, action, root_level);
    }
    item++;
  }

  return responses_level;
}

static void prv_postpone_15_minutes(ActionMenu *action_menu,
                                    const ActionMenuItem *action_menu_item,
                                    void *context) {
  TimelineActionMenu *timeline_action_menu = context;
  const TimelineItem *pin = timeline_action_menu->item;
  const TimelineItemAction *action = action_menu_item->action_data;

  time_t new_time = rtc_get_time() + (15 * SECONDS_PER_MINUTE);
  prv_invoke_remote_action(action_menu, action, pin, (void *)(uintptr_t)new_time);
}

static void prv_postpone_later_today(ActionMenu *action_menu,
                                     const ActionMenuItem *action_menu_item,
                                     void *context) {
  TimelineActionMenu *timeline_action_menu = context;
  const TimelineItem *pin = timeline_action_menu->item;
  const TimelineItemAction *action = action_menu_item->action_data;

  // The new time is:
  // 12pm if created before 10am,
  // 6pm if created before 4pm,
  // 2 hours from time of creation otherwise

  time_t utc_sec = rtc_get_time();
  struct tm local_tm;
  localtime_r(&utc_sec, &local_tm);

  if (local_tm.tm_hour < 10) {
    local_tm.tm_hour = 12;
    local_tm.tm_min = 0;
    local_tm.tm_sec = 0;
  } else if (local_tm.tm_hour < 16) {
    local_tm.tm_hour = 18;
    local_tm.tm_min = 0;
    local_tm.tm_sec = 0;
  } else {
    local_tm.tm_hour += 2;
    local_tm.tm_sec = 0;
  }
  time_t new_time = mktime(&local_tm);

  prv_invoke_remote_action(action_menu, action, pin, (void *)(uintptr_t)new_time);
}

static void prv_postpone_tomorrow(ActionMenu *action_menu,
                                  const ActionMenuItem *action_menu_item,
                                  void *context) {
  TimelineActionMenu *timeline_action_menu = context;
  const TimelineItem *pin = timeline_action_menu->item;
  const TimelineItemAction *action = action_menu_item->action_data;

  // The new time is 9am the following day
  time_t tomorrow_utc = rtc_get_time() + SECONDS_PER_DAY;
  struct tm local_tm;
  localtime_r(&tomorrow_utc, &local_tm);
  local_tm.tm_hour = 9;
  local_tm.tm_min = 0;
  local_tm.tm_sec = 0;
  time_t new_time = mktime(&local_tm);

  prv_invoke_remote_action(action_menu, action, pin, (void *)(uintptr_t)new_time);
}

static ActionMenuLevel *prv_create_postpone_level(TimelineItemAction *action,
                                                  ActionMenuLevel *root_level) {
  time_t utc_sec = rtc_get_time();
  struct tm local_tm;
  localtime_r(&utc_sec, &local_tm);

  // Only show the "later today" option if it is before 8pm
  const bool show_later_today = local_tm.tm_hour >= 20 ? false : true;
  const uint8_t num_postpone_items = show_later_today ? 3 : 2;
  ActionMenuLevel *postpone_level = action_menu_level_create(num_postpone_items);

  // If we weren't given a root, assume this is the root level for i18n ownership
  if (!root_level) {
    root_level = postpone_level;
  }


  action_menu_level_add_action(postpone_level,
                               i18n_get("In 15 minutes", root_level),
                               prv_postpone_15_minutes,
                               action);

  if (show_later_today) {
    action_menu_level_add_action(postpone_level,
                                 i18n_get("Later today", root_level),
                                 prv_postpone_later_today,
                                 action);
  }

  action_menu_level_add_action(postpone_level,
                               i18n_get("Tomorrow", root_level),
                               prv_postpone_tomorrow,
                               action);

  return postpone_level;
}

void timeline_actions_add_action_to_root_level(TimelineItemAction *action,
                                               ActionMenuLevel *root_level) {
  const char *label = attribute_get_string(&action->attr_list, AttributeIdTitle, "[Action]");
  if (action->type == TimelineItemActionTypeResponse) {
    ActionMenuLevel *responses_level = prv_create_responses_level(action, root_level,
                                                                  false /* reply_prefix */);
    action_menu_level_add_child(root_level, responses_level, label);
  } else if (action->type == TimelineItemActionTypePostpone) {
    ActionMenuLevel *responses_level = prv_create_postpone_level(action, root_level);
    action_menu_level_add_child(root_level, responses_level, label);
  } else {
    action_menu_level_add_action(root_level, label, prv_action_menu_cb, action);
  }
}

ActionMenuLevel *timeline_actions_create_action_menu_root_level(uint8_t num_items,
                                                                uint8_t separator_index,
                                                                TimelineItemActionSource source) {
  kernel_ui_set_current_timeline_item_action_source(source);

  ActionMenuLevel *root_level = prv_create_level(num_items, NULL);
  root_level->separator_index = separator_index;

  prv_request_responsive_session();

  return root_level;
}

static void prv_cleanup_action_menu(ActionMenu *action_menu) {
  ActionMenuLevel *root_level = action_menu_get_root_level(action_menu);
  action_menu_hierarchy_destroy(root_level, NULL, NULL);
  i18n_free_all(root_level);
  prv_reset_session_responsiveness();
}

static void prv_timeline_action_menu_did_close(ActionMenu *action_menu, const ActionMenuItem *item,
                                               void *context) {
  TimelineActionMenu *timeline_action_menu = context;
  if (timeline_action_menu->did_close) {
    timeline_action_menu->did_close(action_menu, item, timeline_action_menu->item);
  }
  if (timeline_action_menu->action_result) {
    timeline_action_menu->action_result->action_menu = NULL;
  }
  prv_cleanup_action_menu(action_menu);
  applib_free(timeline_action_menu);
}

ActionMenu *timeline_actions_push_action_menu(ActionMenuConfig *base_config,
                                              WindowStack *window_stack) {
  PBL_ASSERTN(base_config);
  TimelineActionMenu *timeline_action_menu = applib_zalloc(sizeof(TimelineActionMenu));
  if (!timeline_action_menu) {
    return NULL;
  }

  ActionMenuConfig config = *base_config;
  config.context = timeline_action_menu;
  if (gcolor_equal(config.colors.foreground, GColorClear)) {
    config.colors.foreground = gcolor_legible_over(base_config->colors.background);
  }
  config.did_close = prv_timeline_action_menu_did_close;

  timeline_action_menu->item = base_config->context;
  timeline_action_menu->action_menu = action_menu_open(window_stack, &config);
  timeline_action_menu->did_close = base_config->did_close;

  return timeline_action_menu->action_menu;
}

ActionMenu *timeline_actions_push_response_menu(
    TimelineItem *item, TimelineItemAction *reply_action, GColor bg_color,
    ActionMenuDidCloseCb did_close_cb, WindowStack *window_stack, TimelineItemActionSource source,
    bool standalone_reply) {
  kernel_ui_set_current_timeline_item_action_source(source);
  prv_request_responsive_session();
  ActionMenuConfig config = {
    .context = item,
    .colors.background = bg_color,
    .did_close = did_close_cb,
    .root_level = prv_create_responses_level(reply_action, NULL, standalone_reply),
  };
  return timeline_actions_push_action_menu(&config, window_stack);
}

void timeline_actions_dismiss_all(NotificationInfo *notif_list, int num_notifications,
                                  ActionMenu *action_menu,
                                  ActionCompleteCallback dismiss_all_complete_callback,
                                  void *dismiss_all_cb_data) {
  ActionResultData *data = applib_zalloc(sizeof(ActionResultData));
  if (!data) {
    return;
  }

  data->action_menu = action_menu;

  data->action_complete.callback = dismiss_all_complete_callback;
  data->action_complete.callback_data = dismiss_all_cb_data;

  // When performing a bulk request (dismiss all) errors are silently ignored because we want to
  // show success if 1 or more actions were successful. If every result is an error the
  // timeout handler will convey the error message
  const bool ignore_failures = true;
  prv_subscribe_to_action_results_and_timeouts(data, ignore_failures);
  bool performed_actions = false;

  if (action_menu) {
    TimelineActionMenu *timeline_action_menu = action_menu_get_context(action_menu);
    prv_set_action_result(timeline_action_menu, data);

    action_menu_freeze(action_menu);

    // We only show the first use tutorial if this was called from an action menu
    prv_push_dismiss_first_use_dialog(action_menu);
  }

  for (int i = 0; i < num_notifications; i++) {
    TimelineItem item;
    if (notif_list[i].type == NotificationReminder) {
      if (reminder_db_read_item(&item, &notif_list[i].id) != S_SUCCESS) {
        PBL_LOG_ERR("Trying to dismiss all an invalid reminder");
        continue;
      }
    } else if (notif_list[i].type == NotificationMobile) {
      if (!notification_storage_get(&notif_list[i].id, &item)) {
        PBL_LOG_ERR("Trying to dismiss all an invalid notification");
        continue;
      }
    }

    const TimelineItemAction *action = timeline_item_find_dismiss_action(&item);
    if (action) {
      timeline_invoke_action(&item, action, NULL);
      performed_actions = true;

      // FIXME: PBL-34338 There are other actions that should also use bulk mode to avoid crashes
      // such as dismissing notifications on Android while disconnected
      if (action->type == TimelineItemActionTypeAncsNegative) {
        timeline_enable_ancs_bulk_action_mode(true);
      }
    }
    timeline_item_free_allocated_buffer(&item);
  }

  // It's safe to do this even if we didn't enable it
  timeline_enable_ancs_bulk_action_mode(false);

  if (!performed_actions) {
    PBL_LOG_DBG("Didn't take any actions, cleaning up");
    const bool success = false;
    prv_cleanup_action_result(data, success);
  }
}
