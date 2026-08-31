/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "send_text.h"
#include "prefs.h"

#include "applib/app.h"
#include "applib/app_exit_reason.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/ui.h"
#include "apps/system/timeline/peek_layer.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/blob_db/ios_notif_pref_db.h"
#include "pbl/services/blob_db/watch_app_prefs_db.h"
#include "pbl/services/contacts/contacts.h"
#include "pbl/services/notifications/notification_constants.h"
#include "pbl/services/send_text_service.h"
#include "pbl/services/timeline/timeline.h"
#include "pbl/services/timeline/timeline_actions.h"
#include "shell/prefs.h"
#include <pbl/logging/logging.h>
#include "util/time/time.h"

#include <string.h>

#define SEND_TEXT_APP_HIGHLIGHT_COLOR PBL_IF_COLOR_ELSE(SMS_REPLY_COLOR, GColorBlack)

typedef int ContactId;

typedef struct {
  ListNode node;
  ContactId id;
  char *name;
  char *display_number;
  char *number; // Points to a substring within display_number (the part without the ❤)
} ContactNode;

typedef struct SendTextAppData {
  Window window;
  MenuLayer menu_layer;
  PeekLayer no_contacts_layer;
  StatusBarLayer status_layer;

  ContactNode *contact_list_head;

  EventServiceInfo event_service_info;
} SendTextAppData;


///////////////////////////////////////////////////////////////////////////////////////////////////
//! Action menu functions

static void prv_action_menu_did_close(ActionMenu *action_menu, const ActionMenuItem *item,
                                      void *context) {
  TimelineItem *timeline_item = context;
  timeline_item_destroy(timeline_item);
}

static void prv_action_handle_response(PebbleEvent *e, void *context) {
  SendTextAppData *data = context;

  if (e->sys_notification.type != NotificationActionResult) {
    // Not what we want
    return;
  }

  PebbleSysNotificationActionResult *action_result = e->sys_notification.action_result;
  if (action_result == NULL) {
    return;
  }

  // Each action result can only service one response event
  event_service_client_unsubscribe(&data->event_service_info);

  if (action_result->type == ActionResultTypeSuccess) {
    // Set the exit reason as "action performed successfully" so we return to the watchface
    // when we remove the window from the stack to exit the app
    app_exit_reason_set(APP_EXIT_ACTION_PERFORMED_SUCCESSFULLY);
    app_window_stack_remove(&data->window, false);
  }
}

static TimelineItem *prv_create_timeline_item(const char *number) {
  iOSNotifPrefs *notif_prefs = ios_notif_pref_db_get_prefs((uint8_t *)SEND_TEXT_NOTIF_PREF_KEY,
                                                           strlen(SEND_TEXT_NOTIF_PREF_KEY));
  if (!notif_prefs) {
    return NULL;
  }

  AttributeList attr_list = {};
  attribute_list_add_cstring(&attr_list, AttributeIdSender, number);

  TimelineItem *item = timeline_item_create_with_attributes(0, 0,
                                                            TimelineItemTypeNotification,
                                                            LayoutIdNotification,
                                                            &attr_list,
                                                            &notif_prefs->action_group);
  if (item) {
    item->header.id = (Uuid)UUID_SEND_SMS;
    item->header.parent_id = (Uuid)UUID_SEND_TEXT_DATA_SOURCE;
  }

  attribute_list_destroy_list(&attr_list);
  ios_notif_pref_db_free_prefs(notif_prefs);

  return item;
}

static void prv_open_action_menu(SendTextAppData *data, const char *number) {
  TimelineItem *item = prv_create_timeline_item(number);

  // This handles the case where item is NULL, so no need to check for that
  TimelineItemAction *reply_action = timeline_item_find_action_by_type(
      item, TimelineItemActionTypeResponse);

  if (!reply_action) {
    PBL_LOG_ERR("Not opening response menu - unable to load reply action");
    timeline_item_destroy(item);
    return;
  }

  timeline_actions_push_response_menu(item, reply_action, SEND_TEXT_APP_HIGHLIGHT_COLOR,
                                      prv_action_menu_did_close, data->window.parent_window_stack,
                                      TimelineItemActionSourceSendTextApp,
                                      false /* standalone_reply */);

  data->event_service_info = (EventServiceInfo) {
    .type = PEBBLE_SYS_NOTIFICATION_EVENT,
    .handler = prv_action_handle_response,
    .context = data,
  };
  event_service_client_subscribe(&data->event_service_info);
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//! Contact list functions

static void prv_clear_contact_list(SendTextAppData *data) {
  while (data->contact_list_head) {
    ContactNode *old_head = data->contact_list_head;
    data->contact_list_head = (ContactNode *)list_pop_head((ListNode *)old_head);

    // Only need to free display_number since number shares the same buffer
    app_free(old_head->display_number);
    app_free(old_head->name);
    app_free(old_head);
  }
}

static void prv_add_contact_to_list(ContactId id, const char *name, const char *number, bool is_fav,
                                    void *callback_context) {
  SendTextAppData *data = callback_context;

  ContactNode *new_node = app_zalloc_check(sizeof(ContactNode));
  list_init((ListNode *)new_node);
  new_node->id = id;

  const size_t name_size = strlen(name) + 1;
  new_node->name = app_zalloc_check(name_size);
  strcpy(new_node->name, name);

  const char *fav_str = (is_fav ? "❤ " : "");
  const size_t display_number_size = strlen(fav_str) + strlen(number) + 1;
  new_node->display_number = app_zalloc_check(display_number_size);
  strcpy(new_node->display_number, fav_str);
  strcat(new_node->display_number, number);

  // Store the number string (the part after "fav_str") to forward to the phone
  new_node->number = (new_node->display_number + strlen(fav_str));

  list_append((ListNode *)data->contact_list_head, (ListNode *)new_node);

  if (!data->contact_list_head) {
    data->contact_list_head = new_node;
  }
}

static void prv_read_contacts_from_prefs(SendTextAppData *data) {
  SerializedSendTextPrefs *prefs = watch_app_prefs_get_send_text();
  if (prefs) {
    int num_contacts = 0;

    for (int i = 0; i < prefs->num_contacts; i++) {
      SerializedSendTextContact *pref = &prefs->contacts[i];

      Contact *contact = contacts_get_contact_by_uuid(&pref->contact_uuid);
      if (!contact) {
        continue;
      }

      for (int j = 0; j < contact->addr_list.num_addresses; j++) {
        if (uuid_equal(&contact->addr_list.addresses[j].id, &pref->address_uuid)) {
          const char *name = attribute_get_string(&contact->attr_list, AttributeIdTitle,
                                                  (char *)i18n_get("Unknown", data));
          const char *number = attribute_get_string(&contact->addr_list.addresses[j].attr_list,
                                                    AttributeIdAddress, "");
          prv_add_contact_to_list(num_contacts++, name, number, pref->is_fav, data);
        }
      }

      contacts_free_contact(contact);
    }
  }

  task_free(prefs);
}

static void prv_update_contact_list(SendTextAppData *data) {
  prv_clear_contact_list(data);
  prv_read_contacts_from_prefs(data);
}

static bool prv_has_contacts(SendTextAppData *data) {
  return (list_count((ListNode *)data->contact_list_head) > 0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Menu Layer Callbacks

static uint16_t prv_contact_list_get_num_rows_callback(MenuLayer *menu_layer,
                                                       uint16_t section_index,
                                                       void *callback_context) {
  SendTextAppData *data = callback_context;
  return (uint16_t)list_count((ListNode *)data->contact_list_head);
}

#if PBL_ROUND
static int16_t prv_contact_list_get_header_height_callback(MenuLayer *menu_layer,
                                                           uint16_t section_index,
                                                           void *callback_context) {
  return MENU_CELL_ROUND_UNFOCUSED_SHORT_CELL_HEIGHT;
}
#endif

static int16_t prv_contact_list_get_cell_height_callback(MenuLayer *menu_layer,
                                                         MenuIndex *cell_index,
                                                         void *callback_context) {
  return PBL_IF_RECT_ELSE(menu_cell_basic_cell_height(),
                          (menu_layer_is_index_selected(menu_layer, cell_index) ?
                           MENU_CELL_ROUND_FOCUSED_SHORT_CELL_HEIGHT :
                           MENU_CELL_ROUND_UNFOCUSED_TALL_CELL_HEIGHT));
}

#if PBL_ROUND
static void prv_contact_list_draw_header_callback(GContext *ctx, const Layer *cell_layer,
                                                  uint16_t section_index, void *callback_context) {
  SendTextAppData *data = callback_context;
  const GFont font = fonts_get_system_font(FONT_KEY_GOTHIC_14);
  GRect box = cell_layer->bounds;
  box.origin.y -= 2;
  graphics_context_set_text_color(ctx, GColorDarkGray);
  graphics_draw_text(ctx, i18n_get("Select Contact", data), font, box, GTextOverflowModeFill,
                     GTextAlignmentCenter, NULL);
}
#endif

static void prv_contact_list_draw_row_callback(GContext *ctx, const Layer *cell_layer,
                                               MenuIndex *cell_index, void *callback_context) {
  SendTextAppData *data = (SendTextAppData *)callback_context;

  ContactNode *node = (ContactNode *)list_get_at((ListNode *)data->contact_list_head,
                                                 cell_index->row);
  if (!node) {
    return;
  }

  menu_cell_basic_draw(ctx, cell_layer, node->name, node->display_number, NULL);
}

static void prv_contact_list_select_callback(MenuLayer *menu_layer, MenuIndex *cell_index,
                                             void *callback_context) {
  SendTextAppData *data = (SendTextAppData *)callback_context;

  ContactNode *node = (ContactNode *)list_get_at((ListNode *)data->contact_list_head,
                                                 cell_index->row);
  if (!node) {
      return;
    }

  prv_open_action_menu(data, node->number);
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//! App boilerplate

static void prv_init(void) {
  SendTextAppData *data = app_zalloc_check(sizeof(SendTextAppData));
  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, WINDOW_NAME("Send Text"));
  window_set_user_data(window, data);

  Layer *window_root_layer = &window->layer;

  prv_update_contact_list(data);

  if (prv_has_contacts(data)) {
    const GRect menu_layer_frame =
        grect_inset(window_root_layer->bounds,
                    GEdgeInsets(STATUS_BAR_LAYER_HEIGHT, 0,
                                PBL_IF_ROUND_ELSE(STATUS_BAR_LAYER_HEIGHT, 0), 0));
    menu_layer_init(&data->menu_layer, &menu_layer_frame);
    menu_layer_set_callbacks(&data->menu_layer, data, &(MenuLayerCallbacks) {
      .get_num_rows = prv_contact_list_get_num_rows_callback,
      .get_cell_height = prv_contact_list_get_cell_height_callback,
      // On round we show the "Select Contact" text in a menu cell header, but on rect we show it
      // in the status bar (see below)
#if PBL_ROUND
      .draw_header = prv_contact_list_draw_header_callback,
      .get_header_height = prv_contact_list_get_header_height_callback,
#endif
      .draw_row = prv_contact_list_draw_row_callback,
      .select_click = prv_contact_list_select_callback,
    });

    menu_layer_set_highlight_colors(&data->menu_layer, SEND_TEXT_APP_HIGHLIGHT_COLOR, GColorWhite);
    menu_layer_set_click_config_onto_window(&data->menu_layer, &data->window);
    menu_layer_set_scroll_wrap_around(&data->menu_layer, shell_prefs_get_menu_scroll_wrap_around_enable());
    menu_layer_set_scroll_vibe_on_wrap(&data->menu_layer, shell_prefs_get_menu_scroll_vibe_behavior() == MenuScrollVibeOnWrapAround);
    menu_layer_set_scroll_vibe_on_blocked(&data->menu_layer, shell_prefs_get_menu_scroll_vibe_behavior() == MenuScrollVibeOnLocked);
    layer_add_child(&data->window.layer, menu_layer_get_layer(&data->menu_layer));

    StatusBarLayer *status_layer = &data->status_layer;
    status_bar_layer_init(status_layer);
    status_bar_layer_set_colors(status_layer, GColorClear, GColorBlack);
    // On rect we show the "Select Contact" text in the status bar, but on round the status bar
    // shows the clock time and we use a menu cell header to display "Select Contact" (see above)
#if PBL_RECT
    status_bar_layer_set_title(status_layer, i18n_get("Select Contact", data), false /* revert */,
                               false /* animate */);
    status_bar_layer_set_separator_mode(status_layer, StatusBarLayerSeparatorModeDotted);
#endif
    layer_add_child(window_root_layer, status_bar_layer_get_layer(&data->status_layer));
  } else {
    PeekLayer *peek_layer = &data->no_contacts_layer;
    peek_layer_init(peek_layer, &data->window.layer.bounds);
    const GFont title_font = system_theme_get_font_for_default_size(TextStyleFont_Title);
    peek_layer_set_title_font(peek_layer, title_font);
    TimelineResourceInfo timeline_res = {
      .res_id = TIMELINE_RESOURCE_GENERIC_WARNING,
    };
    peek_layer_set_icon(peek_layer, &timeline_res);
    peek_layer_set_title(peek_layer, i18n_get("Add contacts in\nmobile app", data));
    peek_layer_set_background_color(peek_layer, GColorLightGray);
    peek_layer_play(peek_layer);
    layer_add_child(window_root_layer, &peek_layer->layer);
  }

  const bool animated = true;
  app_window_stack_push(&data->window, animated);
}

static void prv_deinit(void) {
  SendTextAppData *data = app_state_get_user_data();
  event_service_client_unsubscribe(&data->event_service_info);
  status_bar_layer_deinit(&data->status_layer);
  peek_layer_deinit(&data->no_contacts_layer);
  menu_layer_deinit(&data->menu_layer);
  i18n_free_all(data);
  prv_clear_contact_list(data);
  app_free(data);
}

static void prv_main(void) {
  prv_init();
  app_event_loop();
  prv_deinit();
}

const PebbleProcessMd *send_text_app_get_info(void) {
  static const PebbleProcessMdSystem s_send_text_app_info = {
    .common = {
      .main_func = prv_main,
      .uuid = UUID_SEND_TEXT_DATA_SOURCE,
    },
    .name = i18n_noop("Send Text"),
    .icon_resource_id = RESOURCE_ID_SEND_TEXT_APP_GLANCE,
  };

  // If the phone doesn't support this app, we will act as if it's not installed by returning NULL
  const bool app_supported = send_text_service_is_send_text_supported();
  return app_supported ? (const PebbleProcessMd *)&s_send_text_app_info : NULL;
}
