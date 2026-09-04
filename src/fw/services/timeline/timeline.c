/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/timeline/timeline.h"

#include "pbl/util/uuid.h"
#include "applib/ui/status_bar_layer.h"
#include "apps/system/timeline/timeline.h"
#include "apps/system/timeline/pin_window.h"
#include "comm/ble/kernel_le_client/ancs/ancs.h"
#include "comm/ble/kernel_le_client/ancs/ancs_types.h"
#include <pbl/drivers/rtc.h>
#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"
#include "kernel/ui/modals/modal_manager.h"
#include "process_management/app_install_manager.h"
#include "process_management/app_manager.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/activity/activity_insights.h"
#include "pbl/services/blob_db/api.h"
#include "pbl/services/blob_db/pin_db.h"
#include "pbl/services/blob_db/reminder_db.h"
#include "pbl/services/notifications/notification_storage.h"
#include "pbl/services/notifications/notifications.h"
#include "pbl/services/phone_call_util.h"
#include "pbl/services/timeline/actions_endpoint.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/list.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"
#include "util/time/time.h"

PBL_LOG_MODULE_DEFINE(service_timeline, CONFIG_SERVICE_TIMELINE_LOG_LEVEL);

struct TimelineNode {
  ListNode node;
  int index;
  Uuid id;
  time_t timestamp;
  uint16_t duration;
  bool all_day;
};

static uint32_t i18n_key;

#define TIMELINE_FUTURE_WINDOW (3 * SECONDS_PER_DAY)
#define TIMELINE_PAST_WINDOW (2 * SECONDS_PER_DAY)

static bool s_bulk_action_mode = false;

/////////////////////////
// Timeline Iterator
/////////////////////////

// Order of events in timeline:
// * All day events appear first
// * (All day events should be timestamped at midnight, the first second in the day)
// * Order all other events by time
// * For concurrent events: order by duration (shortest to longest), then by (TODO) A-Z
// * Events that occur now appear both in timeline past and timeline future until event ends
static int prv_time_comparator(void *a, void *b) {
  TimelineNode *node_a = (TimelineNode *)a;
  TimelineNode *node_b = (TimelineNode *)b;
  if (node_b->timestamp == node_a->timestamp) {
    if (node_b->all_day) {
      return -1;
    } else if (node_a->all_day) {
      return 1;
    } else {
      return (node_b->duration - node_a->duration);
    }
  } else {
    return (node_b->timestamp - node_a->timestamp);
  }
}

static bool prv_filter(ListNode *found_node, void *data) {
  TimelineNode *node = (TimelineNode *)found_node;
  Uuid *uuid = (Uuid *)data;
  return uuid_equal(&node->id, uuid);
}

static TimelineNode *prv_find_by_uuid(TimelineNode *head, Uuid *uuid) {
  TimelineNode *node = (TimelineNode *)list_find((ListNode *)head, prv_filter, uuid);
  return node;
}

static bool prv_is_in_window(time_t node_timestamp, uint16_t node_duration, time_t timestamp) {
  time_t future_window = time_util_get_midnight_of(timestamp + TIMELINE_FUTURE_WINDOW);
  time_t past_window = time_util_get_midnight_of(timestamp - TIMELINE_PAST_WINDOW);
  time_t end_time = node_timestamp + (node_duration * SECONDS_PER_MINUTE);
  time_t start_time = node_timestamp;

  return !(start_time >= future_window || end_time < past_window);
}

static bool prv_show_event(TimelineNode *node, time_t timestamp, time_t midnight,
  TimelineIterDirection direction, bool show_all_day_events) {
  // hide events outside of the window
  if (!prv_is_in_window(node->timestamp, node->duration, timestamp)) {
    return false;
  }

  // An event is in future until it ends
  const time_t fudge_time = node->duration * SECONDS_PER_MINUTE;
  // deal with all day events
  if (node->all_day && node->timestamp == midnight) {
    return show_all_day_events;
  } else if (direction == TimelineIterDirectionFuture) {
    return (node->timestamp >= timestamp - fudge_time);
  } else { // direction == TimelineIterDirectionPast
    return (node->timestamp < timestamp - fudge_time);
  }
}

// All day events show up in future if no timed events have passed today,
// i.e. no events exist between midnight today and now
// iterate and figure out if we had a timed event pass today
static bool prv_should_show_all_day_events(TimelineNode *head, time_t now, time_t today_midnight,
  TimelineIterDirection direction) {
  TimelineNode *current = head;
  // show in future / hide in past all day events unless we find a timed event
  // between midnight and now
  bool show = direction == TimelineIterDirectionFuture;
  while (current) {
    if (current->timestamp > now) {
      break;
    }
    if (!current->all_day && (current->timestamp >= today_midnight)) {
      show = !show;
      break;
    }
    current = (TimelineNode *)current->node.next;
  }
  return show;
}

static TimelineNode *prv_find_first_past(TimelineNode *head, time_t timestamp,
  time_t today_midnight, bool show_all_day_events) {
  TimelineNode *current = (TimelineNode *)list_get_tail((ListNode *)head);
  while (current) {
    if (prv_show_event(current, timestamp, today_midnight, TimelineIterDirectionPast,
     show_all_day_events)) {
      break;
    }
    current = (TimelineNode *)current->node.prev;
  }
  return current;
}

static TimelineNode *prv_find_first_future(TimelineNode *head, time_t timestamp,
  time_t today_midnight, bool show_all_day_events) {
  TimelineNode *current = head;
  while (current) {
    if (prv_show_event(current, timestamp, today_midnight, TimelineIterDirectionFuture,
      show_all_day_events)) {
      break;
    }
    current = (TimelineNode *)current->node.next;
  }
  return current;
}

static TimelineNode *prv_find_first(TimelineNode *head, TimelineIterDirection direction,
  time_t timestamp, time_t today_midnight, bool show_all_day_events) {
  if (direction == TimelineIterDirectionPast) {
    return prv_find_first_past(head, timestamp, today_midnight, show_all_day_events);
  } else {
    return prv_find_first_future(head, timestamp, today_midnight, show_all_day_events);
  }
}

static void prv_remove_node(TimelineNode **head, TimelineNode *node) {
  list_remove((ListNode *)node, (ListNode **)head, NULL);
  task_free(node);
}

static int prv_num_nodes_for_serialized_item(CommonTimelineItemHeader *header) {
  int num_days;
  if (header->all_day) {
    num_days = header->duration ? (header->duration + MINUTES_PER_DAY - 1) / MINUTES_PER_DAY : 1;
  } else {
    // The span is the time between 0:00 on the first day of the event
    // and 24:00 on the last day of the event
    const time_t start_span = time_util_get_midnight_of(header->timestamp - SECONDS_PER_DAY + 1);
    const time_t end_span = time_util_get_midnight_of(header->timestamp + header->duration *
                                                      SECONDS_PER_MINUTE - 1);
    const time_t full_span = end_span - start_span;
    num_days = full_span / SECONDS_PER_DAY;
  }
  return MAX(num_days, 1);
}

static void prv_set_nodes(TimelineNode *nodes[], CommonTimelineItemHeader *header, int num_nodes) {
  // Multiday events:
  // first day: timestamp at beginning of event, duration for rest of the day
  // middle days: all day events
  // end days: event at end time
  //
  // single event:
  // timestamp, duration are same as the original item
  nodes[0]->timestamp = timeline_item_get_tz_timestamp(header);
  const time_t midnight_first = time_util_get_midnight_of(nodes[0]->timestamp);
  if (num_nodes == 1) {
    nodes[0]->duration = header->duration;
  } else {
    // first item has correct timestamp, duration should make it last for the rest of the day
    const time_t until_midnight = midnight_first + SECONDS_PER_DAY - header->timestamp;
    nodes[0]->duration = until_midnight / SECONDS_PER_MINUTE;

    // last item at end of event, duration 0
    time_t endtime = header->timestamp + header->duration * SECONDS_PER_MINUTE;
    nodes[num_nodes - 1]->timestamp = endtime;
    nodes[num_nodes - 1]->duration = 0;
    nodes[num_nodes - 1]->all_day = false;
  }
  nodes[0]->all_day = (nodes[0]->duration == MINUTES_PER_DAY &&
                       nodes[0]->timestamp == midnight_first);

  // middle days are all day events
  time_t midnight = time_util_get_midnight_of(header->timestamp);
  for (int i = 1; i < num_nodes - 1; i++) {
    midnight += SECONDS_PER_DAY;
    nodes[i]->timestamp = midnight;
    nodes[i]->duration = MINUTES_PER_DAY;
    nodes[i]->all_day = true;
  }
}

static void prv_set_nodes_all_day(TimelineNode *nodes[], CommonTimelineItemHeader *header,
  int num_nodes) {
  // Multiday events:
  // Each day is an all day event

  time_t midnight;
  // iOS doesn't correctly send the timestamp at UTC midnight, rather it sends it in local time
  if (header->timestamp % SECONDS_PER_DAY != 0) {
    // NOT at UTC midnight, so presumably an iOS bug
    midnight = time_util_get_midnight_of(header->timestamp);
  } else {
    midnight = timeline_item_get_tz_timestamp(header);
  }
  for (int i = 0; i < num_nodes; i++) {
    nodes[i]->timestamp = midnight;
    nodes[i]->duration = MINUTES_PER_DAY;
    nodes[i]->all_day = true;
    midnight += SECONDS_PER_DAY;
  }
}

static void prv_add_nodes_for_serialized_item(TimelineNode **list_head,
  CommonTimelineItemHeader *header) {
  int num_nodes = prv_num_nodes_for_serialized_item(header);
  TimelineNode *nodes[num_nodes];

  // copy UUID to all the nodes
  for (int i = 0; i < num_nodes; i++) {
    // each node requires its own malloc because each node must be individually free-able
    nodes[i] = task_malloc_check(sizeof(TimelineNode));
    *(nodes[i]) = (TimelineNode){};
    nodes[i]->id = header->id;
  }

  if (header->all_day) {
    prv_set_nodes_all_day(nodes, header, num_nodes);
  } else {
    prv_set_nodes(nodes, header, num_nodes);
  }

  for (int i = 0; i < num_nodes; i++) {
    *list_head = (TimelineNode *)list_sorted_add((ListNode *)*list_head, (ListNode *)nodes[i],
      prv_time_comparator, true);
  }
}

static bool prv_each(SettingsFile *file, SettingsRecordInfo *info, void *context) {
  // check entry is valid
  if (info->key_len != UUID_SIZE || info->val_len == 0) {
    return true; // continue iteration
  }

  TimelineNode **list_head = context;

  CommonTimelineItemHeader header;
  // we don't care about the attributes here, so we don't allocate space for them
  info->get_val(file, (uint8_t *)&header, sizeof(CommonTimelineItemHeader));
  // Flags & Status are stored inverted.
  header.flags = ~header.flags;
  header.status = ~header.status;

  prv_add_nodes_for_serialized_item(list_head, &header);

  return true; // continue iteration
}
static void prv_set_indices(TimelineNode *timeline) {
  TimelineNode *node = timeline;
  int index = 0;
  while (node) {
    node->index = index;
    index++;
    node = (TimelineNode *)list_get_next((ListNode *)node);
  }
}

static int prv_first_event_comparator(TimelineNode *new_node, TimelineNode *old_node,
                                      TimelineIterDirection direction) {
  const time_t now = rtc_get_time();
  const time_t midnight = time_util_get_midnight_of(now);
  const bool show_all_day = false;
  const bool show_new = prv_show_event(new_node, now, midnight, direction, show_all_day);
  const bool show_old = prv_show_event(old_node, now, midnight, direction, show_all_day);
  if (show_new != show_old) {
    return (show_old ? 1 : 0) - (show_new ? 1 : 0);
  } else {
    return prv_time_comparator(old_node, new_node);
  }
}

static void prv_set_node_from_header(CommonTimelineItemHeader *header, TimelineNode *node_out) {
  prv_set_nodes(&(TimelineNode *) { node_out }, header, 1 /* num_nodes */);
}

int timeline_item_time_comparator(CommonTimelineItemHeader *new_common,
                                  CommonTimelineItemHeader *old_common,
                                  TimelineIterDirection direction) {
  TimelineNode new_node;
  TimelineNode old_node;
  prv_set_node_from_header(new_common, &new_node);
  prv_set_node_from_header(old_common, &old_node);
  return prv_first_event_comparator(&new_node, &old_node, direction);
}

bool timeline_item_should_show(CommonTimelineItemHeader *header, TimelineIterDirection direction) {
  TimelineNode node;
  prv_set_node_from_header(header, &node);
  const time_t now = rtc_get_time();
  const time_t midnight = time_util_get_midnight_of(now);
  return prv_show_event(&node, now, midnight, direction, false /* show_all_day */);
}

#ifdef TIMELINE_SERVICE_DEBUG
static void prv_debug_print_pins(TimelineNode *node0) {
  TimelineNode *node = (TimelineNode *)list_get_head((ListNode *)node0);
  PBL_LOG_DBG("= = = = = = = =");
  while (node) {
    PBL_LOG_DBG("======");
    PBL_LOG_DBG("Node with id %x%x...", node->id.byte0, node->id.byte1);
    PBL_LOG_DBG("Index %d", node->index);
    PBL_LOG_DBG("Timestamp %ld", node->timestamp);
    PBL_LOG_DBG("Duration %hu", node->duration);
    PBL_LOG_DBG("All day? %s", node->all_day ? "True": "False");
    PBL_LOG_DBG("Address %p", node);
    node = (TimelineNode *)node->node.next;
  }
}
#endif

// dummy iterator that always returns false
// Useful for when there aren't any items in pindb
// but we don't want an invalid iterator.
static bool prv_iter_dummy(IteratorState state) {
  return false;
}

static bool prv_iter_next(IteratorState state) {
  TimelineIterState *timeline_iter_state = (TimelineIterState *)state;
  if (timeline_iter_state->node == NULL) {
    return false;
  }
  // keep a copy of the original node in case we go to the end without finding a new valid node
  TimelineNode *orig = timeline_iter_state->node;
  do {
    timeline_iter_state->node = (TimelineNode *)timeline_iter_state->node->node.next;
    if (timeline_iter_state->node == NULL) {
      timeline_iter_state->node = orig;
      return false;
    }
  } while (!prv_show_event(timeline_iter_state->node, timeline_iter_state->start_time,
      timeline_iter_state->midnight, timeline_iter_state->direction,
      timeline_iter_state->show_all_day_events) ||
      !timeline_exists(&timeline_iter_state->node->id));

  timeline_item_free_allocated_buffer(&timeline_iter_state->pin);
  timeline_iter_state->pin = (TimelineItem){};

  status_t rv = pin_db_get(&timeline_iter_state->node->id, &timeline_iter_state->pin);
  timeline_iter_state->current_day = time_util_get_midnight_of(
    timeline_iter_state->node->timestamp);
  timeline_iter_state->index = timeline_iter_state->node->index;
#ifdef TIMELINE_SERVICE_DEBUG
  prv_debug_print_pins(timeline_iter_state->node);
#endif
  return (rv == S_SUCCESS);
}

static bool prv_iter_prev(IteratorState state) {
  TimelineIterState *timeline_iter_state = (TimelineIterState *)state;
  // at the past-most item
  if (timeline_iter_state->node == NULL) {
    return false;
  }
  TimelineNode *orig = timeline_iter_state->node;
  do {
    timeline_iter_state->node = (TimelineNode *)timeline_iter_state->node->node.prev;
    if (timeline_iter_state->node == NULL) {
      timeline_iter_state->node = orig;
      return false;
    }
  } while (!prv_show_event(timeline_iter_state->node, timeline_iter_state->start_time,
      timeline_iter_state->midnight, timeline_iter_state->direction,
      timeline_iter_state->show_all_day_events) ||
      !timeline_exists(&timeline_iter_state->node->id));

  timeline_item_free_allocated_buffer(&timeline_iter_state->pin);
  timeline_iter_state->pin = (TimelineItem){};

  status_t rv = pin_db_get(&timeline_iter_state->node->id, &timeline_iter_state->pin);
  timeline_iter_state->current_day = time_util_get_midnight_of(
    timeline_iter_state->node->timestamp);
  timeline_iter_state->index = timeline_iter_state->node->index;
#ifdef TIMELINE_SERVICE_DEBUG
  prv_debug_print_pins(timeline_iter_state->node);
#endif
  return (rv == S_SUCCESS);
}

static void prv_prune_ordered_timeline_list(TimelineNode **head) {
  TimelineNode *node = *head;
  TimelineNode *next_node;
  while (node) {
    next_node = (TimelineNode *)list_get_next((ListNode *)node);
    time_t end_time = node->timestamp + (node->duration * SECONDS_PER_MINUTE);
    if (pin_db_has_entry_expired(end_time)) {
      // remove the pin without emitting an event
      pin_db_delete((uint8_t *)&node->id, sizeof(Uuid));

      // remove the node from out list
      timeline_iter_remove_node(head, node);
    } else {
      break; // the list is ordered so we are done
    }
    node = next_node;
  }
}

static void prv_put_outgoing_call_event(uint32_t call_identifier, const char *caller_id) {
  PebbleEvent event = {
    .type = PEBBLE_PHONE_EVENT,
    .phone = {
      .type = PhoneEventType_Outgoing,
      .source = PhoneCallSource_ANCS_Legacy,
      .call_identifier = call_identifier,
      .caller = phone_call_util_create_caller(caller_id, NULL),
    }
  };

  event_put(&event);
}

///////////////////////////////////////////////////
// Public functions
//////////////////////////////////////////////////

status_t timeline_init(TimelineNode **timeline) {
  PBL_LOG_DBG("Starting to build list.");
  status_t rv = pin_db_each(prv_each, timeline);
  prv_prune_ordered_timeline_list(timeline);
  prv_set_indices(*timeline);
  PBL_LOG_DBG("Finished building list.");
#ifdef TIMELINE_SERVICE_DEBUG
  prv_debug_print_pins(*timeline);
#endif
  return rv;
}

bool timeline_add(TimelineItem *item) {
  return (S_SUCCESS == pin_db_insert_item(item));
}

bool timeline_exists(Uuid *id) {
  return (pin_db_get_len((uint8_t *)id, UUID_SIZE) > 0);
}

bool timeline_remove(const Uuid *id) {
  // Use BlobDB directly in order to emit the BlobDB delete event
  return (S_SUCCESS == blob_db_delete(BlobDBIdPins, (uint8_t *)id, UUID_SIZE));
}

TimelineIterDirection timeline_direction_for_item(TimelineItem *item,
     TimelineNode *timeline, time_t now) {
  if (item->header.all_day) {
    time_t today_midnight = time_util_get_midnight_of(now);
    if (today_midnight > item->header.timestamp ||
        prv_should_show_all_day_events(timeline, now, today_midnight, TimelineIterDirectionPast)) {
      return TimelineIterDirectionPast;
    } else {
      return TimelineIterDirectionFuture;
    }
  } else if (item->header.timestamp < now) {
    return TimelineIterDirectionPast;
  } else {
    return TimelineIterDirectionFuture;
  }
}

bool timeline_nodes_equal(TimelineNode *a, TimelineNode *b) {
  if (a == NULL || b == NULL) {
    return (a == b);
  }
  return (uuid_equal(&a->id, &b->id) && (a->timestamp == b->timestamp));
}

bool timeline_get_originator_id(const TimelineItem *item, Uuid *uuid) {
  TimelineItem pin = {};
  const TimelineItem *pin_p;

  switch (item->header.type) {
    case TimelineItemTypeReminder:
      // Follow the parent id to get to the owner pin
      if (pin_db_get(&item->header.parent_id, &pin) != S_SUCCESS) {
        *uuid = UUID_INVALID;
        return false;
      }
      pin_p = &pin;
      break;
    case TimelineItemTypePin:
    case TimelineItemTypeNotification:
      // Some notifications have parent pins, some don't. If this one has a parent pin, follow it
      if (pin_db_get(&item->header.parent_id, &pin) == S_SUCCESS) {
        pin_p = &pin;
      } else {
        pin_p = item;
      }
      break;
    default:
      // Invalid item type
      *uuid = UUID_INVALID;
      return false;
  }

  *uuid = pin_p->header.parent_id;
  if (pin_p == &pin) {
    timeline_item_free_allocated_buffer(&pin);
  }
  return true;
}


//
// Iter functions
//

// return true if removed a node, false if non left
void timeline_iter_remove_node(TimelineNode **head, TimelineNode *node) {
  PBL_ASSERTN(node);
  prv_remove_node(head, node);
}

// return true if removed a node, false if non left
bool timeline_iter_remove_node_with_id(TimelineNode **head, Uuid *key) {
  // potentially more than one item with this UUID key since multiday events
  TimelineNode *node = prv_find_by_uuid(*head, key);
  if (node) {
    timeline_iter_remove_node(head, node);
    return true;
  } else {
    return false;
  }
}

status_t timeline_iter_init(Iterator *iter, TimelineIterState *iter_state, TimelineNode **head,
    TimelineIterDirection direction, time_t timestamp) {
  iter_state->direction = direction;
  iter_state->start_time = timestamp;
  iter_state->midnight = time_util_get_midnight_of(timestamp);
  iter_state->current_day = iter_state->midnight;
  iter_state->show_all_day_events = prv_should_show_all_day_events(*head, timestamp,
    iter_state->midnight, direction);
  TimelineNode *node = prv_find_first(*head, direction, timestamp, iter_state->midnight,
    iter_state->show_all_day_events);
  if (node == NULL) {
    iter_init(iter, prv_iter_dummy, prv_iter_dummy, iter_state);
    return S_NO_MORE_ITEMS;
  }

  status_t rv = pin_db_get(&node->id, &iter_state->pin);
  if (rv != S_SUCCESS) {
    iter_state->pin = (TimelineItem){};
    iter_init(iter, prv_iter_dummy, prv_iter_dummy, iter_state);
    return rv;
  }

  iter_state->node = node;
  iter_state->current_day = time_util_get_midnight_of(node->timestamp);
  iter_state->index = iter_state->node->index;
  if (direction == TimelineIterDirectionPast) {
    iter_init(iter, prv_iter_prev, prv_iter_next, iter_state);
  } else { // Future
    iter_init(iter, prv_iter_next, prv_iter_prev, iter_state);
  }

  return rv;
}

void timeline_iter_copy_state(TimelineIterState *dst_state, TimelineIterState *src_state,
                              Iterator *dst_iter, Iterator *src_iter) {
  PBL_ASSERTN(dst_state && src_state && dst_iter && src_iter);

  timeline_item_free_allocated_buffer(&dst_state->pin);

  *dst_state = *src_state;
  dst_state->pin = (TimelineItem){};

  *dst_iter = *src_iter;
  dst_iter->state = dst_state;
}

void timeline_iter_deinit(Iterator *iter, TimelineIterState *iter_state, TimelineNode **head) {
  TimelineNode *node = *head;
  while (node) {
    TimelineNode *old = node;
    node = (TimelineNode *)node->node.next;
    prv_remove_node(head, old);
  }
  *head = NULL;

  // free the currently allocated item in the iterator
  timeline_item_free_allocated_buffer(&iter_state->pin);
  iter_init(iter, prv_iter_dummy, prv_iter_dummy, iter_state);
}

void timeline_iter_refresh_pin(TimelineIterState *iter_state) {
  // no-op if the item doesn't exist
  if (timeline_exists(&iter_state->pin.header.id)) {
    timeline_item_free_allocated_buffer(&iter_state->pin);
    Uuid id = iter_state->pin.header.id;
    iter_state->pin = (TimelineItem){};
    pin_db_get(&id, &iter_state->pin);
  }
}

bool timeline_add_missed_call_pin(TimelineItem *pin, uint32_t uid) {
  uuid_generate(&pin->header.id);
  pin->header.layout = LayoutIdGeneric;
  pin->header.type = TimelineItemTypePin;
  pin->header.from_watch = true;
  pin->header.ancs_uid = uid;

  // patch the dismiss action to be a remove action
  TimelineItemAction *remove_action = timeline_item_find_dismiss_action(pin);

  // TODO: PBL-23915
  // We leak this i18n'd string because not leaking it is really hard.
  // We make sure we only ever allocate it once though, so it's not the end of the world.
  remove_action->attr_list.attributes[1].cstring = (char*)i18n_get("Remove", &i18n_key);
  remove_action->type = TimelineItemActionTypeRemove;

  return timeline_add(pin);
}

//
// Actions functions
//

static void prv_put_notification_action_result(const Uuid *id, const char *msg,
                                               uint32_t timeline_res_id, ActionResultType type) {
  // send action result event
  PebbleSysNotificationActionResult *action_result =
    kernel_malloc_check(sizeof(PebbleSysNotificationActionResult) + 2 * sizeof(Attribute));

  AttributeList result_attributes = {
    .num_attributes = 2,
    .attributes = (Attribute *)(((uint8_t *)action_result) + sizeof(*action_result)),
  };
  result_attributes.attributes[0].id = AttributeIdSubtitle;
  result_attributes.attributes[0].cstring = (char *)msg;
  result_attributes.attributes[1].id = AttributeIdIconLarge;
  result_attributes.attributes[1].uint32 = timeline_res_id;

  *action_result = (PebbleSysNotificationActionResult) {
    .id = *id,
    .type = type,
    .attr_list = result_attributes,
  };
  notifications_handle_notification_action_result(action_result);
}

static void prv_do_remote_action(const Uuid *id, TimelineItemActionType type,
                                 uint8_t action_id, const AttributeList *attributes,
                                 bool do_async) {
  if (comm_session_get_system_session()) {
    timeline_action_endpoint_invoke_action(id, type, action_id, attributes, do_async);
  } else {
    // We know we aren't connected, don't wait around for a response that won't come
    prv_put_notification_action_result(
        id, i18n_get("Can't connect. Relaunch Pebble Time app on phone.", &i18n_key),
        TIMELINE_RESOURCE_GENERIC_WARNING, ActionResultTypeFailure);
  }
}

static void prv_remove_pin_action(const TimelineItem *item,
                                  const TimelineItemAction *action,
                                  const AttributeList *attributes) {
  if (item->header.from_watch) {
    // remove it via BlobDB
    blob_db_delete(BlobDBIdPins, (uint8_t *)&item->header.id, UUID_SIZE);

    // TODO: PBL-23915
    // We leak this i18n'd string because not leaking it is really hard.
    // We make sure we only ever allocate it once though, so it's not the end of the world.
    prv_put_notification_action_result(&item->header.id, i18n_get("Removed", &i18n_key),
                                       TIMELINE_RESOURCE_RESULT_DELETED, ActionResultTypeSuccess);
  } else {
    const bool do_async = true;
    prv_do_remote_action(&item->header.id, action->type,
                         action->id, attributes, do_async);
  }
}

static void prv_dismiss_local_notification_action(const TimelineItem *item) {
  // TODO: PBL-23915
  // We leak this i18n'd string because not leaking it is really hard.
  // We make sure we only ever allocate it once though, so it's not the end of the world.
  prv_put_notification_action_result(&item->header.id, i18n_get("Dismissed", &i18n_key),
                                     TIMELINE_RESOURCE_RESULT_DISMISSED, ActionResultTypeSuccess);
}

static void prv_perform_ancs_negative_action(const TimelineItem *item,
                                             const TimelineItemAction *action) {
  uint8_t action_id = attribute_get_uint8(&action->attr_list, AttributeIdAncsAction,
                                          TIMELINE_INVALID_ACTION_ID);

  // Try to load the ancs id from attributes first in case the item's parent id points to another
  // timeline item. If the attribute isn't found, we assume the ancs id is stored in the header
  uint32_t ancs_uid = attribute_get_uint32(&action->attr_list, AttributeIdAncsId,
                                           item->header.ancs_uid);

  PBL_LOG_DBG("Perform ancs notification action (%"PRIu32", %"PRIu8")", ancs_uid,
          action_id);
  ancs_perform_action(ancs_uid, action_id);

  if (!timeline_is_bulk_ancs_action_mode_enabled()) {
    // TODO: PBL-23915
    // We leak this i18n'd string because not leaking it is really hard.
    // We make sure we only ever allocate it once though, so it's not the end of the world.
    char *msg_i18n = i18n_noop("Dismissed");
    int res_id = TIMELINE_RESOURCE_RESULT_DISMISSED;

    if (action->type == TimelineItemActionTypeAncsDelete) {
      msg_i18n = i18n_noop("Deleted");
      res_id = TIMELINE_RESOURCE_RESULT_DELETED;
    }
    prv_put_notification_action_result(&item->header.id, i18n_get(msg_i18n, &i18n_key), res_id,
                                       ActionResultTypeSuccess);
  } else {
    // Bulk mode suppresses the per-item dialog, but we must still remove the notification from the
    // UI ourselves; the phone only echoes a removal for notifications still in its center.
    notifications_handle_notification_removed((Uuid *)&item->header.id);
  }
}

static void prv_get_pin_and_push_pin_window(void *data) {
  Uuid *parent_id = data;
  TimelineItem *pin = task_malloc(sizeof(TimelineItem)); // cleaned-up by the modal
  if (pin && pin_db_get(parent_id, pin) == S_SUCCESS) {
    timeline_pin_window_push_modal(pin);
  } else {
    PBL_LOG_ERR("Failed to fetch parent pin");
  }
  kernel_free(parent_id);
}

static void prv_perform_health_response_action(const TimelineItem *item,
                                               const TimelineItemAction *action) {
  // TODO: PBL-23915
  // We leak this i18n'd string because not leaking it is really hard.
  // We make sure we only ever allocate it once though, so it's not the end of the world.
  const char *message = attribute_get_string(&action->attr_list, AttributeIdBody,
                                             (char *)i18n_get("Thanks!", &i18n_key));

  const uint32_t timeline_res_id = attribute_get_uint32(&action->attr_list, AttributeIdIconLarge,
                                                        TIMELINE_RESOURCE_THUMBS_UP);

  prv_put_notification_action_result(&item->header.id, message, timeline_res_id,
                                     ActionResultTypeSuccess);
}

void timeline_enable_ancs_bulk_action_mode(bool enable) {
  s_bulk_action_mode = enable;
}

bool timeline_is_bulk_ancs_action_mode_enabled(void) {
  return s_bulk_action_mode;
}

typedef struct OpenAppContext {
  EventServiceInfo event_info;
  AppInstallId install_id;
} OpenAppContext;

static void prv_app_render_ready(PebbleEvent *e, void *context) {
  OpenAppContext *ctx = context;

  if (ctx->install_id == app_manager_get_current_app_id()) {
    WindowStack *window_stack = modal_manager_get_window_stack(ModalPriorityNotification);
    window_stack_pop_all(window_stack, true /* animated */);
  }

  event_service_client_unsubscribe(&ctx->event_info);
  kernel_free(ctx);
}

void timeline_invoke_action(const TimelineItem *item, const TimelineItemAction *action,
                            const AttributeList *attributes) {
  char uuid_buffer[UUID_STRING_BUFFER_LENGTH];
  uuid_to_string(&item->header.parent_id, uuid_buffer);

  switch (action->type) {
    case TimelineItemActionTypeOpenWatchApp:
    {
      // find parent app
      AppInstallId install_id = app_install_get_id_for_uuid(&item->header.parent_id);
      if (install_id == INSTALL_ID_INVALID) {
        // This should never happen... but we're not quite there yet
        PBL_LOG_ERR("Could not find parent app %s for pin", uuid_buffer);
        return;
      }
      // fetch the relevant attribute
      const uint32_t launch_code =
          attribute_get_uint32(&action->attr_list, AttributeIdLaunchCode, 0);
      app_manager_put_launch_app_event(&(AppLaunchEventConfig) {
        .id = install_id,
        .common.args = (void *)(uintptr_t)launch_code,
        .common.reason = APP_LAUNCH_TIMELINE_ACTION,
      });
      PBL_LOG_INFO("Opening watch app %s", uuid_buffer);

      // Wait for the app we just launched to have something to render before hiding all modals.
      // If we don't we'll end up with flashing in a blank framebuffer.
      OpenAppContext *ctx = kernel_malloc_check(sizeof(OpenAppContext));
      *ctx = (OpenAppContext) {
        .event_info = {
          .type = PEBBLE_RENDER_READY_EVENT,
          .handler = prv_app_render_ready,
          .context = ctx,
        },
        .install_id = install_id,
      };
      event_service_client_subscribe(&ctx->event_info);
      break;
    }
    case TimelineItemActionTypeOpenPin:
    {
      Uuid *parent_id = kernel_malloc(sizeof(Uuid));
      if (parent_id) {
        *parent_id = item->header.parent_id;
        launcher_task_add_callback(prv_get_pin_and_push_pin_window, parent_id);
        PBL_LOG_INFO("Opening parent pin %s", uuid_buffer);
      }
      break;
    }
    case TimelineItemActionTypeAncsDial:
    {
      const char *caller_id = attribute_get_string(&item->attr_list,
                                                   AttributeIdTitle, "Unknown");
      prv_put_outgoing_call_event(item->header.ancs_uid, caller_id);
      notifications_handle_notification_action_result(NULL);
      ancs_perform_action(item->header.ancs_uid, ActionIDPositive);
      break;
    }
    // FIXME PBL-18673 this is not necessarily dismiss
    case TimelineItemActionTypeAncsPositive:
    case TimelineItemActionTypeAncsNegative:
    case TimelineItemActionTypeAncsDelete:
    {
      prv_perform_ancs_negative_action(item, action);
      notification_storage_set_status(&item->header.id, TimelineItemStatusDismissed);
      break;
    }
    case TimelineItemActionTypeDismiss:
      // This is a notification that was sourced from timeline. The mobile phone does not care
      // about dismissing it. We just confirm and dismiss locally.
      if (item->header.from_watch ||
          (((item->header.type == TimelineItemTypeNotification) ||
           (item->header.type == TimelineItemTypeReminder)) &&
           !timeline_get_private_data_source((Uuid *)&item->header.parent_id))) {
        prv_dismiss_local_notification_action(item);
        return;
      }

      // FALLTHROUGH
    case TimelineItemActionTypeGeneric:
    case TimelineItemActionTypeResponse:
    case TimelineItemActionTypeAncsResponse:
    case TimelineItemActionTypeAncsGeneric:
    case TimelineItemActionTypeHttp:
    case TimelineItemActionTypeComplete:
    case TimelineItemActionTypePostpone:
    case TimelineItemActionTypeRemoteRemove:
    {
      // remote action, send it to the phone
      const bool do_async = false;
      prv_do_remote_action(&item->header.id, action->type,
                           action->id, attributes, do_async);
      break;
    }
    case TimelineItemActionTypeRemove:
      prv_remove_pin_action(item, action, attributes);
      break;
    case TimelineItemActionTypeInsightResponse:
      prv_perform_health_response_action(item, action);
      break;
    default:
      PBL_LOG_ERR("Action type not implemented: %d", action->type);
      break;
  }
}

///////////////////////////////////
//! Timeline datasource functions
///////////////////////////////////

typedef struct {
  Uuid id;
  const char *name;
} PrivateDataSourceInfo;

static const PrivateDataSourceInfo s_data_sources[] = {
  {
    .id = UUID_NOTIFICATIONS_DATA_SOURCE,
    .name = i18n_noop("Notifications"),
  },
  {
    .id = UUID_CALENDAR_DATA_SOURCE,
    .name = i18n_noop("Calendar"),
  },
  {
    .id = UUID_WEATHER_DATA_SOURCE,
    .name = i18n_noop("Weather"),
  },
  {
    .id = UUID_REMINDERS_DATA_SOURCE,
    .name = i18n_noop("Reminders"),
  },
  {
    .id = UUID_ALARMS_DATA_SOURCE,
    .name = i18n_noop("Alarms"),
  },
  {
    .id = UUID_HEALTH_DATA_SOURCE,
    .name = i18n_noop("Health"),
  },
  {
    .id = UUID_INTERCOM_DATA_SOURCE,
    .name = i18n_noop("Intercom"),
  },
};

const char *timeline_get_private_data_source(Uuid *parent_id) {
  for (int i = 0; i < (int) ARRAY_LENGTH(s_data_sources); i++) {
    if (uuid_equal(parent_id, &s_data_sources[i].id)) {
      return s_data_sources[i].name;
    }
  }
  return NULL;
}