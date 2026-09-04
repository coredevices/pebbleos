/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/timeline/item.h"
#include "pbl/services/timeline/attributes_actions.h"

#include "kernel/pbl_malloc.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

PBL_LOG_MODULE_DECLARE(service_timeline, CONFIG_SERVICE_TIMELINE_LOG_LEVEL);

static bool prv_is_valid_item(const TimelineItem *item) {
  return item && !uuid_is_invalid(&item->header.id);
}

static bool prv_item_init(TimelineItem *item, int num_attributes,
    int num_actions, uint8_t attributes_per_action[], size_t required_size_for_strings,
    uint8_t **string_buffer) {

  if (num_actions > 0) {
    PBL_ASSERTN(attributes_per_action != NULL);
  }

  const size_t alloc_size = attributes_actions_get_required_buffer_size(num_attributes, num_actions,
                                                                        attributes_per_action,
                                                                        required_size_for_strings);

  uint8_t *buffer = task_zalloc(alloc_size);
  if (buffer == NULL) {
    return false;
  }

  item->allocated_buffer = buffer;
  attributes_actions_init(&item->attr_list, &item->action_group, &buffer,
                          num_attributes, num_actions, attributes_per_action);

  if (string_buffer != NULL) {
    *string_buffer = buffer;
  }

  return true;
}

T_STATIC bool prv_deep_copy_attributes_actions(AttributeList *attr_list,
                                               TimelineItemActionGroup *action_group,
                                               TimelineItem *item_out) {
  // deep copy our attribute list / action group
  const size_t data_size = attributes_actions_get_buffer_size(attr_list, action_group);

  if (data_size) {
    item_out->allocated_buffer = task_malloc_check(data_size);
    uint8_t *buf_end = item_out->allocated_buffer + data_size;

    bool rv = attributes_actions_deep_copy(attr_list, &item_out->attr_list, action_group,
                                           &item_out->action_group, item_out->allocated_buffer,
                                           buf_end);
    if (!rv) {
      timeline_item_free_allocated_buffer(item_out);
      return false;
    }
  }

  return true;
}

bool timeline_item_create_from_serial_data(TimelineItem *item,
    uint8_t num_attributes, uint8_t num_actions, const uint8_t *data, size_t size,
    size_t *string_alloc_size, uint8_t **string_buffer) {

  PBL_ASSERTN(data != NULL);
  PBL_ASSERTN(string_alloc_size != NULL);

  // Determine string buffer allocation size based on serialized data
  uint8_t attributes_per_action[num_actions];
  bool r = attributes_actions_parse_serial_data(num_attributes, num_actions, data, size,
                                                string_alloc_size, attributes_per_action);
  if (!r) {
    return NULL;
  }

  if (!prv_item_init(item, num_attributes, num_actions, attributes_per_action,
      *string_alloc_size, string_buffer)) {
    return false;
  }

  return true;
}

TimelineItem *timeline_item_create_with_attributes(time_t timestamp, uint16_t duration,
                                                   TimelineItemType type, LayoutId layout,
                                                   AttributeList *attr_list,
                                                   TimelineItemActionGroup *action_group) {
  TimelineItem *item = task_zalloc_check(sizeof(TimelineItem));

  uuid_generate(&item->header.id);
  item->header.type = type;
  item->header.duration = duration;
  item->header.timestamp = timestamp;
  item->header.layout = layout;

  if (!prv_deep_copy_attributes_actions(attr_list, action_group, item)) {
    timeline_item_destroy(item);
    return NULL;
  }

  return item;
}

TimelineItem *timeline_item_create(int num_attributes, int num_actions,
    uint8_t attributes_per_action[], size_t required_size_for_strings, uint8_t **string_buffer) {

  TimelineItem *item = task_zalloc(sizeof(TimelineItem));
  if (item == NULL) {
    return NULL;
  }

  if (!prv_item_init(item, num_attributes, num_actions, attributes_per_action,
      required_size_for_strings, string_buffer)) {
    task_free(item);
    return NULL;
  }

  return item;
}

TimelineItem *timeline_item_copy(TimelineItem *src) {
  if (!src) {
    return NULL;
  }

  TimelineItem *item_out = task_zalloc(sizeof(TimelineItem));
  if (!item_out) {
    return NULL;
  }
  memcpy(&item_out->header, &src->header, sizeof(CommonTimelineItemHeader));

  if (!prv_deep_copy_attributes_actions(&src->attr_list, &src->action_group, item_out)) {
    timeline_item_destroy(item_out);
    return NULL;
  }

  return item_out;
}

size_t timeline_item_get_serialized_payload_size(TimelineItem *item) {
  PBL_ASSERTN(item);
  return attributes_actions_get_serialized_payload_size(&item->attr_list, &item->action_group);
}

bool timeline_item_deserialize_item(TimelineItem *item_out,
                                    const SerializedTimelineItemHeader* header,
                                    const uint8_t *payload) {
  // If the creation / deserialization fails we need to clean up, and if the item contains garbage
  // data we will try to free a garbage allocated buffer field and crash.
  memset(item_out, 0, sizeof(TimelineItem));

  size_t string_alloc_size;
  char *buffer;
  if (!timeline_item_create_from_serial_data(item_out,
                                             header->num_attributes,
                                             header->num_actions,
                                             payload,
                                             header->payload_length,
                                             &string_alloc_size,
                                             (uint8_t **) &buffer)) {
    PBL_LOG_ERR("Failed to get timeline item");
    goto cleanup;
  }

  timeline_item_deserialize_header(item_out, header);

  if (!timeline_item_deserialize_payload(item_out,
                                         buffer,
                                         string_alloc_size,
                                         payload,
                                         header->payload_length)) {
    PBL_LOG_ERR("Failed to deserialize payload");
    goto cleanup;
  }

  return true;

cleanup:
  timeline_item_free_allocated_buffer(item_out);
  return false;
}

void timeline_item_serialize_header(TimelineItem *item,
    SerializedTimelineItemHeader *header) {

  PBL_ASSERTN(item != NULL);
  PBL_ASSERTN(header != NULL);

  size_t payload_length = timeline_item_get_serialized_payload_size(item);

  header->common = item->header;
  header->payload_length = payload_length;
  header->num_attributes = item->attr_list.num_attributes;
  header->num_actions = item->action_group.num_actions;
}

void timeline_item_deserialize_header(TimelineItem *item,
                                      const SerializedTimelineItemHeader *header) {

  PBL_ASSERTN(item != NULL);
  PBL_ASSERTN(header != NULL);

  item->header = header->common;
  item->attr_list.num_attributes = header->num_attributes;
  item->action_group.num_actions = header->num_actions;

  item->header.timestamp = timeline_item_get_tz_timestamp(&item->header);
}

time_t timeline_item_get_tz_timestamp(CommonTimelineItemHeader *hdr) {
  bool should_adjust = hdr->all_day || hdr->is_floating;
  time_t timestamp = hdr->timestamp;
  if (should_adjust) {
    timestamp = time_local_to_utc(timestamp);
  }
  return timestamp;
}

size_t timeline_item_serialize_payload(TimelineItem *item, uint8_t *buffer,
    size_t buffer_size) {

  PBL_ASSERTN(item != NULL);

  return attributes_actions_serialize_payload(&item->attr_list, &item->action_group,
                                              buffer, buffer_size);
}

bool timeline_item_deserialize_payload(TimelineItem *item, char *string_buffer,
                                       size_t string_buffer_size, const uint8_t *payload,
                                       size_t payload_size) {
  PBL_ASSERTN(item != NULL);
  PBL_ASSERTN(string_buffer != NULL);
  PBL_ASSERTN(payload != NULL);

  uint8_t *buf_end = (uint8_t *)string_buffer + string_buffer_size;

  return attributes_actions_deserialize(&item->attr_list, &item->action_group,
                                        (uint8_t*) string_buffer, buf_end, payload, payload_size);
}

void timeline_item_destroy(TimelineItem* item) {
  if (item != NULL) {
    timeline_item_free_allocated_buffer(item);
    task_free(item);
  }
}

void timeline_item_free_allocated_buffer(TimelineItem *item) {
  if (item->allocated_buffer != NULL) {
    task_free(item->allocated_buffer);
    item->allocated_buffer = NULL;
  }
}

bool timeline_item_verify_layout_serialized(const uint8_t *val, int val_len) {
  SerializedTimelineItemHeader *hdr = (SerializedTimelineItemHeader *)val;
  bool has_attribute[NumAttributeIds] = {0};

  // verify that the serialized attributes are well-formed
  const uint8_t *cursor = val + sizeof(SerializedTimelineItemHeader);
  const uint8_t *val_end = val + val_len;
  if (!attribute_check_serialized_list(cursor, val_end, hdr->num_attributes, has_attribute)) {
    PBL_LOG_ERR("Could not deserialize attributes to verify");
    return false;
  }
  // verify that the layout of the item has the attribute it requires
  LayoutId layout = hdr->common.layout;
  PBL_LOG_DBG("Number of attributes: %d for layout: %d", hdr->num_attributes, layout);
  return layout_verify(has_attribute, layout);
}

bool timeline_item_action_is_dismiss(const TimelineItemAction *action) {
    return (action->type == TimelineItemActionTypeAncsNegative ||
        action->type == TimelineItemActionTypeDismiss);
}

bool timeline_item_action_is_ancs(const TimelineItemAction *action) {
  return action->type == TimelineItemActionTypeAncsNegative ||
         action->type == TimelineItemActionTypeAncsDelete ||
         action->type == TimelineItemActionTypeAncsDial ||
         action->type == TimelineItemActionTypeAncsPositive;
}

bool timeline_item_is_ancs_notif(const TimelineItem *item) {
  return item->header.ancs_notif;
}

// ------------------------------------------------------------------------------------------------
// Action finding functions
typedef bool (*ActionCompareFunc)(const TimelineItemAction *action, void* data);

static TimelineItemAction *prv_find_action(const TimelineItemActionGroup *action_group,
                                           ActionCompareFunc compare_func,
                                           void *data) {
  for (int i = 0; i < action_group->num_actions; i++) {
    TimelineItemAction *action = &action_group->actions[i];
    if (compare_func(action, data)) {
      return action;
    }
  }

  return NULL;
}

static TimelineItemAction *prv_item_find_action(const TimelineItem *item,
                                                ActionCompareFunc compare_func,
                                                void *data) {
  if (!prv_is_valid_item(item)) {
    return NULL;
  }

  return prv_find_action(&item->action_group, compare_func, data);
}

static bool prv_action_id_compare_func(const TimelineItemAction *action, void* data) {
  uint8_t *action_id = data;
  return (action->id == *action_id);
}

const TimelineItemAction *timeline_item_find_action_with_id(const TimelineItem *item,
                                                            uint8_t action_id) {
  return prv_item_find_action(item, prv_action_id_compare_func, &action_id);
}

static bool prv_action_type_compare_func(const TimelineItemAction *action, void* data) {
  TimelineItemActionType *type = data;
  return (action->type == *type);
}

TimelineItemAction *timeline_item_find_action_by_type(const TimelineItem *item,
                                                      TimelineItemActionType type) {
  return prv_item_find_action(item, prv_action_type_compare_func, &type);
}

static bool prv_action_dismiss_compare_func(const TimelineItemAction *action, void* data) {
  return timeline_item_action_is_dismiss(action);
}

TimelineItemAction *timeline_item_find_dismiss_action(const TimelineItem *item) {
  return prv_item_find_action(item, prv_action_dismiss_compare_func, NULL);
}

static bool prv_action_reply_compare_func(const TimelineItemAction *action, void* data) {
  return (action->type == TimelineItemActionTypeAncsResponse) ||
      (action->type == TimelineItemActionTypeResponse);
}

TimelineItemAction *timeline_item_find_reply_action(const TimelineItem *item) {
  return prv_item_find_action(item, prv_action_reply_compare_func, NULL);
}

TimelineItemAction *timeline_item_action_group_find_reply_action(
    const TimelineItemActionGroup *action_group) {
  return prv_find_action(action_group, prv_action_reply_compare_func, NULL);
}
