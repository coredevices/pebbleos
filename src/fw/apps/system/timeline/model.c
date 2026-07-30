/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "model.h"

#include <pbl/logging/logging.h>
#include "system/passert.h"

////////////////////////////////////////////////
// Timeline model circular array of iters logic
////////////////////////////////////////////////

static TimelineModel *s_model_data;

static int prv_get_idx_for_timeline_idx(int timeline_idx) {
  for (int i = 0; i < TIMELINE_NUM_ITEMS_IN_MODEL; i++) {
    if (s_model_data->states[i].index == timeline_idx) {
      return i;
    }
  }
  return -1;
}

static int prv_raw_to_adj_idx(int raw_idx) {
  return positive_modulo(raw_idx - s_model_data->first_index, TIMELINE_NUM_ITEMS_IN_MODEL);
}

static int prv_adj_to_raw_idx(int adj_idx) {
  return positive_modulo(adj_idx + s_model_data->first_index, TIMELINE_NUM_ITEMS_IN_MODEL);
}

TimelineIterState *timeline_model_get_iter_state_with_timeline_idx(int index) {
  int raw_idx = prv_get_idx_for_timeline_idx(index);
  if (raw_idx == -1) {
    return NULL;
  } else {
    return &s_model_data->states[raw_idx];
  }
}

int timeline_model_get_idx_for_timeline_idx(int index) {
  int raw_idx = prv_get_idx_for_timeline_idx(index);
  if (raw_idx == -1) {
    return -1;
  } else {
    return prv_raw_to_adj_idx(prv_get_idx_for_timeline_idx(index));
  }
}

TimelineIterState *timeline_model_get_iter_state(int index) {
  return &s_model_data->states[prv_adj_to_raw_idx(index)];
}

bool timeline_model_is_empty(void) {
  return (timeline_model_get_num_items() == 0);
}

int timeline_model_get_num_items(void) {
  if (timeline_model_get_current_state() == NULL) {
    return 0;
  }
  int num = positive_modulo(s_model_data->last_index -
      s_model_data->first_index, TIMELINE_NUM_ITEMS_IN_MODEL) + 1;
  // we always keep one slot marked empty so we can tell if we have zero items
  if (num == TIMELINE_NUM_ITEMS_IN_MODEL) {
    return 0;
  } else {
    return num;
  }
}

static int prv_get_next_item_idx(void) {
  return positive_modulo(s_model_data->last_index + 1,
      TIMELINE_NUM_ITEMS_IN_MODEL);
}

static int prv_get_prev_item_idx(void) {
  return positive_modulo(s_model_data->first_index - 1,
      TIMELINE_NUM_ITEMS_IN_MODEL);
}

bool timeline_model_prepare_prev(void) {
  if (timeline_model_get_current_state() == NULL) {
    return false;
  }
  const int prev_idx = prv_get_prev_item_idx();
  const int first_idx = s_model_data->first_index;
  timeline_iter_copy_state(
      &s_model_data->states[prev_idx],
      &s_model_data->states[first_idx],
      &s_model_data->iters[prev_idx],
      &s_model_data->iters[first_idx]);
  if (!iter_prev(&s_model_data->iters[prev_idx])) {
    s_model_data->states[prev_idx].node = NULL;
    return false;
  }
  return true;
}

static Iterator *prv_get_iter(int index) {
  return &s_model_data->iters[prv_adj_to_raw_idx(index)];
}

TimelineIterState *timeline_model_get_current_state(void) {
  if (timeline_model_get_iter_state(0)->node == NULL) {
    return NULL;
  }
  return timeline_model_get_iter_state(0);
}

static int prv_find_item_by_uuid(Uuid *id) {
  for (int i = 0; i < TIMELINE_NUM_VISIBLE_ITEMS; i++) {
    if (timeline_model_get_iter_state(i)->node &&
        uuid_equal(&timeline_model_get_iter_state(i)->pin.header.id, id)) {
      return i;
    }
  }
  return -1;
}

#ifdef TIMELINE_DEBUG
static void prv_log_all_items(void) {
  PBL_LOG_DBG("First item: %d, last item: %d", s_model_data->first_index,
    s_model_data->last_index);
  for (int i = 0; i < TIMELINE_NUM_VISIBLE_ITEMS; i++) {
    TimelineItem *item = &timeline_model_get_iter_state(i)->pin;
    PBL_LOG_DBG("ID first byte: 0x%x", item->header.id.byte0);
    PBL_LOG_DBG("Address of node: %p", timeline_model_get_iter_state(i)->node);
    PBL_LOG_DBG("Timestamp: %ld", item->header.timestamp);
  }
}
#endif

static void prv_move_first_index(int delta) {
  s_model_data->first_index = positive_modulo(
      s_model_data->first_index + delta, TIMELINE_NUM_ITEMS_IN_MODEL);
  PBL_LOG_DBG("Set origin, initial item: %d, final item: %d",
      s_model_data->first_index, s_model_data->last_index);
}

bool timeline_model_iter_next(int *new_idx, bool *has_next) {
  int next_idx = prv_get_next_item_idx();
  int last_idx = s_model_data->last_index;
  timeline_iter_copy_state(
      &s_model_data->states[next_idx],
      &s_model_data->states[last_idx],
      &s_model_data->iters[next_idx],
      &s_model_data->iters[last_idx]);
  bool rv = iter_next(&s_model_data->iters[next_idx]);
  if (rv) {
    if (has_next) {
      *has_next = true;
    }
    s_model_data->last_index = next_idx;
    prv_move_first_index(1);
    if (new_idx) {
      *new_idx = s_model_data->states[next_idx].index;
    }
  } else {
    if (has_next) {
      *has_next = false;
    }
    rv = timeline_model_get_num_items() > 1;
    if (rv) {
      prv_move_first_index(1);
    }
  }
#ifdef TIMELINE_DEBUG
  prv_log_all_items();
#endif
  return rv;
}

bool timeline_model_iter_prev(int *new_idx, bool *has_prev) {
  int prev_idx = prv_get_prev_item_idx();
  int first_idx = s_model_data->first_index;
  timeline_iter_copy_state(
      &s_model_data->states[prev_idx],
      &s_model_data->states[first_idx],
      &s_model_data->iters[prev_idx],
      &s_model_data->iters[first_idx]);
  bool rv = iter_prev(&s_model_data->iters[prev_idx]);
  if (rv) {
    if (has_prev) {
      *has_prev = true;
    }
    // bring the last_index back if we've succeeded iterating prev-wards and there are at least
    // TIMELINE_NUM_VISIBLE_ITEMS items in the model. If there are fewer, we keep the last_index
    // where it is so the model can "grow" to contain TIMELINE_NUM_VISIBLE_ITEMS
    if (timeline_model_get_num_items() >= TIMELINE_NUM_VISIBLE_ITEMS) {
      s_model_data->last_index = positive_modulo(
          s_model_data->last_index - 1, TIMELINE_NUM_ITEMS_IN_MODEL);
    }
    if (new_idx) {
      *new_idx = s_model_data->states[prev_idx].index;
    }
    prv_move_first_index(-1);
  } else {
    if (has_prev) {
      *has_prev = false;
    }
  }
#ifdef TIMELINE_DEBUG
    prv_log_all_items();
#endif
  return rv;
}

// Initialize the TIMELINE_NUM_VISIBLE_ITEMS iterators and states
// Try to move the iterators except iters[0] next-wards the appropriate number of times
void timeline_model_init(time_t timestamp, TimelineModel *model) {
  PBL_ASSERTN(model);
  s_model_data = model;

  // build the timeline
  status_t rv = timeline_init(&s_model_data->timeline);
  PBL_ASSERTN(PASSED(rv));

  s_model_data->first_index = 0;
  s_model_data->last_index = TIMELINE_NUM_VISIBLE_ITEMS;
  for (int i = 0; i < TIMELINE_NUM_VISIBLE_ITEMS; i++) {
    rv = timeline_iter_init(prv_get_iter(i), timeline_model_get_iter_state(i),
      &s_model_data->timeline, s_model_data->direction, timestamp);
    if (FAILED(rv)) {
      PBL_LOG_ERR("Timeline iterator failed to init!");
    }
    if (FAILED(rv) || rv == S_NO_MORE_ITEMS) {
      timeline_model_get_iter_state(i)->node = NULL;
    }
    bool iter_at_final_position = true;
    for (int num_to_iter = 0; num_to_iter < i; num_to_iter++) {
      iter_at_final_position = iter_at_final_position && iter_next(prv_get_iter(i));
    }
    if (iter_at_final_position) {
      s_model_data->last_index = prv_get_next_item_idx();
    }
  }
#ifdef TIMELINE_DEBUG
  prv_log_all_items();
#endif
}

void timeline_model_deinit(void) {
  for (int i = 0; i < TIMELINE_NUM_ITEMS_IN_MODEL; i++) {
    timeline_iter_deinit(&s_model_data->iters[i],
        &s_model_data->states[i], &s_model_data->timeline);
    s_model_data->states[i].node = NULL;
  }
  s_model_data->first_index = 0;
  s_model_data->last_index = TIMELINE_NUM_VISIBLE_ITEMS;
}

static void prv_remove_index_gracefully(int idx) {
  PBL_ASSERTN(idx >= 0);
  TimelineNode *node = timeline_model_get_iter_state(idx)->node;
  if (iter_next(prv_get_iter(idx))) {
    for (int i = idx + 1; i < TIMELINE_NUM_VISIBLE_ITEMS; i++) {
      // it's possible for an iter to be on a node that is no longer valid, which could leave
      // multiple iterators starting off at different nodes but ending up on the same one
      // after one iter_next, so try to separate them
      do {
        if (!iter_next(prv_get_iter(i))) {
          break;
        }
      } while (timeline_nodes_equal(timeline_model_get_iter_state(i)->node,
            timeline_model_get_iter_state(i - 1)->node));
    }
    timeline_iter_remove_node(&s_model_data->timeline, node);
    PBL_LOG_DBG("Item to delete in view, iterating next");
  } else if (iter_prev(prv_get_iter(idx))) {
    // prv_get_iter(idx) is at the end, so we have to move prev-wards
    // if prv_get_iter(idx) is at the end, all iters > idx must also be at the end
    // so iterate those prev-wards
    for (int i = idx + 1; i < TIMELINE_NUM_VISIBLE_ITEMS; i++) {
      iter_prev(prv_get_iter(i));
    }
    timeline_iter_remove_node(&s_model_data->timeline, node);
    PBL_LOG_DBG("Item to delete in view, iterating prev");
  } else {
    // if we can't iterate next or prev, we've deleted the only item
    timeline_model_deinit();
    PBL_LOG_DBG("Item to delete in view, deiniting ");
  }
}

void timeline_model_remove(Uuid *id) {
  int item_idx;
  // more than one item with the same ID is possible due to multi-day events
  // remove them from our list first
  while ((item_idx = prv_find_item_by_uuid(id)) != -1) {
    prv_remove_index_gracefully(item_idx);
  }

  // remove the rest from the iterator list
  while (timeline_iter_remove_node_with_id(&s_model_data->timeline, id)) {}
}
