/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "common.h"

#include "kernel/events.h"

#define TIMELINE_NUM_ITEMS_IN_MODEL (TIMELINE_NUM_VISIBLE_ITEMS + 1)

// Timeline is a circular array of TIMELINE_NUM_ITEMS_IN_MODEL iter states

typedef struct {
  TimelineNode *timeline;
  TimelineDirection direction;
  Iterator iters[TIMELINE_NUM_ITEMS_IN_MODEL];
  TimelineIterState states[TIMELINE_NUM_ITEMS_IN_MODEL];
  int first_index;
  int last_index;
} TimelineModel;

//! Get the iterator state with the timeline location index, i.e. the iterator with the given index
//! if it's within the model
//! These indices do not change when an iteration has occurred, i.e.
//! timeline_model_get_iter_state_timeline_idx(2) is the same before and after an iter_next or
//! iter_prev
//! @return NULL if the item with that index is not contained in the model's window of items
TimelineIterState *timeline_model_get_iter_state_with_timeline_idx(int index);

//! Get the index with respect to the model of the timeline item with the given timeline index
//! @return -1 if the item with that index is not contained in the model's window of items
int timeline_model_get_idx_for_timeline_idx(int index);

//! Get the iterator state with the index with respect to the model, i.e. the index that represents
//! which item in the model it is
//! These indices do change when an iteration has occurred, i.e.
//! timeline_model_get_iter_state_idx(0) changes after an iter_next or iter_prev
TimelineIterState *timeline_model_get_iter_state(int index);

TimelineIterState *timeline_model_get_current_state(void);

bool timeline_model_is_empty(void);

int timeline_model_get_num_items(void);

//! Load the item before the current one into the retained slot so it can be read with
//! timeline_model_get_iter_state(-1)
//! Only call while no layout references the retained state: preparing frees its pin buffer
//! @return whether an item before the current one exists
bool timeline_model_prepare_prev(void);

//! Iterate the model towards the "next" direction
//! @param new_idx Set the raw index of the new iterator if new_idx is not NULL
//! @param has_next set to whether or not there is a new third item in the list, i.e. false =
//! iterate but show no more new items
//! @return whether or not the current item is at the end of the list, i.e. false = stop iterating
bool timeline_model_iter_next(int *new_idx, bool *has_next);

//! Iterate the model towards the "prev" direction
//! @param new_idx Set the raw index of the new iterator if new_idx is not NULL
//! @param has_prev set to whether or not there is a new item in the list, i.e. false =
//! iterate but show no more new items
//! @return whether or not the current item is at the beginning of the list, i.e. stop iterating
bool timeline_model_iter_prev(int *new_idx, bool *has_prev);

void timeline_model_init(time_t timestamp, TimelineModel *model_data);

void timeline_model_deinit(void);

void timeline_model_remove(Uuid *id);
