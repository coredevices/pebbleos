/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "order.h"

typedef struct SingleListNode {
  struct SingleListNode *next;
} SingleListNode;

typedef bool (*SingleListFilterCallback)(SingleListNode *found_node, void *data);

//! - If a callback returns true, the iteration continues
//! - If a callback returns false, the iteration stops.
typedef bool (*SingleListForEachCallback)(SingleListNode *node, void *context);

#define SINGLE_LIST_NODE_NULL { .next = NULL }

//! Initializes the node.
void slist_init(SingleListNode *node);

//! Inserts new_node after node in the list.
//! Always returns new_node.
SingleListNode* slist_insert_after(SingleListNode *node, SingleListNode *new_node);

//! Prepends new_node to the head of the list.
//! @param head The current head of the list, can be NULL.
//! @param new_node The node to prepend.
//! Always returns the new head of the list.
SingleListNode* slist_prepend(SingleListNode *head, SingleListNode *new_node);

//! Appends new_node to the tail of the list that head is part of.
//! @param head Any node in the list, can be NULL (will result in a list containing only new_node).
//! @param new_node The node to append.
//! Always returns the tail of the list.
SingleListNode* slist_append(SingleListNode *head, SingleListNode *new_node);

//! Removes the head of the list and returns the new head.
SingleListNode* slist_pop_head(SingleListNode *head);

//! Removes the node from the list.
//! @param node the SingleListNode to remove.
//! @param[in,out] *head will be updated if the removed node happens to be the head.
//! @note head must not be NULL.
void slist_remove(SingleListNode *node, SingleListNode **head);

//! Gets the next node.
SingleListNode* slist_get_next(SingleListNode *node);

//! Gets the last node in the list.
SingleListNode* slist_get_tail(SingleListNode *node);

//! @return true if the passed in node is the tail of a list.
bool slist_is_tail(const SingleListNode *node);

//! Counts the number of nodes from head to tail.
uint32_t slist_count(SingleListNode *head);

//! @param[in] head The head of the list to search.
//! @param[in] node The node to search for.
//! @returns True if the list contains node.
bool slist_contains(const SingleListNode *head, const SingleListNode *node);

//! Gets the first node that conforms to the given filter callback.
//! @param head The list node from which to start the search.
//! @param filter_callback A function returning true if the node matches the filter criteria.
//! @param data Optional callback data.
SingleListNode* slist_find(SingleListNode *head, SingleListFilterCallback filter_callback,
                           void *data);

//! Adds a node to a list ordered by given comparator.
//! @param[in] head The head of the list that we want to add to.
//! @param[in] new_node The node being added.
//! @param[in] comparator The comparison function to use.
//! @param[in] ascending True to maintain the list ordered ascending from head to tail.
//! @returns The (new) head of the list.
//! @note This function will not sort existing nodes in the list.
SingleListNode* slist_sorted_add(SingleListNode *head, SingleListNode *new_node,
                                 Comparator comparator, bool ascending);

//! Concatenate two lists.
//! @param list_a list onto which to concatenate list_b.
//! @param list_b list to concatenate onto list_a.
//! @return head of the new list.
SingleListNode* slist_concatenate(SingleListNode *list_a, SingleListNode *list_b);

//! Iterates over each node and passes it into callback given.
//! @param[in] head The head of the list that we want to iterate over.
//! @param[in] each_cb The callback function to pass each node into.
//! @param[in] context Optional callback data.
void slist_foreach(SingleListNode *head, SingleListForEachCallback each_cb, void *context);

//! Dump a list to PBL_LOG.
void slist_debug_dump(SingleListNode *head);
