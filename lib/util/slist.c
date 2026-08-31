/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/util/slist.h"
#include "pbl/util/logging.h"

#include <stddef.h>
#include <stdio.h>

void slist_init(SingleListNode *node) {
  node->next = NULL;
}

SingleListNode* slist_insert_after(SingleListNode *node, SingleListNode *new_node) {
  if (node == NULL) {
    return new_node;
  }
  new_node->next = node->next;
  node->next = new_node;

  return new_node;
}

SingleListNode* slist_prepend(SingleListNode *head, SingleListNode *new_node) {
  if (new_node == NULL) {
    return head;
  }
  new_node->next = head;
  return new_node;
}

SingleListNode* slist_append(SingleListNode *head, SingleListNode *new_node) {
  return slist_insert_after(slist_get_tail(head), new_node);
}

SingleListNode* slist_pop_head(SingleListNode *head) {
  if (head == NULL) {
    return NULL;
  }
  SingleListNode *new_head = head->next;
  head->next = NULL;
  return new_head;
}

void slist_remove(SingleListNode *node, SingleListNode **head) {
  if (node == NULL || head == NULL || *head == NULL) {
    return;
  }

  if (*head == node) {
    *head = node->next;
    node->next = NULL;
    return;
  }

  SingleListNode *prev = *head;
  while (prev->next != NULL && prev->next != node) {
    prev = prev->next;
  }

  if (prev->next == node) {
    prev->next = node->next;
    node->next = NULL;
  }
}

SingleListNode* slist_get_next(SingleListNode *node) {
  if (node == NULL) {
    return NULL;
  }
  return node->next;
}

SingleListNode* slist_get_tail(SingleListNode *node) {
  if (node == NULL) {
    return NULL;
  }
  while (node->next != NULL) {
    node = node->next;
  }
  return node;
}

bool slist_is_tail(const SingleListNode *node) {
  if (!node) {
    return false;
  }
  return !node->next;
}

uint32_t slist_count(SingleListNode *head) {
  if (head == NULL) {
    return 0;
  }
  uint32_t count = 1;
  while ((head = head->next) != NULL) {
    ++count;
  }
  return count;
}

bool slist_contains(const SingleListNode *head, const SingleListNode *node) {
  if (head == NULL || node == NULL) {
    return false;
  }
  while (head) {
    if (head == node) {
      return true;
    }
    head = head->next;
  }
  return false;
}

SingleListNode* slist_find(SingleListNode *head, SingleListFilterCallback filter_callback,
                           void *data) {
  if (head == NULL) {
    return NULL;
  }
  SingleListNode *cursor = head;
  do {
    if (filter_callback(cursor, data)) {
      return cursor;
    }
  } while ((cursor = cursor->next));
  return NULL;
}

SingleListNode* slist_sorted_add(SingleListNode *head, SingleListNode *new_node,
                                 Comparator comparator, bool ascending) {
  if (head == NULL) {
    return new_node;
  }
  if (new_node == NULL) {
    return head;
  }

  SingleListNode *prev = NULL;
  SingleListNode *cursor = head;
  for (;;) {
    int order = comparator(cursor, new_node);
    if (!ascending) {
      order = -order;
    }

    if (order < 0) {
      // Insert before cursor
      new_node->next = cursor;
      if (prev == NULL) {
        return new_node;
      } else {
        prev->next = new_node;
        return head;
      }
    }
    if (cursor->next == NULL) {
      // Append after the last node
      cursor->next = new_node;
      new_node->next = NULL;
      return head;
    }
    prev = cursor;
    cursor = cursor->next;
  }
}

SingleListNode* slist_concatenate(SingleListNode *restrict list_a,
                                  SingleListNode *restrict list_b) {
  if (list_a == NULL) {
    return list_b;
  }
  if (list_b == NULL) {
    return list_a;
  }

  SingleListNode *tail_a = slist_get_tail(list_a);
  tail_a->next = list_b;

  return list_a;
}

void slist_foreach(SingleListNode *head, SingleListForEachCallback each_cb, void *context) {
  if (!each_cb) {
    return;
  }

  SingleListNode *iter = head;
  while (iter) {
    // Save off a pointer so the client can destroy the node in the callback
    SingleListNode *next = iter->next;
    if (!each_cb(iter, context)) {
      return;
    }
    iter = next;
  }
}

void slist_debug_dump(SingleListNode *head) {
  SingleListNode *iter = head;
  char buffer[20];
  while (iter) {
    snprintf(buffer, sizeof(buffer), "node %p (%p)", iter, iter->next);
    UTIL_LOG(buffer);
    iter = iter->next;
  }
}
