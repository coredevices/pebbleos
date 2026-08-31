/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/util/keyed_circular_cache.h"

#include "pbl/util/assert.h"

#include <string.h>

void keyed_circular_cache_init(KeyedCircularCache *c, KeyedCircularCacheKey *key_buffer,
                               void *data_buffer, size_t item_size, size_t total_items) {
  UTIL_ASSERT(c);
  UTIL_ASSERT(key_buffer);
  UTIL_ASSERT(data_buffer);
  UTIL_ASSERT(item_size);

  *c = (KeyedCircularCache) {
    .cache_keys = key_buffer,
    .cache_data = (uint8_t *)data_buffer,
    .item_size = item_size,
    .total_items = total_items,
  };
}

static uint8_t *prv_get_item_at_index(KeyedCircularCache *c, int index) {
  return c->cache_data + (index * c->item_size);
}

void *keyed_circular_cache_get(KeyedCircularCache *c, KeyedCircularCacheKey key) {
  // Optimize for accessing most recently pushed elements.
  int idx = c->next_item_to_erase_idx;
  for (unsigned int i = 0; i < c->total_items; i++) {
    idx--;
    if (idx < 0) {
      idx += c->total_items;
    }
    if (c->cache_keys[idx] == key) {
      return prv_get_item_at_index(c, idx);
    }
  }
  return NULL;
}

void keyed_circular_cache_push(KeyedCircularCache *c, KeyedCircularCacheKey key,
                               const void *new_item) {
  uint8_t *old_item = prv_get_item_at_index(c, c->next_item_to_erase_idx);
  memcpy(old_item, new_item, c->item_size);
  c->cache_keys[c->next_item_to_erase_idx] = key;

  c->next_item_to_erase_idx++;
  UTIL_ASSERT(c->next_item_to_erase_idx <= c->total_items);
  if (c->next_item_to_erase_idx == c->total_items) {
    c->next_item_to_erase_idx = 0;
  }
}
