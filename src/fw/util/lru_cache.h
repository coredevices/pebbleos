/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stddef.h>
#include <stdint.h>

//! This is a pretty simple & lean LRU cache
//! It works with a pre-allocated buffer in which it stores a singly linked list
//! of the items in LRU order (head of the list is the LRU item)
//! put and get are both O[N]
//! Note: we could save 2 bytes per entry by using array indices rather than pointer
//! but this would complicate the code quite a bit

typedef struct CacheEntry {
  struct CacheEntry *more_recent; ///< The next entry in the linked list
  uint32_t key; ///< The key that identifies this entry
  uint8_t data[]; ///< the data associated with this entry
} CacheEntry;

typedef struct {
  uint8_t *buffer; ///< A pointer to the buffer allocated for storing cache data
  size_t item_size; ///< The size in bytes of items in the cache
  int max_items; ///< The max number of items that can fit in the cache
  CacheEntry *least_recent; ///< The head of the singly linked list of cache entries
} LRUCache;

//! Initialize an LRU cache
//! @param c a reference to a cache struct
//! @param item_size the size in bytes of items to be stored in the cache
//! @param buffer a buffer to store cache items into
//! @param buffer_size the size of the buffer
//! @note each entry is 8 bytes larger than item_size, allocate accordingly!
void lru_cache_init(LRUCache* c, size_t item_size, uint8_t *buffer, size_t buffer_size);

//! Retrieve an item from the cache and mark the item as most recently used
//! @param c the cache to retrieve from
//! @param key the key for the item we want retrieved
//! @return a pointer to the item, or NULL if not found
void *lru_cache_get(LRUCache* c, uint32_t key);

//! Add an item to the cache.
//! @note This will evict the least recently used item if the cache is full
//! @note If the key already exist, the old item is overridden
//! @param c the cache to retrieve data from
//! @param key the key to associate to the item
//! @param item a pointer to the item data
void lru_cache_put(LRUCache* c, uint32_t key, void* item);

//! Flush the cache. This removes all data from the cache.
//! @param c the cache to flush
void lru_cache_flush(LRUCache *c);

