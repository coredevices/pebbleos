/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "dict.h"

#include <string.h>

#include "system/passert.h"
#include "kernel/pbl_malloc.h"

static DictionaryResult dict_init(DictionaryIterator *iter, const uint8_t * const buffer, const uint16_t length) {
  if (iter == NULL ||
      buffer == NULL) {
    return DICT_INVALID_ARGS;
  }
  if (length < sizeof(Dictionary)) {
    return DICT_NOT_ENOUGH_STORAGE;
  }
  iter->dictionary = (Dictionary *) buffer;
  iter->cursor = iter->dictionary->head;
  iter->end = buffer + length;
  return DICT_OK;
}

uint32_t dict_size(DictionaryIterator* iter) {
  return (uint8_t*)iter->end - (uint8_t*)iter->dictionary;
}

DictionaryResult dict_write_begin(DictionaryIterator *iter, uint8_t * const buffer, const uint16_t length) {
  const DictionaryResult result = dict_init(iter, buffer, length);
  if (result == DICT_OK) {
    iter->dictionary->count = 0;
  }
  return result;
}

static Tuple * cursor_after_tuple_with_data_length(const DictionaryIterator *iter, const uint16_t length) {
  return (Tuple *) (((uint8_t *)iter->cursor) + sizeof(Tuple) + length);
}

static DictionaryResult dict_write_data_internal(DictionaryIterator *iter, const uint32_t key, const uint8_t * const data, const uint16_t data_length, const TupleType type) {
  if (iter == NULL ||
      iter->dictionary == NULL ||
      iter->cursor == NULL) {
    return DICT_INVALID_ARGS;
  }
  if (iter->cursor == iter->dictionary->head) {
    // Reset implicitly if the cursor is at the head, so writing again after
    // calling dict_write_end() won't screw up the count and will just work:
    iter->dictionary->count = 0;
  }
  Tuple * const next_cursor = cursor_after_tuple_with_data_length(iter, data_length);
  if (iter->end < (void *)next_cursor) {
    return DICT_NOT_ENOUGH_STORAGE;
  }
  iter->cursor->key = key;
  iter->cursor->length = data_length;
  iter->cursor->type = type;
  if (data_length > 0) {
    if (data == NULL) {
      return DICT_INVALID_ARGS;
    }
    memcpy(iter->cursor->value->data, data, data_length);
  }
  iter->cursor = next_cursor;
  ++iter->dictionary->count;
  return DICT_OK;
}

DictionaryResult dict_write_data(DictionaryIterator *iter, const uint32_t key, const uint8_t * const data, const uint16_t length) {
  return dict_write_data_internal(iter, key, data, length, TUPLE_BYTE_ARRAY);
}

DictionaryResult dict_write_cstring(DictionaryIterator *iter, const uint32_t key, const char * const cstring) {
  return dict_write_data_internal(iter, key, (const uint8_t * const) cstring, cstring ? strlen(cstring) + 1 : 0, TUPLE_CSTRING);
}

DictionaryResult dict_write_uint8(DictionaryIterator *iter, const uint32_t key, const uint8_t value) {
  return dict_write_data_internal(iter, key, (const uint8_t * const) &value, sizeof(value), TUPLE_UINT);
}

DictionaryResult dict_write_uint16(DictionaryIterator *iter, const uint32_t key, const uint16_t value) {
  return dict_write_data_internal(iter, key, (const uint8_t * const) &value, sizeof(value), TUPLE_UINT);
}

DictionaryResult dict_write_uint32(DictionaryIterator *iter, const uint32_t key, const uint32_t value) {
  return dict_write_data_internal(iter, key, (const uint8_t * const) &value, sizeof(value), TUPLE_UINT);
}

DictionaryResult dict_write_int8(DictionaryIterator *iter, const uint32_t key, const int8_t value) {
  return dict_write_data_internal(iter, key, (const uint8_t * const) &value, sizeof(value), TUPLE_INT);
}

DictionaryResult dict_write_int16(DictionaryIterator *iter, const uint32_t key, const int16_t value) {
  return dict_write_data_internal(iter, key, (const uint8_t * const) &value, sizeof(value), TUPLE_INT);
}

DictionaryResult dict_write_int32(DictionaryIterator *iter, const uint32_t key, const int32_t value) {
  return dict_write_data_internal(iter, key, (const uint8_t * const) &value, sizeof(value), TUPLE_INT);
}

DictionaryResult dict_write_int(DictionaryIterator *iter, const uint32_t key, const void *integer, const uint8_t width_bytes, const bool is_signed) {
  return dict_write_data_internal(iter, key, integer, width_bytes, is_signed ? TUPLE_INT : TUPLE_UINT);
}

uint32_t dict_write_end(DictionaryIterator *iter) {
  if (iter == NULL ||
      iter->dictionary == NULL ||
      iter->cursor == NULL) {
    return 0;
  }
  iter->end = iter->cursor;
  return dict_size(iter);
}

//! Returns the cursor, or NULL if the tuple at cursor extends beyond the bounds of the backing storage
static Tuple * get_safe_cursor(DictionaryIterator *iter) {
  // If iter->cursor is already at the end, return now so we don't try and read past the end of
  // the malloc'ed block (when fetching iter->cursor->length) and possibly cause a memory read
  // exception. 
  if ((void*)iter->cursor >= iter->end) {
    return NULL;
  }
  Tuple * const next_cursor = cursor_after_tuple_with_data_length(iter, iter->cursor->length);
  if ((void *)next_cursor > iter->end) {
    return NULL;
  }
  return iter->cursor;
}

Tuple * dict_read_begin_from_buffer(DictionaryIterator *iter, const uint8_t * const buffer, const uint16_t length) {
  const DictionaryResult result = dict_init(iter, buffer, length);
  if (result != DICT_OK) {
    return NULL;
  }
  return get_safe_cursor(iter);
}

Tuple * dict_read_next(DictionaryIterator *iter) {
  if (iter == NULL ||
      iter->dictionary == NULL ||
      iter->cursor == NULL) {
    return NULL;
  }
  iter->cursor = cursor_after_tuple_with_data_length(iter, iter->cursor->length);
  return get_safe_cursor(iter);
}

Tuple * dict_read_first(DictionaryIterator *iter) {
  if (iter == NULL ||
      iter->dictionary == NULL ||
      iter->cursor == NULL) {
    return NULL;
  }
  iter->cursor = iter->dictionary->head;
  return get_safe_cursor(iter);
}

uint32_t dict_calc_buffer_size(const uint8_t count, ...) {
  uint32_t total_size = sizeof(Dictionary);
  if (count == 0) {
    return total_size;
  }
  va_list vl;
  va_start(vl, count);
  for (unsigned int i = 0; i < count; ++i) {
    total_size += va_arg(vl, unsigned int) + sizeof(Tuple);
  }
  va_end(vl);
  return total_size;
}

uint32_t dict_calc_buffer_size_from_tuplets(const Tuplet * const tuplets, const uint8_t tuplets_count) {
  uint32_t total_size = sizeof(Dictionary);
  if (tuplets_count == 0) {
    return total_size;
  }
  for (unsigned int i = 0; i < tuplets_count; ++i) {
    const Tuplet * const tuplet = &tuplets[i];
    switch (tuplet->type) {
      case TUPLE_BYTE_ARRAY:
        total_size += tuplet->bytes.length;
        break;
      case TUPLE_CSTRING:
        total_size += tuplet->cstring.length;
        break;
      case TUPLE_INT:
      case TUPLE_UINT:
        total_size += tuplet->integer.width;
        break;
    }
    total_size += sizeof(Tuple);
  }
  return total_size;
}

// Legacy version to prevent previous app breakage, __deprecated preserves order
uint32_t dict_calc_buffer_size_from_tuplets__deprecated(const uint8_t tuplets_count, const Tuplet * const tuplets) {
  return dict_calc_buffer_size_from_tuplets(tuplets, tuplets_count);
}
  

static DictionaryResult dict_write_tuple(DictionaryIterator* iter, Tuple* tuple) {
  return dict_write_data_internal(iter, tuple->key, (uint8_t*)tuple->value,
                                  tuple->length, tuple->type);
}

DictionaryResult dict_write_tuplet(DictionaryIterator *iter, const Tuplet * const tuplet) {
  if (iter == NULL ||
      iter->dictionary == NULL ||
      iter->cursor == NULL) {
    return DICT_INVALID_ARGS;
  }
  switch (tuplet->type) {
    case TUPLE_BYTE_ARRAY:
      return dict_write_data_internal(iter, tuplet->key, tuplet->bytes.data, tuplet->bytes.length, tuplet->type);
    case TUPLE_CSTRING:
      return dict_write_data_internal(iter, tuplet->key, (uint8_t *)tuplet->cstring.data, tuplet->cstring.length, tuplet->type);
    case TUPLE_UINT:
    case TUPLE_INT:
      return dict_write_data_internal(iter, tuplet->key, (uint8_t *)&tuplet->integer.storage, tuplet->integer.width, tuplet->type);
  }
  return DICT_INVALID_ARGS;
}

DictionaryResult dict_serialize_tuplets_to_buffer_with_iter( 
  DictionaryIterator *iter, 
  const Tuplet * const tuplets, const uint8_t tuplets_count, 
  uint8_t *buffer, uint32_t *size_in_out) {
  if (size_in_out == NULL ||
      buffer == NULL ||
      tuplets == NULL) {
    return DICT_INVALID_ARGS;
  }
  DictionaryResult result;
  if ((result = dict_write_begin(iter, buffer, *size_in_out)) != DICT_OK) {
    return result;
  }
  for (unsigned int i = 0; i < tuplets_count; ++i) {
    if ((result = dict_write_tuplet(iter, &tuplets[i])) != DICT_OK) {
      return result;
    }
  }
  *size_in_out = dict_write_end(iter);
  return DICT_OK;
}

// Legacy version to prevent previous app breakage, __deprecated preserves order
DictionaryResult dict_serialize_tuplets_to_buffer_with_iter__deprecated(const uint8_t tuplets_count,
    const Tuplet * const tuplets, DictionaryIterator *iter, uint8_t *buffer, uint32_t *size_in_out) {

  return dict_serialize_tuplets_to_buffer_with_iter(iter, 
      tuplets, tuplets_count, buffer, size_in_out);
}


DictionaryResult dict_serialize_tuplets_to_buffer(const Tuplet * const tuplets, const uint8_t tuplets_count, uint8_t *buffer, uint32_t *size_in_out) {
  DictionaryIterator iter;
  return dict_serialize_tuplets_to_buffer_with_iter(&iter, tuplets, tuplets_count, buffer, size_in_out);
}


// Legacy version to prevent previous app breakage, __deprecated preserves order
DictionaryResult dict_serialize_tuplets_to_buffer__deprecated(
    const uint8_t tuplets_count, const Tuplet * const tuplets, 
    uint8_t *buffer, uint32_t *size_in_out) {
  DictionaryIterator iter;
  return dict_serialize_tuplets_to_buffer_with_iter(&iter, 
      tuplets, tuplets_count, buffer, size_in_out);
}


DictionaryResult dict_serialize_tuplets(DictionarySerializeCallback callback, void *context, const Tuplet * const tuplets, const uint8_t tuplets_count) {
  if (tuplets_count == 0) {
    const Dictionary dict = { .count = 0 };
    callback((const uint8_t *)&dict, sizeof(Dictionary), context);
    return DICT_OK;
  }
  uint32_t size = dict_calc_buffer_size_from_tuplets(tuplets, tuplets_count);
  uint8_t buffer[size];
  DictionaryResult result = dict_serialize_tuplets_to_buffer(tuplets, tuplets_count, buffer, &size);
  if (result != DICT_OK) {
    return result;
  }
  callback(buffer, size, context);
  return DICT_OK;
}


// Legacy version to prevent previous app breakage, __deprecated preserves order
DictionaryResult dict_serialize_tuplets__deprecated(DictionarySerializeCallback callback, void *context, const uint8_t tuplets_count, const Tuplet * const tuplets) {
  return dict_serialize_tuplets(callback, context, tuplets, tuplets_count);
}

static const uint8_t NULL_TUPLE_BUFFER[sizeof(Tuple) + sizeof(uint32_t)] = { 0 };
const Tuple * const NULL_TUPLE = (const Tuple * const) NULL_TUPLE_BUFFER;

static uint8_t* dict_copy(DictionaryIterator* iter) {
  size_t size = dict_size(iter);
  uint8_t* buf = task_malloc(size);
  if (buf == NULL) return NULL;
  memcpy(buf, iter->dictionary, size);
  return buf;
}

// Merge orig_iter and new_iter into dest_iter. Keys which exist in both
// orig_iter and new_iter will get the value they have in new_iter.
static DictionaryResult dict_merge_to(DictionaryIterator* dest_iter,
                                      DictionaryIterator* orig_iter,
                                      DictionaryIterator* new_iter,
                                      const bool update_existing_keys_only,
                                      const DictionaryKeyUpdatedCallback update_key_callback,
                                      void* context) {
  DictionaryResult result = DICT_OK;

  // First, write the updated keys.
  for (Tuple* new = dict_read_first(new_iter); new; new = dict_read_next(new_iter)) {
    uint32_t key = new->key;
    const Tuple* orig = dict_find(orig_iter, key);
    if (orig == NULL && update_existing_keys_only) {
      continue;
    }
    if (orig == NULL) {
      orig = NULL_TUPLE;
    }
    Tuple* dest = dest_iter->cursor;
    result = dict_write_tuple(dest_iter, new);
    if (result != DICT_OK) return result;
    update_key_callback(key, dest, orig, context);
  }

  // Then, write any old keys which were not updated this round.
  // We still call update_key_callback here, even though the values
  // themselves have not changed, because we have shuffled them
  // around in memory, so their old buffers are no longer valid.
  for (Tuple* orig = dict_read_first(orig_iter); orig; orig = dict_read_next(orig_iter)) {
    uint32_t key = orig->key;
    Tuple* new = dict_find(new_iter, key);
    if (new != NULL) {
      // We already wrote this key, above.
      continue;
    }
    Tuple* dest = dest_iter->cursor;
    result = dict_write_tuple(dest_iter, orig);
    if (result != DICT_OK) return result;
    update_key_callback(key, dest, orig, context);
  }

  return DICT_OK;
}

// Calculate the amount of space needed for a dest_iter which can fit the result
// of merging orig_iter and new_iter. This logic should always mirror the logic
// in dict_merge_to, except it should simply count the size, rather than
// actually merging the results.
static size_t dict_merge_to_size(DictionaryIterator* orig_iter,
                                 DictionaryIterator* new_iter,
                                 const bool update_existing_keys_only) {
  size_t total_size_required = sizeof(Dictionary);

  // First, calculate the size of the new/updated keys.
  for (Tuple* new = dict_read_first(new_iter); new; new = dict_read_next(new_iter)) {
    if (dict_find(orig_iter, new->key) == NULL && update_existing_keys_only) continue;
    total_size_required += sizeof(*new) + new->length;
  }

  // Then, add in the size of the keys which have not changed.
  for (Tuple* orig = dict_read_first(orig_iter); orig; orig = dict_read_next(orig_iter)) {
    if (dict_find(new_iter, orig->key) != NULL) continue;
    total_size_required += sizeof(*orig) + orig->length;
  }

  return total_size_required;
}

DictionaryResult dict_merge(DictionaryIterator* dest_iter,
                            uint32_t* dest_buf_length_in_out,
                            DictionaryIterator* new_iter,
                            const bool update_existing_keys_only,
                            const DictionaryKeyUpdatedCallback update_key_callback,
                            void* context) {
  if (dest_iter == NULL || new_iter == NULL || dest_buf_length_in_out == NULL) {
    return DICT_INVALID_ARGS;
  }

  size_t required_size = dict_merge_to_size(dest_iter, new_iter, update_existing_keys_only);
  if (*dest_buf_length_in_out < required_size) {
    return DICT_NOT_ENOUGH_STORAGE;
  }

  uint8_t* orig_buffer = dict_copy(dest_iter);
  if (orig_buffer == NULL) return DICT_MALLOC_FAILED;

  DictionaryIterator orig_iter;
  DictionaryResult result = dict_init(&orig_iter, orig_buffer, dict_size(dest_iter));
  if (result != DICT_OK) goto cleanup;

  result = dict_write_begin(dest_iter,
                            (uint8_t*)dest_iter->dictionary,
                            (uint16_t)*dest_buf_length_in_out);
  if (result != DICT_OK) goto cleanup;

  result = dict_merge_to(dest_iter, &orig_iter, new_iter,
                         update_existing_keys_only,
                         update_key_callback, context);
  if (result != DICT_OK) goto cleanup;

  *dest_buf_length_in_out = dict_write_end(dest_iter);

cleanup:
  task_free(orig_buffer);
  return result;
}

Tuple *dict_find(const DictionaryIterator *iter, const uint32_t key) {
  DictionaryIterator iter_copy = *iter;
  Tuple *tuple = dict_read_first(&iter_copy);
  while (tuple) {
    if (tuple->key == key) {
      return tuple;
    }
    tuple = dict_read_next(&iter_copy);
  }
  return NULL;
}
