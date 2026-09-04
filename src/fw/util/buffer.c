/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "buffer.h"

#include "system/passert.h"

#include "kernel/pbl_malloc.h"

#include <string.h>


size_t buffer_get_bytes_remaining(Buffer *b) {
  return (b->length - b->bytes_written);
}

size_t buffer_add(Buffer* const b, const uint8_t* const data, const size_t length) {
  PBL_ASSERTN(b);
  PBL_ASSERTN(data);
  PBL_ASSERTN(length);

  if (buffer_get_bytes_remaining(b) < length) {
    return 0;
  }

  uint8_t* cursor = b->data + b->bytes_written;
  memcpy(cursor, data, length);
  b->bytes_written += length;

  return length;
}

size_t buffer_remove(Buffer* const b, const size_t offset, const size_t length) {
  PBL_ASSERTN(offset + length <= b->bytes_written);

  memmove(&b->data[offset], &b->data[offset + length], b->bytes_written - length - offset);
  b->bytes_written -= length;
  return length;
}

Buffer* buffer_create(const size_t size_bytes) {
  PBL_ASSERTN(size_bytes);
  Buffer* b = kernel_malloc_check(sizeof(Buffer) + size_bytes);
  *b = (Buffer) {
    .length = size_bytes,
    .bytes_written = 0,
  };
  return b;
}

void buffer_init(Buffer * const buffer, const size_t length) {
  buffer->bytes_written = 0;
  buffer->length = length;
}

void buffer_clear(Buffer * const buffer) {
  buffer->bytes_written = 0;
}

bool buffer_is_empty(Buffer * const buffer) {
  return buffer->bytes_written == 0;
}
