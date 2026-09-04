/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "util/mbuf.h"

#include <stdbool.h>
#include <stdint.h>

//! NOTE: MBufIterator APIs are not thread safe

typedef struct {
  MBuf *m;
  uint32_t data_index;
} MBufIterator;

//! Initializes an MBufIterator
void mbuf_iterator_init(MBufIterator *iter, MBuf *m);

//! Check if there is no data left in the MBuf chain
bool mbuf_iterator_is_finished(MBufIterator *iter);

//! Reads the next byte of data in the MBuf chain
bool mbuf_iterator_read_byte(MBufIterator *iter, uint8_t *data);

//! Writes the next byte of data in the MBuf chain
bool mbuf_iterator_write_byte(MBufIterator *iter, uint8_t data);

//! Gets the MBuf which the next byte of data is in
MBuf *mbuf_iterator_get_current_mbuf(MBufIterator *iter);
