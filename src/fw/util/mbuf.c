/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "mbuf.h"

#include "kernel/pbl_malloc.h"
#include <pbl/logging/logging.h>
#include "pbl/kernel/mutex.h"
#include "system/passert.h"
#include "pbl/util/size.h"

//! Flags used for internal purposes (bits 24-31 are allocated for this purpose)
#define MBUF_FLAG_IS_MANAGED ((uint32_t)(1 << 24))
#define MBUF_FLAG_IS_FREE ((uint32_t)(1 << 25))

T_STATIC MBuf *s_free_list;
static PBL_MUTEX_DEFINE(s_free_list_lock);

//! This array should be initialized with the maximum number of MBufs which may be allocated for
//! each pool.
static int s_mbuf_pool_space[] = {
#if UNITTEST
  [MBufPoolUnitTest] = 100,
#endif
};
_Static_assert(ARRAY_LENGTH(s_mbuf_pool_space) == NumMBufPools,
               "s_mbuf_pool_space array does not match MBufPool enum");


// Init
////////////////////////////////////////////////////////////////////////////////

void mbuf_init(void) {
}


// Allocation / free list management
////////////////////////////////////////////////////////////////////////////////

//! Bug-catcher checks that nobody has corrupted the free list or modified MBufs within it
//! NOTE: the caller must hold s_free_list_lock
static void prv_check_free_list(void) {
  pbl_mutex_assert_held(&s_free_list_lock, true);
  MBuf *m = s_free_list;
  while (m) {
    PBL_ASSERTN(mbuf_is_flag_set(m, MBUF_FLAG_IS_MANAGED));
    PBL_ASSERTN(mbuf_is_flag_set(m, MBUF_FLAG_IS_FREE));
    PBL_ASSERTN(!mbuf_get_data(s_free_list));
    PBL_ASSERTN(mbuf_get_length(m) == 0);
    m = mbuf_get_next(m);
  }
}

#if UNITTEST
MBuf *mbuf_get(void *data, uint32_t length, MBufPool pool) {
  PBL_ASSERTN(pool < NumMBufPools);
  MBuf *m;
  pbl_mutex_lock(&s_free_list_lock, PBL_FOREVER);
  // get an MBuf out of the free list if possible, or else allocate a new one
  if (s_free_list) {
    prv_check_free_list();
    // remove the head of the free list to be returned
    m = s_free_list;
    s_free_list = mbuf_get_next(s_free_list);
    mbuf_clear_next(m);
  } else {
    // check that there is space left in this pool
    PBL_ASSERTN(s_mbuf_pool_space[pool] > 0);
    s_mbuf_pool_space[pool]--;
    // allocate and initialize a new MBuf for the pool
    m = kernel_zalloc_check(sizeof(MBuf));
    mbuf_set_flag(m, MBUF_FLAG_IS_MANAGED, true);
  }
  pbl_mutex_unlock(&s_free_list_lock);

  mbuf_set_flag(m, MBUF_FLAG_IS_FREE, false);
  mbuf_set_data(m, data, length);
  return m;
}
#endif

void mbuf_free(MBuf *m) {
  if (!m) {
    return;
  }
  PBL_ASSERTN(mbuf_is_flag_set(m, MBUF_FLAG_IS_MANAGED));
  PBL_ASSERTN(!mbuf_is_flag_set(m, MBUF_FLAG_IS_FREE)); // double free

  // clear the MBuf
  *m = MBUF_EMPTY;
  mbuf_set_flag(m, MBUF_FLAG_IS_MANAGED, true);
  mbuf_set_flag(m, MBUF_FLAG_IS_FREE, true);

  // add it to the free list
  pbl_mutex_lock(&s_free_list_lock, PBL_FOREVER);
  if (s_free_list) {
    mbuf_append(s_free_list, m);
  } else {
    s_free_list = m;
  }
  prv_check_free_list();
  pbl_mutex_unlock(&s_free_list_lock);
}


// Basic setters and getters
////////////////////////////////////////////////////////////////////////////////

void mbuf_set_data(MBuf *m, void *data, uint32_t length) {
  PBL_ASSERTN(m);
  //! We should never be trying to set the data on an mbuf in the free list
  PBL_ASSERTN(!mbuf_is_flag_set(m, MBUF_FLAG_IS_FREE));
  m->data = data;
  m->length = length;
}

void *mbuf_get_data(MBuf *m) {
  PBL_ASSERTN(m);
  return m->data;
}

bool mbuf_is_flag_set(MBuf *m, uint32_t flag) {
  PBL_ASSERTN(m);
  return m->flags & flag;
}

void mbuf_set_flag(MBuf *m, uint32_t flag, bool is_set) {
  PBL_ASSERTN(m);
  if (is_set) {
    m->flags |= flag;
  } else {
    m->flags &= ~flag;
  }
}

MBuf *mbuf_get_next(MBuf *m) {
  PBL_ASSERTN(m);
  return m->next;
}

uint32_t mbuf_get_length(MBuf *m) {
  PBL_ASSERTN(m);
  return m->length;
}

uint32_t mbuf_get_chain_length(MBuf *m) {
  uint32_t total = 0;
  while (m) {
    total += m->length;
    m = m->next;
  }
  return total;
}


// MBuf chain management
////////////////////////////////////////////////////////////////////////////////

void mbuf_append(MBuf *m, MBuf *new_mbuf) {
  PBL_ASSERTN(m);
  // advance to the tail
  while (m->next) {
    m = m->next;
  }
  m->next = new_mbuf;
}

void mbuf_clear_next(MBuf *m) {
  PBL_ASSERTN(m);
  m->next = NULL;
}


// Debug
////////////////////////////////////////////////////////////////////////////////

void mbuf_debug_dump(MBuf *m) {
  char buffer[80];
  while (m) {
    dbgserial_putstr_fmt(buffer, sizeof(buffer), "MBuf <%p>: length=%"PRIu32", data=%p, "
                         "flags=0x%"PRIx32, m, m->length, m->data, m->flags);
    m = m->next;
  }
}
