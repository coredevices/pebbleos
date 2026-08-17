/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/notifications/notification_image.h"

#include "applib/graphics/gtypes.h"
#include "kernel/pbl_malloc.h"
#include "pbl/os/mutex.h"

//! Responses are delivered on KernelMain while the card that requested and renders the image runs
//! on KernelMain (modal) or the App task (notification history), so the slot needs a lock. It is
//! recursive because the renderer holds it across the blit to stop the bitmap being freed
//! underneath it.
static PebbleRecursiveMutex *s_lock;

static Uuid s_item_id;
static GBitmap *s_bitmap;
static uint8_t s_token;
static bool s_claimed;  //!< The slot belongs to s_item_id (in flight, delivered or empty).
static bool s_pending;  //!< Waiting on a response for s_token.

static void prv_free(GBitmap *bitmap) {
  if (bitmap) {
    kernel_free(bitmap->addr);
    kernel_free(bitmap->palette);
    kernel_free(bitmap);
  }
}

void notification_image_service_init(void) {
  s_lock = mutex_create_recursive();
}

bool notification_image_claim(const Uuid *item_id, uint8_t *token_out) {
  if (!item_id || !token_out) {
    return false;
  }
  mutex_lock_recursive(s_lock);
  if (s_claimed && uuid_equal(&s_item_id, item_id)) {
    mutex_unlock_recursive(s_lock);
    return false;
  }
  prv_free(s_bitmap);
  s_bitmap = NULL;
  s_item_id = *item_id;
  s_claimed = true;
  s_pending = true;
  *token_out = ++s_token;
  mutex_unlock_recursive(s_lock);
  return true;
}

const GBitmap *notification_image_lock(const Uuid *item_id) {
  mutex_lock_recursive(s_lock);
  // Held until notification_image_unlock so the bitmap can't be freed mid-draw.
  if (!s_bitmap || !item_id || !s_claimed || !uuid_equal(&s_item_id, item_id)) {
    return NULL;
  }
  return s_bitmap;
}

void notification_image_unlock(void) {
  mutex_unlock_recursive(s_lock);
}

bool notification_image_is_pending(const Uuid *item_id) {
  mutex_lock_recursive(s_lock);
  const bool pending =
      s_pending && s_claimed && item_id && uuid_equal(&s_item_id, item_id);
  mutex_unlock_recursive(s_lock);
  return pending;
}

bool notification_image_store(uint8_t token, GBitmap *bitmap) {
  mutex_lock_recursive(s_lock);
  if (!s_pending || token != s_token) {
    mutex_unlock_recursive(s_lock);
    prv_free(bitmap);
    return false;
  }
  s_pending = false;
  s_bitmap = bitmap;
  mutex_unlock_recursive(s_lock);
  return true;
}

void notification_image_clear(void) {
  mutex_lock_recursive(s_lock);
  prv_free(s_bitmap);
  s_bitmap = NULL;
  s_claimed = false;
  s_pending = false;
  mutex_unlock_recursive(s_lock);
}
