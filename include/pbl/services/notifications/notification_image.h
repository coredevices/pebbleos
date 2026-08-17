/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pbl/util/uuid.h"

struct GBitmap;

//! Notification images need a colour display and enough RAM for the decoded bitmap, so only emery
//! and gabbro qualify — the same bar as album art.
#if defined(CONFIG_PLATFORM_EMERY) || defined(CONFIG_PLATFORM_GABBRO)
#define NOTIFICATION_IMAGE_SUPPORTED 1
#else
#define NOTIFICATION_IMAGE_SUPPORTED 0
#endif

//! Single slot holding the phone-supplied image for the notification currently on screen. Only one
//! notification card is ever focused, so one slot is the whole requirement.

//! Create the lock. Call once at boot before any other function here.
void notification_image_service_init(void);

//! Claim the slot for `item_id` and write the imaging request token to `token_out`. Returns false
//! if the slot is already this item's — whether the image is in flight, delivered, or the phone
//! already said it has none — so callers can drive this straight from a redraw.
bool notification_image_claim(const Uuid *item_id, uint8_t *token_out);

//! The stored bitmap for `item_id`, or NULL. Ownership stays with the store, which is locked until
//! the matching notification_image_unlock — call that even when this returns NULL.
const struct GBitmap *notification_image_lock(const Uuid *item_id);

void notification_image_unlock(void);

//! True while a response for `item_id` is still outstanding, so the card can show a placeholder
//! rather than an empty band — but stop once the phone has said it has no image.
bool notification_image_is_pending(const Uuid *item_id);

//! Take ownership of a delivered bitmap (NULL when the phone had no image). Ignores stale tokens.
//! @return true if the slot resolved, so anything showing this notification needs to redraw — also
//! when [bitmap] is NULL, which is what takes the pending placeholder back down.
bool notification_image_store(uint8_t token, struct GBitmap *bitmap);

//! Drop the slot and free the bitmap.
void notification_image_clear(void);
