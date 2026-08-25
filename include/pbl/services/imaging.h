/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "kernel/events.h"
#include "pbl/services/imaging_endpoint_types.h"
#include "pbl/util/uuid.h"

struct GBitmap;
typedef struct CommSession CommSession;

//! Generic image-fetch service. Consumers (album art, ...) ask the phone for an image and get the
//! reassembled bitmap back through a registered handler. Watch-pull only, capability-gated.

//! Called on KernelMain when a requested image finishes transferring. `bitmap` is NULL when the
//! phone reported it has no image (ImagingResponseFlagNoImage). Ownership of a non-NULL `bitmap`
//! (and its pixel/palette buffers) passes to the handler. `token` echoes the request.
typedef void (*ImagingReceivedHandler)(uint8_t token, struct GBitmap *bitmap);

//! Register the handler for an image type. One handler per type; overwrites any previous.
void imaging_register_handler(ImagingImageType image_type, ImagingReceivedHandler handler);

//! True if the connected phone advertises image-fetch support and hasn't told us it can't serve
//! this image type (see ImagingResponseFlagUnsupported). Latched state resets on reconnect.
bool imaging_is_type_supported(ImagingImageType image_type);

//! Ask the phone for an album-art image for the named track, at the given size/format. No-op (and
//! returns false) if unsupported. The matching handler is invoked when the transfer completes.
bool imaging_request_album_art(uint8_t token, ImagingFormat format, uint16_t width, uint16_t height,
                               const char *title, const char *artist);

//! Ask the phone for the image it holds for timeline item `item_id`, at the given size/format.
//! No-op (and returns false) if unsupported. The matching handler is invoked when the transfer
//! completes.
bool imaging_request_notification_image(uint8_t token, ImagingFormat format, uint16_t width,
                                        uint16_t height, const Uuid *item_id);

//! Endpoint receive callback (registered in protocol_endpoints_table.json).
void imaging_protocol_msg_callback(CommSession *session, const uint8_t *msg, size_t length);

//! Comm-session event hook (called from the shell event loop): frees a partially received image
//! and clears the unsupported-type latch when the system session closes.
void imaging_handle_comm_session_event(const PebbleCommSessionEvent *event);
