/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "attribute.h"
#include "item.h"

#include "pbl/services/comm_session/session.h"
#include "pbl/util/uuid.h"

//! Sends a request to the phone asking it to invoke an action
//! @param id UUID of the pin/notification
//! @param type Type of the pin/notification
//! @param action_id The id of the action that is being invoked
//! @param attributes The list of attributes
//! @param do_async True = perform send on KernelBG, False = perform send on current task
void timeline_action_endpoint_invoke_action(const Uuid *id, TimelineItemActionType type,
                                            uint8_t action_id, const AttributeList *attributes,
                                            bool do_async);

//! Handles messages from the phone sent to the timeline action endpoint
void timeline_action_endpoint_protocol_msg_callback(CommSession *session, const uint8_t* data,
                                                    size_t length);
