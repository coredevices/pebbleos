/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "ancs_notifications_util.h"

#include "comm/ble/kernel_le_client/ancs/ancs_types.h"
#include "pbl/services/blob_db/ios_notif_pref_db.h"

//! Creates a new timeline item from ANCS data
//! @param notif_attributes ANCS Notification attributes
//! @param app_attributes ANCS App attributes (namely, the display name)
//! @param app_metadata The icon and color associated with the app
//! @param notif_prefs iOS notification prefs for this notification
//! @param timestamp Time the notification occured
//! @param properties Additional ANCS properties (category, flags, etc)
//! @return The newly created timeline item
TimelineItem *ancs_item_create_and_populate(ANCSAttribute *notif_attributes[],
                                            ANCSAttribute *app_attributes[],
                                            const ANCSAppMetadata *app_metadata,
                                            iOSNotifPrefs *notif_prefs,
                                            time_t timestamp,
                                            ANCSProperty properties);

//! Replaces the dismiss action of an existing timeline item with the ancs negative action
//! @param item The timeline item to update
//! @param uid The uid of the ANCS notification we're using the dismiss action from
//! @param attr_action_neg The negative action from the ANCS notification to use as the new
//! dismiss action
void ancs_item_update_dismiss_action(TimelineItem *item, uint32_t uid,
                                     const ANCSAttribute *attr_action_neg);
