/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/util/attributes.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct CommSession CommSession;

// Capabilities are a bitfield set by passing the capabilities character array in
// system_versions.c.  The corresponding mobile applications return an integer
// field indicating which endpoints it has support for over the deprecated ones.
typedef struct PACKED {
  union {
    struct PACKED {
      bool run_state_support:1;
      bool infinite_log_dumping_support:1;
      bool extended_music_service:1;
      bool extended_notification_service:1;
      bool lang_pack_support:1;
      bool app_message_8k_support:1;
      bool activity_insights_support:1;
      bool voice_api_support:1;
      bool send_text_support:1;
      bool notification_filtering_support:1;
      bool unread_coredump_support:1;
      bool weather_app_support:1;
      bool reminders_app_support:1;
      bool workout_app_support:1;
      bool smooth_fw_install_progress_support:1;
      bool custom_vibe_pattern_support:1;
      uint8_t javascript_bytecode_version_appended: 1;
      bool imaging_support:1;  // Phone serves images (album art, ...) via the imaging endpoint
      bool notification_image_support:1;  // Watch renders AttributeIdImageAspectRatio notifications
      uint8_t more_padded_bits:2;
      bool continue_fw_install_across_disconnect_support: 1;
      bool blob_db_version_support: 1;
      bool settings_sync_support: 1;  // Phone supports Settings BlobDB sync
      bool weather_db_v4_support: 1;  // Phone writes the v4 weather BlobDB record (rich forecast)
    };
    uint64_t flags;
  };
} PebbleProtocolCapabilities;

void session_remote_version_start_requests(CommSession *session);
