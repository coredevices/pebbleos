/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "console/prompt.h"
#include <pbl/drivers/mcu.h>
#include "mfg/mfg_info.h"
#include "mfg/mfg_serials.h"
#include "resource/resource.h"
#include "resource/system_resource.h"
#include "pbl/services/bluetooth/bluetooth_persistent_storage.h"
#include "pbl/services/bluetooth/local_id.h"
#include "pbl/services/comm_session/protocol.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/services/comm_session/session_remote_version.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/activity/insights_settings.h"
#include "pbl/services/notifications/notification_image.h"
#include "shell/system_app_ids.auto.h"
#include "system/bootbits.h"
#include <pbl/logging/logging.h>
#include "system/version.h"
#include "pbl/util/attributes.h"
#include "util/net.h"
#include "pbl/util/string.h"

#include <bluetooth/bluetooth_types.h>

#include <string.h>

#define VERSION_REQUEST 0x00
#define VERSION_RESPONSE 0x01

static const uint16_t s_endpoint_id = 0x0010;

struct PACKED VersionsMessage {
  const uint8_t command;
  FirmwareMetadata running_fw_metadata;
  FirmwareMetadata recovery_fw_metadata;
  uint32_t boot_version;
  char hw_version[MFG_HW_VERSION_SIZE];
  char serial_number[MFG_SERIAL_NUMBER_SIZE];
  BTDeviceAddress device_address;
  ResourceVersion system_resources_version;
  char iso_locale[ISO_LOCALE_LENGTH];
  uint16_t lang_version;
  // Use this padding string for additional bit flags passed by
  // >= 2.X versions of the mobile applications.  ISO + locale
  // on 1.X mobile application versions.
  PebbleProtocolCapabilities capabilities;
  bool is_unfaithful;
  net16 activity_insights_version;
  net16 javascript_bytecode_version;
};

static void fixup_string(char* str, unsigned int length) {
  if (memchr(str, 0, length) == NULL) {
    memset(str, 0, length);
  }
}

static void prv_fixup_firmware_metadata(FirmwareMetadata *fw_metadata) {
  fw_metadata->version_timestamp = htonl(fw_metadata->version_timestamp);
  fixup_string(fw_metadata->version_tag, sizeof(fw_metadata->version_tag));
  fixup_string(fw_metadata->version_short, sizeof(fw_metadata->version_short));
}

static void prv_fixup_running_firmware_metadata(FirmwareMetadata *fw_metadata) {
  prv_fixup_firmware_metadata(fw_metadata);

#ifdef CONFIG_MFG
  // Lie to the phone and force this to say we're not a MFG firmware. If we tell the phone app
  // that we're a MFG firmware it will get mad at us and try to update us out of this mode. We
  // want to stay in this mode to collect logs and core dumps at the factory.
  // FIXME: Long term the phone should probably just be able to collect logs and core dumps
  // regardless of the state of the watch, but for now just lie.
  fw_metadata->is_recovery_firmware = false;
#endif
}

static void resource_version_to_network_endian(ResourceVersion *resources_version) {
  resources_version->crc = htonl(resources_version->crc);
  resources_version->timestamp = htonl(resources_version->timestamp);
}

static void prv_send_watch_versions(CommSession *session) {
  struct VersionsMessage versions_msg = {
    .command = VERSION_RESPONSE,
    .boot_version = htonl(boot_version_read()),
  };

  _Static_assert(sizeof(struct VersionsMessage) >=
            126 /* pre-v1.5 version info */ +
            24 /* v1.5 version info or later, added system_resources_version */,
            "");

  version_copy_running_fw_metadata(&versions_msg.running_fw_metadata);
  prv_fixup_running_firmware_metadata(&versions_msg.running_fw_metadata);

  version_copy_recovery_fw_metadata(&versions_msg.recovery_fw_metadata);
  prv_fixup_firmware_metadata(&versions_msg.recovery_fw_metadata);

  // Note: Don't worry about the null terminator if it doesn't fit, the other side should deal with it.
  mfg_info_get_hw_version(versions_msg.hw_version, sizeof(versions_msg.hw_version));
  mfg_info_get_serialnumber(versions_msg.serial_number, sizeof(versions_msg.serial_number));

  strncpy(versions_msg.iso_locale, i18n_get_locale(), ISO_LOCALE_LENGTH - 1);
  versions_msg.iso_locale[ISO_LOCALE_LENGTH - 1] = '\0';
  versions_msg.lang_version = htons(i18n_get_version());
  PBL_LOG_DBG("Sending lang version: %d", versions_msg.lang_version);

  // Set the capabilities as zero, effectively saying that we don't support anything.
  versions_msg.capabilities.flags = 0;
  // Assign the individual bits for the capabilities that we support.
  versions_msg.capabilities.run_state_support = 1;
  versions_msg.capabilities.infinite_log_dumping_support = 1;
  versions_msg.capabilities.extended_music_service = 1;
  versions_msg.capabilities.extended_notification_service = 1;
  versions_msg.capabilities.lang_pack_support = 1;
  versions_msg.capabilities.app_message_8k_support = 1;
  versions_msg.capabilities.activity_insights_support = 1;
  versions_msg.capabilities.voice_api_support = 1;
  versions_msg.capabilities.unread_coredump_support = 1;
  // FIXME: PBL-31627 In PRF, APP_ID_SEND_TEXT isn't defined - requiring the #ifdef and ternary op.
#ifdef APP_ID_SEND_TEXT
  versions_msg.capabilities.send_text_support = (APP_ID_SEND_TEXT != INSTALL_ID_INVALID) ? 1 : 0;
#endif
  versions_msg.capabilities.notification_filtering_support = 1;
#ifdef APP_ID_WEATHER
  versions_msg.capabilities.weather_app_support = (APP_ID_WEATHER != INSTALL_ID_INVALID) ? 1 : 0;
#endif
#ifdef APP_ID_REMINDERS
  versions_msg.capabilities.reminders_app_support =
      (APP_ID_REMINDERS != INSTALL_ID_INVALID) ? 1 : 0;
#endif
#ifdef APP_ID_WORKOUT
  versions_msg.capabilities.workout_app_support = (APP_ID_WORKOUT != INSTALL_ID_INVALID) ? 1 : 0;
#endif
  versions_msg.capabilities.continue_fw_install_across_disconnect_support = 1;
  versions_msg.capabilities.smooth_fw_install_progress_support = 1;
  versions_msg.capabilities.custom_vibe_pattern_support = 1;
  versions_msg.capabilities.blob_db_version_support = 1;
  versions_msg.capabilities.weather_db_v4_support = 1;
  versions_msg.capabilities.notification_image_support = NOTIFICATION_IMAGE_SUPPORTED;
  bt_local_id_copy_address(&versions_msg.device_address);

  versions_msg.system_resources_version = resource_get_system_version();
  resource_version_to_network_endian(&versions_msg.system_resources_version);

  versions_msg.is_unfaithful = bt_persistent_storage_is_unfaithful();
#if !defined(CONFIG_RECOVERY_FW)
  versions_msg.activity_insights_version = hton16(activity_insights_settings_get_version());
#endif

  comm_session_send_data(session, s_endpoint_id, (uint8_t*) &versions_msg, sizeof(versions_msg),
                         COMM_SESSION_DEFAULT_TIMEOUT);
}

void system_version_protocol_msg_callback(CommSession *session, const uint8_t* data, size_t length) {
  switch (data[0]) {
  case VERSION_REQUEST: {
    prv_send_watch_versions(session);
    break;
  }
  default:
    PBL_LOG_ERR("Invalid message received. First byte is %u", data[0]);
    break;
  }
}

void command_version_info(void) {
#ifdef CONFIG_MFG
  prompt_send_response("MANUFACTURING FW");
#endif

  bool (*fun_ptr[2])(FirmwareMetadata*) = { version_copy_running_fw_metadata,
                                            version_copy_recovery_fw_metadata};
  const char *label[2] = {"Running", "Recovery"};

  FirmwareMetadata fw_metadata;
  char buffer[128];
  for (int i = 0; i < 2; ++i) {
    bool success = fun_ptr[i](&fw_metadata);
    if (success) {
      prompt_send_response_fmt(
          buffer, sizeof(buffer),
          "%s FW:\n  ts:%"PRIu32"\n  tag:%s\n  short:%s\n  recov:%u\n  platform:%u",
          label[i], fw_metadata.version_timestamp, fw_metadata.version_tag,
          fw_metadata.version_short, fw_metadata.is_recovery_firmware, fw_metadata.hw_platform);

      if ((i == 0) && fw_metadata.is_dual_slot) {
        prompt_send_response_fmt(buffer, sizeof(buffer), "  dual slot");
        if (!fw_metadata.is_recovery_firmware) {
          prompt_send_response_fmt(buffer, sizeof(buffer), "  current slot:%s",
                                   fw_metadata.is_slot_0 ? "0" : "1");
        }
      }
    } else {
      prompt_send_response_fmt(buffer, sizeof(buffer), "%s FW: no version info or lookup failed",
                               label[i]);
    }
  }

  char build_id_string[64];
  version_copy_current_build_id_hex_string(build_id_string, sizeof(build_id_string));
  prompt_send_response_fmt(buffer, sizeof(buffer), "Build Id:%s", build_id_string);

  char serial_number[MFG_SERIAL_NUMBER_SIZE + 1];
  mfg_info_get_serialnumber(serial_number, sizeof(serial_number));

  char hw_version[MFG_HW_VERSION_SIZE + 1];
  mfg_info_get_hw_version(hw_version, sizeof(hw_version));

  uint8_t mcu_serial[32];
  size_t mcu_serial_size = sizeof(mcu_serial);
  StatusCode err = mcu_get_serial(mcu_serial, &mcu_serial_size);
  if (err != S_SUCCESS) {
    prompt_send_response_fmt(buffer, sizeof(buffer),
                             "MCU Serial: N/A (%d)", err);
  } else {
    char serial_str[sizeof(mcu_serial) * 2 + 1];
    byte_stream_to_hex_string(serial_str, sizeof(serial_str), mcu_serial, mcu_serial_size, false);
    prompt_send_response_fmt(buffer, sizeof(buffer), "MCU Serial: %s", serial_str);
  }

  prompt_send_response_fmt(buffer, sizeof(buffer), "Boot:0x%08"PRIx32"\nHW:%s\nSN:%s",
                           boot_version_read(), hw_version, serial_number);

  ResourceVersion system_resources_version = resource_get_system_version();
  prompt_send_response_fmt(buffer, sizeof(buffer),
                           "System Resources:\n  CRC:0x%"PRIx32"\n  Valid:%s",
                           system_resources_version.crc, bool_to_str(system_resource_is_valid()));
}
