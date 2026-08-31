/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/ui/dialogs/dialog.h"
#include "applib/ui/dialogs/simple_dialog.h"
#include <pbl/drivers/battery.h>
#include "kernel/event_loop.h"
#include "kernel/ui/modals/modal_manager.h"
#include "pbl/services/accel_manager.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/services/system_task.h"
#include <pbl/logging/logging.h>
#include "pbl/util/attributes.h"
#include "util/net.h"

#include <inttypes.h>

PBL_LOG_MODULE_DEFINE(service_ping, CONFIG_SERVICE_PING_LOG_LEVEL);

#define PING_ENDPOINT 2001
#define PING_MIN_PERIOD_SECS  (60 * 60)    // 1 hour

static time_t s_last_send_time;
static bool s_is_ping_kernel_bg_callback_scheduled;


// ---------------------------------------------------------------------------------------------------------
// Ping Pong structures
typedef struct PACKED {
  uint8_t  cmd;
  uint32_t cookie;
} PingMsgHeader;

typedef struct PACKED {
  PingMsgHeader hdr;
} PingMsgV1;

typedef struct PACKED {
  PingMsgHeader hdr;
  uint8_t idle;   // Optional
} PingMsgV2;

typedef struct PACKED {
  PingMsgHeader hdr;
} PongMsg;


static void prv_send_ping_kernel_bg_cb(void *unused) {
  CommSession *system_session = comm_session_get_system_session();
  if (system_session) {
    // Are we idle?
    bool idle = (battery_is_usb_connected() || accel_is_idle());

    PingMsgV2 ping_msg = (PingMsgV2) {
      .hdr = {
        .cmd = 0,
        .cookie = htonl(42)
      },
      .idle = idle
    };
    bool success = comm_session_send_data(system_session, PING_ENDPOINT,
                                          (const uint8_t *) &ping_msg, sizeof(ping_msg),
                                          COMM_SESSION_DEFAULT_TIMEOUT);
    if (success) {
      s_last_send_time = rtc_get_time();
    }
    PBL_LOG_DBG("Sent ping idle=%d, success=%d", (int)idle, (int)success);
  }

  s_is_ping_kernel_bg_callback_scheduled = false;
}

// ------------------------------------------------------------------------------------------------------------
// If a ping is due to be sent, send it.
// bt_lock() is held by the caller
void ping_send_if_due(void) {
  if (s_is_ping_kernel_bg_callback_scheduled) {
    return;
  }

  // Only send if we haven't sent within the last PING_MIN_PERIOD_SECS
  time_t current_time = rtc_get_time();
  if (current_time < s_last_send_time + PING_MIN_PERIOD_SECS) {
    return;
  }

  // Offload to KernelBG, because we cannot use comm_session_send_data() with bt_lock held.
  system_task_add_callback(prv_send_ping_kernel_bg_cb, NULL);
  s_is_ping_kernel_bg_callback_scheduled = true;
}

static void prv_push_window(void *data) {
  SimpleDialog *s_dialog = simple_dialog_create("Ping");
  Dialog *dialog = simple_dialog_get_dialog(s_dialog);

  dialog_set_background_color(dialog, GColorCobaltBlue);
  dialog_set_text_color(dialog, GColorWhite);
  dialog_set_text(dialog, "Ping");

  WindowStack *stack = modal_manager_get_window_stack(ModalPriorityGeneric);
  simple_dialog_push(s_dialog, stack);
}

void ping_protocol_msg_callback(CommSession *session, const uint8_t* data, size_t length) {
  PingMsgV1 *ping = (PingMsgV1 *)data;
  switch (ping->hdr.cmd) {
  case 0:
  {
    if (length != sizeof(PingMsgV1) && length != sizeof(PingMsgV2) /* idle boolean is optional */) {
      PBL_LOG_ERR("Invalid Ping, l=%u", length);
      return;
    }

    // Ping message
    uint32_t cookie = ntohl(ping->hdr.cookie);
    PBL_LOG_DBG("Ping c=%"PRIu32"", cookie);
    launcher_task_add_callback(prv_push_window, NULL);

    // Send the pong response
    PongMsg pong = {
      .hdr = {
        .cmd = 1,
        .cookie = htonl(cookie)
      }
    };
    comm_session_send_data(session, PING_ENDPOINT, (uint8_t *)&pong, sizeof(pong), COMM_SESSION_DEFAULT_TIMEOUT);
    break;
  }

  case 1:
    if (length != sizeof(PongMsg)) {
      PBL_LOG_ERR("Invalid Pong, l=%u", length);
      return;
    }

    PongMsg *pong = (PongMsg *)data;
    PBL_LOG_DBG("Pong c=%"PRIu32, ntohl(pong->hdr.cookie));
    break;

  default:
    PBL_LOG_ERR("Invalid message received. First byte is %u", ping->hdr.cmd);
    break;
  }
}


// Serial Commands
//////////////////////////////////////////////////////////////////////
void command_ping_send(void) {
  // Override last send time
  s_last_send_time = 0;
  ping_send_if_due();
}

