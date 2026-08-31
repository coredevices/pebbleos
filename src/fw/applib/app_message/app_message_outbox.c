/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/applib_malloc.auto.h"
#include "applib/app_message/app_message_internal.h"
#include "applib/app_outbox.h"
#include "process_state/app_state/app_state.h"
#include "syscall/syscall.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/math.h"

static void prv_outbox_prepare(AppMessageCtxOutbox *outbox);

static uint16_t prv_get_next_transaction_id(AppMessageCtxOutbox *outbox) {
  return ++(outbox->transaction_id);
}

static void prv_transition_to_accepting(AppMessageCtxOutbox *outbox) {
  outbox->phase = OUT_ACCEPTING;
  if (outbox->result == APP_MSG_OK) {
    if (outbox->sent_callback) {
      outbox->sent_callback(&outbox->iterator, outbox->user_context);
    }
  } else {
    if (outbox->failed_callback) {
      outbox->failed_callback(&outbox->iterator, outbox->result, outbox->user_context);
    }
  }
}

static void prv_handle_nack_or_ack_timeout(AppMessageCtxOutbox *outbox,
                                           AppMessageResult result) {
  outbox->result = result;
  if (outbox->phase == OUT_AWAITING_REPLY) {
    prv_transition_to_accepting(outbox);
  } else if (outbox->phase == OUT_AWAITING_REPLY_AND_OUTBOX_CALLBACK) {
    outbox->phase = OUT_AWAITING_OUTBOX_CALLBACK;
  } else {
    WTF;
  }
}

static void prv_handle_outbox_error_cb(void *data) {
  AppMessageResult result = (AppMessageResult)(uintptr_t) data;
  _Static_assert(sizeof(result) <= sizeof(data), "AppMessageResult expected to fit in void *");
  AppMessageCtxOutbox *outbox = &app_state_get_app_message_ctx()->outbox;
  if (outbox->phase != OUT_AWAITING_REPLY_AND_OUTBOX_CALLBACK) {
    APP_LOG(LOG_LEVEL_ERROR, "Outbox failure, but unexpected state: %u", outbox->phase);
    return;
  }
  // If app_message_outbox_handle_app_outbox_message_sent() has been called with an error,
  // don't wait for an (N)ACK (it won't ever come), but finish right away:
  outbox->result = result;
  prv_transition_to_accepting(outbox);
}

//! Use sys_current_process_schedule_callback to maximize the stack space available to the
//! app's failed_callback.
static void prv_handle_outbox_error_async(AppMessageResult result) {
  sys_current_process_schedule_callback(prv_handle_outbox_error_cb, (void *)(uintptr_t)result);
}

AppMessageResult app_message_outbox_open(AppMessageCtxOutbox *outbox, size_t size_outbound) {
  const size_t size_maximum = app_message_outbox_size_maximum();
  if (size_outbound > size_maximum) {
    // Truncate if it's more than the max:
    size_outbound = size_maximum;
  } else if (size_outbound == size_maximum) {
    APP_LOG(LOG_LEVEL_INFO, "app_message_open() called with app_message_outbox_size_maximum().");
    APP_LOG(LOG_LEVEL_INFO,
            "This consumes %"PRIu32" bytes of heap memory, potentially more in the future!",
            (uint32_t)size_maximum);
  }
  if (size_outbound == 0) {
    return APP_MSG_OK;
  }

  // Extra space needed by App Message protocol...:
  size_outbound += APP_MSG_HDR_OVRHD_SIZE;

  // ... and extra space header for app outbox message (not counting towards the transmission size):
  outbox->app_outbox_message = applib_zalloc(sizeof(AppMessageAppOutboxData) + size_outbound);
  if (outbox->app_outbox_message == NULL) {
    return APP_MSG_OUT_OF_MEMORY;
  }
  outbox->transmission_size_limit = size_outbound;
  outbox->transaction_id = 0;
  prv_outbox_prepare(outbox);

  outbox->phase = OUT_ACCEPTING;

  return APP_MSG_OK;
}

static void prv_outbox_prepare(AppMessageCtxOutbox *outbox) {
  AppMessagePush *push = (AppMessagePush *)outbox->app_outbox_message->payload;
  dict_write_begin(&outbox->iterator,
                   (uint8_t *)&push->dictionary,
                   outbox->transmission_size_limit - APP_MSG_HDR_OVRHD_SIZE);
}

static void prv_stop_timer(AppMessageCtxOutbox *outbox) {
  if (outbox->ack_nack_timer) {
    app_timer_cancel(outbox->ack_nack_timer);
    outbox->ack_nack_timer = NULL;
  }
}

void app_message_outbox_close(AppMessageCtxOutbox *outbox) {
  // Verify outbox phase.
  if (outbox->phase == OUT_CLOSED) {
    return;
  }

  // Cancel any outstanding timer.
  prv_stop_timer(outbox);

  outbox->transmission_size_limit = 0;
  applib_free(outbox->app_outbox_message);
  outbox->app_outbox_message = NULL;

  // Finish by moving to the next phase.
  outbox->phase = OUT_CLOSED;
}

static void prv_throttle(AppMessageCtxOutbox *outbox) {
  if (outbox->not_ready_throttle_ms == 0) {
    outbox->not_ready_throttle_ms = 1;
  } else {
    outbox->not_ready_throttle_ms = MIN(outbox->not_ready_throttle_ms * 2, 100 /*ms*/);
  }
  sys_psleep(outbox->not_ready_throttle_ms);
}

static bool prv_is_message_pending(AppMessagePhaseOut phase) {
  return (phase == OUT_AWAITING_REPLY_AND_OUTBOX_CALLBACK ||
          phase == OUT_AWAITING_REPLY ||
          phase == OUT_AWAITING_OUTBOX_CALLBACK);
}

static bool prv_is_awaiting_ack(AppMessagePhaseOut phase) {
  return (phase == OUT_AWAITING_REPLY_AND_OUTBOX_CALLBACK ||
          phase == OUT_AWAITING_REPLY);
}

AppMessageResult app_message_outbox_begin(DictionaryIterator **iterator) {
  AppMessageCtxOutbox *outbox = &app_state_get_app_message_ctx()->outbox;
  if (iterator == NULL) {
    return APP_MSG_INVALID_ARGS;
  }

  AppMessagePhaseOut phase = outbox->phase;
  *iterator = NULL;
  if (prv_is_message_pending(phase)) {
    PBL_LOG_ERR("Can't call app_message_outbox_begin() now, wait for sent_callback!");

    // See https://pebbletechnology.atlassian.net/browse/PBL-10146
    // Workaround for apps that sit in a while() loop waiting on app_message_outbox_begin().
    // Sleep a little longer each time we get a consecutive poll that returns failure.
    prv_throttle(outbox);

    return APP_MSG_BUSY;
  } else if (phase == OUT_WRITING) {
    PBL_LOG_ERR("Must call app_message_outbox_send() before calling app_message_outbox_begin() again!");
    return APP_MSG_INVALID_STATE;
  } else if (phase == OUT_CLOSED) {
    PBL_LOG_ERR("Must call app_message_open() before calling app_message_outbox_begin()!");
    return APP_MSG_INVALID_STATE;
  }

  // Reset the send state (dictionary, counters, etc.)
  // We do this here, as this function is only called when we begin a new outbox,
  // so the state should always be clean when we return successfully.
  prv_outbox_prepare(outbox);
  *iterator = &outbox->iterator;
  outbox->phase = OUT_WRITING;
  outbox->result = APP_MSG_OK;

  return APP_MSG_OK;
}

static void ack_nack_timer_callback(void *data) {
  AppMessageCtxOutbox *outbox = &app_state_get_app_message_ctx()->outbox;
  outbox->ack_nack_timer = NULL;
  if (!prv_is_awaiting_ack(outbox->phase)) {
    // Reply was received and handled in the mean time, or app message was closed.
    return;
  }
  prv_handle_nack_or_ack_timeout(outbox, APP_MSG_SEND_TIMEOUT);
}

void app_message_outbox_handle_app_outbox_message_sent(AppOutboxStatus status, void *cb_ctx) {
  AppMessageCtxOutbox *outbox = &app_state_get_app_message_ctx()->outbox;

  AppMessageSenderError e = (AppMessageSenderError)status;
  if (e != AppMessageSenderErrorSuccess) {
    if (e != AppMessageSenderErrorDisconnected) {
      PBL_LOG_ERR("App message corrupted outbox? %"PRIu8, (uint8_t)e);
    }

    // Sleep a bit to prevent apps that hammer app_message_outbox_begin() when disconnected to
    // become battery hogs:
    prv_throttle(outbox);

    prv_stop_timer(outbox);

    // Just report any error as "not connected" to the app.
    prv_handle_outbox_error_async(APP_MSG_NOT_CONNECTED);
  } else {
    // Only stop throttling if outbox message was consumed successfully:
    outbox->not_ready_throttle_ms = 0;

    if (outbox->phase == OUT_AWAITING_REPLY_AND_OUTBOX_CALLBACK) {
      outbox->phase = OUT_AWAITING_REPLY;
      return;
    }

    if (outbox->phase == OUT_AWAITING_OUTBOX_CALLBACK) {
      prv_transition_to_accepting(outbox);
      return;
    }
  }
}

AppMessageResult app_message_outbox_send(void) {
  AppMessageCtxOutbox *outbox = &app_state_get_app_message_ctx()->outbox;
  if (prv_is_message_pending(outbox->phase)) {
    PBL_LOG_ERR("Can't call app_message_outbox_send() now, wait for sent_callback!");
    return APP_MSG_BUSY;
  }
  if (outbox->phase != OUT_WRITING) {
    return APP_MSG_INVALID_STATE;
  }

  const size_t transmission_size = dict_write_end(&outbox->iterator) + APP_MSG_HDR_OVRHD_SIZE;
  if (transmission_size > outbox->transmission_size_limit) {
    return APP_MSG_BUFFER_OVERFLOW;
  }

  uint8_t transaction_id = prv_get_next_transaction_id(outbox);
  AppMessageAppOutboxData *app_outbox_message = outbox->app_outbox_message;
  AppMessagePush *transmission = (AppMessagePush *)app_outbox_message->payload;
  transmission->header.command = CMD_PUSH;
  transmission->header.transaction_id = transaction_id;
  sys_get_app_uuid(&transmission->uuid);

  outbox->phase = OUT_AWAITING_REPLY_AND_OUTBOX_CALLBACK;

  app_outbox_message->session = NULL;
  app_outbox_message->endpoint_id = APP_MESSAGE_ENDPOINT_ID;

  PBL_ASSERTN(!outbox->ack_nack_timer);
  outbox->ack_nack_timer = app_timer_register(ACK_NACK_TIME_OUT_MS,
                                              ack_nack_timer_callback, NULL);

  app_outbox_send((const uint8_t *)app_outbox_message,
                  sizeof(AppMessageAppOutboxData) + transmission_size,
                  app_message_outbox_handle_app_outbox_message_sent, NULL);

  return APP_MSG_OK;
}

void app_message_out_handle_ack_nack_received(const AppMessageHeader *header) {
  AppMessageCtxOutbox *outbox = &app_state_get_app_message_ctx()->outbox;

  if (!prv_is_awaiting_ack(outbox->phase)) {
    PBL_LOG_ERR("Received (n)ack, but was not expecting one");
    return;
  }

  if (outbox->transaction_id != header->transaction_id) {
    PBL_LOG_ERR("Tx ID mismatch: %"PRIu8" != %"PRIu8,
            outbox->transaction_id, header->transaction_id);
    return;
  }

  prv_stop_timer(outbox);

  if (header->command == CMD_NACK) {
    prv_handle_nack_or_ack_timeout(outbox, APP_MSG_SEND_REJECTED);
    return;
  }

  if (outbox->phase == OUT_AWAITING_REPLY_AND_OUTBOX_CALLBACK) {
    outbox->phase = OUT_AWAITING_OUTBOX_CALLBACK;
    return;
  }
  // phase == OUT_AWAITING_REPLY, because of !prv_is_awaiting_ack() check above.
  prv_transition_to_accepting(outbox);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Unit test interfaces

AppTimer *app_message_outbox_get_ack_nack_timer(void) {
  AppMessageCtxOutbox *outbox = &app_state_get_app_message_ctx()->outbox;
  return outbox ? outbox->ack_nack_timer : NULL;
}

