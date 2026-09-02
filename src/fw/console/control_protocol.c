/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "console/control_protocol.h"
#include "console/control_protocol_impl.h"

#include "console/pulse2_transport_impl.h"
#include "kernel/events.h"
#include "kernel/util/sleep.h"
#include "pbl/services/new_timer/new_timer.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include <pbl/util/math.h>
#include <util/net.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MAX_CONFIGURE (10)
#define MAX_TERMINATE (2)
#define RESTART_TIMEOUT_MS (150)

#define LCP_HEADER_LEN (sizeof(LCPPacket))


static void prv_on_timeout(void *context);

static void prv_start_timer(PPPControlProtocol *this) {
  PBL_ASSERTN(this->state->restart_timer != TIMER_INVALID_ID);
  new_timer_start(this->state->restart_timer, RESTART_TIMEOUT_MS,
                  prv_on_timeout, (void *)this, 0);
}

static void prv_stop_timer(PPPControlProtocol *this) {
  new_timer_stop(this->state->restart_timer);
}

static void prv_transition_to(PPPControlProtocol *this,
                              enum LinkState nextstate) {
  if (nextstate == LinkState_Initial ||
      nextstate == LinkState_Starting ||
      nextstate == LinkState_Closed ||
      nextstate == LinkState_Stopped ||
      nextstate == LinkState_Opened) {
    prv_stop_timer(this);
  }

  if (nextstate == LinkState_Opened &&
      this->state->link_state != LinkState_Opened) {
    this->on_this_layer_up(this);
  }
  if (this->state->link_state == LinkState_Opened &&
      nextstate != LinkState_Opened) {
    this->on_this_layer_down(this);
  }

  this->state->link_state = nextstate;
}

static void prv_send_configure_request(PPPControlProtocol *this) {
  this->state->restart_count--;
  prv_start_timer(this);
  // Don't try to be fancy about changing the request identifier only when
  // necessary; keep it simple and increment it for every request sent.
  uint8_t id = this->state->last_configure_request_id + 1;
  this->state->last_configure_request_id = id;

  struct LCPPacket *request = pulse_link_send_begin(this->protocol_number);
  *request = (struct LCPPacket) {
    .code = ControlCode_ConfigureRequest,
    .identifier = id,
    .length = hton16(LCP_HEADER_LEN),
  };
  pulse_link_send(request, LCP_HEADER_LEN);
}

static void prv_send_configure_ack(PPPControlProtocol *this,
                                   struct LCPPacket *triggering_packet) {
  if (ntoh16(triggering_packet->length) > pulse_link_max_send_size()) {
    // Too big to send and truncation will corrupt the packet.
    PBL_LOG_ERR("Configure-Request too large to Ack");
    return;
  }
  struct LCPPacket *packet = pulse_link_send_begin(this->protocol_number);
  memcpy(packet, triggering_packet, ntoh16(triggering_packet->length));
  packet->code = ControlCode_ConfigureAck;
  pulse_link_send(packet, ntoh16(triggering_packet->length));
}

static void prv_send_configure_reject(PPPControlProtocol *this,
                                      struct LCPPacket *bad_packet) {
  if (ntoh16(bad_packet->length) > pulse_link_max_send_size()) {
    // Too big to send and truncation will corrupt the packet.
    // There isn't really anything we can do.
    PBL_LOG_ERR("Configure-Request too large to Reject");
    return;
  }
  struct LCPPacket *packet = pulse_link_send_begin(this->protocol_number);
  memcpy(packet, bad_packet, ntoh16(bad_packet->length));
  packet->code = ControlCode_ConfigureReject;
  pulse_link_send(packet, ntoh16(bad_packet->length));
}

static void prv_send_terminate_request(PPPControlProtocol *this) {
  this->state->restart_count--;
  prv_start_timer(this);
  uint8_t id = this->state->next_terminate_id++;
  struct LCPPacket *packet = pulse_link_send_begin(this->protocol_number);
  *packet = (struct LCPPacket) {
    .code = ControlCode_TerminateRequest,
    .identifier = id,
    .length = hton16(LCP_HEADER_LEN),
  };
  pulse_link_send(packet, LCP_HEADER_LEN);
}

static void prv_send_terminate_ack(PPPControlProtocol *this, int identifier) {
  if (identifier < 0) {  // Not in response to a Terminate-Request
    // Pick an arbitrary identifier to send in ack
    identifier = this->state->next_terminate_id++;
  } else {
    // Update the next-terminate-id so that the next ack sent not in response to
    // a Terminate-Request does not look like a retransmission.
    this->state->next_terminate_id = identifier + 1;
  }
  struct LCPPacket *packet = pulse_link_send_begin(this->protocol_number);
  *packet = (struct LCPPacket) {
    .code = ControlCode_TerminateAck,
    .identifier = identifier,
    .length = hton16(LCP_HEADER_LEN),
  };
  pulse_link_send(packet, LCP_HEADER_LEN);
}

static void prv_send_code_reject(PPPControlProtocol *this,
                                 struct LCPPacket *bad_packet) {
  struct LCPPacket *packet = pulse_link_send_begin(this->protocol_number);
  packet->code = ControlCode_CodeReject;
  packet->identifier = this->state->next_code_reject_id++;
  size_t body_len = MIN(ntoh16(bad_packet->length),
                        pulse_link_max_send_size() - LCP_HEADER_LEN);
  memcpy(packet->data, bad_packet, body_len);
  pulse_link_send(packet, LCP_HEADER_LEN + body_len);
}

static void prv_on_timeout(void *context) {
  PPPControlProtocol *this = context;
  pbl_mutex_lock(&this->state->lock, PBL_FOREVER);
  if (this->state->restart_count > 0) {  // TO+
    switch (this->state->link_state) {
      case LinkState_Closing:
      case LinkState_Stopping:
        prv_send_terminate_request(this);
        break;
      case LinkState_RequestSent:
      case LinkState_AckReceived:
      case LinkState_AckSent:
        prv_send_configure_request(this);
        if (this->state->link_state == LinkState_AckReceived) {
          prv_transition_to(this, LinkState_RequestSent);
        }
        break;
      default:
        break;
    }
  } else {  // TO-
    switch (this->state->link_state) {
      case LinkState_Stopping:
      case LinkState_RequestSent:
      case LinkState_AckReceived:
      case LinkState_AckSent:
        prv_transition_to(this, LinkState_Stopped);
        break;
      case LinkState_Closing:
        prv_transition_to(this, LinkState_Closed);
        break;
      default:
        break;
    }
  }
  pbl_mutex_unlock(&this->state->lock);
}

static bool prv_handle_configure_request(PPPControlProtocol *this,
                                         struct LCPPacket *packet) {
  if (ntoh16(packet->length) == LCP_HEADER_LEN) {  // The request has no options
    prv_send_configure_ack(this, packet);
    return true;
  } else {
    // Packet has options but we don't support any options yet.
    prv_send_configure_reject(this, packet);
    return false;
  }
}

static void prv_on_configure_request(PPPControlProtocol *this,
                                     struct LCPPacket *packet) {
  switch (this->state->link_state) {
    case LinkState_Closing:
    case LinkState_Stopping:
      // Do nothing
      break;
    case LinkState_Closed:
      prv_send_terminate_ack(this, -1);
      break;
    case LinkState_Stopped:
      this->state->restart_count = MAX_CONFIGURE;
      // fallthrough
    case LinkState_Opened:
      prv_send_configure_request(this);
      // fallthrough
    case LinkState_RequestSent:
    case LinkState_AckSent:
      if (prv_handle_configure_request(this, packet)) {
        prv_transition_to(this, LinkState_AckSent);
      } else {
        prv_transition_to(this, LinkState_RequestSent);
      }
      break;
    case LinkState_AckReceived:
      if (prv_handle_configure_request(this, packet)) {
        prv_transition_to(this, LinkState_Opened);
      }
      break;
    default:
      break;
  }
}

static void prv_on_configure_ack(PPPControlProtocol *this,
                                 struct LCPPacket *packet) {
  if (packet->identifier != this->state->last_configure_request_id) {
    // Invalid packet; silently discard
    return;
  }
  if (ntoh16(packet->length) != LCP_HEADER_LEN) {
    // Only configure requests with no options are sent at the moment.
    // If the length is greater than four, there are options in the Ack
    // which means that the Ack'ed options list does not match the
    // options list from the request. The Ack packet is invalid.
    PBL_LOG_WRN("Configure-Ack received with options list which differs from "
            "the sent Configure-Request. Discarding.");
    return;
  }

  switch (this->state->link_state) {
    case LinkState_Closed:
    case LinkState_Stopped:
      prv_send_terminate_ack(this, -1);
      break;
    case LinkState_Closing:
    case LinkState_Stopping:
      // Do nothing
      break;
    case LinkState_RequestSent:
      this->state->restart_count = MAX_CONFIGURE;
      prv_transition_to(this, LinkState_AckReceived);
      break;
    case LinkState_AckReceived:
    case LinkState_Opened:
      PBL_LOG_WRN("Unexpected duplicate Configure-Ack");
      prv_send_configure_request(this);
      prv_transition_to(this, LinkState_RequestSent);
      break;
    case LinkState_AckSent:
      this->state->restart_count = MAX_CONFIGURE;
      prv_transition_to(this, LinkState_Opened);
      break;
    default:
      break;
  }
}

static void prv_handle_nak_or_reject(PPPControlProtocol *this,
                                     struct LCPPacket *packet) {
  // Process nak/rej options
  // respond with new configure request
  // TODO: we don't send options, so no nak/rej is expected yet
}

static void prv_on_configure_nak_or_reject(PPPControlProtocol *this,
                                           struct LCPPacket *packet) {
  if (packet->identifier != this->state->last_configure_request_id) {
    // Invalid packet; silently discard
    return;
  }

  switch (this->state->link_state) {
    case LinkState_Closed:
    case LinkState_Stopped:
      prv_send_terminate_ack(this, -1);
      break;
    case LinkState_Closing:
    case LinkState_Stopping:
      // Do nothing
      break;
    case LinkState_RequestSent:
      this->state->restart_count = MAX_CONFIGURE;
      // fallthrough
    case LinkState_AckReceived:
    case LinkState_Opened:
      PBL_LOG_WRN("Unexpected Configure-Nak/Rej received after Ack");
      prv_handle_nak_or_reject(this, packet);
      prv_transition_to(this, LinkState_RequestSent);
      break;
    case LinkState_AckSent:
      prv_handle_nak_or_reject(this, packet);
      break;
    default:
      break;
  }
}

static void prv_on_terminate_request(PPPControlProtocol *this,
                                     struct LCPPacket *packet) {
  if (this->state->link_state == LinkState_AckReceived ||
      this->state->link_state == LinkState_AckSent) {
    prv_transition_to(this, LinkState_RequestSent);
  } else if (this->state->link_state == LinkState_Opened) {
    this->state->restart_count = 0;
    prv_start_timer(this);
    prv_transition_to(this, LinkState_Stopping);
  }
  prv_send_terminate_ack(this, packet->identifier);
}

static void prv_on_terminate_ack(PPPControlProtocol *this,
                                 struct LCPPacket *packet) {
  if (this->state->link_state == LinkState_Closing) {
    prv_transition_to(this, LinkState_Closed);
  } else if (this->state->link_state == LinkState_Stopping) {
    prv_transition_to(this, LinkState_Stopped);
  } else if (this->state->link_state == LinkState_AckReceived) {
    prv_transition_to(this, LinkState_RequestSent);
  } else if (this->state->link_state == LinkState_Opened) {
    PBL_LOG_WRN("Terminate-Ack received on an open connection");
    prv_send_configure_request(this);
    prv_transition_to(this, LinkState_RequestSent);
  }
}

// Protected interface (control_protocol_impl.h)
// =============================================

void ppp_control_protocol_init(PPPControlProtocol *this) {
  *this->state = (PPPControlProtocolState) {
    .link_state = LinkState_Initial,
    .restart_count = 0,
    .restart_timer = new_timer_create(),
    .last_configure_request_id = -1,
    .next_code_reject_id = 0,
    .next_terminate_id = 0,
  };
  pbl_mutex_init(&this->state->lock);
}

// Public interface (control_protocol.h)
// =====================================

void ppp_control_protocol_lower_layer_is_up(PPPControlProtocol *this) {
  pbl_mutex_lock(&this->state->lock, PBL_FOREVER);
  if (this->state->link_state == LinkState_Initial) {
    prv_transition_to(this, LinkState_Closed);
  } else if (this->state->link_state == LinkState_Starting) {
    this->state->restart_count = MAX_CONFIGURE;
    prv_send_configure_request(this);
    prv_transition_to(this, LinkState_RequestSent);
  }
  pbl_mutex_unlock(&this->state->lock);
}

void ppp_control_protocol_lower_layer_is_down(PPPControlProtocol *this) {
  pbl_mutex_lock(&this->state->lock, PBL_FOREVER);
  switch (this->state->link_state) {
    case LinkState_Closed:
    case LinkState_Closing:
      prv_transition_to(this, LinkState_Initial);
      break;
    case LinkState_Stopped:
    case LinkState_Stopping:
    case LinkState_RequestSent:
    case LinkState_AckReceived:
    case LinkState_AckSent:
    case LinkState_Opened:
      prv_transition_to(this, LinkState_Starting);
      break;
    default:
      break;
  }
  pbl_mutex_unlock(&this->state->lock);
}

void ppp_control_protocol_open(PPPControlProtocol *this) {
  pbl_mutex_lock(&this->state->lock, PBL_FOREVER);
  if (this->state->link_state == LinkState_Initial) {
    prv_transition_to(this, LinkState_Starting);
  } else if (this->state->link_state == LinkState_Closed) {
    this->state->restart_count = MAX_CONFIGURE;
    prv_send_configure_request(this);
    prv_transition_to(this, LinkState_RequestSent);
  } else if (this->state->link_state == LinkState_Stopping) {
    prv_transition_to(this, LinkState_Closing);
  }
  pbl_mutex_unlock(&this->state->lock);
}

void ppp_control_protocol_close(PPPControlProtocol *this,
                                PPPCPCloseWait wait) {
  pbl_mutex_lock(&this->state->lock, PBL_FOREVER);
  switch (this->state->link_state) {
    case LinkState_Starting:
      prv_transition_to(this, LinkState_Initial);
      break;
    case LinkState_Stopped:
      prv_transition_to(this, LinkState_Closed);
      break;
    case LinkState_RequestSent:
    case LinkState_AckReceived:
    case LinkState_AckSent:
    case LinkState_Opened:
      this->state->restart_count = MAX_TERMINATE;
      prv_send_terminate_request(this);
      // fallthrough
    case LinkState_Stopping:
      prv_transition_to(this, LinkState_Closing);
      break;
    default:
      break;
  }
  pbl_mutex_unlock(&this->state->lock);

  if (wait == PPPCPCloseWait_WaitForClosed) {
    // Poll for the state machine to finish closing.
    while (1) {
      pbl_mutex_lock(&this->state->lock, PBL_FOREVER);
      LinkState state = this->state->link_state;
      pbl_mutex_unlock(&this->state->lock);
      if (state == LinkState_Initial || state == LinkState_Closed) {
        return;
      }
      psleep(2);
    }
  }
}

void ppp_control_protocol_handle_incoming_packet(
    PPPControlProtocol *this, void *raw_packet, size_t length) {
  pbl_mutex_lock(&this->state->lock, PBL_FOREVER);
  if (this->state->link_state == LinkState_Initial ||
      this->state->link_state == LinkState_Starting) {
    // No packets should be received while the lower layer is down;
    // silently discard.
    goto done;
  }

  struct LCPPacket *packet = raw_packet;
  if (length < sizeof(*packet) ||
      ntoh16(packet->length) < sizeof(*packet) ||
      length < ntoh16(packet->length)) {
    // Invalid packet; silently discard
    goto done;
  }

  switch (packet->code) {
    case ControlCode_ConfigureRequest:
      prv_on_configure_request(this, packet);
      break;
    case ControlCode_ConfigureAck:
      prv_on_configure_ack(this, packet);
      break;
    case ControlCode_ConfigureNak:
    case ControlCode_ConfigureReject:
      prv_on_configure_nak_or_reject(this, packet);
      break;
    case ControlCode_TerminateRequest:
      prv_on_terminate_request(this, packet);
      break;
    case ControlCode_TerminateAck:
      prv_on_terminate_ack(this, packet);
      break;
    case ControlCode_CodeReject:
      this->on_receive_code_reject(this, packet);
      break;
    default:
      if (!this->on_receive_unrecognized_code ||
          !this->on_receive_unrecognized_code(this, packet)) {
        prv_send_code_reject(this, packet);
      }
      break;
  }

done:
  pbl_mutex_unlock(&this->state->lock);
}
