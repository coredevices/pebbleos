/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/kernel/mutex.h"
#include "pbl/services/new_timer/new_timer.h"
#include <pbl/util/attributes.h>
#include <util/net.h>

#include <stdbool.h>
#include <stdint.h>

typedef struct PACKED LCPPacket {
  uint8_t code;
  uint8_t identifier;
  net16 length;
  char data[];
} LCPPacket;

typedef enum ControlCode {
  ControlCode_ConfigureRequest = 1,
  ControlCode_ConfigureAck = 2,
  ControlCode_ConfigureNak = 3,
  ControlCode_ConfigureReject = 4,
  ControlCode_TerminateRequest = 5,
  ControlCode_TerminateAck = 6,
  ControlCode_CodeReject = 7,
  ControlCode_ProtocolReject = 8,
  ControlCode_EchoRequest = 9,
  ControlCode_EchoReply = 10,
  ControlCode_DiscardRequest = 11,
  ControlCode_Identification = 12,
} ControlCode;

typedef enum LinkState {
  LinkState_Initial,  //!< Lower layer is Down; this layer is Closed
  LinkState_Starting,  //!< Lower layer is Down; this layer is Open
  LinkState_Closed,  //!< Lower layer is Up; this layer is Closed
  LinkState_Stopped,  //!< Waiting passively for a new connection
  LinkState_Closing,  //!< Connection is being terminated before Closed
  LinkState_Stopping,  //!< Connection is being terminated before Stopped
  LinkState_RequestSent,  //!< Configure-Request sent
  LinkState_AckReceived,  //!< Configure-Request sent, Configure-Ack received
  LinkState_AckSent,  //!< Configure-Request and Configure-Ack sent
  LinkState_Opened,
} LinkState;

typedef struct PPPControlProtocolState {
  struct pbl_mutex lock;
  LinkState link_state;
  int restart_count;
  TimerID restart_timer;
  int last_configure_request_id;
  uint8_t next_code_reject_id;
  uint8_t next_terminate_id;
} PPPControlProtocolState;

typedef const struct PPPControlProtocol PPPControlProtocol;
struct PPPControlProtocol {
  PPPControlProtocolState * const state;
  //! Called when the layer is ready to carry traffic.
  void (*const on_this_layer_up)(PPPControlProtocol *this);
  //! Called when the layer is no longe ready to carry traffic.
  void (*const on_this_layer_down)(PPPControlProtocol *this);
  //! Called when a Code-Reject packet is received.
  void (*const on_receive_code_reject)(PPPControlProtocol *this,
                                       LCPPacket *packet);
  //! Called when a packet is received with a code not handled by the
  //! base Control Protocol implementation. May be NULL if no extended
  //! codes are supported by the implementation.
  //!
  //! \return true if the code is handled, false if the code is also
  //!         unknown to the implementation.
  //!
  //! If the code is unknown to the implementation, a Code-Reject
  //! response packet is sent by the base Control Protocol
  //! implementation.
  bool (*const on_receive_unrecognized_code)(PPPControlProtocol *this,
                                             LCPPacket *packet);
  //! PPP Encapsulation protocol number for the control protocol.
  uint16_t protocol_number;
};

//! Initialize the state struct for a PPPControlProtocol
void ppp_control_protocol_init(PPPControlProtocol *this);
