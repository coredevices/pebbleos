/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/kernel/mutex.h"
#include "pbl/util/attributes.h"
#include "util/shared_circular_buffer.h"

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#define QEMU_HEADER_SIGNATURE 0xFEED
#define QEMU_FOOTER_SIGNATURE 0xBEEF
#define QEMU_MAX_DATA_LEN     2048


// Every message sent over the QEMU comm channel has the following header. All
// data is set in network byte order. The maximum data len (not including header or footer)
// allowed is QEMU_MAX_DATA_LEN bytes
typedef struct PACKED {
  uint16_t signature;         // QEMU_HEADER_SIGNATURE
  uint16_t protocol;          // one of QemuProtocol
  uint16_t len;               // number of bytes that follow (not including this header or footer)
} QemuCommChannelHdr;

// Every message sent over the QEMU comm channel has the following footer.
typedef struct PACKED {
  uint16_t signature;         // QEMU_FOOTER_SIGNATURE
} QemuCommChannelFooter;


// Incoming message handlers are defined using this structure
typedef void (*QemuMessageCallback)(const uint8_t* data, uint32_t length);
typedef struct {
  uint16_t protocol_id;
  QemuMessageCallback callback;
} QemuMessageHandler;


// Which state our incoming message state machine is in
typedef enum {
  QemuRecvState_WaitingHdrSignatureMSB,
  QemuRecvState_WaitingHdrSignatureLSB,
  QemuRecvState_WaitingHdr,
  QemuRecvState_WaitingData,
  QemuRecvState_WaitingFooter,
} QemuRecvState;


// Structure of our globals
typedef struct {
  bool initialized;
  struct pbl_mutex qemu_comm_lock;
  SharedCircularBuffer isr_buffer;
  SharedCircularBufferClient  isr_buffer_client;

  QemuRecvState recv_state;
  uint8_t prev_byte;
  QemuCommChannelHdr hdr;
  uint8_t *msg_buffer;
  uint16_t msg_buffer_bytes;
  bool callback_pending;
  uint32_t recv_error_count;
  time_t start_recv_packet_time;
} QemuSerialGlobals;


// -----------------------------------------------------------------------------------
// Defines and private structures
#define UART_SERIAL_BAUD_RATE 115200

#define QEMU_ISR_RECV_HIGH_WATER_DELTA (128)
#define QEMU_ISR_RECV_BUFFER_SIZE    (QEMU_MAX_DATA_LEN + QEMU_ISR_RECV_HIGH_WATER_DELTA)
#define QEMU_RECV_PACKET_TIMEOUT_SEC 10       // We have to receive a complete packet within this
                                              // amount of time

#define QEMU_FOOTER_MSB        ((uint8_t)(QEMU_FOOTER_SIGNATURE >> 8))
#define QEMU_FOOTER_LSB        ((uint8_t)(QEMU_FOOTER_SIGNATURE & 0x00FF))
#define QEMU_HEADER_MSB        ((uint8_t)(QEMU_HEADER_SIGNATURE >> 8))
#define QEMU_HEADER_LSB        ((uint8_t)(QEMU_HEADER_SIGNATURE & 0x00FF))


// -----------------------------------------------------------------------------------
void qemu_serial_private_init_state(QemuSerialGlobals *state);

uint8_t *qemu_serial_private_assemble_message(QemuSerialGlobals *state, uint32_t *msg_bytes,
            uint16_t *protocol);
