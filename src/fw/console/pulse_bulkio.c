/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "console/pulse_bulkio_domain_handler.h"
#include "console/pulse_protocol_impl.h"

#include <stdint.h>

#include "kernel/pbl_malloc.h"
#include "pbl/services/system_task.h"
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/crc32.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"

// Defines how many PULSE file descriptors may be open concurrently
// This is shared across all supported domains
#define MAX_PULSE_FDS 3

#define BULKIO_CMD_DOMAIN_OPEN (1)
#define BULKIO_CMD_DOMAIN_CLOSE (2)
#define BULKIO_CMD_DOMAIN_READ (3)
#define BULKIO_CMD_DOMAIN_WRITE (4)
#define BULKIO_CMD_DOMAIN_CRC (5)
#define BULKIO_CMD_DOMAIN_STAT (6)
#define BULKIO_CMD_DOMAIN_ERASE (7)

#define BULKIO_RESP_DOMAIN_OPEN (128)
#define BULKIO_RESP_DOMAIN_CLOSE (129)
#define BULKIO_RESP_DOMAIN_READ (130)
#define BULKIO_RESP_DOMAIN_WRITE (131)
#define BULKIO_RESP_DOMAIN_CRC (132)
#define BULKIO_RESP_DOMAIN_STAT (133)
#define BULKIO_RESP_DOMAIN_ERASE (134)

#define BULKIO_RESP_MALFORMED_CMD (192)
#define BULKIO_RESP_INTERNAL_ERROR (193)

typedef struct PACKED Command {
  uint8_t opcode;
  union {
    uint8_t fd;
    struct PACKED OpenCommand {
      uint8_t domain;
      uint8_t data[0];
    } open;
    struct PACKED CloseCommand {
      uint8_t fd;
    } close;
    struct PACKED ReadCommand {
      uint8_t fd;
      uint32_t address;
      uint32_t length;
    } read;
    struct PACKED WriteCommand {
      uint8_t fd;
      uint32_t address;
      uint8_t data[0];
    } write;
    struct PACKED CRCCommand {
      uint8_t fd;
      uint32_t address;
      uint32_t length;
    } crc;
    struct PACKED StatCommand {
      uint8_t fd;
    } stat;
    struct PACKED EraseCommand {
      uint8_t domain;
      uint8_t cookie;
      uint8_t data[0];
    } erase;
  };
} Command;

typedef struct PACKED OpenResponse {
  uint8_t opcode;
  uint8_t fd;
} OpenResponse;

typedef struct PACKED CloseResponse {
  uint8_t opcode;
  uint8_t fd;
} CloseResponse;

typedef struct PACKED ReadResponse {
  uint8_t opcode;
  uint8_t fd;
  uint32_t offset;
  uint8_t data[0];
} ReadResponse;

typedef struct PACKED WriteResponse {
  uint8_t opcode;
  uint8_t fd;
  uint32_t address;
  uint32_t length;
} WriteResponse;

typedef struct PACKED CRCResponse {
  uint8_t opcode;
  uint8_t fd;
  uint32_t address;
  uint32_t length;
  uint32_t crc;
} CRCResponse;

typedef struct PACKED StatResponse {
  uint8_t opcode;
  uint8_t fd;
  uint8_t data[0];
} StatResponse;

typedef struct PACKED EraseResponse {
  uint8_t opcode;
  uint8_t domain;
  uint8_t cookie;
  int8_t status;
} EraseResponse;

typedef struct PACKED InternalErrorResponse {
  uint8_t opcode;
  int32_t status_code;
  uint8_t bad_command[0];
} InternalErrorResponse;

#define REGISTER_BULKIO_HANDLER(domain_type, domain_id, vtable) \
    extern PulseBulkIODomainHandler vtable;
#include "pulse_bulkio_handler.def"
#undef REGISTER_BULKIO_HANDLER

static PulseBulkIODomainHandler * const s_domain_handlers[] = {
#define REGISTER_BULKIO_HANDLER(domain_type, domain_id, vtable) \
    [PulseBulkIODomainType_ ## domain_type] = &vtable,
#include "pulse_bulkio_handler.def"
#undef REGISTER_BULKIO_HANDLER
};

#define NUM_DOMAIN_HANDLERS ARRAY_LENGTH(s_domain_handlers)

typedef struct ReadTransferState {
  uint32_t offset;
  uint32_t bytes_left;
} ReadTransferState;

typedef struct PulseTransferFD {
  // impl == NULL means that the FD is free
  PulseBulkIODomainHandler *impl;
  void *domain_state;
  ReadTransferState transfer_state;
} PulseTransferFD;

typedef struct BulkIOPacketCallbackData {
  size_t length;
  uint8_t packet[0];
} BulkIOPacketCallbackData;

static PulseTransferFD s_transfer_fds[MAX_PULSE_FDS];

static void prv_respond_malformed_command(void *cmd, size_t length,
                                     const char *message) {
  uint8_t *resp = pulse_reliable_send_begin(PULSE2_BULKIO_PROTOCOL);
  resp[0] = BULKIO_RESP_MALFORMED_CMD;

  size_t message_len = strlen(message) + 1;
  memcpy(resp + 1, message, message_len);

  size_t response_len =  sizeof(*resp) + message_len;
  size_t command_len = MIN(length, PULSE_MAX_SEND_SIZE - response_len);

  memcpy(resp + response_len, cmd, command_len);
  response_len += command_len;

  pulse_reliable_send(resp, response_len);
}

static void prv_respond_internal_error(Command *cmd, size_t length,
                                          status_t status_code) {
  InternalErrorResponse *resp = pulse_reliable_send_begin(
      PULSE2_BULKIO_PROTOCOL);
  resp->opcode = BULKIO_RESP_INTERNAL_ERROR;
  resp->status_code = status_code;

  size_t response_len = sizeof(*resp);
  size_t command_len = MIN(length, PULSE_MAX_SEND_SIZE - response_len);
  response_len += command_len;

  memcpy(resp->bad_command, cmd, command_len);
  pulse_reliable_send(resp, response_len);
}

static int prv_get_fresh_fd(PulseBulkIODomainHandler *domain_handler, PulseTransferFD **fd) {
  for (int i=0; i < MAX_PULSE_FDS; ++i) {
    if (s_transfer_fds[i].impl == NULL) {
      s_transfer_fds[i] = (PulseTransferFD) {
        .impl = domain_handler,
        .domain_state = NULL,
        .transfer_state = { 0 }
      };
      *fd = &s_transfer_fds[i];
      return i;
    }
  }
  return -1;
}

static void prv_free_fd(int fd) {
  s_transfer_fds[fd].impl = NULL;
}

PulseTransferFD* prv_get_fd(Command *cmd, size_t length) {
  int fd = cmd->fd;
  PulseTransferFD *pulse_fd = &s_transfer_fds[fd];
  if (fd >= 0 && fd < MAX_PULSE_FDS && pulse_fd && pulse_fd->impl) {
    return pulse_fd;
  } else {
    // Invalid, closed or out of range FD
    prv_respond_internal_error(cmd, length, E_INVALID_ARGUMENT);
    return NULL;
  }
}

static PulseBulkIODomainHandler* prv_get_domain_handler(uint8_t domain_id) {
  for (uint8_t i = 0; i < NUM_DOMAIN_HANDLERS; i++) {
    PulseBulkIODomainHandler *domain_handler = s_domain_handlers[i];
    if (domain_handler && domain_handler->id == domain_id) {
      return domain_handler;
    }
  }
  return NULL;
}

static void prv_domain_read_cb(void *data) {
  unsigned int fd_num = (uintptr_t)data;
  PulseTransferFD *pulse_fd = &s_transfer_fds[fd_num];

  const size_t max_read_len = (PULSE_MAX_SEND_SIZE - sizeof(ReadResponse));
  size_t read_len = MIN(pulse_fd->transfer_state.bytes_left, max_read_len);

  ReadResponse *resp = pulse_reliable_send_begin(PULSE2_BULKIO_PROTOCOL);
  resp->opcode = BULKIO_RESP_DOMAIN_READ;
  resp->offset = pulse_fd->transfer_state.offset;

  int ret = pulse_fd->impl->read_proc(resp->data, resp->offset, read_len, pulse_fd->domain_state);
  if (ret > 0) {
    read_len = ret;
    pulse_fd->transfer_state.bytes_left -= read_len;
    pulse_fd->transfer_state.offset += read_len;

    pulse_reliable_send(resp, read_len + sizeof(ReadResponse));

    if (pulse_fd->transfer_state.bytes_left > 0) {
      system_task_add_callback(prv_domain_read_cb, (void*)(uintptr_t)fd_num);
    }
  } else {
    pulse_reliable_send_cancel(resp);

    Command cmd = {
      .opcode = BULKIO_CMD_DOMAIN_READ,
      .read = {
        .fd = fd_num
      }
    };

    prv_respond_internal_error(&cmd, sizeof(cmd), ret);
  }
}

static void prv_handle_open(Command *cmd, size_t length) {
  PulseBulkIODomainHandler *domain_handler = prv_get_domain_handler(cmd->open.domain);

  if (!domain_handler) {
    prv_respond_malformed_command(cmd, length, "Unknown domain");
    return;
  }

  PulseTransferFD *state = NULL;
  int fd = prv_get_fresh_fd(domain_handler, &state);

  if (FAILED(fd)) {
    prv_respond_internal_error(cmd, length, E_OUT_OF_RESOURCES);
    return;
  }

  size_t payload_length = length - sizeof(cmd->opcode) - sizeof(cmd->open);
  status_t ret = state->impl->open_proc(cmd->open.data, payload_length, &state->domain_state);

  if (FAILED(ret)) {
    prv_free_fd(fd);

    if (ret == E_INVALID_ARGUMENT) {
      prv_respond_malformed_command(cmd, length, "Invalid domain data");
    } else {
      prv_respond_internal_error(cmd, length, ret);
    }

    return;
  }

  OpenResponse *resp = pulse_reliable_send_begin(PULSE2_BULKIO_PROTOCOL);
  resp->opcode = BULKIO_RESP_DOMAIN_OPEN;
  resp->fd = fd;
  pulse_reliable_send(resp, sizeof(*resp));
}

static void prv_handle_close(Command *cmd, size_t length) {
  PulseTransferFD *pulse_fd = prv_get_fd(cmd, length);
  if (!pulse_fd) {
    // prv_get_fd has already sent an error response
    return;
  }

  status_t status = pulse_fd->impl->close_proc(pulse_fd->domain_state);

  if (FAILED(status)) {
    prv_respond_internal_error(cmd, length, status);
    return;
  }

  CloseResponse *resp = pulse_reliable_send_begin(PULSE2_BULKIO_PROTOCOL);
  resp->opcode = BULKIO_RESP_DOMAIN_CLOSE;
  resp->fd = cmd->close.fd;
  pulse_reliable_send(resp, sizeof(*resp));

  prv_free_fd(cmd->close.fd);
}

static void prv_handle_read(Command *cmd, size_t length) {
  if (cmd->read.length == 0) {
    prv_respond_internal_error(cmd, length, E_INVALID_ARGUMENT);
    return;
  }

  PulseTransferFD *pulse_fd = prv_get_fd(cmd, length);
  if (!pulse_fd) {
    // prv_get_fd has already sent an error response
    return;
  }

  pulse_fd->transfer_state.offset = cmd->read.address;
  pulse_fd->transfer_state.bytes_left = cmd->read.length;

  system_task_add_callback(prv_domain_read_cb, (void*)(uintptr_t)cmd->fd);
}

static void prv_handle_write(Command *cmd, size_t length) {
  PulseTransferFD *pulse_fd = prv_get_fd(cmd, length);
  if (!pulse_fd) {
    // prv_get_fd has already sent an error response
    return;
  }

  size_t payload_length = length - sizeof(cmd->opcode) - sizeof(cmd->write);
  int ret = pulse_fd->impl->write_proc(cmd->write.data, cmd->write.address, payload_length,
                                       pulse_fd->domain_state);

  if (FAILED(ret)) {
    prv_respond_internal_error(cmd, length, ret);
    return;
  }

  WriteResponse *resp = pulse_reliable_send_begin(PULSE2_BULKIO_PROTOCOL);
  *resp = (WriteResponse) {
    .opcode = BULKIO_RESP_DOMAIN_WRITE,
    .fd = cmd->write.fd,
    .address = cmd->write.address,
    .length = payload_length
  };
  pulse_reliable_send(resp, sizeof(*resp));
}

static void prv_handle_crc(Command *cmd, size_t length) {
  PulseTransferFD *pulse_fd = prv_get_fd(cmd, length);
  if (!pulse_fd) {
    // prv_get_fd has already sent an error response
    return;
  }

  uint32_t bytes_read = 0;
  const unsigned int chunk_size = 128;
  uint8_t buffer[chunk_size];

  uint32_t crc = crc32(0, NULL, 0);
  while (bytes_read < cmd->crc.length) {
    uint32_t read_len = MIN(cmd->crc.length - bytes_read, chunk_size);
    int ret = pulse_fd->impl->read_proc(buffer, cmd->crc.address+bytes_read, read_len,
                                        pulse_fd->domain_state);

    if (FAILED(ret)) {
      prv_respond_internal_error(cmd, length, E_INTERNAL);
      return;
    }

    bytes_read += ret;
    crc = crc32(crc, buffer, read_len);
  }

  CRCResponse *resp = pulse_reliable_send_begin(PULSE2_BULKIO_PROTOCOL);
  *resp = (CRCResponse) {
    .opcode = BULKIO_RESP_DOMAIN_CRC,
    .fd = cmd->crc.fd,
    .address = cmd->crc.address,
    .length = bytes_read,
    .crc = crc
  };
  pulse_reliable_send(resp, sizeof(*resp));
}

static void prv_handle_stat(Command *cmd, size_t length) {
  PulseTransferFD *pulse_fd = prv_get_fd(cmd, length);
  if (!pulse_fd) {
    // prv_get_fd has already sent an error response
    return;
  }

  StatResponse *resp = pulse_reliable_send_begin(PULSE2_BULKIO_PROTOCOL);
  *resp = (StatResponse) {
    .opcode = BULKIO_RESP_DOMAIN_STAT,
    .fd = cmd->stat.fd
  };
  size_t data_max_len = PULSE_MAX_SEND_SIZE - sizeof(StatResponse);
  int ret = pulse_fd->impl->stat_proc(resp->data, data_max_len, pulse_fd->domain_state);
  if (ret >= 0) {
    pulse_reliable_send(resp, ret + sizeof(*resp));
  } else {
    pulse_reliable_send_cancel(resp);
    prv_respond_internal_error(cmd, length, ret);
  }
}

static void prv_handle_erase(Command *cmd, size_t length) {
  PulseBulkIODomainHandler *domain_handler = prv_get_domain_handler(cmd->erase.domain);

  if (domain_handler) {
    size_t payload_length = length - sizeof(cmd->opcode) - sizeof(cmd->erase);
    status_t ret = domain_handler->erase_proc(cmd->erase.data, payload_length, cmd->erase.cookie);

    if (ret == E_INVALID_ARGUMENT) {
      prv_respond_malformed_command(cmd, length, "Invalid domain data");
      return;
    }

    if (ret == S_TRUE) {
      // Handler will send responses itself, we're done here
      return;
    }

    pulse_bulkio_erase_message_send(cmd->erase.domain, ret, cmd->erase.cookie);
  } else {
    prv_respond_malformed_command(cmd, length, "Unknown domain");
  }
}

void pulse_bulkio_erase_message_send(PulseBulkIODomainType domain_type, status_t status,
                                     uint8_t cookie) {
    EraseResponse *resp = pulse_reliable_send_begin(PULSE2_BULKIO_PROTOCOL);
    resp->opcode = BULKIO_RESP_DOMAIN_ERASE;
    resp->domain = domain_type;
    resp->status = status;
    resp->cookie = cookie;
    pulse_reliable_send(resp, sizeof(*resp));
}

static void prv_handle_packet(void *data) {
  BulkIOPacketCallbackData *callback_data = data;
  Command *cmd = (Command*)&callback_data->packet;
  size_t length = callback_data->length;
  if (length) {
    switch (cmd->opcode) {
      case BULKIO_CMD_DOMAIN_OPEN:
        prv_handle_open(cmd, length);
        break;
      case BULKIO_CMD_DOMAIN_CLOSE:
        prv_handle_close(cmd, length);
        break;
      case BULKIO_CMD_DOMAIN_READ:
        prv_handle_read(cmd, length);
        break;
      case BULKIO_CMD_DOMAIN_WRITE:
        prv_handle_write(cmd, length);
        break;
      case BULKIO_CMD_DOMAIN_CRC:
        prv_handle_crc(cmd, length);
        break;
      case BULKIO_CMD_DOMAIN_STAT:
        prv_handle_stat(cmd, length);
        break;
      case BULKIO_CMD_DOMAIN_ERASE:
        prv_handle_erase(cmd, length);
        break;
      default:
        prv_respond_malformed_command(cmd, length, "Unknown command opcode");
        break;
    }
  } else {
    prv_respond_malformed_command(cmd, length, "Empty command");
  }

  kernel_free(data);
}

void pulse2_bulkio_packet_handler(void *packet, size_t length) {
  BulkIOPacketCallbackData *data = kernel_malloc_check(length + sizeof(length));
  data->length = length;
  memcpy(&data->packet, packet, length);
  system_task_add_callback(prv_handle_packet, data);
}

void pulse2_bulkio_link_open_handler(void) {
}

void pulse2_bulkio_link_closed_handler(void) {
  for (int i = 0; i < MAX_PULSE_FDS; i++) {
    if (s_transfer_fds[i].impl) {
      PulseTransferFD *pulse_fd = &s_transfer_fds[i];
      pulse_fd->impl->close_proc(pulse_fd->domain_state);
      prv_free_fd(i);
    }
  }
}
