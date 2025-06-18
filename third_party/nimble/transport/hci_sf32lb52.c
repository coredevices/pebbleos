/*
 * Copyright 2025 Core Devices LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifdef NIMBLE_HCI_SF32LB52_TRACE_BINARY
#include <board/board.h>
#include <drivers/uart.h>
#endif // NIMBLE_HCI_SF32LB52_TRACE_BINARY

#include <kernel/pebble_tasks.h>
#include <system/hexdump.h>
#include <system/logging.h>
#include <system/passert.h>

// NOTE: transport.h needs os_mbuf.h to be included first
// clang-format off
#include <os/os_mbuf.h>
// clang-format on
#include <nimble/hci_common.h>
#include <nimble/transport.h>
#include <nimble/transport/hci_h4.h>
#include <nimble/transport_impl.h>
#include <os/os_mempool.h>

#include <ipc_queue.h>

#define IPC_TIMEOUT_TICKS 10

#define HCI_TRACE_HEADER_LEN 16

#define H4TL_PACKET_HOST 0x61
#define H4TL_PACKET_CTRL 0x62

#define IO_MB_CH (0)
#define TX_BUF_SIZE HCPU2LCPU_MB_CH1_BUF_SIZE
#define TX_BUF_ADDR HCPU2LCPU_MB_CH1_BUF_START_ADDR
#define TX_BUF_ADDR_ALIAS HCPU_ADDR_2_LCPU_ADDR(HCPU2LCPU_MB_CH1_BUF_START_ADDR);
#define RX_BUF_ADDR LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_START_ADDR);
#define RX_BUF_REV_B_ADDR LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_REV_B_START_ADDR);

static TaskHandle_t s_hci_task_handle;
static SemaphoreHandle_t s_ipc_data_ready;
static struct hci_h4_sm s_hci_h4sm;
static ipc_queue_handle_t s_ipc_port;

#ifdef NIMBLE_HCI_SF32LB52_TRACE_BINARY
static uint16_t s_hci_trace_seq;
#endif

// TODO(SF32LB52): adjust according to NimBLE configuration?
static uint8_t s_hci_acl[256];
static uint8_t s_hci_cmd[256];

extern void lcpu_power_on(void);

#if defined(NIMBLE_HCI_SF32LB52_TRACE_LOG)
void prv_hci_trace(uint8_t type, const uint8_t *data, uint16_t len, uint8_t h4tl_packet) {
  const char *type_str;

  switch (type) {
    case HCI_H4_CMD:
      type_str = "CMD";
      break;
    case HCI_H4_ACL:
      type_str = "ACL";
      break;
    case HCI_H4_EVT:
      type_str = "EVT";
      break;
    case HCI_H4_ISO:
      type_str = "ISO";
      break;
    default:
      type_str = "UKN";
      break;
  }

  PBL_LOG_D(LOG_DOMAIN_BT_STACK, LOG_LEVEL_DEBUG, "%s, %s %" PRIu16, type_str,
            (h4tl_packet == H4TL_PACKET_HOST) ? "TX" : "RX", len);
  PBL_HEXDUMP_D(LOG_DOMAIN_BT_STACK, LOG_LEVEL_DEBUG, data, len);
}
#elif defined(NIMBLE_HCI_SF32LB52_TRACE_BINARY)
void prv_hci_trace(uint8_t type, const uint8_t *data, uint16_t len, uint8_t h4tl_packet) {
  uint8_t trace_hdr[HCI_TRACE_HEADER_LEN];

  // Magic for Pebble HCI, 'PBTS'
  trace_hdr[0] = 0x50U;
  trace_hdr[1] = 0x42U;
  trace_hdr[2] = 0x54U;
  trace_hdr[3] = 0x53U;
  trace_hdr[4] = 0x06U;
  trace_hdr[5] = 0x01U;
  trace_hdr[6] = (len + 8U) & 0xFFU;
  trace_hdr[7] = (len + 8U) >> 8U;
  trace_hdr[8] = s_hci_trace_seq & 0xFFU;
  trace_hdr[9] = s_hci_trace_seq >> 8U;
  trace_hdr[10] = 0U;
  trace_hdr[11] = 0U;
  trace_hdr[12] = 0U;
  trace_hdr[13] = 0U;
  trace_hdr[14] = h4tl_packet;
  trace_hdr[15] = type;

  s_hci_trace_seq++;

  for (uint8_t i = 0U; i < HCI_TRACE_HEADER_LEN; i++) {
    uart_write_byte(HCI_TRACE_UART, trace_hdr[i]);
  }

  for (uint16_t i = 0U; i < len; i++) {
    uart_write_byte(HCI_TRACE_UART, data[i]);
  }
}
#else
#define prv_hci_trace(type, data, len, h4tl_packet)
#endif

static int32_t prv_ipc_rx_ind(ipc_queue_handle_t handle, size_t size) {
  BaseType_t woken;

  xSemaphoreGiveFromISR(s_ipc_data_ready, &woken);
  portEND_SWITCHING_ISR(woken);

  return 0;
}

static int prv_config_ipc(void) {
  ipc_queue_cfg_t q_cfg;
  int32_t ret;

  q_cfg.qid = IO_MB_CH;
  q_cfg.tx_buf_size = TX_BUF_SIZE;
  q_cfg.tx_buf_addr = TX_BUF_ADDR;
  q_cfg.tx_buf_addr_alias = TX_BUF_ADDR_ALIAS;

  uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
  if (rev_id < HAL_CHIP_REV_ID_A4) {
    q_cfg.rx_buf_addr = RX_BUF_ADDR;
  } else {
    q_cfg.rx_buf_addr = RX_BUF_REV_B_ADDR;
  }

  q_cfg.rx_ind = NULL;
  q_cfg.user_data = 0;

  if (q_cfg.rx_ind == NULL) {
    q_cfg.rx_ind = prv_ipc_rx_ind;
  }

  s_ipc_port = ipc_queue_init(&q_cfg);
  if (s_ipc_port == IPC_QUEUE_INVALID_HANDLE) {
    PBL_LOG_D(LOG_DOMAIN_BT_STACK, LOG_LEVEL_ERROR, "ipc_queue_init failed");
    return -1;
  }

  ret = ipc_queue_open(s_ipc_port);
  if (ret != 0) {
    PBL_LOG_D(LOG_DOMAIN_BT_STACK, LOG_LEVEL_ERROR, "ipc_queue_open failed (%" PRId32 ")", ret);
    return -1;
  }

  NVIC_EnableIRQ(LCPU2HCPU_IRQn);
  NVIC_SetPriority(LCPU2HCPU_IRQn, 5);

  return 0;
}

static int prv_hci_frame_cb(uint8_t pkt_type, void *data) {
  struct ble_hci_ev *ev;
  struct ble_hci_ev_command_complete *cmd_complete;
  struct os_mbuf *om;

  switch (pkt_type) {
  case HCI_H4_EVT:
      ev = data;
      cmd_complete = (void *)ev->data;

      if (ev->opcode == BLE_HCI_EVCODE_COMMAND_COMPLETE) {
        PBL_LOG_D(LOG_DOMAIN_BT_STACK, LOG_LEVEL_DEBUG, "CMD complete %x", cmd_complete->opcode);
        // NOTE: do not confuse NimBLE with SF32LB52 vendor specific command
        if (cmd_complete->opcode == 0xFC11) {
            break;
        }
      }

      prv_hci_trace(pkt_type, data, ev->length + sizeof(*ev), H4TL_PACKET_CTRL);

      return ble_transport_to_hs_evt(data);
    case HCI_H4_ACL:
      om = (struct os_mbuf *)data;

      prv_hci_trace(pkt_type, OS_MBUF_DATA(om, uint8_t *), OS_MBUF_PKTLEN(om), H4TL_PACKET_CTRL);

      return ble_transport_to_hs_acl(data);
    default:
      WTF;
      break;
  }

  return -1;
}

static void prv_hci_task_main(void *unused) {
  uint8_t buf[64];

  while (true) {
    xSemaphoreTake(s_ipc_data_ready, portMAX_DELAY);

    while (true) {
      size_t len;
      
      len = ipc_queue_read(s_ipc_port, buf, sizeof(buf));
      if (len > 0U) {
        while (len > 0U) {
          int consumed_bytes;

          consumed_bytes = hci_h4_sm_rx(&s_hci_h4sm, buf, len);
          len -= consumed_bytes;
        }
      } else {
        break;
      }
    }
  }
}

void ble_transport_ll_init(void) {
  int ret;

#ifdef NIMBLE_HCI_SF32LB52_TRACE_BINARY
  uart_init_tx_only(HCI_TRACE_UART);
  uart_set_baud_rate(HCI_TRACE_UART, 1000000);
#endif

  hci_h4_sm_init(&s_hci_h4sm, &hci_h4_allocs_from_ll, prv_hci_frame_cb);

  s_ipc_data_ready = xSemaphoreCreateBinary();

  TaskParameters_t task_params = {
    .pvTaskCode = prv_hci_task_main,
    .pcName = "NimbleHCI",
    .usStackDepth = 1024 / sizeof(StackType_t),
    .uxPriority = (tskIDLE_PRIORITY + 3) | portPRIVILEGE_BIT,
    .puxStackBuffer = NULL,
  };

  pebble_task_create(PebbleTask_BTHCI, &task_params, &s_hci_task_handle);
  PBL_ASSERTN(s_hci_task_handle);

  ret = prv_config_ipc();
  PBL_ASSERTN(ret == 0);

  lcpu_power_on();
}

/* APIs to be implemented by HS/LL side of transports */
int ble_transport_to_ll_cmd_impl(void *buf) {
  struct ble_hci_cmd *cmd = buf;
  size_t written;

  PBL_ASSERTN((sizeof(*cmd) + cmd->length) <= sizeof(s_hci_cmd) - 1);

  s_hci_cmd[0] = HCI_H4_CMD;
  memcpy(&s_hci_cmd[1], cmd, sizeof(*cmd) + cmd->length);

  prv_hci_trace(HCI_H4_CMD, (uint8_t *)cmd, sizeof(*cmd) + cmd->length, H4TL_PACKET_HOST);

  written = ipc_queue_write(s_ipc_port, s_hci_cmd, 1 + sizeof(*cmd) + cmd->length,
                            IPC_TIMEOUT_TICKS);
  ble_transport_free(buf);

  return (written >= 0U) ? 0 : -1;
}

int ble_transport_to_ll_acl_impl(struct os_mbuf *om) {
  uint8_t *data;
  size_t written;

  PBL_ASSERTN(OS_MBUF_PKTLEN(om) <= sizeof(s_hci_acl) - 1);

  data = OS_MBUF_DATA(om, uint8_t *);

  s_hci_acl[0] = HCI_H4_ACL;
  memcpy(&s_hci_acl[1], data, OS_MBUF_PKTLEN(om));

  prv_hci_trace(HCI_H4_ACL, data, OS_MBUF_PKTLEN(om), H4TL_PACKET_HOST);

  written = ipc_queue_write(s_ipc_port, s_hci_acl, 1 + OS_MBUF_PKTLEN(om), IPC_TIMEOUT_TICKS);
  os_mbuf_free(om);

  return (written >= 0U) ? 0 : -1;
}

int ble_transport_to_ll_iso_impl(struct os_mbuf *om) {
  uint8_t *data;
  size_t written;

  data = OS_MBUF_DATA(om, uint8_t *);
  
  written = ipc_queue_write(s_ipc_port, data, OS_MBUF_PKTLEN(om), IPC_TIMEOUT_TICKS);
  os_mbuf_free(om);

  return (written >= 0U) ? 0 : -1;
}
