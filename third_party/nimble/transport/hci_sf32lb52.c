/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#ifdef NIMBLE_HCI_SF32LB52_TRACE_BINARY
  #include <board/board.h>
  #include <drivers/uart.h>
#endif // NIMBLE_HCI_SF32LB52_TRACE_BINARY

#include <bf0_hal.h>
#include <kernel/pebble_tasks.h>
#include <pbl/kernel/sem.h>
#include <pbl/kernel/thread.h>
#include <system/hexdump.h>
#include <pbl/logging/logging.h>
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

#define BLE_HCI_EXT_SF32LB52_BLE_READY 0xFC11U

static struct pbl_thread *s_hci_task_handle;
PBL_THREAD_STACK_DEFINE(s_hci_task_stack, 1024);
static PBL_SEM_DEFINE(s_ipc_data_ready, 0, 1);
/* Given by prv_acl_put_signal() whenever an LL-direction ACL mbuf is returned
 * to the transport pool; taken by the HCI RX task when an alloc fails so we
 * unblock as soon as the host has freed something instead of polling.
 */
static PBL_SEM_DEFINE(s_acl_pool_avail, 0, 1);
static struct hci_h4_sm s_hci_h4sm;
static ipc_queue_handle_t s_ipc_port;

static struct os_mbuf *prv_alloc_acl_from_ll(void) {
  struct os_mbuf *om = ble_transport_alloc_acl_from_ll();
  if (om != NULL) {
    return om;
  }

  /* ACL pool exhausted. Block instead of returning NULL (which the H4 state
   * machine would treat as a fatal, unrecoverable framing error). While we
   * wait we stop draining the LCPU->HCPU IPC ring; it fills and backpressures
   * the controller, i.e. transport-level flow control. prv_acl_put_signal()
   * wakes us when the host returns a buffer to the pool; the timeout guards
   * the give-before-wait race and the fact that the from_hs TX path shares
   * mpool_acl, so a freed buffer may be taken before we retry.
   */
  PBL_LOG_D_DBG(LOG_DOMAIN_BT_STACK, "ACL pool empty, waiting for buffer");
  do {
    (void)pbl_sem_take(&s_acl_pool_avail, PBL_MSEC(100));
    om = ble_transport_alloc_acl_from_ll();
  } while (om == NULL);

  return om;
}

static os_error_t prv_acl_put_signal(struct os_mempool_ext *mpe, void *data, void *arg) {
  os_error_t err = os_memblock_put_from_cb(&mpe->mpe_mp, data);
  pbl_sem_give(&s_acl_pool_avail);
  return err;
}

static void *prv_alloc_evt(int discardable) {
  void *buf = ble_transport_alloc_evt(discardable);
  if (!buf) {
    PBL_LOG_ERR("EVT alloc failed (discardable=%d)", discardable);
  }

  return buf;
}

static const struct hci_h4_allocators s_hci_h4_allocs_from_ll = {
  .acl = prv_alloc_acl_from_ll,
  .evt = prv_alloc_evt,
};

#if defined(NIMBLE_HCI_SF32LB52_TRACE_BINARY)
static uint16_t s_hci_trace_seq;
#endif

#if defined(NIMBLE_HCI_SF32LB52_TRACE_BINARY) || defined(NIMBLE_HCI_SF32LB52_TRACE_LOG)
#define MAX_HCI_PKT_SIZE 1024
static uint8_t s_hci_buf[MAX_HCI_PKT_SIZE];
#endif

extern void lcpu_power_on(void);
extern uint8_t lcpu_power_off(void);
extern void lcpu_custom_nvds_config(void);

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

  PBL_LOG_D_DBG(LOG_DOMAIN_BT_STACK, "%s, %s %" PRIu16, type_str,
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

#if defined(NIMBLE_HCI_SF32LB52_TRACE_BINARY) || defined(NIMBLE_HCI_SF32LB52_TRACE_LOG)
void prv_hci_trace_mbuf(uint8_t type, struct os_mbuf *om, uint8_t h4tl_packet)  {
  PBL_ASSERTN(os_mbuf_len(om) < MAX_HCI_PKT_SIZE);
  os_mbuf_copydata(om, 0, os_mbuf_len(om), s_hci_buf);
  prv_hci_trace(type, s_hci_buf, os_mbuf_len(om), h4tl_packet);
}
#else
#define prv_hci_trace_mbuf(type, om, h4tl_packet)
#endif


static int32_t prv_ipc_rx_ind(ipc_queue_handle_t handle, size_t size) {
  pbl_sem_give(&s_ipc_data_ready);

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
    PBL_LOG_D_ERR(LOG_DOMAIN_BT_STACK, "ipc_queue_init failed");
    return -1;
  }

  ret = ipc_queue_open(s_ipc_port);
  if (ret != 0) {
    PBL_LOG_D_ERR(LOG_DOMAIN_BT_STACK, "ipc_queue_open failed (%" PRId32 ")", ret);
    return -1;
  }

  NVIC_EnableIRQ(LCPU2HCPU_IRQn);
  NVIC_SetPriority(LCPU2HCPU_IRQn, 5);

  return 0;
}

static int prv_hci_frame_cb(uint8_t pkt_type, void *data) {
  struct ble_hci_ev *ev;
  struct ble_hci_ev_command_complete *cmd_complete;

  switch (pkt_type) {
  case HCI_H4_EVT:
    ev = data;
    cmd_complete = (void *)ev->data;

    if (ev->opcode == BLE_HCI_EVCODE_COMMAND_COMPLETE) {
      PBL_LOG_D_DBG(LOG_DOMAIN_BT_STACK, "CMD complete %x", cmd_complete->opcode);
      // NOTE: do not confuse NimBLE with SF32LB52 vendor specific command
      if (cmd_complete->opcode == BLE_HCI_EXT_SF32LB52_BLE_READY) {
        break;
      }
    }

    prv_hci_trace(pkt_type, data, ev->length + sizeof(*ev), H4TL_PACKET_CTRL);

    return ble_transport_to_hs_evt(data);
  case HCI_H4_ACL:
    prv_hci_trace(pkt_type, OS_MBUF_DATA((struct os_mbuf *)data, uint8_t *),
                  OS_MBUF_PKTLEN((struct os_mbuf *)data), H4TL_PACKET_CTRL);

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
    pbl_sem_take(&s_ipc_data_ready, PBL_FOREVER);

    while (true) {
      size_t len;

      len = ipc_queue_read(s_ipc_port, buf, sizeof(buf));
      if (len > 0U) {
        uint8_t *pbuf = buf;
        while (len > 0U) {
          int consumed_bytes;

          /* prv_alloc_acl_from_ll() blocks until a buffer is available, so the
           * H4 state machine never reports an OOM on the ACL path; any
           * non-positive return here is unexpected.
           */
          consumed_bytes = hci_h4_sm_rx(&s_hci_h4sm, pbuf, len);
          if (consumed_bytes <= 0) {
            PBL_LOG_D_ERR(LOG_DOMAIN_BT_STACK, "hci_h4_sm_rx returned %d", consumed_bytes);
            break;
          }
          len -= consumed_bytes;
          pbuf += consumed_bytes;
        }
      } else {
        break;
      }
    }
  }
}

void ble_transport_ll_reinit(void) {
  int ret;

  hci_h4_sm_init(&s_hci_h4sm, &s_hci_h4_allocs_from_ll, prv_hci_frame_cb);

  ret = prv_config_ipc();
  PBL_ASSERTN(ret == 0);

  lcpu_custom_nvds_config();
  lcpu_power_on();
}

void ble_transport_ll_init(void) {
#ifdef NIMBLE_HCI_SF32LB52_TRACE_BINARY
  uart_init_tx_only(HCI_TRACE_UART);
  uart_set_baud_rate(HCI_TRACE_UART, 1000000);
#endif

  ble_transport_ll_reinit();

  ble_transport_register_put_acl_from_ll_cb(prv_acl_put_signal);

  struct pbl_thread_attr attr = {
    .name = "NimbleHCI",
    .entry = prv_hci_task_main,
    .prio = PBL_PRIO_IDLE + 3,
    .privileged = true,
    .stack = s_hci_task_stack,
    .stack_size = sizeof(s_hci_task_stack),
  };

  s_hci_task_handle = pebble_task_create(PebbleTask_BTHCI, &attr);
  PBL_ASSERTN(s_hci_task_handle);
}

void ble_transport_ll_deinit(void) {
  NVIC_DisableIRQ(LCPU2HCPU_IRQn);
  ipc_queue_close(s_ipc_port);
  ipc_queue_deinit(s_ipc_port);
  s_ipc_port = IPC_QUEUE_INVALID_HANDLE;
  // lcpu_power_off() touches LPSYS_AON registers, which are only reachable while
  // the HP→LP wake request is held. Without this the second invocation faults
  // because the LP system is asleep after the prior power-on cancelled its request.
  HAL_HPAON_WakeCore(CORE_ID_LCPU);
  lcpu_power_off();
  HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
}

/* APIs to be implemented by HS/LL side of transports */
int ble_transport_to_ll_cmd_impl(void *buf) {
  struct ble_hci_cmd *cmd = buf;
  uint8_t h4_cmd = HCI_H4_CMD;
  size_t written;
  int err = 0;

  prv_hci_trace(HCI_H4_CMD, (uint8_t *)cmd, sizeof(*cmd) + cmd->length, H4TL_PACKET_HOST);

  written = ipc_queue_write(s_ipc_port, &h4_cmd, 1, IPC_TIMEOUT_TICKS);
  if (written != 1U) {
    PBL_LOG_ERR("Failed to write HCI CMD header");
    err = BLE_ERR_MEM_CAPACITY;
    goto exit;
  }

  written = ipc_queue_write(s_ipc_port, cmd, sizeof(*cmd) + cmd->length,
                            IPC_TIMEOUT_TICKS);
  if (written != sizeof(*cmd) + cmd->length) {
    PBL_LOG_ERR("Failed to write HCI CMD data");
    err = BLE_ERR_MEM_CAPACITY;
    goto exit;
  }

exit:
  ble_transport_free(buf);

  return err;
}

int ble_transport_to_ll_acl_impl(struct os_mbuf *om) {
  size_t written;
  uint8_t h4_cmd = HCI_H4_ACL;
  struct os_mbuf *x;
  int err = 0;

  prv_hci_trace_mbuf(HCI_H4_ACL, om, H4TL_PACKET_HOST);

  written = ipc_queue_write(s_ipc_port, &h4_cmd, 1U, IPC_TIMEOUT_TICKS);
  if (written != 1U) {
    PBL_LOG_ERR("Failed to write HCI ACL header");
    err = BLE_ERR_MEM_CAPACITY;
    goto exit;
  }

  x = om;
  while (x != NULL) {
    written = ipc_queue_write(s_ipc_port, x->om_data, x->om_len, IPC_TIMEOUT_TICKS);
    if (written != x->om_len) {
      PBL_LOG_ERR("Failed to write HCI ACL data");
      err = BLE_ERR_MEM_CAPACITY;
      goto exit;
    }

    x = SLIST_NEXT(x, om_next);
  }

exit:
  os_mbuf_free_chain(om);

  return err;
}

int ble_transport_to_ll_iso_impl(struct os_mbuf *om) {
  size_t written;
  uint8_t h4_cmd = HCI_H4_ISO;
  int err = 0;
  struct os_mbuf *x;

  prv_hci_trace_mbuf(HCI_H4_ISO, om, H4TL_PACKET_HOST);

  written = ipc_queue_write(s_ipc_port, &h4_cmd, 1, IPC_TIMEOUT_TICKS);
  if (written != 1U) {
    PBL_LOG_ERR("Failed to write HCI ISO header");
    err = BLE_ERR_MEM_CAPACITY;
    goto exit;
  }

  x = om;
  while (x != NULL) {
    written = ipc_queue_write(s_ipc_port, x->om_data, x->om_len, IPC_TIMEOUT_TICKS);
    if (written != x->om_len) {
      PBL_LOG_ERR("Failed to write HCI ISO data");
      err = BLE_ERR_MEM_CAPACITY;
      goto exit;
    }

    x = SLIST_NEXT(x, om_next);
  }

exit:
  os_mbuf_free_chain(om);

  return err;
}

