/*
 * Copyright 2025 Google LLC
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

// TODO: transport.h needs os_mbuf.h to be included first
// clang-format off
#include <os/os_mbuf.h>
// clang-format on
#include <stdio.h>
#include <board/board.h>
#include <drivers/uart.h>
#include <kernel/pebble_tasks.h>
#include <nimble/transport.h>
#include <nimble/transport/hci_h4.h>
#include <nimble/transport_impl.h>
#include <nimble/hci_common.h>
#include <os/os_mempool.h>
#include <os/os_mbuf.h>
#include <queue.h>
#include <system/passert.h>
#include <util/math.h>
#include "ipc_queue.h"

// DEBUG: 1 print as hex string,   2 print as binary,  others no printing
#define DEBUG 0

static TaskHandle_t s_rx_task_handle;
static SemaphoreHandle_t s_rx_data_ready;
static struct hci_h4_sm hci_uart_h4sm;
struct mbox_env_tag
{
    ipc_queue_handle_t ipc_port;
    uint8_t is_init;
};
struct mbox_env_tag mbox_env;


#define H4TL_PACKET_HOST    0x61
#define H4TL_PACKET_CTRL    0x62

static char * hci_type_str(int type)
{
    char * r;
    switch (type) {
        case 1:
            r="CMD";
            break;
        case 2:
            r="ACL";
            break;
        case 3:
            r="SCO";
            break;
        case 4:
            r="EVT";
            break;
        case 5:
            r="ISO";
            break;
        default:
            r="UKN";            
    }
    return r;
}

/**
 * @brief Send HCI data to debug console.
 * @param[in] type HCI type in H4 format
 * @param[in] data HCI traffic data
 * @param[in] len length of data in bytes.
 * @param[in] host2ctrl 1: host to control  0: contrl to host
 * @retval None
*/
#if DEBUG>0
void hci_log_to_console(uint8_t type, uint8_t * data, uint16_t len, int host2ctrl)
{
    static uint8_t temp[1024], *trace ;
    static uint16_t seq;
    trace = &temp[0];
    // Magic for Pebble HCI, 'PBTS'
    *trace++ = 0x50;
    *trace++ = 0x42;
    *trace++ = 0x54;
    *trace++ = 0x53;    
    *trace++ = 0x06;
    *trace++ = 0x01;
    *trace++ = (len + 8) & 0xff;
    *trace++ = (len + 8) >> 8;
    *trace++ = seq & 0xff;
    *trace++ = seq >> 8;
    *trace++ = 0;
    *trace++ = 0;
    *trace++ = 0;
    *trace++ = 0;
    if (host2ctrl)
        *trace++ = H4TL_PACKET_HOST;
    else
        *trace++ = H4TL_PACKET_CTRL;
    *trace++ = type;
    seq++;
    memcpy(trace, data, len);
#if DEBUG==1
    portDISABLE_INTERRUPTS();
    HAL_DBG_printf("%s, %s %d\n", hci_type_str(type), host2ctrl?"TX":"RX", len);    
    HAL_DBG_print_data((char*)data,0,len);
    portENABLE_INTERRUPTS();
#else    
    len+=16;
    for (int i=0;i<len;i++)
        dbgserial_putchar(temp[i]);
#endif    
}
#else
#define hci_log_to_console(type,data,len,host2ctrl)
#endif

static uint8_t read_buf[64];
static void prv_rx_task_main(void *unused) {
  int consumed_bytes;

  while (true) {
    xSemaphoreTake(s_rx_data_ready, portMAX_DELAY);

    while (true) {
      int len = ipc_queue_read(mbox_env.ipc_port, read_buf, sizeof(read_buf));
      if (len>0) {
          while (len>0) {
            consumed_bytes = hci_h4_sm_rx(&hci_uart_h4sm, read_buf, len);
            len-=consumed_bytes;
          }
      }
      else
        break;
    }    
  }
}


static int32_t mbox_rx_ind(ipc_queue_handle_t handle, size_t size)
{
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(s_rx_data_ready, &xHigherPriorityTaskWoken);
    return 0;
}


#define IO_MB_CH      (0)
#define TX_BUF_SIZE   HCPU2LCPU_MB_CH1_BUF_SIZE
#define TX_BUF_ADDR   HCPU2LCPU_MB_CH1_BUF_START_ADDR
#define TX_BUF_ADDR_ALIAS HCPU_ADDR_2_LCPU_ADDR(HCPU2LCPU_MB_CH1_BUF_START_ADDR);
#define RX_BUF_ADDR   LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_START_ADDR);
#define RX_BUF_REV_B_ADDR   LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_REV_B_START_ADDR);

int pb_config_mbox(void)
{
    ipc_queue_cfg_t q_cfg;

    q_cfg.qid = IO_MB_CH;
    q_cfg.tx_buf_size = TX_BUF_SIZE;
    q_cfg.tx_buf_addr = TX_BUF_ADDR;
    q_cfg.tx_buf_addr_alias = TX_BUF_ADDR_ALIAS;
#ifndef SF32LB52X
    /* Config IPC queue. */
    q_cfg.rx_buf_addr = RX_BUF_ADDR;
#else // SF32LB52X
    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
    if (rev_id < HAL_CHIP_REV_ID_A4)
    {
        q_cfg.rx_buf_addr = RX_BUF_ADDR;
    }
    else
    {
        q_cfg.rx_buf_addr = RX_BUF_REV_B_ADDR;
    }
#endif // !SF32LB52X

    q_cfg.rx_ind = NULL;
    q_cfg.user_data = 0;

    if (q_cfg.rx_ind == NULL) {
        q_cfg.rx_ind = mbox_rx_ind;
    }
    
	mbox_env.ipc_port = ipc_queue_init(&q_cfg);
    PBL_ASSERT(IPC_QUEUE_INVALID_HANDLE != mbox_env.ipc_port, "Invalid Handle");
    if (ipc_queue_open(mbox_env.ipc_port))
        PBL_ASSERT(0,"Could not open IPC");
    NVIC_EnableIRQ(LCPU2HCPU_IRQn);
    NVIC_SetPriority(LCPU2HCPU_IRQn, 5);
    
    mbox_env.is_init = 1;
    return 0;
}


static int
hci_uart_frame_cb(uint8_t pkt_type, void *data)
{
    switch (pkt_type) {
    case HCI_H4_EVT:
        {
            struct ble_hci_ev *ev = data;            
            struct ble_hci_ev_command_complete *cmd_complete = (void *) ev->data;         
            if (ev->opcode==BLE_HCI_EVCODE_COMMAND_COMPLETE)
                PBL_LOG(LOG_LEVEL_INFO, "\tCMD complete %x", cmd_complete->opcode);
            
            if (ev->opcode==BLE_HCI_EVCODE_COMMAND_COMPLETE&&cmd_complete->opcode==0xFC11)
                break;                    
            hci_log_to_console(4, data, ev->length+sizeof(*ev),0);
        }
        return ble_transport_to_hs_evt(data);
    case HCI_H4_ACL:
        struct os_mbuf *om=(struct os_mbuf *)data;       
        hci_log_to_console(4, OS_MBUF_DATA(om, uint8_t*), OS_MBUF_PKTLEN(om),0);
        return ble_transport_to_hs_acl(data);
    default:
        assert(0);
        break;
    }

    return -1;
}
#if DEBUG>0
void HAL_DBG_printf(const char *fmt, ...)
{
    va_list args;
    static char rt_log_buf[128];

    va_start(args, fmt);
    vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, fmt, args);
    PBL_LOG(LOG_LEVEL_INFO,"%s", rt_log_buf);    
    va_end(args);
}
#endif

extern void lcpu_power_on(void);
void ble_transport_ll_init(void) {
  hci_h4_sm_init(&hci_uart_h4sm, &hci_h4_allocs_from_ll, hci_uart_frame_cb);

  s_rx_data_ready = xSemaphoreCreateBinary();

  TaskParameters_t task_params = {
      .pvTaskCode = prv_rx_task_main,
      .pcName = "NimbleRX",
      .usStackDepth = 4000 / sizeof(StackType_t), // TODO: can probably be reduced
      .uxPriority = (tskIDLE_PRIORITY + 3) | portPRIVILEGE_BIT,
      .puxStackBuffer = NULL,
  };

  pebble_task_create(PebbleTask_BTHCI, &task_params, &s_rx_task_handle);

  pb_config_mbox();
  lcpu_power_on();

  PBL_ASSERTN(s_rx_task_handle);

}

void ble_queue_cmd(void *buf, bool needs_free, bool wait) {
  ipc_queue_write(mbox_env.ipc_port, buf, 3 + ((uint8_t *)buf)[2], 10);  
  if (needs_free)
    ble_transport_free(buf);
}

/* APIs to be implemented by HS/LL side of transports */
static uint8_t hci_cmd[256];
int ble_transport_to_ll_cmd_impl(void *buf) {   
  struct ble_hci_cmd *cmd = buf;
  hci_cmd[0]=1;
  memcpy(&(hci_cmd[1]), buf, 3 + cmd->length);
  PBL_LOG(LOG_LEVEL_INFO, "BLE TX CMD %x, len=%d", cmd->opcode, 3 + cmd->length+1);
  hci_log_to_console(1,buf,3+cmd->length,1);
  int written = ipc_queue_write(mbox_env.ipc_port, hci_cmd, 3 + cmd->length+1, 10);
  ble_transport_free(buf);
  return (written>=0)?0:-1;
}

static uint8_t hci_acl[256];
int ble_transport_to_ll_acl_impl(struct os_mbuf *om) {
  uint8_t * data=OS_MBUF_DATA(om, uint8_t*);  
  PBL_LOG(LOG_LEVEL_INFO, "BLE TX ACL len=%d",  OS_MBUF_PKTLEN(om));
  hci_acl[0]=2;
  memcpy(&(hci_acl[1]), data, OS_MBUF_PKTLEN(om));  
  hci_log_to_console(2,data,OS_MBUF_PKTLEN(om),1);
  int written = ipc_queue_write(mbox_env.ipc_port, hci_acl, OS_MBUF_PKTLEN(om)+1, 10);   
  os_mbuf_free(om);
  return (written>=0)?0:-1;
}

int ble_transport_to_ll_iso_impl(struct os_mbuf *om) {
  uint8_t * data=OS_MBUF_DATA(om, uint8_t*);  
  int written = ipc_queue_write(mbox_env.ipc_port, data, OS_MBUF_PKTLEN(om), 10);
  os_mbuf_free(om);
  return (written>=0)?0:-1;
}
