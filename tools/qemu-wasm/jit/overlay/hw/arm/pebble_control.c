/*
 * Pebble "remote control" module.
 *
 * This device is designed to sit in between a qemu_chr module and a UART device used
 * by the emulated Pebble. It intercepts the traffic being sent to the UART, looks for
 * specific packets that should be interpreted by QEMU and acts upon them. For other
 * types of packets, it simply passes them on through to the Pebble UART.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/arm/stm32_common.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "qemu/timer.h"
#include "qemu/sockets.h"
#include "qapi/error.h"

#include "pebble_control.h"
#include "hw/arm/pebble.h"
#include "hw/arm/pebble_simple_uart.h"
#include "hw/arm/pebble_gpio.h"

#ifdef EMSCRIPTEN
#include <emscripten/emscripten.h>
#endif

//#define DEBUG_PEBBLE_CONTROL
#ifdef DEBUG_PEBBLE_CONTROL
#define DPRINTF(fmt, ...)                                 \
    do { printf("PEBBLE_CONTROL: " fmt , ## __VA_ARGS__); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

#define EPRINTF(fmt, ...)                                 \
    do { printf("PEBBLE_CONTROL: " fmt , ## __VA_ARGS__); \
    } while (0)


// ------------------------------------------------------------------------------------------
// NOTE: The following QemuProtocol defines describe the protocol used by the host
// to control/communicate with the emulated Pebble.
#define QEMU_HEADER_SIGNATURE 0xFEED
#define QEMU_FOOTER_SIGNATURE 0xBEEF
#define QEMU_MAX_DATA_LEN     2048

// Every message sent over the QEMU control channel has the following header. All
// data is set in network byte order. The maximum data len (not including header or footer)
// allowed is QEMU_MAX_DATA_LEN bytes
typedef struct QEMU_PACKED {
  uint16_t signature;         // QEMU_HEADER_SIGNATURE
  uint16_t protocol;          // one of QemuProtocol
  uint16_t len;               // number of bytes that follow (not including this header or footer)
} QemuCommChannelHdr;

// Every message sent over the QEMU comm channel has the following footer.
typedef struct QEMU_PACKED {
  uint16_t signature;         // QEMU_FOOTER_SIGNATURE
} QemuCommChannelFooter;


// Protocol IDs
typedef enum {
  QemuProtocol_SPP = 1,
  QemuProtocol_Tap = 2,
  QemuProtocol_BluetoothConnection = 3,
  QemuProtocol_Compass = 4,
  QemuProtocol_Battery = 5,
  QemuProtocol_Accel = 6,
  QemuProtocol_Vibration = 7,
  QemuProtocol_Button = 8
} QemuProtocol;


// Structure of the data for various protocols

// For QemuProtocol_SPP, the data is raw Pebble Protocol

// QemuProtocol_Tap
typedef struct QEMU_PACKED {
  uint8_t axis;              // 0: x-axis, 1: y-axis, 2: z-axis
  int8_t direction;         // either +1 or -1
} QemuProtocolTapHeader;


// QemuProtocol_BluetoothConnection
typedef struct QEMU_PACKED {
  uint8_t connected;         // true if connected
} QemuProtocolBluetoothConnectionHeader;


// QemuProtocol_Compass
typedef struct QEMU_PACKED {
  uint32_t magnetic_heading;      // 0x10000 represents 360 degress
  uint8_t  calib_status:8;        // CompassStatus enum
} QemuProtocolCompassHeader;


// QemuProtocol_Battery
typedef struct QEMU_PACKED {
  uint8_t battery_pct;            // from 0 to 100
  uint8_t charger_connected;
} QemuProtocolBatteryHeader;


// QemuProtocol_Accel request (to Pebble)
//! A single accelerometer sample for all three axes
typedef struct QEMU_PACKED {
  int16_t x;
  int16_t y;
  int16_t z;
} QemuProtocolAccelSample;
typedef struct QEMU_PACKED {
  uint8_t     num_samples;
  QemuProtocolAccelSample samples[0];
} QemuProtocolAccelHeader;

// QemuProtocol_Accel response (back to host)
typedef struct QEMU_PACKED {
  uint16_t     avail_space;   // Number of samples we can accept
} QemuProtocolAccelResponseHeader;


// QemuProtocol_Vibration notification (sent from Pebble to host)
typedef struct QEMU_PACKED {
  uint8_t     on;             // non-zero if vibe is on, 0 if off
} QemuProtocolVibrationNotificationHeader;


// QemuProtocol_Button
typedef struct QEMU_PACKED {
  // New button state. Bit x specifies the state of button x, where x is one of the
  // ButtonId enum values.
  uint8_t     button_state;
} QemuProtocolButtonHeader;



// -----------------------------------------------------------------------------------------
// PebbleControl globals
#define PBLCONTROL_BUF_LEN (QEMU_MAX_DATA_LEN + sizeof(QemuCommChannelHdr) \
                                + sizeof(QemuCommChannelFooter))

struct PebbleControl {
    /* Inherited */
    SysBusDevice parent_obj;

    // The qemu_chr driver that connects to the host over a socket connection. We receive
    // data from this device, interpret it, and either process it directly in here or forward
    // it onto the uart in the emulated pebble.
    CharFrontend chr;

    // The uart used by the emulated Pebble. We send data to it using its IOHandler
    // callbacks. Data written to the UART by the emulated Pebble gets passed onto us
    // because we provide the UART device a pointer to our pebble_control_write() method.
    void *uart;
    IOEventHandler *uart_chr_event;
    IOCanReadHandler *uart_chr_can_read;
    IOReadHandler *uart_chr_read;


    // We buffer the characters we receive from our qemu_chr receive handler here until
    // we get a complete packet. From there, we can figure out if we should process it
    // directly or pass it onto the target's UART
    uint8_t rcv_char_buf[PBLCONTROL_BUF_LEN];
    uint32_t rcv_char_bytes;    /* number of bytes avaialable in rcv_char_buf */


    // If we are passing a packet onto the target UART, this contains the number of bytes left
    // to transfer. The bytes we are transferring are always at the front of the
    // rcv_char_buf.
    uint32_t   target_send_bytes;

    // Timer used to wake us up to pump more data to the target
    struct QEMUTimer *target_send_timer;

    // We buffer the characters the UART from the target wants to send out here.
    // We only send it to the front end once we have a complete packet. This insures
    // that packets we went to send out don't interrupt midstream one that the target is
    // sending.
    uint8_t send_char_buf[PBLCONTROL_BUF_LEN];
    uint32_t send_char_bytes;    /* number of bytes avaialable in send_char_buf */

    // True if this is a generic (non-STM32) UART connection
    bool is_generic;

    // For generic UART: the DeviceState pointer (PblSimpleUart)
    DeviceState *generic_uart;

#ifdef EMSCRIPTEN
    // Timer that drains the browser->guest ring buffer
    struct QEMUTimer *wasm_poll_timer;
#endif
};

#ifdef EMSCRIPTEN
/* Shared-memory serial bridge for the browser build.
 *
 * The page cannot open a TCP connection to the emulated pebble-tool
 * channel, so it exchanges QemuProtocol frames through two ring buffers
 * in wasm memory instead. JS writes host->guest bytes at rx_head and
 * publishes the new head with Atomics.store; a virtual-clock timer in
 * the QEMU thread consumes them through the normal chardev receive
 * path. Guest->host bytes are mirrored into the tx ring, which JS
 * drains by advancing tx_tail. Head/tail are free-running u32 counters
 * (index = counter % size); buffer addresses are u32 because the build
 * caps the address space at 4GB (--enable-wasm64-32bit-address-limit).
 */
#define WASM_SER_RX_SIZE 131072
#define WASM_SER_TX_SIZE 32768

static uint8_t s_wasm_ser_rx[WASM_SER_RX_SIZE];
static uint8_t s_wasm_ser_tx[WASM_SER_TX_SIZE];

static struct {
    uint32_t rx_buf, rx_size, tx_buf, tx_size;
    uint32_t rx_head, rx_tail;   /* JS produces, QEMU consumes */
    uint32_t tx_head, tx_tail;   /* QEMU produces, JS consumes */
} s_wasm_ser_ctrl;

EMSCRIPTEN_KEEPALIVE void *pebble_wasm_serial_ctrl(void)
{
    s_wasm_ser_ctrl.rx_buf = (uint32_t)(uintptr_t)s_wasm_ser_rx;
    s_wasm_ser_ctrl.rx_size = WASM_SER_RX_SIZE;
    s_wasm_ser_ctrl.tx_buf = (uint32_t)(uintptr_t)s_wasm_ser_tx;
    s_wasm_ser_ctrl.tx_size = WASM_SER_TX_SIZE;
    return &s_wasm_ser_ctrl;
}

static void pebble_control_wasm_tx(const uint8_t *buf, int len)
{
    uint32_t head = s_wasm_ser_ctrl.tx_head;
    uint32_t used = head - qatomic_load_acquire(&s_wasm_ser_ctrl.tx_tail);

    if (len <= 0) {
        return;
    }
    if (used + (uint32_t)len > WASM_SER_TX_SIZE) {
        /* JS is not draining (page gone?) — drop, never block the guest */
        return;
    }
    for (int i = 0; i < len; i++) {
        s_wasm_ser_tx[(head + i) % WASM_SER_TX_SIZE] = buf[i];
    }
    qatomic_store_release(&s_wasm_ser_ctrl.tx_head, head + len);
}
#endif

static inline bool pebble_control_wasm_bridge_enabled(void)
{
#ifdef EMSCRIPTEN
    return true;
#else
    return false;
#endif
}

/* Write host-bound bytes: the chardev if one is connected, plus the
 * browser ring under emscripten. */
static int pc_host_write(PebbleControl *s, const uint8_t *buf, int len)
{
#ifdef EMSCRIPTEN
    pebble_control_wasm_tx(buf, len);
#endif
    if (qemu_chr_fe_backend_connected(&s->chr)) {
        return qemu_chr_fe_write_all(&s->chr, buf, len);
    }
    return len;
}


// Control channel handlers are defined using this structure
typedef void (*PebbleControlMessageCallback)(PebbleControl *s, const uint8_t* data,
                                             uint32_t length);
typedef struct {
  uint16_t protocol_id;
  PebbleControlMessageCallback callback;
} PebbleControlMessageHandler;



// -----------------------------------------------------------------------------------
static void pebble_control_button_msg_callback(PebbleControl *s, const uint8_t *data,
                                              uint32_t len)
{
    DPRINTF("%s: \n", __func__);
    QemuProtocolButtonHeader *hdr = (QemuProtocolButtonHeader *)data;
    if (len != sizeof(*hdr)) {
        EPRINTF("%s: invalid packet\n", __func__);
        return;
    }

    DPRINTF("%s: new button state: 0x%x\n", __func__, (int)hdr->button_state);
    if (s->is_generic) {
        pbl_gpio_set_button_state(hdr->button_state);
    } else {
        pebble_set_button_state(hdr->button_state);
    }
}



// -----------------------------------------------------------------------------------------
// Find handler from s_qemu_endpoints for a given protocol
static const PebbleControlMessageHandler* pebble_control_find_handler(PebbleControl *s,
                                                             uint16_t protocol_id) {
    static const PebbleControlMessageHandler s_msg_endpoints[] = {
      // IMPORTANT: These must be in sorted order!!
      { QemuProtocol_Button, pebble_control_button_msg_callback },
    };

    size_t i;
    for (i = 0; i < ARRAY_SIZE(s_msg_endpoints); ++i) {
      const PebbleControlMessageHandler* handler = &s_msg_endpoints[i];
      if (!handler || handler->protocol_id > protocol_id) {
        break;
      }

      if (handler->protocol_id == protocol_id) {
        return handler;
      }
    }

    return NULL;
}




// -----------------------------------------------------------------------------------
// Drop the first N bytes out of the beginning of the receive buffer
static void pebble_control_consume_rcv_bytes(PebbleControl *s, uint32_t n)
{
    assert (n <= s->rcv_char_bytes);
    s->rcv_char_bytes -= n;
    memmove(&s->rcv_char_buf[0], &s->rcv_char_buf[n], s->rcv_char_bytes);
}


// -----------------------------------------------------------------------------------
// Forward the remaining portion of the packet at the front of our receive buffer onto the
// target
static void pebble_control_forward_to_target(PebbleControl *s)
{
    if (s->target_send_bytes == 0) {
        return;
    }
    DPRINTF("%s: %d bytes left to send to target\n", __func__, s->target_send_bytes);

    bool sent = false;
    int can_read_bytes = s->uart_chr_can_read(s->uart);
    if (can_read_bytes > 0) {
        can_read_bytes = MIN(can_read_bytes, s->target_send_bytes);
        s->uart_chr_read(s->uart, s->rcv_char_buf, can_read_bytes);
        pebble_control_consume_rcv_bytes(s, can_read_bytes);
        s->target_send_bytes -= can_read_bytes;
        sent = true;
        DPRINTF("%s: sent %d bytes to target, %d remaining\n", __func__, can_read_bytes,
                  s->target_send_bytes);
    }

    // If more data to send, retry. Use immediate reschedule when we just sent
    // data (UART drained fast enough), but back off 1ms when the UART is busy
    // so the guest CPU gets time to run the ISR and drain the byte.
    if (s->target_send_bytes) {
        DPRINTF("%s: Scheduling pebble_control_forward_to_target timer\n", __func__);
        timer_mod(s->target_send_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_HOST) + (sent ? 0 : 1));
    }
}


// -----------------------------------------------------------------------------------
// Parse through our receive buffer, for each complete control packet, process it
static void pebble_control_parse_receive_buffer(PebbleControl *s)
{
    // If we are still forwarding data to the target, finish that first
    if (s->target_send_bytes) {
        pebble_control_forward_to_target(s);
        if (s->target_send_bytes) {
            return;
        }
    }

    // Look for a complete packet
    while (s->rcv_char_bytes >= sizeof(QemuCommChannelHdr) + sizeof(QemuCommChannelFooter)) {
        QemuCommChannelHdr *hdr = (QemuCommChannelHdr *)s->rcv_char_buf;

        // Check the header signature
        if (ntohs(hdr->signature) != QEMU_HEADER_SIGNATURE) {
            DPRINTF("%s: invalid packet hdr signature detected\n", __func__);
            pebble_control_consume_rcv_bytes(s, sizeof(hdr->signature));
        }

        // Validate the length
        uint16_t data_len = ntohs(hdr->len);
        if (data_len > QEMU_MAX_DATA_LEN) {
            DPRINTF("%s: invalid packet hdr len detected\n", __func__);
            pebble_control_consume_rcv_bytes(s, sizeof(*hdr));
        }

        // If not a complete packet yet, break out
        uint16_t total_size = sizeof(QemuCommChannelHdr) + data_len
                                + sizeof(QemuCommChannelFooter);
        if (s->rcv_char_bytes < total_size) {
            break;
        }

        // We have a complete packet, see if we should process it directly or pass it onto
        // the target
        uint16_t protocol = ntohs(hdr->protocol);
        const PebbleControlMessageHandler* handler = pebble_control_find_handler(s, protocol);
        if (!handler) {
            DPRINTF("%s: passing packet with protocol %d (%d bytes) onto target\n",
                   __func__, protocol, total_size);
            s->target_send_bytes = total_size;
            pebble_control_forward_to_target(s);
            if (s->target_send_bytes) {
                // If we couldn't pass it all on, break out and wait for the timer callback
                // to send the rest out
                break;
            }
        } else {
            handler->callback(s, (uint8_t *)(hdr+1), data_len);
            pebble_control_consume_rcv_bytes(s, total_size);
        }

    }

}


// -----------------------------------------------------------------------------------
// Char device receive handlers
static void pebble_control_event(void *opaque, QEMUChrEvent event)
{
    PebbleControl *s = (PebbleControl *)opaque;

    if (s->uart_chr_event) {
        s->uart_chr_event(s->uart, event);
    }
}

static int pebble_control_can_receive(void *opaque)
{
    PebbleControl *s = (PebbleControl *)opaque;

    /* How much space do we have in our buffer? */
    return (PBLCONTROL_BUF_LEN - s->rcv_char_bytes);
}

static void pebble_control_receive(void *opaque, const uint8_t *buf, int size)
{
    PebbleControl *s = (PebbleControl *)opaque;

    assert(size > 0);
#ifdef DEBUG_PEBBLE_CONTROL
    printf("PEBBLE_CONTROL: %s: received %d bytes from host, first:", __func__, size);
    for (int di = 0; di < size && di < 32; di++) {
        printf(" %02x", buf[di]);
    }
    printf("\n");
#endif

    // Copy the characters into our buffer first
    assert (size <= PBLCONTROL_BUF_LEN - s->rcv_char_bytes);
    memmove(s->rcv_char_buf + s->rcv_char_bytes, buf, size);
    s->rcv_char_bytes += size;

    // Process any complete packets in the receive buffer
    pebble_control_parse_receive_buffer(s);
}


// -----------------------------------------------------------------------------------
// Drop the first N bytes out of the beginning of the send buffer
static void pebble_control_consume_send_bytes(PebbleControl *s, uint32_t n)
{
    assert (n <= s->send_char_bytes);
    s->send_char_bytes -= n;
    memmove(&s->send_char_buf[0], &s->send_char_buf[n], s->send_char_bytes);
}


// -----------------------------------------------------------------------------------
// This method gets passed to the UART's stm32_uart_set_write_handler(). This way
//  we can intercept all writes that the UART sends to the front end and insure that
//  we don't interrupt one mid-stream by sending a packet from QEMU
static int pebble_control_write(void *opaque, const uint8_t *buf, int len) {
    PebbleControl *s = (PebbleControl *)opaque;

    while (len) {
        // Copy the new bytes in
        uint32_t space_left = sizeof(s->send_char_buf) - s->send_char_bytes;

        if (space_left == 0) {
            EPRINTF("%s: overflowed send buffer, aborting queued up data\n", __func__);
            s->send_char_bytes = 0;
            space_left = sizeof(s->send_char_buf);
        }
        uint32_t bytes_to_copy = MIN(space_left, len);
        memmove(&s->send_char_buf[s->send_char_bytes], buf, bytes_to_copy);
        s->send_char_bytes += bytes_to_copy;
        len -= bytes_to_copy;


        // ------------------------------------------------------------------
        // See if we have a complete packet yet
        if (s->send_char_bytes < sizeof(QemuCommChannelHdr)
                                 + sizeof(QemuCommChannelFooter)) {
            break;
        }
        QemuCommChannelHdr *hdr = (QemuCommChannelHdr *)s->send_char_buf;

        // Check the header signature
        if (ntohs(hdr->signature) != QEMU_HEADER_SIGNATURE) {
            DPRINTF("%s: invalid packet hdr signature detected\n", __func__);
            pebble_control_consume_send_bytes(s, sizeof(hdr->signature));
        }

        // Validate the length
        uint16_t data_len = ntohs(hdr->len);
        if (data_len > QEMU_MAX_DATA_LEN) {
            DPRINTF("%s: invalid packet hdr len detected\n", __func__);
            pebble_control_consume_send_bytes(s, sizeof(*hdr));
        }

        // If not a complete packet yet, break out
        uint16_t total_size = sizeof(QemuCommChannelHdr) + data_len
                                + sizeof(QemuCommChannelFooter);
        if (s->send_char_bytes < total_size) {
            if (len > 0) {
                // If we still have not put in all the bytes the caller wanted,
                // we must be off-frame because we ran out of room.
                EPRINTF("%s: overflowed send buffer, aborting queued up data\n", __func__);
                s->send_char_bytes = 0;
                continue;
            }
            break;
        }

        // Intercept vibration packets for local visualization
        if (ntohs(hdr->protocol) == QemuProtocol_Vibration && data_len >= 1) {
            uint8_t *data = s->send_char_buf + sizeof(QemuCommChannelHdr);
            bool vibe_on = data[0] != 0;
            extern void pbl_display_set_vibrating(bool on);
            pbl_display_set_vibrating(vibe_on);
        }

        // We have a complete packet, send it out the front end
        int bytes_sent;
        DPRINTF("%s: Sending packet of %d bytes to host (proto=0x%04x)\n",
               __func__, total_size, ntohs(hdr->protocol));
        while (total_size) {
            bytes_sent = pc_host_write(s, s->send_char_buf, total_size);
            if (bytes_sent <= 0) {
                // Write error (e.g. TCP client disconnected), discard packet
                pebble_control_consume_send_bytes(s, total_size);
                break;
            }
            total_size -= bytes_sent;
            pebble_control_consume_send_bytes(s, bytes_sent);
        }

    }

    return len;
}


// -----------------------------------------------------------------------------------------
static void pebble_control_send_packet(PebbleControl *s, QemuProtocol protocol, void *data,
                                uint32_t len)
{
  // Send the header
  QemuCommChannelHdr hdr = (QemuCommChannelHdr) {
    .signature = htons(QEMU_HEADER_SIGNATURE),
    .protocol = htons(protocol),
    .len = htons(len)
  };
  pc_host_write(s, (uint8_t *)&hdr, sizeof(hdr));

  // Send the data
  pc_host_write(s, data, len);

  // Send the footer
  QemuCommChannelFooter footer = (QemuCommChannelFooter) {
    .signature = htons(QEMU_FOOTER_SIGNATURE)
  };

  pc_host_write(s, (uint8_t *)&footer, sizeof(footer));
}

// -----------------------------------------------------------------------------------
// Send a vibe notification to the host
void pebble_control_send_vibe_notification(PebbleControl *s, bool on)
{
    DPRINTF("%s: vibe %d\n", __func__, (int)on);

    QemuProtocolVibrationNotificationHeader hdr = {
      .on = on
    };
    pebble_control_send_packet(s, QemuProtocol_Vibration, &hdr, sizeof(hdr));
}

#ifdef EMSCRIPTEN
// -----------------------------------------------------------------------------------
// Drain the browser->guest ring through the normal receive path.
static void pebble_control_wasm_poll(void *opaque)
{
    PebbleControl *s = (PebbleControl *)opaque;
    uint32_t head = qatomic_load_acquire(&s_wasm_ser_ctrl.rx_head);
    uint32_t tail = s_wasm_ser_ctrl.rx_tail;

    while (tail != head) {
        int space = pebble_control_can_receive(s);
        if (space <= 0) {
            break;
        }
        uint32_t off = tail % WASM_SER_RX_SIZE;
        uint32_t n = MIN(head - tail, WASM_SER_RX_SIZE - off);
        n = MIN(n, (uint32_t)space);
        pebble_control_receive(s, &s_wasm_ser_rx[off], n);
        tail += n;
        qatomic_store_release(&s_wasm_ser_ctrl.rx_tail, tail);
    }
    timer_mod(s->wasm_poll_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 2);
}
#endif

// -----------------------------------------------------------------------------------
#if 0 /* legacy STM32 machines are not in this build */
PebbleControl *pebble_control_create(Chardev *chr, Stm32Uart *uart)
{
    PebbleControl *s = g_malloc0(sizeof(PebbleControl));

    if (chr) {
        // Initialize our own CharFrontend with the chardev
        qemu_chr_fe_init(&s->chr, chr, &error_abort);
        s->uart = uart;

        // The timer we use to pump more data to the uart
        s->target_send_timer = timer_new_ms(QEMU_CLOCK_HOST,
                                  (QEMUTimerCB *)pebble_control_parse_receive_buffer, s);

        // Have the UART send writes to us
        stm32_uart_set_write_handler(uart, s, pebble_control_write);

        // Save away the receive handlers that the uart installed into chr
        stm32_uart_get_rcv_handlers(uart, &s->uart_chr_can_read, &s->uart_chr_read, &s->uart_chr_event);

        // Install our own receive handlers into the CharFrontend
        qemu_chr_fe_set_handlers(&s->chr,
                        pebble_control_can_receive,
                        pebble_control_receive,
                        pebble_control_event,
                        NULL,
                        (void *)s,
                        NULL,
                        true);
    }

    return s;
}

// -----------------------------------------------------------------------------------
#endif /* legacy create */

PebbleControl *pebble_control_create_generic(Chardev *chr, DeviceState *uart)
{
    PebbleControl *s = g_malloc0(sizeof(PebbleControl));

    if (uart && (chr || pebble_control_wasm_bridge_enabled())) {
        s->is_generic = true;
        s->generic_uart = uart;

        // Initialize our own CharFrontend with the chardev
        if (chr) {
            qemu_chr_fe_init(&s->chr, chr, &error_abort);
        }

        // The timer we use to pump more data to the uart
        s->target_send_timer = timer_new_ms(QEMU_CLOCK_HOST,
                                  (QEMUTimerCB *)pebble_control_parse_receive_buffer, s);

        // Have the simple UART send writes to us
        pbl_uart_set_write_handler(uart, s, pebble_control_write);

        // Get the UART's receive handlers for forwarding data to firmware
        pbl_uart_get_rcv_handlers(uart, &s->uart_chr_can_read,
                                  &s->uart_chr_read, &s->uart_chr_event);
        s->uart = uart;

        // Install our own receive handlers into the CharFrontend
        if (chr) {
            qemu_chr_fe_set_handlers(&s->chr,
                            pebble_control_can_receive,
                            pebble_control_receive,
                            pebble_control_event,
                            NULL,
                            (void *)s,
                            NULL,
                            true);
        }

#ifdef EMSCRIPTEN
        s->wasm_poll_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                          pebble_control_wasm_poll, s);
        timer_mod(s->wasm_poll_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 2);
#endif
    }

    return s;
}

// -----------------------------------------------------------------------------------
// F7xx variant is not yet ported - stub for compilation
PebbleControl *pebble_control_create_stm32f7xx(Chardev *chr, Stm32F7xxUart *uart)
{
    (void)chr;
    (void)uart;
    return NULL;
}
