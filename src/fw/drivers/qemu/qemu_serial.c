/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/qemu/qemu_accel.h>
#include <pbl/drivers/qemu/qemu_battery.h>
#include <pbl/drivers/qemu/qemu_serial.h>
#include <pbl/drivers/qemu/qemu_serial_private.h>
#include <pbl/drivers/uart.h>
#include "kernel/events.h"
#include "popups/timeline/peek.h"
#include "process_management/app_manager.h"
#include "shell/system_theme.h"
#include "pbl/services/activity/activity.h"
#include "pbl/services/activity/activity_private.h"
#include "pbl/services/clock.h"
#include "pbl/services/hrm/hrm_manager.h"
#include "system/hexdump.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "util/net.h"
#include "pbl/util/size.h"

#include <bluetooth/qemu_transport.h>

#include <stdbool.h>

static bool prv_uart_irq_handler(UARTDevice *dev, uint8_t byte, const UARTRXErrorFlags *err_flags);

// Our globals
static QemuSerialGlobals s_qemu_state;


// -----------------------------------------------------------------------------------------
// Handle incoming Tap packet data (QemuProtocol_Tap)
static void prv_tap_msg_callback(const uint8_t *data, uint32_t len) {
  QemuProtocolTapHeader *hdr = (QemuProtocolTapHeader *)data;
  if (len != sizeof(*hdr)) {
    PBL_LOG_ERR("Invalid packet length");
    return;
  }

  PBL_LOG_DBG("Got tap msg: axis: %d, direction: %d", hdr->axis, hdr->direction);
  PebbleEvent e = {
    .type = PEBBLE_ACCEL_SHAKE_EVENT,
    .accel_tap = {
      .axis = hdr->axis,
      .direction = hdr->direction,
    },
  };

  event_put(&e);
}


// -----------------------------------------------------------------------------------------
// Handle incoming Bluetooth connection packet data (QemuProtocol_BluetoothConnection)
static void prv_bluetooth_connection_msg_callback(const uint8_t *data, uint32_t len) {
  QemuProtocolBluetoothConnectionHeader *hdr = (QemuProtocolBluetoothConnectionHeader *)data;
  if (len != sizeof(*hdr)) {
    PBL_LOG_ERR("Invalid packet length");
    return;
  }

  PBL_LOG_DBG("Got bluetooth connection msg: connected:%d", hdr->connected);
  bool current_status = qemu_transport_is_connected();
  bool new_status = (hdr->connected != 0);


  if (new_status != current_status && !bt_ctl_is_airplane_mode_on()) {
    // Change to new status if we're not in airplane mode
    qemu_transport_set_connected(new_status);
  }

}


// -----------------------------------------------------------------------------------------
// Handle incoming compass packet data (QemuProtocol_Compass)
static void prv_compass_msg_callback(const uint8_t *data, uint32_t len) {
  QemuProtocolCompassHeader *hdr = (QemuProtocolCompassHeader *)data;
  if (len != sizeof(*hdr)) {
    PBL_LOG_ERR("Invalid packet length");
    return;
  }

  PBL_LOG_DBG("Got compass msg: magnetic_heading: %"PRId32", calib_status:%u",
        ntohl(hdr->magnetic_heading), hdr->calib_status);
  PebbleEvent e = {
    .type = PEBBLE_COMPASS_DATA_EVENT,
    .compass_data = {
      .magnetic_heading = ntohl(hdr->magnetic_heading),
      .calib_status = hdr->calib_status
    }
  };

  event_put(&e);
}


// -----------------------------------------------------------------------------------------
// Handle incoming time format data (QemuProtocol_TimeFormat)
static void prv_time_format_msg_callback(const uint8_t *data, uint32_t len) {
  QemuProtocolTimeFormatHeader *hdr = (QemuProtocolTimeFormatHeader *)data;
  if (len != sizeof(*hdr)) {
    PBL_LOG_ERR("Invalid packet length");
    return;
  }

  PBL_LOG_DBG("Got time format msg: is 24 hour: %d", hdr->is_24_hour);
  clock_set_24h_style(hdr->is_24_hour);
}


// -----------------------------------------------------------------------------------------
// Handle incoming timeline peek format data (QemuProtocol_TimelinePeek)
static void prv_timeline_peek_msg_callback(const uint8_t *data, uint32_t len) {
  QemuProtocolTimelinePeekHeader *hdr = (QemuProtocolTimelinePeekHeader *)data;
  if (len != sizeof(*hdr)) {
    PBL_LOG_ERR("Invalid packet length");
    return;
  }

  PBL_LOG_DBG("Got timeline peek msg: enabled: %d", hdr->enabled);
#if !defined(CONFIG_RECOVERY_FW)
  timeline_peek_set_enabled(hdr->enabled);
#endif
}


static void prv_content_size_msg_callback(const uint8_t *data, uint32_t len) {
  QemuProtocolContentSizeHeader *hdr = (QemuProtocolContentSizeHeader *)data;
  if (len != sizeof(*hdr)) {
    PBL_LOG_ERR("Invalid packet length");
    return;
  }

  PBL_LOG_DBG("Got content size msg: size: %d", hdr->size);
#if !defined(CONFIG_RECOVERY_FW)
  system_theme_set_content_size(hdr->size);

  // Exit out of any currently running app so we force the UI to update to the new content size
  // (must be called from the KernelMain task)
  PBL_ASSERT_TASK(PebbleTask_KernelMain);
  app_manager_close_current_app(true /* gracefully */);
#endif
}


// -----------------------------------------------------------------------------------------
// Handle incoming health metric data (QemuProtocol_HealthMetric)
static void prv_health_metric_msg_callback(const uint8_t *data, uint32_t len) {
  QemuProtocolHealthMetricHeader *hdr = (QemuProtocolHealthMetricHeader *)data;
  if (len != sizeof(*hdr)) {
    PBL_LOG_ERR("Invalid packet length");
    return;
  }

  const int32_t value = (int32_t)ntohl(hdr->value);
  PBL_LOG_DBG("Got health metric msg: metric: %d, value: %"PRId32, hdr->metric, value);

#if !defined(CONFIG_RECOVERY_FW)
  ActivityMetric metric;
  switch (hdr->metric) {
    case QemuHealthMetric_Steps:               metric = ActivityMetricStepCount; break;
    case QemuHealthMetric_ActiveSeconds:       metric = ActivityMetricActiveSeconds; break;
    case QemuHealthMetric_RestingCalories:     metric = ActivityMetricRestingKCalories; break;
    case QemuHealthMetric_ActiveCalories:      metric = ActivityMetricActiveKCalories; break;
    case QemuHealthMetric_DistanceMeters:      metric = ActivityMetricDistanceMeters; break;
    case QemuHealthMetric_SleepTotalSeconds:   metric = ActivityMetricSleepTotalSeconds; break;
    case QemuHealthMetric_SleepRestfulSeconds: metric = ActivityMetricSleepRestfulSeconds; break;
    default:
      PBL_LOG_WRN("Unknown health metric: %d", hdr->metric);
      return;
  }
  activity_metrics_set_metric_exact(metric, value);
#endif
}


// -----------------------------------------------------------------------------------------
// Handle incoming heart rate data (QemuProtocol_HeartRate)
static void prv_heart_rate_msg_callback(const uint8_t *data, uint32_t len) {
  QemuProtocolHeartRateHeader *hdr = (QemuProtocolHeartRateHeader *)data;
  if (len != sizeof(*hdr)) {
    PBL_LOG_ERR("Invalid packet length");
    return;
  }

  PBL_LOG_DBG("Got heart rate msg: bpm: %d, quality: %d", hdr->bpm, hdr->quality);
#if defined(CONFIG_HRM)
  HRMData hrm_data = {
    .features = HRMFeature_BPM,
    .hrm_bpm = hdr->bpm,
    .hrm_quality = (HRMQuality)hdr->quality,
  };
  hrm_manager_new_data_cb(&hrm_data);
#else
  PBL_LOG_WRN("Heart rate injection unsupported on this board (no HRM)");
#endif
}


// -----------------------------------------------------------------------------------------
// List of incoming message handlers
static const QemuMessageHandler s_qemu_endpoints[] = {
  // IMPORTANT: These must be in sorted order!!
  { QemuProtocol_SPP, qemu_transport_handle_received_data },
  { QemuProtocol_Tap, prv_tap_msg_callback },
  { QemuProtocol_BluetoothConnection, prv_bluetooth_connection_msg_callback },
  { QemuProtocol_Compass, prv_compass_msg_callback },
  { QemuProtocol_Battery, qemu_battery_msg_callack },
  { QemuProtocol_Accel, qemu_accel_msg_callack },
  { QemuProtocol_TimeFormat, prv_time_format_msg_callback },
  { QemuProtocol_TimelinePeek, prv_timeline_peek_msg_callback },
  { QemuProtocol_ContentSize, prv_content_size_msg_callback },
  { QemuProtocol_HealthMetric, prv_health_metric_msg_callback },
  { QemuProtocol_HeartRate, prv_heart_rate_msg_callback },
  // Button messages are handled by QEMU directly
};


// -----------------------------------------------------------------------------------------
// Find handler from s_qemu_endpoints for a given protocol
static const QemuMessageHandler* prv_find_handler(uint16_t protocol_id) {
  for (size_t i = 0; i < ARRAY_LENGTH(s_qemu_endpoints); ++i) {
    const QemuMessageHandler* handler = &s_qemu_endpoints[i];
    if (!handler || handler->protocol_id > protocol_id) {
      break;
    }

    if (handler->protocol_id == protocol_id) {
      return handler;
    }
  }

  return NULL;
}


// -----------------------------------------------------------------------------------------
void qemu_serial_init(void) {
  // Init our state variables
  qemu_serial_private_init_state(&s_qemu_state);

  // Init the UART
  uart_init(QEMU_UART);
  uart_set_baud_rate(QEMU_UART, UART_SERIAL_BAUD_RATE);
  uart_set_rx_interrupt_handler(QEMU_UART, prv_uart_irq_handler);

  // enable the UART RX interrupt
  uart_set_rx_interrupt_enabled(QEMU_UART, true);
}


// -----------------------------------------------------------------------------------------
// KernelMain callback triggred by our ISR handler when we detect a high water mark on our
//  receive buffer or a footer signature
static void prv_process_receive_buffer(void *context) {
  uint32_t msg_bytes;
  uint16_t protocol;

  // Clear the pending flag at entry so the next ISR-side trigger (footer
  // sighting, buffer-full, recv error) can queue another drain callback.
  // Without this, the flag stays true forever after the first callback,
  // any later "interesting" ISR event silently drops its callback request,
  // and once the ISR buffer fills the RX IRQ is disabled — no further drain
  // happens and the protocol stack wedges mid-transfer.
  s_qemu_state.callback_pending = false;

  // Process ISR receive buffer, see if we have a complete message
  // Prevent our ISR from putting more characters in while we muck with the receive buffer by
  //  disabling UART interrupts while we process it.
  while (true) {
    uart_set_rx_interrupt_enabled(QEMU_UART, false);
    uint8_t *msg_ptr = qemu_serial_private_assemble_message(&s_qemu_state, &msg_bytes, &protocol);
    uart_set_rx_interrupt_enabled(QEMU_UART, true);
    if (!msg_ptr) {
      break;
    }

    // Dispatch the received message
    PBL_LOG_DBG("Dispatching msg of len %"PRIu32" for protocol %d", msg_bytes,
              protocol);
    const QemuMessageHandler* handler = prv_find_handler(protocol);
    if (!handler) {
      PBL_LOG_WRN("No handler for protocol: %d", protocol);
    } else {
      handler->callback(msg_ptr, msg_bytes);
    }
  }
}


// -----------------------------------------------------------------------------------------
static bool prv_uart_irq_handler(UARTDevice *dev, uint8_t byte, const UARTRXErrorFlags *err_flags) {
  // The interrupt triggers when a byte has been read from the UART. QEMU's
  // emulated UARTs don't emulate receive overruns by default so we don't have
  // to worry about that case. QEMU will buffer the data stream until we're
  // ready to consume more data by reading from the UART again.
  PBL_ASSERTN(!err_flags->overrun_error);

  // Add to circular buffer. It's safe to assume that the buffer has space
  // remaining as the RX interrupt will be disabled from the time the buffer
  // fills up until when the buffer is drained.
  bool success = shared_circular_buffer_write(&s_qemu_state.isr_buffer, &byte, 1,
                                              false/*advance_slackers*/);
  if (!success) {
    PBL_LOG_ERR("ISR buf too small 0x%x", byte);
    s_qemu_state.recv_error_count++;
  }

  bool buffer_full = false;
  if (!shared_circular_buffer_get_write_space_remaining(&s_qemu_state.isr_buffer)) {
    // There's no more room in the buffer, so disable the RX interrupt. No more
    // data will be read from the UART until prv_process_receive_buffer() is
    // run, draining the buffer and re-enabling the RX interrupt. QEMU will
    // buffer the remaining data until the interrupt is re-enabled.
    uart_set_rx_interrupt_enabled(dev, false);
    buffer_full = true;
  }

  // Is it time to wake up the main thread?
  bool should_context_switch = false;
  if (s_qemu_state.recv_error_count || buffer_full ||
      (byte == QEMU_FOOTER_LSB && s_qemu_state.prev_byte == QEMU_FOOTER_MSB)) {
    if (!s_qemu_state.callback_pending) {
      s_qemu_state.callback_pending = true;
      PebbleEvent e = {
        .type = PEBBLE_CALLBACK_EVENT,
        .callback = {
          .callback = prv_process_receive_buffer,
          .data = NULL
        }
      };
      should_context_switch = event_put_isr(&e);
    }
  }

  s_qemu_state.prev_byte = byte;

  return should_context_switch;
}


// -----------------------------------------------------------------------------------------
static void prv_send(const uint8_t *data, uint32_t len) {
  PBL_LOG_VERBOSE("Sending data:");
  PBL_HEXDUMP(LOG_LEVEL_DEBUG_VERBOSE, data, len);

  while (len--) {
    uart_write_byte(QEMU_UART, *data++);
  }
  uart_wait_for_tx_complete(QEMU_UART);
}


// -----------------------------------------------------------------------------------------
void qemu_serial_send(QemuProtocol protocol, const uint8_t *data, uint32_t len) {
  if (!s_qemu_state.initialized) {
    return;
  }

  pbl_mutex_lock(&s_qemu_state.qemu_comm_lock, PBL_FOREVER);

  // Send the header
  QemuCommChannelHdr hdr = (QemuCommChannelHdr) {
    .signature = htons(QEMU_HEADER_SIGNATURE),
    .protocol = htons(protocol),
    .len = htons(len)
  };
  prv_send((uint8_t *)&hdr, sizeof(hdr));

  // Send the data
  prv_send(data, len);

  // Send the footer
  QemuCommChannelFooter footer = (QemuCommChannelFooter) {
    .signature = htons(QEMU_FOOTER_SIGNATURE)
  };
  prv_send((uint8_t *)&footer, sizeof(footer));

  pbl_mutex_unlock(&s_qemu_state.qemu_comm_lock);
}
