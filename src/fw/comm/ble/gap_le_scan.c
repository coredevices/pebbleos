/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "bluetooth/gap_le_scan.h"
#include "kernel/pbl_malloc.h"
#include "comm/bt_lock.h"
#include "gap_le_scan.h"
#include "kernel/events.h"
#include "system/passert.h"
#include "pbl/util/circular_buffer.h"
#include "pbl/util/likely.h"

// -----------------------------------------------------------------------------
// Static Variables -- MUST be protected with bt_lock/unlock!

//! The current scanning state of the controller.
static bool s_is_scanning;

//! The backing array for the circular buffer of reports to be processed.
static uint8_t *s_reports_buffer;

//! The circular buffer that tracks reports to be processed.
//! [MT] Currently, there is only one potential client that reads from the
//! buffer (the app). In the future, I can imagine the kernel also wants to
//! the scan at the same time. When that happens, we need to keep a cursor for
//! each client.
static CircularBuffer s_circular_buffer;

//! Flag that is set true when the cancelling of the timer was attempted, but
//! did not succeed. If true, the timer callback will not finalize the pending
//! report because it is not the report for which the timer was set initially.
//! @see gap_le_scan_get_dropped_reports_count
static uint32_t s_dropped_reports;

// -----------------------------------------------------------------------------
bool gap_le_start_scan(void) {
  bool success = false;
  bt_lock();
  {
    if (!s_is_scanning) {
      s_dropped_reports = 0;

      success = bt_driver_start_le_scan(
          true /* active_scan */, false /* use_white_list_filter */,
          true /* filter_dups */, 10240, 10240);

      if (success) {
        // Allocate report buffers if advertising started successfully
        const size_t buffer_size = GAP_LE_SCAN_REPORTS_BUFFER_SIZE;
        s_reports_buffer = (uint8_t *) kernel_malloc_check(buffer_size);
        circular_buffer_init(&s_circular_buffer, s_reports_buffer, buffer_size);
        s_is_scanning = true;
      }
    }
  }
  bt_unlock();
  return success;
}

// -----------------------------------------------------------------------------
bool gap_le_stop_scan(void) {
  bool success = false;
  bt_lock();
  {
    if (s_is_scanning) {

      success = bt_driver_stop_le_scan();
      kernel_free(s_reports_buffer);
      s_reports_buffer = NULL;
      s_is_scanning = false;
    }
  }
  bt_unlock();
  return success;
}

// -----------------------------------------------------------------------------
bool gap_le_is_scanning(void) {
  bt_lock();
  const bool is_scanning = s_is_scanning;
  bt_unlock();
  return is_scanning;
}

// -----------------------------------------------------------------------------
//! Copies over the pending report to the circular buffer and free the pending
//! "slot". In case there is no space left, the pending report will be dropped.
//! and a counter will be incremented
void bt_driver_cb_le_scan_handle_report(const GAPLERawAdReport *report_buffer, int length) {
  const bool written = circular_buffer_write(
      &s_circular_buffer, (uint8_t *)report_buffer, length);

  if (!written) {
    ++s_dropped_reports;
  } else { // notify clients there's a new event available
    PebbleEvent e = {
      .type = PEBBLE_BLE_SCAN_EVENT,
    };
    event_put(&e);
  }
}

// -----------------------------------------------------------------------------
bool gap_le_consume_scan_results(uint8_t *buffer, uint16_t *size_in_out) {
  // The number of bytes left to read:
  uint16_t read_space;
  // The space left in the output buffer:
  uint16_t write_space = *size_in_out;
  bt_lock();
  {
    if (UNLIKELY(!s_is_scanning)) {
      // Return, the buffers are deallocated by now already...
      *size_in_out = 0;
      bt_unlock();
      return false;
    }

    // We can't just copy over up to the maximum buffer size, because we could
    // end up with half reports.

    read_space = circular_buffer_get_read_space_remaining(&s_circular_buffer);

    // While there are reports to read and there is enough space for at least
    // the GAPLERawAdReport header:
    while (read_space && write_space >= sizeof(GAPLERawAdReport)) {
      // First copy the header. We know for sure this will fit into buffer,
      // because it was tested in the while() condition:
      circular_buffer_copy(&s_circular_buffer,
                           buffer, sizeof(GAPLERawAdReport));
      const GAPLERawAdReport *report = (const GAPLERawAdReport *) buffer;

      // Now use the copied header to figure out how big the report actually is:
      const uint32_t payload_len = report->payload.ad_data_length +
                                   report->payload.scan_resp_data_length;
      const uint32_t report_len = sizeof(GAPLERawAdReport) + payload_len;

      // There should always be at least enough bytes to read in the circular
      // read the length of report, otherwise there's an internal inconsistency.
      PBL_ASSERTN(read_space >= report_len);

      if (report_len <= write_space) {
        // Mark the bytes of the header as "consumed", as it's already copied:
        circular_buffer_consume(&s_circular_buffer, sizeof(GAPLERawAdReport));
        buffer += sizeof(GAPLERawAdReport);

        // Now copy the payload:
        circular_buffer_copy(&s_circular_buffer, buffer, payload_len);
        circular_buffer_consume(&s_circular_buffer, payload_len);
        buffer += payload_len;

        // Update the counters:
        write_space -= report_len;
        read_space -= report_len;
      } else {
        // No more space in out buffer.
        // The header might have been copied, but this has not been counted
        // towards the "bytes copied" that is reported back to the client.
        break;
      }
    }

    // Out: number of bytes copied:
    *size_in_out -= write_space;
  }
  bt_unlock();
  return (read_space != 0);
}

// -----------------------------------------------------------------------------
void gap_le_scan_init(void) {
  bt_lock();
  s_is_scanning = false;
  bt_unlock();
}


// -----------------------------------------------------------------------------
void gap_le_scan_deinit(void) {
  bt_lock();
  if (s_is_scanning) {
    gap_le_stop_scan();
    s_is_scanning = false;
  }
  bt_unlock();
}

// For UNIT Tests
uint32_t gap_le_scan_get_dropped_reports_count(void) {
  return s_dropped_reports;
}
