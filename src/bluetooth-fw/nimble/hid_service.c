/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "hid_service.h"

#include <bluetooth/hid_service.h>

#ifdef CONFIG_BT_HID_REMOTE

#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_hs.h>
#include <host/ble_uuid.h>
#include <os/os_mbuf.h>
#include <pbl/logging/logging.h>
#include <system/passert.h>

#include "nimble_type_conversions.h"

PBL_LOG_MODULE_DECLARE(bt, CONFIG_BT_LOG_LEVEL);

#define HID_SERVICE_UUID              (0x1812)
#define HID_CHR_HID_INFO_UUID         (0x2A4A)
#define HID_CHR_REPORT_MAP_UUID       (0x2A4B)
#define HID_CHR_CONTROL_POINT_UUID    (0x2A4C)
#define HID_CHR_REPORT_UUID           (0x2A4D)
#define HID_CHR_PROTOCOL_MODE_UUID    (0x2A4E)
#define HID_DSC_REPORT_REFERENCE_UUID (0x2908)

// Report Protocol; the boot protocol is intentionally not supported.
#define HID_PROTOCOL_MODE_REPORT (0x01)

// Report 1's descriptor bytes are cached by every bonded phone, so the seven
// usages it declares are frozen. Each must also stay inside one byte, which is
// the Usage item width the report map below uses.
_Static_assert(BLE_HID_USAGE_VOLUME_UP == 0xE9, "report 1 usage changed");
_Static_assert(BLE_HID_USAGE_VOLUME_DOWN == 0xEA, "report 1 usage changed");
_Static_assert(BLE_HID_USAGE_PLAY_PAUSE == 0xCD, "report 1 usage changed");
_Static_assert(BLE_HID_USAGE_NEXT_TRACK == 0xB5, "report 1 usage changed");
_Static_assert(BLE_HID_USAGE_PREV_TRACK == 0xB6, "report 1 usage changed");
_Static_assert(BLE_HID_USAGE_MUTE == 0xE2, "report 1 usage changed");
_Static_assert(BLE_HID_USAGE_SNAPSHOT == 0x65, "report 1 usage changed");

// Consumer Control only: a Keyboard collection would make iOS hide the
// on-screen keyboard. Report 1 is a bitfield of named usages; report 2 is a
// generic usage selector, appended to the same collection so every byte of
// report 1 stays exactly as the phones already cached it.
static const uint8_t s_hid_report_map[] = {
    0x05, 0x0C,                            // Usage Page (Consumer)
    0x09, 0x01,                            // Usage (Consumer Control)
    0xA1, 0x01,                            // Collection (Application)
    0x85, 0x01,                            //   Report ID (1)
    0x15, 0x00,                            //   Logical Minimum (0)
    0x25, 0x01,                            //   Logical Maximum (1)
    0x75, 0x01,                            //   Report Size (1)
    0x95, 0x07,                            //   Report Count (7)
    0x09, BLE_HID_USAGE_VOLUME_UP,         //   Usage (Volume Increment)  -> bit 0
    0x09, BLE_HID_USAGE_VOLUME_DOWN,       //   Usage (Volume Decrement)  -> bit 1
    0x09, BLE_HID_USAGE_PLAY_PAUSE,        //   Usage (Play/Pause)        -> bit 2
    0x09, BLE_HID_USAGE_NEXT_TRACK,        //   Usage (Scan Next Track)   -> bit 3
    0x09, BLE_HID_USAGE_PREV_TRACK,        //   Usage (Scan Prev Track)   -> bit 4
    0x09, BLE_HID_USAGE_MUTE,              //   Usage (Mute)              -> bit 5
    0x09, BLE_HID_USAGE_SNAPSHOT,          //   Usage (Snapshot)          -> bit 6
    0x81, 0x02,                            //   Input (Data, Var, Abs)
    0x95, 0x01,                            //   Report Count (1)
    0x81, 0x03,                            //   Input (Const, Var, Abs)   padding to a byte
    0x85, 0x02,                            //   Report ID (2)
    // An Array field reports an index, resolved as
    // Usage Minimum + (value - Logical Minimum), so the two minima must match
    // for a raw usage to mean itself. Both are 1, which also puts 0 outside the
    // logical range: HID 1.11 6.2.2.5 makes that "no controls asserted", which
    // is what a release is.
    0x15, 0x01,                            //   Logical Minimum (1)
    0x26, 0xFF, 0x03,                      //   Logical Maximum (0x3FF)
    0x19, 0x01,                            //   Usage Minimum (0x01)
    0x2A, 0xFF, 0x03,                      //   Usage Maximum (0x3FF)
    0x75, 0x10,                            //   Report Size (16)
    0x95, 0x01,                            //   Report Count (1)
    0x81, 0x00,                            //   Input (Data, Array, Abs)
    0xC0
};

// bcdHID 0x0111 (LE), country code 0, flags = NormallyConnectable.
static const uint8_t s_hid_information[] = {0x11, 0x01, 0x00, 0x02};

// Report Reference: Report ID 1, type Input.
static const uint8_t s_report_reference[] = {0x01, 0x01};

// Report Reference: Report ID 2, type Input.
static const uint8_t s_usage_report_reference[] = {0x02, 0x01};

static uint16_t s_report_val_handle;
static uint16_t s_usage_report_val_handle;
//! One global subscription pair per report, not an array:
//! MYNEWT_VAL_BLE_MAX_CONNECTIONS is 1, so there can only ever be one subscriber.
static uint16_t s_subscribed_conn_handle;
static uint16_t s_usage_subscribed_conn_handle;
static bool s_is_subscribed;
static bool s_usage_is_subscribed;
static uint8_t s_last_report;
//! The usage report 2 last carried. Held as one 16-bit word rather than the two
//! wire bytes so the read handler, which runs on NimbleHost, cannot catch a
//! half-written value from the KernelBG sender.
static uint16_t s_last_usage;

static int prv_access_protocol_mode(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg) {
  switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR: {
      const uint8_t mode = HID_PROTOCOL_MODE_REPORT;
      return (os_mbuf_append(ctxt->om, &mode, sizeof(mode)) == 0) ? 0
                                                                 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
      // Accept and ignore: we always stay in report protocol mode.
      return 0;
    default:
      return BLE_ATT_ERR_UNLIKELY;
  }
}

static int prv_access_report_map(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
    return BLE_ATT_ERR_UNLIKELY;
  }
  return (os_mbuf_append(ctxt->om, s_hid_report_map, sizeof(s_hid_report_map)) == 0)
             ? 0
             : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int prv_access_report(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
    return BLE_ATT_ERR_UNLIKELY;
  }
  return (os_mbuf_append(ctxt->om, &s_last_report, sizeof(s_last_report)) == 0)
             ? 0
             : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int prv_access_usage_report(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
    return BLE_ATT_ERR_UNLIKELY;
  }
  // Report Size 16, Report Count 1, little-endian on the wire.
  const uint16_t usage = s_last_usage;
  const uint8_t report[2] = {(uint8_t)(usage & 0xFF), (uint8_t)(usage >> 8)};
  return (os_mbuf_append(ctxt->om, report, sizeof(report)) == 0) ? 0
                                                                 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int prv_access_report_reference(uint16_t conn_handle, uint16_t attr_handle,
                                       struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_DSC) {
    return BLE_ATT_ERR_UNLIKELY;
  }
  return (os_mbuf_append(ctxt->om, s_report_reference, sizeof(s_report_reference)) == 0)
             ? 0
             : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int prv_access_usage_report_reference(uint16_t conn_handle, uint16_t attr_handle,
                                             struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_DSC) {
    return BLE_ATT_ERR_UNLIKELY;
  }
  return (os_mbuf_append(ctxt->om, s_usage_report_reference, sizeof(s_usage_report_reference)) == 0)
             ? 0
             : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int prv_access_hid_information(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
    return BLE_ATT_ERR_UNLIKELY;
  }
  return (os_mbuf_append(ctxt->om, s_hid_information, sizeof(s_hid_information)) == 0)
             ? 0
             : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int prv_access_control_point(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg) {
  // Accept and ignore suspend / exit-suspend.
  return (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) ? 0 : BLE_ATT_ERR_UNLIKELY;
}

// Deliberate HOGP deviation: only Report Map and Report carry READ_ENC.
// BLE_SM_SC_ONLY with BLE_SM_LVL 4 escalates any security flag to
// LESC+authenticated+16-byte key, and iOS reads HID Information before
// encryption is up, so gating the rest would break the pairing flow.
static const struct ble_gatt_svc_def s_hid_svc[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(HID_SERVICE_UUID),
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = BLE_UUID16_DECLARE(HID_CHR_PROTOCOL_MODE_UUID),
                    .access_cb = prv_access_protocol_mode,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP,
                },
                {
                    .uuid = BLE_UUID16_DECLARE(HID_CHR_REPORT_MAP_UUID),
                    .access_cb = prv_access_report_map,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
                },
                {
                    .uuid = BLE_UUID16_DECLARE(HID_CHR_REPORT_UUID),
                    .access_cb = prv_access_report,
                    // READ_ENC also gates the CCCD, so the peer must bond
                    // before it can subscribe. NimBLE adds the CCCD itself.
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY |
                             BLE_GATT_CHR_F_READ_ENC,
                    .val_handle = &s_report_val_handle,
                    .descriptors =
                        (struct ble_gatt_dsc_def[]){
                            {
                                .uuid = BLE_UUID16_DECLARE(HID_DSC_REPORT_REFERENCE_UUID),
                                .att_flags = BLE_ATT_F_READ,
                                .access_cb = prv_access_report_reference,
                            },
                            {
                                0,
                            },
                        },
                },
                {
                    .uuid = BLE_UUID16_DECLARE(HID_CHR_HID_INFO_UUID),
                    .access_cb = prv_access_hid_information,
                    .flags = BLE_GATT_CHR_F_READ,
                },
                {
                    .uuid = BLE_UUID16_DECLARE(HID_CHR_CONTROL_POINT_UUID),
                    .access_cb = prv_access_control_point,
                    .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
                },
                {
                    // Appended last for the same reason the service is
                    // registered last: every handle before it keeps its value.
                    .uuid = BLE_UUID16_DECLARE(HID_CHR_REPORT_UUID),
                    .access_cb = prv_access_usage_report,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY |
                             BLE_GATT_CHR_F_READ_ENC,
                    .val_handle = &s_usage_report_val_handle,
                    .descriptors =
                        (struct ble_gatt_dsc_def[]){
                            {
                                .uuid = BLE_UUID16_DECLARE(HID_DSC_REPORT_REFERENCE_UUID),
                                .att_flags = BLE_ATT_F_READ,
                                .access_cb = prv_access_usage_report_reference,
                            },
                            {
                                0,
                            },
                        },
                },
                {
                    0,
                },
            },
    },
    {
        0,
    },
};

static void prv_handle_subscribe_event(struct ble_gap_event *event) {
  // A zeroed handle is hid_service_deinit()'s, never a real attribute's, so it
  // must not match either characteristic.
  const uint16_t attr_handle = event->subscribe.attr_handle;
  const bool is_report = (s_report_val_handle != 0) && (attr_handle == s_report_val_handle);
  const bool is_usage_report =
      (s_usage_report_val_handle != 0) && (attr_handle == s_usage_report_val_handle);
  if (!is_report && !is_usage_report) {
    return;
  }
  if (event->subscribe.cur_notify == event->subscribe.prev_notify) {
    return;
  }

  // Drop our view of the subscription first: on a BLE_GAP_SUBSCRIBE_REASON_TERM
  // the connection is often already gone, and bailing out with the flag still
  // set would leave us claiming to be subscribed over a dead link.
  const bool is_subscribed = (event->subscribe.cur_notify != 0);
  if (is_report) {
    s_is_subscribed = is_subscribed;
    s_subscribed_conn_handle = event->subscribe.conn_handle;
    if (!is_subscribed) {
      s_last_report = 0;
    }
  } else {
    s_usage_is_subscribed = is_subscribed;
    s_usage_subscribed_conn_handle = event->subscribe.conn_handle;
    if (!is_subscribed) {
      s_last_usage = 0;
    }
  }

  // A zeroed device rather than no call at all: on a
  // BLE_GAP_SUBSCRIBE_REASON_TERM the link is usually already gone, and the FW
  // has to hear that the subscription ended or it keeps a usage held until its
  // own 10 s timeout and answers E_BUSY to every press in between. The one
  // implementation of this callback only reads is_subscribed.
  BTDeviceInternal device = {};
  struct ble_gap_conn_desc desc;
  if (ble_gap_conn_find(event->subscribe.conn_handle, &desc) == 0) {
    nimble_addr_to_pebble_device(&desc.peer_id_addr, &device);
  } else {
    PBL_LOG_DBG("HID subscribe: no conn descriptor for handle %u", event->subscribe.conn_handle);
  }
  bt_driver_cb_hid_service_update_subscription(&device, is_subscribed);
}

static int prv_handle_gap_event(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
    case BLE_GAP_EVENT_SUBSCRIBE:
      prv_handle_subscribe_event(event);
      break;
    default:
      break;
  }
  return 0;
}

static struct ble_gap_event_listener s_gap_event_listener;

void hid_service_init(void) {
  // ble_gatts_start() fills the handles back in; clearing them keeps init and
  // deinit symmetric so a stale handle cannot survive a stack restart.
  s_report_val_handle = 0;
  s_usage_report_val_handle = 0;
  s_subscribed_conn_handle = 0;
  s_usage_subscribed_conn_handle = 0;
  s_is_subscribed = false;
  s_usage_is_subscribed = false;
  s_last_report = 0;
  s_last_usage = 0;

  int rc = ble_gatts_count_cfg(s_hid_svc);
  PBL_ASSERTN(rc == 0);
  rc = ble_gatts_add_svcs(s_hid_svc);
  PBL_ASSERTN(rc == 0);
  // Called on every bt_driver_start, but the listener list is only cleared on
  // nimble_port_init, so tolerate EALREADY.
  rc = ble_gap_event_listener_register(&s_gap_event_listener, prv_handle_gap_event, NULL);
  PBL_ASSERTN(rc == 0 || rc == BLE_HS_EALREADY);
}

void hid_service_deinit(void) {
  // ble_gatts_reset() drops the handles, so none of this may outlive the stack.
  s_report_val_handle = 0;
  s_usage_report_val_handle = 0;
  s_subscribed_conn_handle = 0;
  s_usage_subscribed_conn_handle = 0;
  s_is_subscribed = false;
  s_usage_is_subscribed = false;
  s_last_report = 0;
  s_last_usage = 0;
}

bool bt_driver_is_hid_service_supported(void) { return true; }

bool bt_driver_hid_service_is_subscribed(void) { return s_is_subscribed; }

bool bt_driver_hid_service_send_consumer_report(uint8_t bits) {
  if (!s_is_subscribed) {
    return false;
  }

  struct os_mbuf *om = ble_hs_mbuf_from_flat(&bits, sizeof(bits));
  if (!om) {
    PBL_LOG_ERR("HID report 0x%02x: out of mbufs", bits);
    return false;
  }

  int rc = ble_gatts_notify_custom(s_subscribed_conn_handle, s_report_val_handle, om);
  // ble_gatts_notify_custom always consumes the mbuf, even on error.
  if (rc != 0) {
    PBL_LOG_ERR("HID report 0x%02x notify failed: 0x%04x", bits, (uint16_t)rc);
    return false;
  }

  // Only commit once the peer has been told, so a read of the Report
  // characteristic cannot report a state that was never notified.
  s_last_report = bits;
  return true;
}

bool bt_driver_hid_service_usage_is_subscribed(void) { return s_usage_is_subscribed; }

bool bt_driver_hid_service_send_consumer_usage(uint16_t usage) {
  if (usage > BLE_HID_USAGE_MAX) {
    PBL_LOG_ERR("HID usage 0x%04x is outside the declared range", usage);
    return false;
  }
  if (!s_usage_is_subscribed) {
    return false;
  }

  const uint8_t report[2] = {(uint8_t)(usage & 0xFF), (uint8_t)(usage >> 8)};
  struct os_mbuf *om = ble_hs_mbuf_from_flat(report, sizeof(report));
  if (!om) {
    PBL_LOG_ERR("HID usage 0x%03x: out of mbufs", usage);
    return false;
  }

  int rc = ble_gatts_notify_custom(s_usage_subscribed_conn_handle, s_usage_report_val_handle, om);
  // ble_gatts_notify_custom always consumes the mbuf, even on error.
  if (rc != 0) {
    PBL_LOG_ERR("HID usage 0x%03x notify failed: 0x%04x", usage, (uint16_t)rc);
    return false;
  }

  s_last_usage = usage;
  return true;
}

#endif  // CONFIG_BT_HID_REMOTE
