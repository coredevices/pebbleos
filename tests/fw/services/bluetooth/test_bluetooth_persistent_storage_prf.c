/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include <bluetooth/bonding_sync.h>
#include <bluetooth/gap_le_connect.h>

#include "pbl/services/analytics/analytics.h"
#include "pbl/services/bluetooth/bluetooth_persistent_storage.h"
#include "pbl/services/shared_prf_storage/shared_prf_storage.h"

#include "pbl/services/system_task.h"
#include "flash_region/flash_region.h"
#include "pbl/util/size.h"

typedef struct GAPLEConnection GAPLEConnection;

#include "fake_bonding_sync.h"
#include "fake_new_timer.h"
#include "fake_pbl_malloc.h"
#include "fake_spi_flash.h"
#include "fake_regular_timer.h"

#include "stubs_bluetopia_interface.h"
#include "stubs_bt_lock.h"
#include "stubs_gap_le_advert.h"
#include "stubs_bluetooth_analytics.h"
#include "stubs_gatt_client_discovery.h"
#include "stubs_gatt_client_subscriptions.h"
#include "stubs_hexdump.h"
#include "stubs_hexdump.h"
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_pebble_pairing_service.h"

typedef bool (*BondingSyncFilterCb)(const BleBonding *bonding, void *ctx);
const BleBonding *bonding_sync_find(BondingSyncFilterCb cb, void *ctx) {
  return NULL;
}

void bt_driver_pebble_pairing_service_handle_status_change(const GAPLEConnection *connection) {
}

bool bt_ctl_is_bluetooth_running(void) {
  return true;
}

void bt_driver_handle_le_conn_params_update_event(
    const BleConnectionUpdateCompleteEvent *event) {
}

typedef struct PairingUserConfirmationCtx PairingUserConfirmationCtx;

void bt_driver_cb_pairing_confirm_handle_request(const PairingUserConfirmationCtx *ctx,
                                                 const char *device_name,
                                                 const char *confirmation_token) {
}

void bt_driver_cb_pairing_confirm_handle_completed(const PairingUserConfirmationCtx *ctx,
                                                   bool success) {
}

void bt_local_addr_handle_bonding_change(BTBondingID bonding, BtPersistBondingOp op) {
}

// The v3 shared_prf_storage implementation (promoted to the sole implementation in commit
// 990d9a37d, which removed the old v2_sprf) writes to flash synchronously inside
// shared_prf_storage_store_*(), rather than deferring the write to a regular "writeback" timer as
// v2 did. There is therefore no shared_prf_storage_get_writeback_timer() to fire: the pairing data
// is already persisted by the time each store call returns, so the read-back assertions below
// exercise the same persistence contract directly.

static int s_bonding_change_count;
static BtPersistBondingOp s_bonding_change_ops[2];
void kernel_le_client_handle_bonding_change(BTBondingID bonding, BtPersistBondingOp op) {
  if (s_bonding_change_count <= ARRAY_LENGTH(s_bonding_change_ops)) {
    s_bonding_change_ops[s_bonding_change_count] = op;
  }
  ++s_bonding_change_count;
}

static void prv_reset_change_op_tracking(void) {
  s_bonding_change_count = 0;
  for (int i = 0; i < ARRAY_LENGTH(s_bonding_change_ops); ++i) {
    s_bonding_change_ops[i] = BtPersistBondingOpInvalid;
  }
}

void gap_le_connect_handle_bonding_change(BTBondingID bonding_id, BtPersistBondingOp op) {
}

void gap_le_connection_handle_bonding_change(BTBondingID bonding, BtPersistBondingOp op) {
}

void gap_le_device_name_request(uintptr_t stack_id, GAPLEConnection *connection) {
}

uint16_t gaps_get_starting_att_handle(void) {
  return 4;
}

void gatt_service_changed_server_cleanup_by_connection(GAPLEConnection *connection) {
}

void bt_pairability_update_due_to_bonding_change(void) {
}

void launcher_task_add_callback(void (*callback)(void *data), void *data) {
  callback(data);
}

bool system_task_add_callback(SystemTaskEventCallback cb, void *data) {
  cb(data);
  return true;
}

// Tests
///////////////////////////////////////////////////////////

void test_bluetooth_persistent_storage_prf__initialize(void) {
  bonding_sync_init();
  prv_reset_change_op_tracking();
  fake_spi_flash_init(FLASH_REGION_SHARED_PRF_STORAGE_BEGIN,
                      FLASH_REGION_SHARED_PRF_STORAGE_END - FLASH_REGION_SHARED_PRF_STORAGE_BEGIN);
  shared_prf_storage_init();
  bt_persistent_storage_init();
}

void test_bluetooth_persistent_storage_prf__cleanup(void) {
  fake_spi_flash_cleanup();
  bonding_sync_deinit();
}

void test_bluetooth_persistent_storage_prf__ble_store_and_get(void) {
  bool ret;

  // Output variables
  SMIdentityResolvingKey irk_out;
  BTDeviceInternal device_out;

  // Store a new pairing
  SMPairingInfo pairing_1 = (SMPairingInfo) {
    .irk = (SMIdentityResolvingKey) {
      .data = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00
      },
    },
    .identity = (BTDeviceInternal) {
      .address = (BTDeviceAddress) {
        .octets = {
          0x11, 0x12, 0x13, 0x14, 0x15, 0x16
        },
      },
      .is_classic = false,
      .is_random_address = false,
    },
    .is_remote_identity_info_valid = true,
  };
  BTBondingID id_1 = bt_persistent_storage_store_ble_pairing(&pairing_1, true /* is_gateway */,
                                                             NULL,
                                                             false /* requires_address_pinning */,
                                                             false /* auto_accept_re_pairing */);  cl_assert(id_1 != BT_BONDING_ID_INVALID);
  cl_assert_equal_i(s_bonding_change_count, 1);
  cl_assert_equal_i(s_bonding_change_ops[0], BtPersistBondingOpDidAdd);

  // Read it back
  ret = bt_persistent_storage_get_ble_pairing_by_id(id_1, &irk_out, &device_out, NULL /* name */);
  cl_assert(ret);
  cl_assert_equal_m(&irk_out, &pairing_1.irk, sizeof(irk_out));
  cl_assert_equal_m(&device_out, &pairing_1.identity, sizeof(device_out));

  // Re-pair device 1 again:
  // In case the device is the same as the existing pairing, make sure the operation is "change"
  // and not "delete"  to avoid disconnecting just because the existing pairing is deleted.
  // For bug details see https://pebbletechnology.atlassian.net/browse/PBL-24690
  prv_reset_change_op_tracking();
  id_1 = bt_persistent_storage_store_ble_pairing(&pairing_1, true /* is_gateway */, NULL,
                                                 false /* requires_address_pinning */,
                                                 false /* auto_accept_re_pairing */);  cl_assert(id_1 != BT_BONDING_ID_INVALID);
  cl_assert_equal_i(s_bonding_change_count, 1);
  cl_assert_equal_i(s_bonding_change_ops[0], BtPersistBondingOpDidChange);

  // Store another pairing (different device):
  SMPairingInfo pairing_2 = (SMPairingInfo) {
    .irk = (SMIdentityResolvingKey) {
      .data = {
        0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x08,
        0x09, 0x02, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x20
      },
    },
    .identity = (BTDeviceInternal) {
                 .address = (BTDeviceAddress) {
                   .octets = {
                     0x21, 0x22, 0x13, 0x14, 0x15, 0x26
                   },
                 },
      .is_classic = false,
      .is_random_address = false,
    },
    .is_remote_identity_info_valid = true,
  };
  prv_reset_change_op_tracking();
  BTBondingID id_2 = bt_persistent_storage_store_ble_pairing(&pairing_2, true /* is_gateway */,
                                                             NULL,
                                                             false /* requires_address_pinning */,
                                                             false /* auto_accept_re_pairing */);  cl_assert(id_2 != BT_BONDING_ID_INVALID);
  cl_assert_equal_i(s_bonding_change_count, 2);
  cl_assert_equal_i(s_bonding_change_ops[0], BtPersistBondingOpWillDelete);
  cl_assert_equal_i(s_bonding_change_ops[1], BtPersistBondingOpDidAdd);

  // Read it back
  ret = bt_persistent_storage_get_ble_pairing_by_id(id_2, &irk_out, &device_out, NULL /* name */);
  cl_assert(ret);
  cl_assert_equal_m(&irk_out, &pairing_2.irk, sizeof(irk_out));
  cl_assert_equal_m(&device_out, &pairing_2.identity, sizeof(device_out));

  // Store another pairing, this time it isn't a gateway
  SMPairingInfo pairing_3 = (SMPairingInfo) {
    .irk = (SMIdentityResolvingKey) {
      .data = {
        0x33, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x08,
        0x39, 0x02, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x20
      },
    },
    .identity = (BTDeviceInternal) {
                 .address = (BTDeviceAddress) {
                   .octets = {
                     0x33, 0x22, 0x13, 0x14, 0x15, 0x26
                   },
                 },
      .is_classic = false,
      .is_random_address = false,
    },
    .is_remote_identity_info_valid = true,
  };
  prv_reset_change_op_tracking();
  BTBondingID id_3 = bt_persistent_storage_store_ble_pairing(&pairing_2, false /* is_gateway */,
                                                             NULL,
                                                             false /* requires_address_pinning */,
                                                             false /* auto_accept_re_pairing */);  cl_assert(id_3 == BT_BONDING_ID_INVALID);
  cl_assert_equal_i(s_bonding_change_count, 0);

  // Read out the stored pairing (id_2 should still be stored)
  ret = bt_persistent_storage_get_ble_pairing_by_id(id_1, &irk_out, &device_out, NULL /* name */);
  cl_assert(ret);
  cl_assert_equal_m(&irk_out, &pairing_2.irk, sizeof(irk_out));
  cl_assert_equal_m(&device_out, &pairing_2.identity, sizeof(device_out));

  bt_persistent_storage_register_existing_ble_bondings();
  cl_assert_equal_b(bonding_sync_contains_pairing_info(&pairing_1, true), false);
  cl_assert_equal_b(bonding_sync_contains_pairing_info(&pairing_2, true), true);
  cl_assert_equal_b(bonding_sync_contains_pairing_info(&pairing_3, false), false);
}

void test_bluetooth_persistent_storage_prf__get_ble_by_address(void) {
  bool ret;

  // Output variables
  SMIdentityResolvingKey irk_out;

  // Store a pairing
  SMPairingInfo pairing = (SMPairingInfo) {
    .irk = (SMIdentityResolvingKey) {
      .data = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00,
      },
    },
    .identity = (BTDeviceInternal) {
      .address = (BTDeviceAddress) {
        .octets = {
          0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
        },
      },
      .is_classic = false,
      .is_random_address = false,
    },
    .is_remote_identity_info_valid = true,
  };

  BTBondingID id = bt_persistent_storage_store_ble_pairing(&pairing, true /* is_gateway */, NULL,
                                                           false /* requires_address_pinning */,
                                                           false /* auto_accept_re_pairing */);  cl_assert(id != BT_BONDING_ID_INVALID);

  // Read it back
  ret = bt_persistent_storage_get_ble_pairing_by_addr(&pairing.identity, &irk_out, NULL);
  cl_assert(ret);
  cl_assert_equal_m(&irk_out, &pairing.irk, sizeof(irk_out));
}


void test_bluetooth_persistent_storage_prf__delete_ble_pairing_by_id(void) {
  bool ret;

  // Output variables
  SMIdentityResolvingKey irk_out;
  BTDeviceInternal device_out;

  // Store a pairing
  SMPairingInfo pairing = (SMPairingInfo) {
    .irk = (SMIdentityResolvingKey) {
      .data = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00
      },
    },
    .identity = (BTDeviceInternal) {
                 .address = (BTDeviceAddress) {
                   .octets = {
                     0x11, 0x12, 0x13, 0x14, 0x15, 0x16
                   },
                 },
      .is_classic = false,
      .is_random_address = false,
    },
    .is_remote_identity_info_valid = true,
  };

  BleBonding ble_bonding = (BleBonding) {
    .is_gateway = true,
    .pairing_info = pairing,
  };
  bonding_sync_add_bonding(&ble_bonding);
  BTBondingID id = bt_persistent_storage_store_ble_pairing(&pairing, true /* is_gateway */, NULL,
                                                           false /* requires_address_pinning */,
                                                           false /* auto_accept_re_pairing */);  cl_assert(id != BT_BONDING_ID_INVALID);

  // Delete the Pairing
  bt_persistent_storage_delete_ble_pairing_by_id(id);

  // Try to read it back
  ret = bt_persistent_storage_get_ble_pairing_by_id(id, &irk_out, &device_out, NULL);
  cl_assert(!ret);

  // Add the pairing again
  id = bt_persistent_storage_store_ble_pairing(&pairing, true /* is_gateway */, NULL,
                                               false /* requires_address_pinning */,
                                               false /* auto_accept_re_pairing */);  cl_assert(id != BT_BONDING_ID_INVALID);
}

void test_bluetooth_persistent_storage_prf__delete_ble_pairing_by_addr_if_matches(void) {
  SMIdentityResolvingKey irk_out;
  BTDeviceInternal device_out;

  SMPairingInfo pairing = (SMPairingInfo) {
    .irk = (SMIdentityResolvingKey) {
      .data = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00
      },
    },
    .identity = (BTDeviceInternal) {
      .address = (BTDeviceAddress) {
        .octets = {
          0x11, 0x12, 0x13, 0x14, 0x15, 0x16
        },
      },
      .is_classic = false,
      .is_random_address = false,
    },
    .remote_encryption_info = {
      .ltk = (SMLongTermKey) {
        .data = {
          0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
          0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf
        },
      },
      .ediv = 0x1234,
      .rand = 0x0123456789abcdefULL,
    },
    .is_remote_encryption_info_valid = true,
    .is_remote_identity_info_valid = true,
  };

  BleBonding ble_bonding = (BleBonding) {
    .is_gateway = true,
    .pairing_info = pairing,
  };
  bonding_sync_add_bonding(&ble_bonding);
  BTBondingID id = bt_persistent_storage_store_ble_pairing(&pairing, true /* is_gateway */, NULL,
                                                           false /* requires_address_pinning */,
                                                           0 /* flags */);
  cl_assert(id != BT_BONDING_ID_INVALID);

  // A delete carrying key material we never stored must leave the pairing alone,
  // in shared PRF storage and in the driver's copy both.
  SMPairingInfo other_keys = pairing;
  other_keys.remote_encryption_info.ltk.data[0] = 0xff;
  cl_assert_equal_b(bt_persistent_storage_delete_ble_pairing_by_addr_if_matches(&pairing.identity,
                                                                                &other_keys),
                    false);
  cl_assert_equal_b(bt_persistent_storage_get_ble_pairing_by_id(id, &irk_out, &device_out, NULL),
                    true);
  cl_assert_equal_b(bonding_sync_contains_pairing_info(&pairing, true /* is_gateway */), true);

  // Re-pairing the same phone keeps its identity and its IRK, so the new keys land
  // in this one slot. A delete queued for the keys they replaced must not take it.
  SMPairingInfo repaired = pairing;
  repaired.remote_encryption_info.ltk.data[0] = 0x5a;
  cl_assert(bt_persistent_storage_store_ble_pairing(&repaired, true /* is_gateway */, NULL,
                                                    false /* requires_address_pinning */,
                                                    0 /* flags */) != BT_BONDING_ID_INVALID);
  cl_assert_equal_b(bt_persistent_storage_delete_ble_pairing_by_addr_if_matches(&pairing.identity,
                                                                                &pairing),
                    false);
  cl_assert_equal_b(bt_persistent_storage_get_ble_pairing_by_id(id, &irk_out, &device_out, NULL),
                    true);

  // The keys that are actually stored do delete it.
  cl_assert_equal_b(bt_persistent_storage_delete_ble_pairing_by_addr_if_matches(&pairing.identity,
                                                                                &repaired),
                    true);
  cl_assert_equal_b(bt_persistent_storage_get_ble_pairing_by_id(id, &irk_out, &device_out, NULL),
                    false);
  cl_assert_equal_b(bonding_sync_contains_pairing_info(&pairing, true /* is_gateway */), false);
}


