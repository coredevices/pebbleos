/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include <bluetooth/bonding_sync.h>
#include <bluetooth/gap_le_connect.h>

#include "pbl/services/analytics/analytics.h"
#include "pbl/services/bluetooth/bluetooth_persistent_storage.h"
#include "pbl/services/bluetooth/bluetooth_persistent_storage_unittest_impl.h"
#include "pbl/services/settings/settings_file.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/event_service.h"
#include "flash_region/flash_region.h"

// Stubs
////////////////////////////////////

typedef struct GAPLEConnection GAPLEConnection;

#include "fake_bonding_sync.h"
#include "fake_rtc.h"
#include "fake_spi_flash.h"
#include "fake_system_task.h"
#include "fake_events.h"
#include "fake_new_timer.h"
#include "fake_pbl_malloc.h"
#include "fake_shared_prf_storage.h"

#include "stubs_bluetopia_interface.h"
#include "stubs_bluetooth_persistent_storage_debug.h"
#include "stubs_bt_lock.h"
#include "stubs_gap_le_advert.h"
#include "stubs_gatt_client_discovery.h"
#include "stubs_gatt_client_subscriptions.h"
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_pebble_pairing_service.h"
#include "stubs_print.h"
#include "stubs_prompt.h"
#include "stubs_regular_timer.h"
#include "stubs_serial.h"
#include "stubs_sleep.h"
#include "stubs_system_reset.h"
#include "stubs_task_watchdog.h"

static int s_ble_bonding_change_add_count;
static int s_ble_bonding_change_update_count;
static int s_ble_bonding_change_delete_count;

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

void gap_le_connect_handle_bonding_change(BTBondingID bonding_id, BtPersistBondingOp op) {
}

void gap_le_connection_handle_bonding_change(BTBondingID bonding, BtPersistBondingOp op) {
}

void gap_le_device_name_request(uintptr_t stack_id, GAPLEConnection *connection) {
}

void bt_pairability_update_due_to_bonding_change(void) {
}

void bt_local_addr_handle_bonding_change(BTBondingID bonding, BtPersistBondingOp op) {
}

void kernel_le_client_handle_bonding_change(BTBondingID bonding, BtPersistBondingOp op) {
  if (op == BtPersistBondingOpDidAdd) {
    s_ble_bonding_change_add_count++;
  } else if (op == BtPersistBondingOpDidChange) {
    s_ble_bonding_change_update_count++;
  } else if (op == BtPersistBondingOpWillDelete) {
    s_ble_bonding_change_delete_count++;
  }
  return;
}

uint16_t gaps_get_starting_att_handle(void) {
  return 4;
}

void gatt_service_changed_server_cleanup_by_connection(GAPLEConnection *connection) {
}

void launcher_task_add_callback(void (*callback)(void *data), void *data) {
  callback(data);
}

bool launcher_task_is_current_task(void) {
  return true;
}

void bt_driver_handle_host_added_cccd(const BleCCCD *cccd) {
}

void bt_driver_handle_host_removed_cccd(const BleCCCD *cccd) {
}

void sys_pbl_analytics_set_unsigned(enum pbl_analytics_key key, uint32_t unsigned_value) {
}

// Tests
///////////////////////////////////////////////////////////

void test_bluetooth_persistent_storage__initialize(void) {
  bonding_sync_init();
  fake_spi_flash_init(0, 0x1000000);
  pfs_init(false);

  s_ble_bonding_change_add_count = 0;
  s_ble_bonding_change_update_count = 0;
  s_ble_bonding_change_delete_count = 0;

  fake_shared_prf_storage_reset_counts();

  bt_persistent_storage_init();
}

void test_bluetooth_persistent_storage__cleanup(void) {
  bonding_sync_deinit();
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//! BLE Pairing Info

void test_bluetooth_persistent_storage__ble_address_pinning(void) {
  cl_assert_equal_b(bt_persistent_storage_has_pinned_ble_pairings(), false);

  BTDeviceAddress address_out = {};
  cl_assert_equal_b(bt_persistent_storage_get_ble_pinned_address(&address_out), false);
  BTDeviceAddress address_out_expected = {};
  cl_assert_equal_m(&address_out_expected, &address_out, sizeof(address_out));

  BTDeviceAddress address = (BTDeviceAddress) {
    .octets = {
      0x11, 0x12, 0x13, 0x14, 0x15, 0x16
    },
  };
  cl_assert_equal_b(bt_persistent_storage_set_ble_pinned_address(&address), true);

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
    .is_mitm_protection_enabled = true,
  };

  BleBonding ble_bonding = (BleBonding) {
    .is_gateway = true,
    .pairing_info = pairing_1,
  };
  bonding_sync_add_bonding(&ble_bonding);
  BTBondingID id = bt_persistent_storage_store_ble_pairing(&pairing_1, true /* is_gateway */, NULL,
                                                           true /* requires_address_pinning */,
                                                           false /* auto_accept_re_pairing */);
  cl_assert(id != BT_BONDING_ID_INVALID);

  cl_assert_equal_b(bt_persistent_storage_has_pinned_ble_pairings(), true);

  bt_persistent_storage_delete_ble_pairing_by_id(id);

  cl_assert_equal_b(bt_persistent_storage_has_pinned_ble_pairings(), false);

  cl_assert_equal_b(bt_persistent_storage_set_ble_pinned_address(NULL), true);
  cl_assert_equal_b(bt_persistent_storage_get_ble_pinned_address(NULL), false);
}

void test_bluetooth_persistent_storage__ble_store_and_get(void) {
  bool ret;

  // Output variables
  SMIdentityResolvingKey irk_out;
  BTDeviceInternal device_out;

  // Store a new pairing
  SMPairingInfo pairing_1;
  memset(&pairing_1, 0x00, sizeof(pairing_1));
  pairing_1 = (SMPairingInfo) {
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
    .is_mitm_protection_enabled = true,
  };
  BTBondingID id_1 = bt_persistent_storage_store_ble_pairing(&pairing_1,
                                                             true /* is_gateway */, NULL,
                                                             false /* requires_address_pinning */,
                                                             false /* auto_accept_re_pairing */);
  cl_assert(id_1 != BT_BONDING_ID_INVALID);
  cl_assert_equal_i(s_ble_bonding_change_add_count, 1);
  cl_assert_equal_i(fake_shared_prf_storage_get_ble_store_count(), 1);
  cl_assert_equal_b(bt_persistent_storage_has_pinned_ble_pairings(), false);

  // Read it back
  ret = bt_persistent_storage_get_ble_pairing_by_id(id_1, &irk_out, &device_out, NULL /* name */);
  cl_assert(ret);
  cl_assert_equal_m(&irk_out, &pairing_1.irk, sizeof(irk_out));
  cl_assert_equal_m(&device_out, &pairing_1.identity, sizeof(device_out));

  // Store a pairing with a different identity. Only one BLE pairing is allowed at a time, so this
  // must replace the previous one.
  SMPairingInfo pairing_2;
  memset(&pairing_2, 0x00, sizeof(pairing_2));
  pairing_2 = (SMPairingInfo) {
    .irk = (SMIdentityResolvingKey) {
      .data = {
        0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x08,
        0x09, 0x02, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x20,
      },
    },
    .identity = (BTDeviceInternal) {
      .address = (BTDeviceAddress) {
        .octets = {
          0x21, 0x22, 0x13, 0x14, 0x15, 0x26,
        },
      },
      .is_classic = false,
      .is_random_address = false,
    },
    .is_remote_identity_info_valid = true,
  };
  BTBondingID id_2 = bt_persistent_storage_store_ble_pairing(&pairing_2, true /* is_gateway */,
                                                             NULL,
                                                             false /* requires_address_pinning */,
                                                             false /* auto_accept_re_pairing */);
  cl_assert(id_2 != BT_BONDING_ID_INVALID);
  cl_assert(id_2 != id_1);
  cl_assert_equal_i(s_ble_bonding_change_add_count, 2);
  // pairing_1 should have been removed automatically.
  cl_assert_equal_i(s_ble_bonding_change_delete_count, 1);

  // pairing_1 is gone, pairing_2 remains.
  ret = bt_persistent_storage_get_ble_pairing_by_id(id_1, &irk_out, &device_out, NULL /* name */);
  cl_assert(!ret);

  ret = bt_persistent_storage_get_ble_pairing_by_id(id_2, &irk_out, &device_out, NULL /* name */);
  cl_assert(ret);
  cl_assert_equal_m(&irk_out, &pairing_2.irk, sizeof(irk_out));
  cl_assert_equal_m(&device_out, &pairing_2.identity, sizeof(device_out));

  // Re-store the same pairing (same identity): this is an update, not an add, and must not delete
  // anything.
  BTBondingID id_X = bt_persistent_storage_store_ble_pairing(&pairing_2, true /* is_gateway */,
                                                             NULL,
                                                             false /* requires_address_pinning */,
                                                             false /* auto_accept_re_pairing */);
  cl_assert_equal_i(id_2, id_X);
  cl_assert_equal_i(s_ble_bonding_change_update_count, 1);
  cl_assert_equal_i(s_ble_bonding_change_delete_count, 1);

  ret = bt_persistent_storage_get_ble_pairing_by_id(id_2, &irk_out, &device_out, NULL /* name */);
  cl_assert(ret);
  cl_assert_equal_m(&irk_out, &pairing_2.irk, sizeof(irk_out));
  cl_assert_equal_m(&device_out, &pairing_2.identity, sizeof(device_out));

  // Store yet another distinct pairing: pairing_2 should be replaced.
  SMPairingInfo pairing_3;
  memset(&pairing_3, 0x00, sizeof(pairing_3));
  pairing_3 = (SMPairingInfo) {
    .irk = (SMIdentityResolvingKey) {
      .data = {
        0x91, 0x22, 0x73, 0x24, 0x25, 0x26, 0x27, 0x08,
        0x69, 0x02, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x99,
      },
    },
    .identity = (BTDeviceInternal) {
      .address = (BTDeviceAddress) {
        .octets = {
          0x29, 0x92, 0x13, 0x99, 0x15, 0x96,
        },
      },
      .is_classic = true,
      .is_random_address = true,
    },
    .is_remote_identity_info_valid = true,
  };
  BTBondingID id_3 = bt_persistent_storage_store_ble_pairing(&pairing_3, true /* is_gateway */,
                                                             NULL,
                                                             false /* requires_address_pinning */,
                                                             false /* auto_accept_re_pairing */);
  cl_assert(id_3 != BT_BONDING_ID_INVALID);
  cl_assert_equal_i(s_ble_bonding_change_add_count, 3);
  cl_assert_equal_i(s_ble_bonding_change_delete_count, 2);

  // Only pairing_3 should be findable by identity. Bonding slot IDs may be reused after a delete,
  // so check by address rather than by stale id values.
  ret = bt_persistent_storage_get_ble_pairing_by_addr(&pairing_1.identity, &irk_out, NULL);
  cl_assert(!ret);

  ret = bt_persistent_storage_get_ble_pairing_by_addr(&pairing_2.identity, &irk_out, NULL);
  cl_assert(!ret);

  ret = bt_persistent_storage_get_ble_pairing_by_id(id_3, &irk_out, &device_out, NULL /* name */);
  cl_assert(ret);
  cl_assert_equal_m(&irk_out, &pairing_3.irk, sizeof(irk_out));
  cl_assert_equal_m(&device_out, &pairing_3.identity, sizeof(device_out));

  bt_persistent_storage_register_existing_ble_bondings();
  cl_assert_equal_b(bonding_sync_contains_pairing_info(&pairing_1, true), false);
  cl_assert_equal_b(bonding_sync_contains_pairing_info(&pairing_2, true), false);
  cl_assert_equal_b(bonding_sync_contains_pairing_info(&pairing_3, true), true);
}

 void test_bluetooth_persistent_storage__get_ble_by_addr(void) {
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
                                                            false /* auto_accept_re_pairing */);
   cl_assert(id != BT_BONDING_ID_INVALID);

   // Read it back
   ret = bt_persistent_storage_get_ble_pairing_by_addr(&pairing.identity, &irk_out, NULL);
   cl_assert(ret);
   cl_assert_equal_m(&irk_out, &pairing.irk, sizeof(irk_out));
}


void test_bluetooth_persistent_storage__delete_ble_pairing_by_id(void) {
  bool ret;

  // Output variables
  SMIdentityResolvingKey irk_out;
  BTDeviceInternal device_out;

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

  BleBonding ble_bonding = (BleBonding) {
    .is_gateway = true,
    .pairing_info = pairing,
  };
  bonding_sync_add_bonding(&ble_bonding);
  BTBondingID id = bt_persistent_storage_store_ble_pairing(&pairing, true /* is_gateway */, NULL,
                                                           false /* requires_address_pinning */,
                                                           false /* auto_accept_re_pairing */);
  cl_assert(id != BT_BONDING_ID_INVALID);
  cl_assert_equal_i(s_ble_bonding_change_add_count, 1);
  cl_assert_equal_i(fake_shared_prf_storage_get_ble_store_count(), 1);
  cl_assert_equal_i(fake_shared_prf_storage_get_ble_delete_count(), 1);

  // Delete the Pairing
  bt_persistent_storage_delete_ble_pairing_by_id(id);
  cl_assert_equal_i(s_ble_bonding_change_delete_count, 1);
  cl_assert_equal_i(fake_shared_prf_storage_get_ble_delete_count(), 2);

  // Try to read it back
  ret = bt_persistent_storage_get_ble_pairing_by_id(id, &irk_out, &device_out, NULL);
  cl_assert(!ret);

  // Add the pairing again
  bonding_sync_add_bonding(&ble_bonding);
  id = bt_persistent_storage_store_ble_pairing(&pairing, true /* is_gateway */, NULL,
                                               false /* requires_address_pinning */,
                                               false /* auto_accept_re_pairing */);
  cl_assert(id != BT_BONDING_ID_INVALID);
  cl_assert_equal_i(s_ble_bonding_change_add_count, 2);
  cl_assert_equal_i(fake_shared_prf_storage_get_ble_store_count(), 2);

  // Delete a pairing that doesn't exist. Delete count should stay at 1
  bt_persistent_storage_delete_ble_pairing_by_id(9);
  cl_assert_equal_i(s_ble_bonding_change_delete_count, 1);
  cl_assert_equal_i(fake_shared_prf_storage_get_ble_delete_count(), 3);

  // Make sure the pairing is actually still there
  ret = bt_persistent_storage_get_ble_pairing_by_id(id, &irk_out, &device_out, NULL);
  cl_assert(ret);

  // And delete is again
  bt_persistent_storage_delete_ble_pairing_by_id(id);
  cl_assert_equal_i(s_ble_bonding_change_delete_count, 2);
  cl_assert_equal_i(fake_shared_prf_storage_get_ble_delete_count(), 4);

  // Try to read it back
  ret = bt_persistent_storage_get_ble_pairing_by_id(id, &irk_out, &device_out, NULL);
  cl_assert(!ret);
}

void test_bluetooth_persistent_storage__delete_ble_pairing_by_addr_if_matches(void) {
  SMIdentityResolvingKey irk_out;
  BTDeviceInternal device_out;

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
    .remote_encryption_info = {
      .ltk = (SMLongTermKey) {
        .data = {
          0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
          0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf,
        },
      },
      .ediv = 0x1234,
      .rand = 0x0123456789abcdefULL,
    },
    .is_remote_encryption_info_valid = true,
    .is_remote_identity_info_valid = true,
  };

  BTBondingID id = bt_persistent_storage_store_ble_pairing(&pairing, true /* is_gateway */, NULL,
                                                           false /* requires_address_pinning */,
                                                           0 /* flags */);
  cl_assert(id != BT_BONDING_ID_INVALID);

  // A delete carrying key material we never stored must leave the bonding alone,
  // and must not wipe the shared PRF copy either.
  SMPairingInfo other_keys = pairing;
  other_keys.remote_encryption_info.ltk.data[0] = 0xff;
  const int prf_delete_count = fake_shared_prf_storage_get_ble_delete_count();
  cl_assert_equal_b(bt_persistent_storage_delete_ble_pairing_by_addr_if_matches(&pairing.identity,
                                                                                &other_keys),
                    false);
  cl_assert_equal_i(fake_shared_prf_storage_get_ble_delete_count(), prf_delete_count);
  cl_assert_equal_b(bt_persistent_storage_get_ble_pairing_by_id(id, &irk_out, &device_out, NULL),
                    true);

  // Re-pairing the same phone keeps its identity and its IRK, so the new keys land
  // in this very record. A delete queued for the keys it replaced must not take it.
  SMPairingInfo repaired = pairing;
  repaired.remote_encryption_info.ltk.data[0] = 0x5a;
  cl_assert_equal_i(bt_persistent_storage_store_ble_pairing(&repaired, true /* is_gateway */, NULL,
                                                            false /* requires_address_pinning */,
                                                            0 /* flags */),
                    id);
  cl_assert_equal_b(bt_persistent_storage_delete_ble_pairing_by_addr_if_matches(&pairing.identity,
                                                                                &pairing),
                    false);
  cl_assert_equal_b(bt_persistent_storage_get_ble_pairing_by_id(id, &irk_out, &device_out, NULL),
                    true);

  // The keys that are actually stored do delete it, and take the shared PRF copy with them --
  // through the guarded erase, so a re-pair that has already refreshed that copy keeps it.
  const int prf_guarded_count = fake_shared_prf_storage_get_ble_delete_if_matches_count();
  const int prf_unconditional_count = fake_shared_prf_storage_get_ble_delete_count();
  cl_assert_equal_b(bt_persistent_storage_delete_ble_pairing_by_addr_if_matches(&pairing.identity,
                                                                                &repaired),
                    true);
  cl_assert_equal_b(bt_persistent_storage_get_ble_pairing_by_id(id, &irk_out, &device_out, NULL),
                    false);
  cl_assert_equal_i(fake_shared_prf_storage_get_ble_delete_if_matches_count(),
                    prf_guarded_count + 1);
  cl_assert_equal_i(fake_shared_prf_storage_get_ble_delete_count(), prf_unconditional_count);
}

//! The by-address lookup applies the same key comparison before it ever picks an id, so it cannot
//! hand the delete transaction a record whose keys have moved on. Enter by id, which is where the
//! record is read, checked and deleted under one lock, to exercise that check.
void test_bluetooth_persistent_storage__delete_ble_pairing_by_id_if_matches(void) {
  SMIdentityResolvingKey irk_out;
  BTDeviceInternal device_out;

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
    .remote_encryption_info = {
      .ltk = (SMLongTermKey) {
        .data = {
          0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
          0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf,
        },
      },
      .ediv = 0x1234,
      .rand = 0x0123456789abcdefULL,
    },
    .is_remote_encryption_info_valid = true,
    .is_remote_identity_info_valid = true,
  };

  BTBondingID id = bt_persistent_storage_store_ble_pairing(&pairing, true /* is_gateway */, NULL,
                                                           false /* requires_address_pinning */,
                                                           0 /* flags */);
  cl_assert(id != BT_BONDING_ID_INVALID);

  // A different LTK under the same id is a re-pair that landed here while the delete was queued.
  SMPairingInfo other_ltk = pairing;
  other_ltk.remote_encryption_info.ltk.data[0] = 0x5a;
  cl_assert_equal_b(bt_persistent_storage_delete_ble_pairing_by_id_if_matches(id, &other_ltk),
                    false);
  cl_assert_equal_b(bt_persistent_storage_get_ble_pairing_by_id(id, &irk_out, &device_out, NULL),
                    true);

  // The whole encryption half has to match, not just the LTK.
  SMPairingInfo other_rand = pairing;
  other_rand.remote_encryption_info.rand ^= 1ULL;
  cl_assert_equal_b(bt_persistent_storage_delete_ble_pairing_by_id_if_matches(id, &other_rand),
                    false);
  cl_assert_equal_b(bt_persistent_storage_get_ble_pairing_by_id(id, &irk_out, &device_out, NULL),
                    true);

  // Keys the record does not carry at all do not match either: a half the caller marks invalid is
  // not compared, and with no valid half left there is nothing but the identity to go on.
  SMPairingInfo identity_only = pairing;
  identity_only.is_remote_encryption_info_valid = false;
  cl_assert_equal_b(bt_persistent_storage_delete_ble_pairing_by_id_if_matches(id, &identity_only),
                    false);
  cl_assert_equal_b(bt_persistent_storage_get_ble_pairing_by_id(id, &irk_out, &device_out, NULL),
                    true);

  // The keys that are actually stored do delete it.
  cl_assert_equal_b(bt_persistent_storage_delete_ble_pairing_by_id_if_matches(id, &pairing), true);
  cl_assert_equal_b(bt_persistent_storage_get_ble_pairing_by_id(id, &irk_out, &device_out, NULL),
                    false);
}

void test_bluetooth_persistent_storage__ble_ancs_bonding(void) {
  bool ret;

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

  // This pairing is a heart rate monitor or something similar
  BTBondingID id = bt_persistent_storage_store_ble_pairing(&pairing, false /* is_gateway */, NULL,
                                                           false /* requires_address_pinning */,
                                                           false /* auto_accept_re_pairing */);
  cl_assert(id != BT_BONDING_ID_INVALID);

  // No ANCS bonding yet
  BTBondingID ancs_id = bt_persistent_storage_get_ble_ancs_bonding();
  cl_assert_equal_i(ancs_id, BT_BONDING_ID_INVALID);
  ret = bt_persistent_storage_has_ble_ancs_bonding();
  cl_assert(!ret);
  ret = bt_persistent_storage_is_ble_ancs_bonding(id);
  cl_assert(!ret);

  // Store another pairing, this one is a gateway (supports ancs)
  pairing.identity.address.octets[0] = 0x12;
  BTBondingID id2 = bt_persistent_storage_store_ble_pairing(&pairing, true /* is_gateway */, NULL,
                                                            false /* requires_address_pinning */,
                                                            false /* auto_accept_re_pairing */);
  cl_assert(id2 != BT_BONDING_ID_INVALID);

  // Find it
  ancs_id = bt_persistent_storage_get_ble_ancs_bonding();
  cl_assert_equal_i(ancs_id, id2);
  ret = bt_persistent_storage_has_ble_ancs_bonding();
  cl_assert(ret);
  ret = bt_persistent_storage_is_ble_ancs_bonding(id2);
  cl_assert(ret);
}

void test_bluetooth_persistent_storage__ble_device_name(void) {
  SMPairingInfo pairing = {
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
  const char *device_name = "iPhone";
  BTBondingID id = bt_persistent_storage_store_ble_pairing(&pairing, false /* is_gateway */,
                                                           device_name,
                                                           false /* requires_address_pinning */,
                                                           false /* auto_accept_re_pairing */);

  cl_assert(id != BT_BONDING_ID_INVALID);

  char device_name_out[BT_DEVICE_NAME_BUFFER_SIZE];
  bt_persistent_storage_get_ble_pairing_by_id(id, NULL, NULL, device_name_out);

  cl_assert_equal_i(strcmp(device_name, device_name_out), 0);

  // Update:
  const char *new_device_name = "New iPhone";
  bt_persistent_storage_update_ble_device_name(id, new_device_name);
  bt_persistent_storage_get_ble_pairing_by_id(id, NULL, NULL, device_name_out);

  cl_assert_equal_i(strcmp(new_device_name, device_name_out), 0);
}

void test_bluetooth_persistent_storage__test_root_keys(void) {
  SM128BitKey keys[2] = {
    {{0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16}},
    {{0x21, 0x22, 0x23, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x11, 0x12, 0x13, 0x24, 0x25, 0x26}},
  };

  SM128BitKey keys_out[2];

  bt_persistent_storage_set_root_keys(keys);
  bt_persistent_storage_get_root_key(0, &keys_out[0]);
  bt_persistent_storage_get_root_key(1, &keys_out[1]);
  cl_assert_equal_m(&keys[0], &keys_out[0], sizeof(SM128BitKey));
  cl_assert_equal_m(&keys[1], &keys_out[1], sizeof(SM128BitKey));

  bt_persistent_storage_init();

  bt_persistent_storage_get_root_key(0, &keys_out[0]);
  bt_persistent_storage_get_root_key(1, &keys_out[1]);
  cl_assert_equal_m(&keys[0], &keys_out[0], sizeof(SM128BitKey));
  cl_assert_equal_m(&keys[1], &keys_out[1], sizeof(SM128BitKey));
}


void test_bluetooth_persistent_storage__delete_all(void) {
  bool ret;

  // Add some pairings
  // BLE pairing 1
  SMPairingInfo pairing_1 = (SMPairingInfo) {
    .irk = (SMIdentityResolvingKey) {{
      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
      0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00
    }},
    .identity = (BTDeviceInternal) {
      .address = (BTDeviceAddress) {{0x11, 0x12, 0x13, 0x14, 0x15, 0x16}},
      .is_classic = false,
      .is_random_address = false,
    },
    .is_remote_identity_info_valid = true,
  };
  BTBondingID id_1 = bt_persistent_storage_store_ble_pairing(&pairing_1, true /* is_gateway */,
                                                             NULL,
                                                             false /* requires_address_pinning */,
                                                             false /* auto_accept_re_pairing */);

  // BLE pairing 2
  SMPairingInfo pairing_2 = (SMPairingInfo) {
    .irk = (SMIdentityResolvingKey) {{
      0x02, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
      0x02, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00
    }},
    .identity = (BTDeviceInternal) {
      .address = (BTDeviceAddress) {{0x22, 0x12, 0x13, 0x14, 0x15, 0x16}},
      .is_classic = false,
      .is_random_address = false,
    },
    .is_remote_identity_info_valid = true,
  };
  BTBondingID id_2 = bt_persistent_storage_store_ble_pairing(&pairing_2, false /* is_gateway */,
                                                             NULL,
                                                             false /* requires_address_pinning */,
                                                             false /* auto_accept_re_pairing */);

  // Delete all
  bt_persistent_storage_delete_all_pairings();

  // Try to get the pairings
  ret = bt_persistent_storage_get_ble_pairing_by_id(id_1, NULL, NULL, NULL);
  cl_assert(!ret);
  ret = bt_persistent_storage_get_ble_pairing_by_id(id_2, NULL, NULL, NULL);
  cl_assert(!ret);
}

// Test to make sure we don't accidentally change the serialized data formats.
void test_bluetooth_persistent_storage__ble_serialized_data(void) {
  //0000  01 00 69 50 68 6f 6e 65 20 4d 61 72 74 79 00 00  ..iPhone Marty..
  //0010  00 00 00 00 00 00 90 36 9c 6e 1a 1b eb 5f fb 89  .......6.n..._..
  //0020  db 0b ec a5 95 ab 92 8a aa f6 1c 47 90 53 43 ff  ...........G.SC.
  //0030  75 36 9c 6e 1a 1b eb 5f fb 89 db 0b ec a5 95 7a  u6.n..._.......z
  //0040  f3 e7 44 f6 1c 47 90 53 43 18 d1 6d 89 95 83 aa  ..D..G.SC..m....
  //0050  5e 7f ff 39 b3 47 36 e4 37 7e 05 1b 85 e3 b8 98  ^..9.G6.7~......
  //0060  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
  //0070  00 00 17                                         ...

  const uint8_t expected_raw_data[] = {
    0x01, 0x00, 0x69, 0x50, 0x68, 0x6f, 0x6e, 0x65,
    0x20, 0x4d, 0x61, 0x72, 0x74, 0x79, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x36,
    0x9c, 0x6e, 0x1a, 0x1b, 0xeb, 0x5f, 0xfb, 0x89,
    0xdb, 0x0b, 0xec, 0xa5, 0x95, 0xab, 0x92, 0x8a,
    0xaa, 0xf6, 0x1c, 0x47, 0x90, 0x53, 0x43, 0xff,
    0x75, 0x36, 0x9c, 0x6e, 0x1a, 0x1b, 0xeb, 0x5f,
    0xfb, 0x89, 0xdb, 0x0b, 0xec, 0xa5, 0x95, 0x7a,
    0xf3, 0xe7, 0x44, 0xf6, 0x1c, 0x47, 0x90, 0x53,
    0x43, 0x18, 0xd1, 0x6d, 0x89, 0x95, 0x83, 0xaa,
    0x5e, 0x7f, 0xff, 0x39, 0xb3, 0x47, 0x36, 0xe4,
    0x37, 0x7e, 0x05, 0x1b, 0x85, 0xe3, 0xb8, 0x98,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x17,
  };
  size_t data_size = 115;

  const SMPairingInfo pairing_info = {
    .local_encryption_info = {
      .ltk = {
        .data = {
          0x90, 0x36, 0x9c, 0x6e, 0x1a, 0x1b, 0xeb, 0x5f,
          0xfb, 0x89, 0xdb, 0x0b, 0xec, 0xa5, 0x95, 0xab,
        },
      },
      .rand = 0xff435390471cf6aa,
      .div = 0xf93f,
      .ediv = 0x8a92,
    },
    .remote_encryption_info = {
      .ltk = {
        .data = {
          0x75, 0x36, 0x9c, 0x6e, 0x1a, 0x1b, 0xeb, 0x5f,
          0xfb, 0x89, 0xdb, 0x0b, 0xec, 0xa5, 0x95, 0x7a,
        },
      },
      .rand = 0x18435390471cf644,
      .ediv = 0xe7f3,
    },
    .irk = {
      .data = {
        0xd1, 0x6d, 0x89, 0x95, 0x83, 0xaa, 0x5e, 0x7f,
        0xff, 0x39, 0xb3, 0x47, 0x36, 0xe4, 0x37, 0x7e,
      }
    },
    .identity = {
      {
        {
          .address = {
            .octets = {0x5, 0x1b, 0x85, 0xe3, 0xb8, 0x98}
          },
          .is_classic = 0x0,
          .is_random_address = 0x0,
          .zero = 0x0,
        },
      }
    },
    .csrk = {},
    .is_local_encryption_info_valid = 0x1,
    .is_remote_encryption_info_valid = 0x1,
    .is_remote_identity_info_valid = 0x1,
    .is_remote_signing_info_valid = 0x0,
    .is_mitm_protection_enabled = 0x1,
  };
  BTBondingID key = bt_persistent_storage_store_ble_pairing(&pairing_info, false /* is_gateway */,
                                                            "iPhone Marty",
                                                            false /* requires_address_pinning */,
                                                            false /* auto_accept_re_pairing */);
  cl_assert(key != BT_BONDING_ID_INVALID);

  uint8_t data[data_size];
  memset(data, 0, sizeof(data));
  int data_len = bt_persistent_storage_get_raw_data(&key, sizeof(key), data, data_size);
  cl_assert_equal_i(data_len, data_size);
  cl_assert_equal_m(expected_raw_data, data, sizeof(expected_raw_data));
}

