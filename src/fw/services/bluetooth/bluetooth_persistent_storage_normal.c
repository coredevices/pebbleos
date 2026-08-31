/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/bluetooth/bluetooth_persistent_storage.h"
#include "pbl/services/bluetooth/bluetooth_persistent_storage_debug.h"

#include "comm/ble/gap_le_connect.h"
#include "comm/ble/gap_le_connection.h"
#include "comm/ble/kernel_le_client/kernel_le_client.h"
#include "console/prompt.h"
#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"
#include "pbl/os/mutex.h"
#include "pbl/services/bluetooth/pairability.h"
#include "pbl/services/bluetooth/local_addr.h"
#include "pbl/services/shared_prf_storage/shared_prf_storage.h"
#include "pbl/services/settings/settings_file.h"
#include "system/hexdump.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/math.h"
#include "pbl/util/string.h"

#include <bluetooth/bonding_sync.h>
#include <pbl/btutil/bt_device.h>
#include <pbl/btutil/sm_util.h>

PBL_LOG_MODULE_DECLARE(service_bluetooth, CONFIG_SERVICE_BLUETOOTH_LOG_LEVEL);

#ifdef UNITTEST
// Let the unittest define this using a header override:
#  include "pbl/services/bluetooth/bluetooth_persistent_storage_unittest_impl.h"
#else
#  include "pbl/services/bluetooth/bluetooth_persistent_storage_v2_impl.h"
#endif

//! The BtPersistBonding*Data structs can never shrink, only grow

//! Stores data about a remote BT classic device
typedef struct PACKED {
  BTDeviceAddress addr;
  SM128BitKey link_key;
  char name[BT_DEVICE_NAME_BUFFER_SIZE];
  // These are the lowest bits of Remote.platform_bitfield_cache, which contain the OS type
  uint8_t platform_bits;
} BtPersistBondingBTClassicData;

//! Stores data about a remote BLE device
typedef struct PACKED {
  bool supports_ancs:1;
  bool is_gateway:1;
  bool requires_address_pinning:1;
  uint8_t flags:5;
  char name[BT_DEVICE_NAME_BUFFER_SIZE];
  BtPersistLEPairingInfo pairing_info;
} BtPersistBondingBLEData;

typedef struct PACKED {
  BtPersistBondingType type:8;

  union PACKED {
    BtPersistBondingBTClassicData bt_classic_data;
    BtPersistBondingBLEData ble_data;
  };
} BtPersistBondingData;

typedef struct PACKED {
  BTDeviceInternal peer;
  uint16_t chr_val_handle;
  uint16_t flags;
  unsigned value_changed:1;
} BtPersistCCCDData;

typedef struct PACKED {
  BTCCCDID id;
  BtPersistCCCDData data;
} BtPersistCCCD;

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Settings File

#define BT_PERSISTENT_STORAGE_FILE_NAME "gap_bonding_db"
#define BT_PERSISTENT_STORAGE_FILE_SIZE (4096)

//! All of the actual pairings use a BTBondingID as a key. This is because with BLE pairings an
//! address is not alwaywas available, and it made it easier to have BT Classic and BLE pairings
//! use the same type of key. When adding pairings there is no BTBondingID so a free key has to
//! be found by iterating over all possible keys.

//! All of the local device attributes can be accessed directly with the following keys:

//! This key is used to access the BTBondingID of the current active gateway
static const char ACTIVE_GATEWAY_KEY[] = "ACTIVE_GATEWAY";
//! This key is used to access a bool which stores if we have recently changed active gateways
static const char IS_UNFAITHFUL_KEY[] = "IS_UNFAITHFUL";
//! This key is used to access an array of two SM128BitKey values
static const char ROOT_KEYS_KEY[] = "ROOT_KEYS";
//! This key is used to access a char array which holds the device name
static const char DEVICE_NAME_KEY[] = "DEVICE_NAME";
//! This key is used to access a bool which stores the current airplane mode state
static const char AIRPLANE_MODE_KEY[] = "AIRPLANE_MODE";
//! This key is used to access a uint64_t which stores the most recent system session capabilities
static const char SYSTEM_CAPABILITIES_KEY[] = "SYSTEM_CAPABILITIES";
//! This key is used to access the BLE address that can be used for address pinning.
static const char BLE_PINNED_ADDRESS_KEY[] = "BLE_PINNED_ADDRESS";

static uint8_t s_bt_persistent_storage_updates = 0;

static PebbleMutex *s_db_mutex = NULL;

//! Cache of the last connected system session capabilities. Updated in flash when we get new flags
//! @note prv_lock() must be held when accessing this variable.
static PebbleProtocolCapabilities s_cached_system_capabilities;

static void prv_lock(void) {
  mutex_lock(s_db_mutex);
}

static void prv_unlock(void) {
  mutex_unlock(s_db_mutex);
}

static bool prv_bt_persistent_storage_get_ble_smpairinginfo_by_id(
    BTBondingID bonding, SMPairingInfo *info_out, char *name_out, bool *requires_address_pinning,
    uint8_t *flags);

static void prv_update_bondings(BTBondingID id, BtPersistBondingType type) {
  if (id == BT_BONDING_ID_INVALID) {
    return;
  }

  if (type == BtPersistBondingTypeBLE) {
    SMPairingInfo pairing_info;
    char ble_name[BT_DEVICE_NAME_BUFFER_SIZE] = { };
    bool requires_address_pinning = false;
    uint8_t flags = 0;
    if (prv_bt_persistent_storage_get_ble_smpairinginfo_by_id(
            id, &pairing_info, ble_name, &requires_address_pinning,
            &flags)) {
      // only send the ble_name if we have a name to send!
      char *ble_name_ptr = (strlen(ble_name) == 0) ? NULL : &ble_name[0];
      shared_prf_storage_store_ble_pairing_data(
          &pairing_info, ble_name_ptr, requires_address_pinning, flags);
    }
  }
}

//! Returns the size of the data read. If the buffer provided is too small then 0 is returned
static int prv_file_get(const void *key, size_t key_len, void *data_out, size_t buf_len) {
  unsigned int data_len = 0;
  // Zero the output up front so callers that ignore the return value (a 0 length on
  // open/read failure or an oversized record) read defined zeros, not stack garbage.
  memset(data_out, 0, buf_len);
  prv_lock();
  {
    SettingsFile fd;
    status_t rv = settings_file_open(&fd, BT_PERSISTENT_STORAGE_FILE_NAME,
                                     BT_PERSISTENT_STORAGE_FILE_SIZE);
    if (rv != S_SUCCESS) {
      goto cleanup;
    }

    data_len = settings_file_get_len(&fd, key, key_len);
    // If a big enough buffer wasn't passed in, then the data can't be read.
    if (data_len > buf_len ||
        settings_file_get(&fd, key, key_len, data_out, buf_len) != S_SUCCESS) {
      data_len = 0;
    }

    settings_file_close(&fd);
  }
cleanup:
  prv_unlock();
  return data_len;
}

//! @return the value that was read at that key or `default_value` if the key does not exist,
//! or if the stored data has been corrupted.
static bool prv_file_get_bool(const void *key, size_t key_len, bool default_value) {
  uint8_t bool_data;
  int read_size = prv_file_get(key, key_len, (void*)&bool_data, sizeof(bool_data));
  if (!read_size ||
      ((bool_data != (uint8_t)true) && (bool_data != (uint8_t)false))) {
    return default_value;
  }
  // Default to false in the case of data corruption (anything other than 0x1 or 0x0).
  return bool_data;
}

//! Returns true if the set was successful
typedef enum {
  GapBondingFileSetFail = 0,
  GapBondingFileSetUpdated,
  GapBondingFileSetNoUpdateNeeded,
} GapBondingFileSetStatus;

static GapBondingFileSetStatus prv_file_set(
    const void *key, size_t key_len, const void *data_in, size_t data_len) {
  status_t rv;
  bool do_perform_update = true;
  prv_lock();
  {
    SettingsFile fd;
    rv = settings_file_open(&fd, BT_PERSISTENT_STORAGE_FILE_NAME, BT_PERSISTENT_STORAGE_FILE_SIZE);
    if (rv != S_SUCCESS) {
      goto cleanup;
    }

    // Only store data if data_in is a valid pointer, otherwise, clear the entry
    if (data_in) {
      if (settings_file_get_len(&fd, key, key_len) == (int)data_len) {
        uint8_t curr_val[data_len];

        settings_file_get(&fd, key, key_len, &curr_val[0], data_len);

        // Don't bother rewriting the exact same info. Pairing info is precious,
        // we want to minimize cases where we could mess it up
        if (memcmp(&curr_val, data_in, data_len) == 0) {
          do_perform_update = false;
        }
      }

      if (do_perform_update) {
        s_bt_persistent_storage_updates++;
        PBL_LOG_D_DBG(LOG_DOMAIN_BT_PAIRING_INFO, "Updating GAP Bonding DB Value <key, val>!");
        PBL_HEXDUMP_D(LOG_DOMAIN_BT_PAIRING_INFO, LOG_LEVEL_DEBUG, (uint8_t *)key, key_len);
        PBL_HEXDUMP_D(LOG_DOMAIN_BT_PAIRING_INFO, LOG_LEVEL_DEBUG, (uint8_t *)data_in, data_len);
        rv = settings_file_set(&fd, key, key_len, (uint8_t*) data_in, data_len);
      }
    } else {
      rv = settings_file_delete(&fd, key, key_len);
    }
    settings_file_close(&fd);
  }
cleanup:
  prv_unlock();
  if (rv != S_SUCCESS) {
    PBL_LOG_ERR("Failed to update gap bonding db, rv = %"PRId32, rv);
    return GapBondingFileSetFail;
  }

  return (do_perform_update ? GapBondingFileSetUpdated : GapBondingFileSetNoUpdateNeeded);
}

//! Returns true if things were successful
static bool prv_file_each(SettingsFileEachCallback itr_cb, void *itr_data) {
  status_t rv;
  prv_lock();
  {
    SettingsFile fd;
    rv = settings_file_open(&fd, BT_PERSISTENT_STORAGE_FILE_NAME, BT_PERSISTENT_STORAGE_FILE_SIZE);
    if (rv) {
      goto cleanup;
    }

    settings_file_each(&fd, itr_cb, itr_data);
    settings_file_close(&fd);
  }
cleanup:
  prv_unlock();
  return (rv == S_SUCCESS);
}


//! Get the next available BondingID
//! This function re-uses bonding ids as they are freed. This could be a problem with 3rd party
//! apps. https://pebbletechnology.atlassian.net/browse/PBL-8391
static BTBondingID prv_get_free_key() {
  BTBondingID free_key = BT_BONDING_ID_INVALID;

  prv_lock();
  {
  SettingsFile fd;
  status_t rv = settings_file_open(&fd, BT_PERSISTENT_STORAGE_FILE_NAME,
                                   BT_PERSISTENT_STORAGE_FILE_SIZE);
  if (rv) {
    goto cleanup;
  }

  for (BTBondingID id = 0; id < BT_BONDING_ID_INVALID; id++) {
    if (!settings_file_exists(&fd, &id, sizeof(id))) {
      free_key = id;
      break;
    }
  }

  settings_file_close(&fd);

  }
cleanup:
  prv_unlock();
  return free_key;
}

//! Get the next available CCCDID
static BTCCCDID prv_get_free_cccd() {
  BTCCCDID free_cccd = BT_CCCD_ID_INVALID;

  prv_lock();
  {
  SettingsFile fd;
  status_t rv = settings_file_open(&fd, BT_PERSISTENT_STORAGE_FILE_NAME,
                                   BT_PERSISTENT_STORAGE_FILE_SIZE);
  if (rv) {
    goto cleanup;
  }

  for (BTCCCDID id = 0U; id < BT_CCCD_ID_INVALID; id++) {
    if (!settings_file_exists(&fd, &id, sizeof(id))) {
      free_cccd = id;
      break;
    }
  }

  settings_file_close(&fd);

  }
cleanup:
  prv_unlock();
  return free_cccd;
}

static bool prv_any_pinned_ble_pairings_itr(SettingsFile *file,
                                            SettingsRecordInfo *info,
                                            void *context) {
  if (info->key_len != sizeof(BTBondingID)) {
    return true;
  }
  if (info->val_len == 0) {
    return true;
  }

  BTBondingID key;
  info->get_key(file, (uint8_t*) &key, info->key_len);

  BtPersistBondingData data = {};
  info->get_val(file, &data, MIN((unsigned)info->val_len, sizeof(data)));
  if (data.ble_data.requires_address_pinning) {
    bool *has_pinned_ble_pairings = context;
    *has_pinned_ble_pairings = true;
    return false;
  }

  return true;
}

bool bt_persistent_storage_has_pinned_ble_pairings(void) {
  bool has_pinned_ble_pairings = false;
  prv_file_each(prv_any_pinned_ble_pairings_itr, &has_pinned_ble_pairings);
  return has_pinned_ble_pairings;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//! Shared PRF Storage

static void prv_load_pinned_address_from_prf(void) {
  BTDeviceAddress pinned_address;

  if (shared_prf_storage_get_ble_pinned_address(&pinned_address)) {
    bt_persistent_storage_set_ble_pinned_address(&pinned_address);
  }

  // if we get here there is no pinned address in PRF, let's load the address fw has been
  // using. This shouldn't ever really happen unless we reboot while saving new information to
  // shared PRF
  if (bt_persistent_storage_get_ble_pinned_address(&pinned_address)) {
    shared_prf_storage_set_ble_pinned_address(&pinned_address);
  }
}

static void prv_load_local_data_from_prf(void) {
  char name[BT_DEVICE_NAME_BUFFER_SIZE];
  if (shared_prf_storage_get_local_device_name(name, BT_DEVICE_NAME_BUFFER_SIZE)) {
    bt_persistent_storage_set_local_device_name(name, BT_DEVICE_NAME_BUFFER_SIZE);
  }

  SM128BitKey keys[SMRootKeyTypeNum];
  if (shared_prf_storage_get_root_key(SMRootKeyTypeEncryption, &keys[SMRootKeyTypeEncryption]) &&
      shared_prf_storage_get_root_key(SMRootKeyTypeIdentity, &keys[SMRootKeyTypeIdentity])) {
#if !defined(CONFIG_RELEASE)
    PBL_LOG_INFO("Loading Root Keys from PRF storage:");
    PBL_HEXDUMP(LOG_LEVEL_INFO, (const uint8_t *) keys, sizeof(keys));
#endif
    bt_persistent_storage_set_root_keys(keys);
    return;
  }

  // if we get here there are no root keys in prf storage, let's load the root
  // keys normal fw has been using. This shouldn't ever really happen unless we
  // reboot while saving new information to shared PRF
  if (bt_persistent_storage_get_root_key(SMRootKeyTypeEncryption, &keys[SMRootKeyTypeEncryption]) &&
      bt_persistent_storage_get_root_key(SMRootKeyTypeIdentity, &keys[SMRootKeyTypeIdentity])) {
    PBL_LOG_ERR("Storing Root Keys to PRF storage");
    shared_prf_storage_set_root_keys(keys);
  }
}

static void prv_push_ble_persist_to_shared_prf(void) {
  BTBondingID bonding_id = bt_persistent_storage_get_ble_ancs_bonding();

  if (bonding_id != BT_BONDING_ID_INVALID) {
    prv_update_bondings(bonding_id, BtPersistBondingTypeBLE);
  }
}

static void prv_load_ble_pairing_from_prf(void) {
  SMPairingInfo prf_pairing_info;
  char device_name[BT_DEVICE_NAME_BUFFER_SIZE];
  bool requires_address_pinning;
  uint8_t flags;
  if (!shared_prf_storage_get_ble_pairing_data(&prf_pairing_info, device_name,
                                               &requires_address_pinning,
                                               &flags)) {
    // No pairing available, check to see if we have a pairing in the gapDB
    prv_push_ble_persist_to_shared_prf();
    return;
  }

  // PRF pairing storage has only one pairing slot. Assume is_gateway:
  bt_persistent_storage_store_ble_pairing(&prf_pairing_info, true /* is_gateway */, device_name,
                                          requires_address_pinning, flags);
}

static void prv_load_data_from_prf(void) {
  prv_load_local_data_from_prf();
  prv_load_pinned_address_from_prf();
  prv_load_ble_pairing_from_prf();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Common Helper Functions

BtPersistBondingType prv_get_type_for_id(BTBondingID id) {
  BtPersistBondingData data;
  prv_file_get(&id, sizeof(id), &data, sizeof(data));

  return data.type;
}

bool prv_delete_pairing_with_type_by_id(BTBondingID bonding, BtPersistBondingType type,
                                        BtPersistBondingData *data_out) {
  if (!prv_file_get(&bonding, sizeof(bonding), data_out, sizeof(*data_out))) {
    return false;
  }

  if (data_out->type != type) {
    PBL_LOG_ERR("Type mismatch: not deleting pairing. Is the bonding db corrupted?");
    return false;
  }

  if (prv_file_set(&bonding, sizeof(bonding), NULL, 0) == GapBondingFileSetFail) {
    return false;
  }

  return true;
}

bool prv_has_active_gateway_by_type(BtPersistBondingType desired_type) {
  BTBondingID bonding;
  BtPersistBondingType type;

  if (!bt_persistent_storage_get_active_gateway(&bonding, &type)) {
    return false;
  }

  if (bonding == BT_BONDING_ID_INVALID || type != desired_type) {
    return false;
  }

  return true;
}

static void prv_update_active_gateway_if_needed(BTBondingID bonding, BtPersistBondingOp op) {
  // Invalidate the active gateway if it is getting deleted
  if (op == BtPersistBondingOpWillDelete) {
    // get_active_gateway leaves the out param untouched when there is no active
    // gateway, so only compare when it reports success.
    BTBondingID current_active_gateway;
    if (bt_persistent_storage_get_active_gateway(&current_active_gateway, NULL) &&
        current_active_gateway == bonding) {
      bt_persistent_storage_set_active_gateway(BT_BONDING_ID_INVALID);
    }
  }
}

static void prv_call_common_bonding_change_handlers(BTBondingID bonding, BtPersistBondingOp op) {
  bt_pairability_update_due_to_bonding_change();
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//! BLE Pairing Info
void gap_le_connection_handle_bonding_change(BTBondingID bonding, BtPersistBondingOp op);
typedef struct {
  BTBondingID bonding;
  BtPersistBondingOp op;
} BleBondingChangeContext;

static void prv_call_ble_bonding_change_handlers_impl(BTBondingID bonding,
                                                      BtPersistBondingOp op) {
  prv_update_active_gateway_if_needed(bonding, op);

  if (!bt_ctl_is_bluetooth_running()) {
    return;
  }
  bt_local_addr_handle_bonding_change(bonding, op);
  gap_le_connection_handle_bonding_change(bonding, op);
  gap_le_connect_handle_bonding_change(bonding, op);
  kernel_le_client_handle_bonding_change(bonding, op);
  prv_call_common_bonding_change_handlers(bonding, op);
}

static void prv_call_ble_bonding_change_handlers_cb(void *data) {
  BleBondingChangeContext *context = data;
  prv_call_ble_bonding_change_handlers_impl(context->bonding, context->op);
  kernel_free(context);
}

static void prv_call_ble_bonding_change_handlers(BTBondingID bonding,
                                                 BtPersistBondingOp op) {
  if (launcher_task_is_current_task()) {
    prv_call_ble_bonding_change_handlers_impl(bonding, op);
    return;
  }

  BleBondingChangeContext *context = kernel_malloc_check(sizeof(*context));

  *context = (BleBondingChangeContext) {
    .bonding = bonding,
    .op = op,
  };
  launcher_task_add_callback(prv_call_ble_bonding_change_handlers_cb, context);
}

typedef struct {
  SMPairingInfo pairing_info;
  BTBondingID key_out;
} KeyForSMPairingItrData;

static bool prv_is_pairing_info_equal_identity(const BtPersistLEPairingInfo *a,
                                               const SMPairingInfo *b) {
  return (a->is_remote_identity_info_valid &&
          b->is_remote_identity_info_valid &&
          bt_device_equal(&a->identity.opaque, &b->identity.opaque) &&
          memcmp(&a->irk, &b->irk, sizeof(SMIdentityResolvingKey)) == 0);
}

static bool prv_get_key_for_sm_pairing_info_itr(SettingsFile *file,
                                                SettingsRecordInfo *info, void *context) {
  // check entry is valid
  if (info->val_len == 0 || info->key_len != sizeof(BTBondingID)) {
    return true; // continue iterating
  }

  KeyForSMPairingItrData *itr_data = (KeyForSMPairingItrData*) context;

  BTBondingID key;
  info->get_key(file, (uint8_t*) &key, info->key_len);

  BtPersistBondingData stored_data;
  info->get_val(file, (uint8_t*) &stored_data, MIN((unsigned)info->val_len, sizeof(stored_data)));

  if (stored_data.type == BtPersistBondingTypeBLE &&
      prv_is_pairing_info_equal_identity(&stored_data.ble_data.pairing_info,
                                         &itr_data->pairing_info)) {
    itr_data->key_out = key;
    return false; // stop iterating
  }

  return true;
}

static BTBondingID prv_get_key_for_sm_pairing_info(const SMPairingInfo *pairing_info) {
  KeyForSMPairingItrData itr_data = {
    .pairing_info = *pairing_info,
    .key_out = BT_BONDING_ID_INVALID,
  };
  prv_file_each(prv_get_key_for_sm_pairing_info_itr, &itr_data);

  return itr_data.key_out;
}

static bool prv_delete_ble_pairing_by_id(BTBondingID bonding);

//! Only a single BLE pairing is supported at a time. The buffer below is sized generously to absorb
//! any legacy state where the bonding DB ended up with multiple entries (e.g. after a PRF pairing
//! was merged on top of an existing normal-FW pairing).
#define BT_BONDING_PRUNE_MAX 8

typedef struct {
  BTBondingID keep_id;
  BTBondingID ids[BT_BONDING_PRUNE_MAX];
  uint8_t count;
} CollectOtherBleItrData;

static bool prv_collect_other_ble_bondings_itr(SettingsFile *file, SettingsRecordInfo *info,
                                               void *context) {
  if (info->val_len == 0 || info->key_len != sizeof(BTBondingID)) {
    return true;
  }

  CollectOtherBleItrData *itr_data = context;

  BTBondingID key;
  info->get_key(file, (uint8_t *)&key, info->key_len);
  if (key == itr_data->keep_id) {
    return true;
  }

  BtPersistBondingData stored_data;
  info->get_val(file, (uint8_t *)&stored_data, MIN((unsigned)info->val_len, sizeof(stored_data)));
  if (stored_data.type != BtPersistBondingTypeBLE) {
    return true;
  }

  if (itr_data->count < BT_BONDING_PRUNE_MAX) {
    itr_data->ids[itr_data->count++] = key;
  }
  return true;
}

//! Delete every BLE bonding except `keep_id`. We only ever support one BLE pairing at a time, so
//! any other BLE bonding present is stale and must be removed (e.g. when a new phone pairs and
//! replaces the previous one).
//!
//! Uses the internal delete helper that does not erase shared PRF pairing data, since the kept
//! entry is the one that should remain reflected in PRF storage.
static void prv_delete_other_ble_bondings(BTBondingID keep_id) {
  CollectOtherBleItrData itr_data = {
    .keep_id = keep_id,
    .count = 0,
  };
  prv_file_each(prv_collect_other_ble_bondings_itr, &itr_data);

  for (uint8_t i = 0; i < itr_data.count; i++) {
    PBL_LOG_INFO("Removing stale BLE bonding %d (kept %d)", itr_data.ids[i], keep_id);
    prv_delete_ble_pairing_by_id(itr_data.ids[i]);
  }
}

typedef struct {
  BTBondingID key_out;
  uint32_t last_modified_out;
  uint8_t ble_count;
} MostRecentBleItrData;

static bool prv_find_most_recent_ble_bonding_itr(SettingsFile *file, SettingsRecordInfo *info,
                                                 void *context) {
  if (info->val_len == 0 || info->key_len != sizeof(BTBondingID)) {
    return true;
  }

  MostRecentBleItrData *itr_data = context;

  BtPersistBondingData stored_data;
  info->get_val(file, (uint8_t *)&stored_data, MIN((unsigned)info->val_len, sizeof(stored_data)));
  if (stored_data.type != BtPersistBondingTypeBLE) {
    return true;
  }

  BTBondingID key;
  info->get_key(file, (uint8_t *)&key, info->key_len);

  itr_data->ble_count++;
  if (itr_data->key_out == BT_BONDING_ID_INVALID ||
      info->last_modified > itr_data->last_modified_out) {
    itr_data->key_out = key;
    itr_data->last_modified_out = info->last_modified;
  }
  return true;
}

//! If the bonding DB contains multiple BLE pairings (e.g. left over from an older firmware that
//! allowed more than one, or from a PRF pairing merged on top of an existing one), keep the most
//! recently modified entry and drop the rest.
static void prv_prune_stale_ble_bondings(void) {
  MostRecentBleItrData itr_data = {
    .key_out = BT_BONDING_ID_INVALID,
    .last_modified_out = 0,
    .ble_count = 0,
  };
  prv_file_each(prv_find_most_recent_ble_bonding_itr, &itr_data);

  if (itr_data.ble_count <= 1 || itr_data.key_out == BT_BONDING_ID_INVALID) {
    return;
  }

  PBL_LOG_INFO("Found %u BLE bondings at boot, keeping most recent (id %d)",
               itr_data.ble_count, itr_data.key_out);
  prv_delete_other_ble_bondings(itr_data.key_out);
}

//! For unit testing
int bt_persistent_storage_get_raw_data(const void *key, size_t key_len,
                                       void *data_out, size_t buf_len) {
  return prv_file_get(key, key_len, data_out, buf_len);
}

bool bt_persistent_storage_set_ble_pinned_address(const BTDeviceAddress *addr) {
  GapBondingFileSetStatus rv = prv_file_set(&BLE_PINNED_ADDRESS_KEY, sizeof(BLE_PINNED_ADDRESS_KEY),
                                            addr, addr ? sizeof(*addr) : 0);
  bool success = (rv != GapBondingFileSetFail);
  if (!success) {
    PBL_LOG_ERR("Failed to store pinned address");
  } else if (rv == GapBondingFileSetUpdated) {
    shared_prf_storage_set_ble_pinned_address(addr);
  }
  return success;
}

BTBondingID bt_persistent_storage_store_ble_pairing(const SMPairingInfo *new_pairing_info,
                                                    bool is_gateway, const char *device_name,
                                                    bool requires_address_pinning,
                                                    uint8_t flags) {
  if (!new_pairing_info || sm_is_pairing_info_empty(new_pairing_info)) {
    return BT_BONDING_ID_INVALID;
  }

  // Check if this is an update
  BtPersistBondingOp op = BtPersistBondingOpDidChange;
  BTBondingID key = prv_get_key_for_sm_pairing_info(new_pairing_info);

  if (key == BT_BONDING_ID_INVALID) {
    // This is an add, not an update
    op = BtPersistBondingOpDidAdd;
    key = prv_get_free_key();
    if (key == BT_BONDING_ID_INVALID) {
      // We are out of keys....
      return BT_BONDING_ID_INVALID;
    }
  } else {
    // If we add any optional fields a load will have to happen here so they don't get overwritten
  }

  BtPersistBondingData new_data;
  new_data = (BtPersistBondingData) {
    .type = BtPersistBondingTypeBLE,
    .ble_data.is_gateway = is_gateway,
    .ble_data.flags = flags,
    // This is defaulting to "is_gateway" for now because it is currently being used as the flag
    // for the pairing that we want to reconnect/connect to. If this isn't set then
    // we don't register an intent for the device and thus don't connect.
    // Currently only 1 ble pairing is really supported so this works for now
    // FIXME: https://pebbletechnology.atlassian.net/browse/PBL-15277
    .ble_data.supports_ancs = is_gateway,
    .ble_data.requires_address_pinning = requires_address_pinning,
  };
  bt_persistent_storage_assign_persist_pairing_info(&new_data.ble_data.pairing_info,
                                                    new_pairing_info);

  if (device_name) {
    strncpy(new_data.ble_data.name, device_name, BT_DEVICE_NAME_BUFFER_SIZE);
    new_data.ble_data.name[BT_DEVICE_NAME_BUFFER_SIZE - 1] = '\0';
  }

  GapBondingFileSetStatus status;
  status = prv_file_set(&key, sizeof(key), &new_data, sizeof(new_data));
  if (status == GapBondingFileSetFail) {
    return BT_BONDING_ID_INVALID;
  }

  if (is_gateway && status == GapBondingFileSetUpdated) {
    prv_update_bondings(key, BtPersistBondingTypeBLE);
  }

  // In practice we only support a single BLE pairing at a time, so if we add a new one,
  // set ourselves as unfaithful.
  if (op == BtPersistBondingOpDidAdd) {
    bt_persistent_storage_set_unfaithful(true);
  }

  prv_call_ble_bonding_change_handlers(key, op);

  // We only support a single BLE pairing at a time. Drop any previous BLE bonding so that
  // re-pairing with a different phone (or merging a PRF pairing) replaces the old one instead of
  // leaving it behind and forcing the user to forget it manually.
  prv_delete_other_ble_bondings(key);

  return key;
}

bool bt_persistent_storage_update_ble_device_name(BTBondingID bonding, const char *device_name) {
  BtPersistBondingData data;
  if (!prv_file_get(&bonding, sizeof(bonding), &data, sizeof(data))) {
    return false;
  }

  if (data.type != BtPersistBondingTypeBLE) {
    PBL_LOG_ERR("Not getting BLE id %d. Type mismatch", bonding);
    return false;
  }

  strncpy(data.ble_data.name, device_name, BT_DEVICE_NAME_BUFFER_SIZE);
  data.ble_data.name[BT_DEVICE_NAME_BUFFER_SIZE - 1] = '\0';

  GapBondingFileSetStatus status;
  status = prv_file_set(&bonding, sizeof(bonding), &data, sizeof(data));

  // If this is the gateway, update SPRF so our pairing info betwen PRF and normal
  // FW is in sync
  if (data.ble_data.is_gateway && (status == GapBondingFileSetUpdated)) {
    prv_update_bondings(bonding, BtPersistBondingTypeBLE);
  }

  return (status != GapBondingFileSetFail);
}

static void prv_init_and_assign_ble_bonding(BleBonding *bonding,
                                            const BtPersistBondingData *stored_data) {
  *bonding = (BleBonding){};
  bt_persistent_storage_assign_sm_pairing_info(&bonding->pairing_info,
                                               &stored_data->ble_data.pairing_info);
  bonding->is_gateway = stored_data->ble_data.is_gateway;
}

static void prv_remove_ble_bonding_from_bt_driver(const BtPersistBondingData *deleted_data) {
  if (!bt_ctl_is_bluetooth_running()) {
    return;
  }
  BleBonding bonding;
  prv_init_and_assign_ble_bonding(&bonding, deleted_data);
  bt_driver_handle_host_removed_bonding(&bonding);
}

status_t prv_delete_all_cccd_for_addr(const BTDeviceInternal *dev) {
  status_t rv;

  prv_lock();
  {
  SettingsFile fd;
  rv = settings_file_open(&fd, BT_PERSISTENT_STORAGE_FILE_NAME,
                          BT_PERSISTENT_STORAGE_FILE_SIZE);
  if (rv) {
    goto cleanup;
  }

  for (BTCCCDID id = 0U; id < BT_CCCD_ID_INVALID; id++) {
    if (settings_file_exists(&fd, &id, sizeof(id))) {
      BtPersistCCCDData stored_data;

      rv = settings_file_get(&fd, &id, sizeof(id), &stored_data, sizeof(stored_data));
      if (rv) {
        goto cleanup;
      }

      if (bt_device_internal_equal(dev, &stored_data.peer)) {
        BleCCCD cccd_to_delete = {
          .peer = *dev,
          .chr_val_handle = stored_data.chr_val_handle,
          .flags = stored_data.flags,
          .value_changed = stored_data.value_changed,
        };

        bt_driver_handle_host_removed_cccd(&cccd_to_delete);

        rv = settings_file_delete(&fd, &id, sizeof(id));
        if (rv) {
          goto cleanup;
        }
      }
    }
  }

  settings_file_close(&fd);

  }
cleanup:
  prv_unlock();
  return rv;
}

static bool prv_delete_ble_pairing_by_id(BTBondingID bonding) {
  BtPersistBondingData deleted_data;
  if (!prv_delete_pairing_with_type_by_id(bonding, BtPersistBondingTypeBLE, &deleted_data)) {
    return false;
  }

  status_t rv;
  rv = prv_delete_all_cccd_for_addr(&deleted_data.ble_data.pairing_info.identity);
  PBL_ASSERTN(rv == S_SUCCESS);

  prv_remove_ble_bonding_from_bt_driver(&deleted_data);

  prv_call_ble_bonding_change_handlers(bonding, BtPersistBondingOpWillDelete);
  return true;
}

void bt_persistent_storage_delete_ble_pairing_by_id(BTBondingID bonding) {
  if (!prv_delete_ble_pairing_by_id(bonding)) {
    return;
  }
  // TODO: Make sure this matches what we have stored
  shared_prf_storage_erase_ble_pairing_data();
}

typedef struct {
  BTDeviceInternal device;
  SMIdentityResolvingKey irk_out;
  char name_out[BT_DEVICE_NAME_BUFFER_SIZE];
  BTBondingID id_out;
  bool found;
} FindByAddrItrData;

static bool prv_find_by_addr_itr(SettingsFile *file,
                                 SettingsRecordInfo *info, void *context) {
  // check entry is valid
  if (info->val_len == 0 || info->key_len != sizeof(BTBondingID)) {
    return true; // continue iterating
  }

  FindByAddrItrData *itr_data = (FindByAddrItrData *) context;

  BTBondingID key;
  info->get_key(file, (uint8_t*) &key, info->key_len);

  BtPersistBondingData stored_data;
  info->get_val(file, (uint8_t*) &stored_data, MIN((unsigned)info->val_len, sizeof(stored_data)));

  if (stored_data.type == BtPersistBondingTypeBLE &&
      bt_device_equal(&itr_data->device.opaque,
                      &stored_data.ble_data.pairing_info.identity.opaque)) {
    itr_data->irk_out = stored_data.ble_data.pairing_info.irk;
    strncpy(itr_data->name_out, stored_data.ble_data.name, BT_DEVICE_NAME_BUFFER_SIZE);
    itr_data->id_out = key;
    itr_data->found = true;
    return false; // stop iterating
  }

  return true; // continue iterating
}

void bt_persistent_storage_delete_ble_pairing_by_addr(const BTDeviceInternal *device) {
  FindByAddrItrData itr_data = {
    .device = *device,
    .found = false,
  };
  prv_file_each(prv_find_by_addr_itr, &itr_data);

  if (!itr_data.found) {
    return;
  }

  bt_persistent_storage_delete_ble_pairing_by_id(itr_data.id_out);
}

static void prv_fill_ble_data(SMIdentityResolvingKey *irk_in,
                              BTDeviceInternal *device_in,
                              char *name_in,
                              SMIdentityResolvingKey *irk_out,
                              BTDeviceInternal *device_out,
                              char *name_out) {
  if (irk_out && irk_in) {
    *irk_out = *irk_in;
  }

  if (device_out && device_in) {
    *device_out = *device_in;
  }
  if (name_out && name_in) {
    strncpy(name_out, name_in, BT_DEVICE_NAME_BUFFER_SIZE);
    name_out[BT_DEVICE_NAME_BUFFER_SIZE - 1] = '\0';
  }
}

bool bt_persistent_storage_get_ble_pairing_by_id(BTBondingID bonding,
                                          SMIdentityResolvingKey *irk_out,
                                          BTDeviceInternal *device_out,
                                          char *name_out) {
  BtPersistBondingData data;
  if (!prv_file_get(&bonding, sizeof(bonding), &data, sizeof(data))) {
    return false;
  }

  if (data.type != BtPersistBondingTypeBLE) {
    PBL_LOG_ERR("Not getting BT Classic id %d. Type mismatch", bonding);
    return false;
  }

  prv_fill_ble_data(&data.ble_data.pairing_info.irk, &data.ble_data.pairing_info.identity,
                    data.ble_data.name, irk_out, device_out, name_out);

  return true;
}

static bool prv_bt_persistent_storage_get_ble_smpairinginfo_by_id(
    BTBondingID bonding, SMPairingInfo *info_out, char *name_out, bool *requires_address_pinning,
    uint8_t *flags) {
  BtPersistBondingData data;
  if (!prv_file_get(&bonding, sizeof(bonding), &data, sizeof(data))) {
    return false;
  }

  if (data.type != BtPersistBondingTypeBLE) {
    PBL_LOG_ERR("Not getting BLE id %d. Type mismatch", bonding);
    return false;
  }

  if (info_out) {
    bt_persistent_storage_assign_sm_pairing_info(info_out, &data.ble_data.pairing_info);
  }

  *requires_address_pinning = data.ble_data.requires_address_pinning;
  *flags = data.ble_data.flags;

  prv_fill_ble_data(
      NULL, NULL, data.ble_data.name, NULL, NULL, name_out);
  return true;
}

bool bt_persistent_storage_get_ble_pairing_by_addr(const BTDeviceInternal *device,
                                                   SMIdentityResolvingKey *irk_out,
                                                   char name_out[BT_DEVICE_NAME_BUFFER_SIZE]) {
  FindByAddrItrData itr_data = {
    .device = *device,
    .found = false,
  };
  prv_file_each(prv_find_by_addr_itr, &itr_data);

  if (!itr_data.found) {
    return false;
  }

  prv_fill_ble_data(&itr_data.irk_out, NULL, itr_data.name_out,
                    irk_out, NULL, name_out);

  return true;
}

bool bt_persistent_storage_get_ble_pinned_address(BTDeviceAddress *address_out) {
  BTDeviceAddress address;
  int read_size = prv_file_get(&BLE_PINNED_ADDRESS_KEY, sizeof(BLE_PINNED_ADDRESS_KEY),
                               &address, sizeof(address));
  if (!read_size) {
    return false;
  }
  if (address_out) {
    *address_out = address;
  }
  return true;
}

static bool prv_get_first_ancs_bonding_itr(SettingsFile *file,
                                           SettingsRecordInfo *info, void *context) {
  // check entry is valid
  if (info->val_len == 0 || info->key_len != sizeof(BTBondingID)) {
    return true; // continue iterating
  }

  BTBondingID *first_ancs_supported_bonding_found = (BTBondingID *) context;

  BTBondingID key;
  info->get_key(file, (uint8_t*) &key, info->key_len);

  BtPersistBondingData stored_data;
  info->get_val(file, (uint8_t*) &stored_data, MIN((unsigned)info->val_len, sizeof(stored_data)));

  if (stored_data.type == BtPersistBondingTypeBLE && stored_data.ble_data.supports_ancs) {
    *first_ancs_supported_bonding_found = key;
    return false; // stop iterating
  }

  return true;
}

BTBondingID bt_persistent_storage_get_ble_ancs_bonding(void) {
  BTBondingID first_ancs_supported_bonding_found = BT_BONDING_ID_INVALID;
  prv_file_each(prv_get_first_ancs_bonding_itr, &first_ancs_supported_bonding_found);

  return first_ancs_supported_bonding_found;
}

bool bt_persistent_storage_is_ble_ancs_bonding(BTBondingID bonding) {
  BtPersistBondingData data;
  prv_file_get(&bonding, sizeof(bonding), &data, sizeof(data));

  if (data.type == BtPersistBondingTypeBLE) {
    return data.ble_data.supports_ancs;
  }
  return false;
}

bool bt_persistent_storage_has_ble_ancs_bonding(void) {
  return bt_persistent_storage_get_ble_ancs_bonding() != BT_BONDING_ID_INVALID;
}

bool bt_persistent_storage_has_active_ble_gateway_bonding(void) {
  return prv_has_active_gateway_by_type(BtPersistBondingTypeBLE);
}

typedef void (*BtPersistBondingDBEachBLEInternal)(BTBondingID key,
                                                  BtPersistBondingData *stored_data, void *ctx);

typedef struct {
  BtPersistBondingDBEachBLEInternal cb;
  void *cb_data;
} ForEachBLEPairingInternalData;

typedef void (*BtPersistCCCDDBEachBLEInternal)(BTCCCDID key,
                                               BtPersistCCCDData *stored_data, void *ctx);

typedef struct {
  BtPersistCCCDDBEachBLEInternal cb;
  void *cb_data;
} ForEachBLECCCDInternalData;

typedef struct {
  BtPersistBondingDBEachBLE cb;
  void *cb_data;
} ForEachBLEPairingData;

static void prv_public_for_each_ble_cb(BTBondingID key,
                                       BtPersistBondingData *stored_data, void *context) {
  ForEachBLEPairingData *itr_data = (ForEachBLEPairingData *)context;
  itr_data->cb(&stored_data->ble_data.pairing_info.identity,
               &stored_data->ble_data.pairing_info.irk,
               stored_data->ble_data.name, &key, itr_data->cb_data);
}

static bool prv_ble_pairing_internal_for_each_itr(SettingsFile *file,
                                                  SettingsRecordInfo *info, void *context) {
  // check entry is valid
  if (info->val_len == 0 || info->key_len != sizeof(BTBondingID)) {
    return true; // continue iterating
  }

  ForEachBLEPairingInternalData *internal_itr_data = (ForEachBLEPairingInternalData*) context;

  BTBondingID key;
  info->get_key(file, (uint8_t*) &key, info->key_len);

  BtPersistBondingData stored_data;
  info->get_val(file, (uint8_t*) &stored_data, MIN((unsigned)info->val_len, sizeof(stored_data)));

  if (stored_data.type == BtPersistBondingTypeBLE) {
    internal_itr_data->cb(key, &stored_data, internal_itr_data->cb_data);
  }

  return true;
}

static bool prv_ble_cccd_internal_for_each_itr(SettingsFile *file,
                                               SettingsRecordInfo *info, void *context) {
  // check entry is valid
  if (info->val_len == 0 || info->key_len != sizeof(BTCCCDID)) {
    return true; // continue iterating
  }

  ForEachBLECCCDInternalData *internal_itr_data = (ForEachBLECCCDInternalData*) context;

  BTCCCDID key;
  info->get_key(file, (uint8_t*) &key, info->key_len);

  BtPersistCCCDData stored_data;
  info->get_val(file, (uint8_t*) &stored_data, MIN((unsigned)info->val_len, sizeof(stored_data)));

  internal_itr_data->cb(key, &stored_data, internal_itr_data->cb_data);

  return true;
}

void bt_persistent_storage_for_each_ble_pairing(BtPersistBondingDBEachBLE cb, void *context) {
  ForEachBLEPairingData itr_data = {
    .cb = cb,
    .cb_data = context,
  };
  ForEachBLEPairingInternalData internal_itr_data = {
    .cb = prv_public_for_each_ble_cb,
    .cb_data = &itr_data,
  };
  prv_file_each(prv_ble_pairing_internal_for_each_itr, &internal_itr_data);
}

static void prv_register_bondings_for_each_ble_cb(BTBondingID key,
                                                  BtPersistBondingData *stored_data,
                                                  void *context) {
  BleBonding bonding;
  prv_init_and_assign_ble_bonding(&bonding, stored_data);
  bonding.is_gateway = stored_data->ble_data.is_gateway;
  bonding.flags = stored_data->ble_data.flags;
  bt_driver_handle_host_added_bonding(&bonding);
}

static void prv_register_cccd_for_each_ble_cb(BTCCCDID key,
                                              BtPersistCCCDData *stored_data,
                                              void *context) {
  BleCCCD cccd = {
    .peer = stored_data->peer,
    .chr_val_handle = stored_data->chr_val_handle,
    .flags = stored_data->flags,
    .value_changed = stored_data->value_changed,
  };

  bt_driver_handle_host_added_cccd(&cccd);
}

void bt_persistent_storage_register_existing_ble_bondings(void) {
  ForEachBLEPairingInternalData internal_itr_data_bonding = {
    .cb = prv_register_bondings_for_each_ble_cb,
  };
  prv_file_each(prv_ble_pairing_internal_for_each_itr, &internal_itr_data_bonding);

  ForEachBLECCCDInternalData internal_itr_data_cccd = {
    .cb = prv_register_cccd_for_each_ble_cb,
  };
  prv_file_each(prv_ble_cccd_internal_for_each_itr, &internal_itr_data_cccd);
}

typedef struct {
  const BTDeviceInternal *peer;
  uint16_t chr_val_handle;
  BTCCCDID id;
} FindCCCDItrData;

static bool prv_find_cccd_itr(SettingsFile *file,
                              SettingsRecordInfo *info, void *context) {
  if (info->val_len == 0 || info->key_len != sizeof(BTCCCDID)) {
    return true; // continue iterating
  }

  FindCCCDItrData *itr_data = (FindCCCDItrData *) context;

  BTCCCDID key;
  info->get_key(file, (uint8_t*) &key, info->key_len);

  BtPersistCCCDData stored_data;
  info->get_val(file, (uint8_t*) &stored_data, MIN((unsigned)info->val_len, sizeof(stored_data)));

  if (bt_device_internal_equal(itr_data->peer, &stored_data.peer) && 
      stored_data.chr_val_handle == itr_data->chr_val_handle) {
    itr_data->id = key;
    return false; // stop iterating
  }

  return true;
}

BTCCCDID bt_persistent_storage_store_cccd(const BleCCCD *cccd) {
  PBL_ASSERTN(cccd != NULL);

  FindCCCDItrData itr_data = {
    .peer = &cccd->peer,
    .chr_val_handle = cccd->chr_val_handle,
    .id = BT_CCCD_ID_INVALID,
  };
  prv_file_each(prv_find_cccd_itr, &itr_data);

  BTCCCDID cccd_id = itr_data.id;
  if (cccd_id == BT_CCCD_ID_INVALID) {
    cccd_id = prv_get_free_cccd();
    if (cccd_id == BT_CCCD_ID_INVALID) {
        return BT_CCCD_ID_INVALID;
    }
  }

  BtPersistCCCDData stored_data = {
    .peer = cccd->peer,
    .chr_val_handle = cccd->chr_val_handle,
    .flags = cccd->flags,
    .value_changed = cccd->value_changed,
  };

  BtPersistCCCD stored_cccd = {
    .id = cccd_id,
    .data = stored_data,
  };

  GapBondingFileSetStatus status = prv_file_set(&stored_cccd.id, sizeof(stored_cccd.id),
                                                &stored_cccd.data, sizeof(stored_cccd.data));
  if (status == GapBondingFileSetFail) {
    return BT_CCCD_ID_INVALID;
  }

  return cccd_id;
}

bool bt_persistent_storage_delete_cccd(const BTDeviceInternal *peer, uint16_t chr_val_handle) {
  BTCCCDID cccd_id;

  FindCCCDItrData itr_data = {
    .peer = peer,
    .chr_val_handle = chr_val_handle,
    .id = BT_CCCD_ID_INVALID
  };
  prv_file_each(prv_find_cccd_itr, &itr_data);

  if (itr_data.id == BT_CCCD_ID_INVALID) {
    return false;
  }

  cccd_id = itr_data.id;
  if (prv_file_set(&cccd_id, sizeof(cccd_id), NULL, 0) == GapBondingFileSetFail) {
    return false;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Local Device Info

void bt_persistent_storage_set_active_gateway(BTBondingID bonding) {
  BTBondingID old_active_gateway;
  int read_size = prv_file_get(&ACTIVE_GATEWAY_KEY, sizeof(ACTIVE_GATEWAY_KEY),
                               &old_active_gateway, sizeof(old_active_gateway));

  if (!read_size || old_active_gateway != bonding) {
    prv_file_set(&ACTIVE_GATEWAY_KEY, sizeof(ACTIVE_GATEWAY_KEY), &bonding, sizeof(bonding));
    bt_persistent_storage_set_unfaithful(true);
    bt_persistent_storage_set_cached_system_capabilities(NULL);
  }
}

bool bt_persistent_storage_get_active_gateway(BTBondingID *bonding_out,
                                              BtPersistBondingType *type_out) {
  BTBondingID active_gateway;
  int read_size = prv_file_get(&ACTIVE_GATEWAY_KEY, sizeof(ACTIVE_GATEWAY_KEY),
                               &active_gateway, sizeof(active_gateway));

  if (!read_size || active_gateway == BT_BONDING_ID_INVALID) {
    return false;
  }

  if (bonding_out) {
    *bonding_out = active_gateway;
  }
  if (type_out) {
    *type_out = prv_get_type_for_id(active_gateway);
  }

  return true;
}

bool bt_persistent_storage_is_unfaithful(void) {
  return prv_file_get_bool(&IS_UNFAITHFUL_KEY, sizeof(IS_UNFAITHFUL_KEY), true /* default */);
}

void bt_persistent_storage_set_unfaithful(bool is_unfaithful) {
  PBL_LOG_INFO("Marking the watch as %s", is_unfaithful ? "unfaithful" : "faithful");
  prv_file_set(&IS_UNFAITHFUL_KEY, sizeof(IS_UNFAITHFUL_KEY),
               &is_unfaithful, sizeof(is_unfaithful));
}

bool bt_persistent_storage_get_root_key(SMRootKeyType key_type, SM128BitKey *key_out) {
  SM128BitKey keys[SMRootKeyTypeNum];
  int read_size = prv_file_get(&ROOT_KEYS_KEY, sizeof(ROOT_KEYS_KEY),
                               &keys, sizeof(keys));
  if (!read_size) {
    return false;
  }
  SM128BitKey nil_key = {};
  if (0 == memcmp(&nil_key, &keys[key_type], sizeof(nil_key))) {
    return false;
  }

  if (key_out) {
    memcpy(key_out, &keys[key_type], sizeof(keys[key_type]));
  }

  return true;
}

void bt_persistent_storage_set_root_keys(SM128BitKey *keys_in) {
  if (!keys_in) {
    return;
  }
  shared_prf_storage_set_root_keys(keys_in);

  prv_file_set(&ROOT_KEYS_KEY, sizeof(ROOT_KEYS_KEY),
               keys_in, SMRootKeyTypeNum * sizeof(SM128BitKey));
}

bool bt_persistent_storage_get_local_device_name(char *local_device_name_out, size_t max_size) {
  int read_size = prv_file_get(&DEVICE_NAME_KEY, sizeof(DEVICE_NAME_KEY),
                               local_device_name_out, max_size);
  if (!read_size) {
    return false;
  }
  return true;
}

void bt_persistent_storage_set_local_device_name(char *local_device_name, size_t size) {
  if (!local_device_name) {
    return;
  }
  shared_prf_storage_set_local_device_name(local_device_name);

  prv_file_set(&DEVICE_NAME_KEY, sizeof(DEVICE_NAME_KEY),
                               local_device_name, size);
}

bool bt_persistent_storage_get_airplane_mode_enabled(void) {
  return prv_file_get_bool(&AIRPLANE_MODE_KEY, sizeof(AIRPLANE_MODE_KEY), false /* default */);
}

void bt_persistent_storage_set_airplane_mode_enabled(bool new_state) {
  prv_file_set(&AIRPLANE_MODE_KEY, sizeof(AIRPLANE_MODE_KEY),
               &new_state, sizeof(bool));
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Remote Device Info

static void prv_load_cached_system_capabilities(PebbleProtocolCapabilities *capabilities_out) {
  if (!capabilities_out) {
    return;
  }

  const int read_size = prv_file_get(&SYSTEM_CAPABILITIES_KEY, sizeof(SYSTEM_CAPABILITIES_KEY),
                                     capabilities_out, sizeof(PebbleProtocolCapabilities));
  // Default to zero capabilities if no entry found
  if (!read_size) {
    *capabilities_out = (PebbleProtocolCapabilities) {};
  }
}

void bt_persistent_storage_get_cached_system_capabilities(
    PebbleProtocolCapabilities *capabilities_out) {
  if (!capabilities_out) {
    return;
  }

  prv_lock();
  {
    *capabilities_out = s_cached_system_capabilities;
  }
  prv_unlock();
}

void bt_persistent_storage_set_cached_system_capabilities(
    const PebbleProtocolCapabilities *capabilities) {
  PebbleProtocolCapabilities diff = {};

  prv_lock();
  {
    // If we were passed a null pointer, we'll just clear the cached capability bits
    if (capabilities) {
      diff.flags = s_cached_system_capabilities.flags ^ capabilities->flags;
      s_cached_system_capabilities = *capabilities;
    } else {
      diff.flags = s_cached_system_capabilities.flags;
      s_cached_system_capabilities = (PebbleProtocolCapabilities) {};
    }
  }
  prv_unlock();

  // Only update the cache if the capability flags changed
  if (diff.flags) {
    prv_file_set(&SYSTEM_CAPABILITIES_KEY, sizeof(SYSTEM_CAPABILITIES_KEY),
                 capabilities, sizeof(PebbleProtocolCapabilities));

    PebbleEvent event = {
      .type = PEBBLE_CAPABILITIES_CHANGED_EVENT,
      .capabilities.flags_diff = diff,
    };
    event_put(&event);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Common

void bt_persistent_storage_init(void) {
  // Note: this gets called well before the BT stack is initialized, make sure there is no code
  // that tries to use the BT stack in this path.
  s_db_mutex = mutex_create();

  prv_load_data_from_prf();

  // Clean up any leftover state where the bonding DB ended up with more than one BLE pairing
  // (e.g. inherited from an older firmware build, or a PRF pairing layered on an existing one).
  prv_prune_stale_ble_bondings();

  // Load cached capability bits from flash
  prv_load_cached_system_capabilities(&s_cached_system_capabilities);
}

static void prv_delete_all_pairings_itr(SettingsFile *old_file, SettingsFile *new_file,
                                        SettingsRecordInfo *info, void *context) {
  if (info->key_len == sizeof(BTBondingID)) {
    // Skip pairing entries
    return;
  }

  // Re-write non-pairing entries
  void *key = kernel_zalloc_check(info->key_len);
  info->get_key(old_file, key, info->key_len);

  void *data =  kernel_malloc_check(info->val_len);
  info->get_val(old_file, data, info->val_len);

  settings_file_set(new_file, key, info->key_len, &data, info->val_len);

  kernel_free(key);
  kernel_free(data);
}

void bt_persistent_storage_delete_all_pairings(void) {
  prv_lock();
  {
    SettingsFile fd;
    status_t rv = settings_file_open(&fd, BT_PERSISTENT_STORAGE_FILE_NAME,
                                     BT_PERSISTENT_STORAGE_FILE_SIZE);
    if (rv) {
      return;
    }

    settings_file_rewrite(&fd, prv_delete_all_pairings_itr, NULL);
    settings_file_close(&fd);
  }
  prv_unlock();

  shared_prf_storage_erase_ble_pairing_data();
}

static void prv_dump_bonding_db_data(char display_buf[DISPLAY_BUF_LEN],
                                     BTBondingID bond_id, BtPersistBondingData *data) {
  bool matches_prf;

  if (data->type == BtPersistBondingTypeBTClassic) {
    prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN, "Classic Key %d (legacy)",
                         (int)bond_id);
  } else if (data->type == BtPersistBondingTypeBLE) {
    prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN, "LE Key %d",
                         (int)bond_id);

    prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN, " ANCS: %d Gateway: %d Req Pin: %d",
                         (int)data->ble_data.supports_ancs,
                         (int)data->ble_data.is_gateway,
                         (int)data->ble_data.requires_address_pinning);

    prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN, " Name: %s",
                         data->ble_data.name);

    SMPairingInfo info = {};
    bt_persistent_storage_assign_sm_pairing_info(&info, &data->ble_data.pairing_info);
    bluetooth_persistent_storage_debug_dump_ble_pairing_info(&display_buf[0], &info);

    // does this info match the key stored in shared resources
    SMPairingInfo sprf_info = {};
    bool requires_address_pinning;
    uint8_t flags;
    shared_prf_storage_get_ble_pairing_data(&sprf_info, NULL, &requires_address_pinning,
                                            &flags);
    matches_prf = (memcmp(&sprf_info, &info, sizeof(sprf_info)) == 0);
    matches_prf &= (requires_address_pinning == data->ble_data.requires_address_pinning);
    matches_prf &= (flags == data->ble_data.flags);
    prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN,
                             " SMPairingInfo matches Shared PRF: %s",
                             bool_to_str(matches_prf));
  } else {
    prompt_send_response("Unhandled type of GapBondingDB Data!");
    PBL_HEXDUMP_D_PROMPT(LOG_LEVEL_DEBUG, (uint8_t *)&data, sizeof(*data));
  }
}

static void prv_dump_cccd_db_data(char display_buf[DISPLAY_BUF_LEN],
                                  BTCCCDID cccd_id, BtPersistCCCDData *data) {
  prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN, "CCCD Key %d", (int)cccd_id);

  prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN, " Peer Address: "BT_DEVICE_ADDRESS_FMT,
                           BT_DEVICE_ADDRESS_XPLODE_PTR(&data->peer.address));
  prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN, " Handle: 0x%" PRIx16, 
                           data->chr_val_handle);
  prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN, " Flags: 0x%" PRIx16, data->flags);
  prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN, " Value Changed: %s",
                           bool_to_str(data->value_changed));
}

static bool prv_dump_bt_persistent_storage_contents(
    SettingsFile *file, SettingsRecordInfo *info, void *context) {
  if (info->key_len == 0 || info->val_len == 0) {
    prompt_send_response("key or val of 0 length");
    return true;
  }
  char *display_buf = kernel_malloc_check(DISPLAY_BUF_LEN);

  // get the key
  uint8_t key[info->key_len];
  memset(key, 0x0, info->key_len);
  info->get_key(file, &key[0], info->key_len);
  // prompt_send_response("Raw dump Key");
  // PBL_HEXDUMP_D_PROMPT(LOG_LEVEL_DEBUG, (uint8_t *)&key, info->key_len);

  uint8_t val[info->val_len];
  memset(val, 0x0, info->val_len);
  info->get_val(file, &val[0], info->val_len);
  // prompt_send_response("Raw dump Value:");
  // PBL_HEXDUMP_D_PROMPT(LOG_LEVEL_DEBUG, (uint8_t *)&val, info->val_len);

  if (memcmp(key, ACTIVE_GATEWAY_KEY, info->key_len) == 0) {
    PBL_ASSERTN(info->val_len == sizeof(BTBondingID));
    BTBondingID id;
    memcpy(&id, val, sizeof(BTBondingID));
    prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN, "%s : %d",
                         ACTIVE_GATEWAY_KEY, (int)id);

  } else if (memcmp(key, IS_UNFAITHFUL_KEY, info->key_len) == 0) {
    PBL_ASSERTN(info->val_len == sizeof(bool));
    bool is_unfaithful;
    memcpy(&is_unfaithful, val, sizeof(bool));
    prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN, "%s  : %d",
                         IS_UNFAITHFUL_KEY, (int)is_unfaithful);

  } else if (memcmp(key, ROOT_KEYS_KEY, info->key_len) == 0) {
    SM128BitKey root_keys[SMRootKeyTypeNum], sprf_root_keys[SMRootKeyTypeNum];
    PBL_ASSERTN(info->val_len == sizeof(root_keys));
    memcpy(&root_keys, val, sizeof(root_keys));

    bluetooth_persistent_storage_debug_dump_root_keys(&root_keys[SMRootKeyTypeEncryption],
                                                      &root_keys[SMRootKeyTypeIdentity]);

    if (shared_prf_storage_get_root_key(
        SMRootKeyTypeEncryption, &sprf_root_keys[SMRootKeyTypeEncryption]) &&
        shared_prf_storage_get_root_key(
          SMRootKeyTypeIdentity,  &sprf_root_keys[SMRootKeyTypeIdentity])) {
      bool root_keys_match =
        memcmp(&root_keys, &sprf_root_keys, sizeof(SM128BitKey) * SMRootKeyTypeNum) == 0;
      prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN,
                           " Root keys match shared prf: %s",
                           bool_to_str(root_keys_match));
    }
  } else if (memcmp(key, DEVICE_NAME_KEY, info->key_len) == 0) {
    char dev_name[info->val_len + 1];
    memcpy(&dev_name, val, info->val_len);
    prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN, "Device Name: %s",
                         dev_name);
  } else if (memcmp(key, BLE_PINNED_ADDRESS_KEY, info->key_len) == 0) {
    if (info->val_len == sizeof(BTDeviceAddress)) {
      const BTDeviceAddress *address = (const BTDeviceAddress *)val;
      prompt_send_response_fmt(display_buf, DISPLAY_BUF_LEN,
                               "Pinned address: "BT_DEVICE_ADDRESS_FMT,
                               BT_DEVICE_ADDRESS_XPLODE_PTR(address));
    }
  } else if (info->key_len == sizeof(BTBondingID)) {
    BTBondingID bonding_id;

    memcpy(&bonding_id, key, sizeof(BTBondingID));
    PBL_ASSERTN(sizeof(BtPersistBondingData) == info->val_len);
    prv_dump_bonding_db_data(display_buf, bonding_id, (BtPersistBondingData *)&val);
  } else if (info->key_len == sizeof(BTCCCDID)) {
    BTCCCDID cccd_id;

    memcpy(&cccd_id, key, sizeof(BTCCCDID));
    PBL_ASSERTN(sizeof(BtPersistCCCDData) == info->val_len);
    prv_dump_cccd_db_data(display_buf, cccd_id, (BtPersistCCCDData *)&val);
  } else {
    prompt_send_response("Something new be in the bonding DB!");
    PBL_HEXDUMP_D_PROMPT(LOG_LEVEL_DEBUG, &key[0], info->key_len);
    PBL_HEXDUMP_D_PROMPT(LOG_LEVEL_DEBUG, &val[0], info->val_len);
  }

  prompt_send_response("");

  kernel_free(display_buf);
  return true;
}

void bluetooth_persistent_storage_dump_contents(void) {
  prv_file_each(prv_dump_bt_persistent_storage_contents, NULL);
}
