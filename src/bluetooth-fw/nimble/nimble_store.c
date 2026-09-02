/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <bluetooth/bonding_sync.h>
#include <bluetooth/gap_le_connect.h>
#include <bluetooth/sm_types.h>
#include <host/ble_hs.h>
#include <host/ble_store.h>
#include <kernel/event_loop.h>
#include <kernel/pbl_malloc.h>
#include "pbl/kernel/mutex.h"
#include <pbl/services/bluetooth/bluetooth_persistent_storage.h>
#include <string.h>
#include <pbl/logging/logging.h>
#include <system/passert.h>
#include <pbl/util/list.h>

#include "nimble_type_conversions.h"

PBL_LOG_MODULE_DECLARE(bt, CONFIG_BT_LOG_LEVEL);

#define KEY_SIZE 16

#define BLE_FLAG_SECURE_CONNECTIONS 0x01
#define BLE_FLAG_AUTHENTICATED 0x02

typedef struct {
  ListNode node;
  struct ble_store_value_sec value_sec;
} BleStoreValueSec;

typedef struct {
  ListNode node;
  struct ble_store_value_cccd value_cccd;
} BleStoreValueCCCD;

typedef struct {
  const struct ble_store_key_cccd *key;
  unsigned int skipped;
} BleStoreCCCDFindContext;

static BleStoreValueSec *s_peer_value_secs;
static BleStoreValueSec *s_our_value_secs;
static BleStoreValueCCCD *s_cccds;

static PBL_MUTEX_DEFINE(s_store_mutex);

static bool prv_nimble_store_find_sec_cb(ListNode *node, void *data) {
  BleStoreValueSec *s = (BleStoreValueSec *)node;
  struct ble_store_key_sec *key_sec = (struct ble_store_key_sec *)data;

  return ble_addr_cmp(&s->value_sec.peer_addr, &key_sec->peer_addr) == 0;
}

static ListNode **prv_find_sec_list_for_obj_type(const int obj_type) {
  switch (obj_type) {
    case BLE_STORE_OBJ_TYPE_OUR_SEC:
      return (ListNode **)&s_our_value_secs;
    case BLE_STORE_OBJ_TYPE_PEER_SEC:
      return (ListNode **)&s_peer_value_secs;
    default:
      PBL_ASSERT(0, "Unknown store object type");
  }
}

static BleStoreValueSec *prv_nimble_store_find_sec(const int obj_type,
                                                const struct ble_store_key_sec *key_sec) {
  ListNode *sec_list = *prv_find_sec_list_for_obj_type(obj_type);

  if (!ble_addr_cmp(&key_sec->peer_addr, BLE_ADDR_ANY)) {
    return (BleStoreValueSec *)list_get_at(sec_list, key_sec->idx);
  } else if (key_sec->idx == 0) {
    return (BleStoreValueSec *)list_find(sec_list, prv_nimble_store_find_sec_cb,
                                      (void *)&key_sec->peer_addr);
  }

  return NULL;
}

static int prv_nimble_store_read_sec(const int obj_type, const struct ble_store_key_sec *key_sec,
                                     struct ble_store_value_sec *value_sec) {
  int ret = 0;
  BleStoreValueSec *s;

  pbl_mutex_lock(&s_store_mutex, PBL_FOREVER);

  s = prv_nimble_store_find_sec(obj_type, key_sec);
  if (s == NULL) {
    ret = BLE_HS_ENOENT;
    goto unlock;
  }

  *value_sec = s->value_sec;

unlock:
  pbl_mutex_unlock(&s_store_mutex);

  return ret;
}

static BleStoreValueSec *prv_nimble_store_upsert_sec(const int obj_type,
                                                  const struct ble_store_value_sec *value_sec) {
  BleStoreValueSec *s;
  struct ble_store_key_sec key_sec;
  ble_store_key_from_value_sec(&key_sec, value_sec);
  ListNode **sec_list = prv_find_sec_list_for_obj_type(obj_type);

  pbl_mutex_lock(&s_store_mutex, PBL_FOREVER);

  s = prv_nimble_store_find_sec(obj_type, &key_sec);
  if (s == NULL) {
    s = kernel_zalloc_check(sizeof(BleStoreValueSec));
    if (*sec_list == NULL) {
      *sec_list = (ListNode *)s;
    } else {
      list_append(*sec_list, (ListNode *)s);
    }
  }

  s->value_sec = *value_sec;

  pbl_mutex_unlock(&s_store_mutex);

  return s;
}

static void prv_convert_peer_sec_to_bonding(const struct ble_store_value_sec *value_sec,
                                            BleBonding *bonding) {
  if (value_sec->ltk_present) {
    bonding->pairing_info.is_remote_encryption_info_valid = true;
    bonding->pairing_info.remote_encryption_info.ediv = value_sec->ediv;
    bonding->pairing_info.remote_encryption_info.rand = value_sec->rand_num;
    memcpy(bonding->pairing_info.remote_encryption_info.ltk.data, value_sec->ltk, KEY_SIZE);
  }

  if (value_sec->irk_present) {
    bonding->pairing_info.is_remote_identity_info_valid = true;
    memcpy(bonding->pairing_info.irk.data, value_sec->irk, KEY_SIZE);
  }
}

static void prv_convert_our_sec_to_bonding(const struct ble_store_value_sec *value_sec,
                                           BleBonding *bonding) {
  if (value_sec->ltk_present) {
    bonding->pairing_info.is_local_encryption_info_valid = true;
    bonding->pairing_info.local_encryption_info.ediv = value_sec->ediv;
    bonding->pairing_info.local_encryption_info.rand = value_sec->rand_num;
    memcpy(bonding->pairing_info.local_encryption_info.ltk.data, value_sec->ltk, KEY_SIZE);
  }
}

static void prv_notify_irk_updated(const struct ble_store_value_sec *value_sec) {
  BleIRKChange irk_change_event;

  irk_change_event.irk_valid = true;
  memcpy(irk_change_event.irk.data, value_sec->irk, KEY_SIZE);

  nimble_addr_to_pebble_device(&value_sec->peer_addr, &irk_change_event.device);

  bt_driver_handle_le_connection_handle_update_irk(&irk_change_event);
}

static void prv_notify_host_bonding_changed(const int obj_type,
                                            const struct ble_store_value_sec *value_sec) {
  int rc;
  BleBonding bonding;
  BTDeviceAddress addr;
  struct ble_store_key_sec key_sec;
  struct ble_store_value_sec existing_value_sec;

  ble_store_key_from_value_sec(&key_sec, value_sec);

  // persist bonding
  memset(&bonding, 0, sizeof(bonding));

  bonding.is_gateway = true;

  // read any existing data of the opposite type and combine with the new data before sending to the
  // host
  switch (obj_type) {
    case BLE_STORE_OBJ_TYPE_PEER_SEC:
      rc = prv_nimble_store_read_sec(BLE_STORE_OBJ_TYPE_OUR_SEC, &key_sec, &existing_value_sec);
      if (rc == 0) {
        prv_convert_our_sec_to_bonding(&existing_value_sec, &bonding);
      }
      prv_convert_peer_sec_to_bonding(value_sec, &bonding);

      break;
    case BLE_STORE_OBJ_TYPE_OUR_SEC:
      rc = prv_nimble_store_read_sec(BLE_STORE_OBJ_TYPE_PEER_SEC, &key_sec, &existing_value_sec);
      if (rc == 0) {
        prv_convert_peer_sec_to_bonding(&existing_value_sec, &bonding);
      }
      prv_convert_our_sec_to_bonding(value_sec, &bonding);
      break;
  }

  if (value_sec->sc) {
    bonding.flags |= BLE_FLAG_SECURE_CONNECTIONS;
  }

  if (value_sec->authenticated) {
    bonding.flags |= BLE_FLAG_AUTHENTICATED;
  }

  nimble_addr_to_pebble_device(&value_sec->peer_addr, &bonding.pairing_info.identity);

  nimble_addr_to_pebble_addr(&value_sec->peer_addr, &addr);

  if (bonding.pairing_info.is_remote_encryption_info_valid) {
    bt_driver_cb_handle_create_bonding(&bonding, &addr);
  } else {
    PBL_LOG_DBG("Skipping notifying OS of our keys");
  }
}

typedef struct {
  int obj_type;
  struct ble_store_value_sec value_sec;
} NimbleStoreSecWrittenContext;

static void prv_handle_sec_written_cb(void *data) {
  NimbleStoreSecWrittenContext *ctx = data;

  // inform about new IRK
  if (ctx->obj_type == BLE_STORE_OBJ_TYPE_PEER_SEC && ctx->value_sec.irk_present) {
    prv_notify_irk_updated(&ctx->value_sec);
  }

  prv_notify_host_bonding_changed(ctx->obj_type, &ctx->value_sec);

  kernel_free(ctx);
}

static int prv_nimble_store_write_sec(const int obj_type,
                                      const struct ble_store_value_sec *value_sec) {
  BTDeviceAddress addr;

  nimble_addr_to_pebble_addr(&value_sec->peer_addr, &addr);
  PBL_LOG_INFO("SEC write: obj=%d addr=" BT_DEVICE_ADDRESS_FMT, obj_type,
               BT_DEVICE_ADDRESS_XPLODE(addr));
  PBL_LOG_INFO("SEC write: atype=%u ltk=%u irk=%u csrk=%u sc=%u auth=%u ksz=%u",
               value_sec->peer_addr.type, value_sec->ltk_present, value_sec->irk_present,
               value_sec->csrk_present, value_sec->sc, value_sec->authenticated,
               value_sec->key_size);

  if (value_sec->key_size != KEY_SIZE || value_sec->csrk_present) {
    PBL_LOG_ERR("Unsupported security parameters, dropping bonding keys");
    return BLE_HS_ENOTSUP;
  }

  prv_nimble_store_upsert_sec(obj_type, value_sec);

  NimbleStoreSecWrittenContext *ctx = kernel_malloc_check(sizeof(*ctx));
  *ctx = (NimbleStoreSecWrittenContext) {
    .obj_type = obj_type,
    .value_sec = *value_sec,
  };
  launcher_task_add_callback(prv_handle_sec_written_cb, ctx);

  return 0;
}

static int prv_nimble_store_delete_sec(int obj_type, const struct ble_store_key_sec *key_sec) {
  BTDeviceInternal device;
  BleStoreValueSec *s;
  ListNode **sec_list = prv_find_sec_list_for_obj_type(obj_type);

  pbl_mutex_lock(&s_store_mutex, PBL_FOREVER);
  s = prv_nimble_store_find_sec(obj_type, key_sec);
  if (s == NULL) {
    pbl_mutex_unlock(&s_store_mutex);
    return BLE_HS_ENOENT;
  }

  // Remove from in-memory list before calling into persistent storage,
  // so that NimBLE's ble_store_util_delete_all() loop terminates correctly.
  // Previously we relied on bt_driver_handle_host_removed_bonding() to remove
  // the entry as a side-effect, but that reads the identity from SPRF which
  // may already be erased by a prior iteration, causing an infinite loop.
  list_remove((ListNode *)s, sec_list, NULL);
  pbl_mutex_unlock(&s_store_mutex);

  kernel_free(s);

  nimble_addr_to_pebble_device(&key_sec->peer_addr, &device);
  PBL_LOG_INFO("SEC delete: obj=%d addr=" BT_DEVICE_ADDRESS_FMT, obj_type,
               BT_DEVICE_ADDRESS_XPLODE(device.address));
  bt_persistent_storage_delete_ble_pairing_by_addr(&device);

  return 0;
}

static bool prv_nimble_store_find_cccd_cb(ListNode *node, void *data) {
  BleStoreValueCCCD *s = (BleStoreValueCCCD *)node;
  BleStoreCCCDFindContext *ctx = data;

  if ((ble_addr_cmp(&ctx->key->peer_addr, BLE_ADDR_ANY) != 0) &&
      (ble_addr_cmp(&s->value_cccd.peer_addr, &ctx->key->peer_addr) != 0)) {
    return false;
  }

  if ((ctx->key->chr_val_handle != 0U) &&
      (s->value_cccd.chr_val_handle != ctx->key->chr_val_handle)) {
    return false;
  }

  if (ctx->key->idx > ctx->skipped) {
    ctx->skipped++;
    return false;
  }

  return true;
}

static BleStoreValueCCCD *prv_nimble_store_find_cccd(const struct ble_store_key_cccd *key_cccd) {
  BleStoreCCCDFindContext ctx = {
    .key = key_cccd,
    .skipped = 0U,
  };

  return (BleStoreValueCCCD *)list_find((ListNode *)s_cccds, prv_nimble_store_find_cccd_cb, &ctx);
}

static int prv_nimble_store_read_cccd(const struct ble_store_key_cccd *key_cccd,
                                      struct ble_store_value_cccd *value_cccd) {
  BleStoreValueCCCD *s;
  int ret;

  pbl_mutex_lock(&s_store_mutex, PBL_FOREVER);

  s = prv_nimble_store_find_cccd(key_cccd);
  if (s == NULL) {
    ret = BLE_HS_ENOENT;
    goto unlock;
  }

  *value_cccd = s->value_cccd;

unlock:
  pbl_mutex_unlock(&s_store_mutex);

  return ret;
}

static void prv_nimble_store_insert_cccd(const struct ble_store_value_cccd *value_cccd) {
  struct ble_store_key_cccd key_cccd;
  BleStoreValueCCCD *s;

  ble_store_key_from_value_cccd(&key_cccd, value_cccd);

  s = prv_nimble_store_find_cccd(&key_cccd);
  if (s == NULL) {
    s = kernel_zalloc_check(sizeof(BleStoreValueCCCD));
    if (s_cccds == NULL) {
      s_cccds = s;
    } else {
      list_append((ListNode *)s_cccds, (ListNode *)s);
    }
  }

  s->value_cccd = *value_cccd;
}

static int prv_nimble_store_write_cccd(const struct ble_store_value_cccd *value_cccd) {
  BleCCCD cccd;
  BTCCCDID cccd_id;

  nimble_addr_to_pebble_device(&value_cccd->peer_addr, &cccd.peer);
  cccd.chr_val_handle = value_cccd->chr_val_handle;
  cccd.flags = value_cccd->flags;
  cccd.value_changed = value_cccd->value_changed;

  cccd_id = bt_persistent_storage_store_cccd(&cccd);
  if (cccd_id == BT_CCCD_ID_INVALID) {
    return BLE_HS_ESTORE_CAP;
  }

  pbl_mutex_lock(&s_store_mutex, PBL_FOREVER);
  prv_nimble_store_insert_cccd(value_cccd);
  pbl_mutex_unlock(&s_store_mutex);

  return 0;
}

static int prv_nimble_store_delete_cccd(const struct ble_store_key_cccd *key_cccd) {
  bool res;
  int ret = 0;
  BTDeviceInternal peer;
  BleStoreValueCCCD *s;

  nimble_addr_to_pebble_device(&key_cccd->peer_addr, &peer);
  res = bt_persistent_storage_delete_cccd(&peer, key_cccd->chr_val_handle);
  if (!res) {
    return BLE_HS_ENOENT;
  }

  pbl_mutex_lock(&s_store_mutex, PBL_FOREVER);

  s = prv_nimble_store_find_cccd(key_cccd);
  if (s == NULL) {
    ret = BLE_HS_ENOENT;
    goto unlock;
  }

  list_remove((ListNode *)s, (ListNode **)&s_cccds, NULL);
  kernel_free(s);

unlock:
  pbl_mutex_unlock(&s_store_mutex);

  return ret;
}

static int prv_nimble_store_read(const int obj_type, const union ble_store_key *key,
                                 union ble_store_value *value) {
  switch (obj_type) {
    case BLE_STORE_OBJ_TYPE_OUR_SEC:
    case BLE_STORE_OBJ_TYPE_PEER_SEC:
      return prv_nimble_store_read_sec(obj_type, &key->sec, &value->sec);
    case BLE_STORE_OBJ_TYPE_CCCD:
      return prv_nimble_store_read_cccd(&key->cccd, &value->cccd);
    default:
      return BLE_HS_ENOTSUP;
  }
}

static int prv_nimble_store_write(int obj_type, const union ble_store_value *val) {
  switch (obj_type) {
    case BLE_STORE_OBJ_TYPE_OUR_SEC:
    case BLE_STORE_OBJ_TYPE_PEER_SEC:
      return prv_nimble_store_write_sec(obj_type, &val->sec);
    case BLE_STORE_OBJ_TYPE_CCCD:
      return prv_nimble_store_write_cccd(&val->cccd);
    default:
      return BLE_HS_ENOTSUP;
  }
}

static int prv_nimble_store_delete(int obj_type, const union ble_store_key *key) {
  switch (obj_type) {
    case BLE_STORE_OBJ_TYPE_OUR_SEC:
    case BLE_STORE_OBJ_TYPE_PEER_SEC:
      return prv_nimble_store_delete_sec(obj_type, &key->sec);
    case BLE_STORE_OBJ_TYPE_CCCD:
      return prv_nimble_store_delete_cccd(&key->cccd);
    default:
      return BLE_HS_ENOTSUP;
  }
}

static int prv_nimble_store_gen_key(uint8_t key, struct ble_store_gen_key *gen_key,
                                    uint16_t conn_handle) {
  SM128BitKey stored_keys[SMRootKeyTypeNum];

  if (!bt_persistent_storage_get_root_key(SMRootKeyTypeIdentity,
                                          &stored_keys[SMRootKeyTypeIdentity])) {
    int ret;

    ret = ble_hs_hci_rand(stored_keys, sizeof(stored_keys));
    if (ret != 0) {
      PBL_LOG_ERR("Could not generate root keys: %d", ret);
      return ret;
    }

    bt_persistent_storage_set_root_keys(stored_keys);
  }

  switch (key) {
    case BLE_STORE_GEN_KEY_IRK:
      memcpy(gen_key->irk, stored_keys[SMRootKeyTypeIdentity].data, KEY_SIZE);
      break;
    default:
      return BLE_HS_ENOTSUP;
  }

  return 0;
}

void nimble_store_init(void) {
  ble_hs_cfg.store_read_cb = prv_nimble_store_read;
  ble_hs_cfg.store_write_cb = prv_nimble_store_write;
  ble_hs_cfg.store_delete_cb = prv_nimble_store_delete;
  ble_hs_cfg.store_gen_key_cb = prv_nimble_store_gen_key;
}

static bool prv_store_value_free(ListNode *node, void *context) {
  kernel_free(node);
  return false;
}

void nimble_store_unload(void) {
  pbl_mutex_lock(&s_store_mutex, PBL_FOREVER);

  list_foreach((ListNode *)s_peer_value_secs, prv_store_value_free, NULL);
  list_foreach((ListNode *)s_our_value_secs, prv_store_value_free, NULL);
  list_foreach((ListNode *)s_cccds, prv_store_value_free, NULL);

  s_peer_value_secs = NULL;
  s_our_value_secs = NULL;
  s_cccds = NULL;

  pbl_mutex_unlock(&s_store_mutex);
}

static void prv_convert_bonding_remote_to_store_val(const BleBonding *bonding,
                                                    struct ble_store_value_sec *value_sec) {
  memset(value_sec, 0, sizeof(struct ble_store_value_sec));

  value_sec->key_size = KEY_SIZE;

  if (bonding->pairing_info.is_remote_encryption_info_valid) {
    value_sec->ediv = bonding->pairing_info.remote_encryption_info.ediv;
    value_sec->rand_num = bonding->pairing_info.remote_encryption_info.rand;
    value_sec->ltk_present = true;
    memcpy(value_sec->ltk, bonding->pairing_info.remote_encryption_info.ltk.data, KEY_SIZE);
  }

  if (bonding->pairing_info.is_remote_identity_info_valid) {
    value_sec->irk_present = true;
    memcpy(value_sec->irk, bonding->pairing_info.irk.data, KEY_SIZE);
  }

  value_sec->sc = !!(bonding->flags & BLE_FLAG_SECURE_CONNECTIONS);
  value_sec->authenticated = !!(bonding->flags & BLE_FLAG_AUTHENTICATED);

  pebble_device_to_nimble_addr(&bonding->pairing_info.identity, &value_sec->peer_addr);
}

static void prv_convert_bonding_local_to_store_val(const BleBonding *bonding,
                                                   struct ble_store_value_sec *value_sec) {
  memset(value_sec, 0, sizeof(struct ble_store_value_sec));

  value_sec->key_size = KEY_SIZE;

  if (bonding->pairing_info.is_local_encryption_info_valid) {
    value_sec->ediv = bonding->pairing_info.local_encryption_info.ediv;
    value_sec->rand_num = bonding->pairing_info.local_encryption_info.rand;
    value_sec->ltk_present = true;
    memcpy(value_sec->ltk, bonding->pairing_info.local_encryption_info.ltk.data, KEY_SIZE);
  }

  value_sec->sc = !!(bonding->flags & BLE_FLAG_SECURE_CONNECTIONS);
  value_sec->authenticated = !!(bonding->flags & BLE_FLAG_AUTHENTICATED);

  pebble_device_to_nimble_addr(&bonding->pairing_info.identity, &value_sec->peer_addr);
}

void bt_driver_handle_host_added_bonding(const BleBonding *bonding) {
  struct ble_store_value_sec value_sec;

  PBL_LOG_INFO("Host added bonding: addr=" BT_DEVICE_ADDRESS_FMT " random=%u",
               BT_DEVICE_ADDRESS_XPLODE(bonding->pairing_info.identity.address),
               bonding->pairing_info.identity.is_random_address);
  PBL_LOG_INFO("Host added bonding: remote_enc=%u local_enc=%u irk=%u",
               bonding->pairing_info.is_remote_encryption_info_valid,
               bonding->pairing_info.is_local_encryption_info_valid,
               bonding->pairing_info.is_remote_identity_info_valid);

  prv_convert_bonding_remote_to_store_val(bonding, &value_sec);
  prv_nimble_store_upsert_sec(BLE_STORE_OBJ_TYPE_PEER_SEC, &value_sec);

  prv_convert_bonding_local_to_store_val(bonding, &value_sec);
  prv_nimble_store_upsert_sec(BLE_STORE_OBJ_TYPE_OUR_SEC, &value_sec);
}

void bt_driver_handle_host_removed_bonding(const BleBonding *bonding) {
  BleStoreValueSec *s_sec;
  struct ble_store_key_sec key_sec;

  PBL_LOG_INFO("Host removed bonding: addr=" BT_DEVICE_ADDRESS_FMT " random=%u",
               BT_DEVICE_ADDRESS_XPLODE(bonding->pairing_info.identity.address),
               bonding->pairing_info.identity.is_random_address);

  key_sec.idx = 0;
  pebble_device_to_nimble_addr(&bonding->pairing_info.identity, &key_sec.peer_addr);

  pbl_mutex_lock(&s_store_mutex, PBL_FOREVER);

  s_sec = prv_nimble_store_find_sec(BLE_STORE_OBJ_TYPE_OUR_SEC, &key_sec);
  if (s_sec != NULL) {
    list_remove((ListNode *)s_sec, (ListNode **)&s_our_value_secs, NULL);
    kernel_free(s_sec);
  }

  s_sec = prv_nimble_store_find_sec(BLE_STORE_OBJ_TYPE_PEER_SEC, &key_sec);
  if (s_sec != NULL) {
    list_remove((ListNode *)s_sec, (ListNode **)&s_peer_value_secs, NULL);
    kernel_free(s_sec);
  }

  pbl_mutex_unlock(&s_store_mutex);
}

void bt_driver_handle_host_added_cccd(const BleCCCD *cccd) {
  struct ble_store_value_cccd value_cccd;

  pebble_device_to_nimble_addr(&cccd->peer, &value_cccd.peer_addr);
  value_cccd.chr_val_handle = cccd->chr_val_handle;
  value_cccd.flags = cccd->flags;
  value_cccd.value_changed = cccd->value_changed;

  pbl_mutex_lock(&s_store_mutex, PBL_FOREVER);
  prv_nimble_store_insert_cccd(&value_cccd);
  pbl_mutex_unlock(&s_store_mutex);
}

void bt_driver_handle_host_removed_cccd(const BleCCCD *cccd) {
  BleStoreValueCCCD *s;
  struct ble_store_key_cccd key_cccd;
  
  pebble_device_to_nimble_addr(&cccd->peer, &key_cccd.peer_addr);
  key_cccd.chr_val_handle = cccd->chr_val_handle;
  key_cccd.idx = 0;

  pbl_mutex_lock(&s_store_mutex, PBL_FOREVER);

  s = prv_nimble_store_find_cccd(&key_cccd);
  if (s != NULL) {
    list_remove((ListNode *)s, (ListNode **)&s_cccds, NULL);
    kernel_free(s);
  }

  pbl_mutex_unlock(&s_store_mutex);
}
