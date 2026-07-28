/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <bluetooth/bonding_sync.h>
#include <bluetooth/gap_le_connect.h>
#include <bluetooth/sm_types.h>
#include <host/ble_hs.h>
#include <host/ble_hs_hci.h>
#include <host/ble_sm.h>
#include <host/ble_store.h>
#include <syscfg/syscfg.h>
#include <kernel/event_loop.h>
#include <kernel/pbl_malloc.h>
#include <pbl/btutil/sm_util.h>
#include <pbl/os/mutex.h>
#include <pbl/services/bluetooth/bluetooth_persistent_storage.h>
#include <pbl/services/system_task.h>
#include <string.h>
#include <pbl/logging/logging.h>
#include <system/passert.h>
#include <pbl/util/list.h>

#include "nimble_type_conversions.h"

PBL_LOG_MODULE_DECLARE(bt, CONFIG_BT_LOG_LEVEL);

#define KEY_SIZE 16

#define BLE_FLAG_SECURE_CONNECTIONS 0x01
#define BLE_FLAG_AUTHENTICATED 0x02

//! Free slots KernelBG must still have before we hand it a store *write*.
//! Purely a courtesy to the rest of the system: the store callbacks run on
//! NimbleHost, which can produce CCCD writes faster than KernelBG drains them,
//! and a dropped write self-heals -- the peer rewrites its CCCD on the next
//! connection. A dropped delete does not, so deletes are allowed the last slot.
//! @note This is not what keeps us off the blocking path any more.
//! system_task_add_callback_nonblocking() is: it cannot wait and cannot reboot,
//! which matters because we hold ble_hs_mutex here and KernelBG needs that same
//! lock (advertising, the GATT client op queue) to drain its queue.
#define KERNEL_BG_QUEUE_MARGIN_WRITE 4

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

static PebbleRecursiveMutex *s_store_mutex;

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

  mutex_lock_recursive(s_store_mutex);

  s = prv_nimble_store_find_sec(obj_type, key_sec);
  if (s == NULL) {
    ret = BLE_HS_ENOENT;
    goto unlock;
  }

  *value_sec = s->value_sec;

unlock:
  mutex_unlock_recursive(s_store_mutex);

  return ret;
}

static BleStoreValueSec *prv_nimble_store_upsert_sec(const int obj_type,
                                                  const struct ble_store_value_sec *value_sec) {
  BleStoreValueSec *s;
  struct ble_store_key_sec key_sec;
  ble_store_key_from_value_sec(&key_sec, value_sec);
  ListNode **sec_list = prv_find_sec_list_for_obj_type(obj_type);

  mutex_lock_recursive(s_store_mutex);

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

  mutex_unlock_recursive(s_store_mutex);

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

typedef enum {
  NimbleStoreOpCCCDStore,
  NimbleStoreOpCCCDDelete,
  NimbleStoreOpSecDelete,
} NimbleStoreOp;

//! One deferred settings-file operation.
typedef struct {
  NimbleStoreOp op;
  union {
    BleCCCD cccd;
    struct {
      int obj_type;
      //! The whole entry we were told to drop, not just its address: by the time
      //! KernelBG runs, the address may name a different bonding.
      struct ble_store_value_sec value_sec;
    } sec;
  };
} NimbleStoreWork;

//! @return True if NimBLE still holds keys of `obj_type` for `peer_addr`.
static bool prv_sec_is_present(int obj_type, const ble_addr_t *peer_addr) {
  const struct ble_store_key_sec key_sec = {
    .peer_addr = *peer_addr,
    .idx = 0,
  };

  mutex_lock_recursive(s_store_mutex);
  const bool present = (prv_nimble_store_find_sec(obj_type, &key_sec) != NULL);
  mutex_unlock_recursive(s_store_mutex);

  return present;
}

//! @return True if `peer` still has a bonding, so a CCCD of its own is worth
//! keeping. Runs on KernelBG, off ble_hs_mutex, and takes the store lock and the
//! db lock one after the other rather than nested, which is the order the rest
//! of the glue uses.
static bool prv_peer_is_bonded(const BTDeviceInternal *peer) {
  ble_addr_t peer_addr;
  pebble_device_to_nimble_addr(peer, &peer_addr);

  // NimBLE's own view first. It is written synchronously on the host task before
  // the work that reads it here was queued, so it cannot lag behind this item,
  // and the common case never reaches the settings file at all.
  if (prv_sec_is_present(BLE_STORE_OBJ_TYPE_PEER_SEC, &peer_addr) ||
      prv_sec_is_present(BLE_STORE_OBJ_TYPE_OUR_SEC, &peer_addr)) {
    return true;
  }

  // The bonding reaches flash from KernelMain, on a queue this one is not
  // ordered against, so an empty list is not proof on its own.
  return bt_persistent_storage_get_ble_pairing_by_addr(peer, NULL, NULL);
}

static void prv_handle_sec_delete(const NimbleStoreWork *work) {
  const struct ble_store_value_sec *value_sec = &work->sec.value_sec;
  BleBonding expected;

  // Cheap first cut. The repeat-pairing path drops the old keys and immediately
  // pairs again, and the new keys land in the in-memory list synchronously on
  // the host task while this is still queued.
  if (prv_sec_is_present(work->sec.obj_type, &value_sec->peer_addr)) {
    PBL_LOG_DBG("SEC delete: obj=%d superseded by a newer bonding, skipping", work->sec.obj_type);
    return;
  }

  // The real guard is the key material, not the check above and not the address.
  // bt_persistent_storage_delete_ble_pairing_by_addr() runs a full-file scan, the
  // record delete, a CCCD sweep and a shared-PRF erase, and can be preempted
  // throughout; a re-pair that completes in that window writes its fresh keys
  // over the very record it is walking, because the identity and the IRK are
  // unchanged. Hand the storage the keys we were told to drop so it can tell the
  // two apart.
  //
  // These are the same conversions the write path used to build what was stored,
  // so the ltk / ediv / rand compared on the other side are the ones that were
  // written.
  memset(&expected, 0, sizeof(expected));
  switch (work->sec.obj_type) {
    case BLE_STORE_OBJ_TYPE_PEER_SEC:
      prv_convert_peer_sec_to_bonding(value_sec, &expected);
      break;
    case BLE_STORE_OBJ_TYPE_OUR_SEC:
      prv_convert_our_sec_to_bonding(value_sec, &expected);
      break;
    default:
      return;
  }
  nimble_addr_to_pebble_device(&value_sec->peer_addr, &expected.pairing_info.identity);

  PBL_LOG_INFO("SEC delete: obj=%d addr=" BT_DEVICE_ADDRESS_FMT, work->sec.obj_type,
               BT_DEVICE_ADDRESS_XPLODE(expected.pairing_info.identity.address));
  if (!bt_persistent_storage_delete_ble_pairing_by_addr_if_matches(
          &expected.pairing_info.identity, &expected.pairing_info)) {
    PBL_LOG_DBG("SEC delete: no stored bonding holds these keys, nothing to do");
  }
}

static void prv_handle_store_work_cb(void *data) {
  NimbleStoreWork *work = data;

  switch (work->op) {
    case NimbleStoreOpCCCDStore:
      // A "Forget device" runs on KernelMain and is not ordered against this
      // queue, so the bonding can be gone by the time we get here. Writing then
      // takes a fresh id for a record nothing sweeps any more, and
      // bt_persistent_storage_register_existing_ble_bondings() restores it into
      // NimBLE's list on every boot, so it accumulates with each pair/unpair.
      if (!prv_peer_is_bonded(&work->cccd.peer)) {
        PBL_LOG_WRN("CCCD store for handle %u skipped: peer is no longer bonded",
                    work->cccd.chr_val_handle);
      } else if (bt_persistent_storage_store_cccd(&work->cccd) == BT_CCCD_ID_INVALID) {
        // The subscription is live either way; only its persistence is lost, and
        // the peer rewrites the CCCD on its next connection.
        PBL_LOG_ERR("CCCD store for handle %u failed", work->cccd.chr_val_handle);
      }
      break;
    case NimbleStoreOpCCCDDelete:
      if (!bt_persistent_storage_delete_cccd(&work->cccd.peer, work->cccd.chr_val_handle)) {
        PBL_LOG_WRN("CCCD delete for handle %u matched no stored record",
                    work->cccd.chr_val_handle);
      }
      break;
    case NimbleStoreOpSecDelete:
      prv_handle_sec_delete(work);
      break;
  }

  kernel_free(work);
}

//! @return True if `work` removes state rather than adding it, in which case
//! dropping it leaves the settings file wrong for good.
static bool prv_work_is_delete(const NimbleStoreWork *work) {
  return (work->op != NimbleStoreOpCCCDStore);
}

//! A dropped store costs one subscription until the peer rewrites it. A dropped
//! delete is permanent: a stale CCCD with flags != 0 survives in the file, and
//! bt_persistent_storage_register_existing_ble_bondings() restores it on the
//! next boot as a live subscription, so the FW reports the HID service ready and
//! sends reports the phone never asked for and ignores. Only a re-pair clears
//! that.
static void prv_log_dropped_work(const NimbleStoreWork *work, const char *reason) {
  switch (work->op) {
    case NimbleStoreOpCCCDStore:
      PBL_LOG_WRN("%s, dropping the CCCD store for handle %u", reason, work->cccd.chr_val_handle);
      break;
    case NimbleStoreOpCCCDDelete:
      PBL_LOG_ERR("%s, dropping the CCCD delete for handle %u; nothing will retry it", reason,
                  work->cccd.chr_val_handle);
      break;
    case NimbleStoreOpSecDelete:
      PBL_LOG_ERR("%s, dropping the SEC delete for obj=%d; nothing will retry it", reason,
                  work->sec.obj_type);
      break;
  }
}

//! Queues the settings-file half of a store update onto KernelBG.
//! Each transaction is a full open/scan/write/close, and NimBLE runs the store
//! callbacks with ble_hs_mutex held, so doing that inline stalls every task that
//! needs the host lock for as long as the flash takes.
//! @note One FIFO fed from one task, so the operations reach the settings file
//! in the order NimBLE issued them.
static void prv_queue_work(const NimbleStoreWork *work) {
  if (!prv_work_is_delete(work) &&
      (system_task_get_available_space() <= KERNEL_BG_QUEUE_MARGIN_WRITE)) {
    prv_log_dropped_work(work, "KernelBG backed up");
    return;
  }

  NimbleStoreWork *copy = kernel_malloc(sizeof(*copy));
  if (copy == NULL) {
    prv_log_dropped_work(work, "Out of memory");
    return;
  }
  *copy = *work;

  // Non-blocking on purpose: we hold ble_hs_mutex. The margin above cannot
  // guarantee the send finds room -- KernelMain, KernelHRM and BT ISRs all feed
  // the same queue, and there is a kernel_malloc() between the check and the
  // send -- so the send itself has to be the one that refuses to wait.
  if (!system_task_add_callback_nonblocking(prv_handle_store_work_cb, copy)) {
    prv_log_dropped_work(work, "KernelBG queue full or callbacks blocked");
    kernel_free(copy);
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

  // Not queued onto KernelBG with the rest of the store work, on purpose. The
  // settings file is never touched from here: bt_driver_cb_handle_create_bonding()
  // does all of it on KernelMain, so nothing heavy runs under ble_hs_mutex
  // either way, and moving it would only trade one queue for another. What it
  // would add is a drop policy, and the queued item is the only copy of a
  // freshly negotiated LTK -- losing it leaves NimBLE thinking the peer is
  // bonded while the FW has no keys on file. The cost of keeping it here is
  // that event_put() waits up to 3 s on a full KernelMain queue and then
  // reboots, with ble_hs_mutex held.
  //
  // kernel_malloc_check(), i.e. a panic, and not BLE_HS_ENOMEM, even though the
  // rest of this file now degrades gracefully: ble_sm_persist_keys() discards
  // what the store write returns, so an error here does not abort the bonding.
  // It would leave the link encrypted and the peer bonded with the FW holding no
  // keys at all -- silently, and only until the next reconnect. A failed 70-byte
  // kernel allocation is a system-wide failure, not a BLE one; the panic at
  // least records a reboot reason on a watch that has no console.
  NimbleStoreSecWrittenContext *ctx = kernel_malloc_check(sizeof(*ctx));
  *ctx = (NimbleStoreSecWrittenContext) {
    .obj_type = obj_type,
    .value_sec = *value_sec,
  };
  launcher_task_add_callback(prv_handle_sec_written_cb, ctx);

  return 0;
}

static int prv_other_sec_obj_type(int obj_type) {
  return (obj_type == BLE_STORE_OBJ_TYPE_OUR_SEC) ? BLE_STORE_OBJ_TYPE_PEER_SEC
                                                  : BLE_STORE_OBJ_TYPE_OUR_SEC;
}

static int prv_nimble_store_delete_sec(int obj_type, const struct ble_store_key_sec *key_sec) {
  BleStoreValueSec *s;
  ListNode **sec_list = prv_find_sec_list_for_obj_type(obj_type);

  mutex_lock_recursive(s_store_mutex);
  s = prv_nimble_store_find_sec(obj_type, key_sec);
  if (s == NULL) {
    mutex_unlock_recursive(s_store_mutex);
    return BLE_HS_ENOENT;
  }

  // Remove from in-memory list before calling into persistent storage,
  // so that NimBLE's ble_store_util_delete_all() loop terminates correctly.
  // Previously we relied on bt_driver_handle_host_removed_bonding() to remove
  // the entry as a side-effect, but that reads the identity from SPRF which
  // may already be erased by a prior iteration, causing an infinite loop.
  // The key may name the peer by index rather than by address, so take the
  // whole entry we found rather than anything off the key.
  const struct ble_store_value_sec value_sec = s->value_sec;
  list_remove((ListNode *)s, sec_list, NULL);
  const bool other_half_remains = (prv_nimble_store_find_sec(
      prv_other_sec_obj_type(obj_type),
      &(struct ble_store_key_sec){ .peer_addr = value_sec.peer_addr, .idx = 0 }) != NULL);
  mutex_unlock_recursive(s_store_mutex);

  kernel_free(s);

  if (other_half_remains) {
    // ble_store_util_delete_peer() drops OUR_SEC and then PEER_SEC, but PebbleOS
    // keeps both halves in one bonding record, so queueing on each would run the
    // same full-file delete twice under the db lock and get nothing for it. Let
    // the half that leaves NimBLE with no keys at all for this peer carry it.
    return 0;
  }

  const NimbleStoreWork work = {
    .op = NimbleStoreOpSecDelete,
    .sec = {
      .obj_type = obj_type,
      .value_sec = value_sec,
    },
  };
  prv_queue_work(&work);

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
  int ret = 0;

  mutex_lock_recursive(s_store_mutex);

  s = prv_nimble_store_find_cccd(key_cccd);
  if (s == NULL) {
    ret = BLE_HS_ENOENT;
    goto unlock;
  }

  *value_cccd = s->value_cccd;

unlock:
  mutex_unlock_recursive(s_store_mutex);

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
  NimbleStoreWork work = {
    .op = NimbleStoreOpCCCDStore,
  };

  nimble_addr_to_pebble_device(&value_cccd->peer_addr, &work.cccd.peer);
  work.cccd.chr_val_handle = value_cccd->chr_val_handle;
  work.cccd.flags = value_cccd->flags;
  work.cccd.value_changed = value_cccd->value_changed;

  // The in-memory list is all NimBLE needs before this returns.
  mutex_lock_recursive(s_store_mutex);
  prv_nimble_store_insert_cccd(value_cccd);
  mutex_unlock_recursive(s_store_mutex);

  prv_queue_work(&work);

  // Always success: whether the settings file had room is only known once
  // KernelBG has run, and BLE_HS_ESTORE_CAP would send NimBLE round the
  // overflow path to free space that we cannot report on anyway.
  return 0;
}

static int prv_nimble_store_delete_cccd(const struct ble_store_key_cccd *key_cccd) {
  NimbleStoreWork work = {
    .op = NimbleStoreOpCCCDDelete,
  };
  BleStoreValueCCCD *s;

  mutex_lock_recursive(s_store_mutex);

  s = prv_nimble_store_find_cccd(key_cccd);
  if (s == NULL) {
    mutex_unlock_recursive(s_store_mutex);
    return BLE_HS_ENOENT;
  }

  // The key may leave chr_val_handle at 0 to mean "any", as
  // ble_store_util_delete_peer() does, but the settings file only ever matches
  // an exact handle. Resolve the wildcard against the entry we just found.
  nimble_addr_to_pebble_device(&s->value_cccd.peer_addr, &work.cccd.peer);
  work.cccd.chr_val_handle = s->value_cccd.chr_val_handle;
  work.cccd.flags = s->value_cccd.flags;
  work.cccd.value_changed = s->value_cccd.value_changed;

  list_remove((ListNode *)s, (ListNode **)&s_cccds, NULL);
  kernel_free(s);

  mutex_unlock_recursive(s_store_mutex);

  prv_queue_work(&work);

  return 0;
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

//! Which BLE_STORE_GEN_KEY_* NimBLE can ask for is a syscfg property, and this
//! callback may only touch the settings file for the ones NimBLE requests
//! *without* ble_hs_lock() held. Only BLE_STORE_GEN_KEY_IRK qualifies -- it
//! comes from ble_hs_startup_go(). LTK and CSRK come from
//! ble_sm_key_exch_exec(), which runs under ble_hs_lock(), and reading the
//! settings file there is the deadlock this whole file was rewritten to remove.
//! They are unreachable only because of the settings pinned below:
//! ble_sm_key_dist() clears BLE_SM_PAIR_KEY_DIST_ENC for Secure Connections, so
//! with SC-only pairing no LTK is ever distributed, and with ENC-only key
//! distribution BLE_SM_PAIR_KEY_DIST_SIGN is never set, so no CSRK is either.
_Static_assert(MYNEWT_VAL(BLE_SM_SC_ONLY) == 1,
               "gen_key: SC-only pairing is what keeps BLE_STORE_GEN_KEY_LTK unreachable");
_Static_assert(MYNEWT_VAL(BLE_SM_LEGACY) == 0,
               "gen_key: legacy pairing would distribute an LTK under ble_hs_lock()");
_Static_assert((MYNEWT_VAL(BLE_SM_OUR_KEY_DIST) & BLE_SM_PAIR_KEY_DIST_SIGN) == 0,
               "gen_key: distributing a CSRK would ask for one under ble_hs_lock()");

static int prv_nimble_store_gen_key(uint8_t key, struct ble_store_gen_key *gen_key,
                                    uint16_t conn_handle) {
  SM128BitKey stored_keys[SMRootKeyTypeNum];

  // The settings-file read stays inside the switch, so a syscfg change that
  // re-enables the LTK or CSRK paths costs a rejected pairing rather than the
  // reboot it used to.
  if (key != BLE_STORE_GEN_KEY_IRK) {
    return BLE_HS_ENOTSUP;
  }

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

  memcpy(gen_key->irk, stored_keys[SMRootKeyTypeIdentity].data, KEY_SIZE);

  return 0;
}

void nimble_store_init(void) {
  if (s_store_mutex == NULL) {
    s_store_mutex = mutex_create_recursive();
  }

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
  mutex_lock_recursive(s_store_mutex);

  list_foreach((ListNode *)s_peer_value_secs, prv_store_value_free, NULL);
  list_foreach((ListNode *)s_our_value_secs, prv_store_value_free, NULL);
  list_foreach((ListNode *)s_cccds, prv_store_value_free, NULL);

  s_peer_value_secs = NULL;
  s_our_value_secs = NULL;
  s_cccds = NULL;

  mutex_unlock_recursive(s_store_mutex);
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

//! Drops the peer's keys, but only the ones the caller was handed.
//! @note Content-matched, not address-matched. The host half of a delete releases
//! its lock before calling here, and a re-pair of the same phone keeps its
//! identity address and its IRK, so by the time this runs the lists can already
//! hold the fresh keys of a bonding that flash and shared PRF consider current.
//! Removing those by address leaves the phone bonded and the watch without keys
//! until the next boot rebuilds the lists.
//! @note Both halves are judged as one, the way the settings file holds them:
//! only the halves `bonding` marks valid are compared, and a `bonding` with no
//! valid encryption info at all matches nothing -- the same rule
//! bt_persistent_storage_delete_ble_pairing_by_addr_if_matches() applies, since
//! falling back to the address is exactly what this exists to avoid.
void bt_driver_handle_host_removed_bonding(const BleBonding *bonding) {
  struct ble_store_key_sec key_sec;

  PBL_LOG_INFO("Host removed bonding: addr=" BT_DEVICE_ADDRESS_FMT " random=%u",
               BT_DEVICE_ADDRESS_XPLODE(bonding->pairing_info.identity.address),
               bonding->pairing_info.identity.is_random_address);

  key_sec.idx = 0;
  pebble_device_to_nimble_addr(&bonding->pairing_info.identity, &key_sec.peer_addr);

  mutex_lock_recursive(s_store_mutex);

  BleStoreValueSec *our_sec = prv_nimble_store_find_sec(BLE_STORE_OBJ_TYPE_OUR_SEC, &key_sec);
  BleStoreValueSec *peer_sec = prv_nimble_store_find_sec(BLE_STORE_OBJ_TYPE_PEER_SEC, &key_sec);

  if ((our_sec == NULL) && (peer_sec == NULL)) {
    // The usual outcome of a deferred delete: NimBLE dropped both halves itself
    // before the work was queued.
    mutex_unlock_recursive(s_store_mutex);
    return;
  }

  // These are the conversions the write path used to build what the host stored,
  // so the ltk / ediv / rand compared below are the ones it was given.
  BleBonding held;
  memset(&held, 0, sizeof(held));
  if (our_sec != NULL) {
    prv_convert_our_sec_to_bonding(&our_sec->value_sec, &held);
  }
  if (peer_sec != NULL) {
    prv_convert_peer_sec_to_bonding(&peer_sec->value_sec, &held);
  }

  if (!sm_pairing_info_encryption_keys_match(&held.pairing_info, &bonding->pairing_info)) {
    mutex_unlock_recursive(s_store_mutex);
    PBL_LOG_DBG("Host removed bonding: our keys are a newer pairing's, keeping them");
    return;
  }

  if (our_sec != NULL) {
    list_remove((ListNode *)our_sec, (ListNode **)&s_our_value_secs, NULL);
    kernel_free(our_sec);
  }

  if (peer_sec != NULL) {
    list_remove((ListNode *)peer_sec, (ListNode **)&s_peer_value_secs, NULL);
    kernel_free(peer_sec);
  }

  mutex_unlock_recursive(s_store_mutex);
}

void bt_driver_handle_host_added_cccd(const BleCCCD *cccd) {
  struct ble_store_value_cccd value_cccd;

  pebble_device_to_nimble_addr(&cccd->peer, &value_cccd.peer_addr);
  value_cccd.chr_val_handle = cccd->chr_val_handle;
  value_cccd.flags = cccd->flags;
  value_cccd.value_changed = cccd->value_changed;

  mutex_lock_recursive(s_store_mutex);
  prv_nimble_store_insert_cccd(&value_cccd);
  mutex_unlock_recursive(s_store_mutex);
}

void bt_driver_handle_host_removed_cccd(const BleCCCD *cccd) {
  BleStoreValueCCCD *s;
  struct ble_store_key_cccd key_cccd;
  
  pebble_device_to_nimble_addr(&cccd->peer, &key_cccd.peer_addr);
  key_cccd.chr_val_handle = cccd->chr_val_handle;
  key_cccd.idx = 0;

  mutex_lock_recursive(s_store_mutex);

  s = prv_nimble_store_find_cccd(&key_cccd);
  if (s != NULL) {
    list_remove((ListNode *)s, (ListNode **)&s_cccds, NULL);
    kernel_free(s);
  }

  mutex_unlock_recursive(s_store_mutex);
}
