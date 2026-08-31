/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/blob_db/contacts_db.h"

#include "kernel/pbl_malloc.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/settings/settings_file.h"
#include "pbl/services/contacts/contacts.h"
#include "pbl/os/mutex.h"
#include "system/passert.h"
#include "system/status_codes.h"
#include "util/units.h"
#include "pbl/util/uuid.h"

#define SETTINGS_FILE_NAME "contactsdb"
#define SETTINGS_FILE_SIZE (KiBYTES(30))

static struct {
  SettingsFile settings_file;
  PebbleMutex *mutex;
} s_contacts_db;

//////////////////////
// Settings helpers
//////////////////////

static status_t prv_lock_mutex_and_open_file(void) {
  mutex_lock(s_contacts_db.mutex);
  status_t rv = settings_file_open_growable(&s_contacts_db.settings_file,
                                            SETTINGS_FILE_NAME,
                                            SETTINGS_FILE_SIZE,
                                            KiBYTES(4));
  if (rv != S_SUCCESS) {
    mutex_unlock(s_contacts_db.mutex);
  }
  return rv;
}

static void prv_close_file_and_unlock_mutex(void) {
  settings_file_close(&s_contacts_db.settings_file);
  mutex_unlock(s_contacts_db.mutex);
}

//////////////////////////////
// Contacts DB API
//////////////////////////////

int contacts_db_get_serialized_contact(const Uuid *uuid, SerializedContact **contact_out) {
  *contact_out = NULL;

  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return 0;
  }

  const unsigned contact_len = settings_file_get_len(&s_contacts_db.settings_file,
                                                     (uint8_t *)uuid, UUID_SIZE);
  if (contact_len < sizeof(SerializedContact)) {
    prv_close_file_and_unlock_mutex();
    return 0;
  }

  *contact_out = task_zalloc(contact_len);
  if (!*contact_out) {
    prv_close_file_and_unlock_mutex();
    return 0;
  }

  rv = settings_file_get(&s_contacts_db.settings_file, (uint8_t *)uuid, UUID_SIZE,
                         (void *) *contact_out, contact_len);
  prv_close_file_and_unlock_mutex();
  if (rv != S_SUCCESS) {
    task_free(*contact_out);
    return 0;
  }

  return (contact_len - sizeof(SerializedContact));
}

void contacts_db_free_serialized_contact(SerializedContact *contact) {
  task_free(contact);
}

/////////////////////////
// Blob DB API
/////////////////////////

void contacts_db_init(void) {
  memset(&s_contacts_db, 0, sizeof(s_contacts_db));
  s_contacts_db.mutex = mutex_create();
}

status_t contacts_db_insert(const uint8_t *key, int key_len, const uint8_t *val, int val_len) {
  if (key_len != UUID_SIZE || val_len < (int) sizeof(SerializedContact)) {
    return E_INVALID_ARGUMENT;
  }

  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  rv = settings_file_set(&s_contacts_db.settings_file, key, key_len, val, val_len);

  prv_close_file_and_unlock_mutex();
  return rv;
}

int contacts_db_get_len(const uint8_t *key, int key_len) {
  if (key_len != UUID_SIZE) {
    return 0;
  }

  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  rv = settings_file_get_len(&s_contacts_db.settings_file, key, key_len);

  prv_close_file_and_unlock_mutex();
  return rv;
}

status_t contacts_db_read(const uint8_t *key, int key_len, uint8_t *val_out, int val_out_len) {
  if (key_len != UUID_SIZE || val_out == NULL) {
    return E_INVALID_ARGUMENT;
  }

  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  rv = settings_file_get(&s_contacts_db.settings_file, key, key_len, val_out, val_out_len);
  prv_close_file_and_unlock_mutex();

  return rv;
}

status_t contacts_db_delete(const uint8_t *key, int key_len) {
  if (key_len != UUID_SIZE) {
    return E_INVALID_ARGUMENT;
  }

  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  rv = settings_file_delete(&s_contacts_db.settings_file, key, key_len);

  prv_close_file_and_unlock_mutex();
  return rv;
}

status_t contacts_db_flush(void) {
  mutex_lock(s_contacts_db.mutex);
  status_t rv = pfs_remove(SETTINGS_FILE_NAME);
  mutex_unlock(s_contacts_db.mutex);
  return rv;
}

status_t contacts_db_compact(void) {
  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }
  rv = settings_file_compact(&s_contacts_db.settings_file);
  prv_close_file_and_unlock_mutex();
  return rv;
}
