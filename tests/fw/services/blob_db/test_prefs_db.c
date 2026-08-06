/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "pbl/util/uuid.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/blob_db/prefs_db.h"
#include "shell/prefs.h"
#include "shell/prefs_private.h"

// Fixture
////////////////////////////////////////////////////////////////

// Fakes
////////////////////////////////////////////////////////////////
#include "fake_spi_flash.h"
#include "fake_system_task.h"
#include "fake_kernel_services_notifications.h"

// Stubs
////////////////////////////////////////////////////////////////
#include "stubs_analytics.h"
#include "stubs_app_install_manager.h"
#include "stubs_battery_charge_limit.h"
#include "stubs_event_loop.h"
#include "stubs_hexdump.h"
#include "stubs_logging.h"
#include "stubs_mfg_info.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"
#include "stubs_prompt.h"
#include "stubs_rand_ptr.h"
#include "stubs_sleep.h"
#include "stubs_system_theme.h"
#include "stubs_task_watchdog.h"
#include "stubs_timeline_peek.h"
#include "stubs_ambient_light.h"
#include "stubs_activity.h"

void prefs_sync_init(void) {
}

void event_put(PebbleEvent *event) {
}

void i18n_enable(bool enable) {
}

void test_prefs_db__initialize(void) {
  fake_spi_flash_init(0, 0x1000000);
  pfs_init(false);
}

void test_prefs_db__cleanup(void) {
}

void test_prefs_db__get_length(void) {
  Uuid uuid = {0, 1, 2, 3};
  const char *key = "workerId";
  int key_len = strlen(key);
  cl_assert_equal_i(prefs_db_insert((uint8_t *)key, key_len, (uint8_t *)&uuid, sizeof(uuid)), 0);
  cl_assert_equal_i(prefs_db_get_len((uint8_t *)key, key_len), sizeof(uuid));
}

void test_prefs_db__insert_and_read(void) {
  uint32_t set_value = 42;

  // NOTE: We intentionally put one garbage character after the key to catch errors
  // that assume the key is 0 terminated
  const char *key = "lightTimeoutMsX";
  int key_len = strlen(key) - 1;


  // Set initial value
  backlight_set_timeout_ms(set_value + 1);


  // Insert and check the length
  cl_assert_equal_i(prefs_db_insert((uint8_t *)key, key_len, (uint8_t *)&set_value,
                                    sizeof(set_value)), 0);
  cl_assert_equal_i(prefs_db_get_len((uint8_t *)key, key_len), sizeof(set_value));


  // Read it back
  uint32_t get_value;
  cl_assert_equal_i(prefs_db_read((uint8_t *)key, key_len, (uint8_t *)&get_value,
                                  sizeof(get_value)), 0);
  cl_assert_equal_i(set_value, get_value);


  // If we get the pref setting now, it should still be the old value because we haven't
  // issued the blob_db update event yet
  uint32_t get_pref = backlight_get_timeout_ms();
  cl_assert_equal_i(get_pref, set_value + 1);

  // Issue the blob_db update event
  PebbleBlobDBEvent event = (PebbleBlobDBEvent) {
    .db_id = BlobDBIdPrefs,
    .type = BlobDBEventTypeInsert,
    .key = (uint8_t *)key,
    .key_len = key_len,
  };
  prefs_private_handle_blob_db_event(&event);
  get_pref = backlight_get_timeout_ms();
  cl_assert(get_pref == get_value);


  // Set new value using the set call and read it back using prefs_db
  uint32_t new_set_value = 4242;
  backlight_set_timeout_ms(new_set_value);
  cl_assert_equal_i(prefs_db_read((uint8_t *)key, key_len, (uint8_t *)&get_value,
                                  sizeof(get_value)), 0);
  cl_assert_equal_i(new_set_value, get_value);


  // Try and insert an unknown key. It should fail
  const char *bad_key = "bad_key";
  int bad_key_len = strlen(bad_key);
  cl_assert(prefs_db_insert((uint8_t *)bad_key, bad_key_len, (uint8_t *)&set_value,
                                    sizeof(set_value)) < 0);
  cl_assert(prefs_db_get_len((uint8_t *)bad_key, bad_key_len) < 0);
  cl_assert(prefs_db_read((uint8_t *)bad_key, bad_key_len, (uint8_t *)&get_value,
                                  sizeof(get_value)) < 0);


  // Try and insert the wrong size for a known key, it should fail
  cl_assert(prefs_db_insert((uint8_t *)key, key_len, (uint8_t *)&set_value,
                                    sizeof(set_value) + 1) < 0);
  // Read it back
  cl_assert(prefs_db_read((uint8_t *)key, key_len, (uint8_t *)&get_value,
                                  sizeof(get_value) + 1) < 0);
}


