/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "process_management/process_manager.h"
#include "process_management/app_install_manager.h"
#include "process_management/pebble_process_info.h"
#include "pbl/services/blob_db/app_db.h"
#include "pbl/util/size.h"

// Stubs
#include "stubs_accel_service.h"
#include "stubs_analytics.h"
#include "stubs_animation_service.h"
#include "stubs_app_cache.h"
#include "stubs_app_manager.h"
#include "stubs_app_state.h"
#include "stubs_dls.h"
#include "stubs_evented_timer.h"
#include "stubs_expandable_dialog.h"
#include "stubs_freertos.h"
#include "stubs_heap.h"
#include "stubs_i18n.h"
#include "stubs_logging.h"
#include "stubs_modal_manager.h"
#include "stubs_new_timer.h"
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"
#include "stubs_pebble_process_md.h"
#include "stubs_pebble_tasks.h"
#include "stubs_persist.h"
#include "stubs_queue.h"
#include "stubs_resources.h"
#include "stubs_syscalls.h"
#include "stubs_thread.h"
#include "stubs_tick.h"
#include "stubs_watchface.h"
#include "stubs_worker_manager.h"
#include "stubs_worker_state.h"

char __APP_RAM__[1024*128];
char *__APP_RAM_end__ = &__APP_RAM__[1024*128];
char __WORKER_RAM__[1024*12];
char *__WORKER_RAM_end__ = &__APP_RAM__[1024*12];

typedef struct {
  AppInstallEntry entry;
  bool should_pass;
} AppInstallEntryTestCase;

static AppInstallEntryTestCase s_test_cases[] = {
  {
    .entry = (AppInstallEntry) {
      .install_id = 1,
      .sdk_version = (Version) {
        .major = PROCESS_INFO_CURRENT_SDK_VERSION_MAJOR,
        .minor = PROCESS_INFO_CURRENT_SDK_VERSION_MINOR
      }
    },
    .should_pass = true
  }, {
    .entry = (AppInstallEntry) {
      .install_id = 2,
      .sdk_version = (Version) {
        .major = PROCESS_INFO_CURRENT_SDK_VERSION_MAJOR - 1,
        .minor = PROCESS_INFO_CURRENT_SDK_VERSION_MINOR
      }
    },
    .should_pass = false
  }, {
    .entry = (AppInstallEntry) {
      .install_id = 3,
      .sdk_version = (Version) {
        .major = PROCESS_INFO_CURRENT_SDK_VERSION_MAJOR + 1,
        .minor = PROCESS_INFO_CURRENT_SDK_VERSION_MINOR
      }
    },
    .should_pass = false
  }, {
    .entry = (AppInstallEntry) {
      .install_id = 4,
      .sdk_version = (Version) {
        .major = PROCESS_INFO_CURRENT_SDK_VERSION_MAJOR,
        .minor = PROCESS_INFO_CURRENT_SDK_VERSION_MINOR - 10
      }
    },
    .should_pass = true
  }, {
    .entry = (AppInstallEntry) {
      .install_id = 5,
      .sdk_version = (Version) {
        .major = PROCESS_INFO_CURRENT_SDK_VERSION_MAJOR,
        .minor = PROCESS_INFO_CURRENT_SDK_VERSION_MINOR + 10
      }
    },
    .should_pass = false
  }, {
    .entry = (AppInstallEntry) {
      .install_id = 6,
      .sdk_version = (Version) {
        .major = PROCESS_INFO_CURRENT_SDK_VERSION_MAJOR + 1,
        .minor = PROCESS_INFO_CURRENT_SDK_VERSION_MINOR + 10
      }
    },
    .should_pass = false
  }, {
    .entry = (AppInstallEntry) {
      .install_id = 7,
      .sdk_version = (Version) {
        .major = PROCESS_INFO_CURRENT_SDK_VERSION_MAJOR - 1,
        .minor = PROCESS_INFO_CURRENT_SDK_VERSION_MINOR - 10
      }
    },
    .should_pass = false
  }
};

PlatformType process_metadata_get_app_sdk_platform(const PebbleProcessMd *md) {
  cl_fail("should not be called");
  return (PlatformType)-1;
}

uint32_t pbl_msgq_num_used(const struct pbl_msgq *q) {
  return 0;
}

int pbl_msgq_get(struct pbl_msgq *q, void *msg, pbl_timeout_t timeout) {
  return 0;
}

int pbl_msgq_put(struct pbl_msgq *q, const void *msg, pbl_timeout_t timeout) {
  return 0;
}

void event_queue_cleanup_and_reset(struct pbl_msgq *queue) {
}

void event_service_clear_process_subscriptions(void) {
}

bool app_install_entry_is_watchface(const AppInstallEntry *entry) {
  return false;
}

AppInstallId app_install_get_id_for_uuid(const Uuid *uuid) {
  return 1;
}

bool app_install_get_entry_for_install_id(AppInstallId install_id, AppInstallEntry *entry) {
  *entry = s_test_cases[install_id - 1].entry;
  return true;
}

bool app_install_id_from_app_db(AppInstallId id) {
  return (id > INSTALL_ID_INVALID);
}

bool app_install_entry_is_SDK_compatible(const AppInstallEntry *entry) {
  return (entry->sdk_version.major == PROCESS_INFO_CURRENT_SDK_VERSION_MAJOR &&
          entry->sdk_version.minor <= PROCESS_INFO_CURRENT_SDK_VERSION_MINOR);
}

static PebbleProcessMd *s_app_install_get_md__result;
const PebbleProcessMd *app_install_get_md(AppInstallId id, bool worker) {
  return s_app_install_get_md__result;
}

void app_install_release_md(const PebbleProcessMd *md) {
}

static status_t s_app_db_get_app_entry_for_install_id__result;
static AppDBEntry s_app_db_get_app_entry_for_install_id__entry;
status_t app_db_get_app_entry_for_install_id(AppInstallId app_id, AppDBEntry *entry) {
  *entry = s_app_db_get_app_entry_for_install_id__entry;
  return s_app_db_get_app_entry_for_install_id__result;
}

static int s_process_metadata_get_res_bank_num__result;
int process_metadata_get_res_bank_num(const PebbleProcessMd *md) {
  return s_process_metadata_get_res_bank_num__result;
}

static PebbleEvent* s_event_put__event;
void event_put(PebbleEvent* event) {
  s_event_put__event = event;
  cl_assert(event != NULL);
}

void event_put_from_app(PebbleEvent* event) { cl_fail("unexpected"); }
void event_put_from_process(PebbleTask task, PebbleEvent* event) { cl_fail("unexpected"); }
void event_reset_from_process_queue(PebbleTask task) { cl_fail("unexpected"); }


void test_process_manager__initialize(void) {
  s_app_install_get_md__result = NULL;
  s_app_db_get_app_entry_for_install_id__result = E_DOES_NOT_EXIST;
  s_app_db_get_app_entry_for_install_id__entry = (AppDBEntry){};
  s_process_metadata_get_res_bank_num__result = 123;
  s_app_manager_launch_new_app__callcount = 0;
  s_app_manager_launch_new_app__config = (__typeof__(s_app_manager_launch_new_app__config)){};
  s_event_put__event = NULL;
}

void test_process_manager__check_SDK_compatible(void) {
  for (uint32_t i = 0; i < ARRAY_LENGTH(s_test_cases); i++) {
    // skipping 0, since it's INSTALL_ID_INVALID
    cl_assert_equal_b(process_manager_check_SDK_compatible(i + 1), s_test_cases[i].should_pass);
  }
}

static const Uuid s_uuid_a = {0x9c, 0x40, 0xa7, 0x79, 0x00, 0x11, 0x22, 0x33,
                              0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb};
static const Uuid s_uuid_b = {0x49, 0x82, 0x77, 0x22, 0x00, 0x11, 0x22, 0x33,
                              0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb};

//! The app cache is keyed on the install ID alone and IDs get recycled, so a cache entry orphaned
//! by a previous install leaves a stale binary at that ID. Launching must notice and refetch
//! rather than silently running the wrong app.
void test_process_manager__stale_cache_entry_is_refetched(void) {
  static PebbleProcessMdFlash s_stale_md = { .common = { .uuid = {0} } };
  s_stale_md.common.uuid = s_uuid_b;
  s_app_install_get_md__result = (PebbleProcessMd *)&s_stale_md;

  // app_db says install ID 1 should be uuid A, but the cached binary is uuid B
  s_app_db_get_app_entry_for_install_id__entry.uuid = s_uuid_a;
  s_app_db_get_app_entry_for_install_id__result = S_SUCCESS;

  process_manager_launch_process(&(ProcessLaunchConfig) { .id = 1 });

  cl_assert(s_event_put__event != NULL);
  cl_assert_equal_i(s_event_put__event->type, PEBBLE_APP_FETCH_REQUEST_EVENT);
  cl_assert_equal_i(s_event_put__event->app_fetch_request.id, 1);
  cl_assert_equal_i(s_app_manager_launch_new_app__callcount, 0);
}

void test_process_manager__matching_cache_entry_launches(void) {
  static PebbleProcessMdFlash s_good_md = { .common = { .uuid = {0} } };
  s_good_md.common.uuid = s_uuid_a;
  s_app_install_get_md__result = (PebbleProcessMd *)&s_good_md;

  s_app_db_get_app_entry_for_install_id__entry.uuid = s_uuid_a;
  s_app_db_get_app_entry_for_install_id__result = S_SUCCESS;

  process_manager_launch_process(&(ProcessLaunchConfig) { .id = 1 });

  cl_assert(s_event_put__event == NULL);
  cl_assert_equal_i(s_app_manager_launch_new_app__callcount, 1);
}

