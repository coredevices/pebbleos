/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "process_management/app_manager.h"

#include "applib/app_comm.h"
#include "applib/graphics/framebuffer.h"
#include "applib/ui/window_stack.h"
#include "applib/ui/window_stack_private.h"
#include <pbl/drivers/mpu.h>
#include "kernel/util/segment.h"
#include "popups/crashed_ui.h"
#include "process_management/app_install_manager.h"
#include "process_management/app_manager.h"
#include "process_management/app_run_state.h"
#include "process_management/process_manager.h"
#include "pbl/services/vibe_pattern.h"
#include "resource/resource_ids.auto.h"
#include "pbl/services/blob_db/app_db.h"
#include "pbl/util/heap.h"

// Fakes
#include "fake_new_timer.h"
#include "fake_pbl_malloc.h"
#include "fake_rtc.h"

// Stubs
#include "stubs_accel_service.h"
#include "stubs_analytics.h"
#include "stubs_animation_service.h"
#include "stubs_app_state.h"
#include "stubs_applib_resource.h"
#include "stubs_cache.h"
#include "stubs_compositor.h"
#include "stubs_dialog.h"
#include "stubs_events.h"
#include "stubs_expandable_dialog.h"
#include "stubs_gettext.h"
#include "stubs_i18n.h"
#include "stubs_logging.h"
#include "stubs_modal_manager.h"
#include "stubs_mpu.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_persist.h"
#include "stubs_print.h"
#include "stubs_prompt.h"
#include "stubs_rand_ptr.h"
#include "stubs_resources.h"
#include "stubs_serial.h"
#include "stubs_simple_dialog.h"
#include "stubs_syscall_internal.h"
#include "stubs_task.h"
#include "stubs_tick.h"
#include "stubs_timeline_peek.h"
#include "stubs_worker_manager.h"

// Fake "Apps"
///////////////////////////////////////////////////////////
static PebbleProcessMdSystem s_launch_app = {
  .name = "Launch App",
  .common = {
    // UUID: 7bbff9bc-b762-4219-9003-4086675d625d
    .uuid = {0x7b, 0xbf, 0xf9, 0xbc, 0xb7, 0x62, 0x42, 0x19, 0x90, 0x03, 0x40, 0x86, 0x67, 0x5d, 0x62, 0x5d},
  },
};

static PebbleProcessMdSystem s_root_app = {
  .name = "Root App",
  .common = {
    // UUID: 3fca66e2-8c66-46c6-8011-330fccc9baa9
    .uuid = {0x3f, 0xca, 0x66, 0xe2, 0x8c, 0x66, 0x46, 0xc6, 0x80, 0x11, 0x33, 0x0f, 0xcc, 0xc9, 0xba, 0xa9},
  },
};

static PebbleProcessMdSystem s_third_party_app = {
  .name = "Third Party App", 
  .common = {
    .is_unprivileged = true,
    // UUID: 04c52401-4dbe-408b-b73a-0e80ef09af74
    .uuid = {0x04, 0xc5, 0x24, 0x01, 0x4d, 0xbe, 0x40, 0x8b, 0xb7, 0x3a, 0x0e, 0x80, 0xef, 0x09, 0xaf, 0x74},
  },
};

static PebbleProcessMdFlash s_borked_app = {
  .name = "Borked Mc'Rib",
  .common = {
    .is_unprivileged = true,
    .process_storage = ProcessStorageFlash,
    // UUID: 25a9e7ff-de9e-4dda-b745-afdd75aaa53b
    .uuid = {0x25, 0xa9, 0xe7, 0xff, 0xde, 0x9e, 0x4d, 0xda, 0xb7, 0x45, 0xaf, 0xdd, 0x75, 0xaa, 0xa5, 0x3b},
  },
  .sdk_version = {.major = PROCESS_INFO_CURRENT_SDK_VERSION_MAJOR, .minor = PROCESS_INFO_CURRENT_SDK_VERSION_MINOR },
};

static PebbleEvent s_last_to_app_event;

static uint32_t s_app_task_control_reg;

// Fakes
///////////////////////////////////////////////////////////

// Just fake this to something we can use in the fake functions below
#define APP_ID_DEFAULT_WATCHFACE ((AppInstallId)-1337)

PebbleTask pebble_task_get_current(void) {
  return PebbleTask_App;
}

AppInstallId watchface_get_default_install_id(void) {
  return APP_ID_DEFAULT_WATCHFACE;
}

const PebbleProcessMd* launcher_menu_app_get_app_info(void) {
  return (PebbleProcessMd*)&s_root_app;
}

const PebbleProcessMd *app_install_get_md(AppInstallId id, bool worker) {
  if (id == APP_ID_DEFAULT_WATCHFACE) {
    static const PebbleProcessMdSystem s_default_watchface_md = {
      .common.process_type = ProcessTypeWatchface,
    };
    return (const PebbleProcessMd *)&s_default_watchface_md;
  } else {
    return launcher_menu_app_get_app_info();
  }
}

void app_install_release_md(const PebbleProcessMd *md) {
}

// Stubs
///////////////////////////////////////////////////////////

char __APP_RAM__[1024*128];
char __APP_RAM_end__;
char __WORKER_RAM__[1024*12];
char __WORKER_RAM_end__;

MemorySegment prv_get_app_ram_segment(void) {
  return (MemorySegment) { __APP_RAM__, &__APP_RAM__[1024*128] };
}

size_t prv_get_stack_guard_size(void) {
  return 32;
}

void _REENT_INIT_PTR(void) {
}

void app_comm_set_sniff_interval(const SniffInterval interval) {
}

void app_idle_timeout_start(void) {
}

void app_idle_timeout_stop(void) {
}

void app_inbox_service_unregister_all(void) {
}

void app_outbox_service_cleanup_all_pending_messages(void) {
}

AppInstallId app_install_get_id_for_uuid(const Uuid *uuid) {
  return 1;
}

void app_install_register_callback(struct AppInstallCallbackNode *callback_info) {
}

void app_install_notify_app_closed(void) {
}

void app_install_cleanup_registered_app_callbacks(void) {
}

bool app_install_get_entry_for_install_id(AppInstallId id, AppInstallEntry *entry) {
  return true;
}

bool app_install_entry_is_watchface(const AppInstallEntry *entry) {
  return true;
}

bool app_install_entry_is_hidden(const AppInstallEntry *entry) {
  return false;
}

bool app_install_entry_is_SDK_compatible(const AppInstallEntry *entry) {
  return true;
}

bool app_install_id_from_app_db(AppInstallId id) {
  return false;
}

bool app_cache_entry_exists(AppInstallId app_id) {
  return true;
}

status_t app_cache_remove_entry(AppInstallId app_id) {
  return S_SUCCESS;
}

status_t app_db_get_app_entry_for_install_id(AppInstallId app_id, AppDBEntry *entry) {
  return E_DOES_NOT_EXIST;
}

const PebbleProcessMd* app_fetch_ui_get_app_info(void) {
  return NULL;
}

void app_message_close(void) {
}

void ble_app_cleanup(void) {
}

void dls_inactivate_sessions(PebbleTask task) {
}

void event_service_clear_process_subscriptions(void) {
}

void evented_timer_clear_process_timers(PebbleTask task) {
}

void launcher_task_add_callback(void (*callback)(void *data), void *data) {
  callback(data);
}

void app_run_state_send_update(const Uuid* uuid, AppState app_state) {
  return;
}

const PebbleProcessMd* system_app_state_machine_get_default_app(void) {
  return launcher_menu_app_get_app_info();
}

void launcher_cancel_force_quit(void) {
}

void light_reset_user_controlled(void) {
}

void light_set_system_color(void) {
}

void mpu_set_task_configurable_regions(MemoryRegion_t *task_params, const MpuRegion **region_ptrs) {
}

void task_init(void) {}

void pebble_task_register(PebbleTask task, TaskHandle_t task_handle) {
}

void pebble_task_unregister(PebbleTask task) {
}

const char* pebble_task_get_name(PebbleTask task) {
  return "?";
}

void pebble_task_create(PebbleTask pebble_task, TaskParameters_t *task_params,
                        TaskHandle_t *handle) {
}

void * process_loader_load(const PebbleProcessMd *app_md, PebbleTask task,
                         MemorySegment *segment) {
  if (app_md == (PebbleProcessMd *)&s_borked_app) {
    return NULL;
  } else {
    return __APP_RAM__;
  }
}

void quick_launch_handle_analytics(void) {
}

void reboot_set_slot_of_last_launched_app(uint32_t app_slot) {
}

void sys_exit(int status) {
}

status_t app_cache_app_launched(AppInstallId id) {
  return 0;
}

const PebbleProcessMd* system_app_state_machine_system_start(void) {
  return (PebbleProcessMd*) &s_launch_app;
}
const PebbleProcessMd* system_app_state_machine_get_last_registered_app(void) {
  return (PebbleProcessMd*) &s_root_app;
}
void system_app_state_machine_register_app_launch(const PebbleProcessMd* app) {
}

void health_tracking_ui_register_app_launch(AppInstallId app_id) {
}

void sys_vibe_history_stop_collecting(void) {
}

void vibe_pattern_clear_for_owner(VibePatternOwner owner) {
}

void speaker_service_stop_for_task(PebbleTask task) {
}

Heap *worker_state_get_heap(void) {
  return NULL;
}

void pbl_msgq_init(struct pbl_msgq *q, void *buf, size_t msg_size, uint32_t max_msgs) {
}

void pbl_msgq_kobj_init(void *def) {
}

int pbl_msgq_put(struct pbl_msgq *q, const void *msg, pbl_timeout_t timeout) {
  if (q == app_manager_get_task_context()->to_process_event_queue) {
    s_last_to_app_event = *(PebbleEvent*) msg;
  }
  return 0;
}

void event_queue_cleanup_and_reset(struct pbl_msgq *queue) {
}

int pbl_msgq_get(struct pbl_msgq *q, void *msg, pbl_timeout_t timeout) {
  return 0;
}

void pbl_msgq_purge(struct pbl_msgq *q) {
}

uint32_t pbl_msgq_num_used(const struct pbl_msgq *q) {
  return 0;
}

void watchface_set_default_install_id(AppInstallId id) {
}

void compositor_reset_app_framebuffer_ownership(void) {
}

const char *app_install_get_custom_app_name(AppInstallId install_id) {
  return NULL;
}

void status_bar_push_text(const char *text) {
}

const CompositorTransition* shell_get_open_compositor_animation(
    AppInstallId current_app_id, AppInstallId next_app_id, const LaunchConfigCommon *config) {
  return NULL;
}

const CompositorTransition* shell_get_close_compositor_animation(AppInstallId current_app_id,
                                                                 AppInstallId next_app_id) {
  return NULL;
}

void watchface_launch_default(const CompositorTransition *animation) {
}

void process_heap_set_exception_handlers(Heap *heap, const PebbleProcessMd *app_md) {
}

// Tests
///////////////////////////////////////////////////////////

void test_app_manager__initialize(void) {
  process_manager_init();
  app_manager_init();

  s_last_to_app_event = (PebbleEvent) {  };

  s_app_task_control_reg = 0x1; // Just leave everything as unprivileged
}

void test_app_manager__start_first(void) {
  cl_assert(app_manager_get_current_app_md() == NULL);

  app_manager_start_first_app();

  cl_assert(app_manager_get_current_app_md() == (PebbleProcessMd*) &s_launch_app);
  cl_assert(s_last_to_app_event.type == 0);
  app_manager_get_task_context()->safe_to_kill = false;
}

void test_app_manager__start_third_party(void) {
  test_app_manager__start_first();

  app_manager_launch_new_app(&(AppLaunchConfig) {
    .md = &s_third_party_app.common,
  });

  // We've sent the deinit event to the first app, but it's going to continue running.
  cl_assert_equal_i(s_last_to_app_event.type, PEBBLE_PROCESS_DEINIT_EVENT);
  cl_assert(app_manager_get_current_app_md() == (PebbleProcessMd*) &s_launch_app);
  s_last_to_app_event = (PebbleEvent) {0};

  // Now the app sets the safe_to_kill flag to true and sends a kill event back to
  // the launcher to get the app killed again. This calls close_current_app, which ends
  // up launching s_third_party_app because it's in the next app slot.
  app_manager_get_task_context()->safe_to_kill = true;
  app_manager_close_current_app(true /* gracefully */);

  // The second app should now be running.
  cl_assert(app_manager_get_current_app_md() == (PebbleProcessMd*) &s_third_party_app);
  app_manager_get_task_context()->safe_to_kill = false;
}

void test_app_manager__start_third_party_and_crash_back_to_root(void) {
  test_app_manager__start_third_party();

  // Simulate a crash
  app_manager_get_task_context()->safe_to_kill = true;
  app_manager_close_current_app(false /* gracefully */);

  cl_assert(app_manager_get_current_app_md() == (PebbleProcessMd*) &s_root_app);
}

void test_app_manager__start_borked_app(void) {
  test_app_manager__start_first();

  app_manager_launch_new_app(&(AppLaunchConfig) {
    .md = &s_borked_app.common,
  });
  app_manager_get_task_context()->safe_to_kill = true;
  app_manager_close_current_app(true /* gracefully */);

  // The first app should still be running
  cl_assert(app_manager_get_current_app_md() == (PebbleProcessMd*) &s_launch_app);

}

void test_app_manager__start_third_party_and_force_close_back_to_first(void) {
  test_app_manager__start_third_party();

  s_last_to_app_event = (PebbleEvent) {0};

  // Make the app get stuck in a syscall. This will indicate that the app is running
  // privileged.
  stub_control_reg(0x0);

  // Try to close the app.
  app_manager_close_current_app(true /* gracefully */);

  cl_assert(app_manager_get_current_app_md() == (PebbleProcessMd*) &s_third_party_app);
  cl_assert(s_last_to_app_event.type == PEBBLE_PROCESS_DEINIT_EVENT);

  // Simulate the deinit timer timing out instead of the app actually closing.
  app_manager_close_current_app(false /* gracefully */);

  // However it's still not ready to die.
  cl_assert(app_manager_get_current_app_md() == (PebbleProcessMd*) &s_third_party_app);

  // The trap has been set and eventually the syscall trap finds a good place to kill
  // the app.
  stub_control_reg(0x1);
  app_manager_close_current_app(false /* gracefully */);

  // The app should have exited to the root app.
  cl_assert(app_manager_get_current_app_md() == (PebbleProcessMd*) &s_root_app);
}

void test_app_manager__watchface_crash_on_close(void) {
  test_app_manager__start_first();

  // Launch a new app with a panning animation. This will kick off the closing of the
  // current app.
  app_manager_launch_new_app(&(AppLaunchConfig) {
    .md = &s_third_party_app.common,
  });

  // We've sent the deinit event to the first app, but it's going to continue running.
  cl_assert_equal_i(s_last_to_app_event.type, PEBBLE_PROCESS_DEINIT_EVENT);
  cl_assert(app_manager_get_current_app_md() == (PebbleProcessMd*) &s_launch_app);

  // However, the poor app is going to crash on the way out.
  app_manager_get_task_context()->safe_to_kill = true;
  app_manager_close_current_app(false /* gracefully */);

  // Make sure we correctly launch the root app with the right to left animation as opposed
  // to the panning animation we originally requested.
  cl_assert(app_manager_get_current_app_md() == (PebbleProcessMd*) &s_root_app);
}

void test_app_manager__override_next_app_with_watchface_using_action_performed_exit_reason(void) {
  test_app_manager__start_first();

  // Check that the default exit reason is "not specified"
  const AppExitReason default_exit_reason = app_exit_reason_get();
  cl_assert_equal_i(default_exit_reason, APP_EXIT_NOT_SPECIFIED);

  // Check that calling app_exit_reason_set() with an invalid exit reason does not change it
  app_exit_reason_set((AppExitReason)1337);
  cl_assert_equal_i(app_exit_reason_get(), default_exit_reason);
  app_exit_reason_set(NUM_EXIT_REASONS);
  cl_assert_equal_i(app_exit_reason_get(), default_exit_reason);
  app_exit_reason_set((AppExitReason)-1);
  cl_assert_equal_i(app_exit_reason_get(), default_exit_reason);

  // Specify the exit reason to be that an action was performed successfully
  app_exit_reason_set(APP_EXIT_ACTION_PERFORMED_SUCCESSFULLY);

  // Check that closing the current app takes us to the watchface
  app_manager_get_task_context()->safe_to_kill = true;
  app_manager_close_current_app(true /* gracefully */);
  cl_assert_equal_b(app_manager_is_watchface_running(), true);

  // Check that launching a new app resets the exit reason to the default reason
  cl_assert_equal_b(
      app_manager_launch_new_app(&(AppLaunchConfig) {
        .md = &s_third_party_app.common,
      }),
      true);
  cl_assert_equal_i(app_exit_reason_get(), default_exit_reason);
}
