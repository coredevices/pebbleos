/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stdio.h>

#include "applib/app.h"
#include "applib/ui/ui.h"
#include "apps/prf/mfg_accel.h"
#ifdef CONFIG_MAG
#include "apps/prf/mfg_mag.h"
#endif
#include "apps/prf/mfg_als.h"
#include "apps/prf/mfg_backlight.h"
#include "apps/prf/mfg_button.h"
#include "apps/prf/mfg_charge.h"
#include "apps/prf/mfg_display.h"
#include "apps/prf/mfg_hrm_ctr_leakage_obelix.h"
#include "apps/prf/mfg_mic_asterix.h"
#include "apps/prf/mfg_mic_getafix.h"
#include "apps/prf/mfg_mic_obelix.h"
#include "apps/prf/mfg_program_color.h"
#include "apps/prf/mfg_qr_results.h"
#include "apps/prf/mfg_speaker_asterix.h"
#include "apps/prf/mfg_speaker_obelix.h"
#include "apps/prf/mfg_touch.h"
#include "apps/prf/mfg_test_menu.h"
#include "apps/prf/mfg_test_result.h"
#include "apps/prf/mfg_vibration.h"
#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"
#include "process_management/app_manager.h"
#include "process_state/app_state/app_state.h"
#include "pbl/util/size.h"

typedef const PebbleProcessMd *(*MfgTestGetInfoFn)(void);

typedef struct {
  const char *title;
  MfgTestId test_id;
  MfgTestGetInfoFn get_info;
  uint8_t modes;
} MfgTestMenuEntry;

#define SF MFG_TEST_MODE_SEMI_FINISHED
#define FI MFG_TEST_MODE_FINISHED

static const MfgTestMenuEntry s_all_entries[] = {
  { "Buttons",       MfgTestId_Buttons,       mfg_button_app_get_info,    SF | FI },
  { "Display",       MfgTestId_Display,       mfg_display_app_get_info,   SF | FI },
#ifdef CONFIG_TOUCH
  { "Touch",         MfgTestId_Touch,         mfg_touch_app_get_info,     SF | FI },
#endif
  { "Backlight",     MfgTestId_Backlight,     mfg_backlight_app_get_info, SF | FI },
  { "Accelerometer", MfgTestId_Accel,         mfg_accel_app_get_info,     SF | FI },
#ifdef CONFIG_MAG
  { "Magnetometer",  MfgTestId_Mag,           mfg_mag_app_get_info,       SF | FI },
#endif
#ifdef CONFIG_BOARD_ASTERIX
  { "Speaker",       MfgTestId_Speaker,       mfg_speaker_asterix_app_get_info, SF | FI },
#elif defined(CONFIG_BOARD_OBELIX)
  { "Speaker",       MfgTestId_Speaker,       mfg_speaker_obelix_app_get_info,  SF | FI },
#endif
#ifdef CONFIG_BOARD_ASTERIX
  { "Microphone",    MfgTestId_Mic,           mfg_mic_asterix_app_get_info, SF | FI },
#elif defined(CONFIG_BOARD_OBELIX)
  { "Microphone",    MfgTestId_Mic,           mfg_mic_obelix_app_get_info,  SF | FI },
#elif defined(CONFIG_BOARD_GETAFIX)
  { "Microphone",    MfgTestId_Mic,           mfg_mic_getafix_app_get_info, SF | FI },
#endif
  { "ALS",           MfgTestId_ALS,           mfg_als_app_get_info,       SF | FI },
  { "Vibration",     MfgTestId_Vibration,     mfg_vibration_app_get_info, SF | FI },
#if defined(CONFIG_BOARD_OBELIX) && defined(CONFIG_MFG)
  { "HRM CTR/L",     MfgTestId_HrmCtrLeakage, mfg_hrm_ctr_leakage_obelix_app_get_info, SF | FI },
#endif
  { "Charge",        MfgTestId_Charge,        mfg_charge_app_get_info,         SF | FI },
  { "Program Color", MfgTestId_ProgramColor,  mfg_program_color_app_get_info,  FI },
};

#undef SF
#undef FI

// Filtered entries for the active mode (built at startup)
static const MfgTestMenuEntry *s_entries[ARRAY_LENGTH(s_all_entries)];
static size_t s_num_entries;

static uint8_t s_active_mode;

typedef struct {
  Window *window;
  SimpleMenuLayer *menu_layer;
  SimpleMenuSection menu_section;
} MfgTestMenuAppData;

static bool s_relaunch_menu = false;
static int16_t s_last_selected = -1;

//! Callback to run from the kernel main task
static void prv_launch_app_cb(void *data) {
  app_manager_launch_new_app(&(AppLaunchConfig) { .md = data });
}

static void prv_launch_test(int index, const PebbleProcessMd *md) {
  s_relaunch_menu = true;
  s_last_selected = index;
  launcher_task_add_callback(prv_launch_app_cb, (void*) md);
}

static void prv_select_test(int index, void *context) {
  if ((size_t)index < s_num_entries) {
    prv_launch_test(index, s_entries[index]->get_info());
  }
}

static void prv_select_results(int index, void *context) {
  prv_launch_test(index, mfg_qr_results_app_get_info());
}

static const char * prv_get_status_prefix(MfgTestId test) {
  const MfgTestResult *result = mfg_test_result_get(test);
  if (!result || !result->ran) {
    return "[ ]";
  }
  return result->passed ? "[P]" : "[F]";
}

static void prv_build_filtered_entries(uint8_t mode) {
  s_num_entries = 0;
  for (size_t i = 0; i < ARRAY_LENGTH(s_all_entries); i++) {
    if (s_all_entries[i].modes & mode) {
      s_entries[s_num_entries++] = &s_all_entries[i];
    }
  }
}

static void prv_window_load(Window *window) {
  MfgTestMenuAppData *data = app_state_get_user_data();

  Layer *window_layer = window_get_root_layer(data->window);
  GRect bounds = window_layer->bounds;

  size_t num_items = s_num_entries + 1;  // +1 for RESULTS
  SimpleMenuItem *items = app_malloc(num_items * sizeof(SimpleMenuItem));

  for (size_t i = 0; i < s_num_entries; i++) {
    const char *prefix = prv_get_status_prefix(s_entries[i]->test_id);
    size_t len = snprintf(NULL, 0, "%zu. %s %s", i + 1, prefix, s_entries[i]->title) + 1;
    char *title = app_malloc(len);
    snprintf(title, len, "%zu. %s %s", i + 1, prefix, s_entries[i]->title);

    items[i] = (SimpleMenuItem) {
      .title = title,
      .callback = prv_select_test,
    };
  }

  items[s_num_entries] = (SimpleMenuItem) {
    .title = "RESULTS",
    .callback = prv_select_results,
  };

  data->menu_section = (SimpleMenuSection) {
    .num_items = num_items,
    .items = items
  };

  data->menu_layer = simple_menu_layer_create(bounds, data->window, &data->menu_section, 1, NULL);

  if (s_last_selected >= 0) {
    if (mfg_test_result_was_reported()) {
      const MfgTestResult *result = ((size_t)s_last_selected < s_num_entries)
          ? mfg_test_result_get(s_entries[s_last_selected]->test_id)
          : NULL;

      if (result && result->passed) {
        // Test passed: auto-advance to next test
        int16_t next = s_last_selected + 1;
        if ((size_t)next < s_num_entries) {
          simple_menu_layer_set_selected_index(data->menu_layer, next, false);
          prv_launch_test(next, s_entries[next]->get_info());
        } else {
          // Reached end of test list, launch RESULTS
          simple_menu_layer_set_selected_index(data->menu_layer, s_num_entries, false);
          prv_launch_test(s_num_entries, mfg_qr_results_app_get_info());
        }
      } else {
        // Test failed: stay on same test so operator can retry
        simple_menu_layer_set_selected_index(data->menu_layer, s_last_selected, false);
      }
    } else {
      // User backed out: select the test they came from
      simple_menu_layer_set_selected_index(data->menu_layer, s_last_selected, false);
    }
  }

  layer_add_child(window_layer, simple_menu_layer_get_layer(data->menu_layer));
}

bool mfg_test_menu_should_relaunch(void) {
  if (s_relaunch_menu) {
    s_relaunch_menu = false;
    return true;
  }
  return false;
}

bool mfg_test_menu_is_test_available(MfgTestId test_id) {
  for (size_t i = 0; i < ARRAY_LENGTH(s_all_entries); i++) {
    if (s_all_entries[i].test_id == test_id) {
      return (s_all_entries[i].modes & s_active_mode) != 0;
    }
  }
  return false;
}

static void prv_run_menu(uint8_t mode) {
  if (s_active_mode != mode) {
    s_last_selected = -1;
  }
  s_active_mode = mode;
  mfg_test_result_set_mode(mode);
  prv_build_filtered_entries(mode);

  MfgTestMenuAppData *data = app_malloc_check(sizeof(MfgTestMenuAppData));
  *data = (MfgTestMenuAppData){};

  app_state_set_user_data(data);

  data->window = window_create();
  window_init(data->window, "Tests");
  window_set_window_handlers(data->window, &(WindowHandlers) {
    .load = prv_window_load,
  });
  window_set_fullscreen(data->window, true);
  app_window_stack_push(data->window, true);

  app_event_loop();
}

static void s_main_semi_finished(void) {
  prv_run_menu(MFG_TEST_MODE_SEMI_FINISHED);
}

static void s_main_finished(void) {
  prv_run_menu(MFG_TEST_MODE_FINISHED);
}

const PebbleProcessMd* mfg_test_menu_semi_finished_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common.main_func = &s_main_semi_finished,
    // UUID: 8a3f6c1e-9b2d-4f5a-a7c3-1d4e6b8f9a2c
    .common.uuid = { 0x8a, 0x3f, 0x6c, 0x1e, 0x9b, 0x2d, 0x4f, 0x5a,
                     0xa7, 0xc3, 0x1d, 0x4e, 0x6b, 0x8f, 0x9a, 0x2c },
    .name = "MfgTestMenuSF",
  };
  return (const PebbleProcessMd*) &s_app_info;
}

const PebbleProcessMd* mfg_test_menu_finished_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common.main_func = &s_main_finished,
    // UUID: 8a3f6c1e-9b2d-4f5a-a7c3-1d4e6b8f9a2d
    .common.uuid = { 0x8a, 0x3f, 0x6c, 0x1e, 0x9b, 0x2d, 0x4f, 0x5a,
                     0xa7, 0xc3, 0x1d, 0x4e, 0x6b, 0x8f, 0x9a, 0x2d },
    .name = "MfgTestMenuFI",
  };
  return (const PebbleProcessMd*) &s_app_info;
}
