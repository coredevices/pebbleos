/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "mfg_test_aging.h"

#include "apps/prf/mfg_test_result.h"
#include "applib/app.h"
#include "applib/graphics/graphics.h"
#include "applib/tick_timer_service.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/vibes.h"
#include "applib/ui/window.h"
#include "board/board.h"
#include "console/console_internal.h"
#include <pbl/drivers/accel.h>
#include <pbl/drivers/ambient_light.h>
#include <pbl/drivers/audio.h>
#include <pbl/drivers/battery.h>
#include <pbl/drivers/backlight.h>
#include <pbl/drivers/mag.h>
#include "kernel/pbl_malloc.h"
#include "process_management/pebble_process_md.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/bluetooth/bluetooth_ctl.h"
#include "pbl/services/light.h"
#include "pbl/services/idle_watchdog.h"
#include <pbl/logging/logging.h>
#include "util/time/time.h"

#define STATUS_STRING_LEN 200
#define COMPONENT_TEST_DURATION_SEC 10
#define COMPONENT_BACKLIGHT_DURATION_SEC 2
#define COMPONENT_VIBE_DURATION_SEC 1

// Charge + cycling phase parameters
#define CHARGE_AND_CYCLE_DURATION_SEC (4 * 3600)  // 4 hours total
#define CHARGE_TIMEOUT_SEC (90 * 60)              // PMIC must report charge complete within 90min
#define CHARGE_COMPLETE_MIN_PERCENT 99            // Min SoC to accept a PMIC charge-complete
#define TEMP_MIN_MC 15000                         // 15.0C
#define TEMP_MAX_MC 35000                         // 35.0C

// Idle phase parameters
#define IDLE_DURATION_SEC (10 * 3600)  // 10 hours
#define IDLE_MAX_DROP_PERCENT 6        // Fail if battery drops more than this during idle

// Discharge phase parameters — bring battery down to a safe shipping level.
// Adjust here if the target ever changes.
#define DISCHARGE_TARGET_PERCENT 65

#ifdef CONFIG_SPEAKER
static const int16_t sine_wave_4k[] = {
  0, 32767, 0, -32768, 0, 32767, 0, -32768,
  0, 32767, 0, -32768, 0, 32767, 0, -32768,
};
#endif

typedef enum {
  AgingStateWaitPlug = 0,
  AgingStateChargingAndCycling,
  AgingStateWaitUnplug,
  AgingStateIdle,
  AgingStateDischarging,
  AgingStatePass,
  AgingStateFail,
} AgingState;

// Component sub-states for cycling
typedef enum {
  ComponentAccel = 0,
#ifdef CONFIG_MAG
  ComponentMag,
#endif
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
  ComponentBacklightWhite,
  ComponentBacklightRed,
  ComponentBacklightGreen,
  ComponentBacklightBlue,
#else
  ComponentBacklight,
#endif
#ifdef CONFIG_SPEAKER
  ComponentAudio,
#endif
  ComponentALS,
  ComponentVibe,
  NumComponents,
} ComponentState;

typedef struct {
  Window window;

  TextLayer title;
  TextLayer status;
  char status_string[STATUS_STRING_LEN];

  AgingState state;
  ComponentState component;
  uint32_t component_elapsed_sec;

  uint32_t phase_elapsed_sec;
  uint32_t cycle_count;

  // Set once the PMIC reports charge complete during the charge+cycle phase
  bool charge_complete;

  // Battery state at start of idle phase, used for the 6% drop check
  uint8_t initial_percent;

  char fail_reason[64];

#ifdef CONFIG_SPEAKER
  bool audio_playing;
#endif
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
  uint32_t saved_backlight_color;
#endif
} AppData;

#ifdef CONFIG_SPEAKER
static void prv_audio_trans_handler(uint32_t *free_size) {
  uint32_t available_size = *free_size;
  while (available_size > sizeof(sine_wave_4k)) {
    available_size = audio_write(AUDIO, (void *)&sine_wave_4k[0], sizeof(sine_wave_4k));
  }
}
#endif

static void prv_format_time(char *buffer, size_t size, uint32_t seconds) {
  uint32_t hours = seconds / 3600;
  uint32_t minutes = (seconds % 3600) / 60;
  uint32_t secs = seconds % 60;
  sniprintf(buffer, size, "%02" PRIu32 ":%02" PRIu32 ":%02" PRIu32, hours, minutes, secs);
}

static void prv_cleanup_component(AppData *data) {
#ifdef CONFIG_MAG
  if (data->component == ComponentMag) {
    mag_release();
  }
#endif
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
  if (data->component >= ComponentBacklightWhite &&
      data->component <= ComponentBacklightBlue) {
    backlight_set_color(data->saved_backlight_color);
    light_enable(false);
  }
#else
  if (data->component == ComponentBacklight) {
    light_enable(false);
  }
#endif
#ifdef CONFIG_SPEAKER
  if (data->audio_playing) {
    audio_stop(AUDIO);
    data->audio_playing = false;
  }
#endif
}

static void prv_start_component(AppData *data) {
#ifdef CONFIG_MAG
  if (data->component == ComponentMag) {
    mag_start_sampling();
  }
#endif
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
  if (data->component >= ComponentBacklightWhite &&
      data->component <= ComponentBacklightBlue) {
    light_enable(true);
  }
#else
  if (data->component == ComponentBacklight) {
    light_enable(true);
  }
#endif
}

static uint32_t prv_component_duration(ComponentState comp) {
  switch (comp) {
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
    case ComponentBacklightWhite:
    case ComponentBacklightRed:
    case ComponentBacklightGreen:
    case ComponentBacklightBlue:
#else
    case ComponentBacklight:
#endif
      return COMPONENT_BACKLIGHT_DURATION_SEC;
    case ComponentVibe:
      return COMPONENT_VIBE_DURATION_SEC;
    default:
      return COMPONENT_TEST_DURATION_SEC;
  }
}

static void prv_advance_component(AppData *data) {
  prv_cleanup_component(data);

  data->component_elapsed_sec = 0;
  data->component++;

  if (data->component >= NumComponents) {
    data->component = ComponentAccel;
    data->cycle_count++;
  }

  prv_start_component(data);
}

static void prv_report_result(bool passed) {
  // Always report into the FINISHED bucket: aging is launched outside the
  // per-mode test menu, so we can't rely on whatever mode happened to be
  // active last.
  mfg_test_result_set_mode(MFG_TEST_MODE_FINISHED);
  mfg_test_result_report(MfgTestId_Aging, passed, 0);
}

static void prv_enter_fail(AppData *data, const char *reason) {
  prv_cleanup_component(data);
  // Restore charging in case we disabled it during the charge+cycle phase,
  // so the device is left in a normal state on exit.
  battery_set_charge_enable(true);
  data->state = AgingStateFail;
  sniprintf(data->fail_reason, sizeof(data->fail_reason), "%s", reason);
  PBL_LOG_ERR("Aging test FAIL: %s", reason);
  prv_report_result(false);
}

static void prv_run_component_display(AppData *data) {
  char time_str[16];
  prv_format_time(time_str, sizeof(time_str), data->phase_elapsed_sec);

  uint32_t remaining = CHARGE_AND_CYCLE_DURATION_SEC - data->phase_elapsed_sec;
  char rem_str[16];
  prv_format_time(rem_str, sizeof(rem_str), remaining);

  const char *comp_name = "?";
  char comp_detail[48] = "";
  switch (data->component) {
    case ComponentAccel: {
      AccelDriverSample sample;
      accel_peek(&sample);
      comp_name = "Accel";
      sniprintf(comp_detail, sizeof(comp_detail),
                "X:%" PRIi16 " Y:%" PRIi16 " Z:%" PRIi16,
                sample.x, sample.y, sample.z);
      break;
    }
#ifdef CONFIG_MAG
    case ComponentMag: {
      MagData mag_sample;
      mag_read_data(&mag_sample);
      comp_name = "Mag";
      sniprintf(comp_detail, sizeof(comp_detail),
                "X:%" PRIi16 " Y:%" PRIi16 " Z:%" PRIi16,
                mag_sample.x, mag_sample.y, mag_sample.z);
      break;
    }
#endif
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
    case ComponentBacklightWhite:
      backlight_set_color(0xD0D0D0);
      comp_name = "BL White";
      break;
    case ComponentBacklightRed:
      backlight_set_color(0xFF0000);
      comp_name = "BL Red";
      break;
    case ComponentBacklightGreen:
      backlight_set_color(0x00FF00);
      comp_name = "BL Green";
      break;
    case ComponentBacklightBlue:
      backlight_set_color(0x0000FF);
      comp_name = "BL Blue";
      break;
#else
    case ComponentBacklight:
      comp_name = "Backlight";
      break;
#endif
#ifdef CONFIG_SPEAKER
    case ComponentAudio:
      comp_name = "Audio";
      if (!data->audio_playing) {
        audio_start(AUDIO, prv_audio_trans_handler);
        audio_set_volume(AUDIO, 100);
        data->audio_playing = true;
      }
      break;
#endif
    case ComponentALS: {
      uint32_t level = ambient_light_get_light_level();
      comp_name = "ALS";
      sniprintf(comp_detail, sizeof(comp_detail), "Level: %" PRIu32, level);
      break;
    }
    case ComponentVibe: {
      // Only pulse once at the start of the vibe phase
      if (data->component_elapsed_sec <= 1) {
        static const uint32_t vibe_durations[] = { 250 };
        static const uint32_t vibe_amplitudes[] = { 50 };
        VibePatternWithAmplitudes pat = {
          .durations = vibe_durations,
          .amplitudes = vibe_amplitudes,
          .num_segments = 1,
        };
        vibes_enqueue_custom_pattern_with_amplitudes(pat);
      }
      comp_name = "Vibe";
      break;
    }
    default:
      break;
  }

  BatteryConstants bc;
  battery_get_constants(&bc);
  BatteryChargeState cs = battery_get_charge_state();

  int8_t temp_c = (int8_t)(bc.t_mc / 1000);
  uint8_t temp_c_frac = ((bc.t_mc > 0 ? bc.t_mc : -bc.t_mc) % 1000) / 10;

  BatteryChargeStatus charge_status;
  battery_charge_status_get(&charge_status);
  const char *chg_str;
  if (!cs.is_charging) {
    chg_str = "Idle";
  } else {
    switch (charge_status) {
      case BatteryChargeStatusComplete: chg_str = "Cmpl"; break;
      case BatteryChargeStatusTrickle:  chg_str = "Trk";  break;
      case BatteryChargeStatusCC:       chg_str = "CC";   break;
      case BatteryChargeStatusCV:       chg_str = "CV";   break;
      default:                          chg_str = "?";    break;
    }
  }

  sniprintf(data->status_string, sizeof(data->status_string),
            "CHG+CYC [%s]\n"
            "Cycle: %" PRIu32 "\n"
            "Elapsed: %s\n"
            "Remain: %s\n"
            "%" PRId32 "mV %" PRIu8 "%% %s\n"
            "%" PRId8 ".%02" PRIu8 "C\n"
            "%s",
            comp_name, data->cycle_count,
            time_str, rem_str,
            bc.v_mv, cs.charge_percent, chg_str,
            temp_c, temp_c_frac,
            comp_detail);
}

static void prv_handle_tick(struct tm *tick_time, TimeUnits units_changed) {
  AppData *data = app_state_get_user_data();

  switch (data->state) {
    case AgingStateWaitPlug: {
      BatteryChargeState cs = battery_get_charge_state();
      if (cs.is_plugged) {
        data->state = AgingStateChargingAndCycling;
        data->phase_elapsed_sec = 0;
        // Completion is derived from the PMIC charge state inside the
        // charge+cycle phase, not pre-decided from the SoC at plug-in.
        data->charge_complete = false;
        data->component = ComponentAccel;
        data->component_elapsed_sec = 0;
        data->cycle_count = 1;
        prv_start_component(data);
        sniprintf(data->status_string, sizeof(data->status_string),
                  "CHG+CYC\nStarting...");
      } else {
        sniprintf(data->status_string, sizeof(data->status_string),
                  "PLUG CHARGER\n\nPlug the watch\nto start");
      }
      break;
    }

    case AgingStateChargingAndCycling: {
      data->phase_elapsed_sec++;
      data->component_elapsed_sec++;

      BatteryConstants bc;
      battery_get_constants(&bc);
      BatteryChargeState cs = battery_get_charge_state();

      if (!cs.is_plugged) {
        prv_enter_fail(data, "Unplugged during\ncharge+cycle");
        break;
      }

      if (bc.t_mc < TEMP_MIN_MC || bc.t_mc > TEMP_MAX_MC) {
        prv_enter_fail(data, "Temperature out\nof range");
        break;
      }

      // Charge completion tracking. We're guaranteed plugged here (an unplug
      // fails above). Treat the battery as full once the PMIC stops charging
      // on its own at a near-full SoC. "Plugged but not charging" below the
      // threshold is a transient (just-plugged / between cycles), so we keep
      // waiting and may restart. A charger/cell fault never reaches this and
      // trips the timeout instead.
      if (!data->charge_complete) {
        if (!cs.is_charging && cs.charge_percent >= CHARGE_COMPLETE_MIN_PERCENT) {
          data->charge_complete = true;
          // Now that the PMIC reports full, stop active charging so the cell
          // isn't held near 100% by continuous top-off for the rest of the
          // cycling phase, which is gentler on the cell. The battery may sag a
          // little under cycling load with charging off; that's expected, and
          // charge retention is validated separately by the idle phase.
          battery_set_charge_enable(false);
        } else if (data->phase_elapsed_sec >= CHARGE_TIMEOUT_SEC) {
          prv_enter_fail(data, "Charge timeout\n(90min)");
          break;
        }
      }

      // Phase done?
      if (data->phase_elapsed_sec >= CHARGE_AND_CYCLE_DURATION_SEC) {
        if (!data->charge_complete) {
          prv_enter_fail(data, "Charge incomplete\nat phase end");
          break;
        }
        prv_cleanup_component(data);
        battery_set_charge_enable(true);
        data->state = AgingStateWaitUnplug;
        break;
      }

      // Advance component if needed
      if (data->component_elapsed_sec >= prv_component_duration(data->component)) {
        prv_advance_component(data);
      }

      prv_run_component_display(data);
      break;
    }

    case AgingStateWaitUnplug: {
      BatteryChargeState cs = battery_get_charge_state();
      if (!cs.is_plugged) {
        data->state = AgingStateIdle;
        data->phase_elapsed_sec = 0;
        data->initial_percent = cs.charge_percent;
        sniprintf(data->status_string, sizeof(data->status_string), "IDLE\nStarting...");
      } else {
        sniprintf(data->status_string, sizeof(data->status_string),
                  "UNPLUG WATCH\n\nCharged to %" PRIu8 "%%\n"
                  "Unplug to begin\nidle test",
                  cs.charge_percent);
      }
      break;
    }

    case AgingStateIdle: {
      data->phase_elapsed_sec++;

      BatteryConstants bc;
      battery_get_constants(&bc);
      BatteryChargeState cs = battery_get_charge_state();

      if (cs.is_plugged) {
        prv_enter_fail(data, "Plugged during\nidle test");
        break;
      }

      int8_t drop = (int8_t)data->initial_percent - (int8_t)cs.charge_percent;
      if (drop < 0) {
        drop = 0;
      }

      // Phase done?
      if (data->phase_elapsed_sec >= IDLE_DURATION_SEC) {
        if (drop > IDLE_MAX_DROP_PERCENT) {
          char reason[64];
          sniprintf(reason, sizeof(reason),
                    "Idle drop %" PRId8 "%%\n> %d%%",
                    drop, IDLE_MAX_DROP_PERCENT);
          prv_enter_fail(data, reason);
          break;
        }
        // Idle passed — discharge to safe shipping level with backlight on
        data->state = AgingStateDischarging;
        data->phase_elapsed_sec = 0;
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
        data->saved_backlight_color = backlight_get_color();
        backlight_set_color(0xFFFFFF);
#endif
        light_enable(true);
        // The screen only needs occasional refreshes while discharging, so
        // tick once a minute to cut wake-ups and the test's own power draw.
        tick_timer_service_subscribe(MINUTE_UNIT, prv_handle_tick);
        break;
      }

      char time_str[16], rem_str[16];
      prv_format_time(time_str, sizeof(time_str), data->phase_elapsed_sec);
      prv_format_time(rem_str, sizeof(rem_str),
                      IDLE_DURATION_SEC - data->phase_elapsed_sec);

      int8_t temp_c = (int8_t)(bc.t_mc / 1000);
      uint8_t temp_c_frac = ((bc.t_mc > 0 ? bc.t_mc : -bc.t_mc) % 1000) / 10;

      sniprintf(data->status_string, sizeof(data->status_string),
                "IDLE\n"
                "Elapsed: %s\n"
                "Remain: %s\n"
                "%" PRId32 "mV %" PRIu8 "%%\n"
                "%" PRId8 ".%02" PRIu8 "C\n"
                "Start:%" PRIu8 "%% Drop:%" PRId8 "/%d%%",
                time_str, rem_str,
                bc.v_mv, cs.charge_percent,
                temp_c, temp_c_frac,
                data->initial_percent, drop, IDLE_MAX_DROP_PERCENT);
      break;
    }

    case AgingStateDischarging: {
      // This phase ticks once a minute (see the Idle->Discharge transition).
      data->phase_elapsed_sec += SECONDS_PER_MINUTE;

      BatteryConstants bc;
      battery_get_constants(&bc);
      BatteryChargeState cs = battery_get_charge_state();

      if (cs.is_plugged) {
        light_enable(false);
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
        backlight_set_color(data->saved_backlight_color);
#endif
        prv_enter_fail(data, "Plugged while\ndischarging");
        break;
      }

      if (cs.charge_percent <= DISCHARGE_TARGET_PERCENT) {
        light_enable(false);
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
        backlight_set_color(data->saved_backlight_color);
#endif
        data->state = AgingStatePass;
        sniprintf(data->status_string, sizeof(data->status_string),
                  "PASS\n\nDischarged to\n%" PRIu8 "%%",
                  cs.charge_percent);
        prv_report_result(true);
        tick_timer_service_unsubscribe();
        break;
      }

      char time_str[16];
      prv_format_time(time_str, sizeof(time_str), data->phase_elapsed_sec);

      int8_t temp_c = (int8_t)(bc.t_mc / 1000);
      uint8_t temp_c_frac = ((bc.t_mc > 0 ? bc.t_mc : -bc.t_mc) % 1000) / 10;

      sniprintf(data->status_string, sizeof(data->status_string),
                "DISCHARGING\nTime: %s\n\n"
                "%" PRId32 "mV %" PRIu8 "%%\n"
                "%" PRId8 ".%02" PRIu8 "C\n\n"
                "Target: %d%%",
                time_str, bc.v_mv, cs.charge_percent,
                temp_c, temp_c_frac,
                DISCHARGE_TARGET_PERCENT);
      break;
    }

    case AgingStatePass:
    case AgingStateFail:
      return;
  }

  if (data->state == AgingStateFail) {
    sniprintf(data->status_string, sizeof(data->status_string),
              "FAIL\n\n%s", data->fail_reason);
  }

  text_layer_set_text(&data->status, data->status_string);
}

static void prv_back_click_handler(ClickRecognizerRef recognizer, void *context) {
  AppData *data = app_state_get_user_data();

  if (data->state == AgingStateDischarging) {
    light_enable(false);
#ifdef CONFIG_BACKLIGHT_HAS_COLOR
    backlight_set_color(data->saved_backlight_color);
#endif
  } else if (data->state == AgingStateChargingAndCycling) {
    prv_cleanup_component(data);
    battery_set_charge_enable(true);
  }

  tick_timer_service_unsubscribe();

  serial_console_set_rx_enabled(true);
  bt_ctl_set_enabled(true);
  prf_idle_watchdog_start();

  app_window_stack_pop(true);
}

static void prv_config_provider(void *context) {
  window_single_click_subscribe(BUTTON_ID_BACK, prv_back_click_handler);
}

static void prv_handle_init(void) {
  AppData *data = app_malloc_check(sizeof(AppData));
  *data = (AppData){
    .state = AgingStateWaitPlug,
#ifdef CONFIG_SPEAKER
    .audio_playing = false,
#endif
  };

  app_state_set_user_data(data);

  // Disable power consumers and idle watchdog for the entire test
  serial_console_set_rx_enabled(false);
  bt_ctl_set_enabled(false);
  prf_idle_watchdog_stop();

  // Make sure charging is enabled at start of test in case a previous run
  // left it disabled.
  battery_set_charge_enable(true);

  Window *window = &data->window;
  window_init(window, "");
  window_set_fullscreen(window, true);
  window_set_click_config_provider(window, prv_config_provider);

  Layer *window_layer = &window->layer;
  GRect bounds = window_layer->bounds;

  TextLayer *title = &data->title;
  const int16_t title_y = PBL_IF_ROUND_ELSE(10, 0);
  text_layer_init(title, &GRect(0, title_y, bounds.size.w, 24));
  text_layer_set_font(title, fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD));
  text_layer_set_text_alignment(title, GTextAlignmentCenter);
  text_layer_set_text(title, "AGING TEST");
  layer_add_child(window_layer, &title->layer);

  TextLayer *status = &data->status;
  const int16_t status_y = PBL_IF_ROUND_ELSE(40, 25);
  const int16_t status_x = PBL_IF_ROUND_ELSE(15, 5);
  text_layer_init(status, &GRect(status_x, status_y, bounds.size.w - (status_x * 2),
                                 bounds.size.h - status_y));
  text_layer_set_font(status, fonts_get_system_font(FONT_KEY_GOTHIC_18));
  text_layer_set_text_alignment(status, PBL_IF_ROUND_ELSE(GTextAlignmentCenter,
                                                           GTextAlignmentLeft));
  layer_add_child(window_layer, &status->layer);

  tick_timer_service_subscribe(SECOND_UNIT, prv_handle_tick);

  app_window_stack_push(window, true);
}

static void s_main(void) {
  prv_handle_init();

  app_event_loop();
}

const PebbleProcessMd *mfg_test_aging_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common.main_func = &s_main,
    // UUID: 12345678-ABCD-EF01-2345-6789ABCDEF01
    .common.uuid = { 0x12, 0x34, 0x56, 0x78, 0xAB, 0xCD, 0xEF, 0x01,
                     0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x01 },
    .name = "MfgTestAging",
  };
  return (const PebbleProcessMd *)&s_app_info;
}
