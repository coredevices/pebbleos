/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "console/prompt.h"
#include "console/pulse_internal.h"
#include "services/normal/filesystem/pfs.h"
#include "system/logging.h"
#include "util/size.h"

extern void command_help(void);

extern void command_log_level_set(const char*);
extern void command_log_level_get(void);

extern void command_log_dump_current(void);
extern void command_log_dump_last(void);
extern void command_log_dump_spam(void);
extern void command_log_dump_generation(const char*);

extern void command_put_raw_button_event(const char*, const char*);
extern void command_put_button_event(const char*, const char*);
extern void command_button_press(const char*, const char*);
extern void command_button_press_multiple(const char *, const char *, const char *, const char *);
extern void command_button_press_short(const char*);

extern void command_stats_dump_now(void);
extern void command_stats_dump_current(void);
extern void command_stats_dump_last(void);
extern void command_stats_dump_generation(const char*);

extern void command_crash(void);
extern void command_hard_crash(void);
extern void command_reset(void);
extern void command_boot_prf(void);
extern void command_factory_reset(void);
extern void command_factory_reset_fast(void);

extern void command_infinite_loop(void);
extern void command_assert_fail(void);
extern void command_stuck_timer(void);

extern void command_croak(void);
extern void command_hardfault(void);

extern void command_dump_malloc_kernel(void);
extern void command_dump_malloc_app(void);
extern void command_dump_malloc_worker(void);
extern void command_dump_malloc_bt(void);

extern void command_read_word(const char*);

extern void command_power_2v5(const char*);

extern void command_backlight_ctl(const char*);
extern void command_rgb_set_color(const char*);

extern void command_battery_charge_option(const char*);

extern void command_print_battery_status(void);
extern void command_compass_peek(void);
extern void command_accel_peek(void);
extern void command_accel_num_samples(char *num_samples);
extern void command_accel_status(void);
extern void command_accel_selftest(void);
extern void command_accel_softreset(void);

extern void command_dump_flash(const char*, const char*);
extern void command_crc_flash(const char*, const char*);
extern void command_format_flash(void);
extern void command_erase_flash(const char*, const char*);
extern void command_flash_read(const char*, const char*);
extern void command_flash_switch_mode(const char*);
extern void command_flash_fill(const char*, const char*, const char*);
extern void command_flash_test(const char* test_case_num_str, const char* iterations_str);
extern void command_flash_test_locked_sectors(void);
extern void command_flash_stress(const char *);
extern void command_flash_validate(void);
extern void command_flash_apicheck(const char *len);
extern void command_flash_unprotect(void);
//extern void command_flash_signal_test_init(void);
//extern void command_flash_signal_test_run(void);
extern void command_flash_show_erased_sectors(const char *arg);
#if CAPABILITY_HAS_FLASH_OTP
extern void command_flash_sec_read(const char *);
extern void command_flash_sec_write(const char *, const char *);
extern void command_flash_sec_erase(const char *);
extern void command_flash_sec_wipe(void);
extern void command_flash_sec_info(void);
#if defined(RECOVERY_FW)
extern void command_flash_sec_lock(const char *);
#endif // RECOVERY_FW
#endif // CAPABILITY_HAS_FLASH_OTP

extern void command_get_time(void);
extern void command_set_time(const char *arg);
extern void command_timezone_clear(void);

extern void command_vibe_ctl(const char *arg);

// extern void command_print_task_list(void);
extern void command_timers(void);

extern void command_bt_airplane_mode(const char*);
extern void command_bt_prefs_wipe(void);
extern void command_bt_print_mac(void);
extern void command_bt_set_addr(const char *bd_addr);
extern void command_bt_set_name(const char *bt_name);

extern void command_bt_status(void);

// extern void command_get_remote_prefs(void);
// extern void command_del_remote_pref(const char*);
// extern void command_bt_sniff_bounce(void);
// extern void command_bt_active_enter(void);
// extern void command_bt_active_exit(void);

extern void command_get_active_app_metadata(void);

extern void command_app_list(void);
extern void command_app_launch(const char* app_num_str);
extern void command_app_remove(const char* app_num_str);

extern void command_worker_launch(const char* app_num_str);
extern void command_worker_kill(void);

extern void command_boot_bit_set(const char* bit, const char* value);
extern void command_boot_bits_get(void);

extern void command_window_stack_info(void);
extern void command_modal_stack_info(void);
extern void command_animations_info(void);
extern void command_legacy2_animations_info(void);

extern void command_sim_panic(const char*);

extern void command_alarm(void);

extern void command_bt_test_start(void);
extern void command_bt_test_stop(void);

extern void command_bt_test_hci_passthrough();
extern void command_bt_test_bt_sig_rf_mode();

extern void command_watch(void);

extern void command_print_now_playing(void);

extern void command_selftest(void);
extern void command_enter_mfg(void);
extern void command_enter_standby(void);
extern void command_enter_consumer_mode(void);
extern void command_power_5v(const char*);

extern void command_accessory_imaging_start(void);

extern void command_serial_read(void);
extern void command_hwver_read(void);
extern void command_pcba_serial_read(void);
extern void command_color_read(void);
extern void command_disp_offset_read(void);
extern void command_rtcfreq_read(void);
extern void command_model_read(void);

extern void command_serial_write(const char*);
extern void command_hwver_write(const char*);
extern void command_pcba_serial_write(const char*);
extern void command_color_write(const char*);
extern void command_disp_offset_write(const char*);
extern void command_rtcfreq_write(const char*);
extern void command_model_write(const char*);
extern void command_bootloader_test(const char*);

extern void command_version_info(void);

extern void command_als_read(void);
extern void command_temperature_read(void);

extern void command_get_connected_os(void);

extern void command_dump_window(void);
extern void command_layer_nudge(const char *address);

extern void command_scheduler_force_active(void);
extern void command_scheduler_resume_normal(void);

extern void command_button_read(const char*);

extern void memory_layout_dump_mpu_regions_to_dbgserial(void);

extern void command_dls_list(void);
extern void command_dls_show(const char *id);
extern void command_dls_erase_all(void);
extern void command_dls_send_all(void);

extern void pfs_command_fs_format(const char *erase_headers);
extern void pfs_command_fs_ls(void);
extern void pfs_command_cat(const char *filename, const char *num_chars);
extern void pfs_command_dump_hdr(const char *page);
extern void pfs_command_crc(const char *filename);
extern void pfs_command_stress(void);

extern void command_fs_reset(void);
extern void command_fs_format(const char *erase_headers);
extern void command_fs_ls(void);
extern void command_fs_du(void);
extern void command_fs_stat(const char *filename);
extern void command_fs_rm(const char *filename);
extern void command_fs_header(const char *page_num);

extern void command_pmic_read_registers(void);

extern void command_ping_send(void);

extern void command_display_set(const char *color);
#if CAPABILITY_HAS_ACCESSORY_CONNECTOR
extern void command_accessory_power_set(const char *on);
extern void command_accessory_stress_test(void);
extern void command_smartstrap_status(void);
#endif
extern void command_mic_start(char *timeout_str, char *sample_size_str, char *sample_rate_str,
                              char *volume_str);
extern void command_mic_read(void);
// extern void command_pmic_rails(void);

extern void dump_current_runtime_stats(void);

extern void command_set_runlevel(const char *runlevel);

extern void command_litter_filesystem(void);

typedef struct Command {
  char* cmd_str;
  void* func;
  unsigned int num_params;
} Command;

extern void command_profiler_start(void);
extern void command_profiler_stop(void);
extern void command_profiler_stats(void);

extern void command_battery_ui_display(const char *, const char *, const char *);
extern void command_battery_ui_update(const char *, const char *, const char *);
extern void command_battery_ui_dismiss(void);

extern void command_gapdb_dump(void);
extern void command_force_shared_prf_flush(void);
extern void command_waste_time(const char *count_arg, const char *delay_arg);
extern void command_bt_sprf_nuke(void);

extern void command_pause_animations(void);
extern void command_resume_animations(void);

extern void command_change_le_mode(char *mode);
extern void command_le_mode_chaos_monkey(char *enabled_str);
extern void command_ble_send_service_changed_indication(void);
extern void command_ble_rediscover(void);
extern void command_ble_logging_set_level(const char *level);
extern void command_ble_logging_get_level(void);
extern void command_ble_core_dump(const char *command);

extern void command_low_power_debug(char *enable_arg);

extern void command_audit_delay_us(void);
extern void command_enter_stop(void);

extern void dialog_test_cmds(void);

extern void command_dump_notif_pref_db(void);

extern void command_bt_conn_param_set(
    char *interval_min_ms, char *interval_max_ms, char *slave_latency, char *timeout_ms);
extern void command_bt_disc_start(char *start_handle, char *end_handle);
extern void command_bt_disc_stop(void);

// Commands to support MFG Bluetooth LE-only testing
extern void command_btle_test_le_tx_start(
    char *tx_channel, char *tx_packet_length, char *packet_payload_type);
extern void command_btle_test_rx_start(char *rx_channel);
extern void command_btle_test_end(void);
extern void command_btle_pa_set(char *option);
extern void command_btle_unmod_tx_start(char *tx_channel);
extern void command_btle_unmod_tx_stop(void);

#if CAPABILITY_HAS_BUILTIN_HRM
extern void command_hrm_read(void);
extern void command_hrm_wipe(void);
extern void command_hrm_freeze(void);
#endif

#if MFG_INFO_RECORDS_TEST_RESULTS
extern void command_mfg_info_test_results(void);
#endif

extern void command_perftest_line(const char *, const char *);
extern void command_perftest_line_all(void);
extern void command_perftest_text(const char *, const char *, const char *);
extern void command_perftest_text_all(void);

extern void command_bt_sleep_check(const char *iters);

#if MEMFAULT
extern void command_mflt_export(void);
extern void command_mflt_heartbeat(void);
extern void command_mflt_metrics_dump(void);
extern void command_mflt_device_info(void);
#endif

#if PLATFORM_TINTIN && !TARGET_QEMU
// We don't have space for anything that's not absolutely required for firmware development
// (imaging resources over PULSE). Rip it all out. Note that this breaks test automation on tintin,
// but QEMU will be used as a placeholder. We plan on reintroducing test automation support through
// continued space savings efforts and by introducing RPC commands over PULSE. Stay tuned.
// For reference, this saves about 12k of space. If we leave just the test automation commands in
// roughly 7k of space is saved.
#define KEEP_NON_ESSENTIAL_COMMANDS 0
#else
#define KEEP_NON_ESSENTIAL_COMMANDS 1
#endif
static const Command s_prompt_commands[] = {
  // PULSE entry point, needed for anything PULSE-related to work
  { "PULSEv1", pulse_start, 0 },
#if KEEP_NON_ESSENTIAL_COMMANDS == 1
  // ====================================================================================
  // NOTE: The following commands are used by test automation.
  // Disabling/removing them will break testing against those FW builds.
  { "click short", command_button_press_short, 1 },
  { "click multiple", command_button_press_multiple, 4 },
  { "click long", command_button_press, 2 },
  { "reset", command_reset, 0 },
  { "crash", command_crash, 0 },
  { "hard crash", command_hard_crash, 0 },
#ifndef RECOVERY_FW
  { "factory reset fast", command_factory_reset_fast, 0 },
#endif
  { "factory reset", command_factory_reset, 0 },
  { "set time", command_set_time, 1 },
  { "version", command_version_info, 0 },
  { "boot bit set", command_boot_bit_set, 2 },
  { "window stack", command_window_stack_info, 0 },
  { "modal stack", command_modal_stack_info, 0 },
  { "battery chargeopt", command_battery_charge_option, 1},
  { "bt airplane mode", command_bt_airplane_mode, 1 },
  { "bt prefs wipe", command_bt_prefs_wipe, 0 },
  { "bt mac", command_bt_print_mac, 0 },
  { "bt set addr", command_bt_set_addr, 1 },
  { "bt set name", command_bt_set_name, 1 },
  { "bt cp set", command_bt_conn_param_set, 4 },
  { "bt disc start", command_bt_disc_start, 2 },
  { "bt disc stop", command_bt_disc_stop, 0 },
  { "timezone clear", command_timezone_clear, 0 },
  { "battery status", command_print_battery_status, 0 },
#ifndef RELEASE
  { "audit delay", command_audit_delay_us, 0 },
  { "enter stop", command_enter_stop, 0},
#endif
#ifndef RECOVERY_FW
  { "app list", command_app_list, 0 },
  { "app launch", command_app_launch, 1 },
  { "app remove", command_app_remove, 1 },
#endif
  // End of automation commands
  // ====================================================================================

  { "erase flash", command_erase_flash, 2 },
  { "crc flash", command_crc_flash, 2 },
#ifndef RECOVERY_FW
#if CAPABILITY_HAS_TEMPERATURE
  { "temp read",  command_temperature_read, 0 },
#endif
  { "als read", command_als_read, 0},
#ifndef RELEASE
  { "litter pfs", command_litter_filesystem, 0 },
#endif
#endif

  // ====================================================================================
  // Following commands are used for manufacturing. We use a PRF firmware for manufacturing, so
  // we can only include these commands when we're building for PRF. Some of the commands are
  // specific to snowy manufacturing as well
#ifdef RECOVERY_FW
#if CAPABILITY_HAS_ACCESSORY_CONNECTOR
  { "accessory imaging start", command_accessory_imaging_start, 0 },
#endif

  { "info", command_version_info, 0 },

  { "enter mfg", command_enter_mfg, 0 },

  { "enter standby", command_enter_standby, 0 },

  { "enter consumer", command_enter_consumer_mode, 0 },

  { "serial read", command_serial_read, 0 },
  { "hwver read", command_hwver_read, 0 },
  { "pcbaserial read", command_pcba_serial_read, 0 },
  { "color read", command_color_read, 0 },
#if PBL_ROUND
  { "disp offset read", command_disp_offset_read, 0 },
#endif
  { "rtcfreq read", command_rtcfreq_read, 0 },
  { "model read", command_model_read, 0 },

#if CAPABILITY_HAS_FLASH_OTP
  { "flash sec lock", command_flash_sec_lock, 1},
#endif // CAPABILITY_HAS_FLASH_OTP

  { "serial write", command_serial_write, 1 },
  { "hwver write", command_hwver_write, 1 },
  { "pcbaserial write", command_pcba_serial_write, 1 },
#if MANUFACTURING_FW
  { "color write", command_color_write, 1 },
#if PBL_ROUND
  { "disp offset write", command_disp_offset_write, 2 },
#endif
  { "rtcfreq write", command_rtcfreq_write, 1 },
  { "model write", command_model_write, 1 },
#endif // MANUFACTURING_FW
  { "bootloader test", command_bootloader_test, 1 },

  { "scheduler force active", command_scheduler_force_active, 0 },
  { "scheduler resume normal", command_scheduler_resume_normal, 0 },

  { "bt status", command_bt_status, 0 },
  { "bt test start", command_bt_test_start, 0 },
  { "bt test stop", command_bt_test_stop, 0 },
  { "bt test hcipass", command_bt_test_hci_passthrough, 0 },
#if BT_CONTROLLER_DA14681
  { "bt sleep check", command_bt_sleep_check, 1 },
  { "btle tx test start", command_btle_test_le_tx_start, 3 },
  { "btle rx test start", command_btle_test_rx_start, 1 },
  { "btle test end", command_btle_test_end, 0 },
  { "btle umod tx test start", command_btle_unmod_tx_start, 1 },
  { "btle umod tx test stop", command_btle_unmod_tx_stop, 0 },
# if PLATFORM_ROBERT
  { "btle test pa", command_btle_pa_set, 1 },
# endif
#endif

  { "bt test bt_sig_rf", command_bt_test_bt_sig_rf_mode, 0},

  { "backlight", command_backlight_ctl, 1 },

  { "button read", command_button_read, 1 },

#if CAPABILITY_HAS_MAGNETOMETER
  { "compass peek", command_compass_peek, 0 },
#endif // CAPABILITY_HAS_MAGNETOMETER
  { "accel read", command_accel_peek, 0 },

  { "als read", command_als_read, 0},

#ifdef PLATFORM_TINTIN // TINTIN/BIANCA only
  { "power 2.5", command_power_2v5, 1 },
#else
  { "selftest", command_selftest, 0 },

  { "flash read", command_flash_read, 2},
  { "flash switchmode", command_flash_switch_mode, 1},
  { "flash fill", command_flash_fill, 3},
#if CAPABILITY_USE_PARALLEL_FLASH
  { "flash test", command_flash_test, 2},
#endif
  { "flash validate", command_flash_validate, 0},
  { "flash erased_sectors", command_flash_show_erased_sectors, 1},
#if !RELEASE && (PLATFORM_SILK || PLATFORM_ROBERT || PLATFORM_CALCULUS)
  { "flash apicheck", command_flash_apicheck, 1},
  //{ "flash signal test init", command_flash_signal_test_init, 0 },
  //{ "flash signal test run", command_flash_signal_test_run, 0 },
#endif
#if CAPABILITY_HAS_FLASH_OTP
  { "flash sec read", command_flash_sec_read, 1},
  { "flash sec write", command_flash_sec_write, 2},
  { "flash sec erase", command_flash_sec_erase, 1},
  { "flash sec wipe", command_flash_sec_wipe, 0},
  { "flash sec info", command_flash_sec_info, 0},
#endif // CAPABILITY_HAS_FLASH_OTP

  //{ "pmic rails", command_pmic_rails, 0},


  { "disp", command_display_set, 1},

#if MFG_INFO_RECORDS_TEST_RESULTS
  { "mfg ui test results", command_mfg_info_test_results, 0 },
#endif // MFG_INFO_RECORDS_TEST_RESULTS

#endif // PLATFORM_TINTIN
#endif // RECOVERY_FW

#if CAPABILITY_HAS_BUILTIN_HRM
  { "hrm read", command_hrm_read, 0},
  { "hrm wipe", command_hrm_wipe, 0},
  { "hrm freeze", command_hrm_freeze, 0},
#endif

#if CAPABILITY_HAS_ACCESSORY_CONNECTOR
  { "accessory power", command_accessory_power_set, 1 },
  { "accessory stress", command_accessory_stress_test, 0 },
#if !RELEASE && !RECOVERY_FW
  { "smartstrap status", command_smartstrap_status, 0 },
#endif // RELEASE
#endif // CAPABILITY_HAS_ACCESSORY_CONNECTOR

#if CAPABILITY_HAS_PMIC
  {"pmic regs", command_pmic_read_registers, 0},
#endif

#if CAPABILITY_HAS_MICROPHONE
  { "mic start", command_mic_start, 4},
  { "mic read",  command_mic_read, 0},

#endif


  // End of manufacturing commands
  // ====================================================================================


  // The rest of the commands are pretty much a misc free-for-all of functionality that's useful
  // for debugging.

  // Meta
  { "help", command_help, 0 },
  { "lowpowerdebug", command_low_power_debug, 1 },

  { "log level set", command_log_level_set, 1 },
  { "log level get", command_log_level_get, 0 },

  { "log dump current", command_log_dump_current, 0 },
  { "log dump last", command_log_dump_last, 0 },
  { "log spam", command_log_dump_spam, 0 },
  { "log dump gen", command_log_dump_generation, 1 },

  { "ble mode", command_change_le_mode, 1 },
  { "ble ind svc", command_ble_send_service_changed_indication, 0 },
  { "ble rediscover", command_ble_rediscover, 0 },
  // { "ble mode_monkey", command_le_mode_chaos_monkey, 1 },
  { "ble set log level", command_ble_logging_set_level, 1},
  { "ble get log level", command_ble_logging_get_level, 0},
  { "ble core dump", command_ble_core_dump, 1 },

  /*
  { "stats dump now", command_stats_dump_now, 0 },
  { "stats dump current", command_stats_dump_current, 0 },
  { "stats dump last", command_stats_dump_last, 0 },
  { "stats dump generation", command_stats_dump_generation, 1 },
  */

  // Buttons
  { "raw button event", command_put_raw_button_event, 2 },
  // { "click button event", command_put_button_event, 2 },

  // General utils
  // { "boot prf", command_boot_prf, 0 },

  /*
  { "infinite loop", command_infinite_loop, 0 },
  { "assert fail", command_assert_fail, 0 },
  { "stuck timer", command_stuck_timer, 0 },
  { "hard fault", command_hardfault, 0 },
  */
  { "croak", command_croak, 0 },

#ifdef MALLOC_INSTRUMENTATION
  { "dump malloc kernel", command_dump_malloc_kernel, 0 },
  { "dump malloc app", command_dump_malloc_app, 0 },
  { "dump malloc worker", command_dump_malloc_worker, 0 },
#if BT_CONTROLLER_CC2564X
  { "dump malloc bt", command_dump_malloc_bt, 0 },
#endif /* BT_CONTROLLER_CC2564X */
#endif /* MALLOC_INSTRUMENTATION */

  /*
  { "read word", command_read_word, 1 },

  { "remote os", command_get_connected_os, 0 },
  */

#ifdef UI_DEBUG
  { "window dump", command_dump_window, 0 },
  { "layer nudge", command_layer_nudge, 1 },
#endif


  // Drivers
  //{ "rgb", command_rgb_set_color, 1 },

//  { "watch", command_watch, 0 },

  // Flash manipulation commands
  { "dump flash", command_dump_flash, 2 },
  // { "format flash", command_format_flash, 0 },

#if !PLATFORM_TINTIN
  { "flash unprotect", command_flash_unprotect, 0 },
#endif

#ifndef RECOVERY_FW
  { "worker launch", command_worker_launch, 1 },
  { "worker kill", command_worker_kill, 0},
#endif

#ifdef TEST_FLASH_LOCK_PROTECTION
  { "flash lock test", command_flash_test_locked_sectors, 0 },
#endif

  /*
  { "get time", command_get_time, 0 },
  */

  // Firmware specific
  //{ "task-list", command_print_task_list, 0 },

  //{ "cpustats", dump_current_runtime_stats, 0 },
  //{ "bt prefs get", command_get_remote_prefs, 0 },
  //{ "bt prefs del", command_del_remote_pref, 1 },
  //{ "bt sniff bounce", command_bt_sniff_bounce, 0 },
  //{ "bt active enter", command_bt_active_enter, 0 },
  //{ "bt active exit", command_bt_active_exit, 0 },


#if !defined(RECOVERY_FW)
  { "get active app metadata", command_get_active_app_metadata, 0 },
#endif
//  { "boot bits get", command_boot_bits_get, 0 },

    { "animations", command_animations_info, 0 },
    { "pause animations", command_pause_animations, 0 },
    { "resume animations", command_resume_animations, 0 },

//  { "animations_l2", command_legacy2_animations_info, 0 },

// #if !defined(RECOVERY_FW)
//  { "sim panic", command_sim_panic, 1 },
// #endif

#if !defined(RECOVERY_FW)
  { "alarm", command_alarm, 0 },

  //{ "now playing", command_print_now_playing, 0 },

  { "dls list", command_dls_list, 0 },
  //  { "dls show", command_dls_show, 1 },
  { "dls wipe", command_dls_erase_all, 0 },
  { "dls send", command_dls_send_all, 0 },

#endif // !RECOVERY_FW

  { "dump mpu", memory_layout_dump_mpu_regions_to_dbgserial, 0 },

#ifndef RECOVERY_FW
  {"pfs format", pfs_command_fs_format, 1},
  {"pfs ls", pfs_command_fs_ls, 0},
  // {"pfs cat", pfs_command_cat, 2},
  //  {"pfs rmall", pfs_remove_all, 0},
  {"pfs rm", pfs_remove, 1},
  {"pfs hdr", pfs_command_dump_hdr, 1},
  // {"pfs stress", pfs_command_stress, 0 },
  {"pfs crc", pfs_command_crc, 1},

  // This command is dangerous to your flash.  Be careful.
  {"flash stress", command_flash_stress, 1 },
#endif

  { "ping", command_ping_send, 0},

  { "runlevel", command_set_runlevel, 1 },

#if defined(PROFILER)
  { "profiler start", command_profiler_start, 0 },
  { "profiler stop", command_profiler_stop, 0 },
  { "profiler stats", command_profiler_stats, 0 },
#endif

#if (LOG_DOMAIN_BT_PAIRING_INFO != 0)
  // Note to future codespace saver ... this is on by default for debug builds
  // Removing it will save ~2400 bytes but it is super useful for BT bringup debug!
  { "gapdb dump", command_gapdb_dump, 0 },
  { "sprf nuke", command_bt_sprf_nuke, 0 },
#if !RECOVERY_FW
  { "sprf sync", command_force_shared_prf_flush, 0},
#endif // !RECOVERY_FW
#endif

#if 0
  { "battui show", command_battery_ui_display, 3 },
  { "battui update", command_battery_ui_update, 3 },
  { "battui dismiss", command_battery_ui_dismiss, 0 },
#endif

  { "waste time", command_waste_time, 2 },
#if !defined(RECOVERY_FW)
  { "dump notif_pref_db", command_dump_notif_pref_db, 0 },
#endif

#if PERFORMANCE_TESTS
  { "perftest all line", command_perftest_line_all, 0 },
  { "perftest all text", command_perftest_text_all, 0 },
  { "perftest line", command_perftest_line, 2 },
  { "perftest text", command_perftest_text, 3 },
#endif
#endif // KEEP_NON_ESSENTIAL_COMMANDS

#if PLATFORM_SILK && !TARGET_QEMU
  { "accel samp", command_accel_num_samples, 1 },
  { "accel status", command_accel_status, 0 },
  { "accel selftest", command_accel_selftest, 0 },
  { "accel reset", command_accel_softreset, 0 },
#endif // PLATFORM_SILK
  { "vibe", command_vibe_ctl, 1 },

#if MEMFAULT
  { "mflt export", command_mflt_export, 0 },
  { "mflt heartbeat", command_mflt_heartbeat, 0 },
  { "mflt metrics_dump", command_mflt_metrics_dump, 0 },
  { "mflt device_info", command_mflt_device_info, 0 },
#endif  // MEMFAULT
};

#define NUM_PROMPT_COMMANDS ARRAY_LENGTH(s_prompt_commands)
