/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/bluetooth/ble_hid.h"

#include "comm/ble/gap_le_connection.h"

#include <bluetooth/hid_service.h>
#include <pbl/util/size.h>

#include <clar.h>

#include <string.h>

////////////////////////////////////////////////////////////////////////////////////////////////////
// Stubs & Fakes

#include "fake_new_timer.h"
#include "fake_pbl_malloc.h"
#include "fake_rtc.h"
#include "fake_system_task.h"

#include "stubs_bt_conn_mgr.h"
#include "stubs_bt_lock.h"
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_tick.h"

// The hold timeout ble_hid.c arms after every press, and the window it keeps
// retrying a release for after that.
#define TEST_HOLD_TIMEOUT_MS (10000)
#define TEST_RELEASE_GRACE_MS (3000)
#define TEST_RELEASE_RETRY_MS (30)

//! One entry per report that actually went out, so a test can assert both what
//! was sent and which report it was sent on.
typedef struct {
  BleHidReportPath path;
  uint16_t value;  //!< The report 1 bit mask, or the report 2 usage.
} SendRecord;

static SendRecord s_sends[64];
static int s_num_sends;

static bool s_hid_supported;
static bool s_bitmap_subscribed;
static bool s_usage_subscribed;
static bool s_send_succeeds;

static void prv_record_send(BleHidReportPath path, uint16_t value) {
  cl_assert(s_num_sends < (int)ARRAY_LENGTH(s_sends));
  s_sends[s_num_sends++] = (SendRecord){.path = path, .value = value};
}

bool bt_driver_is_hid_service_supported(void) {
  return s_hid_supported;
}

bool bt_driver_hid_service_is_subscribed(void) {
  return s_bitmap_subscribed;
}

bool bt_driver_hid_service_usage_is_subscribed(void) {
  return s_usage_subscribed;
}

//! @note The subscription check is not decoration: a notify to a peer that has
//! not subscribed to this report is refused by every real backend before it
//! reaches the wire (@see src/bluetooth-fw/nimble/hid_service.c), and the
//! release path branches on exactly that. A fake that sent anyway would let a
//! test pin a branch the hardware cannot reach.
bool bt_driver_hid_service_send_consumer_report(uint8_t bits) {
  if (!s_bitmap_subscribed) {
    return false;
  }
  prv_record_send(BleHidReportPathBitmap, bits);
  return s_send_succeeds;
}

bool bt_driver_hid_service_send_consumer_usage(uint16_t usage) {
  if (!s_usage_subscribed) {
    return false;
  }
  prv_record_send(BleHidReportPathUsage, usage);
  return s_send_succeeds;
}

//! No gateway, so ble_hid_set_low_latency() is a no-op here. The conn_mgr call
//! it would make is covered by test_bt_conn_mgr.
GAPLEConnection *gap_le_connection_get_gateway(void) {
  return NULL;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Helpers

//! @return The single ble_hid release timer, which is the only timer in this
//! test binary.
static TimerID prv_running_timer(void) {
  return stub_new_timer_get_next();
}

static void prv_fire_running_timer(void) {
  const TimerID timer = prv_running_timer();
  cl_assert(timer != TIMER_INVALID_ID);
  cl_assert_equal_b(true, stub_new_timer_fire(timer));
}

static void prv_assert_send(int index, BleHidReportPath path, uint16_t value) {
  cl_assert(index < s_num_sends);
  cl_assert_equal_i(path, s_sends[index].path);
  cl_assert_equal_i(value, s_sends[index].value);
}

//! Moves the clock the release deadline is kept on. Firing a timer does not,
//! so anything that cares about the deadline has to say so.
static void prv_advance_ms(uint32_t ms) {
  fake_rtc_increment_ticks(((RtcTicks)ms * RTC_TICKS_HZ) / 1000);
}

//! Fires the retry timer `count` times with the clock moving as it would on a
//! watch, and checks it is still armed each time round.
static void prv_run_retries(int count) {
  for (int i = 0; i < count; ++i) {
    cl_assert(prv_running_timer() != TIMER_INVALID_ID);
    cl_assert_equal_i(TEST_RELEASE_RETRY_MS, (int)stub_new_timer_timeout(prv_running_timer()));
    prv_advance_ms(TEST_RELEASE_RETRY_MS);
    prv_fire_running_timer();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup / teardown

void test_ble_hid__initialize(void) {
  memset(s_sends, 0, sizeof(s_sends));
  s_num_sends = 0;
  s_hid_supported = true;
  s_bitmap_subscribed = true;
  s_usage_subscribed = true;
  s_send_succeeds = true;
  // fake_system_task decrements this per queued callback and only gives it back
  // when the callback runs, so reset it rather than inherit the last test's.
  system_task_set_available_space(~(uint32_t)0);
  // Same for the clock: the release deadline is absolute, so a test that ran
  // it forward must not shorten the next test's window.
  fake_rtc_init(0 /* initial_ticks */, 0 /* initial_time */);

  ble_hid_init();
}

void test_ble_hid__cleanup(void) {
  // Drops the held usage and stops the timer. The timer and the mutex outlive
  // this on purpose -- ble_hid keeps them for the lifetime of the FW -- so the
  // fakes' own cleanup helpers must not run.
  ble_hid_deinit();
  fake_system_task_callbacks_cleanup();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Tests

void test_ble_hid__press_then_release_on_the_usage_path(void) {
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  // The send is queued to KernelBG, not done in the caller's context.
  cl_assert_equal_i(0, s_num_sends);

  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(1, s_num_sends);
  prv_assert_send(0, BleHidReportPathUsage, BLE_HID_USAGE_VOLUME_UP);

  cl_assert_equal_i(S_SUCCESS, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_UP));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(2, s_num_sends);
  prv_assert_send(1, BleHidReportPathUsage, 0);
}

void test_ble_hid__press_then_release_on_the_bitmap_path(void) {
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathBitmap));
  fake_system_task_callbacks_invoke_pending();
  // The bitmap path sends the bit, not the usage.
  cl_assert_equal_i(1, s_num_sends);
  prv_assert_send(0, BleHidReportPathBitmap, BLE_HID_CONSUMER_VOLUME_UP);

  cl_assert_equal_i(S_SUCCESS, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_UP));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(2, s_num_sends);
  prv_assert_send(1, BleHidReportPathBitmap, 0);
}

void test_ble_hid__release_follows_the_path_the_press_used(void) {
  // Both reports are subscribed, so only the stored path can decide this one.
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(S_SUCCESS, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_UP));
  fake_system_task_callbacks_invoke_pending();

  cl_assert_equal_i(2, s_num_sends);
  prv_assert_send(1, BleHidReportPathUsage, 0);
}

void test_ble_hid__a_tap_sends_the_press_before_the_release(void) {
  // What the Camera app does: press, then release a few ms later, both before
  // KernelBG has run either of them.
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  cl_assert_equal_i(S_SUCCESS, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_UP));

  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(2, s_num_sends);
  prv_assert_send(0, BleHidReportPathUsage, BLE_HID_USAGE_VOLUME_UP);
  prv_assert_send(1, BleHidReportPathUsage, 0);
}

void test_ble_hid__press_while_another_usage_is_held_is_busy(void) {
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  cl_assert_equal_i(E_BUSY,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_DOWN, BleHidReportPathUsage));

  fake_system_task_callbacks_invoke_pending();
  // Only the first press was ever queued.
  cl_assert_equal_i(1, s_num_sends);
  prv_assert_send(0, BleHidReportPathUsage, BLE_HID_USAGE_VOLUME_UP);
}

void test_ble_hid__same_usage_on_another_path_is_busy(void) {
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  // Same usage, other report: still a second key going down, so still E_BUSY.
  cl_assert_equal_i(E_BUSY,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathBitmap));

  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(1, s_num_sends);
}

void test_ble_hid__repeat_press_is_idempotent_and_does_not_rearm_the_timer(void) {
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  const int starts = s_num_new_timer_start_calls;

  cl_assert_equal_i(S_NO_ACTION_REQUIRED,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  // The safety net stays anchored to the first press: re-arming it would let a
  // repeat hold the key open indefinitely.
  cl_assert_equal_i(starts, s_num_new_timer_start_calls);

  fake_system_task_callbacks_invoke_pending();
  // And the repeat did not queue a second report either.
  cl_assert_equal_i(1, s_num_sends);
}

void test_ble_hid__release_without_a_press_is_a_no_op(void) {
  cl_assert_equal_i(S_NO_ACTION_REQUIRED, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_UP));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(0, s_num_sends);
}

void test_ble_hid__release_of_another_usage_is_rejected(void) {
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(1, s_num_sends);

  cl_assert_equal_i(E_INVALID_ARGUMENT, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_DOWN));
  fake_system_task_callbacks_invoke_pending();
  // Nothing was lifted, so the key is still down and still releasable.
  cl_assert_equal_i(1, s_num_sends);

  cl_assert_equal_i(S_SUCCESS, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_UP));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(2, s_num_sends);
  prv_assert_send(1, BleHidReportPathUsage, 0);
}

void test_ble_hid__a_failed_send_leaves_nothing_held(void) {
  s_send_succeeds = false;
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(1, s_num_sends);

  // Nothing went out, so nothing is held: the next press must not see E_BUSY.
  s_send_succeeds = true;
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_DOWN, BleHidReportPathUsage));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(2, s_num_sends);
  prv_assert_send(1, BleHidReportPathUsage, BLE_HID_USAGE_VOLUME_DOWN);
}

void test_ble_hid__hold_timeout_forces_a_release(void) {
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(1, s_num_sends);

  cl_assert_equal_i(TEST_HOLD_TIMEOUT_MS, (int)stub_new_timer_timeout(prv_running_timer()));
  prv_fire_running_timer();
  // The timer runs on NewTimer, so it hands the send to KernelBG rather than
  // talking to the driver itself.
  cl_assert_equal_i(1, s_num_sends);

  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(2, s_num_sends);
  prv_assert_send(1, BleHidReportPathUsage, 0);

  // Nothing is held any more.
  cl_assert_equal_i(S_NO_ACTION_REQUIRED, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_UP));
}

void test_ble_hid__a_stale_release_does_not_lift_a_later_press(void) {
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(1, s_num_sends);

  // Two releases end up queued for the same press: the hold timer's and the
  // caller's. That is the case the generation guard exists for.
  prv_fire_running_timer();
  cl_assert_equal_i(S_SUCCESS, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_UP));
  cl_assert_equal_i(2, (int)fake_system_task_count_callbacks());

  // Run only the first. It lifts the key and clears the held state.
  fake_system_task_callbacks_invoke(1);
  cl_assert_equal_i(2, s_num_sends);
  prv_assert_send(1, BleHidReportPathUsage, 0);

  // A new press starts a new generation while the second release is still
  // queued behind it.
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_DOWN, BleHidReportPathUsage));

  fake_system_task_callbacks_invoke_pending();
  // The leftover release must be dropped, and the new press must go out. If it
  // were not dropped it would send a lift, and the press behind it would then
  // read as stale and send nothing at all.
  cl_assert_equal_i(3, s_num_sends);
  prv_assert_send(2, BleHidReportPathUsage, BLE_HID_USAGE_VOLUME_DOWN);

  // And the new press is still held, i.e. the stale release did not clear it.
  cl_assert_equal_i(S_SUCCESS, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_DOWN));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(4, s_num_sends);
  prv_assert_send(3, BleHidReportPathUsage, 0);
}

void test_ble_hid__a_press_kernel_bg_will_not_take_holds_nothing(void) {
  // Below the margin prv_queue_to_kernel_bg() insists on off the app task.
  system_task_set_available_space(5);
  cl_assert_equal_i(E_INTERNAL,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  cl_assert_equal_i(0, (int)fake_system_task_count_callbacks());

  // Nothing was queued, so nothing is held and the next press is not refused.
  system_task_set_available_space(~(uint32_t)0);
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_DOWN, BleHidReportPathUsage));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(1, s_num_sends);
  prv_assert_send(0, BleHidReportPathUsage, BLE_HID_USAGE_VOLUME_DOWN);
}

void test_ble_hid__a_release_kernel_bg_will_not_take_is_retried(void) {
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(1, s_num_sends);

  system_task_set_available_space(5);
  cl_assert_equal_i(E_INTERNAL, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_UP));
  // The hold timer still covers it, still at its full timeout.
  cl_assert_equal_i(TEST_HOLD_TIMEOUT_MS, (int)stub_new_timer_timeout(prv_running_timer()));

  // It fires, cannot queue either, and re-arms itself as a short retry.
  prv_fire_running_timer();
  cl_assert_equal_i(1, s_num_sends);
  cl_assert(prv_running_timer() != TIMER_INVALID_ID);
  cl_assert_equal_i(TEST_RELEASE_RETRY_MS, (int)stub_new_timer_timeout(prv_running_timer()));

  system_task_set_available_space(~(uint32_t)0);
  prv_advance_ms(TEST_RELEASE_RETRY_MS);
  prv_fire_running_timer();
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(2, s_num_sends);
  prv_assert_send(1, BleHidReportPathUsage, 0);
}

void test_ble_hid__a_release_the_peer_will_not_take_is_retried(void) {
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(1, s_num_sends);

  // Still subscribed, but the notify fails -- an exhausted mbuf pool, say. This
  // is the branch the field hits; the queue-refusal one below is rarer.
  s_send_succeeds = false;
  cl_assert_equal_i(S_SUCCESS, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_UP));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(2, s_num_sends);
  prv_assert_send(1, BleHidReportPathUsage, 0);

  // The lift did not reach the peer, so the key is still down and a retry is
  // armed on the same timer.
  cl_assert(prv_running_timer() != TIMER_INVALID_ID);
  cl_assert_equal_i(TEST_RELEASE_RETRY_MS, (int)stub_new_timer_timeout(prv_running_timer()));

  s_send_succeeds = true;
  prv_advance_ms(TEST_RELEASE_RETRY_MS);
  prv_fire_running_timer();
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(3, s_num_sends);
  prv_assert_send(2, BleHidReportPathUsage, 0);
  cl_assert_equal_i(S_NO_ACTION_REQUIRED, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_UP));
}

void test_ble_hid__a_release_is_retried_for_longer_than_a_kernel_bg_backlog(void) {
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(1, s_num_sends);

  // KernelBG stays below the margin throughout: backed up, not gone. One BLE
  // store transaction on that queue costs far more than the five fires -- about
  // 120 ms -- that the retries used to be budgeted for, so a bound counted in
  // attempts abandoned the key on an ordinary backlog.
  system_task_set_available_space(5);
  cl_assert_equal_i(E_INTERNAL, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_UP));
  prv_fire_running_timer();
  prv_run_retries(60);

  // Still trying, well past the old budget, and nothing has been sent.
  cl_assert(prv_running_timer() != TIMER_INVALID_ID);
  cl_assert_equal_i(1, s_num_sends);

  // The backlog clears and the lift finally goes out.
  system_task_set_available_space(~(uint32_t)0);
  prv_fire_running_timer();
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(2, s_num_sends);
  prv_assert_send(1, BleHidReportPathUsage, 0);
  cl_assert_equal_i(S_NO_ACTION_REQUIRED, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_UP));
}

void test_ble_hid__the_release_gives_up_once_the_deadline_passes(void) {
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(1, s_num_sends);

  system_task_set_available_space(5);
  cl_assert_equal_i(E_INTERNAL, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_UP));

  // The deadline is absolute and taken from the press: the hold timeout plus
  // the grace, whenever within it the release was attempted.
  prv_advance_ms(TEST_HOLD_TIMEOUT_MS + TEST_RELEASE_GRACE_MS + 1);
  prv_fire_running_timer();

  // It stops rather than retrying for ever, and it sends no lift it never
  // managed: the key really is still down on the phone, and the ERR log is the
  // only honest report of that.
  cl_assert_equal_i(TIMER_INVALID_ID, prv_running_timer());
  cl_assert_equal_i(1, s_num_sends);

  // Giving up drops the held state so the service is not wedged, and the press
  // that is now accepted is what can clear the stuck key: its own release
  // lifts whatever the phone still has down.
  system_task_set_available_space(~(uint32_t)0);
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(2, s_num_sends);
  prv_assert_send(1, BleHidReportPathUsage, BLE_HID_USAGE_VOLUME_UP);

  cl_assert_equal_i(S_SUCCESS, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_UP));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(3, s_num_sends);
  prv_assert_send(2, BleHidReportPathUsage, 0);
}

void test_ble_hid__an_unsubscribe_drops_the_held_usage_without_a_lift(void) {
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(1, s_num_sends);

  s_usage_subscribed = false;
  const BTDeviceInternal device = {};
  bt_driver_cb_hid_service_update_subscription(&device, false);
  // Runs on the BT host task, so it queues rather than sends.
  cl_assert_equal_i(1, s_num_sends);

  fake_system_task_callbacks_invoke_pending();
  // No lift goes out and none is needed: the notify would be refused anyway,
  // and a HOGP host drops every key it knows about when the subscription ends.
  // The state is cleared instead, and this is the branch the field takes every
  // time a phone walks away with the shutter down.
  cl_assert_equal_i(1, s_num_sends);
  cl_assert_equal_i(S_NO_ACTION_REQUIRED, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_UP));

  // Nothing is retried either: the leftover hold timer finds nothing held.
  prv_advance_ms(TEST_HOLD_TIMEOUT_MS);
  prv_fire_running_timer();
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(1, s_num_sends);
  cl_assert_equal_i(TIMER_INVALID_ID, prv_running_timer());
}

void test_ble_hid__a_press_queued_before_a_later_one_does_not_disturb_it(void) {
  // A presser on the app task waits inside system_task_add_callback() for a
  // slot on its own eight-deep queue, so the held state is published long
  // before the send is queued.
  fake_system_task_defer_next_add();
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  cl_assert_equal_i(0, (int)fake_system_task_count_callbacks());

  // While it waits, the hold timer lifts the key...
  prv_advance_ms(TEST_HOLD_TIMEOUT_MS);
  prv_fire_running_timer();
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(1, s_num_sends);
  prv_assert_send(0, BleHidReportPathUsage, 0);

  // ...and the same usage goes down again on the same report, so nothing but
  // the generation tells the two presses apart.
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(2, s_num_sends);
  prv_assert_send(1, BleHidReportPathUsage, BLE_HID_USAGE_VOLUME_UP);

  // The first presser finally gets its slot. Its send is for a press that is
  // over, so nothing goes out: a repeat would put a key down that the press it
  // belonged to can no longer lift, and a failed one would clear a held state
  // that is not its own -- after which the owner's release is a no-op.
  fake_system_task_flush_deferred();
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(2, s_num_sends);

  // The second press is still held and still releasable.
  cl_assert_equal_i(S_SUCCESS, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_UP));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(3, s_num_sends);
  prv_assert_send(2, BleHidReportPathUsage, 0);
}

void test_ble_hid__unsubscribe_of_the_other_report_leaves_the_key_alone(void) {
  cl_assert_equal_i(S_SUCCESS,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  fake_system_task_callbacks_invoke_pending();

  // Report 1 goes away; the press was on report 2, which is still subscribed.
  s_bitmap_subscribed = false;
  const BTDeviceInternal device = {};
  bt_driver_cb_hid_service_update_subscription(&device, false);
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(1, s_num_sends);

  cl_assert_equal_i(S_SUCCESS, ble_hid_consumer_release(BLE_HID_USAGE_VOLUME_UP));
}

void test_ble_hid__is_ready_follows_the_path_it_is_asked_about(void) {
  s_bitmap_subscribed = true;
  s_usage_subscribed = false;
  cl_assert_equal_b(true, ble_hid_is_ready(BleHidReportPathBitmap));
  cl_assert_equal_b(false, ble_hid_is_ready(BleHidReportPathUsage));

  s_bitmap_subscribed = false;
  s_usage_subscribed = true;
  cl_assert_equal_b(false, ble_hid_is_ready(BleHidReportPathBitmap));
  cl_assert_equal_b(true, ble_hid_is_ready(BleHidReportPathUsage));

  // A driver without the service is never ready, whatever is subscribed.
  s_hid_supported = false;
  cl_assert_equal_b(false, ble_hid_is_ready(BleHidReportPathBitmap));
  cl_assert_equal_b(false, ble_hid_is_ready(BleHidReportPathUsage));
}

void test_ble_hid__press_on_an_unsubscribed_path_is_refused(void) {
  s_usage_subscribed = false;
  cl_assert_equal_i(E_INVALID_OPERATION,
                    ble_hid_consumer_press(BLE_HID_USAGE_VOLUME_UP, BleHidReportPathUsage));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(0, s_num_sends);
}

void test_ble_hid__bitmap_path_refuses_a_usage_it_cannot_carry(void) {
  // AC Back, which report 1 does not declare. No silent reroute onto report 2.
  cl_assert_equal_i(E_INVALID_ARGUMENT, ble_hid_consumer_press(0x224, BleHidReportPathBitmap));
  cl_assert_equal_i(S_SUCCESS, ble_hid_consumer_press(0x224, BleHidReportPathUsage));

  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(1, s_num_sends);
  prv_assert_send(0, BleHidReportPathUsage, 0x224);
}

void test_ble_hid__out_of_range_usages_are_refused(void) {
  cl_assert_equal_i(E_INVALID_ARGUMENT, ble_hid_consumer_press(0, BleHidReportPathUsage));
  cl_assert_equal_i(E_INVALID_ARGUMENT,
                    ble_hid_consumer_press(BLE_HID_USAGE_MAX + 1, BleHidReportPathUsage));
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(0, s_num_sends);
}
