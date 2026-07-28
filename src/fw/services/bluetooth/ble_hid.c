/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/bluetooth/ble_hid.h"

#include "comm/ble/gap_le_connection.h"
#include "comm/bt_conn_mgr.h"
#include "comm/bt_lock.h"
#include "kernel/pebble_tasks.h"
#include "pbl/os/mutex.h"
#include "pbl/os/tick.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/system_task.h"
#include <pbl/drivers/rtc.h>
#include <pbl/logging/logging.h>

#include <bluetooth/hid_service.h>
#include <bluetooth/responsiveness.h>

PBL_LOG_MODULE_DECLARE(service_bluetooth, CONFIG_SERVICE_BLUETOOTH_LOG_LEVEL);

#ifdef CONFIG_BT_HID_REMOTE

// A lost release leaves the key held down on the phone, which is not harmless:
// a held Volume Up in the iOS Camera app starts a QuickTake video. Release for
// the caller if it never gets round to it. 10 s is long enough not to cut a
// legitimate hold short -- a volume ramp or a fast-forward runs a few seconds --
// and short enough that a stuck key reads as a glitch rather than a dead phone.
#define BLE_HID_HOLD_TIMEOUT_MS (10000)

// How often a release that could not go out is tried again.
#define BLE_HID_RELEASE_RETRY_MS (30)

// How long past the hold timeout those retries keep going. The budget is time
// rather than a number of attempts: what a retry waits out is a KernelBG
// backlog or an exhausted mbuf pool, and neither is measured in attempts.
// 3 s is what system_task_add_callback() itself waits on this very queue before
// it calls the system broken and reboots, so giving up sooner would abandon a
// key on a backlog the kernel still considers survivable. KernelBG now also
// carries the BLE store's settings-file work -- a bonding delete is a full-file
// scan, a transaction per CCCD record and a shared-PRF erase -- so backlogs of
// that order are ordinary here.
#define BLE_HID_RELEASE_GRACE_MS (3000)

//! Free KernelBG slots we insist on before queueing work. From anything but the
//! app task, system_task_add_callback() blocks up to 3 s on a full queue and
//! then reboots, and from KernelBG itself it can never drain.
#define BLE_HID_KERNEL_BG_QUEUE_MARGIN (10)

//! How much of s_press_generation rides in a packed cb_data. The same width on
//! all three layouts below, so a generation compares the same way whichever one
//! it travelled in. A million presses between two wraps, against callbacks that
//! live for milliseconds.
#define BLE_HID_GENERATION_MASK (0xFFFFFU)

//! Timer cb_data layout. The purpose tells a forced release from a retry, and
//! the press generation makes a fire left over from an earlier press
//! detectable: new_timer_start() cannot recall one that is already in flight.
#define BLE_HID_TIMER_PURPOSE_MASK (0x1U)
#define BLE_HID_TIMER_PURPOSE_HOLD (0x0U)
#define BLE_HID_TIMER_PURPOSE_RETRY (0x1U)
#define BLE_HID_TIMER_GENERATION_SHIFT (1U)

//! Press cb_data layout. The usage and the report ride along rather than being
//! read back from the statics, so a press queued while an earlier one is still
//! pending cannot overwrite them, and the generation is the only thing that
//! tells two presses of the same usage on the same report apart. The release
//! layout is the generation on its own, so it needs no shifts of its own.
#define BLE_HID_PRESS_USAGE_MASK (0x3FFU)
#define BLE_HID_PRESS_PATH_SHIFT (10U)
#define BLE_HID_PRESS_PATH_MASK (0x1U)
#define BLE_HID_PRESS_GENERATION_SHIFT (11U)

_Static_assert(BLE_HID_USAGE_MAX <= BLE_HID_PRESS_USAGE_MASK,
               "a usage no longer fits a packed press");
_Static_assert(BleHidReportPathUsage <= BLE_HID_PRESS_PATH_MASK,
               "a path no longer fits a packed press");
_Static_assert(((uint64_t)BLE_HID_GENERATION_MASK << BLE_HID_PRESS_GENERATION_SHIFT) <= UINT32_MAX,
               "a packed press overflows a 32-bit cb_data");

static TimerID s_release_timer = TIMER_INVALID_ID;
//! Usage currently held down (0 if none) and the report it went out on.
//! @note s_held_usage is the publication flag for all of this state, so a press
//! writes it last and a release clears it last. Everything else -- the path, the
//! generation -- must already be in place when a reader sees it non-zero.
//! Volatile because App, KernelBG, NewTimer and the BT host task all touch these
//! and the ordering, not just the value, is what makes the protocol work.
static volatile uint16_t s_held_usage;
static volatile BleHidReportPath s_held_path;
//! Bumped by every press before it publishes a held state. Read on the NewTimer
//! task and on KernelBG; a single aligned word, so a reader never sees a torn
//! value. The increment is a read-modify-write and ble_hid_consumer_press() is
//! called from the app task (the Camera app) and from KernelBG (the prompt
//! commands), so it is done under s_press_mutex rather than left to volatile.
static volatile uint32_t s_press_generation;
//! When the held usage has to be back up by, come what may: the hold timeout
//! plus the retry grace, counted from the press. Written with the held state
//! and under the same lock, so it belongs to exactly one generation and no
//! reader needs a second word to work out which. Truncated rtc ticks: a 64-bit
//! RtcTicks could be read torn by the four tasks that look at this.
static volatile uint32_t s_release_deadline;

//! Serialises the claim-and-publish block of ble_hid_consumer_press(): the
//! s_held_usage test, the generation bump, the writes of s_held_path and
//! s_held_usage, and arming the hold timer. Without it two callers can both find
//! s_held_usage == 0, lose one increment, both arm the one shared timer and both
//! queue a press, and only one of the two can ever be released.
//! @note Held by pressers only. Deliberately not taken by prv_release_timer_cb()
//! or the KernelBG callbacks: NewTimer runs at max priority and holds the timer
//! manager's lock around a start, and the press path takes that lock while
//! holding this one. Everything outside the publish block relies on the
//! generation guard instead, which is what it is there for.
//! @note Created by ble_hid_init(), which bluetooth_ctl.c runs before
//! bt_driver_start() -- the point at which the GATT service becomes
//! subscribable -- so nothing that reaches this lock can find it NULL.
static PebbleMutex *s_press_mutex;

static void prv_release_timer_cb(void *packed);
static void prv_release_system_task_cb(void *packed);

//! @note The generation is a parameter rather than a read of s_press_generation,
//! so a caller that has already validated one cannot accidentally pack a newer.
static void *prv_pack_timer(uint32_t generation, unsigned purpose) {
  return (void *)(uintptr_t)(
      ((generation & BLE_HID_GENERATION_MASK) << BLE_HID_TIMER_GENERATION_SHIFT) |
      (purpose & BLE_HID_TIMER_PURPOSE_MASK));
}

//! @note The generation is all a release needs to carry, and it needs it for the
//! same reason a timer does one queue earlier: a release sits on the KernelBG
//! queue behind other work, and by the time it runs the press it was queued for
//! may be over and another one under way.
static void *prv_pack_release(uint32_t generation) {
  return (void *)(uintptr_t)(generation & BLE_HID_GENERATION_MASK);
}

static void *prv_pack_press(uint32_t generation, uint16_t usage, BleHidReportPath path) {
  return (void *)(uintptr_t)(
      ((generation & BLE_HID_GENERATION_MASK) << BLE_HID_PRESS_GENERATION_SHIFT) |
      (((unsigned)path & BLE_HID_PRESS_PATH_MASK) << BLE_HID_PRESS_PATH_SHIFT) |
      ((unsigned)usage & BLE_HID_PRESS_USAGE_MASK));
}

//! @return True if `generation` is still the press everything is about.
static bool prv_generation_is_current(uint32_t generation) {
  return (generation == (s_press_generation & BLE_HID_GENERATION_MASK));
}

//! The clock the release deadline is kept on. Truncated to 32 bits on purpose:
//! every comparison below is a signed difference, so the wrap is harmless, and
//! at RTC_TICKS_HZ half the range is days against a budget of seconds.
static uint32_t prv_now_ticks(void) {
  return (uint32_t)rtc_get_ticks();
}

static bool prv_deadline_passed(uint32_t deadline) {
  return ((int32_t)(prv_now_ticks() - deadline) >= 0);
}

//! Hands work to KernelBG without ever waiting on it, except from the app task.
//! @return True if the work was queued.
//! @note The app has its own eight-deep queue and is meant to wait on it, which
//! is what system_task_add_callback() does for it by design. Every other caller
//! here is somewhere waiting is not allowed: NewTimer drives every other timer
//! in the system, the BT host task holds ble_hs_mutex, and KernelBG cannot drain
//! its own queue. For those, the margin keeps the BLE store's settings-file work
//! from being crowded out, and the send itself refuses to block -- the margin
//! alone cannot promise the slot is still there when we send.
//! @note KernelBG now also carries that store work, so a shutter press from the
//! Camera app can sit in the app's syscall waiting for a flash transaction, and
//! the UI sits with it. That coupling did not exist before the store deferral.
static bool prv_queue_to_kernel_bg(SystemTaskEventCallback cb, void *data) {
  if (pebble_task_get_current() == PebbleTask_App) {
    return system_task_add_callback(cb, data);
  }
  if (system_task_get_available_space() <= BLE_HID_KERNEL_BG_QUEUE_MARGIN) {
    return false;
  }
  return system_task_add_callback_nonblocking(cb, data);
}

//! Rejects a path that cannot carry `usage`, rather than rerouting it.
static status_t prv_check_path(uint16_t usage, BleHidReportPath path) {
  switch (path) {
    case BleHidReportPathBitmap:
      // No silent fallback to report 2: it would hide which form the usage went
      // out as, and telling the two apart is the only reason the path is a
      // parameter at all.
      return (ble_hid_consumer_usage_to_bit(usage) != 0) ? S_SUCCESS : E_INVALID_ARGUMENT;
    case BleHidReportPathUsage:
      return S_SUCCESS;
    default:
      return E_INVALID_ARGUMENT;
  }
}

//! The only place a report goes out. `usage` 0 releases whatever is held.
static bool prv_send(uint16_t usage, BleHidReportPath path) {
  if (path == BleHidReportPathBitmap) {
    return bt_driver_hid_service_send_consumer_report(ble_hid_consumer_usage_to_bit(usage));
  }
  return bt_driver_hid_service_send_consumer_usage(usage);
}

//! Asks the driver rather than caching: a subscription can end with the link
//! already gone, and a mirror of it here would then be stale in exactly the
//! case that matters.
static bool prv_path_is_subscribed(BleHidReportPath path) {
  switch (path) {
    case BleHidReportPathBitmap:
      return bt_driver_hid_service_is_subscribed();
    case BleHidReportPathUsage:
      return bt_driver_hid_service_usage_is_subscribed();
    default:
      return false;
  }
}

//! Re-arms the release timer, or gives up and lets a new press through.
//! @note `generation` must be the one the caller has just validated: both
//! give-up exits clear s_held_usage, and doing that for a press that has moved
//! on would drop the flag a newer press is relying on. Nothing here holds
//! s_press_mutex, so the guard is re-run against the one action that outlives
//! this call.
//! @note Runs on KernelBG and on NewTimer, neither of which may ask the driver
//! whether the peer is still subscribed. It does not need to: the retry ends up
//! back in prv_release_system_task_cb(), which does ask, and clears the state
//! outright once the peer has gone. So the loop only keeps going while the peer
//! is still there or while KernelBG is too backed up to say.
static void prv_schedule_release_retry(uint32_t generation) {
  if (!prv_generation_is_current(generation)) {
    return;
  }
  if (prv_deadline_passed(s_release_deadline)) {
    // Out of budget with the key still down on the phone. Nothing here can lift
    // it -- the whole window was spent failing to -- so say so at ERR and drop
    // the state rather than imply it went up. Forgetting is the recovery: the
    // next press is accepted, and its own release lifts the stuck key.
    PBL_LOG_ERR("BLE HID: usage 0x%03x is still down after %u ms, giving up on the release",
                (unsigned)s_held_usage,
                (unsigned)(BLE_HID_HOLD_TIMEOUT_MS + BLE_HID_RELEASE_GRACE_MS));
    s_held_usage = 0;
    return;
  }
  // Again, right up against the start. A press can land anywhere above -- the
  // checks read state it writes -- and it arms the one shared timer with its own
  // hold; starting a retry for the older generation on top of that would leave
  // the new press with no safety net at all. Nothing to clear on this exit: that
  // press could only have been accepted with s_held_usage already 0, so the held
  // state is its own now.
  if (!prv_generation_is_current(generation)) {
    return;
  }
  if ((s_release_timer == TIMER_INVALID_ID) ||
      !new_timer_start(s_release_timer, BLE_HID_RELEASE_RETRY_MS, prv_release_timer_cb,
                       prv_pack_timer(generation, BLE_HID_TIMER_PURPOSE_RETRY), 0 /* flags */)) {
    PBL_LOG_ERR("BLE HID: no release timer, usage 0x%03x is left down", (unsigned)s_held_usage);
    s_held_usage = 0;
  }
}

static void prv_release_system_task_cb(void *packed) {
  const uint32_t generation = (uint32_t)(uintptr_t)packed & BLE_HID_GENERATION_MASK;

  if (!prv_generation_is_current(generation)) {
    // Queued for a press that is already over. Two releases can be in flight at
    // once -- the tap timer and an unsubscribe both queue one -- and without
    // this the second would lift the press that came after them and leave the
    // flag clear, so the press that follows *it* reads as stale and sends
    // nothing. The shutter would then silently do nothing.
    return;
  }
  const uint16_t usage = s_held_usage;
  if (usage == 0) {
    // Already released, e.g. by the app just before the hold timer fired.
    return;
  }
  const BleHidReportPath path = s_held_path;
  if (prv_send(0, path) || !prv_path_is_subscribed(path)) {
    // Released, or the peer is gone and whatever key state it had is moot.
    s_held_usage = 0;
    return;
  }
  // The send failed with the peer still there (out of mbufs, notify error).
  PBL_LOG_WRN("BLE HID: release of 0x%03x failed, retrying", (unsigned)usage);
  prv_schedule_release_retry(generation);
}

//! @note Runs on the NewTimer task, which is max priority and drives every
//! other timer, so never talk to the BT stack from here: bt_lock can be held
//! by a lower-priority task for a long time. Hand off to KernelBG instead.
static void prv_release_timer_cb(void *packed) {
  const uintptr_t data = (uintptr_t)packed;
  const unsigned purpose = data & BLE_HID_TIMER_PURPOSE_MASK;
  const uint32_t generation = (data >> BLE_HID_TIMER_GENERATION_SHIFT) & BLE_HID_GENERATION_MASK;

  if (!prv_generation_is_current(generation)) {
    // Armed for a press that has since been replaced. Acting now would release
    // the press that replaced it.
    // @note This rests on task_timer.c dropping the manager mutex before it
    // reads cb_data, and on NewTimer running at configMAX_PRIORITIES - 1.
    // Together they mean nothing can slip a new_timer_start() into that window
    // and swap this fire's packed generation for a newer one, which is the only
    // way a stale fire could read as current.
    return;
  }
  if (s_held_usage == 0) {
    // Released normally; nothing left to force. Not stopping the timer on a
    // normal release is deliberate, but not because the stop would block: it
    // does not. task_timer_stop() takes the manager mutex, unlinks the timer
    // and returns !executing, and new_timer.h documents the return value the
    // same way. The reason is that a release is asynchronous and may retry on
    // this same timer, so the safety net has to stay armed until the state is
    // actually clear. A leftover fire lands here and costs nothing.
    return;
  }
  if (purpose == BLE_HID_TIMER_PURPOSE_HOLD) {
    PBL_LOG_WRN("BLE HID: usage 0x%03x held for %u ms, forcing the release",
                (unsigned)s_held_usage, (unsigned)BLE_HID_HOLD_TIMEOUT_MS);
  }
  // Never waits: stalling here would stall every timer in the system. The
  // generation rides along so a release that queues now but runs after the next
  // press cannot lift it.
  if (prv_queue_to_kernel_bg(prv_release_system_task_cb, prv_pack_release(generation))) {
    return;
  }
  PBL_LOG_WRN("BLE HID: KernelBG would not take the release, retrying");
  prv_schedule_release_retry(generation);
}

//! Arms the safety net that releases a usage the caller forgot about.
//! @note Must be called with s_held_usage and s_press_generation already set.
//! @note Deliberately no TIMER_START_FLAG_FAIL_IF_EXECUTING. It would close the
//! cb_data window prv_release_timer_cb() documents, but it also refuses the
//! start whenever a leftover fire happens to be running, which would turn a
//! benign overlap into a failed shutter press. The generation guard already
//! covers the overlap, and prv_schedule_release_retry() re-arms from inside the
//! callback, where the flag would refuse every time.
static bool prv_arm_hold_timer(uint32_t generation) {
  return (s_release_timer != TIMER_INVALID_ID) &&
         new_timer_start(s_release_timer, BLE_HID_HOLD_TIMEOUT_MS, prv_release_timer_cb,
                         prv_pack_timer(generation, BLE_HID_TIMER_PURPOSE_HOLD), 0 /* flags */);
}

static void prv_press_system_task_cb(void *packed) {
  const uintptr_t data = (uintptr_t)packed;
  const uint16_t usage = (uint16_t)(data & BLE_HID_PRESS_USAGE_MASK);
  const BleHidReportPath path =
      (BleHidReportPath)((data >> BLE_HID_PRESS_PATH_SHIFT) & BLE_HID_PRESS_PATH_MASK);
  const uint32_t generation = (data >> BLE_HID_PRESS_GENERATION_SHIFT) & BLE_HID_GENERATION_MASK;

  // The generation, not just the usage and the path: a presser on the app task
  // waits inside system_task_add_callback() for as long as its queue is full,
  // and by the time it gets a slot the key it published can have been released
  // and the same usage pressed again on the same report. The usage and path
  // alone would match that, so this would send a second report for a press that
  // is over and, on a send failure, clear a held state belonging to the press
  // that replaced it -- whose own release would then be a no-op.
  if (!prv_generation_is_current(generation) || (s_held_usage != usage) ||
      (s_held_path != path)) {
    PBL_LOG_DBG("BLE HID: press of 0x%03x is stale, not sending", (unsigned)usage);
    return;
  }
  PBL_LOG_DBG("BLE HID: press 0x%03x on report %u", (unsigned)usage,
              (path == BleHidReportPathBitmap) ? 1u : 2u);
  if (!prv_send(usage, path)) {
    // Nothing went out, so nothing is held: a later press must not see E_BUSY,
    // and a release must not send a lift for a key that never went down. Only
    // if this press is still the current one, for the reason above.
    if (prv_generation_is_current(generation)) {
      s_held_usage = 0;
    }
  }
}

bool ble_hid_is_ready(BleHidReportPath path) {
  return (bt_driver_is_hid_service_supported() && prv_path_is_subscribed(path));
}

status_t ble_hid_consumer_press(uint16_t usage, BleHidReportPath path) {
  if ((usage == 0) || (usage > BLE_HID_USAGE_MAX)) {
    return E_INVALID_ARGUMENT;
  }
  const status_t rv = prv_check_path(usage, path);
  if (FAILED(rv)) {
    return rv;
  }
  if (!bt_driver_is_hid_service_supported() || !prv_path_is_subscribed(path)) {
    return E_INVALID_OPERATION;
  }
  // Re-arm the low-latency request, which expires after
  // MIN_LATENCY_MODE_TIMEOUT_HID_SECS. Done here rather than in the app so the
  // prompt commands get it too.
  ble_hid_set_low_latency(true);

  // Claiming the held state and publishing it has to be one step: the app task
  // and KernelBG both press, and two callers that each find s_held_usage == 0
  // would otherwise both go on to publish.
  mutex_lock(s_press_mutex);

  if (s_held_usage != 0) {
    const bool same_key = (s_held_usage == usage) && (s_held_path == path);
    mutex_unlock(s_press_mutex);
    // Report Count is 1 on both reports, so only one usage can be held.
    // Already down means the safety net stays anchored to the first press:
    // pushing it out would let a repeat hold the key indefinitely, and there is
    // one shared timer, so re-arming it would silently cancel a release retry.
    return same_key ? S_NO_ACTION_REQUIRED : E_BUSY;
  }

  // Publish the held state here, in the caller's context, before queueing the
  // send. A caller that taps -- presses and releases a few ms later -- has to
  // see it, and the KernelBG callback has not necessarily run by then.
  // s_held_usage goes last: it is what every reader keys off, and NewTimer runs
  // at max priority and can cut in between any two of these. A leftover hold
  // timer that saw the new generation but the old s_held_usage would queue a
  // forced release against the press that is only just starting, and one that
  // saw the new s_held_usage but the old s_held_path would release the wrong
  // report. The deadline goes with the generation for the same reason: nothing
  // reads it without first matching a generation, and nothing can be matching
  // this one until the timer below is armed.
  // Masked on the way in, so the counter itself never holds anything a packed
  // cb_data cannot carry and the comparisons below match what went out.
  const uint32_t generation = (s_press_generation + 1) & BLE_HID_GENERATION_MASK;
  s_press_generation = generation;
  s_release_deadline =
      prv_now_ticks() +
      (uint32_t)milliseconds_to_ticks(BLE_HID_HOLD_TIMEOUT_MS + BLE_HID_RELEASE_GRACE_MS);
  s_held_path = path;
  s_held_usage = usage;
  const bool armed = prv_arm_hold_timer(generation);
  if (!armed) {
    // Nothing has gone out yet, so refusing costs nothing, and without the
    // safety net a lost release holds the key until the link drops -- a held
    // Volume Up is a video rather than a photo.
    s_held_usage = 0;
  }

  mutex_unlock(s_press_mutex);

  if (!armed) {
    PBL_LOG_ERR("BLE HID: no hold timer, refusing the press of 0x%03x", (unsigned)usage);
    return E_INTERNAL;
  }

  // Outside the lock: from the app task this waits on the app's own queue, and
  // holding the lock across that wait would stall a presser on another task.
  if (!prv_queue_to_kernel_bg(prv_press_system_task_cb,
                              prv_pack_press(generation, usage, path))) {
    PBL_LOG_WRN("BLE HID: could not queue the press of 0x%03x", (unsigned)usage);
    // Only if this press is still the current one: a release and a new press can
    // both have run by now, and clearing the flag then would drop theirs.
    if (prv_generation_is_current(generation)) {
      s_held_usage = 0;
    }
    return E_INTERNAL;
  }
  return S_SUCCESS;
}

status_t ble_hid_consumer_release(uint16_t usage) {
  // Generation first, held usage second. A press that slips in between then
  // leaves us carrying the older generation, so the release is discarded rather
  // than lifting a key that has only just gone down; the hold timer covers the
  // one it was meant for. The other order would let it release the new press.
  const uint32_t generation = s_press_generation;
  const uint16_t held = s_held_usage;
  if (held == 0) {
    return S_NO_ACTION_REQUIRED;
  }
  if (usage != held) {
    return E_INVALID_ARGUMENT;
  }
  // No path argument: the release goes out on s_held_path, so a report 2 press
  // cannot be released on report 1 and leave the key down.
  if (!prv_queue_to_kernel_bg(prv_release_system_task_cb, prv_pack_release(generation))) {
    // The hold timer still covers it.
    PBL_LOG_WRN("BLE HID: could not queue the release of 0x%03x", (unsigned)usage);
    return E_INTERNAL;
  }
  return S_SUCCESS;
}

void ble_hid_set_low_latency(bool enabled) {
  bt_lock();
  {
    GAPLEConnection *connection = gap_le_connection_get_gateway();
    if (connection) {
      conn_mgr_set_ble_conn_response_time(
          connection, BtConsumerHidRemote, enabled ? ResponseTimeMin : ResponseTimeMax,
          MIN_LATENCY_MODE_TIMEOUT_HID_SECS);
    }
  }
  bt_unlock();
}

void bt_driver_cb_hid_service_update_subscription(const BTDeviceInternal *device,
                                                  bool is_subscribed) {
  // One event per report characteristic, so read both back rather than trust
  // whichever one arrived last.
  PBL_LOG_DBG("BLE HID: subscribed bits=%u usage=%u", bt_driver_hid_service_is_subscribed(),
              bt_driver_hid_service_usage_is_subscribed());

  // Read before s_held_usage, for the reason in ble_hid_consumer_release().
  const uint32_t generation = s_press_generation;
  if ((s_held_usage == 0) || prv_path_is_subscribed(s_held_path)) {
    return;
  }
  // The peer stopped listening, or went away, with a key held. Let KernelBG
  // clear it through the normal release path rather than write the state from
  // here; if the queue will not take it, the hold timer still will. Runs on the
  // BT host task, so it must not block and must not touch a timer.
  if (!prv_queue_to_kernel_bg(prv_release_system_task_cb, prv_pack_release(generation))) {
    PBL_LOG_WRN("BLE HID: could not queue the forced release of 0x%03x", (unsigned)s_held_usage);
  }
}

void ble_hid_init(void) {
  if (s_press_mutex == NULL) {
    // Kept for the lifetime of the FW, like the timer below: a stack restart
    // must not free a lock a presser may be waiting on.
    s_press_mutex = mutex_create();
  }
  s_held_usage = 0;
  s_held_path = BleHidReportPathBitmap;
  // s_press_generation is deliberately not reset: keeping it monotonic across a
  // stack restart is what stops a timer armed before the restart from matching.

  if (!bt_driver_is_hid_service_supported() || (s_release_timer != TIMER_INVALID_ID)) {
    return;
  }
  // Created outside bt_lock: new_timer_create takes the TaskTimerManager mutex,
  // which NimbleHost may hold while waiting for bt_lock. The timer is kept for
  // the lifetime of the FW so a restart cannot free one that is about to fire.
  s_release_timer = new_timer_create();
  if (s_release_timer == TIMER_INVALID_ID) {
    PBL_LOG_ERR("BLE HID: failed to create the release timer");
  }
}

void ble_hid_deinit(void) {
  // The timer outlives bt_driver_stop(), so an armed one would post a release
  // against a stopped host and could clear a flag owned by the next session.
  if (s_release_timer != TIMER_INVALID_ID) {
    new_timer_stop(s_release_timer);
  }
  // Nothing can be sent to a host that is going down, and a HOGP host drops
  // every key it knows about when the link goes, so forgetting is the release.
  s_held_usage = 0;
}

#else

bool ble_hid_is_ready(BleHidReportPath path) { return false; }

status_t ble_hid_consumer_press(uint16_t usage, BleHidReportPath path) {
  return E_INVALID_OPERATION;
}

status_t ble_hid_consumer_release(uint16_t usage) { return S_NO_ACTION_REQUIRED; }

void ble_hid_set_low_latency(bool enabled) {}

void ble_hid_init(void) {}

void ble_hid_deinit(void) {}

#endif  // CONFIG_BT_HID_REMOTE
