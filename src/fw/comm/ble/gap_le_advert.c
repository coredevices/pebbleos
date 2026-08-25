/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "gap_le_advert.h"
#include "gap_le_connect.h"

#include <bluetooth/bt_driver_advert.h>
#include <bluetooth/init.h>

#include "comm/bt_lock.h"
#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"
#include "pbl/services/analytics/analytics.h"
#include "pbl/services/regular_timer.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/list.h"

PBL_LOG_MODULE_DECLARE(bt, CONFIG_BT_LOG_LEVEL);

//! CC2564 / HCI Advertising Limitation:
//! ------------------------------
//! The Bluetooth chip can accept only one advertising payload, one
//! corresponding scan response and one set of intervals. However, we need to
//! juggle multiple advertising payloads for different needs. For example,
//! to be discoverable we need to advertise, to be reconnectable we need to
//! advertise something else, to be an iBeacon we need to advertise yet
//! something different, etc.
//! Unfortunately, the Ti CC2564 Bluetooth controller does not offer built-in
//! functionality to cope with this, so we need to implement a scheduling
//! mechanism in the firmware of the host / microcontroller.
//!
//! Advertisement Scheduling:
//! -------------------------
//! The advertisement scheduling is pretty dumb and works as follows:
//! The scheduler has "cycles" which are fixed size windows in time, during
//! which one of the scheduled jobs is set to advertise.
//!
//! At the beginning of a cycle, the scheduler decides which job to advertise
//! next. It will just round-robin through the jobs to advertise.
//!
//! Note that only one job is advertising at a time. Even though a job might
//! have such a long interval that another job could be squeezed in between,
//! clever things like that are not considered for simplicity's sake.
//!
//! To-Do's:
//! --------
//! - ble_discoverability/pairability.c
//! - Use private addresses for privacy / harder tracebility.

// Advertising interval parameters in ms, indexed by GAPLEAdvertisingInterval.
// Values comply with Apple Accessory Design Guidelines.
static const uint32_t s_interval_ms[] = {
  [GAPLEAdvertisingInterval_Short] = 20,     // 20ms
  [GAPLEAdvertisingInterval_Long]  = 1022,   // 1022.5ms (truncated to ms)
};

typedef struct GAPLEAdvertisingJob {
  ListNode node;

  //! The callback to call when this job is unscheduled.
  GAPLEAdvertisingJobUnscheduleCallback unscheduled_callback;
  //! The data to pass into the unscheduled callback.
  void *unscheduled_callback_data;

  //! The number of seconds the current term has been on air.
  uint16_t term_time_elapsed_secs;

  uint8_t cur_term;
  uint8_t num_terms;
  //! The terms are run in the order that they appear in this array
  GAPLEAdvertisingJobTerm *terms;

  GAPLEAdvertisingJobTag tag:8;

  //! The advertisement and scan response data
  BLEAdData payload;
} GAPLEAdvertisingJob;
// -----------------------------------------------------------------------------
// Static Variables -- MUST be protected with bt_lock/unlock!

static bool s_gap_le_advert_is_initialized;
static bool s_deinit_in_progress;

//! Circular list! Pointing to the current job that needs air-time.
static GAPLEAdvertisingJob *s_jobs;

//! Job that is currently on air.
static GAPLEAdvertisingJob *s_current;

//! Advertising data that was last configured into the controller.
//! @note This pointer may be dangling, don't try to reference!
static const BLEAdData *s_current_ad_data;

//! The regular timer that marks the end of a cycle and triggers the next job
//! to be aired.
static RegularTimerInfo s_cycle_regular_timer;

static bool s_is_advertising;

static bool s_is_connected;

//! Cache of the last advertising transmission power in dBm. A cache is kept in
//! case the API call fails, for example because Bluetooth is disabled.
//! 12 dBm is what the PAN1315 Bluetooth module reports.
static int8_t s_tx_power_cached = 12;

// -----------------------------------------------------------------------------
//! Prototypes

static void prv_perform_next_job(bool force_refresh);

// -----------------------------------------------------------------------------
//! Analytics helpers for tracking advertising interval time.

static void prv_analytics_stop_timers(void) {
  PBL_ANALYTICS_TIMER_STOP(ble_adv_short_intvl_time_ms);
  PBL_ANALYTICS_TIMER_STOP(ble_adv_long_intvl_time_ms);
}

static void prv_analytics_start_timer(GAPLEAdvertisingInterval interval) {
  switch (interval) {
    case GAPLEAdvertisingInterval_Short:
      PBL_ANALYTICS_TIMER_START(ble_adv_short_intvl_time_ms);
      break;
    case GAPLEAdvertisingInterval_Long:
      PBL_ANALYTICS_TIMER_START(ble_adv_long_intvl_time_ms);
      break;
  }
}

// -----------------------------------------------------------------------------

static const char * prv_string_for_debug_tag(GAPLEAdvertisingJobTag tag) {
  switch (tag) {
    case GAPLEAdvertisingJobTagDiscovery: return "DIS";
    case GAPLEAdvertisingJobTagReconnection: return "RCN";
    default: return "?";
  }
}

// -----------------------------------------------------------------------------
//! Helpers to manage the s_jobs list
//! bt_lock is expected to be taken with all of them!

static bool prv_is_current_term_infinite(const GAPLEAdvertisingJob *job) {
  return (job->terms[job->cur_term].duration_secs == GAPLE_ADVERTISING_DURATION_INFINITE);
}

//! Links the job into the ring of jobs.
static void prv_link_job(GAPLEAdvertisingJob *job) {
  if (!s_jobs) {
    // First job, make it point to itself:
    job->node.next = &job->node;
    job->node.prev = &job->node;
  } else {
    list_insert_after(&s_jobs->node, &job->node);
  }

  s_jobs = job;
}

static void prv_unlink_job(GAPLEAdvertisingJob *job) {
  if (job->node.next == &job->node) {
    // Last job left...
    job->node.next = NULL;
    job->node.prev = NULL;
    s_jobs = NULL;
  } else {
    list_remove(&job->node, (ListNode **) &s_jobs, NULL);
  }
}

static bool prv_is_registered_job(const GAPLEAdvertisingJob *job) {
  if (!job) {
    return false;
  }

  // Search jobs (can't use list_contains(), because circular):
  ListNode *node = &s_jobs->node;
  while (node) {
    if (node == (const ListNode *) job) {
      return true;
    }
    node = node->next;
    if (node == &s_jobs->node) {
      // wrapped around
      break;
    }
  }
  return false;
}

static void prv_increment_elapsed_time_for_job(GAPLEAdvertisingJob **job_ptr, bool *has_new_term) {
  GAPLEAdvertisingJob *job = *job_ptr;
  if (prv_is_current_term_infinite(job)) {
    return;
  }
  // Increment the `time elapsed` counter:
  ++(job->term_time_elapsed_secs);

  if (job->term_time_elapsed_secs >= job->terms[job->cur_term].duration_secs) {
    // The current term has elapsed
    job->cur_term++;
    // Schedule the next term
    if (job->cur_term < job->num_terms) {
      // Take care of GAPLE_ADVERTISING_DURATION_LOOP_AROUND:
      if (job->terms[job->cur_term].duration_secs == GAPLE_ADVERTISING_DURATION_LOOP_AROUND) {
        PBL_LOG_DBG("Job looped around to term %"PRIu16,
                    job->terms[job->cur_term].loop_around_index);
        job->cur_term = job->terms[job->cur_term].loop_around_index;
      }

      job->term_time_elapsed_secs = 0;
      PBL_LOG_DBG("Job is performing next advertising term (%d/%d)",
                  job->cur_term, job->num_terms);
      // force an update to make sure the new requested term takes
      if (has_new_term) {
        *has_new_term = true;
      }
    } else {
      // Job's done, remove done job:

      // If it's the last, this will update s_jobs to NULL as well:
      prv_unlink_job(job);

      // Call the unscheduled callback:
      if (job->unscheduled_callback) {
        job->unscheduled_callback(job, true /* completed */, job->unscheduled_callback_data);
      }

      PBL_LOG_DBG("Unscheduled advertising completed job: %s",
                  prv_string_for_debug_tag(job->tag));
      kernel_free(job->terms);
      kernel_free(job);
      *job_ptr = NULL;
    }
  }
}

// -----------------------------------------------------------------------------
//! Cycle timer callback.
//! It increments the air-time counter of the job's current term.
//! Updates the job's term if the term is done.
//! It removes the job if it's done.
//! It updates the s_jobs list.
//! It calls prv_perform_next_job() to set up the next job.
static void prv_cycle_kernelmain_cb(void *unused) {
  bool force_update = false;

  bt_lock();
  {
    if (!s_current || !s_gap_le_advert_is_initialized) {
      // Job got removed in the meantime.
      goto unlock;
    }

    if (s_is_connected) {
      // Don't do anything if connected
      goto unlock;
    }

    GAPLEAdvertisingJob *job = s_current;

    // Set to next job (round-robin)
    s_jobs = (GAPLEAdvertisingJob *) job->node.next;

    prv_increment_elapsed_time_for_job(&job, &force_update);

    prv_perform_next_job(force_update);
  }
unlock:
  bt_unlock();
}

//! Runs once per second on the NewTimers task; never block on bt_lock there
//! (it stalls every other timer in the system) — bounce to KernelMain.
static void prv_cycle_timer_callback(void *unused) {
  launcher_task_add_callback(prv_cycle_kernelmain_cb, NULL);
}

// -----------------------------------------------------------------------------
//! Timer start / stop utilities
//! bt_lock is expected to be taken!
static void prv_timer_start(void) {
  if (regular_timer_is_scheduled(&s_cycle_regular_timer)) {
    PBL_LOG_ERR("Advertising timer already started");
    regular_timer_remove_callback(&s_cycle_regular_timer);
  }

  regular_timer_add_seconds_callback(&s_cycle_regular_timer);
}

static void prv_timer_stop(void) {
  regular_timer_remove_callback(&s_cycle_regular_timer);
}

// -----------------------------------------------------------------------------
//! Airs the next advertisement job.
//! It sends the ad & scan response data to the Bluetooth controller and
//! enables/disables advertising.
//! It sets up / cleans up the cycle timer.
//! It updates the s_current pointer.
//! It does *not* mutate the s_jobs list.
//! bt_lock is expected to be taken!
//! @param force_refresh If true, the advertisement job will be re-setup even
//! though the current job has not changed. This is (only) useful when the
//! connectability mode has changed.
static void prv_perform_next_job(bool force_refresh) {
  // Pick the next job:
  GAPLEAdvertisingJob *next = s_jobs;

  // s_is_dangling is checked here, in case the next job happens to have been allocated at the
  // same address as the old s_current:
  const bool is_same_job = (next == s_current);

  if (is_same_job && !force_refresh && s_is_advertising) {
    // No change in job to give air time, keep going.
    return;
  }

  if (s_current) {
    // Clean up old job:

    if (!next) {
      // No more jobs. Stop timer:
      prv_timer_stop();
    }

    if (s_is_advertising) {
      // Controller needs to stop advertising before we can start a new job:
      PBL_LOG_DBG("Disable last Ad job");
      bt_driver_advert_advertising_disable();
      prv_analytics_stop_timers();
      s_is_advertising = false;
    }
  }

  if (next) {
    // Set up the next job to be on air:

    if (!s_current) {
      // No current job, start timer:
      prv_timer_start();
    }

    if (s_current_ad_data != &next->payload) {
      // Give the advertisement data to the BT controller:
      bool result = bt_driver_advert_set_advertising_data(&next->payload);
      if (result) {
        s_current_ad_data = &next->payload;
      }
    }

    const GAPLEAdvertisingInterval interval = next->terms[next->cur_term].interval;
    const uint32_t min_interval_ms = s_interval_ms[interval];
    const uint32_t max_interval_ms = s_interval_ms[interval];

    PBL_LOG_DBG("Enable Ad job %s",  prv_string_for_debug_tag(next->tag));
    bool result = bt_driver_advert_advertising_enable(min_interval_ms, max_interval_ms);
    if (result) {
      s_is_advertising = true;
      prv_analytics_start_timer(interval);
      PBL_LOG_DBG("Airing advertising job: %s ", prv_string_for_debug_tag(next->tag));
    }
  }

  s_current = next;
}

// -----------------------------------------------------------------------------
GAPLEAdvertisingJobRef gap_le_advert_schedule(const BLEAdData *payload,
                            const GAPLEAdvertisingJobTerm *terms,
                            uint8_t num_terms,
                            GAPLEAdvertisingJobUnscheduleCallback callback,
                            void *callback_data,
                            GAPLEAdvertisingJobTag tag) {
  // Sanity check payload:
  if (!payload ||
      payload->ad_data_length > GAP_LE_AD_REPORT_DATA_MAX_LENGTH ||
      payload->scan_resp_data_length > GAP_LE_AD_REPORT_DATA_MAX_LENGTH) {
    return NULL;
  }

  // Each job must have at least 1 term
  if (num_terms == 0 || terms == NULL) {
    return NULL;
  }

  for (int i = 0; i < num_terms; i++) {
    // Loop-around term:
    const bool is_loop_around = (terms[i].duration_secs == GAPLE_ADVERTISING_DURATION_LOOP_AROUND);
    if (is_loop_around) {
      if (i == 0) {
        PBL_LOG_ERR("Loop-around term cannot be the first term");
        return NULL;
      }
      continue;
    }
  }

  // Create the job data structure:
  GAPLEAdvertisingJob *job = kernel_malloc_check(sizeof(GAPLEAdvertisingJob) +
                                           payload->ad_data_length +
                                           payload->scan_resp_data_length);

  *job = (const GAPLEAdvertisingJob) {
    .unscheduled_callback = callback,
    .unscheduled_callback_data = callback_data,
    .term_time_elapsed_secs = 0,
    .num_terms = num_terms,
    .tag = tag,
    .payload = {
      .ad_data_length = payload->ad_data_length,
      .scan_resp_data_length = payload->scan_resp_data_length,
    },
  };

  job->terms = kernel_malloc_check(sizeof(GAPLEAdvertisingJobTerm) * num_terms);

  memcpy(job->terms, terms, sizeof(GAPLEAdvertisingJobTerm) * num_terms);

  memcpy(job->payload.data, payload->data,
         payload->ad_data_length + payload->scan_resp_data_length);

  PBL_LOG_INFO("Scheduling advertising job: %s",
          prv_string_for_debug_tag(job->tag));

  // Schedule
  bt_lock();
  {
    if (s_gap_le_advert_is_initialized && !s_deinit_in_progress) {
      prv_link_job(job);
      prv_perform_next_job(false);
    } else {
      kernel_free(job->terms);
      kernel_free(job);
      job = NULL;
    }
  }
  bt_unlock();

  return job;
}

// -----------------------------------------------------------------------------
void gap_le_advert_unschedule(GAPLEAdvertisingJobRef job) {
  if (!job) {
    return;
  }

  bool is_registered = false;

  bt_lock();
  {
    if (!s_gap_le_advert_is_initialized) {
      goto unlock;
    }

    is_registered = prv_is_registered_job(job);

    if (is_registered) {
      PBL_LOG_INFO("Unscheduling advertising job: %s",
              prv_string_for_debug_tag(job->tag));

      prv_unlink_job(job);
      prv_perform_next_job(false);

      // Call the unscheduled callback:
      if (job->unscheduled_callback) {
        job->unscheduled_callback(job, false /* completed */,
                                  job->unscheduled_callback_data);
      }

      // In case the payload pointer of a future jobs ends up being the same, ensure the adv data
      // will get updated in that case:
      if (s_current_ad_data == &job->payload) {
        s_current_ad_data = NULL;
      }
    }
  }
unlock:
  bt_unlock();

  if (is_registered) {
    kernel_free(job->terms);
    kernel_free(job);
  }
}

void gap_le_advert_unschedule_job_types(
    GAPLEAdvertisingJobTag *tag_types, size_t num_types) {
  bt_lock();

  ListNode *first_node = &s_current->node;

  // get the last job in the list
  ListNode *curr_node = list_get_prev(first_node);

  // Note: We attempt to get the currently running job and walk through the
  // list backwards so that we don't keep updating the running job as we remove
  // advertisements from our list

  while (curr_node) {
    GAPLEAdvertisingJob *job = (GAPLEAdvertisingJob *)curr_node;
    ListNode *prev_node = job->node.prev;

    for (size_t i = 0; i < num_types; i++) {
      if (job->tag == tag_types[i]) {
        PBL_LOG_DBG("Removing advertisement of type %s",
                    prv_string_for_debug_tag(job->tag));
        gap_le_advert_unschedule(job);
      }
    }

    if (curr_node == first_node) {
      break; // we have cycled through all the jobs
    }

    curr_node = prev_node;
  }

  bt_unlock();
}

// -----------------------------------------------------------------------------
int8_t gap_le_advert_get_tx_power(void) {
  int8_t tx_power;
  bt_lock();
  {
    // In case this API call fails, (e.g. Airplane Mode),
    // the s_tx_power_cached is untouched:
    if (bt_driver_advert_client_get_tx_power(&tx_power)) {
      s_tx_power_cached = tx_power;
    }
  }
  bt_unlock();
  return tx_power;
}

// -----------------------------------------------------------------------------
void gap_le_advert_init(void) {
  bt_lock();
  {
    if (s_gap_le_advert_is_initialized) {
      PBL_LOG_ERR("gap le advert has already been initialized");
      goto unlock;
    }

    s_deinit_in_progress = false;
    s_jobs = NULL;
    s_current = NULL;
    s_current_ad_data = NULL;
    s_cycle_regular_timer = (const RegularTimerInfo) {
      .cb = prv_cycle_timer_callback,
    };

    s_is_advertising = false;
    // Not cleared by the disconnect handler if the stack went down while
    // connected (airplane mode): a stale true pauses the cycle timer.
    s_is_connected = false;
    s_gap_le_advert_is_initialized = true;
  }
unlock:
  bt_unlock();
}

// -----------------------------------------------------------------------------
void gap_le_advert_deinit(void) {
  bt_lock();
  {
    s_deinit_in_progress = true;

    while (s_jobs) {
      gap_le_advert_unschedule(s_jobs);
    }

    PBL_ASSERTN(!regular_timer_is_scheduled(&s_cycle_regular_timer) ||
                regular_timer_pending_deletion(&s_cycle_regular_timer));
    s_gap_le_advert_is_initialized = false;
  }
  bt_unlock();
}

// -----------------------------------------------------------------------------
void gap_le_advert_handle_connect_as_slave(void) {
  bt_lock();
  {
    if (!s_gap_le_advert_is_initialized) {
      goto unlock;
    }
    // The link layer state machine inside the Bluetooth controller
    // automatically stops advertising when transitioning to "connected", so
    // update our own state. See 7.8.9 of Bluetooth Specification
    //
    // We don't instantly cycle the advertisements because our LE client
    // handler (kernel_le_client.c) will unschedule jobs accordingly and we
    // want to avoid unnecessary refreshes of the advertising state
    s_is_advertising = false;
    prv_analytics_stop_timers();

    s_is_connected = true;
  }
unlock:
  bt_unlock();
}

// -----------------------------------------------------------------------------
void gap_le_advert_handle_disconnect_as_slave(void) {
  bt_lock();
  {
    if (!s_gap_le_advert_is_initialized) {
      goto unlock;
    }

    s_is_connected = false;

    // Call prv_perform_next_job() to trigger refreshing the configuration of
    // the controller: it can advertise connectable packets again.
    prv_perform_next_job(true /* force refresh, connectability mode changed */);
  }
unlock:
  bt_unlock();
}

// -----------------------------------------------------------------------------
void bt_driver_handle_host_resynced(void) {
  bt_lock();
  {
    if (!s_gap_le_advert_is_initialized) {
      goto unlock;
    }

    // The controller's advertising state was wiped by the host re-sync, so any
    // cached pointer to ad data we already pushed is stale and any prior
    // adv-enable failed mid-flight.
    s_current_ad_data = NULL;
    s_is_advertising = false;

    if (s_current && !s_is_connected) {
      prv_perform_next_job(true /* force refresh */);
    }
  }
unlock:
  bt_unlock();
}

#undef GAP_LE_ADVERT_LOG_LEVEL
