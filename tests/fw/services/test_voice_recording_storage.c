/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "services/voice/voice_recording_storage.h"

#include <stdlib.h>

#include "fake_mutex.h"
#include "fake_spi_flash.h"
#include "stubs_analytics.h"
#include "stubs_logging.h"
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"
#include "stubs_pebble_tasks.h"
#include "stubs_print.h"
#include "stubs_prompt.h"
#include "stubs_serial.h"
#include "stubs_sleep.h"
#include "stubs_task_watchdog.h"

#include "pbl/services/filesystem/pfs.h"
#include "pbl/util/size.h"

static void prv_create_recording(VoiceRecordingId id, uint32_t created, uint32_t duration_ms) {
  const int fd = voice_recording_storage_open_temp(id, 0);
  cl_assert(fd >= 0);
  cl_assert_equal_i(S_SUCCESS, pfs_close(fd));

  const VoiceRecordingStorageMetadata metadata = {
      .channels = 1,
      .created = created,
      .duration_ms = duration_ms,
  };
  VoiceRecordingError error = VoiceRecordingError_None;
  cl_assert(voice_recording_storage_finalize(id, &metadata, &error));
  voice_recording_storage_remove_temp(id);
}

void test_voice_recording_storage__initialize(void) {
  fake_spi_flash_init(0, 0x1000000);
  pfs_init(false);
  pfs_format(true);

  VoiceRecordingId next_id;
  voice_recording_storage_init(&next_id);
  cl_assert_equal_i(1, next_id);
}

void test_voice_recording_storage__cleanup(void) {
  fake_mutex_reset(true);
  fake_spi_flash_cleanup();
}

void test_voice_recording_storage__summaries_keep_the_newest_entries_in_order(void) {
  prv_create_recording(1, 100, 1000);
  prv_create_recording(2, 400, 4000);
  prv_create_recording(3, 300, 3000);
  prv_create_recording(4, 200, 2000);

  VoiceRecordingSummary summaries[3];
  bool has_more = false;
  const uint32_t count =
      voice_recording_storage_list_summaries(summaries, ARRAY_LENGTH(summaries), &has_more);

  cl_assert_equal_i(3, count);
  cl_assert(has_more);
  cl_assert_equal_i(2, summaries[0].id);
  cl_assert_equal_i(4000, summaries[0].duration_ms);
  cl_assert_equal_i(3, summaries[1].id);
  cl_assert_equal_i(4, summaries[2].id);
}

void test_voice_recording_storage__summary_order_uses_id_for_equal_timestamps(void) {
  prv_create_recording(10, 100, 1000);
  prv_create_recording(11, 100, 1100);

  VoiceRecordingSummary summaries[2];
  const uint32_t count = voice_recording_storage_list_summaries(
      summaries, ARRAY_LENGTH(summaries), NULL);

  cl_assert_equal_i(2, count);
  cl_assert_equal_i(11, summaries[0].id);
  cl_assert_equal_i(10, summaries[1].id);
}
