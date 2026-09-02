/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "flash_logging.h"

#include <pbl/drivers/flash.h>
#include "pbl/kernel/mutex.h"
#include "system/passert.h"
#include "util/shared_circular_buffer.h"

#include "pbl/services/system_task.h"

static SharedCircularBuffer s_buffer;
static SharedCircularBufferClient s_buffer_client;
// During normal operation, since log messages are hashed, most are only 30-40
// bytes long with the longest being about 80 bytes, so this is enough for 15-40
// or so messages.
static uint8_t s_buffer_storage[1200];
static PBL_MUTEX_DEFINE(s_buffer_mutex); //!< Protects s_buffer
static PBL_MUTEX_DEFINE(s_flash_write_mutex); //!< Protects log line consistency
static bool s_initialized;
static bool s_is_flash_write_scheduled; //!< true if handle_buffer_sync KernelBG callback is scheduled

static void write_message(void) {
  // Note that we should enter this function with the buffer mutex held.

  const uint8_t *data_read;
  uint16_t read_length;

  // Read the header part
  bool result = shared_circular_buffer_read(&s_buffer, &s_buffer_client, sizeof(uint8_t), &data_read, &read_length);
  PBL_ASSERTN(result);
  PBL_ASSERT(read_length == sizeof(uint8_t), "read_length %u sizeof(uint8_t) %u", read_length, sizeof(uint8_t));
  uint8_t msg_length = *data_read;

  if (shared_circular_buffer_get_read_space_remaining(&s_buffer, &s_buffer_client) < msg_length + sizeof(uint8_t)) {
    return; // Not ready yet, consume nothing.
  }

  // Flash_logging_log_start can trigger a flash erase. Release the buffer mutex
  // to allow logging while the (slow) erase completes.
  pbl_mutex_unlock(&s_buffer_mutex);
  uint32_t flash_addr = flash_logging_log_start(msg_length);
  pbl_mutex_lock(&s_buffer_mutex, PBL_FOREVER);
  if (flash_addr == FLASH_LOG_INVALID_ADDR) {
    return;
  }

  shared_circular_buffer_consume(&s_buffer, &s_buffer_client, read_length);

  while (msg_length > 0) {
    pbl_mutex_unlock(&s_buffer_mutex);

    // Note that this buffer read really should be done with the buffer mutex held.
    // This works only because writes to the buffer do not advance slackers.
    result = shared_circular_buffer_read(&s_buffer, &s_buffer_client, msg_length, &data_read, &read_length);
    PBL_ASSERTN(result);
    msg_length -= read_length;

    flash_logging_write(data_read, flash_addr, read_length);
    flash_addr += read_length;

    pbl_mutex_lock(&s_buffer_mutex, PBL_FOREVER);
    shared_circular_buffer_consume(&s_buffer, &s_buffer_client, read_length);
  }

  // We should still be holding the buffer lock here...
}

static void handle_buffer_sync(void *data) {
  const bool is_async = (uintptr_t) data;

  pbl_mutex_lock(&s_flash_write_mutex, PBL_FOREVER);
  pbl_mutex_lock(&s_buffer_mutex, PBL_FOREVER);

  // Bound the drain to what was already pending when this callback ran.  Each
  // write_message() releases the buffer mutex around the flash write so other
  // tasks (BT, KernelMain) can keep producing log lines; if we kept the outer
  // while running on the live "remaining" count, a chatty producer can keep
  // this callback executing on KernelBackground forever.  That starves every
  // other system-task callback (PutBytes processing in particular) and the
  // emulator wedges with no progress and no watchdog reboot.  Drain the
  // initial backlog and leave any newly-arrived bytes for the next callback;
  // the s_is_flash_write_scheduled flag below ensures producers re-arm us.
  size_t budget = shared_circular_buffer_get_read_space_remaining(&s_buffer, &s_buffer_client);
  while (budget > 0 &&
         shared_circular_buffer_get_read_space_remaining(&s_buffer, &s_buffer_client) > 0) {
    size_t before = shared_circular_buffer_get_read_space_remaining(&s_buffer, &s_buffer_client);
    write_message();
    // The above function mucks with the mutex
    pbl_mutex_assert_held(&s_buffer_mutex, true /* is_held */);
    size_t after = shared_circular_buffer_get_read_space_remaining(&s_buffer, &s_buffer_client);
    if (after >= before) {
      // write_message() didn't make progress (e.g. partial message not ready);
      // bail rather than spinning.
      break;
    }
    size_t consumed = before - after;
    budget = (consumed >= budget) ? 0 : budget - consumed;
  }

  if (is_async) {
    s_is_flash_write_scheduled = false;
  }

  pbl_mutex_unlock(&s_buffer_mutex);
  pbl_mutex_unlock(&s_flash_write_mutex);
}


void advanced_logging_init(void) {
  flash_logging_init();

  shared_circular_buffer_init(&s_buffer, s_buffer_storage, sizeof(s_buffer_storage));
  shared_circular_buffer_add_client(&s_buffer, &s_buffer_client);

  s_initialized = true;
}

// Return true on success
static bool write_buffer_locking(char* buffer, int length, bool async) {
  bool success = false;

  do {
    pbl_mutex_lock(&s_buffer_mutex, PBL_FOREVER);
    if (shared_circular_buffer_get_write_space_remaining(&s_buffer) >= length + 1) {
      // Ideally we could figure out a way to skip out on this copy but then you'd potentially need to sniprintf
      // into a non-contiguous buffer... whatever, we have CPU to burn.
      uint8_t msg_length = length;

      // Do not advance slackers. Data loss and/or corruption will occur! See write_message()
      shared_circular_buffer_write(&s_buffer, &msg_length, sizeof(uint8_t), false /*advance_slackers*/);
      shared_circular_buffer_write(&s_buffer, (const uint8_t*) buffer, length, false /*advance_slackers*/);

      success = true;
    }
    pbl_mutex_unlock(&s_buffer_mutex);

    // If we failed to buffer this message, flush the buffer to cache to make room.
    // Otherwise, if this is a sync message, flush this message to flash.
    if (!success || !async) {
      handle_buffer_sync((void *)(uintptr_t) false /* !is_async */);
    }
  } while (!success); // Loop until the buffer copy succeeds. If sync, also wait until this message
                      // is written to flash.
                      // It's highly unlikely that another task will win the race and completely
                      // fill the buffer between the flash write and the next buffer write attempt.
                      // If so, there are bigger issues.

  if (async) {
    pbl_mutex_lock(&s_buffer_mutex, PBL_FOREVER);
    if (!s_is_flash_write_scheduled) {
      s_is_flash_write_scheduled = true;
      system_task_add_callback(handle_buffer_sync, (void *)(uintptr_t) true /* is_async */);
    }
    pbl_mutex_unlock(&s_buffer_mutex);
  }

  return success;
}

void pbl_log_advanced(char* buffer, int length, bool async) {
  if (!s_initialized) {
    return;
  }
  write_buffer_locking(buffer, length, async);
}

char pbl_log_get_level_char(const uint8_t log_level) {
  switch (log_level) {
  case LOG_LEVEL_ALWAYS:
    return '*';
  case LOG_LEVEL_ERROR:
    return 'E';
  case LOG_LEVEL_WARNING:
    return 'W';
  case LOG_LEVEL_INFO:
    return 'I';
  case LOG_LEVEL_DEBUG:
    return 'D';
  case LOG_LEVEL_DEBUG_VERBOSE:
    return 'V';
  default:
    return '?';
  }
}

