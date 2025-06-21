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

#include "applib/data_logging.h"
#include "util/uuid.h"
#include "kernel/pebble_tasks.h"

//! This can be helpful when debugging. It changes the behavior of data logging to send any
//! stored data to the phone immediately after every dls_log()
//! Another helpful testing feature is that doing a long press on any item in the launcher menu will
//! also trigger a flush of all data logging data to the phone
// #define DLS_DEBUG_SEND_IMMEDIATELY


struct DataLoggingSession;
typedef struct DataLoggingSession DataLoggingSession;

//! List of tags used by system services. These are all registered with a uuid of UUID_SYSTEM
typedef enum {
  DlsSystemTagAnalyticsDeviceHeartbeat = 78,
  DlsSystemTagAnalyticsAppHeartbeat = 79,
  DlsSystemTagAnalyticsEvent = 80,
  DlsSystemTagActivityMinuteData = 81,
  DlsSystemTagActivityAccelSamples = 82,
  DlsSystemTagActivitySession = 84,
  DlsSystemTagProtobufLogSession = 85,
#ifdef MEMFAULT
  DlsSystemTagMemfaultChunksSession = 86,
#endif
} DlsSystemTag;

//! Init the data logging service. Called by the system at boot time.
void dls_init(void);

//! Return true if data logging initialized
bool dls_initialized(void);

//! The nuclear option! Clear out all data logging state in memory as well as on the flash storage.
void dls_clear(void);

//! Pause the data logging service
void dls_pause(void);

//! Resume the data logging service.
void dls_resume(void);

//! Find any sessions that the given task may have left in the DataLoggingStatusActive state and
//! moves them forcibly to the inactive state. This may result in data loss if the buffers haven't
//! been flushed to flash yet.
void dls_inactivate_sessions(PebbleTask task);

//! Create a new session using the UUID of the current process. This always creates a buffered
//! session. Unless this is the worker task, buffer must be allocated by the caller and must be at
//! least DLS_SESSION_BUFFER_SIZE bytes large. It will be freed by the data logging service when the
//! session is closed if this method returns no error. The worker task can optionally pass NULL for
//! buffer and the buffer will be allocated in the system heap for it by the data logging service.
DataLoggingSession* dls_create_current_process(uint32_t tag, DataLoggingItemType item_type,
                                               uint16_t item_size, void* buffer, bool resume);

//! Create a new session
DataLoggingSession* dls_create(uint32_t tag, DataLoggingItemType item_type, uint16_t item_size,
                               bool buffered, bool resume, const Uuid* uuid);

//! Append data to a logging session. Buffered sessions log asynchronously. Non buffered ones block.
DataLoggingResult dls_log(DataLoggingSession *s, const void* data, uint32_t num_items);

//! Finish up a session
void dls_finish(DataLoggingSession *s);

//! Checks to see if this is an actual valid data session.
//! Note that we pass in the logging_session parameter without making sure it's the same. Make sure
//! this function handles passing in random pointers that don't actually point to valid sessions or
//! even valid memory.
bool dls_is_session_valid(DataLoggingSession *logging_session);

//! Triggers data logging to immediately send all stored data to the phone rather than wait for the
//! next regular minute heartbeat. As a testing aid, a long press on any item in the launcher menu
//! calls into this method.
void dls_send_all_sessions(void);

//! Get the send_enable setting
bool dls_get_send_enable(void);

//! Set the send_enable setting for PebbleProtocol
void dls_set_send_enable_pp(bool setting);

//! Set the send_enable setting for Run Level
void dls_set_send_enable_run_level(bool setting);
