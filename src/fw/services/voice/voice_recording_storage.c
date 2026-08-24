/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "voice_recording_storage.h"

#include "pbl/os/mutex.h"
#include "pbl/services/filesystem/pfs.h"
#include <pbl/logging/logging.h>
#include <pbl/util/attributes.h>

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

PBL_LOG_MODULE_DECLARE(service_voice, CONFIG_SERVICE_VOICE_LOG_LEVEL);

#define VOICE_REC_MAGIC (0x56524331)
#define VOICE_REC_CONTAINER_VERSION (2)
#define VOICE_REC_PREFIX "vrec_"
#define VOICE_REC_TEMP_PREFIX "vrecT_"
#define VOICE_REC_NAME_MAX (16)
#define VOICE_REC_OPEN_PAYLOADS_MAX (2)

typedef struct PACKED {
  uint32_t magic;
  uint16_t container_version;
  uint16_t channels;
  AudioTransferInfoSpeex speex;
  uint32_t created;
  Uuid app_uuid;
  uint32_t duration_ms;
} VoiceRecordingHeader;

// Cached sum of bytes occupied by valid recordings. Computed once at init, then maintained
// incrementally so voice_recording_start() need not re-open and header-read every file on each
// capture: finalize adds the new file's exact size, deletes invalidate it (recomputed lazily).
// All mutators run under the voice_recording lock.
static uint32_t s_total_bytes;
static bool s_total_bytes_valid;

typedef struct {
  int fd;
  VoiceRecordingId id;
  VoiceRecordingStorageMetadata metadata;
} OpenPayload;

// PFS only permits one descriptor per file. Keep metadata for long-lived payload descriptors so
// list and ownership queries do not need to reopen the active file.
static PebbleMutex *s_open_payload_lock;
static OpenPayload s_open_payloads[VOICE_REC_OPEN_PAYLOADS_MAX];

static uint32_t prv_compute_total_bytes(void);
static bool prv_read_header(int fd, VoiceRecordingHeader *header);
static bool prv_read_info(const char *name, VoiceRecordingInfo *info);

// Build a PFS filename from a prefix and recording id.
static void prv_make_name(char *buf, size_t len, const char *prefix, VoiceRecordingId id) {
  snprintf(buf, len, "%s%u", prefix, (unsigned)id);
}

// Check whether a filename belongs to a prefixed file family.
static bool prv_name_has_prefix(const char *name, const char *prefix) {
  return strncmp(name, prefix, strlen(prefix)) == 0;
}

// Match finalized recording files.
static bool prv_is_recording_file(const char *name) {
  return prv_name_has_prefix(name, VOICE_REC_PREFIX);
}

// Match incomplete recordings that startup cleanup must remove.
static bool prv_is_temp_file(const char *name) {
  return prv_name_has_prefix(name, VOICE_REC_TEMP_PREFIX);
}

// Extract and validate the id stored in a recording filename.
static bool prv_parse_id(const char *name, VoiceRecordingId *id_out) {
  const char *digits = name + strlen(VOICE_REC_PREFIX);
  if (*digits == '\0') {
    return false;
  }

  char *end = NULL;
  const unsigned long val = strtoul(digits, &end, 10);
  if ((end == digits) || (*end != '\0') || (val == 0) || (val > UINT16_MAX)) {
    return false;
  }
  *id_out = (VoiceRecordingId)val;
  return true;
}

// Convert runtime metadata into the persisted file header.
static void prv_fill_header(VoiceRecordingHeader *header,
                            const VoiceRecordingStorageMetadata *metadata) {
  *header = (VoiceRecordingHeader){
      .magic = VOICE_REC_MAGIC,
      .container_version = VOICE_REC_CONTAINER_VERSION,
      .channels = metadata->channels,
      .speex = metadata->speex,
      .created = metadata->created,
      .app_uuid = metadata->app_uuid,
      .duration_ms = metadata->duration_ms,
  };
}

// Rebuild runtime metadata from a file header and its size.
static void prv_fill_metadata(int fd, const VoiceRecordingHeader *header,
                              VoiceRecordingStorageMetadata *metadata) {
  *metadata = (VoiceRecordingStorageMetadata){
      .channels = header->channels,
      .speex = header->speex,
      .created = header->created,
      .app_uuid = header->app_uuid,
      .duration_ms = header->duration_ms,
      .data_bytes = pfs_get_file_size(fd) - sizeof(*header),
  };
}

// Build the public list entry for a recording.
static void prv_fill_info(VoiceRecordingId id, const VoiceRecordingStorageMetadata *metadata,
                          VoiceRecordingInfo *info) {
  *info = (VoiceRecordingInfo){
      .id = id,
      .size_bytes = sizeof(VoiceRecordingHeader) + metadata->data_bytes,
      .duration_ms = metadata->duration_ms,
      .created = (time_t)metadata->created,
      .app_uuid = metadata->app_uuid,
  };
}

// Find cached metadata for an open payload. Caller holds s_open_payload_lock.
static OpenPayload *prv_find_open_payload(VoiceRecordingId id) {
  for (size_t i = 0; i < VOICE_REC_OPEN_PAYLOADS_MAX; i++) {
    if ((s_open_payloads[i].fd >= 0) && (s_open_payloads[i].id == id)) {
      return &s_open_payloads[i];
    }
  }
  return NULL;
}

// Find an unused payload-cache slot. Caller holds s_open_payload_lock.
static OpenPayload *prv_find_free_open_payload(void) {
  for (size_t i = 0; i < VOICE_REC_OPEN_PAYLOADS_MAX; i++) {
    if (s_open_payloads[i].fd < 0) {
      return &s_open_payloads[i];
    }
  }
  return NULL;
}

// Check that a stored file has a valid recording header.
static bool prv_has_valid_header(const char *name) {
  const int fd = pfs_open(name, OP_FLAG_READ, FILE_TYPE_STATIC, 0);
  if (fd < 0) {
    return false;
  }
  VoiceRecordingHeader header;
  const bool ok = prv_read_header(fd, &header);
  pfs_close(fd);
  return ok;
}

void voice_recording_storage_init(VoiceRecordingId *next_id_out) {
  s_open_payload_lock = mutex_create();
  for (size_t i = 0; i < VOICE_REC_OPEN_PAYLOADS_MAX; i++) {
    s_open_payloads[i].fd = -1;
  }

  pfs_remove_files(prv_is_temp_file);

  // Prime the allocator right past the highest stored id. This is only a hint: once the id
  // space has wrapped, voice_recording_storage_id_in_use() is what guarantees a free id.
  VoiceRecordingId max_id = 0;
  PFSFileListEntry *list = pfs_create_file_list(prv_is_recording_file);
  for (PFSFileListEntry *entry = list; entry; entry = (PFSFileListEntry *)entry->list_node.next) {
    VoiceRecordingId id;
    if (prv_parse_id(entry->name, &id) && (id > max_id)) {
      max_id = id;
    }
    // An interrupted finalize (header written last) leaves a file with an invalid header:
    // excluded from listing and the quota, but still occupying flash. Remove it.
    if (!prv_has_valid_header(entry->name)) {
      PBL_LOG_WRN("Removing invalid recording file %s", entry->name);
      pfs_remove(entry->name);
    }
  }
  pfs_delete_file_list(list);
  *next_id_out = (max_id == UINT16_MAX) ? 1 : (max_id + 1);

  s_total_bytes = prv_compute_total_bytes();
  s_total_bytes_valid = true;
}

bool voice_recording_storage_id_in_use(VoiceRecordingId id) {
  mutex_lock(s_open_payload_lock);
  if (prv_find_open_payload(id)) {
    mutex_unlock(s_open_payload_lock);
    return true;
  }

  char name[VOICE_REC_NAME_MAX];
  prv_make_name(name, sizeof(name), VOICE_REC_PREFIX, id);
  const int fd = pfs_open(name, OP_FLAG_READ, FILE_TYPE_STATIC, 0);
  if (fd < 0) {
    mutex_unlock(s_open_payload_lock);
    return false;
  }
  pfs_close(fd);
  mutex_unlock(s_open_payload_lock);
  return true;
}

uint32_t voice_recording_storage_header_size(void) {
  return sizeof(VoiceRecordingHeader);
}

int voice_recording_storage_open_temp(VoiceRecordingId id, uint32_t payload_capacity) {
  char name[VOICE_REC_NAME_MAX];
  prv_make_name(name, sizeof(name), VOICE_REC_TEMP_PREFIX, id);

  const uint32_t size = sizeof(VoiceRecordingHeader) + payload_capacity;
  const int fd = pfs_open(name, OP_FLAG_WRITE, FILE_TYPE_STATIC, size);
  if (fd < 0) {
    return fd;
  }

  const VoiceRecordingHeader placeholder = {0};
  if (pfs_write(fd, &placeholder, sizeof(placeholder)) != (int)sizeof(placeholder)) {
    pfs_close_and_remove(fd);
    return -1;
  }
  return fd;
}

void voice_recording_storage_remove_temp(VoiceRecordingId id) {
  char name[VOICE_REC_NAME_MAX];
  prv_make_name(name, sizeof(name), VOICE_REC_TEMP_PREFIX, id);
  pfs_remove(name);
}

bool voice_recording_storage_finalize(VoiceRecordingId id,
                                      const VoiceRecordingStorageMetadata *metadata,
                                      VoiceRecordingError *error_out) {
  char final_name[VOICE_REC_NAME_MAX];
  char temp_name[VOICE_REC_NAME_MAX];
  prv_make_name(final_name, sizeof(final_name), VOICE_REC_PREFIX, id);
  prv_make_name(temp_name, sizeof(temp_name), VOICE_REC_TEMP_PREFIX, id);

  const int temp_fd = pfs_open(temp_name, OP_FLAG_READ, FILE_TYPE_STATIC, 0);
  if (temp_fd < 0) {
    PBL_LOG_ERR("Failed to reopen temp recording for read (%d)", temp_fd);
    *error_out = VoiceRecordingError_FileOpen;
    return false;
  }

  const uint32_t total = sizeof(VoiceRecordingHeader) + metadata->data_bytes;
  const int final_fd = pfs_open(final_name, OP_FLAG_WRITE, FILE_TYPE_STATIC, total);
  if (final_fd < 0) {
    PBL_LOG_ERR("Failed to create recording file %s (%d)", final_name, final_fd);
    *error_out = VoiceRecordingError_FileOpen;
    pfs_close(temp_fd);
    return false;
  }

  VoiceRecordingHeader header;
  prv_fill_header(&header, metadata);
  bool ok = (pfs_seek(final_fd, sizeof(header), FSeekSet) >= 0) &&
            (pfs_seek(temp_fd, sizeof(header), FSeekSet) >= 0);
  if (ok) {
    uint8_t buf[256];
    uint32_t remaining = metadata->data_bytes;
    while (ok && (remaining > 0)) {
      const size_t chunk = (remaining < sizeof(buf)) ? remaining : sizeof(buf);
      if ((pfs_read(temp_fd, buf, chunk) != (int)chunk) ||
          (pfs_write(final_fd, buf, chunk) != (int)chunk)) {
        ok = false;
        break;
      }
      remaining -= chunk;
    }
  }

  if (ok) {
    ok = (pfs_seek(final_fd, 0, FSeekSet) >= 0) &&
         (pfs_write(final_fd, &header, sizeof(header)) == (int)sizeof(header));
  }

  pfs_close(final_fd);
  pfs_close(temp_fd);
  if (!ok) {
    *error_out = VoiceRecordingError_Write;
    pfs_remove(final_name);
  } else if (s_total_bytes_valid) {
    s_total_bytes += total;  // exact final file size
  }
  return ok;
}

// Read and validate a recording header.
static bool prv_read_header(int fd, VoiceRecordingHeader *header) {
  const size_t file_size = pfs_get_file_size(fd);
  if ((file_size < sizeof(*header)) ||
      (pfs_read(fd, header, sizeof(*header)) != (int)sizeof(*header)) ||
      (header->magic != VOICE_REC_MAGIC) ||
      (header->container_version != VOICE_REC_CONTAINER_VERSION)) {
    return false;
  }
  return true;
}

int voice_recording_storage_open_payload(VoiceRecordingId id, uint32_t *data_bytes_out) {
  mutex_lock(s_open_payload_lock);
  char name[VOICE_REC_NAME_MAX];
  prv_make_name(name, sizeof(name), VOICE_REC_PREFIX, id);
  const int fd = pfs_open(name, OP_FLAG_READ, FILE_TYPE_STATIC, 0);
  if (fd < 0) {
    mutex_unlock(s_open_payload_lock);
    return fd;
  }

  VoiceRecordingHeader header;
  if (!prv_read_header(fd, &header)) {
    pfs_close(fd);
    mutex_unlock(s_open_payload_lock);
    return -1;
  }

  OpenPayload *payload = prv_find_free_open_payload();
  if (!payload) {
    pfs_close(fd);
    mutex_unlock(s_open_payload_lock);
    return E_OUT_OF_RESOURCES;
  }

  prv_fill_metadata(fd, &header, &payload->metadata);
  payload->fd = fd;
  payload->id = id;
  *data_bytes_out = payload->metadata.data_bytes;
  mutex_unlock(s_open_payload_lock);
  return fd;
}

void voice_recording_storage_close_payload(int fd) {
  mutex_lock(s_open_payload_lock);
  pfs_close(fd);
  for (size_t i = 0; i < VOICE_REC_OPEN_PAYLOADS_MAX; i++) {
    if (s_open_payloads[i].fd == fd) {
      s_open_payloads[i].fd = -1;
      break;
    }
  }
  mutex_unlock(s_open_payload_lock);
}

int voice_recording_storage_read_frame(int fd, uint32_t *remaining_bytes, uint8_t *frame_out,
                                       size_t frame_out_size) {
  if (*remaining_bytes < 1) {
    return 0;
  }

  uint8_t len = 0;
  if (pfs_read(fd, &len, 1) != 1) {
    return 0;
  }
  (*remaining_bytes)--;
  if ((len == 0) || (len > *remaining_bytes) || (len > frame_out_size)) {
    PBL_LOG_WRN("Corrupt recording frame (len=%u), ending stream", len);
    return 0;
  }

  if (pfs_read(fd, frame_out, len) != (int)len) {
    return 0;
  }
  *remaining_bytes -= len;
  return len;
}

bool voice_recording_storage_get_metadata(VoiceRecordingId id,
                                          VoiceRecordingStorageMetadata *out) {
  mutex_lock(s_open_payload_lock);
  OpenPayload *payload = prv_find_open_payload(id);
  if (payload) {
    *out = payload->metadata;
    mutex_unlock(s_open_payload_lock);
    return true;
  }

  char name[VOICE_REC_NAME_MAX];
  prv_make_name(name, sizeof(name), VOICE_REC_PREFIX, id);
  const int fd = pfs_open(name, OP_FLAG_READ, FILE_TYPE_STATIC, 0);
  if (fd < 0) {
    mutex_unlock(s_open_payload_lock);
    return false;
  }

  VoiceRecordingHeader header;
  const bool ok = prv_read_header(fd, &header);
  if (ok) {
    prv_fill_metadata(fd, &header, out);
  }
  pfs_close(fd);
  mutex_unlock(s_open_payload_lock);
  return ok;
}

// Read list information from the cache or the stored file.
static bool prv_read_info(const char *name, VoiceRecordingInfo *info) {
  VoiceRecordingId id;
  if (!prv_parse_id(name, &id)) {
    return false;
  }

  mutex_lock(s_open_payload_lock);
  OpenPayload *payload = prv_find_open_payload(id);
  if (payload) {
    prv_fill_info(id, &payload->metadata, info);
    mutex_unlock(s_open_payload_lock);
    return true;
  }

  const int fd = pfs_open(name, OP_FLAG_READ, FILE_TYPE_STATIC, 0);
  if (fd < 0) {
    mutex_unlock(s_open_payload_lock);
    return false;
  }

  VoiceRecordingHeader header;
  const bool ok = prv_read_header(fd, &header);
  if (ok) {
    VoiceRecordingStorageMetadata metadata;
    prv_fill_metadata(fd, &header, &metadata);
    prv_fill_info(id, &metadata, info);
  }
  pfs_close(fd);
  mutex_unlock(s_open_payload_lock);
  return ok;
}

// Enumerate recordings with optional ownership and pagination filters.
static uint32_t prv_list(VoiceRecordingInfo *out, uint32_t max, uint32_t offset,
                         const Uuid *app_uuid, bool *has_more) {
  if (has_more) {
    *has_more = false;
  }
  if (!out || (max == 0)) {
    return 0;
  }

  uint32_t count = 0;
  uint32_t skipped = 0;
  PFSFileListEntry *list = pfs_create_file_list(prv_is_recording_file);
  for (PFSFileListEntry *entry = list; entry;
       entry = (PFSFileListEntry *)entry->list_node.next) {
    VoiceRecordingInfo info;
    if (prv_read_info(entry->name, &info) && (!app_uuid || uuid_equal(&info.app_uuid, app_uuid))) {
      if (skipped < offset) {
        skipped++;
      } else if (count < max) {
        out[count++] = info;
      } else {
        if (has_more) {
          *has_more = true;
        }
        break;
      }
    }
  }
  pfs_delete_file_list(list);
  return count;
}

uint32_t voice_recording_storage_list(VoiceRecordingInfo *out, uint32_t max) {
  return prv_list(out, max, 0, NULL, NULL);
}

uint32_t voice_recording_storage_list_page(VoiceRecordingInfo *out, uint32_t max,
                                           uint32_t offset, bool *has_more) {
  return prv_list(out, max, offset, NULL, has_more);
}

uint32_t voice_recording_storage_list_owned_by(VoiceRecordingInfo *out, uint32_t max,
                                               const Uuid *app_uuid) {
  if (!app_uuid) {
    return 0;
  }
  return prv_list(out, max, 0, app_uuid, NULL);
}

// Order summaries newest first, using the id to make equal timestamps deterministic.
static bool prv_summary_is_newer(const VoiceRecordingSummary *lhs,
                                 const VoiceRecordingSummary *rhs) {
  return (lhs->created > rhs->created) ||
         ((lhs->created == rhs->created) && (lhs->id > rhs->id));
}

uint32_t voice_recording_storage_list_summaries(VoiceRecordingSummary *out, uint32_t max,
                                                bool *has_more) {
  if (has_more) {
    *has_more = false;
  }
  if (!out || (max == 0)) {
    return 0;
  }

  uint32_t count = 0;
  uint32_t valid_count = 0;
  PFSFileListEntry *list = pfs_create_file_list(prv_is_recording_file);
  for (PFSFileListEntry *entry = list; entry; entry = (PFSFileListEntry *)entry->list_node.next) {
    VoiceRecordingInfo info;
    if (!prv_read_info(entry->name, &info)) {
      continue;
    }

    valid_count++;
    const VoiceRecordingSummary summary = {
        .id = info.id,
        .duration_ms = info.duration_ms,
        .created = info.created,
    };
    uint32_t insert_at = 0;
    while ((insert_at < count) && !prv_summary_is_newer(&summary, &out[insert_at])) {
      insert_at++;
    }
    if (insert_at >= max) {
      continue;
    }

    if (count < max) {
      count++;
    }
    memmove(&out[insert_at + 1], &out[insert_at],
            (count - insert_at - 1) * sizeof(*out));
    out[insert_at] = summary;
  }
  pfs_delete_file_list(list);
  if (has_more) {
    *has_more = (valid_count > max);
  }
  return count;
}

// Sum the space occupied by valid recordings.
static uint32_t prv_compute_total_bytes(void) {
  uint32_t total = 0;
  PFSFileListEntry *list = pfs_create_file_list(prv_is_recording_file);
  for (PFSFileListEntry *entry = list; entry; entry = (PFSFileListEntry *)entry->list_node.next) {
    VoiceRecordingInfo info;
    if (prv_read_info(entry->name, &info)) {
      total += info.size_bytes;
    }
  }
  pfs_delete_file_list(list);
  return total;
}

uint32_t voice_recording_storage_total_bytes(void) {
  if (!s_total_bytes_valid) {
    s_total_bytes = prv_compute_total_bytes();
    s_total_bytes_valid = true;
  }
  return s_total_bytes;
}

bool voice_recording_storage_delete(VoiceRecordingId id) {
  char name[VOICE_REC_NAME_MAX];
  prv_make_name(name, sizeof(name), VOICE_REC_PREFIX, id);
  const bool removed = (pfs_remove(name) == S_SUCCESS);
  if (removed) {
    s_total_bytes_valid = false;  // recomputed lazily on next query
  }
  return removed;
}

void voice_recording_storage_delete_owned_by(const Uuid *app_uuid, VoiceRecordingId skip_id) {
  PFSFileListEntry *list = pfs_create_file_list(prv_is_recording_file);
  for (PFSFileListEntry *entry = list; entry; entry = (PFSFileListEntry *)entry->list_node.next) {
    VoiceRecordingId id;
    if ((skip_id != VOICE_RECORDING_ID_INVALID) && prv_parse_id(entry->name, &id) &&
        (id == skip_id)) {
      continue;
    }
    VoiceRecordingInfo info;
    if (prv_read_info(entry->name, &info) && uuid_equal(&info.app_uuid, app_uuid)) {
      pfs_remove(entry->name);
      s_total_bytes_valid = false;  // recomputed lazily on next query
    }
  }
  pfs_delete_file_list(list);
}
