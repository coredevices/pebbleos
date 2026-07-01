/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "voice_recording_storage.h"

#include "pbl/services/filesystem/pfs.h"
#include <pbl/logging/logging.h>
#include <pbl/util/attributes.h>

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

PBL_LOG_MODULE_DECLARE(service_voice, CONFIG_SERVICE_VOICE_LOG_LEVEL);

#define VOICE_REC_MAGIC (0x56524331)
#define VOICE_REC_CONTAINER_VERSION (1)
#define VOICE_REC_PREFIX "vrec_"
#define VOICE_REC_TEMP_PREFIX "vrecT_"
#define VOICE_REC_NAME_MAX (16)

typedef struct PACKED {
  uint32_t magic;
  uint16_t container_version;
  uint16_t channels;
  AudioTransferInfoSpeex speex;
  uint32_t created;
  Uuid app_uuid;
  uint32_t frame_count;
  uint32_t duration_ms;
  uint32_t data_bytes;
} VoiceRecordingHeader;

// Cached sum of bytes occupied by valid recordings. Computed once at init, then maintained
// incrementally so voice_recording_start() need not re-open and header-read every file on each
// capture: finalize adds the new file's exact size, deletes invalidate it (recomputed lazily).
// All mutators run under the voice_recording lock.
static uint32_t s_total_bytes;
static bool s_total_bytes_valid;

// Recording excluded from the current voice_recording_storage_delete_all() pass (its file is
// held open by a transcription stream). Only valid while that call runs.
static VoiceRecordingId s_delete_all_skip_id = VOICE_RECORDING_ID_INVALID;

static uint32_t prv_compute_total_bytes(void);
static bool prv_read_header(int fd, VoiceRecordingHeader *header);

static void prv_make_name(char *buf, size_t len, const char *prefix, VoiceRecordingId id) {
  snprintf(buf, len, "%s%u", prefix, (unsigned)id);
}

static bool prv_name_has_prefix(const char *name, const char *prefix) {
  return strncmp(name, prefix, strlen(prefix)) == 0;
}

static bool prv_is_recording_file(const char *name) {
  return prv_name_has_prefix(name, VOICE_REC_PREFIX);
}

static bool prv_is_temp_file(const char *name) {
  return prv_name_has_prefix(name, VOICE_REC_TEMP_PREFIX);
}

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

static void prv_fill_header(VoiceRecordingHeader *header,
                            const VoiceRecordingStorageMetadata *metadata) {
  *header = (VoiceRecordingHeader){
      .magic = VOICE_REC_MAGIC,
      .container_version = VOICE_REC_CONTAINER_VERSION,
      .channels = metadata->channels,
      .speex = metadata->speex,
      .created = metadata->created,
      .app_uuid = metadata->app_uuid,
      .frame_count = metadata->frame_count,
      .duration_ms = metadata->duration_ms,
      .data_bytes = metadata->data_bytes,
  };
}

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
  pfs_remove_files(prv_is_temp_file);

  VoiceRecordingId next_id = 1;
  PFSFileListEntry *list = pfs_create_file_list(prv_is_recording_file);
  for (PFSFileListEntry *entry = list; entry; entry = (PFSFileListEntry *)entry->list_node.next) {
    VoiceRecordingId id;
    if (prv_parse_id(entry->name, &id) && (id >= next_id)) {
      next_id = (id == UINT16_MAX) ? 1 : (id + 1);
    }
    // An interrupted finalize (header written last) leaves a file with an invalid header:
    // excluded from listing and the quota, but still occupying flash. Remove it.
    if (!prv_has_valid_header(entry->name)) {
      PBL_LOG_WRN("Removing invalid recording file %s", entry->name);
      pfs_remove(entry->name);
    }
  }
  pfs_delete_file_list(list);
  *next_id_out = next_id;

  s_total_bytes = prv_compute_total_bytes();
  s_total_bytes_valid = true;
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

void voice_recording_storage_close_temp(int fd, bool remove) {
  if (fd < 0) {
    return;
  }
  if (remove) {
    pfs_close_and_remove(fd);
  } else {
    pfs_close(fd);
  }
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

static bool prv_read_header(int fd, VoiceRecordingHeader *header) {
  const size_t file_size = pfs_get_file_size(fd);
  if ((file_size < sizeof(*header)) ||
      (pfs_read(fd, header, sizeof(*header)) != (int)sizeof(*header)) ||
      (header->magic != VOICE_REC_MAGIC) ||
      (header->container_version != VOICE_REC_CONTAINER_VERSION) ||
      (header->data_bytes > file_size - sizeof(*header))) {
    return false;
  }
  return true;
}

int voice_recording_storage_open_payload(VoiceRecordingId id, uint32_t *data_bytes_out) {
  char name[VOICE_REC_NAME_MAX];
  prv_make_name(name, sizeof(name), VOICE_REC_PREFIX, id);
  const int fd = pfs_open(name, OP_FLAG_READ, FILE_TYPE_STATIC, 0);
  if (fd < 0) {
    return fd;
  }

  VoiceRecordingHeader header;
  if (!prv_read_header(fd, &header)) {
    pfs_close(fd);
    return -1;
  }
  *data_bytes_out = header.data_bytes;
  return fd;
}

bool voice_recording_storage_get_metadata(VoiceRecordingId id,
                                          VoiceRecordingStorageMetadata *out) {
  char name[VOICE_REC_NAME_MAX];
  prv_make_name(name, sizeof(name), VOICE_REC_PREFIX, id);
  const int fd = pfs_open(name, OP_FLAG_READ, FILE_TYPE_STATIC, 0);
  if (fd < 0) {
    return false;
  }

  VoiceRecordingHeader header;
  const bool ok = prv_read_header(fd, &header);
  if (ok) {
    *out = (VoiceRecordingStorageMetadata){
        .channels = header.channels,
        .speex = header.speex,
        .created = header.created,
        .app_uuid = header.app_uuid,
        .frame_count = header.frame_count,
        .duration_ms = header.duration_ms,
        .data_bytes = header.data_bytes,
    };
  }
  pfs_close(fd);
  return ok;
}

static bool prv_read_info(const char *name, VoiceRecordingInfo *info) {
  const int fd = pfs_open(name, OP_FLAG_READ, FILE_TYPE_STATIC, 0);
  if (fd < 0) {
    return false;
  }

  VoiceRecordingHeader header;
  const bool ok = prv_read_header(fd, &header);
  if (ok) {
    VoiceRecordingId id;
    info->id = prv_parse_id(name, &id) ? id : VOICE_RECORDING_ID_INVALID;
    info->size_bytes = pfs_get_file_size(fd);
    info->duration_ms = header.duration_ms;
    info->created = (time_t)header.created;
    info->app_uuid = header.app_uuid;
  }
  pfs_close(fd);
  return ok;
}

uint32_t voice_recording_storage_list(VoiceRecordingInfo *out, uint32_t max) {
  if (!out || (max == 0)) {
    return 0;
  }

  uint32_t count = 0;
  PFSFileListEntry *list = pfs_create_file_list(prv_is_recording_file);
  for (PFSFileListEntry *entry = list; entry && (count < max);
       entry = (PFSFileListEntry *)entry->list_node.next) {
    if (prv_read_info(entry->name, &out[count])) {
      count++;
    }
  }
  pfs_delete_file_list(list);
  return count;
}

static uint32_t prv_compute_total_bytes(void) {
  uint32_t total = 0;
  PFSFileListEntry *list = pfs_create_file_list(prv_is_recording_file);
  for (PFSFileListEntry *entry = list; entry; entry = (PFSFileListEntry *)entry->list_node.next) {
    const int fd = pfs_open(entry->name, OP_FLAG_READ, FILE_TYPE_STATIC, 0);
    if (fd >= 0) {
      VoiceRecordingHeader header;
      if (prv_read_header(fd, &header)) {
        total += pfs_get_file_size(fd);
      }
      pfs_close(fd);
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

static bool prv_is_deletable_recording_file(const char *name) {
  if (!prv_is_recording_file(name)) {
    return false;
  }
  VoiceRecordingId id;
  return !((s_delete_all_skip_id != VOICE_RECORDING_ID_INVALID) && prv_parse_id(name, &id) &&
           (id == s_delete_all_skip_id));
}

void voice_recording_storage_delete_all(VoiceRecordingId skip_id) {
  s_delete_all_skip_id = skip_id;
  pfs_remove_files(prv_is_deletable_recording_file);
  s_delete_all_skip_id = VOICE_RECORDING_ID_INVALID;
  if (skip_id == VOICE_RECORDING_ID_INVALID) {
    s_total_bytes = 0;
    s_total_bytes_valid = true;
  } else {
    s_total_bytes_valid = false;  // one file was kept; recompute lazily
  }
}
