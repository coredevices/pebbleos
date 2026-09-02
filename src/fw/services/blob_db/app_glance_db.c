/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/blob_db/app_glance_db.h"

#include "pbl/services/blob_db/app_glance_db_private.h"

#include "applib/app_glance.h"
#include <pbl/drivers/rtc.h>
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "pbl/kernel/mutex.h"
#include "resource/resource_ids.auto.h"
#include "process_management/app_install_manager.h"
#include "pbl/services/app_cache.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/settings/settings_file.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/math.h"
#include "util/units.h"

PBL_LOG_MODULE_DECLARE(service_blob_db, CONFIG_SERVICE_BLOB_DB_LOG_LEVEL);

#define SETTINGS_FILE_NAME "appglancedb"
//! The defines below calculate `APP_GLANCE_DB_MAX_USED_SIZE` which is the actual minimum space we
//! need to guarantee all of the apps's glances on the watch can have the same number of slices,
//! and that number currently evaluates to 69050 bytes. We provide some additional space beyond that
//! for some safety margin and easy future expansion, and thus use 80KB for the settings file size.
#define SETTINGS_FILE_SIZE (KiBYTES(80))

#define APP_GLANCE_DB_GLANCE_MAX_SIZE \
    (sizeof(SerializedAppGlanceHeader) + \
        (APP_GLANCE_DB_SLICE_MAX_SIZE * APP_GLANCE_DB_MAX_SLICES_PER_GLANCE))
#define APP_GLANCE_DB_MAX_USED_SIZE \
    (APP_GLANCE_DB_GLANCE_MAX_SIZE * APP_GLANCE_DB_MAX_NUM_APP_GLANCES)

_Static_assert(APP_GLANCE_DB_MAX_USED_SIZE <= SETTINGS_FILE_SIZE, "AppGlanceDB is too small!");

static struct {
  SettingsFile settings_file;
  struct pbl_mutex mutex;
} s_app_glance_db;

//////////////////////////////////////////
// Slice Type Implementation Definition
//////////////////////////////////////////

//! Return true if the type-specific serialized slice's attribute list is valid. You don't have to
//! check the attribute list pointer (we check it before calling this callback).
typedef bool (*AttributeListValidationFunc)(const AttributeList *attr_list);

//! Callback for copying the type-specific attributes from a serialized slice's attribute list
//! to the provided slice. You can assume that the attribute list and the slice_out pointers are
//! valid because we check them before calling this callback.
typedef void (*InitSliceFromAttributeListFunc)(const AttributeList *attr_list,
                                               AppGlanceSliceInternal *slice_out);

//! Callback for adding the type-specific fields from a slice to the provided attribute list.
//! You can assume that the slice and attribute list pointers are valid because we check them
//! before calling this callback.
typedef void (*InitAttributeListFromSliceFunc)(const AppGlanceSliceInternal *slice,
                                               AttributeList *attr_list_to_init);

typedef struct SliceTypeImplementation {
  AttributeListValidationFunc is_attr_list_valid;
  InitSliceFromAttributeListFunc init_slice_from_attr_list;
  InitAttributeListFromSliceFunc init_attr_list_from_slice;
} SliceTypeImplementation;

////////////////////////////////////////////////////////
// AppGlanceSliceType_IconAndSubtitle Implementation
////////////////////////////////////////////////////////

static bool prv_is_icon_and_subtitle_slice_attribute_list_valid(const AttributeList *attr_list) {
  // The icon and subtitle are optional.
  return true;
}

static void prv_init_icon_and_subtitle_slice_from_attr_list(const AttributeList *attr_list,
                                                            AppGlanceSliceInternal *slice_out) {
  slice_out->icon_and_subtitle.icon_resource_id = attribute_get_uint32(attr_list,
                                                                       AttributeIdIcon,
                                                                       INVALID_RESOURCE);
  strncpy(slice_out->icon_and_subtitle.template_string,
          attribute_get_string(attr_list, AttributeIdSubtitleTemplateString, NULL),
          ATTRIBUTE_APP_GLANCE_SUBTITLE_MAX_LEN + 1);
}

static void prv_init_attribute_list_from_icon_and_subtitle_slice(
    const AppGlanceSliceInternal *slice, AttributeList *attr_list_to_init) {
  attribute_list_add_cstring(attr_list_to_init, AttributeIdSubtitleTemplateString,
                             slice->icon_and_subtitle.template_string);
  attribute_list_add_resource_id(attr_list_to_init, AttributeIdIcon,
                                 slice->icon_and_subtitle.icon_resource_id);
}

//////////////////////////////////
// Slice Type Implementations
//////////////////////////////////

//! Add new entries to this array as we introduce new slice types
static const SliceTypeImplementation s_slice_type_impls[AppGlanceSliceTypeCount] = {
  [AppGlanceSliceType_IconAndSubtitle] = {
    .is_attr_list_valid = prv_is_icon_and_subtitle_slice_attribute_list_valid,
    .init_slice_from_attr_list = prv_init_icon_and_subtitle_slice_from_attr_list,
    .init_attr_list_from_slice = prv_init_attribute_list_from_icon_and_subtitle_slice,
  },
};

//////////////////////////////////
// Serialized Slice Iteration
//////////////////////////////////

//! Return true to continue iteration and false to stop it.
typedef bool (*SliceForEachCb)(SerializedAppGlanceSliceHeader *serialized_slice, void *context);

//! Returns true if iteration completed successfully, either due to reaching the end of the slices
//! or if the client's callback returns false to stop iteration early.
//! Returns false if an error occurred during iteration due to the slices' `.total_size` values
//! not being consistent with the provided `serialized_glance_size` argument.
static bool prv_slice_for_each(SerializedAppGlanceHeader *serialized_glance,
                               size_t serialized_glance_size, SliceForEachCb cb, void *context) {
  if (!serialized_glance || !cb) {
    return false;
  }

  SerializedAppGlanceSliceHeader *current_slice =
      (SerializedAppGlanceSliceHeader *)serialized_glance->data;

  size_t glance_size_processed = sizeof(SerializedAppGlanceHeader);

  // Note that we'll stop iterating after reading the max supported number of slices per glance
  for (unsigned int i = 0; i < APP_GLANCE_DB_MAX_SLICES_PER_GLANCE; i++) {
    // Stop iterating if we've read all of the slices by hitting the end of the glance data
    if (glance_size_processed == serialized_glance_size) {
      break;
    }

    // Stop iterating and report an error if we've somehow gone beyond the end of the glance data
    if (glance_size_processed > serialized_glance_size) {
      return false;
    }

    // Stop iterating if the client's callback function returns false
    if (!cb(current_slice, context)) {
      break;
    }

    // Advance to the next slice
    glance_size_processed += current_slice->total_size;
    current_slice =
        (SerializedAppGlanceSliceHeader *)(((uint8_t *)current_slice) + current_slice->total_size);
  }

  return true;
}

/////////////////////////////////////////
// Serialized Slice Validation Helpers
/////////////////////////////////////////

static bool prv_is_slice_type_valid(uint8_t type) {
  return (type < AppGlanceSliceTypeCount);
}

//! Returns true if the provided AttributeList is valid for the specified AppGlanceSliceType,
//! false otherwise.
static bool prv_is_slice_attribute_list_valid(uint8_t type, const AttributeList *attr_list) {
  // Check if the slice type is valid before we plug it into the validation func array below
  if (!prv_is_slice_type_valid(type)) {
    return false;
  }

  // Check if the AttributeList has the attributes required for this specific slice type
  return s_slice_type_impls[type].is_attr_list_valid(attr_list);
}

//////////////////////////////////
// Slice Deserialization
//////////////////////////////////

//! Returns true if a non-empty AttributeList was successfully deserialized from `serialized_slice`,
//! `attr_list` was filled with the result, and `attr_list_data_buffer_out` was filled with the data
//! buffer for `attr_list_out`. Returns false otherwise.
//! @note If function returns true, client must call `attribute_list_destroy_list()`
//!       on `attr_list_out` and `kernel_free()` on `attr_list_data_buffer_out`.
static bool prv_deserialize_attribute_list(const SerializedAppGlanceSliceHeader *serialized_slice,
                                           AttributeList *attr_list_out,
                                           char **attr_list_data_buffer_out) {
  if (!serialized_slice || !attr_list_out || !attr_list_data_buffer_out) {
    return false;
  }

  const uint8_t num_attributes = serialized_slice->num_attributes;
  // If there aren't any attributes, set `attr_list_out` to be an empty AttributeList and return
  // true because technically we did successfully deserialize the AttributeList
  if (!num_attributes) {
    *attr_list_out = (AttributeList) {};
    *attr_list_data_buffer_out = NULL;
    return true;
  }

  const uint8_t * const serialized_attr_list_start = serialized_slice->data;
  const uint8_t * const serialized_attr_list_end =
      serialized_attr_list_start + serialized_slice->total_size;

  // Get the buffer size needed for the attributes we're going to deserialize
  const uint8_t *buffer_size_cursor = serialized_attr_list_start;
  const int32_t buffer_size =
      attribute_get_buffer_size_for_serialized_attributes(num_attributes, &buffer_size_cursor,
                                                          serialized_attr_list_end);
  if (buffer_size < 0) {
    PBL_LOG_WRN("Failed to measure the buffer size required for deserializing an AttributeList from a "
            "serialized slice");
    return false;
  }

  if (buffer_size) {
    // Allocate buffer for the data attached to the attributes
    *attr_list_data_buffer_out = kernel_zalloc(buffer_size);
    if (!*attr_list_data_buffer_out) {
      PBL_LOG_ERR("Failed to alloc memory for the Attributes' data buffer while deserializing an "
              "AttributeList from a serialized slice");
      return false;
    }
  } else {
    // No buffer needed, but set the output pointer to NULL because we might blindly free it below
    // if we fail to alloc memory for the attribute_buffer
    *attr_list_data_buffer_out = NULL;
  }

  // Allocate buffer for the Attribute's
  // Note that this doesn't need to be passed back as an output because it gets freed as part of the
  // client calling `attribute_list_destroy_list()` on `attr_list_out`
  Attribute *attribute_buffer = kernel_zalloc(num_attributes * sizeof(*attribute_buffer));
  if (!attribute_buffer) {
    PBL_LOG_ERR("Failed to alloc memory for the buffer of Attribute's while deserializing an "
            "AttributeList from a serialized slice");
    // Free the `*attr_list_data_buffer_out` we might have allocated above
    kernel_free(*attr_list_data_buffer_out);
    return false;
  }

  // Setup the arguments for `attribute_deserialize_list()`
  char *attribute_data_buffer_pointer = *attr_list_data_buffer_out;
  char * const attribute_data_buffer_end = attribute_data_buffer_pointer + buffer_size;
  *attr_list_out = (AttributeList) {
    .num_attributes = num_attributes,
    .attributes = attribute_buffer,
  };
  const uint8_t *deserialization_cursor = serialized_attr_list_start;

  // Try to deserialize the AttributeList
  const bool was_attr_list_deserialized = attribute_deserialize_list(&attribute_data_buffer_pointer,
                                                                     attribute_data_buffer_end,
                                                                     &deserialization_cursor,
                                                                     serialized_attr_list_end,
                                                                     *attr_list_out);
  if (!was_attr_list_deserialized) {
    kernel_free(attribute_buffer);
    kernel_free(*attr_list_data_buffer_out);
  }

  return was_attr_list_deserialized;
}

typedef struct SliceDeserializationIteratorContext {
  AppGlance *glance_out;
  bool deserialization_failed;
} SliceDeserializationIteratorContext;

static bool prv_deserialize_slice(SerializedAppGlanceSliceHeader *serialized_slice, void *context) {
  SliceDeserializationIteratorContext *deserialization_context = context;

  // Deserialize the serialized slice's attribute list
  AttributeList attr_list = {};
  char *attr_list_data_buffer = NULL;
  if (!prv_deserialize_attribute_list(serialized_slice, &attr_list, &attr_list_data_buffer)) {
    deserialization_context->deserialization_failed = true;
    return false;
  }

  // Check that the deserialized attribute list is valid
  const bool success = prv_is_slice_attribute_list_valid(serialized_slice->type, &attr_list);
  if (!success) {
    goto cleanup;
  }

  AppGlance *glance_out = deserialization_context->glance_out;

  // Copy the common serialized slice fields to the output glance's slice
  const unsigned int current_slice_index = glance_out->num_slices;
  AppGlanceSliceInternal *current_slice_out = &glance_out->slices[current_slice_index];
  // Note that we default the expiration time to "never expire" if one was not provided
  *current_slice_out = (AppGlanceSliceInternal) {
    .expiration_time = attribute_get_uint32(&attr_list, AttributeIdTimestamp,
                                            APP_GLANCE_SLICE_NO_EXPIRATION),
    .type = (AppGlanceSliceType)serialized_slice->type,
  };
  // Copy type-specific fields from the serialized slice to the output glance's slice
  s_slice_type_impls[serialized_slice->type].init_slice_from_attr_list(&attr_list,
                                                                       current_slice_out);

  // Increment the number of slices in the glance
  glance_out->num_slices++;

cleanup:
  attribute_list_destroy_list(&attr_list);
  kernel_free(attr_list_data_buffer);

  return success;
}

static status_t prv_deserialize_glance(SerializedAppGlanceHeader *serialized_glance,
                                       size_t serialized_glance_size, AppGlance *glance_out) {
  if (!serialized_glance || !glance_out) {
    return E_INVALID_ARGUMENT;
  }

  // Zero out the output glance
  *glance_out = (AppGlance) {};

  // Iterate over the slices to deserialize them
  SliceDeserializationIteratorContext context = (SliceDeserializationIteratorContext) {
    .glance_out = glance_out,
  };
  if (!prv_slice_for_each(serialized_glance, serialized_glance_size,
                          prv_deserialize_slice, &context) ||
      context.deserialization_failed) {
    return E_ERROR;
  }

  return S_SUCCESS;
}

//////////////////////////////////
// Slice Serialization
//////////////////////////////////

typedef struct SliceSerializationAttributeListData {
  AttributeList attr_list;
  size_t attr_list_size;
} SliceSerializationAttributeListData;

//! Returns S_SUCCESS if the provided glance was successfully serialized into serialized_glance_out
//! and its serialized size copied to serialized_glance_size_out.
//! @note If function returns S_SUCCESS, client must call  `kernel_free()` on the pointer provided
//! for `serialized_glance_out`.
static status_t prv_serialize_glance(const AppGlance *glance,
                                     SerializedAppGlanceHeader **serialized_glance_out,
                                     size_t *serialized_glance_size_out) {
  if (!glance || (glance->num_slices > APP_GLANCE_DB_MAX_SLICES_PER_GLANCE) ||
      !serialized_glance_out || !serialized_glance_size_out) {
    return E_INVALID_ARGUMENT;
  }

  // Allocate a buffer for data about each slice's attribute list, but only if we have at least
  // one slice because allocating 0 bytes would return NULL and that is a return value we want to
  // reserve for the case when we've run out of memory
  SliceSerializationAttributeListData *attr_lists = NULL;
  if (glance->num_slices > 0) {
    attr_lists = kernel_zalloc(sizeof(SliceSerializationAttributeListData) * glance->num_slices);
    if (!attr_lists) {
      return E_OUT_OF_MEMORY;
    }
  }

  status_t rv;

  // Iterate over the glance slices, creating attribute lists and summing the size we need for the
  // overall serialized slice
  size_t serialized_glance_size = sizeof(SerializedAppGlanceHeader);
  for (unsigned int slice_index = 0; slice_index < glance->num_slices; slice_index++) {
    SliceSerializationAttributeListData *current_attr_list_data = &attr_lists[slice_index];
    const AppGlanceSliceInternal *current_slice = &glance->slices[slice_index];
    // Check the slice's type, fail the entire serialization if it's invalid
    if (!prv_is_slice_type_valid(current_slice->type)) {
      PBL_LOG_WRN("Tried to serialize a glance containing a slice with invalid type: %d",
              current_slice->type);
      rv = E_INVALID_ARGUMENT;
      goto cleanup;
    }

    serialized_glance_size += sizeof(SerializedAppGlanceSliceHeader);

    AttributeList *attr_list = &current_attr_list_data->attr_list;
    // Initialize the attributes common to all slice types in the attribute list
    attribute_list_add_uint32(attr_list, AttributeIdTimestamp,
                              (uint32_t)current_slice->expiration_time);
    // Initialize the type-specific attributes in the attribute list
    s_slice_type_impls[current_slice->type].init_attr_list_from_slice(current_slice, attr_list);

    // Record size of the attribute list in the data struct as well as the overall size accumulator
    current_attr_list_data->attr_list_size = attribute_list_get_serialized_size(attr_list);
    serialized_glance_size += current_attr_list_data->attr_list_size;
  }

  // Allocate a buffer for the serialized glance
  SerializedAppGlanceHeader *serialized_glance = kernel_zalloc(serialized_glance_size);
  if (!serialized_glance) {
    rv = E_OUT_OF_MEMORY;
    goto cleanup;
  }

  // Populate the header of the serialized glance
  *serialized_glance = (SerializedAppGlanceHeader) {
    .version = APP_GLANCE_DB_CURRENT_VERSION,
    .creation_time = (uint32_t)rtc_get_time(),
  };

  uint8_t *glance_buffer_start = (uint8_t *)serialized_glance;
  uint8_t *glance_buffer_end = glance_buffer_start + serialized_glance_size;
  // Start the cursor where the serialized slices go
  uint8_t *glance_buffer_cursor = serialized_glance->data;

  // Serialize each slice into the serialized glance buffer
  for (unsigned int slice_index = 0; slice_index < glance->num_slices; slice_index++) {
    const AppGlanceSliceInternal *current_slice = &glance->slices[slice_index];

    SliceSerializationAttributeListData *current_attr_list_data = &attr_lists[slice_index];
    AttributeList *attr_list = &current_attr_list_data->attr_list;
    const size_t attr_list_size = current_attr_list_data->attr_list_size;

    // Calculate the total size of this serialized slice
    const uint16_t serialized_slice_total_size =
        sizeof(SerializedAppGlanceSliceHeader) + attr_list_size;

    // Populate the serialized slice header
    SerializedAppGlanceSliceHeader *serialized_slice_header =
        (SerializedAppGlanceSliceHeader *)glance_buffer_cursor;
    *serialized_slice_header = (SerializedAppGlanceSliceHeader) {
      .type = current_slice->type,
      .total_size = serialized_slice_total_size,
      .num_attributes = attr_list->num_attributes,
    };

    // Serialize the slice's attribute list
    attribute_list_serialize(attr_list, serialized_slice_header->data, glance_buffer_end);

    // Note that we'll destroy the attribute list's attributes below in the cleanup section

    // Advance the cursor by the serialized slice's total size
    glance_buffer_cursor += serialized_slice_total_size;
  }

  // Check that we fully populated the serialized glance buffer
  rv = (glance_buffer_cursor == glance_buffer_end) ? S_SUCCESS : E_ERROR;
  if (rv == S_SUCCESS) {
    *serialized_glance_out = serialized_glance;
    *serialized_glance_size_out = serialized_glance_size;
  } else {
    kernel_free(serialized_glance);
  }

cleanup:
  // Destroy the attributes of each of the attribute lists in attr_lists
  for (unsigned int i = 0; i < glance->num_slices; i++) {
    attribute_list_destroy_list(&attr_lists[i].attr_list);
  }
  kernel_free(attr_lists);
  return rv;
}

//////////////////////////////////
// Serialized Slice Validation
//////////////////////////////////

static bool prv_is_serialized_slice_valid(const SerializedAppGlanceSliceHeader *serialized_slice) {
  if (!serialized_slice ||
      !prv_is_slice_type_valid(serialized_slice->type) ||
      !WITHIN(serialized_slice->total_size, APP_GLANCE_DB_SLICE_MIN_SIZE,
              APP_GLANCE_DB_SLICE_MAX_SIZE)) {
    return false;
  }

  // Deserialize the AttributeList from `serialized_slice`
  AttributeList attr_list = {};
  char *attr_list_data_buffer = NULL;
  if (!prv_deserialize_attribute_list(serialized_slice, &attr_list, &attr_list_data_buffer)) {
    PBL_LOG_WRN("Failed to deserialize an AttributeList from a serialized slice");
    return false;
  }

  // Check if the AttributeList has the attributes required for the slice
  const bool is_attr_list_valid = prv_is_slice_attribute_list_valid(serialized_slice->type,
                                                                    &attr_list);
  if (!is_attr_list_valid) {
    PBL_LOG_WRN("Serialized slice AttributeList is invalid");
  }

  attribute_list_destroy_list(&attr_list);
  kernel_free(attr_list_data_buffer);

  return is_attr_list_valid;
}

typedef struct SliceValidationIteratorContext {
  bool is_at_least_one_slice_invalid;
  size_t validated_size;
} SliceValidationIteratorContext;

//! If any slices are invalid, context.is_at_least_one_slice_invalid will be set to true.
//! If all the slices are valid, context.validated_size will hold the size of the entire serialized
//! glance after trimming any slices that go beyond the APP_GLANCE_DB_MAX_SLICES_PER_GLANCE limit.
//! @note This assumes that context.validated_size has been initialized to take into account
//!       the serialized app glance's header.
static bool prv_validate_slice(SerializedAppGlanceSliceHeader *serialized_slice, void *context) {
  SliceValidationIteratorContext *validation_context = context;
  PBL_ASSERTN(validation_context);

  if (!prv_is_serialized_slice_valid(serialized_slice)) {
    *validation_context = (SliceValidationIteratorContext) {
      .is_at_least_one_slice_invalid = true,
      .validated_size = 0,
    };
    return false;
  }

  validation_context->validated_size += serialized_slice->total_size;

  return true;
}

/////////////////////////
// AppGlanceDB API
/////////////////////////

status_t app_glance_db_insert_glance(const Uuid *uuid, const AppGlance *glance) {
  if (!uuid || !glance) {
    return E_INVALID_ARGUMENT;
  }

  SerializedAppGlanceHeader *serialized_glance = NULL;
  size_t serialized_glance_size = 0;
  status_t rv = prv_serialize_glance(glance, &serialized_glance, &serialized_glance_size);
  if (rv == S_SUCCESS) {
    rv = app_glance_db_insert((uint8_t *)uuid, UUID_SIZE, (uint8_t *)serialized_glance,
                              serialized_glance_size);
  }

  kernel_free(serialized_glance);

  return rv;
}

status_t app_glance_db_read_glance(const Uuid *uuid, AppGlance *glance_out) {
  if (!uuid || !glance_out) {
    return E_INVALID_ARGUMENT;
  }

  const uint8_t *key = (uint8_t *)uuid;
  const int key_size = UUID_SIZE;

  const int serialized_glance_size = app_glance_db_get_len(key, key_size);
  if (!serialized_glance_size) {
    return E_DOES_NOT_EXIST;
  } else if (serialized_glance_size < 0) {
    WTF;
  }

  uint8_t *serialized_glance = kernel_zalloc((size_t)serialized_glance_size);
  if (!serialized_glance) {
    return E_OUT_OF_MEMORY;
  }

  status_t rv = app_glance_db_read(key, key_size, serialized_glance, serialized_glance_size);
  if (rv != S_SUCCESS) {
    goto cleanup;
  }

  rv = prv_deserialize_glance((SerializedAppGlanceHeader *)serialized_glance,
                              (size_t)serialized_glance_size, glance_out);

cleanup:
  kernel_free(serialized_glance);
  return rv;
}

status_t app_glance_db_read_creation_time(const Uuid *uuid, time_t *time_out) {
  if (!uuid || !time_out) {
    return E_INVALID_ARGUMENT;
  }

  SerializedAppGlanceHeader serialized_glance_header = {};
  const status_t rv = app_glance_db_read((uint8_t *)uuid, UUID_SIZE,
                                         (uint8_t *)&serialized_glance_header,
                                         sizeof(serialized_glance_header));
  if (rv == S_SUCCESS) {
    *time_out = serialized_glance_header.creation_time;
  }
  return rv;
}

status_t app_glance_db_delete_glance(const Uuid *uuid) {
  return app_glance_db_delete((uint8_t *)uuid, UUID_SIZE);
}

//////////////////////
// Settings helpers
//////////////////////

// TODO PBL-38080: Extract out settings file opening/closing and mutex locking/unlocking for BlobDB

static status_t prv_lock_mutex_and_open_file(void) {
  pbl_mutex_lock(&s_app_glance_db.mutex, PBL_FOREVER);
  const status_t rv = settings_file_open_growable(&s_app_glance_db.settings_file,
                                                  SETTINGS_FILE_NAME, SETTINGS_FILE_SIZE,
                                                  KiBYTES(4));
  if (rv != S_SUCCESS) {
    pbl_mutex_unlock(&s_app_glance_db.mutex);
  }
  return rv;
}

static void prv_close_file_and_unlock_mutex(void) {
  settings_file_close(&s_app_glance_db.settings_file);
  pbl_mutex_unlock(&s_app_glance_db.mutex);
}

/////////////////////////
// Blob DB API
/////////////////////////

void app_glance_db_init(void) {
  pbl_mutex_init(&s_app_glance_db.mutex);
}

status_t app_glance_db_flush(void) {
  pbl_mutex_lock(&s_app_glance_db.mutex, PBL_FOREVER);
  pfs_remove(SETTINGS_FILE_NAME);
  pbl_mutex_unlock(&s_app_glance_db.mutex);

  return S_SUCCESS;
}

status_t app_glance_db_compact(void) {
  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }
  rv = settings_file_compact(&s_app_glance_db.settings_file);
  prv_close_file_and_unlock_mutex();
  return rv;
}

static status_t prv_validate_glance(const Uuid *app_uuid,
                             const SerializedAppGlanceHeader *serialized_glance, size_t *len) {
  // Change this block if we support multiple app glance versions in the future
  // For now report an error if the glance's version isn't the current database version
  if (serialized_glance->version != APP_GLANCE_DB_CURRENT_VERSION) {
    PBL_LOG_WRN("Tried to insert AppGlanceDB entry with invalid version!"
            " Entry version: %"PRIu8", AppGlanceDB version: %u",
            serialized_glance->version, APP_GLANCE_DB_CURRENT_VERSION);
    return E_INVALID_ARGUMENT;
  }

  // Check that the creation_time of this new glance value is newer than any existing glance value
  SerializedAppGlanceHeader existing_glance = {};
  status_t rv = app_glance_db_read((uint8_t *)app_uuid, UUID_SIZE, (uint8_t *)&existing_glance,
                                   sizeof(existing_glance));
  if ((rv == S_SUCCESS) && (serialized_glance->creation_time <= existing_glance.creation_time)) {
    PBL_LOG_WRN("Tried to insert AppGlanceDB entry with older creation_time (%"PRIu32")"
            " than existing entry (%"PRIu32")", serialized_glance->creation_time,
            existing_glance.creation_time);
    return E_INVALID_ARGUMENT;
  }

  // Validate the slices (which also records a `validated_size` we'll use to trim excess slices)
  SliceValidationIteratorContext validation_context = {
    // Start by taking into account the header of the serialized glance
    .validated_size = sizeof(SerializedAppGlanceHeader),
  };
  // Iteration will fail if the slices report `total_size` values that
  const bool iteration_succeeded =
      prv_slice_for_each((SerializedAppGlanceHeader *)serialized_glance, *len,
                         prv_validate_slice, &validation_context);
  if (!iteration_succeeded) {
    PBL_LOG_WRN("Tried to insert AppGlanceDB entry but failed to iterate over the serialized slices");
    return E_INVALID_ARGUMENT;
  } else if (validation_context.is_at_least_one_slice_invalid) {
    PBL_LOG_WRN("Tried to insert AppGlanceDB entry with at least one invalid slice");
    return E_INVALID_ARGUMENT;
  }

  // Trim the serialized glance of excess slices by shrinking `val_len` to `validated_size`
  // We do this if the glance entry has more slices than the max number of slices per glance.
  // This can happen for glance entries sent to us by the mobile apps because they don't have a way
  // of knowing the max number of slices supported by the firmware, and so they send us as many
  // slices as they can fit in a BlobDB packet. We just take as many slices as we support and trim
  // the excess.
  if (validation_context.validated_size < *len) {
    PBL_LOG_WRN("Trimming AppGlanceDB entry of excess slices before insertion");
    *len = validation_context.validated_size;
  }
  return S_SUCCESS;
}

status_t app_glance_db_insert(const uint8_t *key, int key_len, const uint8_t *val, int val_len) {
  if ((key_len != UUID_SIZE) || (val_len < (int)sizeof(SerializedAppGlanceHeader))) {
    return E_INVALID_ARGUMENT;
  }

  const Uuid *app_uuid = (const Uuid *)key;
  const SerializedAppGlanceHeader *serialized_glance = (const SerializedAppGlanceHeader *)val;
  size_t len = val_len;
  status_t rv = prv_validate_glance(app_uuid, serialized_glance, &len);
  if (rv != S_SUCCESS) {
    return rv;
  }

  // Fetch app if it's in the app DB, but not cached. If it's not in the app db and not a system app
  // reject the glance insert

  AppInstallId app_id = app_install_get_id_for_uuid(app_uuid);
  if (app_install_id_from_app_db(app_id)) {
    // Bump the app's priority by telling the cache we're using it
    if (app_cache_entry_exists(app_id)) {
      app_cache_app_launched(app_id);
    } else {
      // The app isn't cached. Fetch it!
      PebbleEvent e = {
        .type = PEBBLE_APP_FETCH_REQUEST_EVENT,
        .app_fetch_request = {
          .id = app_id,
          .with_ui = false,
          .fetch_args = NULL,
        },
      };
      event_put(&e);
     }
  } else if (!app_install_id_from_system(app_id)) {
    // App is not installed (not in app db and not a system app). Do not insert the glance

    // String initialized on the heap to reduce stack usage
    char *app_uuid_string = kernel_malloc_check(UUID_STRING_BUFFER_LENGTH);
    uuid_to_string(app_uuid, app_uuid_string);
    PBL_LOG_WRN("Attempted app glance insert for an app that's not installed. UUID: %s",
            app_uuid_string);
    kernel_free(app_uuid_string);
    return E_DOES_NOT_EXIST;
  }

  rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  rv = settings_file_set(&s_app_glance_db.settings_file, key, (size_t)key_len, serialized_glance,
                         len);

  prv_close_file_and_unlock_mutex();

  return rv;
}

int app_glance_db_get_len(const uint8_t *key, int key_len) {
  if (key_len != UUID_SIZE) {
    return 0;
  }

  const status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return 0;
  }

  const int length = settings_file_get_len(&s_app_glance_db.settings_file, key, (size_t)key_len);

  prv_close_file_and_unlock_mutex();

  return length;
}

status_t app_glance_db_read(const uint8_t *key, int key_len, uint8_t *val_out, int val_out_len) {
  if ((key_len != UUID_SIZE) || val_out == NULL) {
    return E_INVALID_ARGUMENT;
  }

  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  rv = settings_file_get(&s_app_glance_db.settings_file, key, (size_t)key_len, val_out,
                         (size_t)val_out_len);
  if (rv == S_SUCCESS) {
    SerializedAppGlanceHeader *serialized_app_glance = (SerializedAppGlanceHeader *)val_out;

    // Change this block if we support multiple app glance versions in the future
    if (serialized_app_glance->version != APP_GLANCE_DB_CURRENT_VERSION) {
      // Clear out the stale entry
      PBL_LOG_WRN("Read a AppGlanceDB entry with an outdated version; deleting it");
      settings_file_delete(&s_app_glance_db.settings_file, key, (size_t)key_len);
      rv = E_DOES_NOT_EXIST;
    }
  }

  prv_close_file_and_unlock_mutex();

  return rv;
}

status_t app_glance_db_delete(const uint8_t *key, int key_len) {
  if (key_len != UUID_SIZE) {
    return E_INVALID_ARGUMENT;
  }

  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  if (settings_file_exists(&s_app_glance_db.settings_file, key, (size_t)key_len)) {
    rv = settings_file_delete(&s_app_glance_db.settings_file, key, (size_t)key_len);
  } else {
    rv = S_SUCCESS;
  }


  prv_close_file_and_unlock_mutex();

  return rv;
}

/////////////////////////
// Testing code
/////////////////////////

#if UNITTEST
void app_glance_db_deinit(void) {
  app_glance_db_flush();
  pbl_mutex_deinit(&s_app_glance_db.mutex);
}

status_t app_glance_db_insert_stale(const uint8_t *key, int key_len, const uint8_t *val,
                                    int val_len) {
  // Quick and dirty insert which doesn't do any error checking. Used to insert stale entries
  // for testing
  status_t rv = prv_lock_mutex_and_open_file();
  if (rv != S_SUCCESS) {
    return rv;
  }

  rv = settings_file_set(&s_app_glance_db.settings_file, key, key_len, val, val_len);

  prv_close_file_and_unlock_mutex();
  return rv;
}
#endif
