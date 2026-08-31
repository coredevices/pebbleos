/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pebble_process_md.h"

#include "process_loader.h"
#include <string.h>

//////////////////////
// Md Field Accessors
//////////////////////

const char* process_metadata_get_name(const PebbleProcessMd *md) {
  if (md->process_storage == ProcessStorageFlash) {
    return ((const PebbleProcessMdFlash*) md)->name;
  } else if (md->process_storage == ProcessStorageResource) {
    return ((const PebbleProcessMdResource*) md)->name;
  }
  return ((const PebbleProcessMdSystem*) md)->name;
}

uint32_t process_metadata_get_size_bytes(const PebbleProcessMd *md) {
  if (md->process_storage == ProcessStorageFlash) {
    return ((const PebbleProcessMdFlash*) md)->size_bytes;
  } else if (md->process_storage == ProcessStorageResource) {
    return ((const PebbleProcessMdResource*) md)->size_bytes;
  }
  return 0;
}

Version process_metadata_get_process_version(const PebbleProcessMd *md) {
  if (md->process_storage == ProcessStorageFlash) {
    return ((const PebbleProcessMdFlash*) md)->process_version;
  }
  return (Version) { 0, 0 };
}

Version process_metadata_get_sdk_version(const PebbleProcessMd *md) {
  if (md->process_storage == ProcessStorageFlash) {
    return ((const PebbleProcessMdFlash*) md)->sdk_version;
  }
  return (Version) { PROCESS_INFO_CURRENT_SDK_VERSION_MAJOR, PROCESS_INFO_CURRENT_SDK_VERSION_MINOR };
}

ProcessAppRunLevel process_metadata_get_run_level(const PebbleProcessMd *md) {
  if (md->process_storage == ProcessStorageFlash) {
    return ProcessAppRunLevelNormal;
  } else if (md->process_storage == ProcessStorageResource) {
    return ProcessAppRunLevelNormal;
  }
  return ((const PebbleProcessMdSystem*) md)->run_level;
}

int process_metadata_get_code_bank_num(const PebbleProcessMd *md) {
  if (md->process_storage == ProcessStorageFlash) {
    return ((const PebbleProcessMdFlash*) md)->code_bank_num;
  }
  return 0;
}

int process_metadata_get_res_bank_num(const PebbleProcessMd *md) {
  if (md->process_storage == ProcessStorageFlash) {
    return ((const PebbleProcessMdFlash*) md)->res_bank_num;
  }
  return 0;
}

ResourceVersion process_metadata_get_res_version(const PebbleProcessMd *md) {
  if (md->process_storage == ProcessStorageFlash) {
    return ((const PebbleProcessMdFlash*) md)->res_version;
  }
  return (ResourceVersion) { 0, 0 } ;
}

const uint8_t *process_metadata_get_build_id(const PebbleProcessMd *md) {
  if (md->process_storage == ProcessStorageFlash) {
    return ((const PebbleProcessMdFlash*) md)->build_id;
  }
  return NULL;
}

//////////////////////
// Md Builders
//////////////////////

static void prv_init_from_info_common(PebbleProcessMd *common, const PebbleProcessInfo *info,
      PebbleTask task, ProcessStorage process_storage) {

  memcpy(&common->uuid, &info->uuid, sizeof(Uuid));
  common->process_storage = process_storage;

  // Flags
  common->process_type = process_metadata_flags_process_type(info->flags, task);
  common->visibility = process_metadata_flags_visibility(info->flags);
  common->allow_js = process_metadata_flags_allow_js(info->flags);
  common->has_worker = process_metadata_flags_has_worker(info->flags);
  common->is_moddable_app = process_metadata_flags_moddable_app(info->flags);
  common->is_rocky_app = process_metadata_flags_rocky_app(info->flags);
  common->stored_sdk_platform = process_metadata_flags_stored_sdk_platform(info->flags);
  common->is_unprivileged = true;
  // We don't know the load address of the process until the process is
  // actually loaded, so we can't convert the entry point's offset into
  // an address until it's actually been loaded into that address.
  // Just shove the unmodified offset into the struct and let the
  // process loader convert it into an absolute address.
  // TODO: refactor the PebbleProcessMd stuff so that metadata that
  // isn't needed before load or can't be fully resolved until then
  // is left out of PebbleProcessMdCommon.
  common->main_func = (void *)(uintptr_t)info->offset;
}


void process_metadata_init_with_flash_header(PebbleProcessMdFlash *md, 
  const PebbleProcessInfo *info, int code_bank_num, PebbleTask task, uint8_t *build_id_buffer) {

  *md = (PebbleProcessMdFlash){};
  prv_init_from_info_common(&md->common, info, task, ProcessStorageFlash);

  // Flash app specific fields
  strncpy(md->name, info->name, sizeof(md->name));
  md->name[sizeof(md->name) - 1] = 0;

  md->size_bytes = info->virtual_size;

  md->process_version = info->process_version;
  md->sdk_version = info->sdk_version;

  md->code_bank_num = code_bank_num;
  md->res_bank_num = code_bank_num;

  md->res_version = (ResourceVersion) {
    .crc = info->resource_crc,
    .timestamp = info->resource_timestamp
  };

  if (build_id_buffer) {
    memcpy(md->build_id, build_id_buffer, BUILD_ID_EXPECTED_LEN);
  }
}

void process_metadata_init_with_resource_header(PebbleProcessMdResource *md,
    const PebbleProcessInfo *info, int bin_resource_id, PebbleTask task) {

  *md = (PebbleProcessMdResource){};
  prv_init_from_info_common(&md->common, info, task, ProcessStorageResource);

  // Resource app specific fields
  strncpy(md->name, info->name, sizeof(md->name));
  md->name[sizeof(md->name) - 1] = 0;

  md->size_bytes = info->virtual_size;
  md->bin_resource_id = bin_resource_id;
}

//////////////////////////////////
// PebbleProcessInfoFlags Helpers
//////////////////////////////////

ProcessVisibility process_metadata_flags_visibility(PebbleProcessInfoFlags flags) {
  if (flags & PROCESS_INFO_VISIBILITY_HIDDEN) {
    return ProcessVisibilityHidden;
  } else if (flags & PROCESS_INFO_VISIBILITY_SHOWN_ON_COMMUNICATION) {
    return ProcessVisibilityShownOnCommunication;
  } else {
    return ProcessVisibilityShown;
  }
}

ProcessType process_metadata_flags_process_type(PebbleProcessInfoFlags flags, PebbleTask task) {
  // Flags
  if (flags & PROCESS_INFO_WATCH_FACE) {
    return ProcessTypeWatchface;
  } else if (task == PebbleTask_Worker) {
    // BG_TODO: Set a bit in the PebbleProcessInfo to indicate it's a worker instead of having to
    // pass in the is_worker argument. Need to update process_metadata_get_flags_bitfield() to
    // match as well.
    return ProcessTypeWorker;
  } else {
    return ProcessTypeApp;
  }
}

bool process_metadata_flags_allow_js(PebbleProcessInfoFlags flags) {
  return ((flags & PROCESS_INFO_ALLOW_JS) != 0);
}

bool process_metadata_flags_has_worker(PebbleProcessInfoFlags flags) {
  return ((flags & PROCESS_INFO_HAS_WORKER) != 0);
}

bool process_metadata_flags_moddable_app(PebbleProcessInfoFlags flags) {
  return ((flags & PROCESS_INFO_MODDABLE_APP) != 0);
}

bool process_metadata_flags_rocky_app(PebbleProcessInfoFlags flags) {
  return ((flags & PROCESS_INFO_ROCKY_APP) != 0);
}

uint16_t process_metadata_flags_stored_sdk_platform(PebbleProcessInfoFlags flags) {
  return (flags & PROCESS_INFO_PLATFORM_MASK);
}

static const Version first_3x_version = {
  PROCESS_INFO_FIRST_3X_SDK_VERSION_MAJOR,
  PROCESS_INFO_FIRST_3X_SDK_VERSION_MINOR,
};
static const Version first_4x_version = {
  PROCESS_INFO_FIRST_4X_SDK_VERSION_MAJOR,
  PROCESS_INFO_FIRST_4X_SDK_VERSION_MINOR,
};
static const Version first_4_2_version = {
  PROCESS_INFO_FIRST_4_2_X_SDK_VERSION_MAJOR,
  PROCESS_INFO_FIRST_4_2_X_SDK_VERSION_MINOR,
};


PlatformType process_metadata_get_app_sdk_platform(const PebbleProcessMd *md) {
  if (!md->is_unprivileged) {
    return PBL_PLATFORM_TYPE_CURRENT;
  }

  const Version app_sdk_version = process_metadata_get_sdk_version(md);

  // 2.0 <= SDK < 3.0
  if (version_compare(app_sdk_version, first_3x_version) < 0) {
    // 2.x SDKs didn't support anything but Aplite
    return PlatformTypeAplite;
  }
  // 3.0 <= SDK < 4.0
  if (version_compare(app_sdk_version, first_4x_version) < 0) {
    return PBL_PLATFORM_SWITCH(PBL_PLATFORM_TYPE_CURRENT,
         /* aplite */  PlatformTypeAplite, // unreachable, since we don't build for Tintin anymore
         /* basalt */  PlatformTypeBasalt,
         /* chalk */   PlatformTypeChalk,
         /* diorite */ PlatformTypeAplite, // there's was no Diorite SDK prior to 4.0
         /* emery */   PlatformTypeBasalt,
         /* flint */   PlatformTypeAplite,
         /* gabbro */  PlatformTypeChalk);
  }
  // 4.0 <= SDK < 4.2
  if (version_compare(app_sdk_version, first_4_2_version) < 0) {
    return PBL_PLATFORM_SWITCH(PBL_PLATFORM_TYPE_CURRENT,
        /* aplite */  PlatformTypeAplite, // unreachable, since we don't build for Tintin anymore
        /* basalt */  PlatformTypeBasalt,
        /* chalk */   PlatformTypeChalk,
        /* diorite */ PlatformTypeDiorite, // there's was no Aplite SDK after 4.0
        /* emery */   PlatformTypeBasalt,
        /* flint */   PlatformTypeDiorite,
        /* gabbro */  PlatformTypeChalk);
  }

  // 4.2 <= SDK --> the flags should be filled correctly.
  const uint32_t stored_value = md->stored_sdk_platform;
  switch (stored_value) {
    case PROCESS_INFO_PLATFORM_APLITE:
      return PlatformTypeAplite;
    case PROCESS_INFO_PLATFORM_BASALT:
      return PlatformTypeBasalt;
    case PROCESS_INFO_PLATFORM_CHALK:
      return PlatformTypeChalk;
    case PROCESS_INFO_PLATFORM_DIORITE:
      return PlatformTypeDiorite;
    case PROCESS_INFO_PLATFORM_EMERY:
      return PlatformTypeEmery;
    case PROCESS_INFO_PLATFORM_FLINT:
      return PlatformTypeFlint;
    case PROCESS_INFO_PLATFORM_GABBRO:
      return PlatformTypeGabbro;
    default: {
      // If we encounter an unknown platform, we assume that it's meant for the current platform
      // (as it's most-likely a system-app). This is not a security risk as developers could always
      // patch the binaries as they wish anyway
      return PBL_PLATFORM_TYPE_CURRENT;
    }
  }
}

ProcessAppSDKType process_metadata_get_app_sdk_type(const PebbleProcessMd *md) {
  if (!md->is_unprivileged) {
    return ProcessAppSDKType_System;
  } else {
    const Version app_sdk_version = process_metadata_get_sdk_version(md);

    if (version_compare(app_sdk_version, first_3x_version) < 0) {
      return ProcessAppSDKType_Legacy2x;
    } else if (version_compare(app_sdk_version, first_4x_version) < 0) {
      return ProcessAppSDKType_Legacy3x;
    } else {
      return ProcessAppSDKType_4x;
    }
  }
}
