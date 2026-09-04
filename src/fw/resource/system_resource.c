/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "system_resource.h"

#include "applib/fonts/fonts.h"
#include "applib/graphics/text_resources.h"
#include "kernel/event_loop.h"
#include "kernel/memory_layout.h"
#include "kernel/panic.h"
#include "kernel/util/fw_reset.h"
#include "resource/resource.h"
#include "resource/resource_storage.h"
#include "syscall/syscall_internal.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/size.h"

#include "resource/resource_ids.auto.h"
#include "resource/resource_version.auto.h"
#include "font_resource_table.auto.h"

void system_resource_init(void) {
  if (!resource_init_app(SYSTEM_APP, &SYSTEM_RESOURCE_VERSION)) {
    // System resources are missing!
#if defined(CONFIG_IS_BIGBOARD)
    static const uint32_t ERROR_BAD_RESOURCES = 0xfe504505;
    pbl_log(LOG_LEVEL_ERROR, __FILE_NAME__, __LINE__,
        "System resources are missing or corrupt, time to sad watch");
    launcher_panic(ERROR_BAD_RESOURCES);
#else
    PBL_LOG_ERR("System resources are missing or corrupt! Going to PRF");
    fw_reset_into_prf();
#endif
  }
}

bool system_resource_is_valid(void) {
  return resource_init_app(SYSTEM_APP, &SYSTEM_RESOURCE_VERSION);
}

#define NUM_SYSTEM_FONTS ARRAY_LENGTH(s_font_resource_keys)

// Total number of fonts = NUM_SYSTEM_FONTS + 1 for the fallback font
FontInfo s_system_fonts_info_table[NUM_SYSTEM_FONTS + 1] KERNEL_READONLY_DATA;

static GFont prv_load_system_font(const char *font_key) {
  if (font_key == NULL) {
    PBL_LOG_DBG("GETTING FALLBACK FONT");
    // load fallback font
    if (!s_system_fonts_info_table[NUM_SYSTEM_FONTS].loaded) {
      PBL_ASSERTN(text_resources_init_font(SYSTEM_APP, RESOURCE_ID_FONT_FALLBACK_INTERNAL, 0,
          &s_system_fonts_info_table[NUM_SYSTEM_FONTS]));
    }
    return &s_system_fonts_info_table[NUM_SYSTEM_FONTS];
  }

  for (int i = 0; i < (int) NUM_SYSTEM_FONTS; ++i) {
    if (0 == strcmp(font_key, s_font_resource_keys[i].key_name)) {
      FontInfo *fontinfo = &s_system_fonts_info_table[i];
      uint32_t resource = s_font_resource_keys[i].resource_id;
      uint32_t extension = s_font_resource_keys[i].extension_id;
      // if the font has not been initialized yet
      if (!fontinfo->loaded) {
        if (!text_resources_init_font(SYSTEM_APP,
            resource, extension, &s_system_fonts_info_table[i])) {
          // Can't initialize the font for some reason
          return NULL;
        }
        resource_get_and_cache(SYSTEM_APP, resource);
        resource_get_and_cache(SYSTEM_APP, extension);
      }
      return &s_system_fonts_info_table[i];
    }
  }

  // Didn't find the given font, invalid key.
  return NULL;
}

GFont system_resource_get_font(const char *font_key) {
  GFont result = prv_load_system_font(font_key);
  return result;
}

DEFINE_SYSCALL(GFont, sys_font_get_system_font, const char *font_key) {
  if (font_key && PRIVILEGE_WAS_ELEVATED) {
    if (!memory_layout_is_cstring_in_region(memory_layout_get_app_region(), font_key, 100) &&
        !memory_layout_is_cstring_in_region(memory_layout_get_microflash_region(), font_key, 100)) {
      PBL_LOG_ERR("Pointer %p not in app or microflash region", font_key);
      syscall_failed();
    }
  }

  return system_resource_get_font(font_key);
}

DEFINE_SYSCALL(void, sys_font_reload_font, FontInfo *fontinfo) {
  if (PRIVILEGE_WAS_ELEVATED) {
    if (!memory_layout_is_pointer_in_region(memory_layout_get_readonly_bss_region(), fontinfo)) {
      syscall_failed();
    }
  }

  text_resources_init_font(fontinfo->base.app_num, fontinfo->base.resource_id,
      fontinfo->extension.resource_id, fontinfo);
}

