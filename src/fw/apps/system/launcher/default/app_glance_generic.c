/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "app_glance_generic.h"

#include "app_glance_structured.h"

#include "applib/app_glance.h"
#include "applib/app_timer.h"
#include "applib/template_string.h"
#include "applib/ui/kino/kino_reel.h"
#include "apps/system/timeline/text_node.h"
#include "kernel/pbl_malloc.h"
#include "process_management/app_install_manager.h"
#include "process_management/pebble_process_info.h"
#include "pbl/services/timeline/timeline_resources.h"
#include "system/passert.h"
#include "pbl/util/struct.h"

#define APP_GLANCE_MIN_SUPPORTED_SDK_VERSION_MAJOR (PROCESS_INFO_FIRST_4X_SDK_VERSION_MAJOR)
#define APP_GLANCE_MIN_SUPPORTED_SDK_VERSION_MINOR (PROCESS_INFO_FIRST_4X_SDK_VERSION_MINOR)

typedef struct LauncherAppGlanceGeneric {
  //! The title that will be displayed
  char title_buffer[APP_NAME_SIZE_BYTES];
  //! The icon that will be displayed
  KinoReel *displayed_icon;
  //! The resource info of the displayed icon
  AppResourceInfo displayed_icon_resource_info;
  //! The resource info of the default app icon
  AppResourceInfo default_icon_resource_info;
  //! Fallback icon to use if other icons aren't available; owned by client
  const KinoReel *fallback_icon;
  //! The resource ID of the fallback icon; used for comparisons
  uint32_t fallback_icon_resource_id;
  //! App timer used for re-evaluating the current slice's subtitle template string
  AppTimer *slice_subtitle_template_string_reeval_timer;
  //! UTC timestamp of when the current slice's subtitle template string must be re-evaluated
  //! A zero value indicates that there is no need to re-evaluate the subtitle template string
  time_t next_slice_subtitle_template_string_reeval_time;
  //! Whether to use the legacy 28x28 icon size limit
  bool use_legacy_28x28_icon_size_limit;
} LauncherAppGlanceGeneric;

static KinoReel *prv_get_icon(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceGeneric *generic_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  return NULL_SAFE_FIELD_ACCESS(generic_glance, displayed_icon, NULL);
}

static void prv_generic_glance_destroy_displayed_icon(LauncherAppGlanceGeneric *generic_glance) {
  // Only delete the displayed icon if it doesn't match the fallback icon because we don't own it
  if (generic_glance && (generic_glance->displayed_icon != generic_glance->fallback_icon)) {
    kino_reel_destroy(generic_glance->displayed_icon);
  }
}

static KinoReel *prv_create_glance_icon(const AppResourceInfo *res_info,
                                        bool legacy_icon_size_limit) {
  if (!res_info) {
    return NULL;
  }

  KinoReel *icon = kino_reel_create_with_resource_system(res_info->res_app_num, res_info->res_id);
  if (!icon) {
    return NULL;
  }

  const GSize size = kino_reel_get_size(icon);
  // Not const to deal with horrifying GCC bug
  GSize max_size = legacy_icon_size_limit ? LAUNCHER_APP_GLANCE_STRUCTURED_ICON_LEGACY_MAX_SIZE
                                          : LAUNCHER_APP_GLANCE_STRUCTURED_ICON_MAX_SIZE;
  if ((size.w > max_size.w) ||
      (size.h > max_size.h)) {
    // The icon is too big
    kino_reel_destroy(icon);
    return NULL;
  }

  return icon;
}

static bool prv_app_resource_info_equal(const AppResourceInfo *a, const AppResourceInfo *b) {
  PBL_ASSERTN(a && b);
  return ((a == b) || ((a->res_id == b->res_id) && (a->res_app_num == b->res_app_num)));
}

static void prv_generic_glance_set_icon(LauncherAppGlanceGeneric *generic_glance,
                                        const AppResourceInfo *res_info) {
  if (!generic_glance || !res_info) {
    return;
  }

  const bool is_requested_resource_the_default_icon =
      ((res_info->res_app_num == generic_glance->default_icon_resource_info.res_app_num) &&
        ((res_info->res_id == APP_GLANCE_SLICE_DEFAULT_ICON) ||
         (res_info->res_id == generic_glance->default_icon_resource_info.res_id)));
  const bool does_default_icon_need_to_be_loaded =
      (is_requested_resource_the_default_icon &&
       !prv_app_resource_info_equal(&generic_glance->displayed_icon_resource_info,
                                    &generic_glance->default_icon_resource_info));
  const bool is_icon_stale =
      (!prv_app_resource_info_equal(&generic_glance->displayed_icon_resource_info, res_info));

  if (generic_glance->displayed_icon &&
      !does_default_icon_need_to_be_loaded &&
      !is_icon_stale) {
    // Nothing to do, bail out
    return;
  }

  // Destroy the currently displayed icon
  prv_generic_glance_destroy_displayed_icon(generic_glance);

  AppResourceInfo res_info_to_load = *res_info;

  // Set the resource info to the real default icon resource info if the default icon was requested
  if (is_requested_resource_the_default_icon) {
    res_info_to_load = generic_glance->default_icon_resource_info;
  }

  const bool legacy_icon_size_limit = generic_glance->use_legacy_28x28_icon_size_limit;
  // Try loading the requested icon
  generic_glance->displayed_icon = prv_create_glance_icon(&res_info_to_load,
                                                          legacy_icon_size_limit);

  if (!generic_glance->displayed_icon) {
    // Try again with the app's default icon if we didn't just try it
    if (!prv_app_resource_info_equal(&res_info_to_load,
                                     &generic_glance->default_icon_resource_info)) {
      res_info_to_load = generic_glance->default_icon_resource_info;
      generic_glance->displayed_icon = prv_create_glance_icon(&res_info_to_load,
                                                              legacy_icon_size_limit);
    }

    // If we don't have a valid icon at this point, use the fallback icon (casting to non-const so
    // we can use it)
    if (!generic_glance->displayed_icon && generic_glance->fallback_icon) {
      // Note that this (reasonably) assumes that the system fallback icon is a system icon
      res_info_to_load = (AppResourceInfo) {
        .res_app_num = SYSTEM_APP,
        .res_id = generic_glance->fallback_icon_resource_id,
      };
      generic_glance->displayed_icon = (KinoReel *)generic_glance->fallback_icon;
    }
  }

  // We require that we have some sort of icon at this point
  PBL_ASSERTN(generic_glance->displayed_icon);

  // Update our recording of the resource info of the displayed icon
  generic_glance->displayed_icon_resource_info = res_info_to_load;
}

static void prv_cancel_subtitle_reeval_timer(LauncherAppGlanceGeneric *generic_glance) {
  if (!generic_glance) {
    return;
  }

  if (generic_glance->slice_subtitle_template_string_reeval_timer) {
    app_timer_cancel(generic_glance->slice_subtitle_template_string_reeval_timer);
    generic_glance->slice_subtitle_template_string_reeval_timer = NULL;
  }

  // Set the next re-evaluation time to "never"
  generic_glance->next_slice_subtitle_template_string_reeval_time = 0;
}

static void prv_subtitle_reeval_timer_cb(void *data) {
  LauncherAppGlanceStructured *structured_glance = data;
  PBL_ASSERTN(structured_glance);
  LauncherAppGlanceGeneric *generic_glance =
      launcher_app_glance_structured_get_data(structured_glance);

  // Reset the timer
  generic_glance->slice_subtitle_template_string_reeval_timer = NULL;
  prv_cancel_subtitle_reeval_timer(generic_glance);

  // Notify the service that the glance changed
  launcher_app_glance_structured_notify_service_glance_changed(structured_glance);
}


static void prv_update_subtitle_template_string_reeval_timer_if_necessary(
    LauncherAppGlanceStructured *structured_glance, time_t new_reeval_time) {
  LauncherAppGlanceGeneric *generic_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  const time_t existing_reeval_time =
      generic_glance->next_slice_subtitle_template_string_reeval_time;
  // Bail out if the new re-evaluation time is not earlier than the existing one we have
  if ((new_reeval_time == 0) ||
      ((existing_reeval_time != 0) && (new_reeval_time >= existing_reeval_time))) {
    return;
  }

  const int time_until_next_reeval = new_reeval_time - rtc_get_time();
  // On the off chance that we missed the reeval, immediately call the timer callback
  if (time_until_next_reeval <= 0) {
    prv_subtitle_reeval_timer_cb(structured_glance);
    return;
  }

  const uint64_t time_until_next_reeval_ms = (uint64_t)time_until_next_reeval * MS_PER_SECOND;
  if (time_until_next_reeval_ms > UINT32_MAX) {
    // Next reeval time is so far in the future that its offset in milliseconds from the
    // current time would overflow the argument to AppTimer, so just ignore this reeval because it's
    // not worth setting a timer for it
    return;
  }

  prv_cancel_subtitle_reeval_timer(generic_glance);

  generic_glance->slice_subtitle_template_string_reeval_timer =
    app_timer_register((uint32_t)time_until_next_reeval_ms, prv_subtitle_reeval_timer_cb,
                       structured_glance);
  generic_glance->next_slice_subtitle_template_string_reeval_time = new_reeval_time;
}

static void prv_current_slice_updated(LauncherAppGlance *glance) {
  // Ignore slices that aren't of the IconAndSubtitle type beyond this point for now
  if (glance->current_slice.type != AppGlanceSliceType_IconAndSubtitle) {
    return;
  }

  LauncherAppGlanceStructured *structured_glance = (LauncherAppGlanceStructured *)glance;
  LauncherAppGlanceGeneric *generic_glance =
      launcher_app_glance_structured_get_data(structured_glance);

  const TimelineResourceId timeline_res_id =
      (TimelineResourceId)glance->current_slice.icon_and_subtitle.icon_resource_id;

  // Initialize the resource info to be the default icon
  AppResourceInfo resource_info = generic_glance->default_icon_resource_info;
  // Override it if we have a valid timeline resource ID from the new app glance slice
  if (timeline_res_id != APP_GLANCE_SLICE_DEFAULT_ICON) {
    // NOTE: This variant of the timeline_resources_get_id() function is safe to call here with
    // respect to the app supporting published resources because we only consider slices if the
    // glance is for a system app (where it doesn't matter) or for apps that were compiled with
    // an SDK that supports app glances (which is newer than the first SDK that supported published
    // resources as proved by the following asserts)
    _Static_assert((APP_GLANCE_MIN_SUPPORTED_SDK_VERSION_MAJOR >
                        TIMELINE_RESOURCE_PBW_SUPPORT_FIRST_SDK_VERSION_MAJOR) ||
                   ((APP_GLANCE_MIN_SUPPORTED_SDK_VERSION_MAJOR ==
                         TIMELINE_RESOURCE_PBW_SUPPORT_FIRST_SDK_VERSION_MAJOR) &&
                    (APP_GLANCE_MIN_SUPPORTED_SDK_VERSION_MINOR >=
                         TIMELINE_RESOURCE_PBW_SUPPORT_FIRST_SDK_VERSION_MINOR)),
                   "App glance min supported SDK version must be equal to or newer than first "
                   "timeline/published resource PBW supported SDK version");
    timeline_resources_get_id_system(timeline_res_id, LAUNCHER_APP_GLANCE_GENERIC_ICON_SIZE_TYPE,
                                     resource_info.res_app_num, &resource_info);
  }

  prv_generic_glance_set_icon(generic_glance, &resource_info);

  prv_cancel_subtitle_reeval_timer(generic_glance);

  // The glance will automatically be redrawn after this function is called (which will also update
  // the glance's state regarding its subtitle template string), so no need to mark it as dirty
  // (see launcher_app_glance_update_current_slice())
}

static const char *prv_get_title(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceGeneric *generic_glance =
    launcher_app_glance_structured_get_data(structured_glance);
  return NULL_SAFE_FIELD_ACCESS(generic_glance, title_buffer, NULL);
}

static void prv_generic_glance_dynamic_text_node_update(PBL_UNUSED GContext *ctx,
                                                        PBL_UNUSED GTextNode *node,
                                                        PBL_UNUSED const GRect *box,
                                                        PBL_UNUSED const GTextNodeDrawConfig *config,
                                                        PBL_UNUSED bool render, char *buffer,
                                                        size_t buffer_size, void *user_data) {
  LauncherAppGlanceStructured *structured_glance = user_data;
  if (!structured_glance) {
    return;
  } else if (structured_glance->glance.current_slice.type != AppGlanceSliceType_IconAndSubtitle) {
    PBL_LOG_WRN("Generic glance doesn't know how to handle slice type %d",
            structured_glance->glance.current_slice.type);
    return;
  }

  // Evaluate the subtitle as a template string
  const char *subtitle_template_string =
      structured_glance->glance.current_slice.icon_and_subtitle.template_string;
  TemplateStringEvalConditions template_string_reeval_conditions = {0};
  const TemplateStringVars template_string_vars = (TemplateStringVars) {
    .current_time = rtc_get_time(),
  };
  TemplateStringError template_string_error = {0};
  template_string_evaluate(subtitle_template_string, buffer, buffer_size,
                           &template_string_reeval_conditions, &template_string_vars,
                           &template_string_error);

  if (template_string_error.status != TemplateStringErrorStatus_Success) {
    // Zero out the buffer and return
    buffer[0] = '\0';
    PBL_LOG_WRN("Error at index %zu in evaluating template string: %s",
            template_string_error.index_in_string, subtitle_template_string);
    return;
  }

  // Update the timer for re-evaluating the template string, if necessary
  prv_update_subtitle_template_string_reeval_timer_if_necessary(
      structured_glance, template_string_reeval_conditions.eval_time);
}

static GTextNode *prv_create_subtitle_node(LauncherAppGlanceStructured *structured_glance) {
  return launcher_app_glance_structured_create_subtitle_text_node(
      structured_glance, prv_generic_glance_dynamic_text_node_update);
}

static void prv_destructor(LauncherAppGlanceStructured *structured_glance) {
  LauncherAppGlanceGeneric *generic_glance =
      launcher_app_glance_structured_get_data(structured_glance);
  prv_cancel_subtitle_reeval_timer(generic_glance);
  prv_generic_glance_destroy_displayed_icon(generic_glance);
  app_free(generic_glance);
}

static const LauncherAppGlanceStructuredImpl s_generic_structured_glance_impl = {
  .base_handlers.current_slice_updated = prv_current_slice_updated,
  .get_icon = prv_get_icon,
  .get_title = prv_get_title,
  .create_subtitle_node = prv_create_subtitle_node,
  .destructor = prv_destructor,
};

LauncherAppGlance *launcher_app_glance_generic_create(const AppMenuNode *node,
                                                      const KinoReel *fallback_icon,
                                                      uint32_t fallback_icon_resource_id) {
  if (!node) {
    return NULL;
  }

  LauncherAppGlanceGeneric *generic_glance = app_zalloc_check(sizeof(*generic_glance));
  const size_t title_buffer_size = sizeof(generic_glance->title_buffer);
  strncpy(generic_glance->title_buffer, node->name, title_buffer_size);
  generic_glance->title_buffer[title_buffer_size - 1] = '\0';
  generic_glance->default_icon_resource_info = (AppResourceInfo) {
    .res_app_num = node->app_num,
    .res_id = node->icon_resource_id,
  };
  generic_glance->fallback_icon = fallback_icon;
  generic_glance->fallback_icon_resource_id = fallback_icon_resource_id;

  const Version app_glance_min_supported_sdk_version = (Version) {
    .major = APP_GLANCE_MIN_SUPPORTED_SDK_VERSION_MAJOR,
    .minor = APP_GLANCE_MIN_SUPPORTED_SDK_VERSION_MINOR,
  };

  const bool app_glances_supported =
      (version_compare(node->sdk_version, app_glance_min_supported_sdk_version) >= 0);

  generic_glance->use_legacy_28x28_icon_size_limit = !app_glances_supported;

  // Our unit tests rely on system app icons for testing generic glances which means
  // the != SYSTEM_APP condition can't be satisfied easily, so just skip this part for unit tests
#if !UNITTEST
  // Only consider slices for non-system apps that were compiled with an SDK that supports glances
  const bool should_consider_slices = (node->app_num != SYSTEM_APP) &&
                                      app_glances_supported;
#else
  const bool should_consider_slices = true;
#endif

  prv_generic_glance_set_icon(generic_glance, &generic_glance->default_icon_resource_info);

  LauncherAppGlanceStructured *structured_glance =
      launcher_app_glance_structured_create(&node->uuid, &s_generic_structured_glance_impl,
                                            should_consider_slices, generic_glance);
  if (structured_glance) {
    if (generic_glance->use_legacy_28x28_icon_size_limit) {
      launcher_app_glance_structured_set_icon_max_size(structured_glance,
          LAUNCHER_APP_GLANCE_STRUCTURED_ICON_LEGACY_MAX_SIZE);
    }
    return &structured_glance->glance;
  } else {
    return NULL;
  }
}
