/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "app_glance_service.h"

#include "app_glance.h"
#include "app_glance_alarms.h"
#include "app_glance_generic.h"
#include "app_glance_music.h"
#include "app_glance_notifications.h"
#include "app_glance_settings.h"
#include "app_glance_watchfaces.h"
#include "app_glance_weather.h"
#include "app_glance_workout.h"
#include "menu_layer_private.h"

#include "applib/app_glance.h"
#include "applib/ui/kino/kino_reel.h"
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "system/passert.h"
#include "pbl/util/size.h"
#include "pbl/util/struct.h"
#include "pbl/util/uuid.h"

//! Cache twice the number of glances we'll show simultaneously in the launcher
#define LAUNCHER_APP_GLANCE_SERVICE_CACHE_NUM_ENTRIES (2 * LAUNCHER_MENU_LAYER_NUM_VISIBLE_ROWS)

typedef struct LauncherAppGlanceCacheEntry {
  ListNode node;
  LauncherAppGlance *glance;
} LauncherAppGlanceCacheEntry;

//////////////////////////////////////
// KinoPlayer callbacks

static void prv_glance_reel_player_frame_did_change_cb(KinoPlayer *player, void *context) {
  LauncherAppGlanceService *service = context;
  launcher_app_glance_service_notify_glance_changed(service);
}

///////////////////////////
// Slice expiration timer

static void prv_slice_expiration_timer_cb(void *data);

static void prv_reset_slice_expiration_timer(LauncherAppGlanceService *service) {
  if (!service) {
    return;
  }

  if (service->slice_expiration_timer) {
    app_timer_cancel(service->slice_expiration_timer);
    service->slice_expiration_timer = NULL;
  }

  // Set the next slice expiration time to "never"
  service->next_slice_expiration_time = APP_GLANCE_SLICE_NO_EXPIRATION;
}

static void prv_update_slice_expiration_timer_if_necessary(LauncherAppGlanceService *service,
                                                           time_t new_slice_expire_time) {
  const time_t next_slice_expiration_time = service->next_slice_expiration_time;
  const bool is_new_slice_expire_time_earlier_than_existing_earliest =
      (new_slice_expire_time != APP_GLANCE_SLICE_NO_EXPIRATION) &&
       ((next_slice_expiration_time == APP_GLANCE_SLICE_NO_EXPIRATION) ||
        (new_slice_expire_time < next_slice_expiration_time));
  if (!is_new_slice_expire_time_earlier_than_existing_earliest) {
    return;
  }

  const int time_until_slice_expires = new_slice_expire_time - rtc_get_time();
  // On the off chance that this slice has already expired, immediately call the timer callback
  if (time_until_slice_expires <= 0) {
    prv_slice_expiration_timer_cb(service);
    return;
  }

  const uint64_t time_until_slice_expires_ms = (uint64_t)time_until_slice_expires * MS_PER_SECOND;
  if (time_until_slice_expires_ms > UINT32_MAX) {
    // Slice expiration time is so far in the future that its offset in milliseconds from the
    // current time would overflow the argument to AppTimer, so just ignore this slice because it's
    // not worth setting a timer for it
    return;
  }

  prv_reset_slice_expiration_timer(service);

  service->slice_expiration_timer =
      app_timer_register((uint32_t)time_until_slice_expires_ms, prv_slice_expiration_timer_cb,
                         service);
  service->next_slice_expiration_time = new_slice_expire_time;
}

static bool prv_glance_cache_slice_expiration_foreach_cb(ListNode *node, void *context) {
  LauncherAppGlanceService *service = context;
  PBL_ASSERTN(service);

  const LauncherAppGlanceCacheEntry *entry = (LauncherAppGlanceCacheEntry *)node;
  LauncherAppGlance *glance = entry->glance;

  // Update the glance's current slice
  launcher_app_glance_update_current_slice(glance);

  // If necessary, update the slice expiration timer with the updated current slice
  prv_update_slice_expiration_timer_if_necessary(service, glance->current_slice.expiration_time);

  // Continue iterating until we've looked at all of the glances in the cache
  return true;
}

static void prv_slice_expiration_timer_cb(void *data) {
  LauncherAppGlanceService *service = data;
  PBL_ASSERTN(service);

  // Reset the timer
  prv_reset_slice_expiration_timer(service);

  // Iterate over the glances in the cache to find the next earliest expiring slice
  list_foreach(service->glance_cache, prv_glance_cache_slice_expiration_foreach_cb, service);
}

/////////////////////
// Glance cache

_Static_assert((offsetof(LauncherAppGlanceCacheEntry, node) == 0),
               "ListNode is not the first field of LauncherAppGlanceCacheEntry");

static void prv_glance_cache_destroy_entry(LauncherAppGlanceService *service,
                                           LauncherAppGlanceCacheEntry *entry) {
  if (!entry) {
    return;
  }

  if (service) {
    PBL_ASSERTN(entry->glance);
    KinoReel *glance_reel = entry->glance->reel;

    // Set the glance reel player's reel to NULL if it belongs to the glance we're going to destroy
    KinoPlayer *glance_reel_player = &service->glance_reel_player;
    if (glance_reel && (glance_reel == kino_player_get_reel(glance_reel_player))) {
      kino_player_set_reel(glance_reel_player, NULL, false /* take_ownership */);
    }
  }

  launcher_app_glance_destroy(entry->glance);
  app_free(entry);
}

static bool prv_glance_cache_deinit_foreach_cb(ListNode *node, void *context) {
  LauncherAppGlanceService *service = context;
  LauncherAppGlanceCacheEntry *entry = (LauncherAppGlanceCacheEntry *)node;

  prv_glance_cache_destroy_entry(service, entry);
  // Continue iterating to destroy all of the entries
  return true;
}

static void prv_glance_cache_deinit(LauncherAppGlanceService *service) {
  if (service) {
    list_foreach(service->glance_cache, prv_glance_cache_deinit_foreach_cb, service);
    service->glance_cache = NULL;
  }
}

//! Don't call this directly; it's used by prv_get_glance_for_node() below
static void prv_glance_cache_put(LauncherAppGlanceService *service, const Uuid *uuid,
                                 LauncherAppGlance *glance) {
  if (!service || !uuid || !glance) {
    return;
  }

  // If necessary, evict the LRU cache entry
  const uint32_t cache_entry_count = list_count(service->glance_cache);
  PBL_ASSERTN(cache_entry_count <= LAUNCHER_APP_GLANCE_SERVICE_CACHE_NUM_ENTRIES);
  if (cache_entry_count == LAUNCHER_APP_GLANCE_SERVICE_CACHE_NUM_ENTRIES) {
    LauncherAppGlanceCacheEntry *cache_entry_to_destroy =
        (LauncherAppGlanceCacheEntry *)list_get_tail(service->glance_cache);
    list_remove(&cache_entry_to_destroy->node, &service->glance_cache, NULL);
    prv_glance_cache_destroy_entry(service, cache_entry_to_destroy);
  }

  // Initialize a new cache entry, add it to the head of the cache list, and return it
  LauncherAppGlanceCacheEntry *new_cache_entry = app_zalloc_check(sizeof(*new_cache_entry));
  *new_cache_entry = (LauncherAppGlanceCacheEntry) {
    .glance = glance,
  };
  service->glance_cache = list_insert_before(service->glance_cache, &new_cache_entry->node);
}

static bool prv_glance_cache_entry_find_cb(ListNode *current_node, void *context) {
  LauncherAppGlanceCacheEntry *current_entry = (LauncherAppGlanceCacheEntry *)current_node;
  Uuid *uuid_to_find = context;
  return (current_entry && uuid_equal(&current_entry->glance->uuid, uuid_to_find));
}

static LauncherAppGlance *prv_load_glance_for_node(const AppMenuNode *node,
                                                   LauncherAppGlanceService *service) {
  typedef struct {
    Uuid uuid;
    LauncherAppGlance* (*constructor)(const AppMenuNode *);
  } SystemAppGlanceFactory;

  static const SystemAppGlanceFactory s_system_glance_factories[] = {
    { // Settings
      .uuid = {0x07, 0xe0, 0xd9, 0xcb, 0x89, 0x57, 0x4b, 0xf7,
               0x9d, 0x42, 0x35, 0xbf, 0x47, 0xca, 0xad, 0xfe},
      .constructor = launcher_app_glance_settings_create,
    },
    { // Music
      .uuid = {0x1f, 0x03, 0x29, 0x3d, 0x47, 0xaf, 0x4f, 0x28,
               0xb9, 0x60, 0xf2, 0xb0, 0x2a, 0x6d, 0xd7, 0x57},
      .constructor = launcher_app_glance_music_create,
    },
    { // Weather
      .uuid = {0x61, 0xb2, 0x2b, 0xc8, 0x1e, 0x29, 0x46, 0xd,
               0xa2, 0x36, 0x3f, 0xe4, 0x9, 0xa4, 0x39, 0xff},
      .constructor = launcher_app_glance_weather_create,
    },
    { // Notifications
      .uuid = {0xb2, 0xca, 0xe8, 0x18, 0x10, 0xf8, 0x46, 0xdf,
               0xad, 0x2b, 0x98, 0xad, 0x22, 0x54, 0xa3, 0xc1},
      .constructor = launcher_app_glance_notifications_create,
    },
    { // Alarms
      .uuid = {0x67, 0xa3, 0x2d, 0x95, 0xef, 0x69, 0x46, 0xd4,
               0xa0, 0xb9, 0x85, 0x4c, 0xc6, 0x2f, 0x97, 0xf9},
      .constructor = launcher_app_glance_alarms_create,
    },
    { // Watchfaces
      .uuid = {0x18, 0xe4, 0x43, 0xce, 0x38, 0xfd, 0x47, 0xc8,
               0x84, 0xd5, 0x6d, 0x0c, 0x77, 0x5f, 0xbe, 0x55},
      .constructor = launcher_app_glance_watchfaces_create,
    },
    {
      // Workout
      .uuid = {0xfe, 0xf8, 0x2c, 0x82, 0x71, 0x76, 0x4e, 0x22,
               0x88, 0xde, 0x35, 0xa3, 0xfc, 0x18, 0xd4, 0x3f},
      .constructor = launcher_app_glance_workout_create,
    },
  };

  LauncherAppGlance *glance = NULL;

  // Check if the UUID matches a known system glance
  for (unsigned int i = 0; i < ARRAY_LENGTH(s_system_glance_factories); i++) {
    const SystemAppGlanceFactory *factory = &s_system_glance_factories[i];
    if (uuid_equal(&factory->uuid, &node->uuid)) {
      glance = factory->constructor(node);
      break;
    }
  }

  // If we haven't loaded a glance yet, try loading a generic glance for the node
  if (!glance) {
    glance = launcher_app_glance_generic_create(node, service->generic_glance_icon,
                                                service->generic_glance_icon_resource_id);
  }

  // If we successfully loaded a glance, set its service field
  if (glance) {
    glance->service = service;
  }

  return glance;
}

static LauncherAppGlanceCacheEntry *prv_find_glance_entry_in_cache(
  LauncherAppGlanceService *service, Uuid *uuid) {
  return (LauncherAppGlanceCacheEntry *)list_find(service->glance_cache,
                                                  prv_glance_cache_entry_find_cb, uuid);
}

static LauncherAppGlance *prv_find_glance_in_cache(LauncherAppGlanceService *service, Uuid *uuid) {
  const LauncherAppGlanceCacheEntry *entry = prv_find_glance_entry_in_cache(service, uuid);
  return NULL_SAFE_FIELD_ACCESS(entry, glance, NULL);
}

//! Request a glance for an icon ID from an "MRU linked list" (list sorted by accesses so that
//! most recent accesses are at the head of the list)
static LauncherAppGlance *prv_fetch_from_cache_or_load_glance_for_node(AppMenuNode *node,
    LauncherAppGlanceService *service) {
  if (!service || !node) {
    return NULL;
  }

  Uuid *uuid = &node->uuid;

  LauncherAppGlance *glance = NULL;

  // Try to find the requested glance in the cache
  LauncherAppGlanceCacheEntry *cache_entry = prv_find_glance_entry_in_cache(service, uuid);
  if (cache_entry) {
    // Move the found cache entry to the front of the cache list (to mark it as "MRU")
    // This makes it easy to remove the "LRU" entry later by simply removing the tail
    list_remove(&cache_entry->node, &service->glance_cache, NULL);
    service->glance_cache = list_insert_before(service->glance_cache, &cache_entry->node);
    glance = cache_entry->glance;
  }

  // Try to load the glance requested if we didn't find it in the cache
  if (!glance) {
    glance = prv_load_glance_for_node(node, service);
    if (!glance) {
      // Just bail out and don't modify the cache if we fail
      return NULL;
    }

    // Add the new glance to the cache
    prv_glance_cache_put(service, uuid, glance);
  }

  // Update the slice expiration timer if the glance's current slice expires soon
  prv_update_slice_expiration_timer_if_necessary(service, glance->current_slice.expiration_time);

  return glance;
}

static bool prv_should_use_glance_cache_for_app_with_uuid(const Uuid *uuid) {
  // Use the glance cache only if the app does not have the system UUID (all zeros)
  return !uuid_is_system(uuid);
}

/////////////////////
// Glance events

static void prv_handle_glance_event(PebbleEvent *event, void *context) {
  LauncherAppGlanceService *service = context;
  PBL_ASSERTN(service);

  // Update the current slice of the glance that was changed if the glance is in the cache
  LauncherAppGlance *glance_in_cache = prv_find_glance_in_cache(service,
                                                                event->app_glance.app_uuid);
  if (glance_in_cache) {
    launcher_app_glance_update_current_slice(glance_in_cache);

    // If necessary, update the slice expiration timer with the updated current slice
    prv_update_slice_expiration_timer_if_necessary(service,
                                                   glance_in_cache->current_slice.expiration_time);
  }
}

/////////////////////
// Public API

void launcher_app_glance_service_draw_glance_for_app_node(LauncherAppGlanceService *service,
                                                          GContext *ctx, const GRect *frame,
                                                          bool is_highlighted,
                                                          int16_t screen_center_y,
                                                          AppMenuNode *node) {
  const bool use_glance_cache = prv_should_use_glance_cache_for_app_with_uuid(&node->uuid);

  LauncherAppGlance *glance =
      use_glance_cache ? prv_fetch_from_cache_or_load_glance_for_node(node, service) :
                         prv_load_glance_for_node(node, service);

  // Set the screen Y position for arc effect on round displays
  if (glance) {
    glance->screen_center_y = screen_center_y;
  }

  // Draw the glance in the provided frame
  launcher_app_glance_draw(ctx, frame, glance, is_highlighted);

  // If we didn't use the glance cache, destroy the glance now
  if (!use_glance_cache) {
    launcher_app_glance_destroy(glance);
  }
}

void launcher_app_glance_service_rewind_current_glance(LauncherAppGlanceService *service) {
  if (!service) {
    return;
  }
  kino_player_rewind(&service->glance_reel_player);
}

void launcher_app_glance_service_pause_current_glance(LauncherAppGlanceService *service) {
  if (!service) {
    return;
  }
  kino_player_pause(&service->glance_reel_player);
}

void launcher_app_glance_service_play_current_glance(LauncherAppGlanceService *service) {
  if (!service) {
    return;
  }
  kino_player_play(&service->glance_reel_player);
}

void launcher_app_glance_service_play_glance_for_app_node(LauncherAppGlanceService *service,
                                                          AppMenuNode *node) {
  if (!service || !node) {
    return;
  }

  KinoPlayer *glance_reel_player = &service->glance_reel_player;

  // Rewind the player for any previously played glance
  kino_player_rewind(glance_reel_player);

  const bool use_glance_cache = prv_should_use_glance_cache_for_app_with_uuid(&node->uuid);
  if (!use_glance_cache) {
    // Don't play glances that we don't store in the cache since they don't live long enough
    // to advance frames
    return;
  }

  LauncherAppGlance *glance = prv_fetch_from_cache_or_load_glance_for_node(node, service);
  PBL_ASSERTN(glance);
  kino_player_set_reel(glance_reel_player, glance->reel, false /* take_ownership */);
  kino_player_play(glance_reel_player);
}

void launcher_app_glance_service_notify_glance_changed(LauncherAppGlanceService *service) {
  if (service && service->handlers.glance_changed) {
    service->handlers.glance_changed(service->handlers_context);
  }
}

void launcher_app_glance_service_init(LauncherAppGlanceService *service,
                                      uint32_t generic_glance_icon_resource_id) {
  if (!service) {
    return;
  }

  *service = (LauncherAppGlanceService) {};

  prv_reset_slice_expiration_timer(service);

  service->glance_event_info = (EventServiceInfo) {
    .type = PEBBLE_APP_GLANCE_EVENT,
    .handler = prv_handle_glance_event,
    .context = service,
  };
  event_service_client_subscribe(&service->glance_event_info);

  service->generic_glance_icon = kino_reel_create_with_resource(generic_glance_icon_resource_id);
  PBL_ASSERTN(service->generic_glance_icon);
  service->generic_glance_icon_resource_id = generic_glance_icon_resource_id;

  const KinoPlayerCallbacks glance_reel_player_callbacks = (KinoPlayerCallbacks) {
    .frame_did_change = prv_glance_reel_player_frame_did_change_cb,
  };
  kino_player_set_callbacks(&service->glance_reel_player, glance_reel_player_callbacks, service);
}

void launcher_app_glance_service_set_handlers(LauncherAppGlanceService *service,
                                              const LauncherAppGlanceServiceHandlers *handlers,
                                              void *context) {
  if (!service) {
    return;
  } else if (!handlers) {
    service->handlers = (LauncherAppGlanceServiceHandlers) {};
  } else {
    service->handlers = *handlers;
  }
  service->handlers_context = context;
}

void launcher_app_glance_service_deinit(LauncherAppGlanceService *service) {
  if (!service) {
    return;
  }

  kino_player_deinit(&service->glance_reel_player);
  event_service_client_unsubscribe(&service->glance_event_info);
  prv_glance_cache_deinit(service);
  prv_reset_slice_expiration_timer(service);
  kino_reel_destroy(service->generic_glance_icon);
}
