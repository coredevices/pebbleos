/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/fonts/fonts.h"
#include "applib/graphics/framebuffer.h"
#include "applib/graphics/graphics.h"
#include "applib/ui/window_private.h"
#include "pbl/services/imaging.h"
#include "pbl/services/music.h"
#include "pbl/util/size.h"
#include "shell/system_theme.h"

#include "clar.h"

#include <stdio.h>
#include <string.h>

// Fakes
/////////////////////

#include "fake_spi_flash.h"
#include "fixtures/load_test_resources.h"

// Stubs
/////////////////////

#include "stubs_analytics.h"
#include "stubs_animation_timing.h"
#include "stubs_app_install_manager.h"
#include "stubs_app_state.h"
#include "stubs_app_timer.h"
#include "stubs_app_window_stack.h"
#include "stubs_bootbits.h"
#include "stubs_click.h"
#include "stubs_event_service_client.h"
#include "stubs_i18n.h"
#include "stubs_layer.h"
#include "stubs_logging.h"
#include "stubs_memory_layout.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"
#include "stubs_pebble_tasks.h"
#include "stubs_process_manager.h"
#include "stubs_prompt.h"
#include "stubs_serial.h"
#include "stubs_sleep.h"
#include "stubs_syscalls.h"
#include "stubs_task_watchdog.h"
#include "stubs_vibe_score.h"
#include "stubs_window_manager.h"
#include "stubs_window_stack.h"

// Music service fake
/////////////////////

static char s_music_title[MUSIC_BUFFER_LENGTH];
static char s_music_artist[MUSIC_BUFFER_LENGTH];
static MusicPlayState s_music_play_state;
static uint32_t s_music_track_pos_ms;
static uint32_t s_music_track_length_ms;
static bool s_music_needs_user_to_start_playback;
static bool s_music_progress_supported;

void music_get_now_playing(char *title, char *artist, char *album) {
  if (title) {
    strcpy(title, s_music_title);
  }
  if (artist) {
    strcpy(artist, s_music_artist);
  }
  if (album) {
    album[0] = '\0';
  }
}

MusicPlayState music_get_playback_state(void) {
  return s_music_play_state;
}

void music_get_pos(uint32_t *track_pos_ms, uint32_t *track_length_ms) {
  *track_pos_ms = s_music_track_pos_ms;
  *track_length_ms = s_music_track_length_ms;
}

bool music_is_progress_reporting_supported(void) {
  return s_music_progress_supported;
}

bool music_needs_user_to_start_playback_on_phone(void) {
  return s_music_needs_user_to_start_playback;
}

bool music_is_command_supported(MusicCommand command) {
  return true;
}

void music_command_send(MusicCommand command) {}

void music_request_reduced_latency(bool reduced_latency) {}

void music_request_low_latency_for_period(uint32_t period_ms) {}

bool music_has_now_playing(void) {
  return s_music_title[0] != '\0' || s_music_artist[0] != '\0';
}

uint8_t music_get_now_playing_generation(void) {
  return 0;
}

// Album art (no art in these tests, so the text layout is exercised)
const struct GBitmap *music_album_art_lock(void) {
  return NULL;
}

void music_album_art_unlock(void) {}

bool music_album_art_is_current(void) {
  return true;
}

// Album art is fetched via the generic imaging service; these tests never show art.
bool imaging_is_type_supported(ImagingImageType image_type) {
  return false;
}

bool imaging_request_album_art(uint8_t token, ImagingFormat format, uint16_t width, uint16_t height,
                               const char *title, const char *artist) {
  return false;
}

// Shell prefs fake
/////////////////////

static bool s_prefs_music_show_volume_controls;
static bool s_prefs_music_show_progress_bar;

bool shell_prefs_get_music_show_volume_controls(void) {
  return s_prefs_music_show_volume_controls;
}

bool shell_prefs_get_music_show_progress_bar(void) {
  return s_prefs_music_show_progress_bar;
}

bool shell_prefs_get_music_show_album_art(void) {
  return false;
}

// Misc stubs
/////////////////////

void app_event_loop(void) {}

void tick_timer_service_subscribe(TimeUnits tick_units, TickHandler handler) {}

void tick_timer_service_unsubscribe(void) {}

void accel_tap_service_subscribe(AccelTapHandler handler) {}

void accel_tap_service_unsubscribe(void) {}

VibeScore *vibe_score_create_with_resource(uint32_t resource_id) {
  return NULL;
}

PropertyAnimation *property_animation_create_bounds_origin(struct Layer *layer, GPoint *from,
                                                           GPoint *to) {
  return NULL;
}

void property_animation_update_grect(PropertyAnimation *property_animation,
                                     const uint32_t distance_normalized) {}

bool scroll_layer_is_instance(const Layer *layer) {
  return false;
}

uint16_t time_ms(time_t *tloc, uint16_t *out_ms) {
  return 0;
}

void clock_copy_time_string(char *buffer, uint8_t size) {
  strncpy(buffer, "12:00 PM", size);
}

// Mirror the real system theme font table for the default content size, so
// text renders exactly as it does on the target platform.
GFont system_theme_get_font_for_default_size(TextStyleFont font) {
  const bool large = (PreferredContentSizeDefault == PreferredContentSizeLarge);
  const char *key;
  switch (font) {
    case TextStyleFont_Header:
      key = large ? FONT_KEY_GOTHIC_24_BOLD : FONT_KEY_GOTHIC_18_BOLD;
      break;
    case TextStyleFont_Subtitle:
      key = large ? FONT_KEY_GOTHIC_28 : FONT_KEY_GOTHIC_24_BOLD;
      break;
    default:
      key = FONT_KEY_GOTHIC_18_BOLD;
      break;
  }
  return fonts_get_system_font(key);
}

// Helper Functions
/////////////////////

#include "fw/graphics/util.h"

// App under test
/////////////////////

#include "apps/system/music.c"

// Setup and Teardown
////////////////////////////////////

static GContext s_ctx;
static FrameBuffer s_fb;

GContext *graphics_context_get_current_context(void) {
  return &s_ctx;
}

void test_music__initialize(void) {
  s_music_title[0] = '\0';
  s_music_artist[0] = '\0';
  s_music_play_state = MusicPlayStateUnknown;
  s_music_track_pos_ms = 0;
  s_music_track_length_ms = 0;
  s_music_needs_user_to_start_playback = false;
  s_music_progress_supported = false;

  // Firmware defaults
  s_prefs_music_show_volume_controls = true;
  s_prefs_music_show_progress_bar = true;

  framebuffer_init(&s_fb, &(GSize) {DISP_COLS, DISP_ROWS});
  framebuffer_clear(&s_fb);
  graphics_context_init(&s_ctx, &s_fb, GContextInitializationMode_App);
  s_app_state_get_graphics_context = &s_ctx;

  fake_spi_flash_init(0 /* offset */, 0x1000000 /* length */);
  pfs_init(false /* run filesystem check */);
  pfs_format(true /* write erase headers */);
  load_resource_fixture_in_flash(RESOURCES_FIXTURE_PATH, SYSTEM_RESOURCES_FIXTURE_NAME,
                                 false /* is_next */);
  resource_init();
}

void test_music__cleanup(void) {
}

// Helpers
//////////////////////

static void prv_set_now_playing(const char *title, const char *artist) {
  strncpy(s_music_title, title, sizeof(s_music_title));
  s_music_title[sizeof(s_music_title) - 1] = '\0';
  strncpy(s_music_artist, artist, sizeof(s_music_artist));
  s_music_artist[sizeof(s_music_artist) - 1] = '\0';
}

static void prv_launch_app_and_render(void) {
  prv_handle_init();
  MusicAppData *data = app_state_get_user_data();
  Window *window = data->no_music_window ? &data->no_music_window->window : &data->window;
  window_set_on_screen(window, true, true);
  window_render(window, &s_ctx);
}

// Tests
//////////////////////

void test_music__playing(void) {
  prv_set_now_playing("Just Like Heaven", "The Cure");
  s_music_play_state = MusicPlayStatePlaying;
  s_music_track_pos_ms = 75 * 1000;
  s_music_track_length_ms = 245 * 1000;
  s_music_progress_supported = true;

  prv_launch_app_and_render();
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_music__playing_long_text(void) {
  prv_set_now_playing("It Could Be The First Day Of Springtime",
                      "Godspeed You! Black Emperor");
  s_music_play_state = MusicPlayStatePlaying;
  s_music_track_pos_ms = 754 * 1000;
  s_music_track_length_ms = 3945 * 1000;
  s_music_progress_supported = true;

  prv_launch_app_and_render();
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_music__paused(void) {
  prv_set_now_playing("Just Like Heaven", "The Cure");
  s_music_play_state = MusicPlayStatePaused;
  s_music_track_pos_ms = 75 * 1000;
  s_music_track_length_ms = 245 * 1000;
  s_music_progress_supported = true;

  prv_launch_app_and_render();
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_music__no_progress_reporting(void) {
  prv_set_now_playing("Just Like Heaven", "The Cure");
  s_music_play_state = MusicPlayStatePlaying;

  prv_launch_app_and_render();
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}

void test_music__no_music(void) {
  s_music_needs_user_to_start_playback = true;

  prv_launch_app_and_render();
  cl_check(gbitmap_pbi_eq(&s_ctx.dest_bitmap, TEST_PBI_FILE));
}
