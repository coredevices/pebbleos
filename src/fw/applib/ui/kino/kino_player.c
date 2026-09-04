/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kino_player.h"

#include "applib/ui/animation_interpolate.h"
#include <pbl/logging/logging.h>
#include "pbl/util/math.h"

//////////////////////////////////
// Callbacks
//////////////////////////////////

static void prv_announce_frame_did_change(KinoPlayer *player, bool frame_changed) {
  if (player->callbacks.frame_did_change && frame_changed) {
    player->callbacks.frame_did_change(player, player->context);
  }
}

static void prv_announce_did_stop(KinoPlayer *player, bool finished) {
  if (player->callbacks.did_stop) {
    player->callbacks.did_stop(player, finished, player->context);
  }
}

///////////////////////////////
// Play Animation
///////////////////////////////

T_STATIC void prv_play_animation_update(Animation *animation, const AnimationProgress normalized) {
  KinoPlayer *player = animation_get_context(animation);
  int32_t animation_elapsed_ms = 0;
  uint32_t elapsed_ms = 0;
  uint32_t kino_reel_duration = kino_reel_get_duration(player->reel);
  bool is_reel_infinite = (kino_reel_duration == PLAY_DURATION_INFINITE);
  bool is_animation_reversed = animation_get_reverse(animation);
  bool is_animation_infinite =
    (animation_get_duration(animation, false, false) == PLAY_DURATION_INFINITE);

  if (!is_animation_infinite && !is_reel_infinite) {
    // If neither animation nor reel is infinite
    elapsed_ms = interpolate_uint32(normalized, player->from_elapsed_ms, player->to_elapsed_ms);

  } else if ((is_animation_infinite || is_reel_infinite) && !is_animation_reversed) {
    // If either animation or reel is infinite and animation is not reversed
    animation_get_elapsed(animation, &animation_elapsed_ms);
    elapsed_ms = (int32_t)player->from_elapsed_ms + animation_elapsed_ms;

  } else if (is_animation_infinite && !is_reel_infinite && is_animation_reversed) {
    // If animation is infinite, reel is not infinite and animation is reversed
    animation_get_elapsed(animation, &animation_elapsed_ms);
    elapsed_ms = MAX(0, (int32_t)player->to_elapsed_ms - animation_elapsed_ms);

  } else {
    elapsed_ms = player->to_elapsed_ms;
  }

  bool frame_changed = kino_reel_set_elapsed(player->reel, elapsed_ms);
  prv_announce_frame_did_change(player, frame_changed);
}

static void prv_play_anim_stopped(Animation *anim, bool finished, void *context) {
  KinoPlayer *player = context;
  player->animation = NULL;
  prv_announce_did_stop(player, finished);
}

static const AnimationImplementation s_play_animation_impl = {
  .update = prv_play_animation_update,
};

static const AnimationHandlers s_play_anim_handlers = {
  .stopped = prv_play_anim_stopped,
};

//////////////////////////////////
// API
//////////////////////////////////

void kino_player_set_callbacks(KinoPlayer *player, KinoPlayerCallbacks callbacks, void *context) {
  player->callbacks = callbacks;
  player->context = context;
}

void kino_player_set_reel(KinoPlayer *player, KinoReel *reel, bool take_ownership) {
  if (!player) {
    return;
  }

  // stop any ongoing animation
  kino_player_pause(player);

  // delete the old reel if owned
  if (player->reel && player->owns_reel && player->reel != reel) {
    kino_reel_destroy(player->reel);
  }

  player->reel = reel;
  player->owns_reel = take_ownership;

  prv_announce_frame_did_change(player, true /*frame_changed*/);
}

void kino_player_set_reel_with_resource(KinoPlayer *player, uint32_t resource_id) {
  kino_player_set_reel(player, NULL, false);
  KinoReel *new_reel = kino_reel_create_with_resource(resource_id);
  kino_player_set_reel(player, new_reel, true);
}

void kino_player_set_reel_with_resource_system(KinoPlayer *player, ResAppNum app_num,
                                               uint32_t resource_id) {
  kino_player_set_reel(player, NULL, false);
  KinoReel *new_reel = kino_reel_create_with_resource_system(app_num, resource_id);
  kino_player_set_reel(player, new_reel, true);
}

KinoReel *kino_player_get_reel(KinoPlayer *player) {
  return player->reel;
}

static void prv_create_play_animation(KinoPlayer *player, uint32_t from_value, uint32_t to_value) {
  // stop any ongoing animation
  kino_player_pause(player);

  player->from_elapsed_ms = from_value;
  player->to_elapsed_ms = to_value;

  Animation *animation = animation_create();
  if (!animation) {
    return;
  }

  animation_set_implementation(animation, &s_play_animation_impl);
  animation_set_curve(animation, AnimationCurveLinear);
  animation_set_duration(animation, to_value - from_value);
  animation_set_handlers(animation, s_play_anim_handlers, (void *)player);
  animation_set_immutable(animation);

  player->animation = animation;
}

void kino_player_play(KinoPlayer *player) {
  Animation *animation = (Animation *)kino_player_create_play_animation(player);
  if (animation) {
    animation_schedule(animation);
  }
}

void kino_player_play_section(KinoPlayer *player, uint32_t from_elapsed_ms,
                              uint32_t to_elapsed_ms) {
  if (player && player->reel) {
    kino_reel_set_elapsed(player->reel, from_elapsed_ms);
    prv_create_play_animation(player, from_elapsed_ms, to_elapsed_ms);
    animation_schedule(player->animation);
  }
}

ImmutableAnimation *kino_player_create_play_animation(KinoPlayer *player) {
  if (player && player->reel) {
    const uint32_t from_value = kino_reel_get_elapsed(player->reel);
    const uint32_t to_value = kino_reel_get_duration(player->reel);
    prv_create_play_animation(player, from_value, to_value);
    return (ImmutableAnimation *)player->animation;
  }
  return NULL;
}

ImmutableAnimation *kino_player_create_play_section_animation(
    KinoPlayer *player, uint32_t from_elapsed_ms, uint32_t to_elapsed_ms) {
  if (player && player->reel) {
    prv_create_play_animation(player, from_elapsed_ms, to_elapsed_ms);
    return (ImmutableAnimation *)player->animation;
  }
  return NULL;
}

void kino_player_pause(KinoPlayer *player) {
  if (player && player->reel) {
    animation_unschedule(player->animation);
    player->animation = NULL;
  }
}

void kino_player_rewind(KinoPlayer *player) {
  if (player && player->reel) {
    // first pause the player, in case it is running
    kino_player_pause(player);
    // reset the elapsed time to the start
    bool frame_changed = kino_reel_set_elapsed(player->reel, 0);
    prv_announce_frame_did_change(player, frame_changed);
  }
}

void kino_player_draw(KinoPlayer *player, GContext *ctx, GPoint offset) {
  kino_player_draw_processed(player, ctx, offset, NULL);
}

void kino_player_draw_processed(KinoPlayer *player, GContext *ctx, GPoint offset,
                                KinoReelProcessor *processor) {
  if (player && player->reel) {
    kino_reel_draw_processed(player->reel, ctx, offset, processor);
  }
}

void kino_player_deinit(KinoPlayer *player) {
  player->callbacks = (KinoPlayerCallbacks) { 0 };
  kino_player_set_reel(player, NULL, false);
}
