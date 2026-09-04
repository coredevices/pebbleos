/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/graphics/gtypes.h"
#include "applib/fonts/fonts.h"

#ifdef CONFIG_PLATFORM_GABBRO

#define LOCAL_HOUR_HAND_LENGTH_DEFAULT 74
#define LOCAL_HOUR_HAND_THICKNESS_DEFAULT 9
#define LOCAL_HOUR_HAND_COLOR_DEFAULT GColorWhite
#define LOCAL_HOUR_HAND_BACK_EXT_DEFAULT 0

#define LOCAL_MINUTE_HAND_LENGTH_DEFAULT 84
#define LOCAL_MINUTE_HAND_THICKNESS_DEFAULT 9
#define LOCAL_MINUTE_HAND_COLOR_DEFAULT GColorWhite
#define LOCAL_MINUTE_HAND_BACK_EXT_DEFAULT 0

#define LOCAL_BOB_RADIUS_DEFAULT 9
#define LOCAL_BOB_COLOR_DEFAULT GColorRed

#define LOCAL_TEXT_OFFSET 60
#define LOCAL_TEXT_COLOR GColorWhite
#define LOCAL_TEXT_FONT FONT_KEY_GOTHIC_24_BOLD
#define LOCAL_TEXT_FONT_SIZE 24

#define NON_LOCAL_HOUR_HAND_LENGTH_DEFAULT 16
#define NON_LOCAL_HOUR_HAND_WIDTH_DEFAULT 4

#define NON_LOCAL_MINUTE_HAND_LENGTH_DEFAULT 30
#define NON_LOCAL_MINUTE_HAND_WIDTH_DEFAULT 4

#define NON_LOCAL_TEXT_OFFSET 25
#define NON_LOCAL_TEXT_COLOR GColorWhite
#define NON_LOCAL_TEXT_FONT FONT_KEY_GOTHIC_18_BOLD
#define NON_LOCAL_TEXT_FONT_SIZE 18

#else

#define LOCAL_HOUR_HAND_LENGTH_DEFAULT 51
#define LOCAL_HOUR_HAND_THICKNESS_DEFAULT 6
#define LOCAL_HOUR_HAND_COLOR_DEFAULT GColorWhite
#define LOCAL_HOUR_HAND_BACK_EXT_DEFAULT 0

#define LOCAL_MINUTE_HAND_LENGTH_DEFAULT 58
#define LOCAL_MINUTE_HAND_THICKNESS_DEFAULT 6
#define LOCAL_MINUTE_HAND_COLOR_DEFAULT GColorWhite
#define LOCAL_MINUTE_HAND_BACK_EXT_DEFAULT 0

#define LOCAL_BOB_RADIUS_DEFAULT 6
#define LOCAL_BOB_COLOR_DEFAULT GColorRed

#define LOCAL_TEXT_OFFSET 50
#define LOCAL_TEXT_COLOR GColorWhite
#define LOCAL_TEXT_FONT FONT_KEY_GOTHIC_18_BOLD
#define LOCAL_TEXT_FONT_SIZE 18

#define NON_LOCAL_HOUR_HAND_LENGTH_DEFAULT 11
#define NON_LOCAL_HOUR_HAND_WIDTH_DEFAULT 3

#define NON_LOCAL_MINUTE_HAND_LENGTH_DEFAULT 21
#define NON_LOCAL_MINUTE_HAND_WIDTH_DEFAULT 3

#define NON_LOCAL_TEXT_OFFSET 15
#define NON_LOCAL_TEXT_COLOR GColorWhite
#define NON_LOCAL_TEXT_FONT FONT_KEY_GOTHIC_18_BOLD
#define NON_LOCAL_TEXT_FONT_SIZE 18

#endif

#define NUM_NON_LOCAL_CLOCKS 3

#define GLANCE_TIME_OUT_MS 8000

typedef enum {
  CLOCK_TEXT_TYPE_NONE = 0,
  CLOCK_TEXT_TYPE_TIME,
  CLOCK_TEXT_TYPE_DATE,
  CLOCK_TEXT_TYPE_BUFFER,
} ClockTextType;

typedef enum {
  CLOCK_TEXT_LOCATION_NONE = 0,
  CLOCK_TEXT_LOCATION_BOTTOM,
  CLOCK_TEXT_LOCATION_LEFT,
} ClockTextLocation;

typedef enum {
  CLOCK_HAND_STYLE_ROUNDED = 0,
  CLOCK_HAND_STYLE_ROUNDED_WITH_HIGHLIGHT,
  CLOCK_HAND_STYLE_POINTED,
  CLOCK_HAND_STYLE_SQUARE,
} ClockHandStyle;

typedef enum {
  CLOCK_LOCATION_CENTER,
  CLOCK_LOCATION_LEFT,
  CLOCK_LOCATION_BOTTOM,
  CLOCK_LOCATION_RIGHT,
  CLOCK_LOCATION_TOP,
} ClockLocation;

typedef struct {
  uint16_t length;
  uint16_t thickness;
  uint16_t backwards_extension;
  int32_t angle;
  GColor color;
  ClockHandStyle style;
} ClockHand;

typedef struct {
  ClockTextType type;
  ClockTextLocation location;
  char buffer[10]; // FIXME magic number
  GColor color;
  uint16_t offset;
  GFont font;
  uint16_t font_size;
} ClockText;

typedef struct {
  ClockHand hour_hand;
  ClockHand minute_hand;
  uint16_t bob_radius;
  uint16_t bob_center_radius;
  GColor bob_color;
  GColor bob_center_color;
  ClockLocation location;
  ClockText text;
  uint32_t bg_bitmap_id;
} ClockFace;

typedef struct {
  ClockFace local_clock;
  uint32_t num_non_local_clocks;
  ClockFace non_local_clock[NUM_NON_LOCAL_CLOCKS];
  uint32_t bg_bitmap_id;
} ClockModel;

void watch_model_init(void);

void watch_model_handle_change(ClockModel *model);

void watch_model_start_intro(void);

void watch_model_cleanup(void);
