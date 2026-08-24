#pragma once

#include "qemu/osdep.h"

typedef struct {
  uint8_t red, green, blue;
} PSDisplayPixelColor;

typedef struct {
  uint8_t alpha;
  PSDisplayPixelColor color;
} PSDisplayPixelColorWithAlpha;
