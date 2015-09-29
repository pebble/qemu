#pragma once

#include "qemu-common.h"

typedef struct {
  uint8_t red, green, blue;
} PSDisplayPixelColor;

typedef struct {
  uint8_t alpha;
  PSDisplayPixelColor color;
} PSDisplayPixelColorWithAlpha;
