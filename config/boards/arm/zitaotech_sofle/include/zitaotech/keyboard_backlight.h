/* SPDX-License-Identifier: MIT */
#pragma once
#include <stdint.h>

#define SOFLE_BL_DEC 0
#define SOFLE_BL_INC 1
#define SOFLE_BL_TOGGLE 2
#define SOFLE_BL_PREV 3
#define SOFLE_BL_NEXT 4
#define SOFLE_BL_SET 5 /* Internal absolute split command. */
#define SOFLE_BL_SOLID 0
#define SOFLE_BL_BREATHE 1
#define SOFLE_BL_MODE_COUNT 2
#define SOFLE_BL_MAX_BRIGHTNESS 20

/* Calculate without mutating, then dispatch the same absolute state to both halves. */
int sofle_backlight_calculate(uint32_t command, uint32_t *packed);
int sofle_backlight_apply(uint32_t packed);
