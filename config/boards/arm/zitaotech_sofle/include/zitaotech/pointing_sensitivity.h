/* SPDX-License-Identifier: MIT */
#pragma once

#include <stdint.h>

/* A firmware sensitivity level, not a hardware DPI value or LED brightness. */
#define SOFLE_POINTER_LEVEL_MIN 10
#define SOFLE_POINTER_LEVEL_MAX 100

uint8_t sofle_pointer_sensitivity_get(void);
int sofle_pointer_sensitivity_adjust(int direction);
