// SPDX-License-Identifier: MIT

#pragma once

#include <climits>

#if __cplusplus < 202002L
#error "This plugin requires C++20 or later"
#endif

static_assert(CHAR_BIT == 8, "This plugin requires 8-bit bytes (CHAR_BIT == 8)");
static_assert(sizeof(float) == 4, "Expected float to be 4 bytes");
static_assert(sizeof(double) == 8, "Expected double to be 8 bytes");
