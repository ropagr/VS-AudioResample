// SPDX-License-Identifier: MIT

#pragma once

#include <cassert>

// Use (void) to silence unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))
