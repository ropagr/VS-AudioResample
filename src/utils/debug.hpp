// SPDX-License-Identifier: MIT

#ifndef UTILS_DEBUG_HPP
#define UTILS_DEBUG_HPP

#include <cassert>

// Use (void) to silence unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

#endif // UTILS_DEBUG_HPP
