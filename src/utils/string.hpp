// SPDX-License-Identifier: MIT

#ifndef UTILS_STRING_HPP
#define UTILS_STRING_HPP

#include <string>
#include <vector>

namespace utils
{
    std::string stringJoin(const std::vector<std::string>& items, const std::string& delim);
}

#endif // UTILS_STRING_HPP
