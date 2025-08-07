// SPDX-License-Identifier: MIT

#ifndef UTILS_ARRAY_HPP
#define UTILS_ARRAY_HPP

#include <cstddef>
#include <string>
#include <string_view>
#include <utility>

namespace utils
{
    template <typename T, size_t N>
    constexpr bool constArrayContains(const T (&arr)[N], T key)
    {
        for (const T& t : arr)
        {
            if (key == t)
            {
                return true;
            }
        }
        return false;
    }

    template <typename T, size_t N>
    std::map<std::string, T> constStringViewPairArrayToStringMap(const std::pair<std::string_view, T> (&arr)[N])
    {
        std::map<std::string, T> result;

        for (const std::pair<std::string_view, T>& p : arr)
        {
            result.insert({ std::string(p.first), p.second });
        }
        return result;
    }
}

#endif // UTILS_ARRAY_HPP
