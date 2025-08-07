// SPDX-License-Identifier: MIT

#ifndef UTILS_BITOPS_HPP
#define UTILS_BITOPS_HPP

#include <climits>
#include <concepts>
#include <cstddef>
#include <type_traits>

namespace utils
{
    template <typename T>
    constexpr size_t bitwidth = sizeof(T) * CHAR_BIT;


    template <typename T, size_t NumBits>
    requires std::integral<T> && (NumBits <= bitwidth<T>)
    constexpr T maxInt = static_cast<T>((1LL << (NumBits - 1)) - 1);


    template <typename T, size_t NumBits>
    requires std::integral<T> && (NumBits <= bitwidth<T>)
    constexpr T minInt = static_cast<T>(-(1LL << (NumBits - 1)));
}

#endif // UTILS_BITOPS_HPP
