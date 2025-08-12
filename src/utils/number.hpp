// SPDX-License-Identifier: MIT

#pragma once

#include <climits>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
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


    /// this will prevent overflowing when casting a double to float
    /// return: for positive d: a floating point value f with: 0 <= f <= d
    ///         for negative d: a floating point value f with: d <= f <= 0
    template <typename T>
    requires std::floating_point<T>
    T castToFloatTowardsZero(double d)
    {
        constexpr int doubleDigits = std::numeric_limits<double>::digits;
        constexpr int typeDigits = std::numeric_limits<T>::digits;

        if constexpr (doubleDigits <= typeDigits)
        {
            // T has a higher or the same precision as double
            return static_cast<T>(d);
        }
        else
        {
            // double has a higher precision than T

            // cast double to T -> T value can be greater, equal, or smaller than the double value
            T t = static_cast<T>(d);

            // cast t back to double (with less precision) to compare it with the original value
            double d_lowPrec = static_cast<double>(t);

            if ((0 < d && d < d_lowPrec) || (d < 0 && d_lowPrec < d))
            {
                // select the next floating point number towards zero
                return std::nextafter(t, static_cast<T>(0));
            }

            return t;
        }
    }
}
