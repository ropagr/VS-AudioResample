// SPDX-License-Identifier: MIT

#pragma once

#include <algorithm>
#include <climits>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <type_traits>

#include "utils/number.hpp"

namespace utils
{
    template <typename sample_t, size_t IntSampleBits>
    requires std::integral<sample_t> || std::floating_point<sample_t>
    bool isSampleOverflowing(sample_t sample)
    {
        if constexpr (std::is_integral_v<sample_t>)
        {
            return sample == utils::minInt<sample_t, IntSampleBits>;
        }

        if constexpr (std::is_floating_point_v<sample_t>)
        {
            return sample < static_cast<sample_t>(-1) || static_cast<sample_t>(1) < sample;
        }
    }


    // minInt gets clamped to -maxInt
    template <typename sample_t, size_t IntSampleBits>
    requires std::integral<sample_t>
    sample_t clampSymIntSample(sample_t sample)
    {
        if (sample == utils::minInt<sample_t, IntSampleBits>)
        {
            return -utils::maxInt<sample_t, IntSampleBits>;
        }
        return sample;
    }


    template <typename sample_t>
    requires std::floating_point<sample_t>
    sample_t clampFloatSample(sample_t sample)
    {
        return std::clamp<sample_t>(sample, static_cast<sample_t>(-1), static_cast<sample_t>(1));
    }


    template <typename sample_t, size_t IntSampleBits>
    requires std::integral<sample_t> || std::floating_point<sample_t>
    sample_t castSample(double sample)
    {
        if constexpr (std::is_integral_v<sample_t>)
        {
            return static_cast<sample_t>(
                std::clamp(std::round(sample),
                           static_cast<double>(-utils::maxInt<sample_t, IntSampleBits>),
                           static_cast<double>(utils::maxInt<sample_t, IntSampleBits>)));
        }

        if constexpr (std::is_floating_point_v<sample_t>)
        {
            return static_cast<sample_t>(sample);
        }
    }


    /**
     * convert any integer or floating point type sample to double
     * integer will be clamped to symmetrical range
     * no clamping for float types
     */
    template <typename sample_t, size_t IntSampleBits>
    requires std::integral<sample_t> || std::floating_point<sample_t>
    double convSampleToDouble(sample_t sample)
    {
        if constexpr (std::is_integral_v<sample_t>)
        {
            sample = clampSymIntSample<sample_t, IntSampleBits>(sample);

            return static_cast<double>(sample) / static_cast<double>(utils::maxInt<sample_t, IntSampleBits>);
        }

        if constexpr (std::is_floating_point_v<sample_t>)
        {
            return static_cast<sample_t>(sample);
        }
    }


    /**
     * convert a double sample to any integer or floating point type
     * optional clamping for float types (default: false)
     */
    template <typename sample_t, size_t IntSampleBits>
    requires std::integral<sample_t> || std::floating_point<sample_t>
    sample_t convSampleFromDouble(double sample, bool clampFloat = false)
    {
        if constexpr (std::is_integral_v<sample_t>)
        {
            sample = clampFloatSample<double>(sample);

            return static_cast<sample_t>(std::round(sample * static_cast<double>(utils::maxInt<sample_t, IntSampleBits>)));
        }

        if constexpr (std::is_floating_point_v<sample_t>)
        {
            if (clampFloat)
            {
                return clampFloatSample<sample_t>(static_cast<sample_t>(sample));
            }
            return utils::castToFloatTowardsZero<sample_t>(sample);
        }
    }
}
