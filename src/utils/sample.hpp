// SPDX-License-Identifier: MIT

#ifndef UTILS_SAMPLE_HPP
#define UTILS_SAMPLE_HPP

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace utils
{
    template <typename sample_t>
    constexpr size_t bitcount = sizeof(sample_t) * CHAR_BIT;

    // ignore overflow
    #ifdef _MSC_VER
    #pragma warning(push)
    #pragma warning(disable: 4307)
    #endif // _MSC_VER
    template <typename sample_t, size_t SampleIntBits>
    constexpr sample_t maxSignedInt = bitcount<sample_t> <= SampleIntBits ? std::numeric_limits<sample_t>::max() : static_cast<sample_t>((1LL << (SampleIntBits - 1)) - 1);

    template <typename sample_t, size_t SampleIntBits>
    constexpr sample_t minSignedInt = bitcount<sample_t> <= SampleIntBits ? std::numeric_limits<sample_t>::min() : static_cast<sample_t>(-(1LL << (SampleIntBits - 1)));
    #ifdef _MSC_VER
    #pragma warning(pop)
    #endif // _MSC_VER

    template <typename sample_t, size_t SampleIntBits>
    double clampSignedInt(double sample)
    {
        if constexpr (bitcount<sample_t> <= SampleIntBits)
        {
            return sample;
        }

        if (sample < minSignedInt<sample_t, SampleIntBits>)
        {
            return minSignedInt<sample_t, SampleIntBits>;
        }

        if (maxSignedInt<sample_t, SampleIntBits> < sample)
        {
            return maxSignedInt<sample_t, SampleIntBits>;
        }

        return sample;
    }

    /**
     * sample is expected to be in the range of the *source* sample type 'src_sample_t'
     * e.g. [    -1,    +1] for float or double
     *      [  -127,   128] for int8_t
     *      [-32768, 32767] for int16_t
     *      ...
     */
    template <typename src_sample_t, size_t SrcSampleIntBits, typename dst_sample_t, size_t DstSampleIntBits>
    dst_sample_t convSampleType(double sample)
    {
        if constexpr (std::is_same_v<src_sample_t, dst_sample_t>)
        {
            if constexpr (std::is_integral_v<dst_sample_t>)
            {
                return static_cast<dst_sample_t>(std::round(sample));
            }
            return static_cast<dst_sample_t>(sample);
        }

        if constexpr (std::is_integral_v<src_sample_t>)
        {
            if constexpr (std::is_integral_v<dst_sample_t>)
            {
                sample = clampSignedInt<src_sample_t, SrcSampleIntBits>(sample);

                if (sample < 0)
                {
                    constexpr double minDstDivMinSrc = static_cast<double>(minSignedInt<dst_sample_t, DstSampleIntBits>) / static_cast<double>(minSignedInt<src_sample_t, SrcSampleIntBits>);
                    return static_cast<dst_sample_t>(std::round(sample * minDstDivMinSrc));
                }

                constexpr double maxDstDivMaxSrc = static_cast<double>(maxSignedInt<dst_sample_t, DstSampleIntBits>) / static_cast<double>(maxSignedInt<src_sample_t, SrcSampleIntBits>);
                return static_cast<dst_sample_t>(std::round(sample * maxDstDivMaxSrc));
            }

            if constexpr (std::is_floating_point_v<dst_sample_t>)
            {
                if (sample < 0)
                {
                    return static_cast<dst_sample_t>(sample / -static_cast<double>(minSignedInt<src_sample_t, SrcSampleIntBits>));
                }

                return static_cast<dst_sample_t>(sample / static_cast<double>(maxSignedInt<src_sample_t, SrcSampleIntBits>));
            }

            // not supposed to happen
            return 0;
        }

        if constexpr (std::is_floating_point_v<src_sample_t>)
        {
            if constexpr (std::is_integral_v<dst_sample_t>)
            {
                sample = std::clamp<double>(sample, -1, 1);

                if (sample < 0)
                {
                    return static_cast<dst_sample_t>(std::round(static_cast<double>(sample) * -static_cast<double>(minSignedInt<dst_sample_t, DstSampleIntBits>)));
                }

                return static_cast<dst_sample_t>(std::round(static_cast<double>(sample) * static_cast<double>(maxSignedInt<dst_sample_t, DstSampleIntBits>)));
            }

            if constexpr (std::is_floating_point_v<dst_sample_t>)
            {
                return static_cast<dst_sample_t>(sample);
            }

            // not supposed to happen
            return 0;
        }

        // not supposed to happen
        return 0;
    }
}

#endif // UTILS_SAMPLE_HPP
