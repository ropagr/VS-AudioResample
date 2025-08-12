#pragma once

#include <concepts>
#include <cstddef>
#include <type_traits>

#include "utils/number.hpp"

namespace vsutils
{
    struct BitShift
    {
        size_t count;
        bool required;
    };

    template <typename sample_t, size_t SampleIntBits>
    requires std::integral<sample_t> || std::floating_point<sample_t>
    constexpr BitShift getSampleBitShift()
    {
        size_t bitShiftNum = utils::bitwidth<sample_t> - SampleIntBits;
        return { .count = bitShiftNum,
                 .required = std::is_integral_v<sample_t> && 0 < bitShiftNum };
    }
}
