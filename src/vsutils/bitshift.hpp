#ifndef VSUTILS_BITSHIFT_HPP
#define VSUTILS_BITSHIFT_HPP

#include <concepts>
#include <type_traits>

#include "utils/bitops.hpp"

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

#endif // VSUTILS_BITSHIFT_HPP
