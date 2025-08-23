// SPDX-License-Identifier: MIT

#include <string>
#include <string_view>
#include <utility>

#include "common/resquality.hpp"
#include "utils/array.hpp"

namespace common
{
    constexpr std::pair<std::string_view, ResampleQuality> strResampleQualityPairs[] =
    {
        { "quick",     ResampleQuality::Quick },
        { "low",       ResampleQuality::Low },
        { "medium",    ResampleQuality::Medium },
        { "high",      ResampleQuality::High },
        { "very_high", ResampleQuality::VeryHigh },
        { "max",       ResampleQuality::Maximum },
    };


    std::map<std::string, ResampleQuality> getStringResampleQualityMap()
    {
        return utils::constStringViewPairArrayToStringMap(strResampleQualityPairs);
    }
}
