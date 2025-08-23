// SPDX-License-Identifier: MIT

#pragma once

#include <map>
#include <string>

namespace common
{
    enum class ResampleQuality
    {
        Quick,
        Low,
        Medium,
        High,
        VeryHigh,
        Maximum,
    };

    std::map<std::string, ResampleQuality> getStringResampleQualityMap();
}
