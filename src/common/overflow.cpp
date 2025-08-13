// SPDX-License-Identifier: MIT

#include <algorithm>
#include <format>
#include <map>
#include <string>
#include <string_view>
#include <utility>

#include "VapourSynth4.h"

#include "common/overflow.hpp"
#include "utils/array.hpp"

namespace common
{
    constexpr std::pair<std::string_view, OverflowMode> strOverflowModePairs[] =
    {
        { "error",      OverflowMode::Error },
        { "clip",       OverflowMode::Clip },
        { "clip_int",   OverflowMode::ClipInt },
        { "keep_float", OverflowMode::KeepFloat },
    };


    constexpr std::pair<std::string_view, OverflowLog> strOverflowLogPairs[] =
    {
        { "all",  OverflowLog::All },
        { "once", OverflowLog::Once },
        { "none", OverflowLog::None },
    };


    std::map<std::string, OverflowMode> getStringOverflowModeMap()
    {
        return utils::constStringViewPairArrayToStringMap(strOverflowModePairs);
    }


    std::map<std::string, OverflowLog> getStringOverflowLogMap()
    {
        return utils::constStringViewPairArrayToStringMap(strOverflowLogPairs);
    }


    void OverflowStats::addSample(double sample)
    {
        ++count;

        double absSample = std::abs(sample);
        if (peak < absSample)
        {
            peak = absSample;
        }
    }

    void OverflowStats::logVS(const char* funcName, OverflowMode ofMode, bool floatSampleType, VSCore* core, const VSAPI* vsapi)
    {
        std::string logMsg;

        switch (ofMode)
        {
            case OverflowMode::ClipInt:
            case OverflowMode::KeepFloat:
                if (floatSampleType)
                {
                    logMsg = std::format("{}: {} sample overflows detected. Peak: {:.6f}", funcName, count, peak);
                    vsapi->logMessage(VSMessageType::mtWarning, logMsg.c_str(), core);
                    break;
                }
                [[fallthrough]];

            case OverflowMode::Clip:
                logMsg = std::format("{}: {} sample overflows detected. Peak: {:.6f}. All overflows clipped.", funcName, count, peak);
                vsapi->logMessage(VSMessageType::mtInformation, logMsg.c_str(), core);
                break;

            case OverflowMode::Error:
            default:
                break;
        }
    }
}
