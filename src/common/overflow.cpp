// SPDX-License-Identifier: MIT

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
        { "error",         OverflowMode::Error },
        { "clip",          OverflowMode::Clip },
        { "clip_int_only", OverflowMode::ClipIntOnly },
        { "ignore_float",  OverflowMode::ClipIntOnly },
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

    void logNumOverflows(int64_t numOverflows, const char* funcName, VSCore* core, const VSAPI* vsapi)
    {
        if (0 < numOverflows)
        {
            std::string warnMsg = std::format("{}: {} sample overflows detected", funcName, numOverflows);
            vsapi->logMessage(VSMessageType::mtWarning, warnMsg.c_str(), core);
        }
    }
}
