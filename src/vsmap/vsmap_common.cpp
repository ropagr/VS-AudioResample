// SPDX-License-Identifier: MIT

#include <optional>

#include "VapourSynth4.h"

#include "common/overflow.hpp"
#include "common/sampletype.hpp"
#include "vsmap/vsmap_common.hpp"


namespace vsmap
{
    std::optional<common::OverflowMode> getOverflowModeFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi)
    {
        return getValueFromString(varName, logFuncName, in, out, vsapi, common::getStringOverflowModeMap());
    }


    std::optional<common::OverflowMode> getOptOverflowModeFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi, common::OverflowMode defaultValue)
    {
        return getOptValueFromString(varName, logFuncName, in, out, vsapi, common::getStringOverflowModeMap(), defaultValue);
    }


    std::optional<common::OverflowLog> getOverflowLogFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi)
    {
        return getValueFromString(varName, logFuncName, in, out, vsapi, common::getStringOverflowLogMap());
    }


    std::optional<common::OverflowLog> getOptOverflowLogFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi, common::OverflowLog defaultValue)
    {
        return getOptValueFromString(varName, logFuncName, in, out, vsapi, common::getStringOverflowLogMap(), defaultValue);
    }


    std::optional<common::ResampleQuality> getResampleQualityFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi)
    {
        return getValueFromString(varName, logFuncName, in, out, vsapi, common::getStringResampleQualityMap());
    }


    std::optional<common::ResampleQuality> getOptResampleQualityFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi, common::ResampleQuality defaultValue)
    {
        return getOptValueFromString(varName, logFuncName, in, out, vsapi, common::getStringResampleQualityMap(), defaultValue);
    }


    std::optional<common::SampleType> getSampleTypeFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi)
    {
        return getValueFromString(varName, logFuncName, in, out, vsapi, common::getStringSampleTypeMap());
    }


    std::optional<common::SampleType> getOptSampleTypeFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi, common::SampleType defaultValue)
    {
        return getOptValueFromString(varName, logFuncName, in, out, vsapi, common::getStringSampleTypeMap(), defaultValue);
    }


    std::optional<common::SampleType> getVapourSynthSampleTypeFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi)
    {
        return getValueFromString(varName, logFuncName, in, out, vsapi, common::getStringVapourSynthSampleTypeMap());
    }


    std::optional<common::SampleType> getOptVapourSynthSampleTypeFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi, common::SampleType defaultValue)
    {
        return getOptValueFromString(varName, logFuncName, in, out, vsapi, common::getStringVapourSynthSampleTypeMap(), defaultValue);
    }
}
