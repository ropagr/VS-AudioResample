// SPDX-License-Identifier: MIT

#include "VapourSynth4.h"

#include "common/sampletype.hpp"
#include "vsmap/vsmap_common.hpp"


namespace vsmap
{
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
