// SPDX-License-Identifier: MIT

#pragma once

#include <format>
#include <map>
#include <optional>
#include <string>

#include "VapourSynth4.h"

#include "common/overflow.hpp"
#include "common/sampletype.hpp"
#include "utils/map.hpp"
#include "utils/string.hpp"

namespace vsmap
{
    common::SampleType getOptSampleType(const char* varName, const char* varNameS, const VSMap* in, VSMap* out, const VSAPI* vsapi, common::SampleType defaultValue);

    template<typename T>
    static std::optional<T> getValueFromStringImpl(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi, const std::map<std::string, T>& strValueMap, std::optional<T> defaultValue)
    {
        int err = 0;
        const char* strVarChars = vsapi->mapGetData(in, varName, 0, &err);
        if (err)
        {
            // string variable not defined
            if (defaultValue.has_value())
            {
                // return provided default value
                return defaultValue;
            }

            // no default value provided -> error
            std::string allowedValues = utils::stringJoin(utils::mapGetKeys(strValueMap), ", ");

            std::string errMsg = std::format("{}: {} not specified, must be one of: {}", logFuncName, varName, allowedValues);
            vsapi->mapSetError(out, errMsg.c_str());
            return std::nullopt;
        }

        std::string strVar(strVarChars);
        if (auto optSampleType = utils::mapGet(strValueMap, strVar))
        {
            return optSampleType;
        }

        std::string allowedValues = utils::stringJoin(utils::mapGetKeys(strValueMap), ", ");

        std::string errMsg = std::format("{}: invalid {} value: {}, must be one of: {}", logFuncName, varName, strVar, allowedValues);
        vsapi->mapSetError(out, errMsg.c_str());
        return std::nullopt;
    }


    template<typename T>
    std::optional<T> getOptValueFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi, std::map<std::string, T> strValueMap, T defaultValue)
    {
        return getValueFromStringImpl<T>(varName, logFuncName, in, out, vsapi, strValueMap, defaultValue);
    }


    template<typename T>
    std::optional<T> getValueFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi, std::map<std::string, T> strValueMap)
    {
        return getValueFromStringImpl<T>(varName, logFuncName, in, out, vsapi, strValueMap, std::nullopt);
    }

    /** no error handling needed **/
    std::optional<common::OverflowMode> getOverflowModeFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi);

    /** no error handling needed **/
    std::optional<common::OverflowMode> getOptOverflowModeFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi, common::OverflowMode defaultValue);

    /** no error handling needed **/
    std::optional<common::OverflowLog> getOverflowLogFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi);

    /** no error handling needed **/
    std::optional<common::OverflowLog> getOptOverflowLogFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi, common::OverflowLog defaultValue);

    /** no error handling needed **/
    std::optional<common::SampleType> getSampleTypeFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi);

    /** no error handling needed **/
    std::optional<common::SampleType> getOptSampleTypeFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi, common::SampleType defaultValue);

    /** no error handling needed **/
    std::optional<common::SampleType> getVapourSynthSampleTypeFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi);

    /** no error handling needed **/
    std::optional<common::SampleType> getOptVapourSynthSampleTypeFromString(const char* varName, const char* logFuncName, const VSMap* in, VSMap* out, const VSAPI* vsapi, common::SampleType defaultValue);
}
