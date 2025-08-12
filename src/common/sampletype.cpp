// SPDX-License-Identifier: MIT

#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "VapourSynth4.h"

#include "common/sampletype.hpp"
#include "utils/array.hpp"

namespace common
{
    constexpr SampleType vapourSynthSampleTypes[] =
    {
        SampleType::Int16,
        SampleType::Int24,
        SampleType::Int32,
        SampleType::Float32,
    };


    constexpr std::pair<std::string_view, SampleType> strSampleTypePairs[] =
    {
        { "i8",  SampleType::Int8 },
        { "i16", SampleType::Int16 },
        { "i24", SampleType::Int24 },
        { "i32", SampleType::Int32 },
        { "f32", SampleType::Float32 },
        { "f64", SampleType::Float64 },
    };


    std::map<std::string, SampleType> getStringSampleTypeMap()
    {
        return utils::constStringViewPairArrayToStringMap(strSampleTypePairs);
    }


    std::map<std::string, SampleType> getStringVapourSynthSampleTypeMap()
    {
        std::map<std::string, SampleType> result;

        for (std::pair<std::string_view, SampleType> p : strSampleTypePairs)
        {
            if (utils::constArrayContains(vapourSynthSampleTypes, p.second))
            {
                result.insert({ std::string(p.first), p.second });
            }
        }
        return result;
    }


    void applySampleTypeToAudioFormat(SampleType st, VSAudioFormat& af)
    {
        switch (st)
        {
            case SampleType::Int8:
                af.sampleType = VSSampleType::stInteger;
                af.bitsPerSample = 8;
                af.bytesPerSample = 1;
                break;
            case SampleType::Int16:
                af.sampleType = VSSampleType::stInteger;
                af.bitsPerSample = 16;
                af.bytesPerSample = 2;
                break;
            case SampleType::Int24:
                af.sampleType = VSSampleType::stInteger;
                af.bitsPerSample = 24;
                af.bytesPerSample = 4;
                break;
            case SampleType::Int32:
                af.sampleType = VSSampleType::stInteger;
                af.bitsPerSample = 32;
                af.bytesPerSample = 4;
                break;
            case SampleType::Float32:
                af.sampleType = VSSampleType::stFloat;
                af.bitsPerSample = 32;
                af.bytesPerSample = 4;
                break;
            case SampleType::Float64:
                af.sampleType = VSSampleType::stFloat;
                af.bitsPerSample = 64;
                af.bytesPerSample = 8;
                break;
        }
    }


    std::optional<SampleType> getSampleTypeFromAudioFormat(const VSAudioFormat& af)
    {
        if (af.sampleType == VSSampleType::stInteger)
        {
            if (af.bitsPerSample == 8 && af.bytesPerSample == 1)
            {
                return SampleType::Int8;
            }

            if (af.bitsPerSample == 16 && af.bytesPerSample == 2)
            {
                return SampleType::Int16;
            }

            if (af.bitsPerSample == 24 && af.bytesPerSample == 4)
            {
                return SampleType::Int24;
            }

            if (af.bitsPerSample == 32 && af.bytesPerSample == 4)
            {
                return SampleType::Int32;
            }
        }


        if (af.sampleType == VSSampleType::stFloat)
        {
            if (af.bitsPerSample == 32 && af.bytesPerSample == 4)
            {
                return SampleType::Float32;
            }

            if (af.bitsPerSample == 64 && af.bytesPerSample == 8)
            {
                return SampleType::Float64;
            }
        }

        return std::nullopt;
    }


    bool isFloatSampleType(SampleType st)
    {
        switch (st)
        {
            case SampleType::Float32:
            case SampleType::Float64:
                return true;
            default:
                return false;
        }
    }
}
