// SPDX-License-Identifier: MIT

#include <string>

#include "VapourSynth4.h"

#include "common/sampletype.hpp"

namespace common
{
    SampleType getSampleTypeFromInt(int s)
    {
        if (static_cast<int>(SampleType::Int8) <= s && s <= static_cast<int>(SampleType::Unknown))
        {
            return static_cast<SampleType>(s);
        }

        return SampleType::Unknown;
    }


    SampleType getSampleTypeFromString(std::string const &str)
    {
        if (str == "i8")
        {
            return SampleType::Int8;
        }

        if (str == "i16")
        {
            return SampleType::Int16;
        }

        if (str == "i24")
        {
            return SampleType::Int24;
        }

        if (str == "i32")
        {
            return SampleType::Int32;
        }

        if (str == "f32")
        {
            return SampleType::Float32;
        }

        if (str == "f64")
        {
            return SampleType::Float64;
        }

        return SampleType::Unknown;
    }


    SampleType getSampleTypeFromAudioFormat(const VSAudioFormat* af)
    {
        if (af->sampleType == VSSampleType::stInteger)
        {
            switch (af->bitsPerSample)
            {
            case 8:
                return SampleType::Int8;
            case 16:
                return SampleType::Int16;
            case 24:
                return SampleType::Int24;
            case 32:
                return SampleType::Int32;
            default:
                return SampleType::Unknown;
            }
        }

        if (af->sampleType == VSSampleType::stFloat)
        {
            switch (af->bitsPerSample)
            {
            case 32:
                return SampleType::Float32;
            case 64:
                return SampleType::Float64;
            default:
                return SampleType::Unknown;
            }
        }

        return SampleType::Unknown;
    }


    void applySampleTypeToAudioFormat(SampleType sampleType, VSAudioFormat* af)
    {
        // assuming: CHAR_BIT == 8

        switch (sampleType)
        {
        case SampleType::Int8:
            af->sampleType = VSSampleType::stInteger;
            af->bitsPerSample = 8;
            af->bytesPerSample = 1;
            break;
        case SampleType::Int16:
            af->sampleType = VSSampleType::stInteger;
            af->bitsPerSample = 16;
            af->bytesPerSample = 2;
            break;
        case SampleType::Int24:
            af->sampleType = VSSampleType::stInteger;
            af->bitsPerSample = 24;
            af->bytesPerSample = 4;
            break;
        case SampleType::Int32:
            af->sampleType = VSSampleType::stInteger;
            af->bitsPerSample = 32;
            af->bytesPerSample = 4;
            break;
        case SampleType::Float32:
            af->sampleType = VSSampleType::stFloat;
            af->bitsPerSample = 32;
            af->bytesPerSample = 4;
            break;
        case SampleType::Float64:
            af->sampleType = VSSampleType::stFloat;
            af->bitsPerSample = 64;
            af->bytesPerSample = 8;
            break;
        default:
            break;
        }
    }
}
