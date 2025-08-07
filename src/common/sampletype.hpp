// SPDX-License-Identifier: MIT

#ifndef COMMON_SAMPLETYPE_HPP
#define COMMON_SAMPLETYPE_HPP

#include <cstdint>
#include <string>

#include "VapourSynth4.h"

namespace common
{
    enum SampleType
    {
        Int8      = 0,
        Int16     = 1,
        Int24     = 2,
        Int32     = 3,
        Float32   = 4,
        Float64   = 5,
        Unknown   = 6,
    };


    SampleType getSampleTypeFromInt(int s);

    SampleType getSampleTypeFromString(std::string const &str);

    SampleType getSampleTypeFromAudioFormat(const VSAudioFormat* af);

    void applySampleTypeToAudioFormat(SampleType sampleType, VSAudioFormat* af);
}

#endif // COMMON_SAMPLETYPE_HPP
