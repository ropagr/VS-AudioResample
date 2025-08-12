// SPDX-License-Identifier: MIT

#pragma once

#include <map>
#include <optional>
#include <string>

#include "VapourSynth4.h"

namespace common
{
    enum class SampleType
    {
        // signed int, 1 byte, 8 bits
        Int8,
        // signed int, 2 bytes, 16 bits
        Int16,
        // signed int, 4 bytes, upper 24 bits
        Int24,
        // signed int, 4 bytes, 32 bits
        Int32,
        // float, 4 byte, 32 bits
        Float32,
        // float, 8 byte, 64 bits
        Float64
    };

    std::map<std::string, SampleType> getStringSampleTypeMap();

    std::map<std::string, SampleType> getStringVapourSynthSampleTypeMap();

    void applySampleTypeToAudioFormat(SampleType st, VSAudioFormat& af);

    std::optional<SampleType> getSampleTypeFromAudioFormat(const VSAudioFormat& af);

    bool isFloatSampleType(SampleType st);
}
