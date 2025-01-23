// SPDX-License-Identifier: MIT

#ifndef _SAMPLETYPE_HPP
#define _SAMPLETYPE_HPP

#include <cstdint>
#include <string>

#include "VapourSynth4.h"


enum SampleType {
  Int8      = 0,
  Int16     = 1,
  Int24     = 2,
  Int32     = 3,
  Float32   = 4,
  Float64   = 5,
  Unknown   = 6,
};


namespace sampletype
{
    SampleType fromInt(int s);

    SampleType fromString(std::string const &str);

    SampleType fromAudioFormat(const VSAudioFormat* af);

    void applyToAudioFormat(SampleType sampleType, VSAudioFormat* af);
}

#endif // _SAMPLETYPE_HPP
