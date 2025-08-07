// SPDX-License-Identifier: MIT

#ifndef VSMAP_VSMAP_COMMON_HPP
#define VSMAP_VSMAP_COMMON_HPP

#include "VapourSynth4.h"

#include "common/sampletype.hpp"

namespace vsmap
{
    common::SampleType getOptSampleType(const char* varName, const char* varNameS, const VSMap* in, VSMap* out, const VSAPI* vsapi, common::SampleType defaultValue);
}

#endif // VSMAP_VSMAP_COMMON_HPP
