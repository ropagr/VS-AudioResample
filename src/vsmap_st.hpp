// SPDX-License-Identifier: MIT

#ifndef _VSMAP_ST_HPP
#define _VSMAP_ST_HPP

#include "VapourSynth4.h"

#include "sampletype.hpp"

namespace vsmap_st
{
    SampleType getOptSampleType(const char* varName, const char* varNameS, const VSMap* in, VSMap* out, const VSAPI* vsapi, SampleType defaultValue);
}

#endif // _VSMAP_ST_HPP
