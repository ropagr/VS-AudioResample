// SPDX-License-Identifier: MIT

#ifndef VSMAP_VSMAP_HPP
#define VSMAP_VSMAP_HPP

#include "VapourSynth4.h"

namespace vsmap
{
    bool getOptBool(const char* varName, const VSMap* in, const VSAPI* vsapi, bool defaultValue);

    int getOptInt(const char* varName, const VSMap* in, const VSAPI* vsapi, int defaultValue);
}

#endif // VSMAP_VSMAP_HPP
