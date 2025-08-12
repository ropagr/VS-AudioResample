// SPDX-License-Identifier: MIT

#pragma once

#include "VapourSynth4.h"

namespace vsmap
{
    bool getOptBool(const char* varName, const VSMap* in, const VSAPI* vsapi, bool defaultValue);

    int getOptInt(const char* varName, const VSMap* in, const VSAPI* vsapi, int defaultValue);
}
