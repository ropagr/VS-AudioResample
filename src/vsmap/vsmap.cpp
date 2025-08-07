// SPDX-License-Identifier: MIT

#include "VapourSynth4.h"


namespace vsmap
{
    bool getOptBool(const char* varName, const VSMap* in, const VSAPI* vsapi, bool defaultValue)
    {
        int err = 0;
        int64_t result = vsapi->mapGetInt(in, varName, 0, &err);
        if (err)
        {
            return defaultValue;
        }

        return static_cast<bool>(result);
    }

    int getOptInt(const char* varName, const VSMap* in, const VSAPI* vsapi, int defaultValue)
    {
        int err = 0;
        int result = vsapi->mapGetIntSaturated(in, varName, 0, &err);
        if (err)
        {
            return defaultValue;
        }

        return result;
    }
}
