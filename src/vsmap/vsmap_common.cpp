// SPDX-License-Identifier: MIT

#include <format>
#include <string>

#include "VapourSynth4.h"

#include "common/sampletype.hpp"


namespace vsmap
{
    common::SampleType getOptSampleType(const char* varName, const char* varNameS, const VSMap* in, VSMap* out, const VSAPI* vsapi, common::SampleType defaultValue)
    {
        // int sample type
        int err = 0;
        int64_t sampleType_i = vsapi->mapGetInt(in, varName, 0, &err);
        if (err)
        {
            // int sample type not defined -> try string sample type
            err = 0;
            const char* sampleType_s = vsapi->mapGetData(in, varNameS, 0, &err);
            if (err)
            {
                sampleType_i = -1;
            }
            else
            {
                std::string sampleType_str(sampleType_s);
                return common::getSampleTypeFromString(sampleType_str);
            }
        }

        return sampleType_i < 0 || 6 < sampleType_i ? defaultValue : common::getSampleTypeFromInt(static_cast<int>(sampleType_i));
    }
}
