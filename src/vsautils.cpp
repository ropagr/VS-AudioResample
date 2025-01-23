// SPDX-License-Identifier: MIT

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

#include "vsautils.hpp"
#include "VapourSynth4.h"

// Use (void) to silence unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

namespace vsautils
{
    int samplesToFrames(int64_t samples)
    {
        assertm(0 <= samples, "Negative samples");

        if (samples == 0)
        {
            return 0;
        }

        int64_t frames = ((samples - 1) / VS_AUDIO_FRAME_SAMPLES) + 1;

        assertm(frames <= std::numeric_limits<int>::max(), "frames out of int range");

        return static_cast<int>(frames);
    }


    // returns the first sample of a frame
    int64_t frameToSample(int frame)
    {
        assertm(0 <= frame, "Negative frame");

        return static_cast<int64_t>(frame) * VS_AUDIO_FRAME_SAMPLES;
    }


    int sampleToFrame(int64_t sample)
    {
        assertm(0 <= sample, "Negative sample");

        int64_t frame = sample / VS_AUDIO_FRAME_SAMPLES;

        assertm(frame <= std::numeric_limits<int>::max(), "frame out of int range");

        return static_cast<int>(frame);
    }
}
