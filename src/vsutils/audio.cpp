// SPDX-License-Identifier: MIT

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

#include "VapourSynth4.h"

#include "utils/debug.hpp"

namespace vsutils
{
    int samplesToFrames(int64_t samples)
    {
        assertm(0 <= samples, "negative samples");

        if (samples == 0)
        {
            return 0;
        }

        int64_t frames = ((samples - 1) / VS_AUDIO_FRAME_SAMPLES) + 1;

        assertm(frames <= std::numeric_limits<int>::max(), "frames out of int range");

        return static_cast<int>(frames);
    }


    int getFrameSampleCount(int frame, int64_t totalSamples)
    {
        assertm(0 <= frame, "negative frame");

        int totalFrames = samplesToFrames(totalSamples);

        assertm(frame < totalFrames, "frame out of range");

        if (totalFrames <= frame)
        {
            return 0;
        }

        if (frame == totalFrames - 1)
        {
            // last frame
            return static_cast<int>(totalSamples - (frame * VS_AUDIO_FRAME_SAMPLES));
        }

        return VS_AUDIO_FRAME_SAMPLES;
    }


    // returns the first sample of a frame (inclusive)
    int64_t frameToFirstSample(int frame)
    {
        assertm(0 <= frame, "negative frame");

        return static_cast<int64_t>(frame) * VS_AUDIO_FRAME_SAMPLES;
    }


    // returns the last sample of a frame (exclusive)
    // or -1 if frame is outside of all samples
    int64_t frameToLastSample(int frame, int64_t totalSamples)
    {
        assertm(0 <= frame, "negative frame");

        int frameSamples = getFrameSampleCount(frame, totalSamples);
        if (frameSamples == 0)
        {
            return -1;
        }

        return frameToFirstSample(frame) + frameSamples;
    }


    int sampleToFrame(int64_t sample)
    {
        assertm(0 <= sample, "negative sample");

        int64_t frame = sample / VS_AUDIO_FRAME_SAMPLES;

        assertm(frame <= std::numeric_limits<int>::max(), "frame out of int range");

        return static_cast<int>(frame);
    }
}
