// SPDX-License-Identifier: MIT

#ifndef VSUTILS_AUDIO_HPP
#define VSUTILS_AUDIO_HPP

#include <cstdint>

namespace vsutils
{
    int samplesToFrames(int64_t samples);

    int getFrameSampleCount(int frame, int64_t totalSamples);

    // returns the first sample of a frame (inclusive)
    int64_t frameToFirstSample(int frame);

    // returns the last sample of a frame (exclusive)
    // or -1 if frame is outside of all samples
    int64_t frameToLastSample(int frame, int64_t totalSamples);

    int sampleToFrame(int64_t sample);
}

#endif // VSUTILS_AUDIO_HPP
