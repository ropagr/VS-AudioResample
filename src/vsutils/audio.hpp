// SPDX-License-Identifier: MIT

#ifndef VSUTILS_AUDIO_HPP
#define VSUTILS_AUDIO_HPP

#include <cstdint>

namespace vsutils
{
    int samplesToFrames(int64_t samples);

    // returns the first sample of a frame
    int64_t frameToSample(int frame);

    int sampleToFrame(int64_t sample);
}

#endif // VSUTILS_AUDIO_HPP
