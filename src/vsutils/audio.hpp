// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>

namespace vsutils
{
    int64_t secondsToSamples(double seconds, int sampleRate);

    double samplesToSeconds(int64_t samples, int sampleRate);

    int64_t secondsToSampleCount(double seconds, int sampleRate);

    double sampleCountToSeconds(int64_t numSamples, int sampleRate);

    int sampleCountToFrames(int64_t samples);

    int getFrameSampleCount(int frame, int64_t totalSamples);

    bool isLastFrame(int frame, int64_t totalSamples);

    // returns the first sample of a frame (inclusive)
    int64_t frameToFirstSample(int frame);

    // returns the last sample of a frame (exclusive)
    // or -1 if frame is outside of all samples
    int64_t frameToLastSample(int frame, int64_t totalSamples);

    int sampleToFrame(int64_t sample);
}
