// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include <samplerate.h>

#include "VapourSynth4.h"

#include "common/overflow.hpp"
#include "common/sampletype.hpp"


class Resample
{
public:
    static std::optional<Resample*> newResample(
            VSNode* inAudio, const VSAudioInfo* inAudioInfo, int outSampleRate, common::SampleType outSampleType, int resampleType,
            common::OverflowMode overflowMode, common::OverflowLog overflowLog, VSMap* outMap, const VSAPI* vsapi);

    VSNode* getInAudio();

    const VSAudioInfo& getInAudioInfo();

    const VSAudioInfo& getOutAudioInfo();

    bool isPassthrough();

    int getInBufLen();

    void logOverflowStats(VSCore* core, const VSAPI* vsapi);

    void logProcDone(VSCore* core, const VSAPI* vsapi);

    void free(const VSAPI* vsapi);

    bool writeFrame(VSFrame* outFrm, int outFrmNum, int64_t inPosReadStart, int64_t inPosReadEnd,
                    const std::vector<const VSFrame*>& inFrms, int inFrmsNumStart,
                    VSFrameContext* frameCtx, VSCore* core, const VSAPI* vsapi);

private:
    VSNode* inAudio;
    const VSAudioInfo inAudioInfo;

    VSAudioInfo outAudioInfo;

    const int outSampleRate;

    const common::SampleType inSampleType;
    const common::SampleType outSampleType;

    const common::OverflowMode overflowMode;
    const common::OverflowLog overflowLog;

    common::OverflowStats overflowStats = { .count = 0, .peak = 0.0 };

    // number of channels for convenience; same for input and output
    const int numChannels;

    // last processed output frame
    // *not* using -1 as initial value because -1 would indicate that the actual first frame (0)
    // is a subsequent frame of an already processed frame (-1)
    int lastOutFrmNum = -10;

    // libsamplerate resampler instance
    SRC_STATE* resState;

    double resRatio = 0;

    int64_t inPosReadNext = 0;

    // input buffer, channel interleaved samples
    float* inBuf = nullptr;
    // number of allocated interleaved samples, i.e. one sample of each channel
    int inBufLen = 0;
    // number of interleaved samples currently stored in inBuf
    int inBufUsed = 0;

    // output buffer, channel interleaved samples
    float* outBuf = nullptr;
    // number of allocated interleaved samples, i.e. one sample of each channel
    int outBufLen = 0;
    // number of interleaved samples currently stored in outBuf
    int outBufUsed = 0;

    int64_t totalUsedInSamples = 0;
    int64_t totalGenOutSamples = 0;


    Resample(VSNode* inAudio, const VSAudioInfo* inAudioInfo, int outSampleRate, common::SampleType outSampleType,
             common::OverflowMode overflowMode, common::OverflowLog overflowLog, SRC_STATE* resState, int _inBufLen, int _outBufLen);

    template <typename in_sample_t, size_t InIntSampleBits, typename out_sample_t, size_t OutIntSampleBits>
    bool writeFrameNoResampling(VSFrame* outFrame, int64_t outPosFrmStart, int outFrmLen, const VSFrame* inFrame,
                                const common::OverflowContext& ofCtx);

    template <typename in_sample_t, size_t InIntSampleBits>
    int fillInterleavedSamples(float* buf, int bufLen, int bufUsed, int64_t inPosReadStart, int64_t inPosReadEnd,
                               const std::vector<const VSFrame*>& inFrms, int inFrmsNumStart, const VSAPI* vsapi);

    template <typename out_sample_t, size_t OutIntSampleBits>
    std::optional<int> writeFrameFromInterleavedSamples(VSFrame* outFrm, int64_t outPosFrmStart, int outFrmLen,
                                                        float* buf, int bufLen, int bufUsed, const common::OverflowContext& ofCtx);

    bool resampleChunks(int outFrmNum, int outFrmLen, const common::OverflowContext& ofCtx);

    template <typename in_sample_t, size_t InIntSampleBits, typename out_sample_t, size_t OutIntSampleBits>
    bool writeFrameImpl(VSFrame* outFrm, int outFrmNum, int64_t inPosReadStart, int64_t inPosReadEnd,
                        const std::vector<const VSFrame*>& inFrms, int inFrmsNumStart,
                        const common::OverflowContext& ofCtx);
};


void resampleInit(VSPlugin* plugin, const VSPLUGINAPI* vspapi);
