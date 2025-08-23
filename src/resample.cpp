// SPDX-License-Identifier: MIT

#include <algorithm>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <soxr.h>

#include "VapourSynth4.h"

#include "resample.hpp"
#include "common/overflow.hpp"
#include "common/resquality.hpp"
#include "common/sampletype.hpp"
#include "utils/debug.hpp"
#include "utils/sample.hpp"
#include "vsmap/vsmap.hpp"
#include "vsmap/vsmap_common.hpp"
#include "vsutils/audio.hpp"
#include "vsutils/bitshift.hpp"

constexpr const char* FuncName = "Resample";

constexpr common::ResampleQuality DefaultResampleQuality = common::ResampleQuality::VeryHigh;
constexpr common::OverflowMode DefaultOverflowMode = common::OverflowMode::Error;
constexpr common::OverflowLog DefaultOverflowLog = common::OverflowLog::Once;


static unsigned long resQualityToSoxrQuality(common::ResampleQuality resQuality)
{
    switch (resQuality)
    {
        case common::ResampleQuality::Quick:
            return SOXR_QQ;
        case common::ResampleQuality::Low:
            return SOXR_LQ;
        case common::ResampleQuality::Medium:
            return SOXR_MQ;
        case common::ResampleQuality::High:
            return SOXR_HQ;
        case common::ResampleQuality::VeryHigh:
            return SOXR_VHQ;
        case common::ResampleQuality::Maximum:
            return SOXR_32_BITQ;
        default:
            return SOXR_VHQ;
    }
}


static int64_t convSamples(int64_t inSample, int inSampleRate, int outSampleRate)
{
    double inSeconds = vsutils::samplesToSeconds(inSample, inSampleRate);
    double outSeconds = inSeconds;
    int64_t outSample = vsutils::secondsToSamples(outSeconds, outSampleRate);
    return outSample;
}


/**
 * number of input samples to cover the output samples (generously extended)
 */
static int calcInBufLen(int numOutSamples, int outSampleRate, int inSampleRate)
{
    return static_cast<int>(convSamples(numOutSamples, outSampleRate, inSampleRate) * 1.1);
}


static bool checkResampleBufferSizes(soxr_t resState, int inBufLen, int outBufLen, int numChannels, int minOutLen)
{
    float* inBuf = new float[inBufLen * numChannels]();
    float* outBuf = new float[outBufLen * numChannels]();

    size_t inSamplesUsed;
    size_t outSamplesGen;

    soxr_error_t error = soxr_process(
        resState,
        inBuf,
        inBufLen,
        &inSamplesUsed,
        outBuf,
        outBufLen,
        &outSamplesGen
    );

    soxr_clear(resState);

    delete[] inBuf;
    delete[] outBuf;

    return (minOutLen < static_cast<int>(outSamplesGen)) && !error;
}


static std::optional<std::pair<int, int>> findFittingBufferSizes(
        soxr_t resState, int inSampleRate, int outSampleRate, int numChannels, int minOutLen)
{
    // soxr usually needs big buffers for the first input samples to fill one output frame
    // try several output length multipliers
    for (int m = 2; m < 100; ++m)
    {
        int outBufLen = static_cast<int>(minOutLen * m);
        int inBufLen = calcInBufLen(outBufLen, outSampleRate, inSampleRate);

        if (checkResampleBufferSizes(resState, inBufLen, outBufLen, numChannels, minOutLen))
        {
            return std::make_pair(inBufLen, outBufLen);
        }
    }
    return std::nullopt;
}


static int setInterleavedSamples(float* buf, int bufLen, float sampleValue, int startSample, int numSamples, int numChannels)
{
    int samplesToSet = std::max(std::min(bufLen - startSample, numSamples), 0);

    for (int ch = 0; ch < numChannels; ++ch)
    {
        for (int s = 0; s < samplesToSet; ++s)
        {
            buf[(startSample + s) * numChannels + ch] = sampleValue;
        }
    }

    return samplesToSet;
}


static int moveInterleavedSamplesLeft(float* buf, int bufLen, int startSample, int numSamples, int numChannels)
{
    int samplesToMove = std::max(std::min(bufLen - startSample, numSamples), 0);

    std::memmove(buf, &buf[startSample * numChannels], sizeof(float) * samplesToMove * numChannels);

    return samplesToMove;
}


std::optional<Resample*> Resample::newResample(
        VSNode* inAudio, const VSAudioInfo* inAudioInfo, int outSampleRate, common::SampleType outSampleType, common::ResampleQuality resQuality,
        common::OverflowMode overflowMode, common::OverflowLog overflowLog, VSMap* out, VSCore* core, const VSAPI* vsapi)
{
    if (inAudioInfo->sampleRate == outSampleRate)
    {
        // no resampling required
        return new Resample(inAudio, inAudioInfo, outSampleRate, outSampleType, overflowMode, overflowLog, nullptr, 0, 0);
    }

    const soxr_io_spec_t io_spec = soxr_io_spec(SOXR_FLOAT32_I, SOXR_FLOAT32_I);
    const soxr_quality_spec_t quality_spec = soxr_quality_spec(resQualityToSoxrQuality(resQuality), 0);

    soxr_error_t error;

    soxr_t resState = soxr_create(
        inAudioInfo->sampleRate,
        outSampleRate,
        inAudioInfo->format.numChannels,
        &error,
        &io_spec,
        &quality_spec,
        nullptr
    );

    if (error)
    {
        std::string errMsg = std::format("{}: soxr_create error: {}", FuncName, error);
        vsapi->mapSetError(out, errMsg.c_str());
        return std::nullopt;
    }

    if (auto optBufSizes = findFittingBufferSizes(resState, inAudioInfo->sampleRate, outSampleRate, inAudioInfo->format.numChannels, VS_AUDIO_FRAME_SAMPLES))
    {
        return new Resample(inAudio, inAudioInfo, outSampleRate, outSampleType, overflowMode, overflowLog, resState, optBufSizes.value().first, optBufSizes.value().second);
    }

    std::string errMsg = std::format("{}: No fitting buffer sizes found", FuncName);
    vsapi->mapSetError(out, errMsg.c_str());
    return std::nullopt;
}


Resample::Resample(VSNode* _inAudio, const VSAudioInfo* _inAudioInfo, int _outSampleRate, common::SampleType _outSampleType,
                   common::OverflowMode _overflowMode, common::OverflowLog _overflowLog, soxr_t _resState, int _inBufLen, int _outBufLen) :
    inAudio(_inAudio), inAudioInfo(*_inAudioInfo), outSampleRate(_outSampleRate),
    inSampleType(common::getSampleTypeFromAudioFormat(_inAudioInfo->format).value()), outSampleType(_outSampleType),
    overflowMode(_overflowMode), overflowLog(_overflowLog), numChannels(_inAudioInfo->format.numChannels),
    resState(_resState), inBufLen(_inBufLen), outBufLen(_outBufLen)
{
    // output audio information
    outAudioInfo = inAudioInfo;
    outAudioInfo.sampleRate = outSampleRate;

    if (inAudioInfo.sampleRate != outSampleRate)
    {
        // overwrite numSamples and numFrames when resampling is required
        outAudioInfo.numSamples = convSamples(inAudioInfo.numSamples, inAudioInfo.sampleRate, outSampleRate);
        outAudioInfo.numFrames = vsutils::sampleCountToFrames(outAudioInfo.numSamples);
    }

    resRatio = static_cast<double>(outAudioInfo.sampleRate) / static_cast<double>(inAudioInfo.sampleRate);

    common::applySampleTypeToAudioFormat(outSampleType, outAudioInfo.format);

    inBuf = new float[inBufLen * numChannels];
    outBuf = new float[outBufLen * numChannels];
}


VSNode* Resample::getInAudio()
{
    return inAudio;
}


const VSAudioInfo& Resample::getInAudioInfo()
{
    return inAudioInfo;
}


const VSAudioInfo& Resample::getOutAudioInfo()
{
    return outAudioInfo;
}


bool Resample::isPassthrough()
{
    return inSampleType == outSampleType && inAudioInfo.sampleRate == outAudioInfo.sampleRate;
}


int Resample::getInBufLen()
{
    return inBufLen;
}


void Resample::logOverflowStats(VSCore* core, const VSAPI* vsapi)
{
    if (0 < overflowStats.count)
    {
        overflowStats.logVS(FuncName, overflowMode, isFloatSampleType(outSampleType), core, vsapi);
    }
}


void Resample::logProcDone(VSCore* core, const VSAPI* vsapi)
{
    if (0 < outBufUsed)
    {
        std::string logMsg = std::format("{}: Process finished. {} unused output sample(s) after the last frame",
                                         FuncName, outBufUsed);
        vsapi->logMessage(VSMessageType::mtWarning, logMsg.c_str(), core);
    }
}


void Resample::free(const VSAPI* vsapi)
{
    vsapi->freeNode(inAudio);

    soxr_delete(resState);

    delete[] inBuf;
    delete[] outBuf;
}


template <typename in_sample_t, size_t InIntSampleBits, typename out_sample_t, size_t OutIntSampleBits>
bool Resample::writeFrameNoResampling(VSFrame* outFrm, int64_t outPosFrmStart, int outFrmLen, const VSFrame* inFrm, const common::OverflowContext& ofCtx)
{
    // for 24-bit audio samples stored in int32_t
    constexpr vsutils::BitShift inBitShift = vsutils::getSampleBitShift<in_sample_t, InIntSampleBits>();

    for (int ch = 0; ch < numChannels; ++ch)
    {
        const in_sample_t* inFrmPtr = reinterpret_cast<const in_sample_t*>(ofCtx.vsapi->getReadPtr(inFrm, ch));
        out_sample_t* outFrmPtr = reinterpret_cast<out_sample_t*>(ofCtx.vsapi->getWritePtr(outFrm, ch));

        for (int s = 0; s < outFrmLen; ++s)
        {
            int64_t outPos = outPosFrmStart + s;

            in_sample_t inSample = inFrmPtr[s];
            if constexpr (inBitShift.required)
            {
                // only for 24-bit integer samples
                inSample >>= inBitShift.count;
            }

            double sample = utils::convSampleToDouble<in_sample_t, InIntSampleBits>(inSample);

            if (!common::safeWriteSample<out_sample_t, OutIntSampleBits>(sample, outFrmPtr, s, outPos, ch, ofCtx, overflowStats))
            {
                // overflow and error
                return false;
            }
        }
    }
    return true;
}


/**
 * fills buf with samples from all channels from 'inPosReadStart' (inclusive) to 'inPosReadEnd' (exclusive)
 * samples will be stored channel interleaved
 * obviously the provided input frames should cover the specified sample range
 */
template <typename in_sample_t, size_t InIntSampleBits>
int Resample::fillInterleavedSamples(float* buf, int bufLen, int bufUsed, int64_t inPosReadStart, int64_t inPosReadEnd,
                                     const std::vector<const VSFrame*>& inFrms, int inFrmsNumStart, const VSAPI* vsapi)
{
    int writtenSamples = 0;

    // for 24-bit audio samples stored in int32_t
    constexpr vsutils::BitShift inBitShift = vsutils::getSampleBitShift<in_sample_t, InIntSampleBits>();

    for (size_t f = 0; f < inFrms.size(); ++f)
    {
        const VSFrame* inFrm = inFrms[f];

        int64_t inPosFrmStart = vsutils::frameToFirstSample(inFrmsNumStart + static_cast<int>(f));
        int64_t inPosFrmEnd = vsutils::frameToLastSample(inFrmsNumStart + static_cast<int>(f), inAudioInfo.numSamples);

        int64_t inPosFrmReadStart = std::max(inPosFrmStart, inPosReadStart);
        int64_t inPosFrmReadEnd = std::min(inPosFrmEnd, inPosReadEnd);

        // samples to read from this frame
        int bufFree = static_cast<int>(bufLen) - bufUsed;
        int inFrmSamplesToRead = std::min(static_cast<int>(inPosFrmReadEnd - inPosFrmReadStart), bufFree);

        if (0 < inFrmSamplesToRead)
        {
            int startPos = static_cast<int>(inPosFrmReadStart - inPosFrmStart);

            for (int ch = 0; ch < numChannels; ++ch)
            {
                const in_sample_t* inFrmPtr = reinterpret_cast<const in_sample_t*>(vsapi->getReadPtr(inFrm, ch));

                for (int s = 0; s < inFrmSamplesToRead; ++s)
                {
                    // s: sample position in inFrm
                    in_sample_t inSample = inFrmPtr[startPos + s];
                    if constexpr (inBitShift.required)
                    {
                        // only for 24-bit integer samples
                        inSample >>= inBitShift.count;
                    }

                    double sample = utils::convSampleToDouble<in_sample_t, InIntSampleBits>(inSample);

                    buf[(bufUsed + s) * numChannels + ch] = utils::convSampleFromDouble<float, 0>(sample, false);
                }
            }

            bufUsed += inFrmSamplesToRead;
            writtenSamples += inFrmSamplesToRead;
        }
    }

    return writtenSamples;
}


template <typename out_sample_t, size_t OutIntSampleBits>
std::optional<int> Resample::writeFrameFromInterleavedSamples(VSFrame* outFrm, int64_t outPosFrmStart, int outFrmLen,
                                                              float* buf, int bufLen, int bufUsed, const common::OverflowContext& ofCtx)
{
    int samplesToWrite = std::min(bufUsed, outFrmLen);

    for (int ch = 0; ch < numChannels; ++ch)
    {
        out_sample_t* outFrmPtr = reinterpret_cast<out_sample_t*>(ofCtx.vsapi->getWritePtr(outFrm, ch));

        for (int s = 0; s < samplesToWrite; ++s)
        {
            int64_t outPos = outPosFrmStart + s;

            double sample = static_cast<double>(buf[s * numChannels + ch]);

            if (!common::safeWriteSample<out_sample_t, OutIntSampleBits>(sample, outFrmPtr, s, outPos, ch, ofCtx, overflowStats))
            {
                // overflow and error
                return std::nullopt;
            }
        }
    }

    return samplesToWrite;
}


bool Resample::resampleChunks(int outFrmNum, int outFrmLen, const common::OverflowContext& ofCtx)
{
    bool endOfInput = inAudioInfo.numSamples <= inPosReadNext;

    while (outBufUsed < outFrmLen)
    {
        // one sample actually means one sample of each channel
        size_t inSamplesUsed;
        size_t outSamplesGen;

        // resample samples in inBuf and write to outBuf
        soxr_error_t error = soxr_process(
            resState,
            endOfInput && (inBufUsed <= 0) ? nullptr : inBuf,
            inBufUsed,
            &inSamplesUsed,
            &outBuf[outBufUsed * numChannels],
            outBufLen - outBufUsed,
            &outSamplesGen
        );

        if (error)
        {
            std::string logMsg = std::format("{}: soxr_process error in frame: {}, error: {}",
                                             FuncName, outFrmNum, error);
            ofCtx.vsapi->logMessage(VSMessageType::mtCritical, logMsg.c_str(), ofCtx.core);
            ofCtx.vsapi->setFilterError(logMsg.c_str(), ofCtx.frameCtx);
            return false;
        }

        totalUsedInSamples += inSamplesUsed;
        totalGenOutSamples += outSamplesGen;

        inBufUsed -= static_cast<int>(inSamplesUsed);
        outBufUsed += static_cast<int>(outSamplesGen);

        moveInterleavedSamplesLeft(inBuf, inBufLen, static_cast<int>(inSamplesUsed), inBufUsed, numChannels);

        // debugging message
        /*
        double outDelay = soxr_delay(resState);
        std::string logMsg = std::format("{}: resampleChunks: frame: {}, inSamplesUsed: {}, outSamplesGen: {}, endOfInput: {}, outDelay: {}, inBufUsed: {}, outBufUsed: {}, totalUsedInSamples: {}, totalGenOutSamples: {}",
                                         FuncName, outFrmNum, inSamplesUsed, outSamplesGen, endOfInput, outDelay, inBufUsed, outBufUsed, totalUsedInSamples, totalGenOutSamples);
        ofCtx.vsapi->logMessage(VSMessageType::mtInformation, logMsg.c_str(), ofCtx.core);
        */

        if (inSamplesUsed == 0 && outSamplesGen == 0)
        {
            // no changes to input or output buffer
            break;
        }
    }

    return true;
}


template <typename in_sample_t, size_t InIntSampleBits, typename out_sample_t, size_t OutIntSampleBits>
bool Resample::writeFrameImpl(VSFrame* outFrm, int outFrmNum, int64_t inPosReadStart, int64_t inPosReadEnd,
                              const std::vector<const VSFrame*>& inFrms, int inFrmsNumStart,
                              const common::OverflowContext& ofCtx)
{
    int64_t outPosFrmStart = vsutils::frameToFirstSample(outFrmNum);
    int outFrmLen = ofCtx.vsapi->getFrameLength(outFrm);

    assertm(outFrmLen <= outBufLen, "output buffer too small");

    if (inAudioInfo.sampleRate == outAudioInfo.sampleRate)
    {
        assertm(inFrms.size() == 1, "exactly 1 input frame expected if sample rates are equal");

        return writeFrameNoResampling<in_sample_t, InIntSampleBits, out_sample_t, OutIntSampleBits>(
                outFrm, outPosFrmStart, outFrmLen, inFrms[0], ofCtx);
    }

    if (lastOutFrmNum + 1 == outFrmNum)
    {
        // subsequent frames
        // there are probably already some read input samples which should not be read again
        if (inPosReadStart <= inPosReadNext)
        {
            inPosReadStart = inPosReadNext;
        }
        else
        {
            // inPosReadNext < inPosReadStart
            // not supposed to happen since we always consume more input samples than needed
            int64_t missingSamples = inPosReadStart - inPosReadNext;

            std::string logMsg = std::format("{}: {} missing input sample(s) in frame {}, last consumed input sample: {}, next input sample: {}",
                                             FuncName, missingSamples, outFrmNum, inPosReadNext - 1, inPosReadStart);
            ofCtx.vsapi->logMessage(VSMessageType::mtCritical, logMsg.c_str(), ofCtx.core);
            ofCtx.vsapi->setFilterError(logMsg.c_str(), ofCtx.frameCtx);
            return false;
        }
    }
    else
    {
        // current frame is *not* a subsequent frame -> reset state
        inBufUsed = 0;
        outBufUsed = 0;
        inPosReadNext = 0;

        lastOutFrmNum = -10;

        totalUsedInSamples = 0;
        totalGenOutSamples = 0;

        soxr_error_t error = soxr_clear(resState);
        if (error)
        {
            std::string errMsg = std::format("{}: soxr_create error: {}", FuncName, error);
            ofCtx.vsapi->logMessage(VSMessageType::mtCritical, errMsg.c_str(), ofCtx.core);
            ofCtx.vsapi->setFilterError(errMsg.c_str(), ofCtx.frameCtx);
            return false;
        }

        overflowStats = { .count = 0, .peak = 0.0 };
    }

    // can safely be casted to an int since the number of source samples
    // will not exceed 256 * VS_AUDIO_FRAME_SAMPLES (3072) = 786432
    // with 256 being the maximum resampling factor of libsamplerate
    int inSamplesToRead = static_cast<int>(inPosReadEnd - inPosReadStart);

    if (0 < inSamplesToRead)
    {
        // fill source buffer

        int readSamples = fillInterleavedSamples<in_sample_t, InIntSampleBits>(
                inBuf, inBufLen, inBufUsed, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx.vsapi);

        inBufUsed += readSamples;

        inPosReadNext = inPosReadStart + readSamples;
    }

    if (!resampleChunks(outFrmNum, outFrmLen, ofCtx))
    {
        return false;
    }

    // handle output samples

    if (outBufUsed < outFrmLen)
    {
        // not enough output samples generated
        // not supposed to happen (rounding errors only?)

        int missingSamples = outFrmLen - outBufUsed;

        if (outFrmNum < outAudioInfo.numFrames - 1)
        {
            // outFrm is *not* the last frame
            // not supposed to happen
            std::string logMsg = std::format("{}: Not enough output samples generated for frame: {}. Missing samples: {}",
                                             FuncName, outFrmNum, missingSamples);
            // this is critical
            ofCtx.vsapi->logMessage(VSMessageType::mtCritical, logMsg.c_str(), ofCtx.core);
            ofCtx.vsapi->setFilterError(logMsg.c_str(), ofCtx.frameCtx);
            return false;
        }

        // last frame
        double soxrDelay = soxr_delay(resState);

        std::string logMsg = std::format("{}: Process finished. Not enough output samples generated for the last frame {}. Last {} sample(s) will be muted. "
                                         "soxr_delay: {}, inPosReadNext: {}, inBufUsed: {}",
                                         FuncName, outFrmNum, missingSamples, soxrDelay, inPosReadNext, inBufUsed);
        ofCtx.vsapi->logMessage(VSMessageType::mtWarning, logMsg.c_str(), ofCtx.core);

        // mute missing samples
        setInterleavedSamples(outBuf, outBufLen, 0, outBufUsed, outFrmLen - outBufUsed, numChannels);

        outBufUsed = outFrmLen;
    }

    // write samples from output buffer to output frame
    std::optional<int> optWritten = writeFrameFromInterleavedSamples<out_sample_t, OutIntSampleBits>(
            outFrm, outPosFrmStart, outFrmLen, outBuf, outBufLen, outBufUsed, ofCtx);
    if (!optWritten.has_value())
    {
        // overflow and error
        return false;
    }

    outBufUsed -= optWritten.value();

    // shift remaining ouput samples to the left
    moveInterleavedSamplesLeft(outBuf, outBufLen, optWritten.value(), outBufUsed, numChannels);

    lastOutFrmNum = outFrmNum;
    return true;
}


bool Resample::writeFrame(VSFrame* outFrm, int outFrmNum, int64_t inPosReadStart, int64_t inPosReadEnd,
                          std::vector<const VSFrame*> const &inFrms, int inFrmsNumStart,
                          VSFrameContext* frameCtx, VSCore* core, const VSAPI* vsapi)
{
    common::OverflowContext ofCtx =
        { .mode = overflowMode, .log = overflowLog, .funcName = FuncName,
          .frameCtx = frameCtx, .core = core, .vsapi = vsapi };

    switch (inSampleType)
    {
        case common::SampleType::Int8:
            switch (outSampleType)
            {
                case common::SampleType::Int8:
                    return writeFrameImpl<int8_t, 8, int8_t, 8>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Int16:
                    return writeFrameImpl<int8_t, 8, int16_t, 16>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Int24:
                    return writeFrameImpl<int8_t, 8, int32_t, 24>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Int32:
                    return writeFrameImpl<int8_t, 8, int32_t, 32>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Float32:
                    return writeFrameImpl<int8_t, 8, float, 0>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Float64:
                    return writeFrameImpl<int8_t, 8, double, 0>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                default:
                    return false;
            }

        case common::SampleType::Int16:
            switch (outSampleType)
            {
                case common::SampleType::Int8:
                    return writeFrameImpl<int16_t, 16, int8_t, 8>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Int16:
                    return writeFrameImpl<int16_t, 16, int16_t, 16>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Int24:
                    return writeFrameImpl<int16_t, 16, int32_t, 24>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Int32:
                    return writeFrameImpl<int16_t, 16, int32_t, 32>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Float32:
                    return writeFrameImpl<int16_t, 16, float, 0>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Float64:
                    return writeFrameImpl<int16_t, 16, double, 0>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                default:
                    return false;
            }

        case common::SampleType::Int24:
            switch (outSampleType)
            {
                case common::SampleType::Int8:
                    return writeFrameImpl<int32_t, 24, int8_t, 8>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Int16:
                    return writeFrameImpl<int32_t, 24, int16_t, 16>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Int24:
                    return writeFrameImpl<int32_t, 24, int32_t, 24>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Int32:
                    return writeFrameImpl<int32_t, 24, int32_t, 32>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Float32:
                    return writeFrameImpl<int32_t, 24, float, 0>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Float64:
                    return writeFrameImpl<int32_t, 24, double, 0>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                default:
                    return false;
            }

        case common::SampleType::Int32:
            switch (outSampleType)
            {
                case common::SampleType::Int8:
                    return writeFrameImpl<int32_t, 32, int8_t, 8>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Int16:
                    return writeFrameImpl<int32_t, 32, int16_t, 16>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Int24:
                    return writeFrameImpl<int32_t, 32, int32_t, 24>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Int32:
                    return writeFrameImpl<int32_t, 32, int32_t, 32>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Float32:
                    return writeFrameImpl<int32_t, 32, float, 0>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Float64:
                    return writeFrameImpl<int32_t, 32, double, 0>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                default:
                    return false;
            }

        case common::SampleType::Float32:
            switch (outSampleType)
            {
                case common::SampleType::Int8:
                    return writeFrameImpl<float, 0, int8_t, 8>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Int16:
                    return writeFrameImpl<float, 0, int16_t, 16>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Int24:
                    return writeFrameImpl<float, 0, int32_t, 24>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Int32:
                    return writeFrameImpl<float, 0, int32_t, 32>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Float32:
                    return writeFrameImpl<float, 0, float, 0>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Float64:
                    return writeFrameImpl<float, 0, double, 0>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                default:
                    return false;
            }

        case common::SampleType::Float64:
            switch (outSampleType)
            {
                case common::SampleType::Int8:
                    return writeFrameImpl<double, 0, int8_t, 8>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Int16:
                    return writeFrameImpl<double, 0, int16_t, 16>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Int24:
                    return writeFrameImpl<double, 0, int32_t, 24>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Int32:
                    return writeFrameImpl<double, 0, int32_t, 32>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Float32:
                    return writeFrameImpl<double, 0, float, 0>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                case common::SampleType::Float64:
                    return writeFrameImpl<double, 0, double, 0>(outFrm, outFrmNum, inPosReadStart, inPosReadEnd, inFrms, inFrmsNumStart, ofCtx);
                default:
                    return false;
            }

        default:
            return false;
    }
}


static void VS_CC resampleFree(void* instanceData, VSCore* core, const VSAPI* vsapi)
{
    Resample* data = static_cast<Resample*>(instanceData);
    data->free(vsapi);

    delete data;
}


static const VSFrame* VS_CC resampleGetFrame(int outFrmNum, int activationReason, void* instanceData, void** frameData, VSFrameContext* frameCtx, VSCore* core, const VSAPI* vsapi)
{
    Resample* data = static_cast<Resample*>(instanceData);

    bool passthrough = data->isPassthrough();

    // inclusive
    int64_t outPosFrmStart = vsutils::frameToFirstSample(outFrmNum);

    int outFrmLen = vsutils::getFrameSampleCount(outFrmNum, data->getOutAudioInfo().numSamples);

    // exclusive
    int64_t outPosFrmEnd = outPosFrmStart + outFrmLen;

    // required input samples to cover the output frame
    // inclusive
    int64_t inPosReadStart;
    // exclusive
    int64_t inPosReadEnd;

    if (data->getInAudioInfo().sampleRate == data->getOutAudioInfo().sampleRate)
    {
        // no resampling required, avoid any rounding errors
        inPosReadStart = outPosFrmStart;
        inPosReadEnd = outPosFrmEnd;
    }
    else
    {
        inPosReadStart = convSamples(outPosFrmStart, data->getOutAudioInfo().sampleRate, data->getInAudioInfo().sampleRate);
        inPosReadEnd = inPosReadStart + data->getInBufLen();
    }

    inPosReadEnd = std::min(inPosReadEnd, data->getInAudioInfo().numSamples);

    // inclusive
    int inFrmsNumStart = vsutils::sampleToFrame(inPosReadStart);
    // exclusive
    int inFrmsNumEnd = vsutils::sampleToFrame(inPosReadEnd - 1) + 1;

    int inFrmsSize = inFrmsNumEnd - inFrmsNumStart;

    if (activationReason == VSActivationReason::arInitial)
    {
        if (passthrough)
        {
            vsapi->requestFrameFilter(outFrmNum, data->getInAudio(), frameCtx);
            return nullptr;
        }

        for (int i = 0; i < inFrmsSize; ++i)
        {
            vsapi->requestFrameFilter(inFrmsNumStart + i, data->getInAudio(), frameCtx);
        }

        return nullptr;
    }

    if (activationReason == VSActivationReason::arAllFramesReady)
    {
        if (passthrough)
        {
            return vsapi->getFrameFilter(outFrmNum, data->getInAudio(), frameCtx);
        }

        std::vector<const VSFrame*> inFrms;
        inFrms.reserve(inFrmsSize);

        for (int i = 0; i < inFrmsSize; ++i)
        {
            inFrms.push_back(vsapi->getFrameFilter(inFrmsNumStart + i, data->getInAudio(), frameCtx));
        }

        VSFrame* outFrm = vsapi->newAudioFrame(&data->getOutAudioInfo().format, outFrmLen, nullptr, core);

        bool success = data->writeFrame(outFrm, outFrmNum, inPosReadStart, inPosReadEnd,
                                        inFrms, inFrmsNumStart, frameCtx, core, vsapi);

        for (const auto& inFrm : inFrms)
        {
            vsapi->freeFrame(inFrm);
        }

        if (outFrmNum == data->getOutAudioInfo().numFrames - 1)
        {
            // last frame
            data->logProcDone(core, vsapi);

            data->logOverflowStats(core, vsapi);
        }

        if (success)
        {
            return outFrm;
        }
        vsapi->freeFrame(outFrm);
    }

    return nullptr;
}


static void VS_CC resampleCreate(const VSMap* in, VSMap* out, void* userData, VSCore* core, const VSAPI* vsapi)
{
    // clip:anode
    int err = 0;
    VSNode* inAudio = vsapi->mapGetNode(in, "clip", 0, &err);
    if (err)
    {
        return;
    }

    const VSAudioInfo* inAudioInfo = vsapi->getAudioInfo(inAudio);

    // check for supported audio format
    auto optInSampleType = common::getSampleTypeFromAudioFormat(inAudioInfo->format);
    if (!optInSampleType.has_value())
    {
        std::string errMsg = std::format("{}: unsupported audio format", FuncName);
        vsapi->mapSetError(out, errMsg.c_str());
        vsapi->freeNode(inAudio);
        return;
    }

    // sample_rate:int:opt
    int outSampleRate = vsmap::getOptInt("sample_rate", in, vsapi, -1);
    if (outSampleRate <= 0)
    {
        outSampleRate = inAudioInfo->sampleRate;
    }

    // sample_type:data:opt
    std::optional<common::SampleType> optOutSampleType = vsmap::getOptVapourSynthSampleTypeFromString("sample_type", FuncName, in, out, vsapi, optInSampleType.value());
    if (!optOutSampleType.has_value())
    {
        vsapi->freeNode(inAudio);
        return;
    }

    // quality:data:opt
    std::optional<common::ResampleQuality> optResQuality = vsmap::getOptResampleQualityFromString("quality", FuncName, in, out, vsapi, DefaultResampleQuality);
    if (!optResQuality.has_value())
    {
        vsapi->freeNode(inAudio);
        return;
    }

    // overflow:data:opt
    std::optional<common::OverflowMode> optOverflowMode = vsmap::getOptOverflowModeFromString("overflow", FuncName, in, out, vsapi, DefaultOverflowMode);
    if (!optOverflowMode.has_value())
    {
        vsapi->freeNode(inAudio);
        return;
    }

    if (optOverflowMode.value() == common::OverflowMode::KeepFloat && !common::isFloatSampleType(optOutSampleType.value()))
    {
        std::string errMsg = std::format("{}: cannot use 'keep_float' overflow mode with an integer sample type", FuncName);
        vsapi->mapSetError(out, errMsg.c_str());
        vsapi->freeNode(inAudio);
        return;
    }

    // overflow_log:data:opt
    std::optional<common::OverflowLog> optOverflowLog = vsmap::getOptOverflowLogFromString("overflow_log", FuncName, in, out, vsapi, DefaultOverflowLog);
    if (!optOverflowLog.has_value())
    {
        vsapi->freeNode(inAudio);
        return;
    }


    std::optional<Resample*> optData = Resample::newResample(inAudio, inAudioInfo, outSampleRate, optOutSampleType.value(), optResQuality.value(), optOverflowMode.value(), optOverflowLog.value(), out, core, vsapi);
    if (!optData.has_value())
    {
        vsapi->freeNode(inAudio);
        return;
    }

    VSFilterDependency deps[] = {{ inAudio, VSRequestPattern::rpGeneral }};

    // fmParallelRequests: strict sequential frame requests
    vsapi->createAudioFilter(out, FuncName, &optData.value()->getOutAudioInfo(), resampleGetFrame, resampleFree, VSFilterMode::fmParallelRequests, deps, 1, optData.value(), core);
}


void resampleInit(VSPlugin* plugin, const VSPLUGINAPI* vspapi)
{
    vspapi->registerFunction(FuncName,
                             "clip:anode;"
                             "sample_rate:int:opt;"
                             "sample_type:data:opt;"
                             "quality:data:opt;"
                             "overflow:data:opt;"
                             "overflow_log:data:opt;",
                             "return:anode;",
                             resampleCreate, nullptr, plugin);
}
