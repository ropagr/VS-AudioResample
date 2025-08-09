// SPDX-License-Identifier: MIT

#ifndef COMMON_OVERFLOW_HPP
#define COMMON_OVERFLOW_HPP

#include <cstdint>
#include <format>
#include <map>
#include <optional>
#include <string>
#include <type_traits>

#include "VapourSynth4.h"

#include "utils/sample.hpp"
#include "vsutils/bitshift.hpp"

namespace common
{
    enum class OverflowMode
    {
        // abort with an error message
        Error,
        // clip all sample types
        Clip,
        // clip integer sample types only, ignore float (i.e. let float overflow)
        ClipIntOnly,
    };


    enum class OverflowLog
    {
        // log all overflowing samples
        All,
        // log only the first overflowing sample
        Once,
        // do not log any overflowing samples
        None,
    };


    struct OverflowContext
    {
        OverflowMode mode;
        OverflowLog log;
        const char* funcName;
        VSFrameContext* frameCtx;
        VSCore* core;
        const VSAPI* vsapi;
    };

    std::map<std::string, OverflowMode> getStringOverflowModeMap();

    std::map<std::string, OverflowLog> getStringOverflowLogMap();

    void logNumOverflows(int64_t numOverflows, const char* funcName, VSCore* core, const VSAPI* vsapi);

    static std::string genOverflowMsg(double sample, int64_t totalPos, int channel, const char* funcName)
    {
        return std::format("{}: Overflow detected. position: {}, channel: {}, sample: {:.4f}", funcName, totalPos, channel, sample);
    }


    static void logOverflow(double sample, int64_t totalPos, int channel, const OverflowContext& ofCtx, int64_t& numOverflows)
    {
        switch (ofCtx.log)
        {
            case OverflowLog::All:
                // log all overflowing samples
                ofCtx.vsapi->logMessage(VSMessageType::mtWarning, genOverflowMsg(sample, totalPos, channel, ofCtx.funcName).c_str(), ofCtx.core);
                break;

            case OverflowLog::Once:
                // log only the first overflowing sample
                // compare with 1 because numOverflows was already incremented for this overflow
                // so we log only if no previous overflow was logged
                if (numOverflows == 1)
                {
                    ofCtx.vsapi->logMessage(VSMessageType::mtWarning, genOverflowMsg(sample, totalPos, channel, ofCtx.funcName).c_str(), ofCtx.core);

                    std::string firstHint = std::format("{}: Only the first overflow will be logged.", ofCtx.funcName);
                    ofCtx.vsapi->logMessage(VSMessageType::mtWarning, firstHint.c_str(), ofCtx.core);
                }
                break;

            case OverflowLog::None:
                break;
        }
    }


    template <typename sample_t, size_t SampleIntBits>
    static std::optional<sample_t> handleOverflow(double sample, int64_t totalPos, int channel, const OverflowContext& ofCtx, int64_t& numOverflows)
    {
        logOverflow(sample, totalPos, channel, ofCtx, numOverflows);

        switch (ofCtx.mode)
        {
            case OverflowMode::Error:
                ofCtx.vsapi->setFilterError(genOverflowMsg(sample, totalPos, channel, ofCtx.funcName).c_str(), ofCtx.frameCtx);
                return std::nullopt;

            case OverflowMode::ClipIntOnly:
                if constexpr (std::is_floating_point_v<sample_t>)
                {
                    // do not clamp float
                    return utils::convSampleFromDouble<sample_t, SampleIntBits>(sample, false);
                }
                [[fallthrough]];

            case OverflowMode::Clip:
                return utils::convSampleFromDouble<sample_t, SampleIntBits>(sample, true);
        }
        return std::nullopt;
    }


    // numOverflows will be incremented if an overflow happened
    template <typename sample_t, size_t SampleIntBits>
    std::optional<sample_t> safeConvertSample(double sample, int64_t totalPos, int channel, const OverflowContext& ofCtx, int64_t& numOverflows)
    {
        if (utils::isSampleOverflowing<double, 0>(sample))
        {
            // sample is overflowing
            ++numOverflows;

            return handleOverflow<sample_t, SampleIntBits>(sample, totalPos, channel, ofCtx, numOverflows);
        }

        // sample is not overflowing
        return utils::convSampleFromDouble<sample_t, SampleIntBits>(sample);
    }


    template <typename sample_t, size_t SampleIntBits>
    bool safeWriteSample(double sample, sample_t* frmPtr, int frmPtrPos, int64_t totalPos, int channel, const OverflowContext& ofCtx, int64_t& numOverflows)
    {
        if (auto optSample = safeConvertSample<sample_t, SampleIntBits>(sample, totalPos, channel, ofCtx, numOverflows))
        {
            sample_t outSample = optSample.value();

            constexpr vsutils::BitShift outBitShift = vsutils::getSampleBitShift<sample_t, SampleIntBits>();

            if constexpr (outBitShift.required)
            {
                outSample <<= outBitShift.count;
            }

            frmPtr[frmPtrPos] = outSample;
            return true;
        }

        // overflow and error
        return false;
    }
}

#endif // COMMON_OVERFLOW_HPP
