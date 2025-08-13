// SPDX-License-Identifier: MIT

#pragma once

#include <concepts>
#include <cstddef>
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
        // clip integer sample types only, keep float (i.e. let float overflow)
        ClipInt,
        // keep float, raise an error if clip is not float
        KeepFloat,
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


    struct OverflowStats
    {
        int64_t count = 0;
        double peak = 0.0;

        void addSample(double sample);

        void logVS(const char* funcName, OverflowMode ofMode, bool floatSampleType, VSCore* core, const VSAPI* vsapi);
    };


    std::map<std::string, OverflowMode> getStringOverflowModeMap();

    std::map<std::string, OverflowLog> getStringOverflowLogMap();

    void logNumOverflows(int64_t numOverflows, const char* funcName, VSCore* core, const VSAPI* vsapi);

    static std::string genOverflowMsg(double sample, int64_t totalPos, int channel, const char* funcName)
    {
        return std::format("{}: Overflow detected. position: {}, channel: {}, sample: {:.6f}", funcName, totalPos, channel, sample);
    }


    template <typename sample_t>
    requires std::integral<sample_t> || std::floating_point<sample_t>
    static std::string genOverflowHandlingMsg(const OverflowContext& ofCtx)
    {
        switch (ofCtx.mode)
        {
            case OverflowMode::Error:
                return std::format("{}: Exiting with an error.", ofCtx.funcName);

            case OverflowMode::ClipInt:
            case OverflowMode::KeepFloat:
                if constexpr (std::is_floating_point_v<sample_t>)
                {
                    return std::format("{}: Overflowing samples will *not* be clipped.", ofCtx.funcName);
                }
                [[fallthrough]];

            case OverflowMode::Clip:
                return std::format("{}: Overflowing samples will be clipped.", ofCtx.funcName);

            default:
                return std::string();
        }
    }


    template <typename sample_t>
    requires std::integral<sample_t> || std::floating_point<sample_t>
    static void logOverflow(double sample, int64_t totalPos, int channel, const OverflowContext& ofCtx, const OverflowStats& ofStats)
    {
        switch (ofCtx.log)
        {
            case OverflowLog::All:
                // log all overflowing samples
                ofCtx.vsapi->logMessage(VSMessageType::mtWarning, genOverflowMsg(sample, totalPos, channel, ofCtx.funcName).c_str(), ofCtx.core);

                if (ofStats.count == 0)
                {
                    ofCtx.vsapi->logMessage(VSMessageType::mtInformation, genOverflowHandlingMsg<sample_t>(ofCtx).c_str(), ofCtx.core);
                }
                break;

            case OverflowLog::Once:
                // log only the first overflowing sample
                if (ofStats.count == 0)
                {
                    bool errorMode = ofCtx.mode == OverflowMode::Error;

                    ofCtx.vsapi->logMessage(errorMode ? VSMessageType::mtCritical : VSMessageType::mtWarning, genOverflowMsg(sample, totalPos, channel, ofCtx.funcName).c_str(), ofCtx.core);

                    ofCtx.vsapi->logMessage(VSMessageType::mtInformation, genOverflowHandlingMsg<sample_t>(ofCtx).c_str(), ofCtx.core);

                    if (!errorMode)
                    {
                        std::string firstHint = std::format("{}: Only the first overflow will be logged.", ofCtx.funcName);
                        ofCtx.vsapi->logMessage(VSMessageType::mtInformation, firstHint.c_str(), ofCtx.core);
                    }
                }
                break;

            case OverflowLog::None:
                break;
        }
    }


    template <typename sample_t, size_t IntSampleBits>
    requires std::integral<sample_t> || std::floating_point<sample_t>
    static std::optional<sample_t> handleOverflow(double sample, int64_t totalPos, int channel, const OverflowContext& ofCtx, OverflowStats& ofStats)
    {
        logOverflow<sample_t>(sample, totalPos, channel, ofCtx, ofStats);

        ofStats.addSample(sample);

        switch (ofCtx.mode)
        {
            case OverflowMode::Error:
                ofCtx.vsapi->setFilterError(genOverflowMsg(sample, totalPos, channel, ofCtx.funcName).c_str(), ofCtx.frameCtx);
                return std::nullopt;

            case OverflowMode::KeepFloat:
                if constexpr (std::is_integral_v<sample_t>)
                {
                    ofCtx.vsapi->setFilterError(std::format("{}: Overflow detected. keep_float cannot be used with integer sample types", ofCtx.funcName).c_str(), ofCtx.frameCtx);
                    return std::nullopt;
                }
                [[fallthrough]];

            case OverflowMode::ClipInt:
                if constexpr (std::is_floating_point_v<sample_t>)
                {
                    // do not clamp float
                    return utils::convSampleFromDouble<sample_t, IntSampleBits>(sample, false);
                }
                [[fallthrough]];

            case OverflowMode::Clip:
                return utils::convSampleFromDouble<sample_t, IntSampleBits>(sample, true);
        }
        return std::nullopt;
    }


    // numOverflows will be incremented if an overflow happened
    template <typename sample_t, size_t IntSampleBits>
    requires std::integral<sample_t> || std::floating_point<sample_t>
    std::optional<sample_t> safeConvertSample(double sample, int64_t totalPos, int channel, const OverflowContext& ofCtx, OverflowStats& ofStats)
    {
        if (utils::isSampleOverflowing<double, 0>(sample))
        {
            // sample is overflowing
            return handleOverflow<sample_t, IntSampleBits>(sample, totalPos, channel, ofCtx, ofStats);
        }

        // sample is not overflowing
        return utils::convSampleFromDouble<sample_t, IntSampleBits>(sample);
    }


    template <typename sample_t, size_t IntSampleBits>
    requires std::integral<sample_t> || std::floating_point<sample_t>
    bool safeWriteSample(double sample, sample_t* frmPtr, int frmPtrPos, int64_t totalPos, int channel, const OverflowContext& ofCtx, OverflowStats& ofStats)
    {
        if (auto optSample = safeConvertSample<sample_t, IntSampleBits>(sample, totalPos, channel, ofCtx, ofStats))
        {
            sample_t outSample = optSample.value();

            constexpr vsutils::BitShift outBitShift = vsutils::getSampleBitShift<sample_t, IntSampleBits>();

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
