// SPDX-License-Identifier: MIT

#include <format>
#include <limits>
#include <string>

#include "VapourSynth4.h"


namespace vsmap
{

static std::string initVarNotFoundErrorMsg(const char* varName, const char* funcName)
{
    return std::format("{}: could not find: {}", funcName, varName);
}


static std::string initOutOfRangeErrorMsg(const char* varName, const char* funcName)
{
    return std::format("{}: out of range: {}", funcName, varName);
}


static std::string initNotAudioClipErrorMsg(const char* varName, const char* funcName)
{
    return std::format("{}: not an audio clip: {}", funcName, varName);
}


static std::string initNotVideoClipErrorMsg(const char* varName, const char* funcName)
{
    return std::format("{}: not a video clip: {}", funcName, varName);
}


bool getAudioClip(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, VSNode** result)
{
    int err = 0;
    VSNode* clip = vsapi->mapGetNode(in, varName, 0, &err);
    if (err)
    {
        vsapi->mapSetError(out, initVarNotFoundErrorMsg(varName, funcName).c_str());
        return false;
    }

    if (vsapi->getNodeType(clip) != VSMediaType::mtAudio)
    {
        vsapi->mapSetError(out, initNotAudioClipErrorMsg(varName, funcName).c_str());
        return false;
    }

    *result = clip;
    return true;
}


bool getOptAudioClip(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, VSNode** result, VSNode* defaultValue)
{
    int err = 0;
    VSNode* clip = vsapi->mapGetNode(in, varName, 0, &err);
    if (err)
    {
        *result = defaultValue;
        return true;
    }

    if (vsapi->getNodeType(clip) != VSMediaType::mtAudio)
    {
        vsapi->mapSetError(out, initNotAudioClipErrorMsg(varName, funcName).c_str());
        return false;
    }

    *result = clip;
    return true;
}


bool getVideoClip(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, VSNode** result)
{
    int err = 0;
    VSNode* clip = vsapi->mapGetNode(in, varName, 0, &err);
    if (err)
    {
        vsapi->mapSetError(out, initVarNotFoundErrorMsg(varName, funcName).c_str());
        return false;
    }

    if (vsapi->getNodeType(clip) != VSMediaType::mtVideo)
    {
        vsapi->mapSetError(out, initNotVideoClipErrorMsg(varName, funcName).c_str());
        return false;
    }

    *result = clip;
    return true;
}


bool getOptVideoClip(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, VSNode** result, VSNode* defaultValue)
{
    int err = 0;
    VSNode* clip = vsapi->mapGetNode(in, varName, 0, &err);
    if (err)
    {
        *result = defaultValue;
        return true;
    }

    if (vsapi->getNodeType(clip) != VSMediaType::mtVideo)
    {
        vsapi->mapSetError(out, initNotVideoClipErrorMsg(varName, funcName).c_str());
        return false;
    }

    *result = clip;
    return true;
}


bool getBool(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, bool* result)
{
    int err = 0;
    int64_t value64 = vsapi->mapGetInt(in, varName, 0, &err);
    if (err)
    {
        vsapi->mapSetError(out, initVarNotFoundErrorMsg(varName, funcName).c_str());
        return false;
    }

    *result = static_cast<bool>(value64);
    return true;
}


bool getOptBool(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, bool defaultValue)
{
    int err = 0;
    int64_t value64 = vsapi->mapGetInt(in, varName, 0, &err);
    if (err)
    {
        return defaultValue;
    }

    return static_cast<bool>(value64);
}


bool getDouble(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, double* result)
{
    int err = 0;
    *result = vsapi->mapGetFloat(in, varName, 0, &err);
    if (err)
    {
        vsapi->mapSetError(out, initVarNotFoundErrorMsg(varName, funcName).c_str());
        return false;
    }

    return true;
}


double getOptDouble(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, double defaultValue)
{
    int err = 0;
    double value64 = vsapi->mapGetFloat(in, varName, 0, &err);
    if (err)
    {
        return defaultValue;
    }

    return value64;
}


bool getFloat(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, float* result)
{
    int err = 0;
    double value64 = vsapi->mapGetFloat(in, varName, 0, &err);
    if (err)
    {
        vsapi->mapSetError(out, initVarNotFoundErrorMsg(varName, funcName).c_str());
        return false;
    }

    if (value64 < std::numeric_limits<float>::min() || std::numeric_limits<float>::max() < value64)
    {
        vsapi->mapSetError(out, initOutOfRangeErrorMsg(varName, funcName).c_str());
        return false;
    }

    *result = static_cast<float>(value64);

    return true;
}


bool getOptFloat(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, float* result, float defaultValue)
{
    int err = 0;
    double value64 = vsapi->mapGetFloat(in, varName, 0, &err);
    if (err)
    {
        *result = defaultValue;
        return true;
    }

    if (value64 < std::numeric_limits<float>::min() || std::numeric_limits<float>::max() < value64)
    {
        vsapi->mapSetError(out, initOutOfRangeErrorMsg(varName, funcName).c_str());
        return false;
    }

    *result = static_cast<float>(value64);

    return true;
}


bool getInt(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, int* result)
{
    int err = 0;
    int64_t value64 = vsapi->mapGetInt(in, varName, 0, &err);
    if (err)
    {
        vsapi->mapSetError(out, initVarNotFoundErrorMsg(varName, funcName).c_str());
        return false;
    }

    if (value64 < std::numeric_limits<int>::min() || std::numeric_limits<int>::max() < value64)
    {
        vsapi->mapSetError(out, initOutOfRangeErrorMsg(varName, funcName).c_str());
        return false;
    }

    *result = static_cast<int>(value64);
    return true;
}


bool getOptInt(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, int* result, int defaultValue)
{
    int err = 0;
    int64_t value64 = vsapi->mapGetInt(in, varName, 0, &err);
    if (err)
    {
        *result = defaultValue;
        return true;
    }

    if (value64 < std::numeric_limits<int>::min() || std::numeric_limits<int>::max() < value64)
    {
        vsapi->mapSetError(out, initOutOfRangeErrorMsg(varName, funcName).c_str());
        return false;
    }

    *result = static_cast<int>(value64);
    return true;
}


bool getInt64(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, int64_t* result)
{
    int err = 0;
    int64_t value64 = vsapi->mapGetInt(in, varName, 0, &err);
    if (err)
    {
        vsapi->mapSetError(out, initVarNotFoundErrorMsg(varName, funcName).c_str());
        return false;
    }

    *result = value64;
    return true;
}


int64_t getOptInt64(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, int64_t defaultValue)
{
    int err = 0;
    int64_t value64 = vsapi->mapGetInt(in, varName, 0, &err);
    if (err)
    {
        return defaultValue;
    }

    return value64;
}

}
