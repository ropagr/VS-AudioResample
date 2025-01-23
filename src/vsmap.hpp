// SPDX-License-Identifier: MIT

#ifndef _VSMAP_HPP
#define _VSMAP_HPP

#include "VapourSynth4.h"

namespace vsmap
{

bool getAudioClip(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, VSNode** result);

bool getOptAudioClip(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, VSNode** result, VSNode* defaultValue);

bool getVideoClip(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, VSNode** result);

bool getOptVideoClip(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, VSNode** result, VSNode* defaultValue);

bool getBool(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, bool* result);

bool getOptBool(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, bool defaultValue);

bool getDouble(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, double* result);

double getOptDouble(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, double defaultValue);

bool getFloat(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, float* result);

bool getOptFloat(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, float* result, float defaultValue);

bool getInt(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, int* result);

bool getOptInt(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, int* result, int defaultValue);

bool getInt64(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, int64_t* result);

int64_t getOptInt64(const char* varName, const char* funcName, const VSMap* in, VSMap* out, const VSAPI* vsapi, int64_t defaultValue);

}

#endif // _VSMAP_HPP
