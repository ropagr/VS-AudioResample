// SPDX-License-Identifier: MIT

#include "VapourSynth4.h"

#include "config.hpp"
#include "resample.hpp"


VS_EXTERNAL_API(void) VapourSynthPluginInit2(VSPlugin* plugin, const VSPLUGINAPI* vspapi)
{
    vspapi->configPlugin("com.ropagr.ares", "ares", "audio sample rate and sample type converter", VS_MAKE_VERSION(0, 3), VAPOURSYNTH_API_VERSION, 0, plugin);

    resampleInit(plugin, vspapi);
}
