# VapourSynth-AudioResample
This is an audio sample rate and sample type converter for VapourSynth utilizing [libsamplerate](https://github.com/libsndfile/libsamplerate)

## Usage
```python
ares.Resample(audio: vs.AudioNode,
              sample_rate: int = -1,
              sample_type: int = -1,
              sample_type_s: str = None,
              conv_type: int = 0) -> vs.AudioNode
```

### Arguments

*audio* - input clip (all audio formats)

*sample_rate* - new sample rate (e.g. 16000, 44100, 48000, ...); same as input if negative; default: -1

*sample_type* - new sample type as int; same as input if negative; default: -1
```text
    1 - integer 16-bit
    2 - integer 24-bit
    3 - integer 32-bit
    4 - float   32-bit
```

*sample_type_s* - new sample type as string; default: None
```text
    'i16' - integer 16-bit
    'i24' - integer 24-bit
    'i32' - integer 32-bit
    'f32' - float   32-bit
```

*conv_type* - resample conversion type; see [libsamplerate docs](https://libsndfile.github.io/libsamplerate/api_misc.html#converters) for details; default: 0
```text
    0 - SRC_SINC_BEST_QUALITY
    1 - SRC_SINC_MEDIUM_QUALITY
    2 - SRC_SINC_FASTEST
    3 - SRC_ZERO_ORDER_HOLD
    4 - SRC_LINEAR
```

### Example 1

basic usage
```python
import vapoursynth as vs

# load audio
audio = ...

# convert sample rate to 48000 and sample type to 24-bit integer
audio = vs.core.ares.Resample(audio, sample_rate=48000, sample_type_s='i24', conv_type=0)
```

### Example 2

change speed of an audio clip (but keep the sample rate)
```python
import vapoursynth as vs

# load audio
audio = ...

# speedup factor
factor = 2.0

resample_rate = round(audio.sample_rate / factor)
res_audio = vs.core.ares.Resample(audio, sample_rate=resample_rate)
audio = vs.core.std.AssumeSampleRate(res_audio, samplerate=audio.sample_rate)
```

## Build from source
use cmake to configure your preferred build system and run it\
this will build the dependency libsamplerate as well as this plugin\
e.g. cmake with Ninja:
```sh
# EITHER build with statically linked libsamplerate
cmake -G Ninja -B ./build-ninja -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release

# OR build with dynamically linked libsamplerate
cmake -G Ninja -B ./build-ninja -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release

ninja -C ./build-ninja
```
