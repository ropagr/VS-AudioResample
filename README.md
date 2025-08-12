# VS-AudioResample
This is an audio sample rate and sample type converter for VapourSynth utilizing [libsamplerate](https://github.com/libsndfile/libsamplerate).

## Usage
```python
ares.Resample(clip: vs.AudioNode,
              sample_rate: int = -1,
              sample_type: str = None,
              conv_type: int = 0,
              overflow: str = 'error',
              overflow_log: str = 'once'
              ) -> vs.AudioNode
```

### Arguments

*clip* - audio input clip (any format)

*sample_rate* - new sample rate (e.g. 16000, 44100, 48000, ...); same as input clip if negative; default: -1

*sample_type* - new sample type; same as input clip if None; default: None
```text
    'i16' - integer 16-bit
    'i24' - integer 24-bit
    'i32' - integer 32-bit
    'f32' - float   32-bit
```

*conv_type* - resample conversion type; see [libsamplerate docs](https://libsndfile.github.io/libsamplerate/api_misc.html#converters) for details; default: 0
```text
    0 - SRC_SINC_BEST_QUALITY (default)
    1 - SRC_SINC_MEDIUM_QUALITY
    2 - SRC_SINC_FASTEST
    3 - SRC_ZERO_ORDER_HOLD
    4 - SRC_LINEAR
```

*overflow* - sample overflow handling; default: 'error'
```text
    'error'         - raise an error (default)
    'clip'          - clip overflowing samples
    'clip_int_only' - clip overflowing samples only for integer output sample types
                      i.e. keep overflowing samples for float output sample types;
                      use this with sample_type='f32' if you want to handle overflowing samples
                      afterwards with another filter like AudioGain
    'ignore_float'  - an alias for 'clip_int_only'
```

**Note**: overflowing samples are always clipped for integer sample types


*overflow_log* - sample overflow logging; default: 'once'
```text
    'all'  - log all sample overflows (not recommended, this can be a lot)
    'once' - log only the first sample overflow (default)
    'none' - do not log any sample overflows
```

**Note**: a summary of all overflowing samples will be logged at the end of the process (if any)

### Example 1

basic usage
```python
import vapoursynth as vs

# load audio
audio = ...

# convert sample rate to 48000 and sample type to 24-bit integer
audio = vs.core.ares.Resample(audio, sample_rate=48000, sample_type='i24')
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


### Example 3

handle overflowing samples
```python
import vapoursynth as vs

# load audio (integer or float sample type)
audio = ...

# convert sample rate to 48000 and sample type to 32-bit float
# overflowing samples will be kept
audio = vs.core.ares.Resample(audio, sample_rate=48000, sample_type='f32', overflow='clip_int_only')

# scale audio samples
# choose a factor that limits the peak value below or to equal 1
audio = vs.core.std.AudioGain(audio, 0.5)

# optional: convert sample type to integer if needed (e.g. 24-bit)
audio = vs.core.ares.Resample(audio, sample_type='i24')
```

## Dependencies
This project uses [libsamplerate](https://github.com/libsndfile/libsamplerate) for audio sample rate conversion.

You do **not** need to install it separately. The build script will automatically download and compile the library as part of the build process.


## Build from source
Use cmake to configure your preferred build system and run it.\
This will build the dependency libsamplerate as well as this plugin\
e.g. cmake with Ninja:
```sh
# EITHER build with statically linked libsamplerate
cmake -G Ninja -B ./build-ninja -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release

# OR build with dynamically linked libsamplerate
cmake -G Ninja -B ./build-ninja -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release

ninja -C ./build-ninja
```

## License

This project is licensed under the MIT License.
