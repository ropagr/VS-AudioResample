# VS-AudioResample
This is an audio sample rate and sample type converter for VapourSynth utilizing the [SoX Resampler](https://sourceforge.net/projects/soxr/) library.

## Usage
```python
ares.Resample(clip: vs.AudioNode,
              sample_rate: int = -1,
              sample_type: str = None,
              quality: str = 'very_high',
              overflow: str = 'error',
              overflow_log: str = 'once'
              ) -> vs.AudioNode
```

*clip* - input audio clip (any format)

*sample_rate* - new sample rate (e.g. 16000, 44100, 48000, ...); same as input clip if negative; default: -1

*sample_type* - new sample type; same as input clip if None; default: None
```text
    'i16' - integer 16-bit
    'i24' - integer 24-bit
    'i32' - integer 32-bit
    'f32' - float   32-bit
```

*quality* - resample quality; default: 'very_high'
```text
    'quick'     - quick cubic interpolation
    'low'       - low quality
    'medium'    - medium quality
    'high'      - high quality
    'very_high' - very high quality (default)
    'max'       - maximum quality
```

*overflow* - sample overflow handling; default: 'error'
```text
    'error'      - raise an error (default)
    'clip'       - clip overflowing samples (all types)
    'clip_int'   - clip overflowing samples for integer output sample types
                   keep overflowing samples for float output sample types
    'keep_float' - keep overflowing samples for float output sample types
                   raise an error if output sample type is not float
```

To properly handle overflows the clip should be converted to a float sample type ('f32'), if not already.

⚠️ Overflowing samples of integer sample types (output) are always clipped (disruptive), or they raise an error

Use `overflow='keep_float'` for float output sample types to leave overflowing samples unchanged.  
Then call a scaling function like `std.AudioGain` that scales the peak sample value below or to equal 1.0 (see [Example 3](#example-3))


*overflow_log* - sample overflow logging; default: 'once'
```text
    'all'  - log all sample overflows (not recommended, this can be a lot)
    'once' - log only the first sample overflow (default)
    'none' - do not log any sample overflows
```

**Note**: a summary of all overflowing samples will be logged at the end of each function (if any)

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
# leave possible overflowing samples unchanged with 'keep_float'
audio = vs.core.ares.Resample(audio, sample_rate=48000, sample_type='f32', overflow='keep_float')

# scale audio samples
# choose a factor that limits the peak value below or to equal 1
audio = vs.core.std.AudioGain(audio, 0.5)

# optional: convert sample type back to integer if needed (e.g. 24-bit)
audio = vs.core.ares.Resample(audio, sample_type='i24')
```

## Dependencies
This project uses the [SoX Resampler](https://sourceforge.net/projects/soxr/) library (`soxr`), which is licensed under the [GNU Lesser General Public License (LGPL) v2.1](https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html).

The distributed binaries of this plugin statically link to `soxr` for performance and ease of use. If you prefer to use a customized version of `soxr`, or to link dynamically instead, you can build the plugin from source. See the [Build from source](#build-from-source) section for instructions.


## Build from source
To build the plugin, you’ll need CMake and a C++20-compatible compiler. OpenMP support is optional.

Run CMake to configure your preferred build system, then build the project. This process will automatically download and build the `soxr` dependency along with the plugin.

**Note:** You don’t need to download or build `soxr` yourself. However, if you prefer to use a local or customized version of `soxr`, you can provide its source path during configuration (see options below).

e.g. CMake with Ninja:
```sh
# EITHER build with statically linked soxr
cmake -G Ninja -B ./build-ninja -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release

# OR build with dynamically linked soxr
cmake -G Ninja -B ./build-ninja -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release

ninja -C ./build-ninja
```

Custom options for cmake:
```text
-DSOXR_SOURCE_DIR=<PATH>     Use a custom soxr source directory
                             (must contain CMakeLists.txt)
-DSOXR_USE_PATCHES=<ON|OFF>  Enable patches needed for outdated 
                             soxr build scripts (default: ON)
-DWITH_OPENMP=<ON|OFF>       Enable or disable OpenMP support (default: ON)
```


## License

This project is licensed under the MIT License.
