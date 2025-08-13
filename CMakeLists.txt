cmake_minimum_required(VERSION 3.18)

project(
    VS-AudioResample
    DESCRIPTION "audio sample rate and sample type converter for VapourSynth"
    VERSION 0.3.0
    LANGUAGES CXX
)

include(FetchContent)
FetchContent_Declare(
    libsamplerate
    GIT_REPOSITORY https://github.com/libsndfile/libsamplerate.git
    GIT_PROGRESS ON
)

FetchContent_MakeAvailable(libsamplerate)

add_library(AudioResample SHARED
    ${CMAKE_SOURCE_DIR}/src/plugin.cpp
    ${CMAKE_SOURCE_DIR}/src/resample.cpp
    ${CMAKE_SOURCE_DIR}/src/resample.hpp
    ${CMAKE_SOURCE_DIR}/src/common/overflow.cpp
    ${CMAKE_SOURCE_DIR}/src/common/overflow.hpp
    ${CMAKE_SOURCE_DIR}/src/common/sampletype.cpp
    ${CMAKE_SOURCE_DIR}/src/common/sampletype.hpp
    ${CMAKE_SOURCE_DIR}/src/utils/array.hpp
    ${CMAKE_SOURCE_DIR}/src/utils/debug.hpp
    ${CMAKE_SOURCE_DIR}/src/utils/map.hpp
    ${CMAKE_SOURCE_DIR}/src/utils/number.hpp
    ${CMAKE_SOURCE_DIR}/src/utils/sample.hpp
    ${CMAKE_SOURCE_DIR}/src/utils/string.cpp
    ${CMAKE_SOURCE_DIR}/src/utils/string.hpp
    ${CMAKE_SOURCE_DIR}/src/vsmap/vsmap.cpp
    ${CMAKE_SOURCE_DIR}/src/vsmap/vsmap.hpp
    ${CMAKE_SOURCE_DIR}/src/vsmap/vsmap_common.cpp
    ${CMAKE_SOURCE_DIR}/src/vsmap/vsmap_common.hpp
    ${CMAKE_SOURCE_DIR}/src/vsutils/audio.cpp
    ${CMAKE_SOURCE_DIR}/src/vsutils/audio.hpp
    ${CMAKE_SOURCE_DIR}/src/vsutils/bitshift.hpp
)

target_include_directories(AudioResample
    PRIVATE ${CMAKE_SOURCE_DIR}/include/vapoursynth
            ${CMAKE_SOURCE_DIR}/src
)

target_compile_features(AudioResample PRIVATE cxx_std_20)

target_link_libraries(AudioResample samplerate)

target_compile_options(AudioResample
    PRIVATE
        # Debug flags for GNU C++
        $<$<AND:$<CONFIG:Debug>,$<COMPILE_LANG_AND_ID:CXX,GNU>>:
            -g -O0 -Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-unused-variable
        >

        # Release flags for GNU C++
        $<$<AND:$<CONFIG:Release>,$<COMPILE_LANG_AND_ID:CXX,GNU>>:
            -O2 -Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-unused-variable
        >

        # Debug flags for MSVC C++
        $<$<AND:$<CONFIG:Debug>,$<COMPILE_LANG_AND_ID:CXX,MSVC>>:
            /MDd /W4 /Zi /wd4100 /wd4702 /Zc:__cplusplus
        >

        # Release flags for MSVC C++
        $<$<AND:$<CONFIG:Release>,$<COMPILE_LANG_AND_ID:CXX,MSVC>>:
            /MD /O2 /W4 /wd4100 /wd4702 /Zc:__cplusplus
        >
)


if (WIN32)
    target_link_options(AudioResample
        PRIVATE $<$<LINK_LANG_AND_ID:CXX,GNU>:-fPIC -static -static-libgcc -static-libstdc++ -s>
    )
else()
    target_link_options(AudioResample
        PRIVATE $<$<LINK_LANG_AND_ID:CXX,GNU>:-s>
    )
endif()

set_target_properties(AudioResample PROPERTIES PREFIX "")
