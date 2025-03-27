cmake_minimum_required(VERSION 3.18)

project(
    VapourSynth-AudioResample
    DESCRIPTION "audio sample rate and sample type converter for VapourSynth"
    VERSION 0.1.1
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
    ${CMAKE_SOURCE_DIR}/src/sampletype.cpp
    ${CMAKE_SOURCE_DIR}/src/vsautils.cpp
    ${CMAKE_SOURCE_DIR}/src/vsmap.cpp
    ${CMAKE_SOURCE_DIR}/src/vsmap_st.cpp
    ${CMAKE_SOURCE_DIR}/src/resample.hpp
    ${CMAKE_SOURCE_DIR}/src/sampletype.hpp
    ${CMAKE_SOURCE_DIR}/src/vsautils.hpp
    ${CMAKE_SOURCE_DIR}/src/vsmap.hpp
    ${CMAKE_SOURCE_DIR}/src/vsmap_st.hpp
)

target_include_directories(AudioResample
    PRIVATE ${CMAKE_SOURCE_DIR}/include/vapoursynth
)

target_compile_features(AudioResample PRIVATE cxx_std_20)

target_link_libraries(AudioResample samplerate)

target_compile_options(AudioResample
    PRIVATE $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-O2 -Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-unused-variable>
            $<$<COMPILE_LANG_AND_ID:CXX,MSVC>:/MD /O2 /W4 /wd4100 /wd4702>
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
