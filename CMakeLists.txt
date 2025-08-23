cmake_minimum_required(VERSION 3.18)

project(
    VS-AudioResample
    DESCRIPTION "audio sample rate and sample type converter for VapourSynth"
    VERSION 0.4.0
    LANGUAGES C CXX
)

# original soxr repository on sourceforge
# set(SOXR_GIT "https://git.code.sf.net/p/soxr/code")

# mirror on github (faster)
set(SOXR_GIT "https://github.com/chirlu/soxr.git")

# use commit from February 24, 2018
# commit id is the same for the sourceforge and github repository
set(SOXR_COMMIT "945b592b70470e29f917f4de89b4281fbbd540c0")


# Include modules
include(FetchContent)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(Clang)
include(fetch)
include(soxr)

Clang_GetStyle_CXX(IS_CLANG_GCC IS_CLANG_MSVC)


# Custom options
set(SOXR_SOURCE_DIR "" CACHE PATH "Path to a custom soxr source directory")

option(SOXR_USE_PATCHES "Apply patches to soxr for compatibility" ON)

# OpenMP support

if (DEFINED WITH_OPENMP)
    set(WITH_OPENMP_SET_BY_USER ON)
else()
    set(WITH_OPENMP_SET_BY_USER OFF)
endif()


option(WITH_OPENMP "Enable OpenMP support" ON)


if (WITH_OPENMP)
    if (WITH_OPENMP_SET_BY_USER)
        find_package(OpenMP)
        if (NOT OpenMP_C_FOUND)
            message(FATAL_ERROR "OpenMP requested, but not found.")
        endif()
    else()
        find_package(OpenMP)
        if (NOT OpenMP_C_FOUND)
            message(WARNING "OpenMP enabled by default, but not found. Proceeding without it.")
            set(WITH_OPENMP OFF)
        endif()
    endif()
endif()


# Prepare soxr source directory

if (SOXR_SOURCE_DIR)
    if (NOT EXISTS "${SOXR_SOURCE_DIR}/CMakeLists.txt")
        message(FATAL_ERROR "Error: Specified soxr source directory does not contain CMakeLists.txt")
    endif()

    message(STATUS "Using custom soxr from ${SOXR_SOURCE_DIR}")
    set(SOXR_BINARY_DIR "${CMAKE_BINARY_DIR}/soxr-build")
else()
    fetch_FromGit("${SOXR_GIT}" "${SOXR_COMMIT}" SOXR_SOURCE_DIR SOXR_BINARY_DIR)
endif()


# Add soxr source directory to the project

# disable soxr tests and libsamplerate bindings
set(BUILD_TESTS OFF CACHE INTERNAL "Disable tests")
set(WITH_LSR_BINDINGS OFF CACHE INTERNAL "Disable LSR bindings")

if (${SOXR_USE_PATCHES})
    soxr_AddPatched("${SOXR_SOURCE_DIR}" "${SOXR_BINARY_DIR}")
else()
    add_subdirectory("${SOXR_SOURCE_DIR}" "${SOXR_BINARY_DIR}")
endif()


add_library(AudioResample SHARED
    "${CMAKE_SOURCE_DIR}/src/plugin.cpp"
    "${CMAKE_SOURCE_DIR}/src/resample.cpp"
    "${CMAKE_SOURCE_DIR}/src/resample.hpp"
    "${CMAKE_SOURCE_DIR}/src/common/overflow.cpp"
    "${CMAKE_SOURCE_DIR}/src/common/overflow.hpp"
    "${CMAKE_SOURCE_DIR}/src/common/resquality.cpp"
    "${CMAKE_SOURCE_DIR}/src/common/resquality.hpp"
    "${CMAKE_SOURCE_DIR}/src/common/sampletype.cpp"
    "${CMAKE_SOURCE_DIR}/src/common/sampletype.hpp"
    "${CMAKE_SOURCE_DIR}/src/utils/array.hpp"
    "${CMAKE_SOURCE_DIR}/src/utils/debug.hpp"
    "${CMAKE_SOURCE_DIR}/src/utils/map.hpp"
    "${CMAKE_SOURCE_DIR}/src/utils/number.hpp"
    "${CMAKE_SOURCE_DIR}/src/utils/sample.hpp"
    "${CMAKE_SOURCE_DIR}/src/utils/string.cpp"
    "${CMAKE_SOURCE_DIR}/src/utils/string.hpp"
    "${CMAKE_SOURCE_DIR}/src/vsmap/vsmap.cpp"
    "${CMAKE_SOURCE_DIR}/src/vsmap/vsmap.hpp"
    "${CMAKE_SOURCE_DIR}/src/vsmap/vsmap_common.cpp"
    "${CMAKE_SOURCE_DIR}/src/vsmap/vsmap_common.hpp"
    "${CMAKE_SOURCE_DIR}/src/vsutils/audio.cpp"
    "${CMAKE_SOURCE_DIR}/src/vsutils/audio.hpp"
    "${CMAKE_SOURCE_DIR}/src/vsutils/bitshift.hpp"
)

target_include_directories(AudioResample
    PRIVATE
    "${CMAKE_SOURCE_DIR}/include/vapoursynth"
    "${CMAKE_SOURCE_DIR}/src"
)

target_compile_features(AudioResample PRIVATE cxx_std_20)

target_link_libraries(AudioResample PRIVATE soxr)


# Linker options for OpenMP support
if (WITH_OPENMP)
    if ((CMAKE_SYSTEM_NAME STREQUAL "Windows") AND ((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") OR ${IS_CLANG_GCC}) AND NOT ${BUILD_SHARED_LIBS})
        # Windows + GCC/Clang(GCC) + static
        # use -fopenmp as linker argument to let the linker find a static library
        # because apparently "target_link_libraries(AudioResample PRIVATE OpenMP::OpenMP_CXX)" prefers a dynamically linked library
        target_link_options(AudioResample PRIVATE -static -fopenmp)
    else()
        target_link_libraries(AudioResample PRIVATE OpenMP::OpenMP_C)
    endif()
endif()


target_compile_options(AudioResample
    PRIVATE
    # Debug + CXX + GCC/Clang(GCC)
    $<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>,$<OR:$<CXX_COMPILER_ID:GNU>,$<BOOL:${IS_CLANG_GCC}>>>:
        -Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-unused-variable>

    # Release + CXX + GCC/Clang(GCC)
    $<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:CXX>,$<OR:$<CXX_COMPILER_ID:GNU>,$<BOOL:${IS_CLANG_GCC}>>>:
        -O2 -Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-unused-variable>

    # Debug + CXX + MSVC/Clang(MSVC)
    $<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>,$<OR:$<CXX_COMPILER_ID:MSVC>,$<BOOL:${IS_CLANG_MSVC}>>>:
        /W4 /wd4100 /wd4702 /Zc:__cplusplus>

    # Release + CXX + MSVC/Clang(MSVC)
    $<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:CXX>,$<OR:$<CXX_COMPILER_ID:MSVC>,$<BOOL:${IS_CLANG_MSVC}>>>:
        /W4 /wd4100 /wd4702 /Zc:__cplusplus>
)


target_link_options(AudioResample
    PRIVATE
    # Windows + CXX + GCC/Clang(GCC)
    $<$<AND:$<PLATFORM_ID:Windows>,$<LINK_LANGUAGE:CXX>,$<OR:$<CXX_COMPILER_ID:GNU>,$<BOOL:${IS_CLANG_GCC}>>>:
        -static -static-libstdc++ -s>

    # Non-Windows + CXX + GCC/Clang(GCC)
    $<$<AND:$<NOT:$<PLATFORM_ID:Windows>>,$<LINK_LANGUAGE:CXX>,$<OR:$<CXX_COMPILER_ID:GNU>,$<BOOL:${IS_CLANG_GCC}>>>:
        -static -s>
)


set_target_properties(AudioResample PROPERTIES PREFIX "")
