include(FetchContent)

function(soxr_AddPatched SOXR_SOURCE_DIR SOXR_BINARY_DIR)
    if (CMAKE_VERSION VERSION_GREATER_EQUAL 4.0)
        # store original CMAKE_POLICY_VERSION_MINIMUM
        if (DEFINED CMAKE_POLICY_VERSION_MINIMUM)
            set(DEFINED_CMAKE_POLICY_VERSION_MINIMUM ON)
        else()
            set(DEFINED_CMAKE_POLICY_VERSION_MINIMUM OFF)
        endif()

        set(ORIG_CMAKE_POLICY_VERSION_MINIMUM "${CMAKE_POLICY_VERSION_MINIMUM}")

        # set min version to 3.5 
        set(CMAKE_POLICY_VERSION_MINIMUM "3.5" CACHE INTERNAL "")
    endif()

    # inject patches
    set(CMAKE_PROJECT_soxr_INCLUDE "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/soxr_patches.cmake")

    # add soxr source directory
    add_subdirectory("${SOXR_SOURCE_DIR}" "${SOXR_BINARY_DIR}")

    if (CMAKE_VERSION VERSION_GREATER_EQUAL 4.0)
        # restore original CMAKE_POLICY_VERSION_MINIMUM
        if (${DEFINED_CMAKE_POLICY_VERSION_MINIMUM})
            set(CMAKE_POLICY_VERSION_MINIMUM "${ORIG_CMAKE_POLICY_VERSION_MINIMUM}")
        else()
            unset(ORIG_CMAKE_POLICY_VERSION_MINIMUM)
        endif()
    endif()

    target_include_directories(soxr
        INTERFACE
        "${SOXR_SOURCE_DIR}/src"
    )
endfunction()
