include(FetchContent)

function(fetch_FromGit SOXR_GIT SOXR_COMMIT SOXR_SOURCE_DIR SOXR_BINARY_DIR)
    if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.30)
        FetchContent_Populate(
            soxr
            GIT_REPOSITORY ${SOXR_GIT}
            GIT_PROGRESS ON
            GIT_TAG ${SOXR_COMMIT}
        )
    else()
        FetchContent_Declare(
            soxr
            GIT_REPOSITORY ${SOXR_GIT}
            GIT_PROGRESS ON
            GIT_TAG ${SOXR_COMMIT}
        )

        FetchContent_Populate(soxr)
    endif()

    set(${SOXR_SOURCE_DIR} "${soxr_SOURCE_DIR}" PARENT_SCOPE)
    set(${SOXR_BINARY_DIR} "${soxr_BINARY_DIR}" PARENT_SCOPE)
endfunction()
