# cmake/deps.cmake — scout-drone's dependencies, fetched at configure time.
#
# anatomy is the mission framework (racing-drone ADR 0008), pinned to a
# release commit. It fetches what it builds on itself: yaml-cpp, reflex (the
# plant and the controller adapters) and Universal. A mission takes reflex at
# the commit anatomy pins, so the two never disagree.
include(FetchContent)

set(FETCHCONTENT_UPDATES_DISCONNECTED ON CACHE BOOL "Skip FetchContent update steps")

if(SCOUT_BUILD_TESTS AND BUILD_TESTING)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.15.0
        GIT_SHALLOW    TRUE
        SYSTEM
    )
    FetchContent_MakeAvailable(Catch2)
    list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
endif()

FetchContent_Declare(
    anatomy
    GIT_REPOSITORY https://github.com/branes-ai/anatomy.git
    GIT_TAG        6e943a60823676cf55ac95420e84c9856a108994  # v0.4.0
    SYSTEM
)
FetchContent_MakeAvailable(anatomy)
