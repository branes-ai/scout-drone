# cmake/deps.cmake — scout-drone's dependencies, fetched at configure time.
#
# anatomy is the mission framework (racing-drone ADR 0008), pinned to a
# release commit. It fetches what it builds on itself: yaml-cpp, reflex (the
# plant and the controller adapters) and Universal. A mission takes reflex at
# the commit anatomy pins, so the two never disagree.
include(FetchContent)

# Every pin is a tag or a commit, so a configure fetches only when a pin moves:
# the update step finds the commit checked out and stops. With updates
# disconnected, a moved pin fails to configure in an existing build tree whose
# clone predates the new commit. Earlier versions of this file cached
# FETCHCONTENT_UPDATES_DISCONNECTED ON, and anatomy and the reflex it fetches
# cache it ON (reflex with FORCE), so this normal variable, which wins over the
# cache entry here and in their scopes, keeps it OFF.
set(FETCHCONTENT_UPDATES_DISCONNECTED OFF)

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
    GIT_TAG        2c6c872429411039b5b70b8f6f9020181043d2f0  # v0.9.0
    SYSTEM
)
FetchContent_MakeAvailable(anatomy)
