# Regression tests. Each .cpp under a layer subdirectory becomes a Catch2 test
# executable automatically — see cmake/compile_all.cmake. To add a test, drop a
# `<name>.cpp` into the matching tests/<layer>/ subdirectory; no edit here is
# needed. File stems must be globally unique (they name the targets).

include(${PROJECT_SOURCE_DIR}/cmake/compile_all.cmake)

# ── SDK layer ──────────────────────────────────────────────────────────────
file(GLOB SDK_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/sdk/*.cpp)
compile_all(FOLDER "Tests/sdk" LIBS branes::scout SOURCES ${SDK_SRC})
foreach(source ${SDK_SRC})
    get_filename_component(name ${source} NAME_WE)
    target_compile_definitions(${name} PRIVATE SCOUT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
endforeach()

# ── SITL layer ─────────────────────────────────────────────────────────────
file(GLOB SIM_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/sim/*.cpp)
compile_all(FOLDER "Tests/sim" LIBS branes::scout_sim SOURCES ${SIM_SRC})
foreach(source ${SIM_SRC})
    get_filename_component(name ${source} NAME_WE)
    target_compile_definitions(${name} PRIVATE SCOUT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
endforeach()

# ── Baselines ──────────────────────────────────────────────────────────────
# Two test executables per baseline, each from one source, against the
# baseline's generated composition header (cmake/baselines.cmake): the
# composition itself, and the baseline flown in the closed loop.
foreach(baseline null s0-gt-pid)
    string(REPLACE "-" "_" id ${baseline})
    foreach(test composition closed_loop)
        set(name baseline_${id}_${test})
        if(test STREQUAL "composition")
            set(name baseline_${id})  # the name it had before the closed-loop test
        endif()
        add_executable(${name} ${CMAKE_CURRENT_SOURCE_DIR}/baselines/${test}.cpp)
        target_link_libraries(${name} PRIVATE Catch2::Catch2WithMain scout_baseline_${baseline} branes::scout_sim)
        target_compile_features(${name} PRIVATE cxx_std_20)
        target_compile_definitions(${name} PRIVATE SCOUT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
        scout_set_warnings(${name})
        set_target_properties(${name} PROPERTIES FOLDER "Tests/baselines")
        catch_discover_tests(${name} TEST_PREFIX "${baseline}.")
    endforeach()
endforeach()
