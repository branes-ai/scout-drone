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

# ── Baselines ──────────────────────────────────────────────────────────────
# One test executable per baseline, from the same source, against the
# baseline's generated composition header (cmake/baselines.cmake).
foreach(baseline null s0-gt-pid)
    string(REPLACE "-" "_" id ${baseline})
    set(name baseline_${id})
    add_executable(${name} ${CMAKE_CURRENT_SOURCE_DIR}/baselines/composition.cpp)
    target_link_libraries(${name} PRIVATE Catch2::Catch2WithMain scout_baseline_${baseline})
    target_compile_features(${name} PRIVATE cxx_std_20)
    target_compile_definitions(${name} PRIVATE SCOUT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
    scout_set_warnings(${name})
    set_target_properties(${name} PROPERTIES FOLDER "Tests/baselines")
    catch_discover_tests(${name} TEST_PREFIX "${baseline}.")
endforeach()
