# cmake/baselines.cmake — turn baselines/<name>.yaml into build targets.
#
# Steps 1–4 are anatomy's composition_generate() (cmake/composition.cmake),
# over scout's registry (cmake/slot_registry.cmake).
#
# A baseline file has a `composition:` block (slot -> implementation, resolved
# at compile time) and a `parameters:` block (loaded at startup; racing-drone
# docs/adr/0001-baseline-composition-framework.md). For each baseline this
# module:
#
#   1. parses the `composition:` block (two-space-indented `slot: Impl` lines),
#   2. checks that all six slots are named exactly once with an implementation
#      registered for that slot (cmake/slot_registry.cmake),
#   3. hashes the canonical form "slot=Impl\n" (slots sorted) with SHA-256 —
#      the same canonical form branes::scout::config::composition_hash()
#      computes at startup,
#   4. generates <build>/generated/<name>/scout_baseline.hpp, which defines
#      branes::scout::baseline::{name, composition_hash, Composition<T>},
#   5. adds the interface target scout_baseline_<name>: the generated header's
#      directory, and the libraries its slots need;
#   6. adds the baseline's harnesses, sitl_<name>, replay_<name> and
#      sitl_batch_<name>, from the sources in apps/<harness>/ against that
#      header (docs/plan/bring-up/, S0d-2).
#
#   scout_add_baseline(<name>)
include_guard(GLOBAL)
include(${ANATOMY_CMAKE_DIR}/composition.cmake)

function(scout_add_baseline name)
    cmake_parse_arguments(PARSE_ARGV 1 arg "" "" "")
    if(arg_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "scout_add_baseline(${name}): unexpected arguments '${arg_UNPARSED_ARGUMENTS}'")
    endif()
    set(file ${PROJECT_SOURCE_DIR}/baselines/${name}.yaml)
    if(NOT EXISTS ${file})
        message(FATAL_ERROR "scout_add_baseline(${name}): ${file} not found")
    endif()

    # ── 1–4. the composition: parsed, checked, hashed, and its header ───
    set(gen_dir ${CMAKE_BINARY_DIR}/generated/${name})
    composition_generate(scout ${name}
        FILE ${file}
        TEMPLATE ${PROJECT_SOURCE_DIR}/cmake/scout_baseline.hpp.in
        OUTPUT ${gen_dir}/scout_baseline.hpp)

    # ── 5. the baseline's target ────────────────────────────────────────
    set(target scout_baseline_${name})
    add_library(${target} INTERFACE)
    target_include_directories(${target} INTERFACE ${gen_dir})
    target_link_libraries(${target} INTERFACE branes::scout branes::anatomy_glue)
    target_compile_definitions(${target} INTERFACE SCOUT_BASELINES_DIR="${PROJECT_SOURCE_DIR}/baselines")

    # ── 6. the harnesses ────────────────────────────────────────────────
    foreach(harness sitl replay sitl_batch)
        set(app ${harness}_${name})
        add_executable(${app} ${PROJECT_SOURCE_DIR}/apps/${harness}/main.cpp)
        target_link_libraries(${app} PRIVATE ${target} branes::scout_sim)
        scout_set_warnings(${app})
        set_target_properties(${app} PROPERTIES
            FOLDER "Baselines/${name}"
            VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")
    endforeach()
    target_link_libraries(sitl_batch_${name} PRIVATE Threads::Threads)
endfunction()
