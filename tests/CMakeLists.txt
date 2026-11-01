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

# ── The apps (docs/plan/bring-up/, S0d-2) ──────────────────────────────────
# Determinism (acceptance 3): two SITL runs of s0-gt-pid on a forest, with
# every random stream in use, are byte-identical, and two replays verify
# every output and are byte-identical to the SITL log.
add_test(NAME gate.s0.determinism
    COMMAND ${CMAKE_COMMAND}
        -DSITL=$<TARGET_FILE:sitl_s0-gt-pid>
        -DREPLAY=$<TARGET_FILE:replay_s0-gt-pid>
        -DSCORE=$<TARGET_FILE:score_log>
        -DSCENARIO=${PROJECT_SOURCE_DIR}/scenarios/s0-determinism.yaml
        -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/gate_s0
        -P ${CMAKE_CURRENT_SOURCE_DIR}/gate/sitl_gate.cmake)

# The null baseline (acceptance 4): it runs to the duration, scored as timed out.
add_test(NAME gate.null.smoke
    COMMAND ${CMAKE_COMMAND}
        -DSITL=$<TARGET_FILE:sitl_null>
        -DSCORE=$<TARGET_FILE:score_log>
        -DSCENARIO=${PROJECT_SOURCE_DIR}/scenarios/null-smoke.yaml
        -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/gate_null
        -P ${CMAKE_CURRENT_SOURCE_DIR}/gate/null_gate.cmake)

# log_dump writes every record, scout's oracle_obstacles channel among them.
add_test(NAME tools.log_dump
    COMMAND ${CMAKE_COMMAND}
        -DSITL=$<TARGET_FILE:sitl_s0-gt-pid>
        -DDUMP=$<TARGET_FILE:log_dump>
        -DSCENARIO=${PROJECT_SOURCE_DIR}/scenarios/s0-determinism.yaml
        -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/tools_log_dump
        -P ${CMAKE_CURRENT_SOURCE_DIR}/gate/log_dump_test.cmake)

# S0 flies (acceptance 5): on the open field, s0-gt-pid reaches the goal in
# at least 99 of 100 nominal seeds.
add_test(NAME gate.s0.open
    COMMAND sitl_batch_s0-gt-pid
        --scenario ${PROJECT_SOURCE_DIR}/scenarios/s0-open.yaml --seeds 100 --min-success 0.99)
set_tests_properties(gate.s0.open PROPERTIES LABELS "gate" TIMEOUT 600)

# The forest at another cruise speed, through --set: the batch runs and
# reports. No threshold: s0-gt-pid avoids nothing.
add_test(NAME gate.s0.forest.smoke
    COMMAND sitl_batch_s0-gt-pid
        --scenario ${PROJECT_SOURCE_DIR}/scenarios/s0-forest.yaml --seeds 10 --min-success 0
        --set cruise_speed_m_s=2)
set_tests_properties(gate.s0.forest.smoke PROPERTIES
    LABELS "gate" PASS_REGULAR_EXPRESSION "overrides +cruise_speed_m_s=2\n")

# --set refuses a key the baseline lacks, and a value that isn't a number.
add_test(NAME tools.batch_set.unknown_key
    COMMAND sitl_batch_s0-gt-pid
        --scenario ${PROJECT_SOURCE_DIR}/scenarios/s0-open.yaml --seeds 1 --set cruise_sped_m_s=2)
set_tests_properties(tools.batch_set.unknown_key PROPERTIES
    PASS_REGULAR_EXPRESSION "the baseline has no parameter cruise_sped_m_s")
add_test(NAME tools.batch_set.not_a_number
    COMMAND sitl_batch_s0-gt-pid
        --scenario ${PROJECT_SOURCE_DIR}/scenarios/s0-open.yaml --seeds 1 --set cruise_speed_m_s=fast)
set_tests_properties(tools.batch_set.not_a_number PROPERTIES
    PASS_REGULAR_EXPRESSION "'fast' is not a finite number")

# trace_export writes a log as a reflex-trace/1 replay with the field as its
# scene (S0e, docs/plan/bring-up/).
add_test(NAME tools.trace_export
    COMMAND ${CMAKE_COMMAND}
        -DSITL=$<TARGET_FILE:sitl_s0-gt-pid>
        -DEXPORT=$<TARGET_FILE:trace_export>
        -DSOURCE_DIR=${PROJECT_SOURCE_DIR}
        -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/tools_trace_export
        -P ${CMAKE_CURRENT_SOURCE_DIR}/gate/trace_export_test.cmake)
