# apps/ — the harness executables (docs/plan/bring-up/, S0d-2).
#
# Each baseline in baselines/ gets sitl_<name>, replay_<name> and
# sitl_batch_<name>, built from the sources in apps/<harness>/ against that
# baseline's generated composition header (cmake/baselines.cmake). The tools
# here read any baseline's log.

# score_log: the score of any baseline's SITL log (sim/score.hpp).
add_executable(score_log ${CMAKE_CURRENT_SOURCE_DIR}/score/main.cpp)
target_link_libraries(score_log PRIVATE branes::scout_sim)
scout_set_warnings(score_log)
set_target_properties(score_log PROPERTIES FOLDER "Tools" VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

# log_dump: a log's channels as CSV, for plotting (any baseline's log).
add_executable(log_dump ${CMAKE_CURRENT_SOURCE_DIR}/log_dump/main.cpp)
target_link_libraries(log_dump PRIVATE branes::scout)
scout_set_warnings(log_dump)
set_target_properties(log_dump PROPERTIES FOLDER "Tools" VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

# trace_export: a SITL log as a reflex-trace/1 replay with the field as its
# scene, for the docs site's trace player (sim/replay_trace.hpp, tools/gen_replay.sh).
add_executable(trace_export ${CMAKE_CURRENT_SOURCE_DIR}/trace_export/main.cpp)
target_link_libraries(trace_export PRIVATE branes::scout_sim)
scout_set_warnings(trace_export)
set_target_properties(trace_export PROPERTIES FOLDER "Tools" VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")
