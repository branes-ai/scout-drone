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
