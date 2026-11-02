# sim/ — scout's SITL harness library (docs/plan/bring-up/, S0d).
#
# `branes::scout_sim` is header-only: scout's simulator (the obstacle oracle on
# anatomy's BasicSimulator), the scorer, the scored run of the batch runner,
# and the replay of a log as a reflex-trace/1 trace (with reflex's writer,
# branes::reflex_trace). It runs in double, like the logs. The closed-loop
# simulator itself is branes::anatomy_mobility_sim.

add_library(branes_scout_sim INTERFACE)
add_library(branes::scout_sim ALIAS branes_scout_sim)

target_include_directories(branes_scout_sim INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(branes_scout_sim INTERFACE cxx_std_20)
target_link_libraries(branes_scout_sim INTERFACE branes::scout branes::anatomy_mobility_sim branes::reflex_trace)
