# sdk/ — scout-drone's mission library (docs/plan/bring-up/).
#
# `branes::scout` will hold the mission's task: its world (the field), the
# slots that see it, its stack, its configs and its channels. It builds on
# anatomy's mobility profile, its simulator and its reflex glue. For now it
# only names itself.

add_library(branes_scout INTERFACE)
add_library(branes::scout ALIAS branes_scout)

target_include_directories(branes_scout INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(branes_scout INTERFACE cxx_std_20)
target_link_libraries(branes_scout INTERFACE branes::anatomy_mobility_sim branes::anatomy_glue)
