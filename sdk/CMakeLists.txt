# sdk/ — scout-drone's mission library (docs/plan/bring-up/).
#
# `branes::scout` carries scout's task: its world (the field: msg/field.hpp),
# the field file and the field a seed generates (config/field.hpp), and its
# scenario schema (config/config.hpp). It compiles the field loader, with
# anatomy's YAML helpers (branes::anatomy_yaml). Its slots, stack and channels
# follow in S0c, and its simulator and scorer in S0d.

add_library(branes_scout STATIC
    src/config/field.cpp
)
add_library(branes::scout ALIAS branes_scout)

target_include_directories(branes_scout PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(branes_scout PUBLIC cxx_std_20)
# yaml-cpp stays an implementation detail of the loaders.
target_link_libraries(branes_scout PUBLIC branes::anatomy branes::anatomy_mobility PRIVATE branes::anatomy_yaml)
scout_set_warnings(branes_scout)
set_target_properties(branes_scout PROPERTIES FOLDER "SDK")
