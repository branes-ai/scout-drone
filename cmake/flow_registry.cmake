# cmake/flow_registry.cmake — the operators a scout graph baseline can name
# (anatomy docs/adr/0004, step 2; anatomy docs/plan/flow-graph, 2c).
#
# The registry mechanism is anatomy's (cmake/flow.cmake). A baseline's
# `graph:` block maps each composite to an operator. scout_register_operator()
# records, for that name, the header that declares it, its C++ class template
# and its binding's (templated on the scalar type): scout's for the estimator
# and the planner (flow/bindings.hpp), anatomy's for the mission-neutral
# composites (mobility/flow/bindings.hpp). Every slot implementation is an
# operator of its composite, unchanged, so the names are the slot registry's
# (cmake/slot_registry.cmake).
include_guard(GLOBAL)
include(${ANATOMY_CMAKE_DIR}/flow.cmake)

function(scout_register_operator name)
    flow_register_operator(scout ${name} ${ARGN})
endfunction()

set(_null branes/scout/slots/null.hpp)
set(_scout branes::scout::graph)
set(_neutral branes::anatomy::flow)
scout_register_operator(NullEstimator    HEADER ${_null} TYPE branes::scout::NullEstimator
                        BINDING ${_scout}::EstimatorBinding)
scout_register_operator(NullPlanner      HEADER ${_null} TYPE branes::scout::NullPlanner
                        BINDING ${_scout}::PlannerBinding)
scout_register_operator(NullController   HEADER ${_null} TYPE branes::anatomy::NullController
                        BINDING ${_neutral}::ControllerBinding)
scout_register_operator(NullSafetyFilter HEADER ${_null} TYPE branes::anatomy::NullSafetyFilter
                        BINDING ${_neutral}::SafetyFilterBinding)
scout_register_operator(NullDaaTracker   HEADER ${_null} TYPE branes::anatomy::NullDaaTracker
                        BINDING ${_neutral}::DaaTrackerBinding)
scout_register_operator(NullFcBridge     HEADER ${_null} TYPE branes::anatomy::NullFcBridge
                        BINDING ${_neutral}::FcBridgeBinding)

scout_register_operator(GroundTruthEstimator HEADER branes/scout/slots/ground_truth.hpp
                        TYPE branes::scout::GroundTruthEstimator BINDING ${_scout}::EstimatorBinding)
scout_register_operator(GoalPlanner HEADER branes/scout/slots/goal.hpp
                        TYPE branes::scout::GoalPlanner BINDING ${_scout}::PlannerBinding)
scout_register_operator(CascadedPid HEADER branes/anatomy/glue/cascaded_pid.hpp
                        TYPE branes::anatomy::glue::CascadedPid BINDING ${_neutral}::ControllerBinding)
