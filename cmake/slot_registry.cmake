# cmake/slot_registry.cmake — scout's slots, and the implementations a scout baseline can name.
#
# The registry mechanism is anatomy's (cmake/composition.cmake). A baseline's
# `composition:` block maps each slot to an implementation name.
# scout_register_impl() records, for that name, the slot it fills, the header
# that declares it, and its C++ class template (templated on the scalar type).
# Platform components (cortex, reflex, synapse) register here as they land.
include_guard(GLOBAL)
include(${ANATOMY_CMAKE_DIR}/composition.cmake)

# The six slots (sdk/include/branes/scout/slots/slots.hpp), in the order the
# generated Stack<> takes them.
set(SCOUT_SLOTS estimator planner controller safety_filter daa_tracker fc_bridge)
composition_define_slots(scout ${SCOUT_SLOTS})

function(scout_register_impl impl)
    composition_register_impl(scout ${impl} ${ARGN})
endfunction()

# Null implementations: stateless stubs for the `null` baseline. The task
# slots' nulls are scout's; the mission-neutral slots' are anatomy's.
set(_null branes/scout/slots/null.hpp)
scout_register_impl(NullEstimator    SLOT estimator     HEADER ${_null} TYPE branes::scout::NullEstimator)
scout_register_impl(NullPlanner      SLOT planner       HEADER ${_null} TYPE branes::scout::NullPlanner)
scout_register_impl(NullController   SLOT controller    HEADER ${_null} TYPE branes::anatomy::NullController)
scout_register_impl(NullSafetyFilter SLOT safety_filter HEADER ${_null} TYPE branes::anatomy::NullSafetyFilter)
scout_register_impl(NullDaaTracker   SLOT daa_tracker   HEADER ${_null} TYPE branes::anatomy::NullDaaTracker)
scout_register_impl(NullFcBridge     SLOT fc_bridge     HEADER ${_null} TYPE branes::anatomy::NullFcBridge)

# Baseline s0-gt-pid (docs/plan/bring-up/): the SITL ground-truth estimator,
# the straight-line goal planner, and anatomy's reflex cascade adapter.
scout_register_impl(GroundTruthEstimator SLOT estimator HEADER branes/scout/slots/ground_truth.hpp
                    TYPE branes::scout::GroundTruthEstimator)
scout_register_impl(GoalPlanner SLOT planner HEADER branes/scout/slots/goal.hpp
                    TYPE branes::scout::GoalPlanner)
scout_register_impl(CascadedPid SLOT controller HEADER branes/anatomy/glue/cascaded_pid.hpp
                    TYPE branes::anatomy::glue::CascadedPid)
