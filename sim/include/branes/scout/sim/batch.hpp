#pragma once
/// @file batch.hpp
/// @brief run_one: one scored scout SITL run without a log, for the batch runner.
///
/// The closed loop, the run's JSON line and the statistics are anatomy's
/// (sim/closed_loop.hpp); scout supplies its simulator, its stack, its scorer
/// (with the scenario's collision radius and speed limit) and its channels.
/// sitl_batch_<baseline> and the tests share it, so a run means the same in both.

#include <branes/anatomy/sim/closed_loop.hpp>
#include <branes/scout/config/config.hpp>
#include <branes/scout/harness/harness.hpp>
#include <branes/scout/msg/field.hpp>
#include <branes/scout/sim/score.hpp>
#include <branes/scout/sim/simulator.hpp>

#include <cstdint>
#include <span>

namespace branes::scout::sim {

/// One seed's scout run.
using Run = anatomy::sim::BasicRun<Score>;

/// One closed-loop run of StackT on the field `config` grows with `seed`,
/// scored. The scenario's own seed is replaced by `seed`, so the field, the
/// simulator's streams and the oracle's noise all follow it.
template <class StackT>
[[nodiscard]] Run run_one(const config::BaselineConfig& baseline,
                          const anatomy::config::VehicleConfig& vehicle,
                          const config::FieldConfig& field_config,
                          config::ScenarioConfig scenario,
                          double control_hz,
                          std::uint64_t seed) {
    Run out;
    out.seed = seed;
    scenario.seed = seed;
    auto field = config::field_for_seed(field_config, seed);
    if (!field) {
        out.error = field.error();
        return out;
    }
    auto sim = Simulator::create(vehicle, *field, scenario, control_hz);
    if (!sim) {
        out.error = sim.error();
        return out;
    }
    auto stack = harness::make_stack<StackT>(*field, vehicle, baseline);
    if (!stack) {
        out.error = stack.error();
        return out;
    }
    Scorer scorer(*field, config::collision_radius_m(scenario), scenario.speed_limit_m_s);
    // oracle_obstacles is the stack's input (ScoutProfile), not a pass-through channel.
    return anatomy::sim::run_closed_loop<Score>(*sim, *stack, std::span<const anatomy::log::Channel>{}, scorer, seed);
}

}  // namespace branes::scout::sim
