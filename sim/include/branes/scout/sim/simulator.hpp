#pragma once
/// @file simulator.hpp
/// @brief Simulator: scout's SITL event source, anatomy's BasicSimulator with the obstacle oracle.
///
/// The closed loop (the reflex plant, the simulated FC, the IMU, faults,
/// camera timing, truth) is anatomy's (sim/basic_simulator.hpp); scout adds
/// its world (the field: the start pose and the obstacles) and its oracle
/// sensor (ObstacleOracle, oracle.hpp), which reports the obstacles seen at
/// every camera frame on the oracle_obstacles channel. The oracle's range and
/// noise are the scenario's (`sim.oracle_range_m`, `sim.oracle_sigma_m`), and
/// its noise stream is seeded from the scenario's seed.
///
/// Events come out as BasicSimulator documents, with oracle_obstacles as the
/// oracle event:
///
///   sim_config (t = 0 only) → IMU deliveries → camera → oracle_obstacles → truth → tick

#include <branes/anatomy/mobility/config/scenario.hpp>
#include <branes/anatomy/mobility/config/vehicle.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/anatomy/sim/basic_simulator.hpp>
#include <branes/scout/config/config.hpp>
#include <branes/scout/msg/field.hpp>
#include <branes/scout/sim/oracle.hpp>

#include <cmath>
#include <string>
#include <utility>

namespace branes::scout::sim {

class Simulator : public anatomy::sim::BasicSimulator<ObstacleOracle> {
public:
    using Base = anatomy::sim::BasicSimulator<ObstacleOracle>;

    /// Validate the inputs and build the run over `field`, the field the
    /// scenario flies (config::load_scenario_field). Fails, besides
    /// BasicSimulator's checks, if the field isn't valid, or the scenario does
    /// not carry scout's sim parameters (config::follows_scout_schema) or one
    /// of them isn't finite and ≥ 0. The loader checks them; a scenario built
    /// in code is checked here, before the oracle and the scorer take them.
    [[nodiscard]] static anatomy::result<Simulator, std::string> create(const anatomy::config::VehicleConfig& vehicle,
                                                                        const Field<double>& field,
                                                                        const config::ScenarioConfig& scenario,
                                                                        double control_hz,
                                                                        const anatomy::sim::SimOptions& options = {}) {
        if (!valid(field)) {
            return std::string("the field is not valid (see Field's invariants, msg/field.hpp)");
        }
        if (!config::follows_scout_schema(scenario)) {
            return std::string("the scenario has no scout sim parameters (set world_sim, see config::kScoutScenario)");
        }
        const double range = config::oracle_range_m(scenario);
        const double sigma = config::oracle_sigma_m(scenario);
        const double radius = config::collision_radius_m(scenario);
        for (const double v : {range, sigma, sigma * std::sqrt(3.0), radius}) {
            if (!(std::isfinite(v) && v >= 0)) {
                return std::string("the scenario's sim parameters must be finite and ≥ 0");
            }
        }
        ObstacleOracle oracle(field, range, sigma, scenario.seed);
        auto base = Base::create(vehicle, field.start, scenario, control_hz, std::move(oracle), options);
        if (!base) {
            return base.error();
        }
        return Simulator(std::move(base).value());
    }

private:
    explicit Simulator(Base&& base) : Base(std::move(base)) {}
};

}  // namespace branes::scout::sim
