// One baseline flown in the closed loop (sim/batch.hpp, run_one), built once
// per baseline against its own scout_baseline.hpp: on the open field the null
// baseline stays on the pad and times out, and s0-gt-pid reaches the goal; on
// the forest, a run is scored; and the same seed gives the same score.

#include <branes/anatomy/mobility/config/vehicle.hpp>
#include <branes/anatomy/sim/closed_loop.hpp>
#include <branes/scout/config/config.hpp>
#include <branes/scout/harness/harness.hpp>
#include <branes/scout/sim/batch.hpp>
#include <branes/scout/sim/score.hpp>

#include "scout_baseline.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <limits>
#include <string>

using namespace branes::scout;
namespace an = branes::anatomy;

namespace {

const std::filesystem::path src = SCOUT_SOURCE_DIR;
const std::filesystem::path baselines = SCOUT_BASELINES_DIR;

config::ScenarioConfig scenario(const std::string& field, double duration_s) {
    auto s = config::parse_scenario("name: closed-loop-test\n"
                                    "field: fields/" +
                                        field +
                                        ".yaml\n"
                                        "vehicle: vehicles/rp1.yaml\n"
                                        "duration_s: " +
                                        std::to_string(duration_s) + "\nseed: 7\n",
                                    "closed-loop-test",
                                    src);
    REQUIRE(s);
    return *s;
}

sim::Run fly(const config::ScenarioConfig& s, std::uint64_t seed) {
    const auto b = harness::check_baseline(baselines, baseline::name, baseline::composition_hash);
    REQUIRE(b);
    const auto vehicle = an::config::load_vehicle(s.vehicle_path);
    REQUIRE(vehicle);
    const auto field = config::load_field(s.world_path);
    REQUIRE(field);
    const double control_hz =
        an::config::numeric_parameter(*b, "control_rate_hz").value_or(an::config::kDefaultControlRateHz);
    return sim::run_one<baseline::Composition<double>>(*b, *vehicle, *field, s, control_hz, seed);
}

}  // namespace

TEST_CASE("closed loop: the baseline on the open field", "[closed_loop]") {
    const sim::Run r = fly(scenario("open-60", 40.0), 7);
    INFO(r.error);
    REQUIRE(r.error.empty());
    INFO(sim::to_json(r.score));
    if (baseline::name == "null") {
        // The null slots command nothing: the vehicle stays on the pad until the end.
        CHECK(r.score.outcome == sim::Outcome::timed_out);
        CHECK(r.score.path_length_m == 0);
    } else {
        // 50 m along the field, after the climb, at the cruise speed: in well under 40 s.
        CHECK(r.score.outcome == sim::Outcome::reached);
        CHECK(r.score.success);
        CHECK(r.score.time_to_goal_s > 0);
        CHECK(r.score.time_to_goal_s < 40);
        CHECK(r.score.path_length_m >= 49.0);
        CHECK(r.score.min_clearance_m == std::numeric_limits<double>::infinity());  // no obstacles
    }
}

TEST_CASE("closed loop: the same seed gives the same score, on the forest", "[closed_loop]") {
    const auto s = scenario("forest-60", 30.0);
    const sim::Run a = fly(s, 1700);
    const sim::Run b = fly(s, 1700);
    INFO(a.error);
    REQUIRE(a.error.empty());
    CHECK(an::sim::to_json(a) == an::sim::to_json(b));
    if (baseline::name != "null") {
        CHECK(std::isfinite(a.score.min_clearance_m));  // it flew among the trees
    }
}
