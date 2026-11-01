// Scout's simulator (sim/simulator.hpp): anatomy's closed loop with the
// obstacle oracle. After each camera frame comes the oracle's event at the
// frame's time, the obstacles seen from the truth then; the same scenario
// gives the same events; and it refuses a scenario without scout's sim
// parameters or with one that isn't finite and ≥ 0, or a field that isn't valid.

#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/mobility/config/vehicle.hpp>
#include <branes/anatomy/mobility/msg/codec.hpp>
#include <branes/scout/config/config.hpp>
#include <branes/scout/log/channels.hpp>
#include <branes/scout/msg/obstacle_codec.hpp>
#include <branes/scout/sim/oracle.hpp>
#include <branes/scout/sim/simulator.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>
#include <vector>

using namespace branes::scout;
namespace an = branes::anatomy;

namespace {

const std::filesystem::path src = SCOUT_SOURCE_DIR;

config::ScenarioConfig scenario(double duration_s, double sigma_m) {
    auto s = config::parse_scenario("name: sim-test\n"
                                    "field: fields/forest-60.yaml\n"
                                    "vehicle: vehicles/rp1.yaml\n"
                                    "duration_s: " +
                                        std::to_string(duration_s) +
                                        "\n"
                                        "seed: 1700\n"
                                        "sim:\n"
                                        "  oracle_range_m: 12.0\n"
                                        "  oracle_sigma_m: " +
                                        std::to_string(sigma_m) + "\n",
                                    "sim-test",
                                    src);
    REQUIRE(s);
    return *s;
}

an::config::VehicleConfig vehicle() {
    auto v = an::config::load_vehicle(src / "vehicles/rp1.yaml");
    REQUIRE(v);
    return *v;
}

Field<double> forest(const config::ScenarioConfig& s) {
    auto f = config::load_scenario_field(s);
    REQUIRE(f);
    REQUIRE(f->obstacle_count > 0u);
    return *f;
}

}  // namespace

TEST_CASE("simulator: every camera frame is followed by the obstacles seen then", "[simulator]") {
    const auto s = scenario(0.5, 0.0);
    const Field<double> f = forest(s);
    auto sim = sim::Simulator::create(vehicle(), f, s, 100.0);
    INFO((sim ? std::string() : sim.error()));
    REQUIRE(sim);
    std::uint64_t frames = 0;
    std::uint64_t seen = 0;
    std::optional<an::Timestamp> frame;
    while (auto ev = sim->next()) {
        if (ev->channel == an::log::Channel::camera) {
            ++frames;
            frame = ev->t;
            // The truth the oracle sees at the frame: the plant's, before any later step.
            const auto expected = sim::see_obstacles(f, sim->truth(ev->t), ev->t, 12.0);
            auto next = sim->next();
            REQUIRE(next);
            REQUIRE(next->channel == log::oracle_obstacles);
            CHECK(next->t == ev->t);
            const auto o = codec::decode_obstacles(next->payload);
            REQUIRE(o);
            CHECK(*o == expected);
            seen += o->count;
        } else {
            CHECK(ev->channel != log::oracle_obstacles);  // only straight after a frame
        }
    }
    CHECK(frames > 0u);
    CHECK(seen > 0u);  // the forest is in range of the pad
}

TEST_CASE("simulator: the same scenario gives the same events", "[simulator]") {
    const auto s = scenario(0.3, 0.05);
    const Field<double> f = forest(s);
    const auto events = [&] {
        auto sim = sim::Simulator::create(vehicle(), f, s, 100.0);
        REQUIRE(sim);
        std::vector<an::sim::SimEvent> out;
        while (auto ev = sim->next()) {
            out.push_back(*ev);
        }
        return out;
    };
    const auto a = events();
    const auto b = events();
    REQUIRE(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        CHECK(a[i].channel == b[i].channel);
        CHECK(a[i].t == b[i].t);
        CHECK(a[i].payload == b[i].payload);
    }
}

TEST_CASE("simulator: it refuses a scenario without scout's sim parameters, a bad parameter, or a bad field",
          "[simulator]") {
    auto s = scenario(0.3, 0.0);
    const Field<double> f = forest(s);
    config::ScenarioConfig bare = s;
    bare.world_sim.clear();
    const auto no_schema = sim::Simulator::create(vehicle(), f, bare, 100.0);
    REQUIRE_FALSE(no_schema);
    CHECK(no_schema.error().find("scout sim parameters") != std::string::npos);

    for (const std::size_t i : {0u, 1u, 2u}) {  // the range, the noise, the collision radius
        for (const double v : {-1.0, std::nan(""), std::numeric_limits<double>::infinity()}) {
            config::ScenarioConfig odd = s;
            odd.world_sim[i] = v;
            const auto r = sim::Simulator::create(vehicle(), f, odd, 100.0);
            REQUIRE_FALSE(r);
            CHECK(r.error() == "the scenario's sim parameters must be finite and ≥ 0");
        }
    }

    Field<double> bad = f;
    bad.goal.radius = 0;
    const auto no_field = sim::Simulator::create(vehicle(), bad, s, 100.0);
    REQUIRE_FALSE(no_field);
    CHECK(no_field.error().find("field is not valid") != std::string::npos);
}
