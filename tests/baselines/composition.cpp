// One baseline's generated composition (cmake/baselines.cmake), built once
// per baseline against its own scout_baseline.hpp: the hash compiled in
// equals the hash config::composition_hash() computes from the baseline file
// at startup (otherwise every harness would refuse to start), and the
// composition builds from the baseline's parameters and runs a tick.

#include <branes/anatomy/harness/runner.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/mobility/config/vehicle.hpp>
#include <branes/anatomy/mobility/msg/codec.hpp>
#include <branes/scout/config/config.hpp>
#include <branes/scout/harness/harness.hpp>

#include "scout_baseline.hpp"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <span>
#include <string>

using namespace branes::scout;
namespace an = branes::anatomy;

namespace {

const std::filesystem::path src = SCOUT_SOURCE_DIR;
const std::filesystem::path baselines = SCOUT_BASELINES_DIR;

}  // namespace

TEST_CASE("baseline: the generated hash matches the startup hash", "[baseline]") {
    const auto b = config::load_baseline(baselines / (std::string(baseline::name) + ".yaml"));
    REQUIRE(b);
    CHECK(b->name == baseline::name);
    CHECK(config::composition_hash(*b) == baseline::composition_hash);
}

TEST_CASE("baseline: check_baseline accepts the repo file and rejects a stale hash", "[baseline]") {
    CHECK(harness::check_baseline(baselines, baseline::name, baseline::composition_hash));
    CHECK_FALSE(harness::check_baseline(baselines, baseline::name, "0000"));
}

TEST_CASE("baseline: the composition builds from its parameters and runs a tick", "[baseline]") {
    const auto b = harness::check_baseline(baselines, baseline::name, baseline::composition_hash);
    REQUIRE(b);
    const auto vehicle = an::config::load_vehicle(src / "vehicles/rp1.yaml");
    REQUIRE(vehicle);
    const auto c = config::load_field(src / "fields/open-60.yaml");
    REQUIRE(c);
    const auto f = config::field_for_seed(*c, 1);
    REQUIRE(f);
    auto stack = harness::make_stack<baseline::Composition<double>>(*f, *vehicle, *b);
    INFO((stack ? std::string() : stack.error()));
    REQUIRE(stack);

    an::harness::Runner runner(*stack, std::span<const an::log::Channel>{});
    an::TruthState<double> x{};
    x.t = an::Timestamp{4'000'000};
    x.position = f->start.position;
    x.on_ground = true;
    REQUIRE(runner.feed(an::log::LogRecord{an::log::Channel::truth, x.t, an::codec::encode(x)}));
    const auto out = runner.feed(an::log::LogRecord{an::log::Channel::tick, x.t, {}});
    REQUIRE(out);
    REQUIRE(out->size() == 3u);
    const auto u = an::codec::decode_command((*out)[2].payload);
    REQUIRE(u);
    CHECK(valid(*u));
}
