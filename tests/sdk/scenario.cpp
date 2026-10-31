// Scout's scenario schema (config/config.hpp): the field key, scout's `sim:`
// parameters, their defaults, the round trip, and the field a scenario flies.

#include <branes/scout/config/config.hpp>

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <string>

using namespace branes::scout;
using namespace branes::scout::config;

namespace {

const std::filesystem::path fixtures = std::filesystem::path(SCOUT_SOURCE_DIR) / "tests" / "fixtures" / "scenarios";

ScenarioConfig load(const char* name) {
    auto s = load_scenario(fixtures / name);
    INFO((s ? std::string() : s.error().to_string()));
    REQUIRE(s);
    return *s;
}

}  // namespace

TEST_CASE("scenario: the field file and scout's sim parameters", "[scenario][config]") {
    const ScenarioConfig s = load("forest.yaml");
    REQUIRE(s.world_ref == "../../../fields/forest-60.yaml");
    REQUIRE(std::filesystem::exists(s.world_path));
    REQUIRE(follows_scout_schema(s));
    REQUIRE(oracle_range_m(s) == 12.0);
    REQUIRE(oracle_sigma_m(s) == 0.05);
}

TEST_CASE("scenario: scout's sim parameters default to the schema's fallbacks", "[scenario][config]") {
    const ScenarioConfig s = load("defaults.yaml");
    REQUIRE(oracle_range_m(s) == kScoutWorldSim[0].fallback);
    REQUIRE(oracle_sigma_m(s) == kScoutWorldSim[1].fallback);
}

TEST_CASE("scenario: emitting and loading again gives the same scenario", "[scenario][config]") {
    const ScenarioConfig s = load("forest.yaml");
    auto again = parse_scenario(emit_scenario(s), "emitted", fixtures);
    REQUIRE(again);
    REQUIRE(*again == s);
}

TEST_CASE("scenario: a negative oracle range is an error", "[scenario][config]") {
    auto s = parse_scenario("name: t\n"
                            "field: ../../../fields/open-60.yaml\n"
                            "vehicle: ../../../vehicles/rp1.yaml\n"
                            "duration_s: 10\n"
                            "seed: 1\n"
                            "sim: {oracle_range_m: -1}\n",
                            "t",
                            fixtures);
    REQUIRE_FALSE(s);
    REQUIRE(s.error().key == "sim.oracle_range_m");
}

TEST_CASE("scenario: the field it flies is its field file grown with its seed", "[scenario][config]") {
    const ScenarioConfig s = load("forest.yaml");
    auto f = load_scenario_field(s);
    REQUIRE(f);
    auto c = load_field(s.world_path);
    REQUIRE(c);
    auto expected = field_for_seed(*c, s.seed);
    REQUIRE(expected);
    REQUIRE(*f == *expected);
    REQUIRE(f->obstacle_count > 0);
}
