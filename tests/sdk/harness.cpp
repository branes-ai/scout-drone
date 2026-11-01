// Scout's harness helpers (harness/harness.hpp): the field a log records is
// the field flown, its obstacles listed, and reads back equal; a field that
// generates its obstacles is refused there; and --set overrides numeric
// parameters the baseline has, refusing any other.

#include <branes/scout/config/config.hpp>
#include <branes/scout/harness/harness.hpp>

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <string>

using namespace branes::scout;

namespace {

const std::filesystem::path src = SCOUT_SOURCE_DIR;

config::BaselineConfig baseline() {
    auto b = config::load_baseline(src / "baselines/s0-gt-pid.yaml");
    REQUIRE(b);
    return *b;
}

}  // namespace

TEST_CASE("harness: a log records the field flown, which reads back equal", "[harness]") {
    const auto c = config::load_field(src / "fields/forest-60.yaml");
    REQUIRE(c);
    REQUIRE(c->generate);
    const auto flown = config::field_for_seed(*c, 1700);
    REQUIRE(flown);
    REQUIRE(flown->obstacle_count > 0u);
    const std::string yaml = harness::flown_field_yaml(*c, *flown);
    const auto back = harness::field_from_yaml(yaml, "log");
    INFO((back ? std::string() : back.error()));
    REQUIRE(back);
    CHECK(*back == *flown);  // every obstacle listed, every double exact
}

TEST_CASE("harness: a field that generates its obstacles can't be read from a log", "[harness]") {
    const auto c = config::load_field(src / "fields/forest-60.yaml");
    REQUIRE(c);
    const auto r = harness::field_from_yaml(config::emit_field(*c), "log");
    REQUIRE_FALSE(r);
    CHECK(r.error().find("needs the run's seed") != std::string::npos);
}

TEST_CASE("harness: --set overrides the numeric parameters the baseline has", "[harness]") {
    const auto b = baseline();
    const auto one = harness::override_parameters(b, "cruise_speed_m_s=4");
    REQUIRE(one);
    CHECK(branes::anatomy::config::numeric_parameter(*one, "cruise_speed_m_s") == 4.0);
    CHECK(one->composition == b.composition);
    CHECK(config::composition_hash(*one) == config::composition_hash(b));  // the composition is unchanged

    const auto two = harness::override_parameters(b, "cruise_speed_m_s=2.5,max_speed_m_s=6");
    REQUIRE(two);
    CHECK(branes::anatomy::config::numeric_parameter(*two, "cruise_speed_m_s") == 2.5);
    CHECK(branes::anatomy::config::numeric_parameter(*two, "max_speed_m_s") == 6.0);
    CHECK(branes::anatomy::config::numeric_parameter(*two, "lookahead_m") ==
          branes::anatomy::config::numeric_parameter(b, "lookahead_m"));
}

TEST_CASE("harness: --set refuses an unknown key, a value that isn't a number, and a malformed list", "[harness]") {
    const auto b = baseline();
    const auto unknown = harness::override_parameters(b, "cruise_sped_m_s=4");
    REQUIRE_FALSE(unknown);
    CHECK(unknown.error() == "--set cruise_sped_m_s: the baseline has no parameter cruise_sped_m_s");
    const auto word = harness::override_parameters(b, "cruise_speed_m_s=fast");
    REQUIRE_FALSE(word);
    CHECK(word.error() == "--set cruise_speed_m_s: 'fast' is not a number");
    for (const char* bad : {"", "cruise_speed_m_s", "=4", "cruise_speed_m_s=", "cruise_speed_m_s=4,"}) {
        INFO(bad);
        CHECK_FALSE(harness::override_parameters(b, bad));
    }
}
